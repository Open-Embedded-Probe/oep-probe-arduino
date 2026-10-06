// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.wire.rvswd / oep.target.riscv-dm over Ch32Dm on a fake DMI PHY with a small debug module behind it.
// - A version-3 (debug spec 1.0) module is worked with like a version-2 one: attach(halt) stops it and says halted,
//   riscv-dm halt answers ok (oep-if-debug §1: found = DMSTATUS.version 2 or 3).
// - The pins go to their free state - Hi-Z with no pull, or the idle the settings give them - whenever nothing holds
//   them: a connection closed (also with idle_clock low, which rests SWCLK driven low), a scan's try, a failed attach
//   (oep-core §8, oep-if-debug §1).
// - scan brings a pair up without the write check (the PHY's bringUp: nothing written through write()); an attach with
//   halt answers the DMSTATUS after the halt; search_retries (TLV 0x12) counts the attaches tried again; the attach
//   budget stops them; no pair of a scan starts after the scan budget; a request's wire retries stop after
//   wire_retry_ms, and the next request has its own (oep-if-debug §1, §2).
// - Wire loss (oep-if-debug §2): a request that gets nothing back answers status line and keeps the connection; it
//   closes only after wire_lost_ms of failures with no good exchange between (requests and the liveness check alike),
//   a reset's hold and the wire_lost_ms after it not counted. A read of all zeros / all ones is no good exchange: on
//   DMSTATUS it counts as no answer, on another register it leaves the clock as it is (a line with no module behind
//   it closes after wire_lost_ms, through the liveness check, riscv-dm's ops and the console alike).
// - Held pins (core §4.3, §8.1): a refusal says cause 1, the channel and its holder_kind; the pair the link was last
//   on is refused while a plan holds it.
// - How a live connection's line rests (oep-if-debug §1, §3): only an attach that carries idle_clock changes it; one
//   joining without it keeps it, a new connection without it rests high, a scan never changes it.
// - A link that misses one DMI access (a glitch: a write lost, a read answering the read before it): the console's three
//   mechanisms against targets that play by their rules take every byte once both ways with a read missed (dmseq with a
//   write missed too); attach, halt, resume, read_block and scan with a misleading word read before them answer as the
//   target is and change nothing they should not (a running hart left running, no revive, no restart counted).
#include <stdio.h>
#include <stdlib.h>

#include <map>
#include <vector>

#include "OepDmConsole.h"
#include "OepPinTable.h"
#include "OepTarget.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace wire = reg::wire_rvswd;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// The pins as a PHY leaves them, and a debug module that answers when `present`.
class FakePhy final : public DmiPhy {
 public:
  enum State { kFree, kReleased, kDriven };   // free: Hi-Z no pull; released: Hi-Z with SWDIO's pull-up; driven
  State state = kFree;
  bool idle_low = false, attached_flag = false, present = true;
  uint32_t version = 2;
  int dio = -1, clk = -1;
  bool halted = false, resumeack = false;
  uint32_t data0 = 0, data1 = 0;
  // what the layer above did: attach() / bringUp() calls, write() calls, and how long each attach / bring-up takes
  int attaches = 0, bring_ups = 0, writes = 0, fail_attaches = 0;
  uint32_t attach_ms = 0, bring_up_ms = 0;
  // flaky: every read fails once per retry try (1 ms each) until the request's allowance is spent
  bool flaky = false;
  uint32_t retry_reads = 0;
  // stuck: every read comes back with this value (a line held low: 0; one rising through its pull-up, no module: ~0)
  bool stuck = false;
  uint32_t stuck_value = 0;
  // stale: the target lost its power and came back - the link the probe holds is out of step and reads all ones
  // until it is brought up afresh (attach() from not attached); the module then answers with havereset set
  bool stale = false, havereset = false;
  // drop_on_change: the link drops when the hart changes state (a CH32L103): reads all ones until reinit() (Ch32Dm's
  // relink) brings it back in step
  bool drop_on_change = false, dropped = false;
  int reinits = 0;
  // The drop's timing (the upload loop below draws them at random): the link drops only after drop_delay_reads more
  // reads at the change (those still answer, with what the module has - halted or not as it now is), and a reinit()
  // within drop_hold_us of the drop leaves it dropped (the target's debug unit not back yet).
  int drop_delay_reads = 0, pending_drop = -1;
  uint32_t drop_hold_us = 0, dropped_at_us = 0;
  void startDrop() {
    if (drop_delay_reads > 0) { pending_drop = drop_delay_reads; return; }
    dropped = true;
    dropped_at_us = micros();
  }
  void reinit() override {
    ++reinits;
    if (micros() - dropped_at_us >= drop_hold_us) dropped = false;
  }
  // With the link dropped, writes are lost too (drop_loses_writes), and reads give the previous read's value instead of
  // all ones (drop_stale: the DTM answers with what it last had).
  bool drop_loses_writes = false, drop_stale = false;
  uint32_t last_read = 0;
  int lost_writes = 0;
  // run_reads: a resumereq lets the hart run for that many DMSTATUS reads before it stops by itself (an ebreak), a
  // change of state (0: stop at once when step_returns, else run on)
  int run_reads = 0, running_reads = 0;
  // loader_pc: a resumereq reaches the ebreak (run_reads later, dpc then loader_pc + 0x40) only from dpc == loader_pc
  // with dcsr.ebreakm set - else the hart runs the application on (a lost dpc or dcsr write: the run's timeout)
  uint32_t loader_pc = 0;
  // model_block: the abstract commands behind the block ops - GPRs, dpc / dcsr, the program buffer (the probe's block
  // reader / writer), abstractauto on DATA0, cmderr 4 while the hart runs - over a word memory; DATA0 / DATA1 sit at
  // 0xe0000380 / 0xe0000384 (HARTINFO)
  bool model_block = false;
  uint32_t gpr[32] = {}, dpc = 0, progbuf[8] = {}, abstractauto = 0, last_command = 0, cmderr = 0, autoexec_runs = 0;
  // mstatus (0x300) with model_block: mpp_fixed - MPP (12:11) reads 3 whatever is written (WARL: a hart with machine mode
  // only, the CH32V003's QingKe V2A), so a write of 0 reads back 0x1800, the value it had. With loader_pc, a run with
  // mstatus.MIE set does not reach its ebreak (an interrupt takes the hart into the application: a lost write)
  uint32_t mstatus = 0;
  bool mpp_fixed = false;
  std::map<uint32_t, uint32_t> mem;
  void execute(uint32_t command) {
    if (cmderr) return;                                     // sticky: nothing runs until cleared
    if (!halted) { cmderr = 4; return; }                    // halt / resume: not halted
    const uint16_t regno = command & 0xffff;
    if (command & (1u << 17)) {                             // transfer
      uint32_t *r = regno >= 0x1000 && regno < 0x1020 ? &gpr[regno - 0x1000] : regno == 0x7b1 ? &dpc
                    : regno == 0x7b0 ? &dcsr : regno == 0x300 ? &mstatus : nullptr;
      if (!r) { cmderr = 2; return; }
      if (command & (1u << 16)) *r = data0; else data0 = *r;
      if (mpp_fixed) mstatus |= 0x1800u;
    }
    if (command & (1u << 18)) {                             // postexec: the program buffer, through a0 / a1 as they are
      if (progbuf[0] == 0x40044180u) {                      // reader: lw s0,0(a1); lw s1,0(s0); addi s0,4; sw s1,0(a0); sw s0,0(a1)
        gpr[8] = load(gpr[11]); gpr[9] = load(gpr[8]); gpr[8] += 4; store(gpr[10], gpr[9]); store(gpr[11], gpr[8]);
      } else if (progbuf[0] == 0x41044180u) {               // writer: lw s0,0(a1); lw s1,0(a0); sw s1,0(s0); addi s0,4; sw s0,0(a1)
        gpr[8] = load(gpr[11]); gpr[9] = load(gpr[10]);
        if (fault_store && gpr[8] == fault_store) { cmderr = 3; return; }   // the store's exception: nothing stored
        store(gpr[8], gpr[9]); gpr[8] += 4; store(gpr[11], gpr[8]);
      } else {
        cmderr = 3;
      }
    }
  }
  // the hart's view of memory: DATA0 / DATA1 at 0xe0000380 / 0xe0000384, the rest a word memory (a0 or a1 left wrong
  // would read or write the target's memory somewhere else - stray_stores counts the stores outside 0x20000000-0x2000ffff)
  int stray_stores = 0;
  uint32_t fault_store = 0;         // a store to this address faults (cmderr 3), nothing stored (0: none)
  std::map<uint32_t, int> stores;   // the hart's stores to memory, by address (a register with side effects - a flash
                                    // key register - takes each write as one: written twice is not written once)
  uint32_t load(uint32_t a) { return a == 0xe0000380u ? data0 : a == 0xe0000384u ? data1 : mem[a]; }
  void store(uint32_t a, uint32_t v) {
    if (a == 0xe0000380u) data0 = v;
    else if (a == 0xe0000384u) data1 = v;
    else { if ((a >> 16) != 0x2000u) ++stray_stores; mem[a] = v; ++stores[a]; }
  }
  // pending_drop_access: the drop after that many more DMI accesses, reads and writes alike (pending_drop counts reads
  // only): a drop between two writes - after the one that started the block writer, say - and not only at a read
  int pending_drop_access = -1, accesses = 0;
  // A glitch: the access glitch_at (glitch_at2: a second one) accesses on is missed on its own and the link is up again
  // at the next one - nothing to relink, the looks around it pass. A write is lost - with cmderr 6 set when
  // glitch_parity (the module took the frame for one with a bad parity: QingKe's cmderr 6 "parity bit error during
  // communication") - and a read answers the value of the read before it.
  int glitch_at = -1, glitch_at2 = -1, glitches = 0, glitched_reads = 0, glitched_writes = 0;
  bool glitch_parity = false;
  bool countAccess() {
    ++accesses;
    if (pending_drop_access >= 0 && pending_drop_access-- == 0) { dropped = true; dropped_at_us = micros(); }
    const bool glitch = glitch_at == 0 || glitch_at2 == 0;
    if (glitch_at >= 0) --glitch_at;
    if (glitch_at2 >= 0) --glitch_at2;
    if (glitch) ++glitches;
    return glitch;
  }

  bool attach() override {
    if (attached_flag) return true;
    ++attaches;
    stale = false;
    g_millis += attach_ms;
    state = kDriven;
    if (!present || fail_attaches > 0) { --fail_attaches; state = kReleased; return false; }
    attached_flag = true;
    return true;
  }
  bool bringUp(uint32_t &status) override {   // the wake / configuration and dmactive (not seen here), DMSTATUS read
    ++bring_ups;
    g_millis += bring_up_ms;
    state = kReleased;
    if (!present) return false;
    status = version | (1u << 7) | (halted ? (3u << 8) : (3u << 10));
    return true;
  }
  void release() override { state = kReleased; attached_flag = false; }
  void park() override {
    attached_flag = false;
    state = idle_low ? kDriven : kReleased;   // SWCLK low is driven
  }
  void free() override { state = kFree; attached_flag = false; }
  bool attached() const override { return attached_flag; }
  // ignore_halt / ignore_resume: haltreq / resumereq do not change the hart (a halt or resume that never lands);
  // step_returns: a resumereq runs one instruction and the hart is back in debug mode at once (a step); each read takes
  // read_us of time
  bool ignore_halt = false, ignore_resume = false, step_returns = false;
  uint32_t abstract_cmderr = 0;   // every abstract command fails with it (0: none)
  uint32_t hartsel = 0;           // DMCONTROL's hasel / hartsello / hartselhi as last written
  uint32_t dcsr_written = 0;      // the value of the last abstract command that wrote dcsr
  std::vector<uint32_t> dcsr_writes;   // every one
  bool model_dcsr = false;        // dcsr as a register of its own (abstract reads of it give it back); else DATA0 as is
  uint32_t dcsr = 0;
  uint32_t read_us = 10, dmcontrol = 0, target_id = 0;
  std::map<uint32_t, uint32_t> regs;   // the registers an access-register command reaches, without model_block
  int reads = 0;
  bool readWire(uint8_t address, uint32_t &value) override {
    ++reads;
    advanceMicros(read_us);
    if (!present || !attached_flag) return false;
    const bool glitch = countAccess();
    if (stuck) { value = stuck_value; return true; }
    if (stale) { value = 0xffffffffu; return true; }
    if (pending_drop >= 0 && pending_drop-- == 0) { dropped = true; dropped_at_us = micros(); }
    if (dropped) { value = drop_stale ? last_read : 0xffffffffu; return true; }
    if (glitch) { ++glitched_reads; value = last_read; return true; }
    if (address == 0x11 && running_reads > 0 && --running_reads == 0) {   // the run reaches its ebreak
      halted = true;
      if (loader_pc) dpc = loader_pc + 0x40;
      if (drop_on_change) startDrop();
      if (dropped) { value = drop_stale ? last_read : 0xffffffffu; return true; }
    }
    if (flaky) {   // the RVSWD PHY's way: retries while the request's allowance lasts, then a failed read
      while (retryLeft()) { g_millis += 1; spentRetrying(1000); ++retry_reads; }
      return false;
    }
    switch (address) {
      case 0x04: value = data0; break;
      case 0x05: value = data1; break;
      case 0x10: value = 1 | hartsel; break;   // DMCONTROL: dmactive, the hart selected (haltreq reads 0)
      case 0x11:                     // DMSTATUS: version, authenticated, all/any halted or running, allresumeack
        value = version | (1u << 7) | (halted ? (3u << 8) : (3u << 10)) | (resumeack ? (3u << 16) : 0) |
                (havereset ? (3u << 18) : 0);
        break;
      case 0x12: value = 0x0002'1000u | 0x380; break;   // HARTINFO: DATA0 at 0x380 (memory-mapped), datacount 2
      case 0x7f: value = target_id; break;              // the WCH target id (wch_dmi_7f)
      case 0x16: value = 2 | ((model_block ? cmderr : abstract_cmderr) << 8); break;   // ABSTRACTCS: datacount 2, cmderr
      case 0x18: value = abstractauto; break;
      default: value = address >= 0x20 && address < 0x28 ? progbuf[address - 0x20] : 0; break;   // the program buffer
    }
    last_read = value;
    if (model_block && address == 0x04 && (abstractauto & 1)) { ++autoexec_runs; execute(last_command); }
    return true;
  }
  void write(uint8_t address, uint32_t value) override {
    ++writes;
    if (!present || !attached_flag) return;
    const bool glitch = countAccess();
    if (dropped && drop_loses_writes) { ++lost_writes; return; }
    if (glitch) {
      ++glitched_writes;
      ++lost_writes;
      if (glitch_parity && model_block && !cmderr) cmderr = 6;
      return;
    }
    if (model_block) {
      if (address == 0x04) { data0 = value; if (abstractauto & 1) { ++autoexec_runs; execute(last_command); } return; }
      if (address == 0x05) { data1 = value; return; }
      if (address == 0x16) { if (value & 0x700) cmderr = 0; return; }
      if (address == 0x17) { last_command = value; execute(value); return; }
      if (address == 0x18) { abstractauto = value; return; }
      if (address >= 0x20 && address < 0x28) { progbuf[address - 0x20] = value; return; }
    }
    if (address == 0x04) data0 = value;
    if (address == 0x05) data1 = value;
    if (address == 0x18) abstractauto = value;
    if (address >= 0x20 && address < 0x28) progbuf[address - 0x20] = value;
    if (address == 0x17 && (value & (1u << 16)) && (value & 0xffff) == 0x7b0) {   // write dcsr
      dcsr_written = data0;
      dcsr_writes.push_back(data0);
      dcsr = data0;
    }
    if (model_dcsr && address == 0x17 && !(value & (1u << 16)) && (value & 0xffff) == 0x7b0) data0 = dcsr;   // read it
    // the other registers as a plain store: an access-register command writes DATA0 into one and reads one into DATA0
    // (the probe reads a register twice over two sentinels in DATA0, and reads what it writes back)
    if (address == 0x17 && (value & (1u << 17)) && !abstract_cmderr && (value & 0xffff) != 0x7b0) {
      if (value & (1u << 16)) regs[value & 0xffff] = data0;
      else data0 = regs[value & 0xffff];
    }
    if (address == 0x17 && (value & (1u << 17)) && !abstract_cmderr && !model_dcsr && (value & 0xffff) == 0x7b0) {
      if (value & (1u << 16)) regs[0x7b0] = data0;
      else data0 = regs[0x7b0];
    }
    if (address == 0x10) {
      dmcontrol = value;
      hartsel = value & 0x07ffffc0u;
      const bool was = halted;
      if ((value & (1u << 31)) && !ignore_halt) { halted = true; resumeack = false; }
      if ((value & (1u << 30)) && !ignore_resume) {
        halted = step_returns && !run_reads;
        resumeack = true;
        running_reads = run_reads;
        if (loader_pc && run_reads && (dpc != loader_pc || !(dcsr & 0x8000u) || (model_block && (mstatus & 8u)))) {
          halted = false;
          running_reads = 0;
        }
      }
      if (drop_on_change && ((value & (3u << 30)) && (halted != was || (value & (1u << 30))))) startDrop();
      if (value & (1u << 28)) havereset = false;   // ackhavereset
    }
  }
  bool setIdleClockLow(bool low) override { idle_low = low; return true; }
  bool setMaxHz(uint32_t) override { return true; }
  bool keepsMaxHz(uint32_t) const override { return true; }
  bool canIdleClockLow() const override { return true; }
  bool usePins(int swdio, int swclk) override {
    if (attached_flag) return false;
    dio = swdio;
    clk = swclk;
    state = kReleased;
    return true;
  }
  uint32_t dmiNs() const override { return 1000; }
  uint32_t clockHz() const override { return 1000000; }
  uint32_t retries() const override { return 0; }
  uint32_t transactions() const override { return 0; }
};

#ifndef OEP_UPLOAD_SEED
#define OEP_UPLOAD_SEED 0x2463534u   // the upload loop's draws (another seed: -DOEP_UPLOAD_SEED=...)
#endif

static bool drives(int pin, int level) { return g_pin_mode[pin] == OUTPUT && g_pin_level[pin] == level; }
static Result call(Interface &i, uint8_t op, const Bytes &payload, Bytes &out) {
  out.assign(256, 0);
  const Result r = i.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
// attach: method, max_speed (critical), [pins (critical)], [idle_clock (critical): 0 high, 1 low; -1 not carried]
// (every TLV tag(u8) len(u16) value, core §2.2)
static Bytes attachRequest(uint8_t method, int swdio = -1, int swclk = -1, int idle_clock = -1) {
  Bytes p = {method, uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 4, 0, 0x40, 0x42, 0x0f, 0x00};
  if (swdio >= 0) {
    p.insert(p.end(), {uint8_t(wire::kTlvAttachPins | kTagCritical), 4, 0, uint8_t(swdio), 0, uint8_t(swclk), 0});
  }
  if (idle_clock >= 0) p.insert(p.end(), {uint8_t(wire::kTlvAttachIdleClock | kTagCritical), 1, 0, uint8_t(idle_clock)});
  return p;
}
static Bytes detachRequest(uint16_t number) { return {uint8_t(number), uint8_t(number >> 8)}; }
// The value of answer TLV `tag` after `from` (core §2.2: tag(u8) len(u16) value), or nullptr.
static const uint8_t *answerTlv(const Bytes &out, size_t from, uint8_t tag, size_t &len) {
  for (size_t at = from; at + 3 <= out.size(); at += 3u + (out[at + 1] | out[at + 2] << 8)) {
    const size_t n = out[at + 1] | out[at + 2] << 8;
    if (out[at] == tag && at + 3u + n <= out.size()) { len = n; return out.data() + at + 3; }
  }
  return nullptr;
}

// rejected unavailable, cause 5, the channel, holder_kind 7 settings_idle (debug §1)
static bool isSettingsIdle(const Result &r, const Bytes &out, uint16_t channel) {
  if (r.resolution != kResolutionRejected || r.detail != kRejectUnavailable) return false;
  size_t len = 0;
  const uint8_t *cause = answerTlv(out, 0, reg::core::kTlvUnavailablePayloadCause, len);
  const uint8_t *ch = answerTlv(out, 0, reg::core::kTlvUnavailablePayloadChannel, len);
  const uint8_t *kind = answerTlv(out, 0, reg::core::kTlvUnavailablePayloadHolderKind, len);
  return cause && cause[0] == reg::core::kUnavailableCauseHeldBySettings && ch && (ch[0] | ch[1] << 8) == channel &&
         kind && kind[0] == reg::core::kHolderKindSettingsIdle;
}

// rejected unavailable, cause 1 pin in use, the channel, its holder_kind (core §4.3)
static bool isHeld(const Result &r, const Bytes &out, uint16_t channel, uint8_t holder_kind) {
  if (r.resolution != kResolutionRejected || r.detail != kRejectUnavailable) return false;
  size_t len = 0;
  const uint8_t *cause = answerTlv(out, 0, reg::core::kTlvUnavailablePayloadCause, len);
  const uint8_t *ch = answerTlv(out, 0, reg::core::kTlvUnavailablePayloadChannel, len);
  const uint8_t *kind = answerTlv(out, 0, reg::core::kTlvUnavailablePayloadHolderKind, len);
  return cause && cause[0] == reg::core::kUnavailableCausePinInUse && ch && (ch[0] | ch[1] << 8) == channel && kind &&
         kind[0] == holder_kind;
}

// ---- the target's side of the console's mailbox, for the glitch tests (each writes DATA1 before DATA0, directly) ----
// SDI (oep-if-console §3.1): waits for DATA0 to read 0, then posts the next frame of 1-7 bytes.
struct SdiTarget {
  SdiTarget(FakePhy &phy, const std::vector<Bytes> &f) : p(phy), frames(f) {}
  FakePhy &p;
  std::vector<Bytes> frames;
  size_t next = 0;
  void service() {
    if (p.data0 != 0 || next >= frames.size()) return;
    const Bytes &f = frames[next++];
    uint8_t b[7] = {};
    for (size_t i = 0; i < f.size(); ++i) b[i] = f[i];
    p.data1 = b[3] | b[4] << 8 | b[5] << 16 | uint32_t(b[6]) << 24;
    p.data0 = uint32_t(f.size()) | b[0] << 8 | b[1] << 16 | uint32_t(b[2]) << 24;
  }
};
// DMDATA (§3.2): a word with bit 7 clear is the answer - it takes the answer's bytes (L 5-7), then posts its next slot
// (or the empty one, 0x84, with nothing left to send).
struct DmdataTarget {
  DmdataTarget(FakePhy &phy, const std::vector<Bytes> &f) : p(phy), frames(f) {}
  FakePhy &p;
  std::vector<Bytes> frames;
  size_t next = 0;
  bool started = false;
  Bytes rx;
  void service() {
    if (p.data0 & 0x80u) return;   // its slot still there
    const uint32_t w = p.data0;
    const uint32_t l = w & 0x3fu;
    if (started && l >= 5 && l <= 7) for (uint32_t i = 0; i < l - 4; ++i) rx.push_back(uint8_t(w >> (8 + 8 * i)));
    started = true;
    if (next >= frames.size()) { p.data0 = 0x84u; return; }
    const Bytes &f = frames[next++];
    uint8_t b[7] = {};
    for (size_t i = 0; i < f.size(); ++i) b[i] = f[i];
    if (f.size() >= 4) p.data1 = b[3] | b[4] << 8 | b[5] << 16 | uint32_t(b[6]) << 24;
    p.data0 = 0x80u | uint32_t(f.size() + 4) | b[0] << 8 | b[1] << 16 | uint32_t(b[2]) << 24;
  }
};
// dmseq (target-console-dmseq): frames of up to 6 bytes with S / A / SYN and the CRC; target rules 0-3 (a word with
// bit 7 that is not its own: posted again; 0: still waiting; an invalid answer or K != S: posted again; an ack: S
// toggled, input taken once per H, the next frame posted - an empty one when there is nothing to send).
struct SeqSender {
  SeqSender(FakePhy &phy, const Bytes &o) : p(phy), out(o) {}
  FakePhy &p;
  Bytes out;
  size_t at = 0;
  uint8_t s = 0, last_h = 1, n = 0;
  bool posted = false, syn = true;
  uint32_t w0 = 0, w1 = 0;
  Bytes rx;
  void post() {
    n = uint8_t(out.size() - at > 6 ? 6 : out.size() - at);
    uint8_t b[8] = {uint8_t(0x80 | (s << 5) | (last_h << 4) | (syn ? 0x08 : 0) | n)};
    for (uint8_t i = 0; i < n; ++i) b[1 + i] = out[at + i];
    b[1 + n] = DmConsole::crc8(b, 1 + n);
    w0 = b[0] | b[1] << 8 | b[2] << 16 | uint32_t(b[3]) << 24;
    w1 = b[4] | b[5] << 8 | b[6] << 16 | uint32_t(b[7]) << 24;
    p.data1 = w1;
    p.data0 = w0;
    posted = true;
  }
  void again() { p.data1 = w1; p.data0 = w0; }
  void service() {
    if (!posted) { post(); return; }
    const uint32_t w = p.data0;
    if (w & 0x80u) { if (w != w0) again(); return; }
    if (w == 0) return;
    const uint8_t a[4] = {uint8_t(w), uint8_t(w >> 8), uint8_t(w >> 16), uint8_t(w >> 24)};
    const uint8_t k = (a[0] >> 5) & 1, h = (a[0] >> 4) & 1, m = a[0] & 7;
    if (m > 2 || DmConsole::crc8(a, 1 + m) != a[1 + m] || k != s) { again(); return; }
    at += n;
    s ^= 1;
    syn = false;
    if (m && h != last_h) { rx.insert(rx.end(), a + 1, a + 1 + m); last_h = h; }
    post();
  }
};
static void pushTo(void *ctx, uint8_t byte) { static_cast<Bytes *>(ctx)->push_back(byte); }

int main() {
  // A fixed pair (the wire's own channels 0 / 1, not in the pin table) and a host-chosen one among 4-7.
  static FakePhy phy;
  static Ch32Dm dm(phy);
  static PinTable pins((1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7));
  static DebugPort fixed{dm, 0, 1};
  static WireRvswd wire_fixed(fixed, 0);
  static TargetRiscvDm riscv(fixed, 0);
  Bytes out;

  // ---- a version-3 module: attach(halt) stops it and says so; halt is ok; detach frees the pins ----
  for (uint32_t version : {2u, 3u}) {
    phy.version = version;
    phy.halted = false;
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1), out);
    CHECK(ok(r) && out.size() >= 11 && (out[6] & wire::kAttachFlagsHalted));
    CHECK(phy.halted && dm.halted());
    CHECK(dm.checkHalted() && dm.halted());   // what DMSTATUS says now: halted (0.0.28: a version-3 module never was)
    r = call(riscv, TargetRiscvDm::kOpHalt, detachRequest(fixed.number), out);   // connection(u16)
    CHECK(ok(r) && out.size() == 1 && out[0] == 0);
    CHECK(fixed.connected);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    CHECK(phy.state == FakePhy::kFree);
  }

  // ---- idle_clock low rests SWCLK driven; closing the connection lets it go (core §8) ----
  {
    phy.halted = false;
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0, -1, -1, 1), out);
    CHECK(ok(r) && fixed.connected);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    CHECK(phy.state == FakePhy::kFree);   // 0.0.28: park() left SWCLK driven low
    phy.idle_low = false;
  }

  // ---- how a live connection's line rests (oep-if-debug §1, §3; oep-spec 59dd028): an attach joining it keeps the
  // settings it does not carry - only a carried idle_clock changes the rest; a new connection without the TLV rests
  // high; a scan never changes a live connection's settings (it switched to high on every attach without the TLV:
  // a tool joining a slot's idle_clock-low connection changed how the line rests) ----
  {
    phy.halted = false;
    const Bytes conn_scan = {0};
    const Bytes scan_high = {0, uint8_t(wire::kTlvScanIdleClock | kTagCritical), 1, 0, 0};
    const Bytes scan_low = {0, uint8_t(wire::kTlvScanIdleClock | kTagCritical), 1, 0, 1};
    // a slot's connection (attachRunning with the slot's idle_clock low)
    uint32_t dmstatus = 0;
    CHECK(attachRunning(fixed, DebugPort::kUserSlot, dmstatus, 0, true) && fixed.connected && phy.idle_low);
    const uint16_t number = fixed.number;
    auto joined = [&](const Result &r) {   // the existing connection returned, flags bit1
      return ok(r) && fixed.connected && fixed.number == number && out.size() >= 11 &&
             (out[0] | out[1] << 8) == number && (out[6] & wire::kAttachFlagsExisting);
    };
    // a host attach carrying no idle_clock joins it and leaves it low
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(joined(r) && phy.idle_low);
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1), out);   // method halt alike
    CHECK(joined(r) && phy.idle_low);
    phy.halted = false;
    // a scan through it, with idle_clock 0 or none: the live connection's rest unchanged
    r = call(wire_fixed, WireRvswd::kOpScan, scan_high, out);
    CHECK(ok(r) && out.size() == 11 && out[1] == 1 && fixed.connected && phy.idle_low);
    r = call(wire_fixed, WireRvswd::kOpScan, conn_scan, out);
    CHECK(ok(r) && out[1] == 1 && phy.idle_low);
    // a host attach carrying idle_clock 0 switches it to high
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0, -1, -1, 0), out);
    CHECK(joined(r) && !phy.idle_low);
    r = call(wire_fixed, WireRvswd::kOpScan, scan_low, out);   // a scan with idle_clock 1: still high
    CHECK(ok(r) && out[1] == 1 && !phy.idle_low);
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);   // no TLV: stays high
    CHECK(joined(r) && !phy.idle_low);
    // an attach that carried idle_clock 1 sets it low; a later one without the TLV keeps it low
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0, -1, -1, 1), out);
    CHECK(joined(r) && phy.idle_low);
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(joined(r) && phy.idle_low);
    r = call(wire_fixed, WireRvswd::kOpScan, scan_high, out);
    CHECK(ok(r) && out[1] == 1 && phy.idle_low);
    // the host's detach leaves the slot's use; the slot's release closes it
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(number), out);
    CHECK(ok(r) && fixed.connected && fixed.users == DebugPort::kUserSlot);
    releaseConnection(fixed, DebugPort::kUserSlot, false);
    CHECK(!fixed.connected && phy.state == FakePhy::kFree);
    // a new connection without the TLV rests high, whatever the last one rested at (the PHY still set low)
    CHECK(phy.idle_low);
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected && !(out[6] & wire::kAttachFlagsExisting) && !phy.idle_low);
    // ... and a host attach's own idle_clock-low connection: joined without the TLV, it stays low
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0, -1, -1, 1), out);
    CHECK(ok(r) && fixed.connected && !(out[6] & wire::kAttachFlagsExisting) && phy.idle_low);
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && (out[6] & wire::kAttachFlagsExisting) && phy.idle_low);
    // a slot joining it (attachRunning with its own idle_clock high) keeps it as well
    CHECK(attachRunning(fixed, DebugPort::kUserSlot, dmstatus, 0, false) && phy.idle_low);
    releaseConnection(fixed, DebugPort::kUserSlot, false);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    phy.idle_low = false;
  }

  // ---- swio: a listed combination with swclk other than 0xFFFF is one the declaration does not allow: unsupported
  // (payload 0x00) in scan as in attach (oep-if-debug §3; it was malformed in scan)
  {
    static FakePhy phy_swio;
    static Ch32Dm dm_swio(phy_swio);
    static DebugPort one{dm_swio, 0xfffe, 0xffff};   // one wire, no channel chosen yet
    one.pin_choice = (1ull << 8) | (1ull << 9);
    static WireRvswd wire_swio(one, 7, reg::wire_swio::kName);
    Result r = call(wire_swio, WireRvswd::kOpScan, {1, 8, 0, 9, 0}, out);
    // tag 0x00, then TLV 0x40 index: the combination's position in the request (oep-if-debug §1; it had no index)
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out == Bytes({0, 0x40, 1, 0, 0}));
    r = call(wire_swio, WireRvswd::kOpScan, {1, 8, 0, 9}, out);   // cut short: still malformed
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    r = call(wire_swio, WireRvswd::kOpAttach, attachRequest(0, 8, 9), out);   // attach alike (pins TLV, critical)
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && !one.connected);
    // attach without pins, no live connection: two candidates (8, 9) - the host names one; one candidate left (a plan
    // holds 9) - it is used (oep-if-debug §1; it was refused unavailable)
    static PinTable pins_swio((1ull << 8) | (1ull << 9));
    one.pins = &pins_swio;
    r = call(wire_swio, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnavailable && !one.connected);
    CHECK(pins_swio.claim(9, 0x01));
    r = call(wire_swio, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && one.connected && one.swdio == 8 && pins_swio.owner(8) == one.pin_owner);
    r = call(wire_swio, WireRvswd::kOpDetach, detachRequest(one.number), out);
    CHECK(ok(r) && !one.connected);
    // the only allowed combination with an idle item: no candidate, unavailable cause 5 holder_kind 7 (debug §1)
    one.pin_choice = 1ull << 8;
    CHECK(pins_swio.setIdle(8, PinTable::kIdlePullUp));
    r = call(wire_swio, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(isSettingsIdle(r, out, 8) && !one.connected);
    CHECK(pins_swio.setIdle(8, PinTable::kIdleUnset));
    r = call(wire_swio, WireRvswd::kOpAttach, attachRequest(0), out);   // the only one, free: used
    CHECK(ok(r) && one.connected && one.swdio == 8);
    r = call(wire_swio, WireRvswd::kOpDetach, detachRequest(one.number), out);
    CHECK(ok(r) && !one.connected);
    pins_swio.release(0x01);
  }

  // ---- host-chosen pins: a closed connection, a scan's try and a failed attach leave each channel at its idle ----
  {
    static FakePhy phy2;
    static Ch32Dm dm2(phy2);
    static DebugPort chosen{dm2, 0xfffe, 0xfffe};   // two wires, no pair chosen yet
    chosen.pins = &pins;
    chosen.pin_choice = (1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7);
    static WireRvswd wire_chosen(chosen, 1);
    CHECK(pins.setIdle(4, PinTable::kIdlePullUp));
    CHECK(pins.setIdle(5, PinTable::kIdleOutputLow));
    // a channel whose idle is an output, named: unavailable cause 5, the channel, holder_kind 7 settings_idle (debug §1)
    Result r = call(wire_chosen, WireRvswd::kOpAttach, attachRequest(0, 4, 5, 1), out);
    CHECK(isSettingsIdle(r, out, 5) && !chosen.connected);
    const Bytes scan_out_idle = {1, 4, 0, 5, 0};
    r = call(wire_chosen, WireRvswd::kOpScan, scan_out_idle, out);
    CHECK(isSettingsIdle(r, out, 5));
    // a combination the declaration does not allow comes first, wherever it is listed: unsupported with its index
    // (core §4.3 order 6 before 7; it answered unavailable for the output idle of the first)
    r = call(wire_chosen, WireRvswd::kOpScan, {2, 4, 0, 5, 0, 9, 0, 5, 0}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out == Bytes({0, 0x40, 1, 0, 1}));
    CHECK(pins.setIdle(5, PinTable::kIdlePullDown));   // an input idle, named: accepted
    // attach on 4 / 5, then detach: 4 pulled up, 5 pulled down by its idle, the PHY's own drive gone
    r = call(wire_chosen, WireRvswd::kOpAttach, attachRequest(0, 4, 5, 1), out);
    CHECK(ok(r) && chosen.connected && pins.owner(4) == chosen.pin_owner);
    g_pin_mode[4] = g_pin_mode[5] = -1;
    r = call(wire_chosen, WireRvswd::kOpDetach, detachRequest(chosen.number), out);
    CHECK(ok(r) && !chosen.connected && pins.free(4) && pins.free(5));
    CHECK(phy2.state == FakePhy::kFree);
    CHECK(g_pin_mode[4] == INPUT_PULLUP);
    CHECK(g_pin_mode[5] == INPUT_PULLDOWN);
    // a scan of 4 / 5 (found) and 6 / 7 (nothing there): each tried pair back to its free state
    g_pin_mode[4] = g_pin_mode[5] = -1;
    const Bytes scan_found = {1, 4, 0, 5, 0};
    r = call(wire_chosen, WireRvswd::kOpScan, scan_found, out);
    CHECK(ok(r) && out.size() >= 2 && out[0] == 1 && out[1] == 1);
    CHECK(phy2.state == FakePhy::kFree && !chosen.connected);
    CHECK(g_pin_mode[4] == INPUT_PULLUP && g_pin_mode[5] == INPUT_PULLDOWN);
    phy2.present = false;
    CHECK(pins.setIdle(6, PinTable::kIdlePullDown));
    g_pin_mode[6] = -1;
    const Bytes scan_none = {1, 6, 0, 7, 0};
    r = call(wire_chosen, WireRvswd::kOpScan, scan_none, out);
    CHECK(ok(r) && out.size() >= 2 && out[0] == 1 && out[1] == 0);
    CHECK(phy2.state == FakePhy::kFree);
    CHECK(g_pin_mode[6] == INPUT_PULLDOWN);   // 0.0.28: left Hi-Z, not its idle
    // a failed attach: nothing held, the pair free
    g_pin_mode[6] = -1;
    r = call(wire_chosen, WireRvswd::kOpAttach, attachRequest(0, 6, 7), out);
    CHECK(r.resolution == kResolutionCompleted && r.detail != kOutcomeSuccess && !chosen.connected);
    CHECK(phy2.state == FakePhy::kFree && pins.free(6) && pins.free(7));
    CHECK(g_pin_mode[6] == INPUT_PULLDOWN);
    // a pair this wire does not declare: unsupported with the pins tag as received; sent without the critical bit it is
    // ignored and listed (core §2.3)
    phy2.present = true;
    Bytes bad = {0, uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 4, 0, 0x40, 0x42, 0x0f, 0x00,
                 uint8_t(wire::kTlvAttachPins | kTagCritical), 4, 0, 9, 0, 5, 0};
    r = call(wire_chosen, WireRvswd::kOpAttach, bad, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out.size() >= 1 &&
          out[0] == (wire::kTlvAttachPins | kTagCritical));
    bad[8] = wire::kTlvAttachPins;   // not critical: ignored; no live connection and the host must name a pair
    r = call(wire_chosen, WireRvswd::kOpAttach, bad, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnavailable);
    // count = 0 leaves out every channel with an idle item, input ones too (4, 5, 6 here): only 7 is left, no pair
    const Bytes scan_all = {0};
    r = call(wire_chosen, WireRvswd::kOpScan, scan_all, out);
    CHECK(ok(r) && out.size() >= 2 && out[0] == 0);
    CHECK(pins.setIdle(6, PinTable::kIdleUnset) && pins.setIdle(5, PinTable::kIdleUnset));
    phy2.bring_ups = 0;
    r = call(wire_chosen, WireRvswd::kOpScan, scan_all, out);   // 5, 6, 7: six ordered pairs, none with 4
    CHECK(ok(r) && out.size() >= 2 && out[0] == 6 && phy2.bring_ups == 6);
    // count x (kind swdio(u16) swclk(u16) DMSTATUS(u32)), 9 bytes each, no element length
    CHECK(out.size() == 2u + 9u * 6u);
    for (size_t at = 2; at + 9 <= out.size(); at += 9) CHECK(out[at + 1] != 4 && out[at + 3] != 4);
    CHECK(pins.setIdle(4, PinTable::kIdleUnset));
  }

  // ---- a fixed pair with an idle item: not in count = 0, not a candidate of an attach without pins ----
  {
    static FakePhy phy4;
    static Ch32Dm dm4(phy4);
    static PinTable pins4((1ull << 2) | (1ull << 3));
    static DebugPort fixed4{dm4, 2, 3};
    fixed4.pins = &pins4;
    static WireRvswd wire4(fixed4, 3);
    CHECK(pins4.setIdle(3, PinTable::kIdlePullUp));
    const Bytes scan_all = {0};
    Result r = call(wire4, WireRvswd::kOpScan, scan_all, out);
    CHECK(ok(r) && out.size() >= 2 && out[0] == 0 && phy4.bring_ups == 0);
    r = call(wire4, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(isSettingsIdle(r, out, 3) && !fixed4.connected);
    r = call(wire4, WireRvswd::kOpAttach, attachRequest(0, 2, 3), out);   // named, an input idle: accepted
    CHECK(ok(r) && fixed4.connected);
    r = call(wire4, WireRvswd::kOpScan, scan_all, out);                   // the live pair is listed
    CHECK(ok(r) && out[0] == 1 && out[1] == 1);
    r = call(wire4, WireRvswd::kOpDetach, detachRequest(fixed4.number), out);
    CHECK(ok(r));
    CHECK(pins4.setIdle(3, PinTable::kIdleOutputHigh));
    r = call(wire4, WireRvswd::kOpAttach, attachRequest(0, 2, 3), out);   // named, an output idle: refused
    CHECK(isSettingsIdle(r, out, 3));
    // a slot's own attach on that pair (usePair) does not drive it either
    CHECK(!usePair(fixed4, 2, 3));
  }

  // ---- attach's reset TLV naming a channel whose idle is an output: unavailable cause 5, holder_kind 7, nothing done
  // (oep-if-debug §1, oep-spec 975d88c; 0.0.28 pulled the line) ----
  {
    static FakePhy phy7;
    static Ch32Dm dm7(phy7);
    static PinTable pins7((1ull << 2) | (1ull << 3) | (1ull << 8));
    static DebugPort fixed7{dm7, 2, 3};
    fixed7.pins = &pins7;
    fixed7.reset_allowed = 1ull << 8;
    static WireRvswd wire7(fixed7, 6);
    CHECK(pins7.setIdle(8, PinTable::kIdleOutputHigh));
    Bytes with_reset = attachRequest(1);
    with_reset.insert(with_reset.end(), {uint8_t(wire::kTlvAttachReset | kTagCritical), 4, 0, 8, 0, 10, 0});
    const uint32_t before = millis();
    Result r = call(wire7, WireRvswd::kOpAttach, with_reset, out);
    CHECK(isSettingsIdle(r, out, 8) && !fixed7.connected);
    CHECK(phy7.attaches == 0 && millis() == before && drives(8, HIGH));   // not pulled, not held for hold_ms
    CHECK(pins7.setIdle(8, PinTable::kIdlePullUp));                       // an input idle: the reset goes ahead
    r = call(wire7, WireRvswd::kOpAttach, with_reset, out);
    CHECK(ok(r) && fixed7.connected);
    r = call(wire7, WireRvswd::kOpDetach, detachRequest(fixed7.number), out);
    CHECK(ok(r));
  }

  // ---- the pair the link was last on, taken by a plan since: attach, a slot's usePair and scan all refuse it ----
  {
    static FakePhy phy5;
    static Ch32Dm dm5(phy5);
    static PinTable pins5((1ull << 4) | (1ull << 5) | (1ull << 6));
    static DebugPort chosen5{dm5, 0xfffe, 0xfffe};
    chosen5.pins = &pins5;
    chosen5.pin_choice = (1ull << 4) | (1ull << 5) | (1ull << 6);
    static WireRvswd wire5(chosen5, 4);
    Result r = call(wire5, WireRvswd::kOpAttach, attachRequest(0, 4, 5), out);
    CHECK(ok(r) && chosen5.connected);
    r = call(wire5, WireRvswd::kOpDetach, detachRequest(chosen5.number), out);
    CHECK(ok(r) && !chosen5.connected && chosen5.swdio == 4 && chosen5.swclk == 5);   // the link stays on 4 / 5
    CHECK(pins5.claim(5, 0x01));                                                    // a gpio plan takes 5
    phy5.attaches = phy5.bring_ups = 0;
    r = call(wire5, WireRvswd::kOpAttach, attachRequest(0, 4, 5), out);
    // unavailable cause 1, channel 5, holder_kind 1 plan (core §4.3; 0.0.28: attached)
    CHECK(isHeld(r, out, 5, reg::core::kHolderKindPlan) && !chosen5.connected);
    CHECK(phy5.attaches == 0 && pins5.owner(5) == 0x01);
    CHECK(!usePair(chosen5, 4, 5));                                                 // a slot's attach: the same
    const Bytes scan45 = {1, 4, 0, 5, 0};
    r = call(wire5, WireRvswd::kOpScan, scan45, out);
    CHECK(isHeld(r, out, 5, reg::core::kHolderKindPlan) && phy5.bring_ups == 0);   // 0.0.28: no holder_kind
    pins5.release(0x01);
    r = call(wire5, WireRvswd::kOpAttach, attachRequest(0, 4, 5), out);
    CHECK(ok(r) && chosen5.connected);
    // another wire on the same pins meets this connection: holder_kind 2
    static FakePhy phy6;
    static Ch32Dm dm6(phy6);
    static DebugPort other{dm6, 0xfffe, 0xfffe};
    other.pins = &pins5;
    other.pin_choice = chosen5.pin_choice;
    other.pin_owner = 0xf2;
    static WireRvswd wire6(other, 5);
    r = call(wire6, WireRvswd::kOpAttach, attachRequest(0, 6, 4), out);
    CHECK(isHeld(r, out, 4, reg::core::kHolderKindConnection) && !other.connected);
    const Bytes scan64 = {1, 6, 0, 4, 0};
    r = call(wire6, WireRvswd::kOpScan, scan64, out);
    CHECK(isHeld(r, out, 4, reg::core::kHolderKindConnection));
    // the one seat taken by the host on 4 / 5: another pair of this wire is a count limit (cause 2)
    r = call(wire5, WireRvswd::kOpAttach, attachRequest(0, 6, 5), out);
    size_t clen = 0;
    const uint8_t *cause = answerTlv(out, 0, reg::core::kTlvUnavailablePayloadCause, clen);
    CHECK(r.detail == kRejectUnavailable && cause && cause[0] == reg::core::kUnavailableCauseLimit);
    r = call(wire5, WireRvswd::kOpDetach, detachRequest(chosen5.number), out);
    CHECK(ok(r));
  }

  // ---- scan: the bring-up, no write check, nothing written through the link; attach(halt): DMSTATUS after the halt ----
  {
    phy.version = 2;
    phy.halted = false;
    phy.writes = phy.attaches = phy.bring_ups = 0;
    const Bytes scan_fixed = {0};
    Result r = call(wire_fixed, WireRvswd::kOpScan, scan_fixed, out);
    CHECK(ok(r) && out.size() == 11 && out[0] == 1 && out[1] == 1 && out[2] == wire::kScanKindRiscvDm);
    CHECK(phy.bring_ups == 1 && phy.attaches == 0 && phy.writes == 0);   // 0.0.28: a full attach, its write check, DMCONTROL
    CHECK(phy.state == FakePhy::kFree && !fixed.connected);
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1), out);
    CHECK(ok(r) && out.size() >= 11 && (out[6] & wire::kAttachFlagsHalted));
    const uint32_t dmstatus = out[2] | out[3] << 8 | out[4] << 16 | uint32_t(out[5]) << 24;
    CHECK((dmstatus & (1u << 9)) && !(dmstatus & (1u << 11)));   // allhalted, not running (0.0.28: the value before the halt)
    size_t len = 0;
    const uint8_t *v = answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len);
    CHECK(v && len == 2 && v[0] == 0 && v[1] == 0);
    // joining it again: no search, no search_retries
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && (out[6] & wire::kAttachFlagsExisting) && !answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len));
    CHECK(!(out[6] & wire::kAttachFlagsHaveresetAcked));
    // joining with a havereset pending: acknowledged, flags bit0 (oep-if-debug §4.6; it was left pending)
    phy.havereset = true;
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && (out[6] & wire::kAttachFlagsExisting) && (out[6] & wire::kAttachFlagsHaveresetAcked) && !phy.havereset);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    // the first attach() fails, the second takes: search_retries 1
    phy.fail_attaches = 1;
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r));
    v = answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len);
    CHECK(v && len == 2 && v[0] == 1 && v[1] == 0);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r));
  }

  // ---- halt / resume wait dm_wait_ms of time (oep-if-debug §4, §4.2): halt then clears haltreq and answers timeout,
  // resume answers state (halt went by 8 rounds of polls, resume by 25 reads, and halt left haltreq set) ----
  {
    phy.halted = false;
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    const Bytes conn = {uint8_t(fixed.number), uint8_t(fixed.number >> 8)};
    phy.ignore_halt = phy.ignore_resume = true;
    uint32_t before = millis();
    r = call(riscv, TargetRiscvDm::kOpHalt, conn, out);
    const uint32_t halt_ms = millis() - before;
    CHECK(r.detail == kOutcomeFailed && out.size() == 1 && out[0] == kStatusTimeout);
    CHECK(halt_ms >= reg::kLimitDmWaitMs && halt_ms <= reg::kLimitDmWaitMs + 5);
    CHECK(phy.dmcontrol == 1 && !dm.halted());   // haltreq cleared, dmactive kept
    phy.ignore_halt = phy.ignore_resume = false;
    r = call(riscv, TargetRiscvDm::kOpHalt, conn, out);
    CHECK(ok(r) && phy.halted);
    phy.ignore_halt = phy.ignore_resume = true;
    before = millis();
    r = call(riscv, TargetRiscvDm::kOpResume, conn, out);
    const uint32_t resume_ms = millis() - before;
    CHECK(r.detail == kOutcomeFailed && out.size() == 1 && out[0] == kStatusState);
    CHECK(resume_ms >= reg::kLimitDmWaitMs && resume_ms <= reg::kLimitDmWaitMs + 5);
    phy.ignore_halt = phy.ignore_resume = false;
    // step (oep-if-debug §4.2): back by itself - ok; not back in dm_wait_ms but stopped by the probe's haltreq - state,
    // dpc_after valid, haltreq lowered; still running dm_wait_ms after that - state with TLV 0x01 step_left (01 00 00),
    // haltreq cleared (it stepped twice the old 50 ms, then halted, and answered ok)
    phy.step_returns = true;
    r = call(riscv, TargetRiscvDm::kOpStep, conn, out);
    CHECK(ok(r) && out.size() == 10 && out[0] == kStatusOk && phy.halted);
    phy.step_returns = false;
    before = millis();
    r = call(riscv, TargetRiscvDm::kOpStep, conn, out);
    CHECK(r.detail == kOutcomeFailed && out.size() == 10 && out[0] == kStatusState && phy.halted && phy.dmcontrol == 1);
    CHECK(millis() - before >= reg::kLimitDmWaitMs && millis() - before <= reg::kLimitDmWaitMs + 10);
    phy.ignore_halt = true;
    before = millis();
    r = call(riscv, TargetRiscvDm::kOpStep, conn, out);
    CHECK(r.detail == kOutcomeFailed && out.size() == 13 && out[0] == kStatusState && out[10] == 0x01 && out[11] == 0 &&
          out[12] == 0);
    CHECK(!phy.halted && phy.dmcontrol == 1 && !dm.halted());
    CHECK(millis() - before >= 2 * reg::kLimitDmWaitMs && millis() - before <= 2 * reg::kLimitDmWaitMs + 10);
    phy.ignore_halt = false;
    // a dmi request within max_op_ms of time (oep-if-debug §4.1): one that reaches it ends at that step with status
    // timeout, done = the step's index (it ran on: a poll of 65535 reads at 200 us each took 13 s)
    phy.read_us = 200;
    phy.data0 = 0;
    uint32_t t0 = millis();
    r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 1, 0, 0x03, 0x04, 0xff, 0xff, 0xff, 0xff, 1, 0, 0, 0,
                                            0xff, 0xff}, out);
    CHECK(out.size() == 9 && out[0] == 0 && out[2] == kStatusTimeout && out[3] == 1);
    CHECK(millis() - t0 >= kMaxOpMs && millis() - t0 <= kMaxOpMs + 5);
    // waits adding up to max_op_ms (allowed) with a read between: the last wait is cut where the request reaches it
    const uint32_t half_us = kMaxOpMs * 500u;
    Bytes waits = {conn[0], conn[1], 3, 0, 0x04};
    for (int b = 0; b < 4; ++b) waits.push_back(uint8_t(half_us >> (8 * b)));
    waits.insert(waits.end(), {0x02, 0x04, 0x04});
    for (int b = 0; b < 4; ++b) waits.push_back(uint8_t(half_us >> (8 * b)));
    t0 = millis();
    r = call(riscv, TargetRiscvDm::kOpDmi, waits, out);
    CHECK(r.detail == kOutcomePartial && out.size() == 9 && out[0] == 2 && out[2] == kStatusTimeout && out[3] == 1);
    CHECK(millis() - t0 >= kMaxOpMs && millis() - t0 <= kMaxOpMs + 1);
    phy.read_us = 10;
    // run leaves ebreakm and prv = M changed only (oep-if-debug §4.4; it left ebreaks and ebreaku set). They are set
    // for the run itself and put back once the hart has stopped: a loader the hart still runs in U mode stops at its
    // ebreak (without ebreaku it trapped, and the run timed out - ch32rv uploads to a CH32L103 through the RP2350)
    r = call(riscv, TargetRiscvDm::kOpHalt, conn, out);
    CHECK(ok(r) && phy.halted);
    phy.model_dcsr = true;
    phy.dcsr = 0x00000040u;   // dcsr as read: prv 0 (U), bit 6 kept
    phy.dcsr_written = 0;
    phy.dcsr_writes.clear();
    r = call(riscv, TargetRiscvDm::kOpRun, {conn[0], conn[1], 0, 0, 0, 0x20, 1, 0, 0, 0, 0, 0}, out);   // timeout 1 ms
    CHECK(r.resolution == kResolutionCompleted && phy.dcsr_written == 0x8043u);
    CHECK(phy.dcsr_writes.size() == 2 && phy.dcsr_writes[0] == 0xb043u);   // for the run: ebreaks / ebreaku too
    phy.model_dcsr = false;
    // a target whose DMI link drops at every change of hart state (a CH32L103: DMSTATUS reads all ones until the link
    // is brought up again): run, step and resume relink on such a DMSTATUS as on a failed read. They relinked only on a
    // failed read, and run timed out with the hart stopped at its ebreak (ch32rv uploads through the RP2350, 1c940ca)
    phy.drop_on_change = true;
    r = call(riscv, TargetRiscvDm::kOpHalt, conn, out);
    CHECK(ok(r) && phy.halted);
    phy.step_returns = true;   // the run reaches its ebreak at once
    uint32_t before_run = millis();
    r = call(riscv, TargetRiscvDm::kOpRun, {conn[0], conn[1], 0, 0, 0, 0x20, 0xe8, 0x03, 0, 0, 0, 0}, out);   // 1000 ms
    CHECK(ok(r) && out.size() >= 11 && out[0] == kStatusOk && out[1] == reg::target_riscv_dm::kRunStoppedStopped);
    CHECK(millis() - before_run < 50);
    r = call(riscv, TargetRiscvDm::kOpStep, conn, out);
    CHECK(ok(r) && out.size() == 10 && out[0] == kStatusOk);
    phy.step_returns = false;
    r = call(riscv, TargetRiscvDm::kOpResume, conn, out);
    CHECK(ok(r) && !phy.halted);
    r = call(riscv, TargetRiscvDm::kOpHalt, conn, out);
    CHECK(ok(r) && phy.halted);
    phy.drop_on_change = phy.dropped = false;
    // the high-level ops on hart 0 (oep-if-debug §4): a hartsel the host's dmi left goes back to 0 (it was left)
    for (uint8_t op : {TargetRiscvDm::kOpHalt, TargetRiscvDm::kOpResume, TargetRiscvDm::kOpHalt}) {
      r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 1, 0, 0x01, 0x10, 0x01, 0x00, 0x05, 0x00}, out);
      CHECK(ok(r) && phy.hartsel == 0x00050000u);
      r = call(riscv, op, conn, out);
      CHECK(ok(r) && phy.hartsel == 0);
    }
    // reset (oep-if-debug §4.3): the procedure redone at most once (reset_retries; it ran 3 times), and a cmderr of
    // its own abstract commands is status fault (it was timeout)
    phy.ignore_resume = true;
    r = call(riscv, TargetRiscvDm::kOpReset, {conn[0], conn[1], 0}, out);
    CHECK(r.detail == kOutcomeFailed && out.size() == 7 && out[0] == kStatusTimeout && out[2] == 2 &&
          (out[1] & reg::target_riscv_dm::kResetFlagsRetried));
    phy.ignore_resume = false;
    phy.abstract_cmderr = 2;
    r = call(riscv, TargetRiscvDm::kOpReset, {conn[0], conn[1], 2}, out);
    CHECK(r.detail == kOutcomeFailed && out.size() == 7 && out[0] == kStatusFault);
    phy.abstract_cmderr = 0;
    r = call(riscv, TargetRiscvDm::kOpReset, {conn[0], conn[1], 2}, out);
    CHECK(ok(r) && out[0] == kStatusOk && out[2] == 1 && (out[1] & reg::target_riscv_dm::kResetFlagsReached));
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    phy.halted = false;
  }

  // ---- a link that drops at every change of hart state (a CH32L103), under the block ops, run and resume, with the
  // abstract commands modelled (program buffer, autoexecdata): the line reads all ones until the link is brought up
  // again, or the last value read (oep-if-debug §4's table: a read right after the change may give the previous value),
  // and writes are lost meanwhile. checkHalted looked once: after a host's raw halt the block op answered line (all
  // ones) or state with 0 words (the stale "running" - "write_block stopped after 0: state" on the L103); the run's wait
  // relinked only on all ones, so a stale "running" kept it to its timeout with the hart at its ebreak, and a stale
  // "halted" hid a resume. A block op and a run put abstractauto back as they found it (oep-if-debug §4's table); they
  // forced 0, and a host's raw ABSTRACTAUTO = 1 made the op's first DATA0 access run the last command again ----
  for (bool stale_reads : {false, true}) {
    phy.halted = false;
    phy.model_block = phy.drop_on_change = phy.drop_loses_writes = true;
    phy.drop_stale = stale_reads;
    phy.abstractauto = phy.cmderr = 0;
    phy.mem.clear();
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1), out);
    CHECK(ok(r) && phy.halted && fixed.connected);
    const Bytes conn = {uint8_t(fixed.number), uint8_t(fixed.number >> 8)};
    for (uint32_t k = 1; k < 32; ++k) phy.gpr[k] = 0x11110000u + k;   // the target's GPRs (s0, s1, a0, a1 the ops use)
    phy.data0 = 0x0000aa55u;
    phy.data1 = 0x12345678u;
    auto writeBlock = [&](uint32_t address, uint32_t first, uint16_t n) {
      Bytes req = conn;
      for (int b = 0; b < 4; ++b) req.push_back(uint8_t(address >> (8 * b)));
      req.insert(req.end(), {uint8_t(n), uint8_t(n >> 8)});
      for (uint16_t i = 0; i < n; ++i) for (int b = 0; b < 4; ++b) req.push_back(uint8_t((first + i) >> (8 * b)));
      return call(riscv, TargetRiscvDm::kOpWriteBlock, req, out);
    };
    auto landed = [&](uint32_t address, uint32_t first, uint16_t n) {
      for (uint16_t i = 0; i < n; ++i) if (phy.mem[address + 4u * i] != first + i) return false;
      return true;
    };
    auto keptAsFound = [&]() {   // every GPR
      for (uint32_t k = 1; k < 32; ++k) if (phy.gpr[k] != 0x11110000u + k) return false;
      return phy.data0 == 0x0000aa55u && phy.data1 == 0x12345678u;
    };
    r = writeBlock(0x20000000u, 0xc0de0000u, 4);
    CHECK(ok(r) && out.size() == 3 && out[2] == kStatusOk && landed(0x20000000u, 0xc0de0000u, 4));
    CHECK(keptAsFound() && phy.abstractauto == 0);
    r = call(riscv, TargetRiscvDm::kOpReadBlock, {conn[0], conn[1], 0, 0, 0, 0x20, 4, 0}, out);
    CHECK(ok(r) && out.size() == 19 && out[2] == kStatusOk && out[3] == 0x00 && out[5] == 0xde && out[15] == 0x03);
    CHECK(keptAsFound());
    // the hart let run, then halted by the host's raw dmi write - a change the probe did not make: the link dropped
    r = call(riscv, TargetRiscvDm::kOpResume, conn, out);
    CHECK(ok(r) && !phy.halted);
    phy.dropped = false;   // the drop at the resume is over (a host's request later finds it back, the PHY's revive)
    // the haltreq lands and the link drops at the change: the look after the step (P4) meets the drop - line, done 1
    // (the write may have been done), no values
    r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 1, 0, 0x01, 0x10, 0x01, 0x00, 0x00, 0x80}, out);
    CHECK(out.size() == 5 && out[0] == 1 && out[1] == 0 && out[2] == kStatusLine && out[3] == 0 && phy.halted &&
          phy.dropped);
    r = writeBlock(0x20000100u, 0xbeef0000u, 3);
    CHECK(ok(r) && out.size() == 3 && out[0] == 3 && out[2] == kStatusOk && landed(0x20000100u, 0xbeef0000u, 3));
    CHECK(keptAsFound());
    // run: the hart runs a few reads' worth and stops at its ebreak, the link dropping at both changes
    r = call(riscv, TargetRiscvDm::kOpHalt, conn, out);   // (the link in step: the run on its own)
    CHECK(ok(r) && phy.halted);
    phy.run_reads = 5;
    const uint32_t before_run = millis();
    r = call(riscv, TargetRiscvDm::kOpRun, {conn[0], conn[1], 0, 0, 0, 0x20, 0xe8, 0x03, 0, 0, 0, 0}, out);   // 1000 ms
    CHECK(ok(r) && out.size() >= 11 && out[0] == kStatusOk && out[1] == reg::target_riscv_dm::kRunStoppedStopped);
    CHECK(millis() - before_run < 50 && phy.halted);
    phy.run_reads = 0;
    // resume: the hart leaves debug mode; a stale "halted" read after it no longer hides that
    r = call(riscv, TargetRiscvDm::kOpResume, conn, out);
    CHECK(ok(r) && out[0] == kStatusOk && !phy.halted);
    r = call(riscv, TargetRiscvDm::kOpHalt, conn, out);
    CHECK(ok(r) && phy.halted);
    // abstractauto as the op found it: a host's raw ABSTRACTAUTO = 1 (autoexecdata 0) is put back, and the op's own
    // accesses to DATA0 run no command of it (the sentinel past the block stays)
    phy.mem[0x20000210u] = 0x5e5e5e5eu;
    r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 1, 0, 0x01, 0x18, 0x01, 0x00, 0x00, 0x00}, out);
    CHECK(ok(r) && phy.abstractauto == 1);
    phy.data0 = 0x0000aa55u;
    phy.data1 = 0x12345678u;
    const uint32_t runs = phy.autoexec_runs;
    r = writeBlock(0x20000200u, 0x0d0d0000u, 4);
    CHECK(ok(r) && out[2] == kStatusOk && landed(0x20000200u, 0x0d0d0000u, 4) && phy.mem[0x20000210u] == 0x5e5e5e5eu);
    CHECK(keptAsFound() && phy.abstractauto == 1);
    CHECK(phy.autoexec_runs - runs == 3);   // the block's own words 1-3, nothing else
    r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 1, 0, 0x01, 0x18, 0x00, 0x00, 0x00, 0x00}, out);
    CHECK(ok(r) && phy.abstractauto == 0);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    phy.model_block = phy.drop_on_change = phy.drop_loses_writes = phy.drop_stale = phy.dropped = false;
    phy.halted = false;
  }

  // ---- an undefined attach method (2+): unsupported, payload 0x00 (C-02) ----
  {
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(2), out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out.size() == 1 && out[0] == 0);
    // ... but a format error anywhere in the request comes first (core §4.3 order 5 before 6; it answered unsupported):
    // max_speed absent, a reset TLV of the wrong length, an unknown critical tag with a max_speed of the wrong length
    r = call(wire_fixed, WireRvswd::kOpAttach, {2}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    Bytes bad_reset = attachRequest(2);
    bad_reset.insert(bad_reset.end(), {uint8_t(wire::kTlvAttachReset | kTagCritical), 2, 0, 8, 0});
    r = call(wire_fixed, WireRvswd::kOpAttach, bad_reset, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    r = call(wire_fixed, WireRvswd::kOpAttach, {0, 0xbf, 0, 0, uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 2, 0, 1, 0}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    r = call(wire_fixed, WireRvswd::kOpScan, {0, 0xbf, 0, 0, uint8_t(wire::kTlvScanMaxSpeed), 2, 0, 1, 0}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    r = call(wire_fixed, WireRvswd::kOpScan, {0, 0xbf, 0, 0}, out);   // the unknown critical tag alone: unsupported
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out == Bytes({0xbf}));
    // riscv-dm reset: a method TLV shorter than its form is malformed before a mode of 3 is unsupported; one longer
    // than its form (a request TLV never grows, core §2.3), critical: unsupported with the tag as received
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    r = call(riscv, TargetRiscvDm::kOpReset, {uint8_t(fixed.number), uint8_t(fixed.number >> 8), 3,
                                              uint8_t(reg::target_riscv_dm::kTlvResetMethod | kTagCritical), 0, 0}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    r = call(riscv, TargetRiscvDm::kOpReset, {uint8_t(fixed.number), uint8_t(fixed.number >> 8), 2,
                                              uint8_t(reg::target_riscv_dm::kTlvResetMethod | kTagCritical), 2, 0, 0, 0}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported &&
          out == Bytes({uint8_t(reg::target_riscv_dm::kTlvResetMethod | kTagCritical)}));
    r = call(riscv, TargetRiscvDm::kOpReset, {uint8_t(fixed.number), uint8_t(fixed.number >> 8), 3}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out == Bytes({0}));
    // a connection it does not know is the last refusal (core §4.3 order 8; it answered no_connection first)
    const uint16_t gone = uint16_t(fixed.number + 100);
    r = call(riscv, TargetRiscvDm::kOpDmi, {uint8_t(gone), uint8_t(gone >> 8), 1, 0, 0x07}, out);   // unknown step kind
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    r = call(riscv, TargetRiscvDm::kOpReadBlock, {uint8_t(gone), uint8_t(gone >> 8), 2, 0, 0, 0x20, 1, 0}, out);   // address & 3
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    r = call(riscv, TargetRiscvDm::kOpReset, {uint8_t(gone), uint8_t(gone >> 8), 3}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported);
    r = call(riscv, TargetRiscvDm::kOpHalt, {uint8_t(gone), uint8_t(gone >> 8)}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectNoConnection);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
  }

  // ---- the attach budget: attach() tried again only while it lasts ----
  {
    phy.fail_attaches = 100;
    phy.attach_ms = 400;
    phy.attaches = 0;
    const uint32_t before = millis();
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(r.resolution == kResolutionCompleted && r.detail != kOutcomeSuccess && out.size() >= 1 && out[0] == kStatusLine);
    CHECK(phy.attaches == 2 && millis() - before <= reg::kLimitAttachBudgetMs);   // a third would start at 800 ms
    phy.fail_attaches = 0;
    phy.attach_ms = 0;
  }

  // ---- the scan budget: no pair starts 500 ms after the request ----
  {
    static FakePhy phy3;
    static Ch32Dm dm3(phy3);
    static PinTable pins3((1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7));
    static DebugPort chosen{dm3, 0xfffe, 0xfffe};
    chosen.pins = &pins3;
    chosen.pin_choice = (1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7);
    static WireRvswd wire3(chosen, 2);
    phy3.present = false;
    phy3.bring_up_ms = 200;
    const Bytes scan_all = {0};
    Result r = call(wire3, WireRvswd::kOpScan, scan_all, out);
    CHECK(ok(r) && out.size() >= 2 && out[0] == 3 && out[1] == 0);   // at 0, 200 and 400 ms; not at 600
    CHECK(phy3.bring_ups == 3);
  }

  // ---- wire retries inside a request: at most wire_retry_ms, the next request starts afresh (oep-if-debug §2) ----
  {
    phy.flaky = false;
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    phy.flaky = true;
    phy.retry_reads = 0;
    // dmi: three reads of DMSTATUS
    const Bytes reads = {uint8_t(fixed.number), uint8_t(fixed.number >> 8), 3, 0, 0x02, 0x11, 0x02, 0x11, 0x02, 0x11};
    const uint32_t before = millis();
    r = call(riscv, TargetRiscvDm::kOpDmi, reads, out);
    CHECK(r.resolution == kResolutionCompleted && out.size() >= 3 && out[2] == kStatusLine);
    CHECK(phy.retry_reads == reg::kLimitWireRetryMs && millis() - before >= reg::kLimitWireRetryMs &&
          millis() - before < reg::kLimitWireRetryMs + 20);
    // one request that got nothing back is not wire loss: status line, the connection kept (0.0.28: closed)
    CHECK(fixed.connected);
    // ... nor are failures for less than wire_lost_ms: still kept 900 ms after the first failure
    g_millis += 600;
    r = call(riscv, TargetRiscvDm::kOpDmi, reads, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine && fixed.connected);
    // a good exchange in between stops the clock
    phy.flaky = false;
    r = call(riscv, TargetRiscvDm::kOpDmi, reads, out);
    CHECK(ok(r) && fixed.connected);
    phy.flaky = true;
    r = call(riscv, TargetRiscvDm::kOpDmi, reads, out);   // the clock starts again here
    CHECK(out[2] == kStatusLine && fixed.connected);
    g_millis += reg::kLimitWireLostMs - 200;               // with this request's 200: 1000 ms since this run's first failure
    r = call(riscv, TargetRiscvDm::kOpDmi, reads, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine);
    CHECK(!fixed.connected && fixed.lost);                 // wire_lost_ms of failures: the answer, then closed
    phy.flaky = false;
    phy.present = true;
  }

  // ---- the liveness check (probe.config §3.1) runs the same clock: one failed check keeps the connection ----
  {
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    CHECK(checkConnection(fixed));
    phy.present = false;
    CHECK(checkConnection(fixed) && fixed.connected);      // 0.0.28: three failed reads closed it
    g_millis += 500;
    CHECK(checkConnection(fixed) && fixed.connected);
    g_millis += 500;
    CHECK(!checkConnection(fixed) && !fixed.connected && fixed.lost);
    phy.present = true;
  }

  // ---- a line that reads all zeros / all ones (no module behind it) is no good exchange: it closes after wire_lost_ms
  // (0.0.28: each DMSTATUS read of it counted as an answer, and the clock restarted at every check - never closed) ----
  for (uint32_t stuck : {0u, 0xffffffffu}) {
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    phy.stuck = true;
    phy.stuck_value = stuck;
    CHECK(checkConnection(fixed) && fixed.connected);
    g_millis += 500;
    CHECK(checkConnection(fixed) && fixed.connected);
    // a host's dmi read of DATA0 in between (the same value: it may be the register's own) does not stop the clock -
    // the request's look at the link before its steps (P4) gets no answer: line, nothing run, no values
    const Bytes data0 = {uint8_t(fixed.number), uint8_t(fixed.number >> 8), 1, 0, 0x02, 0x04};
    r = call(riscv, TargetRiscvDm::kOpDmi, data0, out);
    CHECK(out.size() == 5 && out[0] == 0 && out[2] == kStatusLine && out[3] == 0 && fixed.connected);
    g_millis += 500;
    CHECK(!checkConnection(fixed) && !fixed.connected && fixed.lost);
    // riscv-dm's ops see it the same way: halt answers line, and the connection closes after wire_lost_ms
    phy.stuck = false;
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    phy.stuck = true;
    const Bytes halt = {uint8_t(fixed.number), uint8_t(fixed.number >> 8)};
    phy.halted = false;
    r = call(riscv, TargetRiscvDm::kOpHalt, halt, out);
    CHECK(out.size() == 1 && out[0] == kStatusLine && fixed.connected);
    g_millis += reg::kLimitWireLostMs;
    r = call(riscv, TargetRiscvDm::kOpHalt, halt, out);
    CHECK(out.size() == 1 && out[0] == kStatusLine && !fixed.connected);
    phy.stuck = false;
    phy.halted = false;
  }

  // ---- every request on the connection looks at the wire-loss clock, whatever its op answered (oep-if-debug §2) ----
  // The target's power floating: DMSTATUS reads all ones. 0.0.28+68d9694: a host's dmi read of DMSTATUS answered ok
  // with 0xffffffff, and the connection never closed.
  {
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    Bytes read = {uint8_t(fixed.number), uint8_t(fixed.number >> 8), 1, 0, 0x02, 0x11};
    phy.stuck = true;
    phy.stuck_value = 0xffffffffu;
    // a DMSTATUS of all ones is no answer: the step fails with line, no value; one such request keeps the connection
    r = call(riscv, TargetRiscvDm::kOpDmi, read, out);
    CHECK(r.detail == kOutcomeFailed && out.size() == 5 && out[0] == 0 && out[2] == kStatusLine && out[3] == 0);
    CHECK(fixed.connected);
    // a poll of DMSTATUS for allhalted is not met by all ones (0.0.28+68d9694: met at once)
    Bytes poll = {uint8_t(fixed.number), uint8_t(fixed.number >> 8), 1, 0, 0x03, 0x11};
    for (uint32_t v : {1u << 9, 1u << 9}) for (int b = 0; b < 4; ++b) poll.push_back(uint8_t(v >> (8 * b)));
    poll.insert(poll.end(), {10, 0});
    r = call(riscv, TargetRiscvDm::kOpDmi, poll, out);
    CHECK(out.size() == 5 && out[2] == kStatusLine && fixed.connected);
    // resume is not acknowledged by all ones (allresumeack reads set): line, not ok
    const Bytes conn = {uint8_t(fixed.number), uint8_t(fixed.number >> 8)};
    r = call(riscv, TargetRiscvDm::kOpResume, conn, out);
    CHECK(out.size() == 1 && out[0] == kStatusLine && fixed.connected);
    // a block op on a hart whose DMSTATUS reads all ones: line, not state
    const Bytes block = {uint8_t(fixed.number), uint8_t(fixed.number >> 8), 0, 0, 0, 0x20, 1, 0};
    r = call(riscv, TargetRiscvDm::kOpReadBlock, block, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine && fixed.connected);
    // wire_lost_ms after the first: the request that sees it answers line and the connection closes after it
    g_millis += reg::kLimitWireLostMs;
    r = call(riscv, TargetRiscvDm::kOpDmi, read, out);
    CHECK(out.size() == 5 && out[2] == kStatusLine && !fixed.connected && fixed.lost);

    // reads of another register (all ones may be its own value: no answer either way) once the clock runs: the
    // request's look at the link before its steps (P4: DMSTATUS, DMCONTROL) gets no answer, so it answers line with
    // nothing run and no values, the clock running on; after wire_lost_ms the connection closes
    phy.stuck = false;
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    read[0] = uint8_t(fixed.number);
    read[1] = uint8_t(fixed.number >> 8);
    phy.stuck = true;
    r = call(riscv, TargetRiscvDm::kOpDmi, read, out);   // DMSTATUS: the clock starts
    CHECK(out[2] == kStatusLine && fixed.connected);
    const Bytes data0 = {uint8_t(fixed.number), uint8_t(fixed.number >> 8), 2, 0, 0x02, 0x04, 0x02, 0x04};
    g_millis += 500;
    r = call(riscv, TargetRiscvDm::kOpDmi, data0, out);
    CHECK(out.size() == 5 && out[0] == 0 && out[2] == kStatusLine && out[3] == 0 && fixed.connected);
    g_millis += 500;
    r = call(riscv, TargetRiscvDm::kOpDmi, data0, out);
    CHECK(r.detail == kOutcomeFailed && out.size() == 5 && out[0] == 0 && out[2] == kStatusLine && out[3] == 0);
    CHECK(!fixed.connected && fixed.lost);

    // a request with no read at all (a delay) on a clock that has run out: the answer is line all the same, closed
    phy.stuck = false;
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    phy.stuck = true;
    r = call(riscv, TargetRiscvDm::kOpDmi, {uint8_t(fixed.number), uint8_t(fixed.number >> 8), 1, 0, 0x02, 0x11}, out);
    g_millis += reg::kLimitWireLostMs;
    r = call(riscv, TargetRiscvDm::kOpDmi, {uint8_t(fixed.number), uint8_t(fixed.number >> 8), 1, 0, 0x04, 0, 0, 0, 0}, out);
    CHECK(r.detail == kOutcomeFailed && out.size() == 5 && out[2] == kStatusLine && !fixed.connected);   // a delay alone
    phy.stuck = false;
    phy.halted = false;

    // scan through the live connection runs the same clock: lost there, the connection closes
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    phy.stuck = true;
    CHECK(checkConnection(fixed));
    g_millis += reg::kLimitWireLostMs;
    r = call(wire_fixed, WireRvswd::kOpScan, {0}, out);
    CHECK(ok(r) && out.size() >= 2 && out[1] == 0 && !fixed.connected && fixed.lost);
    phy.stuck = false;
  }

  // ---- attach to a live connection whose module does not answer: the same connection, brought up afresh ----
  // The target's power floated and came back: the link the probe held reads all ones. 0.0.28+68d9694: every attach
  // answered timeout (the all-ones DMSTATUS read as halted, the halt never landed) until a forced detach.
  for (bool lost : {false, true}) {
    phy.halted = false;
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    const uint16_t number = fixed.number;
    const uint32_t closes = fixed.closes;
    phy.stale = true;
    phy.havereset = true;
    if (lost) {   // the clock ran out with no request to see it (idle connections are not watched)
      CHECK(checkConnection(fixed));   // the clock starts
      g_millis += reg::kLimitWireLostMs + 500;
    }
    const int attaches = phy.attaches;
    for (uint8_t method : {uint8_t(0), uint8_t(1)}) {
      r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(method), out);
      CHECK(ok(r) && fixed.connected && fixed.number == number && fixed.closes == closes);
      CHECK(out.size() >= 11 && (out[0] | out[1] << 8) == number && (out[6] & wire::kAttachFlagsExisting));
      if (method == 0) {
        CHECK(phy.attaches == attaches + 1);                          // brought up afresh
        CHECK(out[6] & wire::kAttachFlagsHaveresetAcked);             // the power-up's havereset acknowledged
        size_t len = 0;
        CHECK(answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len) != nullptr);   // a bring-up ran
      } else {
        CHECK(phy.halted && (out[6] & wire::kAttachFlagsHalted));     // the halt lands on the fresh link
      }
    }
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    phy.halted = false;
  }
  // ... and one whose target is still gone: line, the connection kept until wire_lost_ms
  {
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    phy.present = false;
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(r.detail == kOutcomeFailed && out.size() >= 1 && out[0] == kStatusLine && fixed.connected);
    g_millis += reg::kLimitWireLostMs;
    r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(r.detail == kOutcomeFailed && out[0] == kStatusLine && !fixed.connected && fixed.lost);
    phy.present = true;
  }

  // ---- the console's reads run the same clock: lost only after wire_lost_ms of reads that got nothing ----
  {
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    static DmConsole console(dm, phy);
    CHECK(console.start(0));
    console.poll();
    CHECK(!console.lineLost());
    phy.present = false;
    for (int i = 0; i < 9; ++i) { g_millis += 100; console.poll(); }
    CHECK(!console.lineLost());
    phy.present = true;
    g_millis += 100;
    console.poll();                                        // an answer: the clock stops
    phy.present = false;
    for (int i = 0; i < 9; ++i) { g_millis += 100; console.poll(); }
    CHECK(!console.lineLost());
    // wire_lost_ms after the first read that got nothing: link-lost
    g_millis += 200;
    console.poll();
    CHECK(console.lineLost());
    console.stop();
    phy.present = true;
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r));
  }

  // ---- the console on a line that reads all ones (no module): link-lost after wire_lost_ms (0.0.28: never) ----
  {
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
    CHECK(ok(r) && fixed.connected);
    static DmConsole console2(dm, phy);
    CHECK(console2.start(0));
    console2.poll();
    phy.stuck = true;
    phy.stuck_value = 0xffffffffu;
    for (int i = 0; i < 9; ++i) { g_millis += 100; console2.poll(); }
    CHECK(!console2.lineLost());
    g_millis += 200;
    console2.poll();
    CHECK(console2.lineLost());
    console2.stop();
    phy.stuck = false;
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r));
  }

  // ---- the wire-loss clock: a reset's hold and the wire_lost_ms after it are not counted ----
  {
    WireLossClock clock;
    clock.silent();
    g_millis += 300;
    clock.excuseReset();                                   // a reset line let go of now
    g_millis += 900;
    CHECK(!clock.lost());                                  // 1200 ms of failures, all but 300 inside the excuse
    g_millis += 100;
    CHECK(!clock.lost());
    g_millis += reg::kLimitWireLostMs;
    CHECK(clock.lost());
    clock.answered();
    CHECK(!clock.lost());
  }

  // ---- ch32rv's upload, 100 times, over a link that drops at every change of hart state with random timing (a
  // CH32L103 on the RP2350; the bench saw 1 in 15 fail back to back, 3 in 12 with other sessions between, run: timeout):
  // a slot's at-boot connection resting idle_clock low; each upload a session that attaches (joining it, idle_clock low
  // carried), halts, writes the loader, runs it to its ebreak with arguments, reads the results and resumes; then the
  // session ends (its share goes, the slot keeps the connection) and, half the time, another session uses the probe
  // (a raw DMSTATUS read, a scan, an attach / detach, a halt / resume). Every answer must be ok ----
  {
    phy.halted = false;
    phy.model_block = phy.drop_on_change = phy.drop_loses_writes = true;
    phy.abstractauto = phy.cmderr = 0;
    phy.mem.clear();
    uint32_t status = 0;
    CHECK(attachRunning(fixed, DebugPort::kUserSlot, status, 1000000, true) && fixed.connected && phy.idle_low);
    const Bytes conn = {uint8_t(fixed.number), uint8_t(fixed.number >> 8)};
    uint32_t seed = OEP_UPLOAD_SEED;
    auto rnd = [&](uint32_t n) { seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; return seed % n; };
    int bad = 0;
    const char *last_failed = "";
    for (int i = 0; i < 100; ++i) {
      phy.drop_delay_reads = static_cast<int>(rnd(4));
      phy.drop_hold_us = rnd(4) * 700;   // 0 .. 2.1 ms
      phy.drop_stale = rnd(2);
      phy.run_reads = 1 + static_cast<int>(rnd(30));
      const char *failed = nullptr;
      Bytes failed_answer;
      auto expect = [&](bool good, const char *what) { if (!good && !failed) { failed = what; failed_answer = out; } };
      Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1, -1, -1, 1), out);
      expect(ok(r) && (out[6] & wire::kAttachFlagsExisting) && phy.halted && phy.idle_low, "attach");
      r = call(riscv, TargetRiscvDm::kOpHalt, conn, out);
      expect(ok(r) && phy.halted, "halt");
      Bytes wb = conn;
      wb.insert(wb.end(), {0, 0, 0, 0x20, 64, 0});   // the loader: 64 words at 0x20000000
      for (uint32_t k = 0; k < 64; ++k) for (int b = 0; b < 4; ++b) wb.push_back(uint8_t((0x1000u * i + k) >> (8 * b)));
      // every GPR as the block ops found it (the sketch's, then the loader's after its run)
      uint32_t gprs[32];
      auto snapshot = [&]() { for (int k = 0; k < 32; ++k) gprs[k] = phy.gpr[k] = k ? 0x3c000000u | (uint32_t(i) << 8) | uint32_t(k) : 0; };
      auto unchanged = [&]() { for (int k = 0; k < 32; ++k) if (phy.gpr[k] != gprs[k]) return false; return true; };
      snapshot();
      r = call(riscv, TargetRiscvDm::kOpWriteBlock, wb, out);
      expect(ok(r) && out.size() == 3 && out[2] == kStatusOk && phy.mem[0x20000000u + 4 * 63] == 0x1000u * i + 63, "write_block");
      expect(unchanged(), "write_block's GPRs");
      // run: pc 0x20000000, 1000 ms, a0 = 0x20000400 and a1 = 16 in, a0 out
      Bytes run = conn;
      run.insert(run.end(), {0, 0, 0, 0x20, 0xe8, 0x03, 0, 0, 2, 0x0a, 0x10, 0, 0x04, 0, 0x20, 0x0b, 0x10, 16, 0, 0, 0,
                             1, 0x0a, 0x10});
      const uint32_t t0 = millis();
      r = call(riscv, TargetRiscvDm::kOpRun, run, out);
      expect(ok(r) && out.size() >= 11 + 4 && out[0] == kStatusOk && out[1] == reg::target_riscv_dm::kRunStoppedStopped,
             "run");
      expect(millis() - t0 < 200, "run's time");
      // the arguments landed (a write lost to a drop would start the loader on a register as it was) and a0 came back
      expect(phy.gpr[10] == 0x20000400u && phy.gpr[11] == 16 && out.size() >= 15 && out[10] == 1 &&
             out[11] == 0x00 && out[12] == 0x04 && out[13] == 0x00 && out[14] == 0x20, "run's registers");
      snapshot();
      r = call(riscv, TargetRiscvDm::kOpReadBlock, {conn[0], conn[1], 0, 0x04, 0, 0x20, 16, 0}, out);
      expect(ok(r) && out.size() == 3 + 64 && out[2] == kStatusOk, "read_block");
      expect(unchanged(), "read_block's GPRs");
      phy.run_reads = 0;   // the application runs on (no ebreak ahead)
      r = call(riscv, TargetRiscvDm::kOpResume, conn, out);
      expect(ok(r) && out[0] == kStatusOk && !phy.halted, "resume");
      wire_fixed.sessionOver();   // the upload's session ends: its share goes, the slot's connection stays
      expect(fixed.connected && fixed.users == DebugPort::kUserSlot && phy.idle_low, "session end");
      switch (rnd(8)) {   // another session between uploads, half the time
        case 0: r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 1, 0, 0x02, 0x11}, out); expect(ok(r), "other: dmi"); break;
        case 1: r = call(wire_fixed, WireRvswd::kOpScan, {0}, out); expect(ok(r) && out[1] == 1, "other: scan"); break;
        case 2:
          r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0), out);
          expect(ok(r) && phy.idle_low, "other: attach");
          r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
          expect(ok(r) && fixed.connected, "other: detach");
          break;
        case 3:
          r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1, -1, -1, 1), out);
          expect(ok(r), "other: attach (halt)");
          r = call(riscv, TargetRiscvDm::kOpResume, conn, out);
          expect(ok(r) && !phy.halted, "other: resume");
          wire_fixed.sessionOver();
          break;
        default: break;
      }
      g_millis += rnd(50);   // the CLI's next process
      if (failed) {
        ++bad;
        last_failed = failed;
        printf("  upload %d failed at %s (delay %d reads, hold %u us, stale %d, run %d reads): answer",
               i, failed, phy.drop_delay_reads, phy.drop_hold_us, phy.drop_stale, phy.run_reads);
        for (uint8_t b : failed_answer) printf(" %02x", b);
        printf("\n");
        phy.dropped = false;
        phy.pending_drop = -1;
      }
    }
    CHECK(bad == 0);
    if (bad) printf("  uploads: %d of 100 failed (last at %s)\n", bad, last_failed);
    releaseConnection(fixed, DebugPort::kUserSlot, true);
    phy.model_block = phy.drop_on_change = phy.drop_loses_writes = phy.drop_stale = phy.dropped = false;
    phy.pending_drop = -1;
    phy.drop_delay_reads = 0;
    phy.drop_hold_us = 0;
    phy.run_reads = 0;
    phy.idle_low = false;
    phy.halted = false;
  }

  // ---- a drop anywhere inside a block op: the link drops (a CH32L103's, late after a change of hart state) at every
  // point of read_block / write_block in turn - all ones or the last value read, held 0 / 0.7 / 2.1 ms against relinks,
  // writes lost meanwhile, a host's abstractauto set or not. Every GPR, dpc, dcsr, DATA0 / DATA1 and abstractauto as
  // found; no store outside the block; an ok answer with the right words (bench, 0.0.29-dev+9942787: in about 1 of 200
  // read_blocks on the L103 through the RP2350 a0 0x000ec8fe came back 0x20000000 - kept from a stale read or a write
  // going back lost to a drop) ----
  {
    phy.halted = false;
    phy.model_block = phy.drop_on_change = phy.drop_loses_writes = true;
    phy.abstractauto = phy.cmderr = 0;
    phy.mem.clear();
    phy.stray_stores = 0;
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1), out);
    CHECK(ok(r) && phy.halted && fixed.connected);
    const Bytes conn = {uint8_t(fixed.number), uint8_t(fixed.number >> 8)};
    int cases = 0, answered_ok = 0, clobbered = 0, wrong_words = 0, stray = 0, dropped_at_all = 0;
    int op_reads[2] = {0, 0};   // an op's reads with no drop: the drop is put at each of them
    for (int stale = 0; stale < 2; ++stale)
      for (uint32_t hold : {0u, 700u, 2100u})
        for (int op = 0; op < 2; ++op)
          for (int at = -1; at < op_reads[op]; ++at) {   // -1: no drop, the op's reads counted
            ++cases;
            uint32_t gpr[32];
            for (uint32_t k = 0; k < 32; ++k) gpr[k] = phy.gpr[k] = k ? 0x5a000000u | (k << 16) | uint32_t(at) : 0;
            phy.gpr[9] = gpr[9] = 0x20000000u;   // s1 holding the block's address (as a sketch's pointer may)
            phy.dpc = 0x00000a3cu;
            phy.dcsr = 0x4000b003u;
            phy.data0 = 0x0000aa55u;
            phy.data1 = 0x12345678u;
            const uint32_t autoexec = (at % 4 == 3) ? 1 : 0;   // a host's raw ABSTRACTAUTO = 1 now and then
            phy.abstractauto = autoexec;
            phy.last_command = 0x00221000u;                    // the host's last command: read zero (harmless when re-run)
            phy.cmderr = 0;
            for (uint32_t k = 0; k < 10; ++k) phy.mem[0x20000000u + 4 * k] = 0x0b000000u | (k << 8) | uint32_t(at);
            const uint32_t fence = phy.mem[0x20000000u + 4 * 8];
            phy.stray_stores = 0;
            phy.dropped = false;
            phy.drop_stale = stale;
            phy.drop_hold_us = hold;
            phy.drop_delay_reads = 0;
            phy.pending_drop = at;   // the drop after `at` more reads, whatever the op is doing then
            const int reads_before = phy.reads;
            if (op == 0) {
              r = call(riscv, TargetRiscvDm::kOpReadBlock, {conn[0], conn[1], 0, 0, 0, 0x20, 8, 0}, out);
            } else {
              Bytes wb = conn;
              wb.insert(wb.end(), {0, 0, 0, 0x20, 8, 0});
              for (uint32_t k = 0; k < 8; ++k) for (int b = 0; b < 4; ++b) wb.push_back(uint8_t((0xc0000000u | (k << 8) | uint32_t(at)) >> (8 * b)));
              r = call(riscv, TargetRiscvDm::kOpWriteBlock, wb, out);
            }
            if (at < 0) op_reads[op] = phy.reads - reads_before;
            else if (phy.pending_drop < 0) ++dropped_at_all;
            phy.pending_drop = -1;
            bool same = phy.dpc == 0x00000a3cu && phy.dcsr == 0x4000b003u && phy.data0 == 0x0000aa55u &&
                        phy.data1 == 0x12345678u && phy.abstractauto == autoexec;
            for (uint32_t k = 0; k < 32; ++k) same = same && phy.gpr[k] == gpr[k];
            if (!same) {
              if (clobbered < 5) {
                printf("  %s, drop at read %d (hold %u us, stale %d): changed", op ? "write_block" : "read_block", at, hold, stale);
                for (uint32_t k = 0; k < 32; ++k)
                  if (phy.gpr[k] != gpr[k]) printf(" x%u %08x -> %08x", k, gpr[k], phy.gpr[k]);
                printf(" data0 %08x data1 %08x auto %u\n", phy.data0, phy.data1, phy.abstractauto);
              }
              ++clobbered;
            }
            if (phy.stray_stores || phy.mem[0x20000000u + 4 * 8] != fence) ++stray;
            const bool good = ok(r) && out.size() >= 3 && out[2] == kStatusOk;
            if (getenv("OEP_SHOW_NOT_OK") && !good) printf("  not ok: op %d at %d hold %u stale %d status %02x\n", op, at, hold, stale, out.size() >= 3 ? out[2] : 0xff);
            if (good) {
              ++answered_ok;
              bool right = out[0] == 8;
              for (uint32_t k = 0; k < 8 && right; ++k) {
                if (op == 0) {
                  uint32_t w = 0;
                  for (int b = 0; b < 4; ++b) w |= uint32_t(out[3 + 4 * k + b]) << (8 * b);
                  right = out.size() == 3 + 32 && w == (0x0b000000u | (k << 8) | uint32_t(at));
                } else {
                  right = phy.mem[0x20000000u + 4 * k] == (0xc0000000u | (k << 8) | uint32_t(at));
                }
              }
              if (!right) ++wrong_words;
            }
            phy.dropped = false;
            g_millis += 2;   // the host's next request
          }
    CHECK(clobbered == 0);
    CHECK(stray == 0);
    CHECK(wrong_words == 0);
    CHECK(dropped_at_all == cases - 12);     // every drop met an op (12: the counting runs, no drop)
    CHECK(answered_ok == cases);             // and the ops came through them, all (checkHalted waits out a drop too)
    printf("  block ops with a drop inside (%d / %d reads): %d cases, %d ok, %d with state changed, %d stray stores, "
           "%d wrong words\n", op_reads[0], op_reads[1], cases, answered_ok, clobbered, stray, wrong_words);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    phy.model_block = phy.drop_on_change = phy.drop_loses_writes = phy.drop_stale = phy.dropped = false;
    phy.drop_hold_us = 0;
    phy.abstractauto = phy.cmderr = 0;
    phy.halted = false;
  }

  // ---- a glitch anywhere inside a block op: one DMI access missed on its own, the link up again at once and every look
  // passing - a write lost (cmderr 6 set or not), a read answering the value of the read before it - at every access of
  // read_block and write_block in turn, singly and in pairs, a host's abstractauto set or not. Every GPR, dpc, dcsr,
  // DATA0 / DATA1 and abstractauto as found after any answer; an ok answer with the right words (bench,
  // 0.0.29-dev+4c310a2, tests/hw test_wire on the L103 through the RP2350: a1 0x20004f6e came back 0x00000002 after a
  // read_block answered ok, read so twice over two sentinels by the host - and s1 -> 0x00000002 four times with
  // bd19b00) ----
  {
    phy.halted = false;
    phy.model_block = true;
    phy.abstractauto = phy.cmderr = 0;
    phy.mem.clear();
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1), out);
    CHECK(ok(r) && phy.halted && fixed.connected);
    const Bytes conn = {uint8_t(fixed.number), uint8_t(fixed.number >> 8)};
    int cases = 0, answered_ok = 0, changed_ok = 0, changed_other = 0, wrong_words = 0, stray = 0, glitched = 0;
    int op_accesses[2] = {0, 0};
    int single_cases = 0, single_bad = 0;   // one glitch: the cases, and those with state changed, a stray store or wrong words
    for (int pairs = 0; pairs < 2; ++pairs)
      for (int parity = 0; parity < 2; ++parity)
        for (int op = 0; op < 2; ++op)
          for (int at = -1; at < op_accesses[op]; ++at)
            for (int at2 = pairs ? at + 1 : -1; at2 < (pairs ? op_accesses[op] : 0); ++at2) {
              if (pairs && (at < 0 || at2 >= op_accesses[op])) break;
              ++cases;
              const uint32_t tag = uint32_t(at * 131 + at2 + 7);
              uint32_t gpr[32];
              for (uint32_t k = 0; k < 32; ++k) gpr[k] = phy.gpr[k] = k ? 0x5a000000u | (k << 16) | (tag & 0xffff) : 0;
              phy.gpr[9] = gpr[9] = 0x20000000u;
              phy.dpc = 0x00000a3cu;
              phy.dcsr = 0x4000b003u;
              phy.data0 = 0x0000aa55u;
              phy.data1 = 0x12345678u;
              const uint32_t autoexec = (at >= 0 && (at & 1)) ? 1 : 0;   // counted with 0 (the shorter op)
              phy.abstractauto = autoexec;
              phy.last_command = 0x00221000u;
              phy.cmderr = 0;
              for (uint32_t k = 0; k < 12; ++k) phy.mem[0x20000000u + 4 * k] = 0x0b000000u | (k << 8) | (tag & 0xff);
              const uint32_t fence = phy.mem[0x20000000u + 4 * 8];
              phy.stray_stores = 0;
              phy.glitch_parity = parity;
              phy.glitches = 0;
              phy.glitch_at = at;
              phy.glitch_at2 = at2;
              const int accesses_before = phy.accesses;
              if (op == 0) {
                r = call(riscv, TargetRiscvDm::kOpReadBlock, {conn[0], conn[1], 0, 0, 0, 0x20, 8, 0}, out);
              } else {
                Bytes wb = conn;
                wb.insert(wb.end(), {0, 0, 0, 0x20, 8, 0});
                for (uint32_t k = 0; k < 8; ++k) for (int b = 0; b < 4; ++b) wb.push_back(uint8_t((0xc0000000u | (k << 8) | (tag & 0xff)) >> (8 * b)));
                r = call(riscv, TargetRiscvDm::kOpWriteBlock, wb, out);
              }
              if (at < 0) op_accesses[op] = phy.accesses - accesses_before;
              if (phy.glitches) ++glitched; else if (getenv("OEP_SHOW_UNGLITCHED")) printf("  no glitch: op %d at %d / %d\n", op, at, at2);
              phy.glitch_at = phy.glitch_at2 = -1;
              bool same = phy.dpc == 0x00000a3cu && phy.dcsr == 0x4000b003u && phy.data0 == 0x0000aa55u &&
                          phy.data1 == 0x12345678u && phy.abstractauto == autoexec;
              for (uint32_t k = 0; k < 32; ++k) same = same && phy.gpr[k] == gpr[k];
              const bool good = ok(r) && out.size() >= 3 && out[2] == kStatusOk;
              if (!same) {
                if (changed_ok + changed_other < 6) {
                  printf("  %s, glitch at access %d / %d (cmderr 6 %d, auto %u): %s, changed", op ? "write_block" : "read_block",
                         at, at2, parity, autoexec, good ? "ok" : "not ok");
                  for (uint32_t k = 0; k < 32; ++k)
                    if (phy.gpr[k] != gpr[k]) printf(" x%u %08x -> %08x", k, gpr[k], phy.gpr[k]);
                  printf(" data0 %08x data1 %08x auto %u\n", phy.data0, phy.data1, phy.abstractauto);
                }
                ++(good ? changed_ok : changed_other);
              }
              const bool strayed = phy.stray_stores || phy.mem[0x20000000u + 4 * 8] != fence;
              if (strayed) ++stray;
              const int wrong_before = wrong_words;
              if (good) {
                ++answered_ok;
                bool right = out[0] == 8;
                for (uint32_t k = 0; k < 8 && right; ++k) {
                  if (op == 0) {
                    uint32_t w = 0;
                    for (int b = 0; b < 4; ++b) w |= uint32_t(out[3 + 4 * k + b]) << (8 * b);
                    right = out.size() == 3 + 32 && w == (0x0b000000u | (k << 8) | (tag & 0xff));
                  } else {
                    right = phy.mem[0x20000000u + 4 * k] == (0xc0000000u | (k << 8) | (tag & 0xff));
                  }
                }
                if (!right) ++wrong_words;
              }
              if (!pairs && at >= 0) {
                ++single_cases;
                if (!same || strayed || wrong_words != wrong_before) ++single_bad;
              }
              phy.cmderr = 0;
              g_millis += 2;
            }
    CHECK(changed_ok == 0);
    CHECK(changed_other == 0);
    CHECK(stray == 0);
    CHECK(wrong_words == 0);
    CHECK(glitched == cases - 4);   // every glitch met the op (4: the counting runs)
    printf("  block ops with glitches inside (%d / %d accesses, singly and in pairs): %d cases, %d ok, %d changed with ok, "
           "%d changed otherwise, %d stray stores, %d wrong words; one glitch: %d of %d cases wrong\n", op_accesses[0],
           op_accesses[1], cases, answered_ok, changed_ok, changed_other, stray, wrong_words, single_bad, single_cases);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    phy.model_block = false;
    phy.abstractauto = phy.cmderr = 0;
    phy.glitch_parity = false;
    phy.halted = false;
  }

  // ---- write_block's stores exactly once over a link that drops (bench, f594f04: ch32rv writes the CH32L103's flash
  // keys - KEYR KEY1 / KEY2, MODEKEYR KEY1 / KEY2 - as four 1-word write_blocks; in 1 of 60 uploads CTLR stayed locked
  // after every write_block answered success). The link dropped at every DMI access of a 1-word and an 8-word
  // write_block in turn - reads and writes alike, stale or all ones, held 0 / 0.7 / 2.1 ms, writes lost. A success has
  // every word stored exactly once; a failure (fault / line) has no word stored twice, and its done says how many were
  // stored, in order. f594f04 redid the whole writer after a drop met past the store: the key written twice is the
  // wrong sequence, the flash stays locked, and the answer said success ----
  {
    phy.halted = false;
    phy.model_block = phy.drop_on_change = phy.drop_loses_writes = true;
    phy.abstractauto = phy.cmderr = 0;
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1), out);
    CHECK(ok(r) && phy.halted && fixed.connected);
    const Bytes conn = {uint8_t(fixed.number), uint8_t(fixed.number >> 8)};
    int cases = 0, succeeded = 0, failed_ok = 0, twice = 0, unstored_success = 0, wrong_done = 0, clobbered = 0;
    int other = 0, accesses_seen[3] = {0, 0, 0}, faulted = 0;
    for (int stale = 0; stale < 2; ++stale)
      for (uint32_t hold : {0u, 700u, 2100u})
        for (int big = 0; big < 3; ++big)   // 1 word, 8 words, 8 words whose 4th store faults
          for (int at = -1; at < accesses_seen[big]; ++at) {
            ++cases;
            const uint32_t words = big ? 8 : 1, address = 0x20000400u;
            phy.fault_store = big == 2 ? address + 12 : 0;
            for (uint32_t k = 0; k < 32; ++k) phy.gpr[k] = k ? 0x3c000000u | (k << 16) : 0;
            phy.data0 = 0x0000aa55u;
            phy.data1 = 0x12345678u;
            phy.cmderr = phy.abstractauto = 0;
            phy.mem.clear();
            phy.stores.clear();
            phy.stray_stores = 0;
            phy.dropped = false;
            phy.drop_stale = stale;
            phy.drop_hold_us = hold;
            phy.drop_delay_reads = 0;
            phy.pending_drop = -1;
            Bytes wb = conn;
            wb.insert(wb.end(), {uint8_t(address), uint8_t(address >> 8), uint8_t(address >> 16), uint8_t(address >> 24),
                                 uint8_t(words), 0});
            for (uint32_t k = 0; k < words; ++k)
              for (int b = 0; b < 4; ++b) wb.push_back(uint8_t((0x45670123u + k) >> (8 * b)));
            const int before = phy.accesses;
            phy.pending_drop_access = at;   // the drop after `at` more accesses, whatever the op is doing then
            r = call(riscv, TargetRiscvDm::kOpWriteBlock, wb, out);
            if (at < 0) accesses_seen[big] = phy.accesses - before;
            phy.pending_drop_access = -1;
            int stored = 0;
            bool in_order = true;
            for (uint32_t k = 0; k < words; ++k) {
              const int n = phy.stores[address + 4 * k];
              if (n > 1) ++twice;
              if (n == 1 && phy.mem[address + 4 * k] == 0x45670123u + k) {
                if (stored != static_cast<int>(k)) in_order = false;
                ++stored;
              } else if (n) {
                in_order = false;
              }
            }
            bool same = phy.data0 == 0x0000aa55u && phy.data1 == 0x12345678u && phy.abstractauto == 0;
            for (uint32_t k = 0; k < 32; ++k) same = same && phy.gpr[k] == (k ? 0x3c000000u | (k << 16) : 0);
            if (!same || phy.stray_stores) ++clobbered;
            const bool good = ok(r) && out.size() >= 3 && out[2] == kStatusOk;
            const uint16_t done = out.size() >= 2 ? uint16_t(out[0] | out[1] << 8) : 0xffff;
            if (big == 2) {   // never success; fault with the 3 words before it stored once (a drop may stop it sooner)
              if (good || stored > 3) ++unstored_success;
              if (out.size() >= 3 && out[2] == kStatusFault && done == 3) ++faulted;
            }
            if (good) {
              ++succeeded;
              if (stored != static_cast<int>(words) || done != words) ++unstored_success;
            } else if (out.size() >= 3 && (out[2] == kStatusFault || out[2] == kStatusLine)) {
              ++failed_ok;
              if (!in_order || done != stored) ++wrong_done;   // done: the words stored, in order (none twice)
            } else {
              ++other;
            }
            if (getenv("OEP_SHOW_NOT_OK") && !good)
              printf("  write_block %u words, drop at access %d (hold %u, stale %d): status %02x done %u stored %d\n", words,
                     at, hold, stale, out.size() >= 3 ? out[2] : 0xff, done, stored);
            phy.dropped = false;
            g_millis += 2;
          }
    CHECK(accesses_seen[0] > 30 && accesses_seen[1] > accesses_seen[0]);
    CHECK(faulted >= accesses_seen[2]);     // the faulting store: fault, done 3, at least whenever no drop stopped it first
    phy.fault_store = 0;
    CHECK(twice == 0);
    CHECK(unstored_success == 0);
    CHECK(wrong_done == 0);
    CHECK(clobbered == 0);
    CHECK(other == 0);
    CHECK(succeeded + failed_ok == cases);
    printf("  write_block with a drop at each access (%d / %d / %d accesses): %d cases, %d success, %d fault / line "
           "(%d of the faulting store: fault, done 3), %d stored twice, %d success not stored once, %d wrong done, "
           "%d state changed\n", accesses_seen[0], accesses_seen[1], accesses_seen[2], cases, succeeded, failed_ok,
           faulted, twice, unstored_success, wrong_done, clobbered);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    phy.model_block = phy.drop_on_change = phy.drop_loses_writes = phy.drop_stale = phy.dropped = false;
    phy.drop_hold_us = 0;
    phy.abstractauto = phy.cmderr = 0;
    phy.halted = false;
  }

  // ---- a drop anywhere inside a run (bench, 0.0.29-dev+9942787: 2 of about 12 ch32rv uploads to the L103 through the
  // RP2350 answered "run: timeout" with steady() in): the link dropped at every read of the run in turn - stale or all
  // ones, held 0 / 0.7 / 2.1 ms, writes lost. The loader is reached only from dpc = pc with ebreakm set: a dcsr or dpc
  // write lost to the drop and taken as done (a stale ABSTRACTCS read: not busy, no cmderr) sent the hart on through
  // the application to the run's timeout. Every run now stops at its ebreak with the arguments in place, or (a drop
  // that swallowed the resumereq itself) stops where it started for the host to judge - never a timeout ----
  {
    phy.halted = false;
    phy.model_block = phy.drop_on_change = phy.drop_loses_writes = true;
    phy.abstractauto = phy.cmderr = 0;
    phy.loader_pc = 0x20000000u;
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1), out);
    CHECK(ok(r) && phy.halted && fixed.connected);
    const Bytes conn = {uint8_t(fixed.number), uint8_t(fixed.number >> 8)};
    // pc 0x20000000, 1000 ms, a0 = 0x20000400, a1 = 16 and mstatus = 0 in (MIE set before; MPP writable, and fixed at
    // M as on a CH32V003), a0 out
    Bytes run = conn;
    run.insert(run.end(), {0, 0, 0, 0x20, 0xe8, 0x03, 0, 0, 3, 0x0a, 0x10, 0, 0x04, 0, 0x20, 0x0b, 0x10, 16, 0, 0, 0,
                           0x00, 0x03, 0, 0, 0, 0, 1, 0x0a, 0x10});
    int cases = 0, stopped = 0, timeouts = 0, not_started = 0, other = 0, bad_args = 0, run_reads_seen = 0;
    for (int fixed = 0; fixed < 2; ++fixed)
    for (int stale = 0; stale < 2; ++stale)
      for (uint32_t hold : {0u, 700u, 2100u})
        for (int at = -1; at < run_reads_seen; ++at) {
          ++cases;
          phy.halted = true;
          phy.mpp_fixed = fixed;
          phy.mstatus = 0x1888u;
          phy.dpc = 0x00000a3cu;           // where the application was stopped
          phy.dcsr = 0x40000003u;          // ebreakm clear: the application's ebreak traps
          phy.gpr[10] = 0x5a5a0010u;
          phy.gpr[11] = 0x5a5a0011u;
          phy.cmderr = 0;
          phy.abstractauto = 0;
          phy.run_reads = 6;
          phy.dropped = false;
          phy.drop_stale = stale;
          phy.drop_hold_us = hold;
          phy.drop_delay_reads = 0;
          phy.pending_drop = at;
          const int reads_before = phy.reads;
          r = call(riscv, TargetRiscvDm::kOpRun, run, out);
          if (at < 0) run_reads_seen = phy.reads - reads_before;
          phy.pending_drop = -1;
          const bool answered_ok = ok(r) && out.size() >= 15 && out[0] == kStatusOk;
          if (answered_ok && out[1] == reg::target_riscv_dm::kRunStoppedStopped && getU32(out.data() + 2) == 0x20000040u) {
            ++stopped;
            // a0 as the run left it: the argument; mstatus.MIE clear (the fake's run reaches its ebreak only so)
            if (getU32(out.data() + 11) != 0x20000400u || phy.mstatus != (fixed ? 0x1800u : 0u)) ++bad_args;
          } else if (answered_ok && out[1] == reg::target_riscv_dm::kRunStoppedStopped && getU32(out.data() + 2) == 0x00000a3cu) {
            ++not_started;   // the resumereq lost: stopped where it was, dpc unmoved (the host judges, oep-if-debug §4.4)
          } else if (out.size() >= 1 && out[0] == kStatusTimeout) {
            ++timeouts;
            if (timeouts <= 3) printf("  run with a drop at read %d (hold %u us, stale %d): timeout\n", at, hold, stale);
          } else {
            ++other;
          }
          if (!phy.halted) { phy.halted = true; }   // (a timed-out run was halted by the probe)
          phy.dropped = false;
          phy.run_reads = 0;
          g_millis += 2;
        }
    CHECK(run_reads_seen > 20);
    CHECK(timeouts == 0);
    CHECK(bad_args == 0);
    CHECK(stopped + not_started == cases && other == 0);
    printf("  runs with a drop inside (%d reads): %d cases, %d stopped at the ebreak, %d not started, %d timeouts, %d other\n",
           run_reads_seen, cases, stopped, not_started, timeouts, other);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    phy.model_block = phy.drop_on_change = phy.drop_loses_writes = phy.drop_stale = phy.dropped = phy.mpp_fixed = false;
    phy.loader_pc = phy.mstatus = 0;
    phy.drop_hold_us = 0;
    phy.halted = false;
  }

  // ---- a glitch anywhere inside a run and a step (one access missed on its own, the looks around it passing; a write
  // lost - cmderr 6 set or not - or a read answering the read before it), at every access in turn: a run stops at its
  // ebreak with its arguments in place and its outputs read right, or (the resumereq itself lost) where it started;
  // never a timeout, a wrong argument, a wrong dpc or output, or dcsr's ebreaks / ebreaku left changed; a step answers
  // the right dpcs with dcsr.step clear after it ----
  {
    phy.halted = false;
    phy.model_block = true;
    phy.abstractauto = phy.cmderr = 0;
    phy.loader_pc = 0x20000000u;
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1), out);
    CHECK(ok(r) && phy.halted && fixed.connected);
    const Bytes conn = {uint8_t(fixed.number), uint8_t(fixed.number >> 8)};
    Bytes run = conn;   // pc 0x20000000, 1000 ms, a0 = 0x20000400 and a1 = 16 in, a0 out
    run.insert(run.end(), {0, 0, 0, 0x20, 0xe8, 0x03, 0, 0, 2, 0x0a, 0x10, 0, 0x04, 0, 0x20, 0x0b, 0x10, 16, 0, 0, 0,
                           1, 0x0a, 0x10});
    int cases = 0, stopped = 0, not_started = 0, bad = 0, accesses_seen = 0;
    for (int parity = 0; parity < 2; ++parity)
      for (int at = -1; at < accesses_seen; ++at) {
        ++cases;
        phy.halted = true;
        phy.dpc = 0x00000a3cu;
        phy.dcsr = 0x40000003u;
        phy.gpr[10] = 0x5a5a0010u;
        phy.gpr[11] = 0x5a5a0011u;
        phy.cmderr = 0;
        phy.abstractauto = 0;
        phy.run_reads = 6;
        phy.glitch_parity = parity;
        phy.glitch_at = at;
        const int before = phy.accesses;
        r = call(riscv, TargetRiscvDm::kOpRun, run, out);
        if (at < 0) accesses_seen = phy.accesses - before;
        phy.glitch_at = -1;
        const bool answered_ok = ok(r) && out.size() >= 15 && out[0] == kStatusOk;
        if (answered_ok && out[1] == reg::target_riscv_dm::kRunStoppedStopped && getU32(out.data() + 2) == 0x20000040u &&
            getU32(out.data() + 11) == 0x20000400u && phy.gpr[11] == 16 && (phy.dcsr & 0x3000u) == 0) {
          ++stopped;
        } else if (answered_ok && out[1] == reg::target_riscv_dm::kRunStoppedStopped &&
                   getU32(out.data() + 2) == 0x20000000u && phy.gpr[11] == 16) {
          ++not_started;   // the resumereq lost: stopped at pc, dpc unmoved (the host judges, oep-if-debug §4.4)
        } else {
          if (++bad <= 3)
            printf("  run, glitch at access %d (cmderr 6 %d): status %02x dpc %08x a0 %08x a1 %08x dcsr %08x\n", at, parity,
                   out.size() ? out[0] : 0xff, out.size() >= 6 ? getU32(out.data() + 2) : 0,
                   out.size() >= 15 ? getU32(out.data() + 11) : 0, phy.gpr[11], phy.dcsr);
        }
        phy.halted = true;
        phy.run_reads = 0;
        g_millis += 2;
      }
    CHECK(accesses_seen > 40);
    CHECK(bad == 0);
    printf("  runs with a glitch inside (%d accesses): %d cases, %d stopped at the ebreak, %d not started, %d other\n",
           accesses_seen, cases, stopped, not_started, bad);
    // step: one instruction (the fake's resumereq comes back halted at once), dpc moved by the probe's view only
    phy.loader_pc = 0;
    phy.step_returns = true;
    cases = 0;
    bad = 0;
    accesses_seen = 0;
    for (int parity = 0; parity < 2; ++parity)
      for (int at = -1; at < accesses_seen; ++at) {
        ++cases;
        phy.halted = true;
        phy.dpc = 0x00000a3cu;
        phy.dcsr = 0x40000003u;
        phy.cmderr = 0;
        phy.abstractauto = 0;
        phy.glitch_parity = parity;
        phy.glitch_at = at;
        const int before = phy.accesses;
        r = call(riscv, TargetRiscvDm::kOpStep, conn, out);
        if (at < 0) accesses_seen = phy.accesses - before;
        phy.glitch_at = -1;
        const bool good = ok(r) && out.size() == 10 && out[0] == kStatusOk && getU32(out.data() + 2) == 0x00000a3cu &&
                          getU32(out.data() + 6) == 0x00000a3cu && (phy.dcsr & 0x4u) == 0;
        const bool failed_clean = !ok(r) && (phy.dcsr & 0x4u) == 0;   // not done, nothing left set
        if (!good && !failed_clean && ++bad <= 3)
          printf("  step, glitch at access %d (cmderr 6 %d): status %02x before %08x after %08x dcsr %08x\n", at, parity,
                 out.size() ? out[0] : 0xff, out.size() >= 6 ? getU32(out.data() + 2) : 0,
                 out.size() >= 10 ? getU32(out.data() + 6) : 0, phy.dcsr);
        phy.halted = true;
        g_millis += 2;
      }
    phy.step_returns = false;
    CHECK(accesses_seen > 30);
    CHECK(bad == 0);
    printf("  steps with a glitch inside (%d accesses): %d cases, %d wrong\n", accesses_seen, cases, bad);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    phy.model_block = false;
    phy.glitch_parity = false;
    phy.abstractauto = phy.cmderr = 0;
    phy.halted = false;
  }

  // ---- run with a CSR whose WARL bits keep the value it had (bench, 0.0.29-dev+82e1ec9, the V003 jig: every ch32rv
  // upload failed "run: timeout" at the loader's run - stopped 2, could not be halted - where 3c0cd99 passed; the
  // CH32V003's mstatus reads 0x00001800 before and after a write of 0, MPP fixed at M: e7b903c's check took a CSR read
  // back as it was, and not as asked, for a lost write in every try, and the run never started). ch32rv's loader run:
  // a0, a1 and mstatus = 0 in. mstatus as on a V003 (MPP fixed) and as on a part with U mode (writable), MIE set or not
  // before: every run stops at its ebreak with mstatus.MIE clear; with a glitch at every access in turn (cmderr 6 set
  // or not), every run stops at its ebreak with its arguments in place, or (the resumereq lost) where it started ----
  {
    phy.halted = false;
    phy.model_block = true;
    phy.abstractauto = phy.cmderr = 0;
    phy.loader_pc = 0x20000000u;
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(1), out);
    CHECK(ok(r) && phy.halted && fixed.connected);
    const Bytes conn = {uint8_t(fixed.number), uint8_t(fixed.number >> 8)};
    Bytes run = conn;   // pc 0x20000000, 1000 ms, a0 = 0x20000400, a1 = 16 and mstatus = 0 in, a0 out
    run.insert(run.end(), {0, 0, 0, 0x20, 0xe8, 0x03, 0, 0, 3, 0x0a, 0x10, 0, 0x04, 0, 0x20, 0x0b, 0x10, 16, 0, 0, 0,
                           0x00, 0x03, 0, 0, 0, 0, 1, 0x0a, 0x10});
    struct Kind { bool fixed; uint32_t before; };
    const Kind kinds[] = {{true, 0x1800u}, {true, 0x1888u}, {false, 0x1888u}, {false, 0u}};
    int cases = 0, stopped = 0, not_started = 0, bad = 0;
    for (const Kind &kind : kinds) {
      int accesses_seen = 0;
      for (int parity = 0; parity < 2; ++parity)
        for (int at = -1; at < accesses_seen; ++at) {
          if (parity && at < 0) continue;
          ++cases;
          phy.halted = true;
          phy.mpp_fixed = kind.fixed;
          phy.mstatus = kind.before;
          phy.dpc = 0x00000a3cu;
          phy.dcsr = 0x40000003u;
          phy.gpr[10] = 0x5a5a0010u;
          phy.gpr[11] = 0x5a5a0011u;
          phy.cmderr = 0;
          phy.abstractauto = 0;
          phy.run_reads = 6;
          phy.glitch_parity = parity;
          phy.glitch_at = at;
          const int before = phy.accesses;
          r = call(riscv, TargetRiscvDm::kOpRun, run, out);
          if (at < 0) accesses_seen = phy.accesses - before;
          phy.glitch_at = -1;
          const uint32_t mstatus_ok = kind.fixed ? 0x1800u : 0u;
          const bool answered_ok = ok(r) && out.size() >= 15 && out[0] == kStatusOk;
          if (answered_ok && out[1] == reg::target_riscv_dm::kRunStoppedStopped && getU32(out.data() + 2) == 0x20000040u &&
              getU32(out.data() + 11) == 0x20000400u && phy.gpr[11] == 16 && phy.mstatus == mstatus_ok) {
            ++stopped;
          } else if (at >= 0 && answered_ok && out[1] == reg::target_riscv_dm::kRunStoppedStopped &&
                     getU32(out.data() + 2) == 0x20000000u && phy.gpr[11] == 16 && phy.mstatus == mstatus_ok) {
            ++not_started;   // the resumereq lost: stopped at pc, dpc unmoved (the host judges, oep-if-debug §4.4)
          } else if (++bad <= 3) {
            printf("  run with mstatus = 0 (MPP %s, %08x before), glitch at access %d (cmderr 6 %d): status %02x stopped "
                   "%02x dpc %08x mstatus %08x\n", kind.fixed ? "fixed" : "writable", kind.before, at, parity,
                   out.size() ? out[0] : 0xff, out.size() >= 2 ? out[1] : 0xff,
                   out.size() >= 6 ? getU32(out.data() + 2) : 0, phy.mstatus);
          }
          phy.halted = true;
          phy.run_reads = 0;
          g_millis += 2;
        }
      CHECK(accesses_seen > 40);
    }
    CHECK(bad == 0);
    printf("  runs with mstatus = 0 in (MPP fixed / writable), a glitch inside: %d cases, %d stopped at the ebreak, %d not "
           "started, %d other\n", cases, stopped, not_started, bad);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    phy.model_block = phy.mpp_fixed = false;
    phy.glitch_parity = false;
    phy.loader_pc = phy.mstatus = 0;
    phy.abstractauto = phy.cmderr = 0;
    phy.halted = false;
  }

  // ---- the other places that act on one DMI read, with a glitch at every access, singly and in pairs 1-3 apart (a write
  // lost, a read answering the value of the read before it), the read before the request leaving a word that misleads
  // when read stale: one that looks like a halted module's DMSTATUS, with havereset (0x000c0398: version 8,
  // authenticated, allhalted, havereset - a dmseq frame can read so) and without (0x00000398), 0 (the console's idle
  // DATA0) and all ones (the sentinel a host's register read leaves).
  // attach (method 0) joining a live connection with the hart running: the hart left running, no revive (a re-attach:
  // a wake restarts an L103), the DMSTATUS and target_id answered as they are, no restart counted; halt: ok only with
  // the hart halted; resume that never takes: never ok; read_block over a running hart: never ok (it was: the block op
  // ran on the running hart); scan of the live pair: found with the DMSTATUS as it is ----
  {
    const uint32_t kRunning = 2u | (1u << 7) | (3u << 10);   // what the fake's DMSTATUS reads with the hart running
    int cases = 0, bad[5] = {0, 0, 0, 0, 0}, seen[5] = {0, 0, 0, 0, 0};
    const char *what[5] = {"attach", "halt", "resume", "read_block", "scan"};
    for (const uint32_t stale : {0x000c0398u, 0x00000398u, 0u, 0xffffffffu}) {
      for (int op = 0; op < 5; ++op) {
        // every access (the first 300 of resume's wait, which goes on the same way), singly and in pairs 1-3 apart
        for (int k = -1; k < 4 * (seen[op] < 300 ? seen[op] : 300) || k < 0; ++k) {
          const int at = k < 0 ? -1 : k / 4, apart = k < 0 ? 0 : k % 4;
          FakePhy p;
          p.halted = op == 2;
          p.ignore_resume = op == 2;
          p.target_id = 0x2c5a1b03u;
          Ch32Dm d(p);
          DebugPort port{d, 0, 1};
          WireRvswd w(port, 0);
          TargetRiscvDm r(port, 0);
          Bytes o;
          CHECK(ok(call(w, WireRvswd::kOpAttach, attachRequest(op == 2 ? 1 : 0), o)) && port.connected);
          const uint16_t number = port.number;
          const int attaches = p.attaches;
          const uint32_t restarts = d.restarts();
          p.data0 = stale;
          uint32_t x = 0;
          d.readDmi(0x04, x);   // the read before the request: the stale word
          p.glitch_at = at;
          p.glitch_at2 = apart ? at + apart : -1;
          const int before = p.accesses;
          bool good = true;
          if (op == 0) {
            const Result res = call(w, WireRvswd::kOpAttach, attachRequest(0), o);
            size_t len = 0;
            const uint8_t *tid = answerTlv(o, 11, wire::kTlvAttachAnswerTargetId, len);
            good = ok(res) && !p.halted && p.attaches == attaches && d.restarts() == restarts && o.size() >= 11 &&
                   (o[2] | o[3] << 8 | o[4] << 16 | uint32_t(o[5]) << 24) == kRunning && tid && len == 5 &&
                   (tid[1] | tid[2] << 8 | tid[3] << 16 | uint32_t(tid[4]) << 24) == 0x2c5a1b03u;
          } else if (op == 1) {
            const Result res = call(r, TargetRiscvDm::kOpHalt, detachRequest(number), o);
            good = !(ok(res) && o.size() == 1 && o[0] == 0) || p.halted;
          } else if (op == 2) {
            const Result res = call(r, TargetRiscvDm::kOpResume, detachRequest(number), o);
            good = !(ok(res) && o.size() == 1 && o[0] == 0);
          } else if (op == 3) {
            Bytes req = detachRequest(number);
            req.insert(req.end(), {0x00, 0x00, 0x00, 0x20, 4, 0});
            const Result res = call(r, TargetRiscvDm::kOpReadBlock, req, o);
            good = !(ok(res) && o.size() >= 3 && o[2] == 0);
          } else {
            const Result res = call(w, WireRvswd::kOpScan, Bytes{0}, o);
            good = ok(res) && o.size() == 11 && o[1] == 1 &&
                   (o[7] | o[8] << 8 | o[9] << 16 | uint32_t(o[10]) << 24) == kRunning;
          }
          p.glitch_at = p.glitch_at2 = -1;
          releaseConnection(port, 0xff, true);
          if (at < 0) { seen[op] = p.accesses - before; CHECK(good); continue; }
          ++cases;
          if (!good) {
            ++bad[op];
            if (getenv("OEP_SHOW_ONE_READ"))
              printf("  %s, stale %08x, glitch at %d / +%d: wrong (halted %d)\n", what[op], stale, at, apart, p.halted);
          }
        }
      }
    }
    for (int op = 0; op < 5; ++op) CHECK(bad[op] == 0);
    printf("  one-read places with glitches inside (attach %d, halt %d, resume %d, read_block %d, scan %d accesses; "
           "4 stale words; singly and in pairs): %d cases, wrong: attach %d, halt %d, resume %d, read_block %d, scan %d\n", seen[0], seen[1],
           seen[2], seen[3], seen[4], cases, bad[0], bad[1], bad[2], bad[3], bad[4]);
  }

  // ---- the console with a glitch inside (one DMI access missed on its own, the link up again at once: a write lost, a
  // read answering the value of the read before it): SDI, DMDATA and dmseq against targets that play by their rules,
  // a glitch at every access of the run, singly and in pairs 1-3 accesses apart. A word the console acts on is read
  // twice (DmConsole::confirm): with a read missed, every byte arrives once and in order both ways. A write missed:
  // dmseq's sequence numbers take it (exact both ways); SDI / DMDATA carry none - their receipt / answer lost, the next
  // poll takes the same frame again (one frame's bytes twice; DMDATA's answer's input bytes lost) - counted, the
  // mechanism's declared limit ----
  {
    // frames with bytes 1-7 at DATA1's low byte (a DATA1 read missed into DATA0's place looks like an SDI length), the
    // same frame twice in a row, and every length
    const std::vector<Bytes> frames = {{'a'}, {'b', 'c'}, {'d', 'e', 'f'}, {'g', 'h', 'i', 3, 'j'}, {'k', 'l', 'm', 1},
                                       {'n', 'o', 'p', 'q', 'r', 's', 't'}, {'n', 'o', 'p', 'q', 'r', 's', 't'},
                                       {0x85, 0x86}, {'u', 'v', 'w', 7, 7, 7, 7}, {0x81}, {'x', 'y', 'z', 2, 'Z'},
                                       {'1', '2', '3', '4', '5', '6'}, {'!'}};
    Bytes all_out;
    for (const Bytes &f : frames) all_out.insert(all_out.end(), f.begin(), f.end());
    const Bytes input = {'h', 'e', 'l', 'l', 'o', ' ', 'c', 'o', 'n', 's', 'o', 'l', 'e', '\n'};
    constexpr int kPolls = 160;
    struct Outcome { bool out_ok, in_ok; int glitches, read_glitches, write_glitches, accesses; };
    auto runOnce = [&](uint8_t mechanism, int at, int at2) {
      FakePhy p;
      p.halted = false;
      Ch32Dm d(p);
      DmConsole c(d, p);
      Bytes got;
      c.setSink(pushTo, &got);
      CHECK(c.start(mechanism));
      if (mechanism) c.queue(input.data(), input.size());
      SdiTarget sdi{p, frames};
      DmdataTarget dmdata{p, frames};
      SeqSender seq{p, all_out};
      p.glitch_at = at;
      p.glitch_at2 = at2;
      const int before = p.accesses;
      for (int i = 0; i < kPolls; ++i) {
        if (mechanism == 0) sdi.service(); else if (mechanism == 1) dmdata.service(); else seq.service();
        g_millis += 1;
        c.poll();
      }
      const Bytes &rx = mechanism == 1 ? dmdata.rx : seq.rx;
      return Outcome{got == all_out, mechanism == 0 || rx == input, p.glitches, p.glitched_reads, p.glitched_writes,
                     p.accesses - before};
    };
    const char *names[3] = {"SDI", "DMDATA", "dmseq"};
    for (uint8_t mechanism = 0; mechanism < 3; ++mechanism) {
      const Outcome clean = runOnce(mechanism, -1, -1);
      CHECK(clean.out_ok && clean.in_ok);
      int cases = 0, glitched = 0, read_cases = 0, read_bad = 0, write_cases = 0, write_bad = 0, write_out_bad = 0,
          write_in_bad = 0;
      for (int at = 0; at < clean.accesses; ++at) {
        for (int apart = 0; apart <= 3; ++apart) {
          const Outcome o = runOnce(mechanism, at, apart ? at + apart : -1);
          ++cases;
          if (o.glitches) ++glitched;
          const bool good = o.out_ok && o.in_ok;
          if (!o.write_glitches) {
            ++read_cases;
            if (!good) {
              ++read_bad;
              if (getenv("OEP_SHOW_CONSOLE")) printf("  %s: glitch at %d / %d (reads only) wrong\n", names[mechanism], at, apart);
            }
          } else {
            ++write_cases;
            if (!good) ++write_bad;
            if (!o.out_ok) ++write_out_bad;
            if (!o.in_ok) ++write_in_bad;
          }
        }
      }
      CHECK(read_bad == 0);                       // a read missed: every byte once, in order, both ways
      if (mechanism == 2) CHECK(write_bad == 0);  // dmseq: a write missed too
      CHECK(glitched > cases * 9 / 10);           // the glitches met the console (the last ones fall after its run)
      printf("  console %s with glitches inside (%d accesses, singly and in pairs): %d cases, reads only: %d of %d wrong; "
             "a write among them: %d of %d wrong (%d output, %d input)\n", names[mechanism], clean.accesses, cases,
             read_bad, read_cases, write_bad, write_cases, write_out_bad, write_in_bad);
    }
  }

  // ---- the console's DMI accesses per poll: idle (nothing for the console), a short frame, a 7-byte one (6 on dmseq),
  // the DMSTATUS look every kStatusMs ----
  {
    auto cost = [&](uint8_t mechanism, uint32_t data0, uint32_t data1, bool status_due) {
      FakePhy p;
      p.halted = false;
      Ch32Dm d(p);
      DmConsole c(d, p);
      Bytes got;
      c.setSink(pushTo, &got);
      c.start(mechanism);
      g_millis += DmConsole::kStatusMs;
      c.poll();                                         // the DMSTATUS look done, the mailbox empty
      p.data0 = data0;
      p.data1 = data1;
      if (status_due) g_millis += DmConsole::kStatusMs; else g_millis += 1;
      const int before = p.accesses;
      c.poll();
      return p.accesses - before;
    };
    auto seqWord = [](uint8_t n, uint32_t &w1) {
      uint8_t b[8] = {uint8_t(0x80 | 0x08 | n)};
      for (uint8_t i = 0; i < n; ++i) b[1 + i] = uint8_t('a' + i);
      b[1 + n] = DmConsole::crc8(b, 1 + n);
      w1 = b[4] | b[5] << 8 | b[6] << 16 | uint32_t(b[7]) << 24;
      return uint32_t(b[0] | b[1] << 8 | b[2] << 16 | uint32_t(b[3]) << 24);
    };
    uint32_t s1 = 0, s6 = 0;
    const uint32_t seq2 = seqWord(2, s1), seq6 = seqWord(6, s6);
    printf("  console DMI accesses per poll: SDI idle %d, 3 bytes %d, 7 bytes %d; DMDATA idle %d, 3 bytes %d, 7 bytes %d; "
           "dmseq idle %d, 2 bytes %d, 6 bytes %d; the DMSTATUS look (every %u ms) adds %d\n",
           cost(0, 0, 0, false), cost(0, 0x00636203u, 0, false), cost(0, 0x63626107u, 0x67666564u, false),
           cost(1, 0, 0, false), cost(1, 0x636261 << 8 | 0x87u, 0, false), cost(1, 0x63626180u | 11u, 0x67666564u, false),
           cost(2, 0, 0, false), cost(2, seq2, 0, false), cost(2, seq6, s6, false), unsigned(DmConsole::kStatusMs),
           cost(0, 0, 0, true) - cost(0, 0, 0, false));
  }

  printf("wire: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
