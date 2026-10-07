// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: attach / detach on oep.wire.rvswd through the whole stack below the endpoint - WireRvswd -> Ch32Dm ->
// RvswdPhy (OEP_HOST_FAKE_RVSWD) -> a simulated target that decodes the RVSWD frames and keeps the time (as
// test_rvswd_phy.cpp's, plus haltreq / resumereq).
// - Every attach without a live connection is a fresh bring-up: flags bit1 clear, search_retries (TLV 0x12) present; a
//   connection a slot shares outlives the host's detach, and the next attach joins it (flags bit1, no TLV 0x12).
// - The attach budget (oep-if-debug §1, limits.attach_budget_ms) is a hard bound on one attach answer, a reset's hold_ms
//   aside, in the worst cases the simulation can make (writes that never land, reads that fail at the faster periods,
//   a slow max_speed, the reset TLV with halt): 0.0.28 took up to 4.4 s.
// - A max_speed under describe's min_clock_hz (50 kHz) is refused unsupported with the tag as received: under it one
//   attach's minimum checks cannot fit the budget.
#include <stdio.h>

#include <set>
#include <vector>

#include "OepCh32Dm.h"
#include "OepPinTable.h"
#include "OepRvswdPhy.h"
#include "OepTarget.h"
#include "fake_rvswd_io.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace wire = reg::wire_rvswd;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                                          \
  do {                                                                                       \
    ++checks;                                                                                \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); }      \
  } while (0)

struct Target {
  bool begun = false, driven = false, pulled = false;
  uint32_t half_ns = 0;
  uint32_t cell_overhead_ps = 220000;
  uint64_t ps = 0;
  bool clk = true, dio = true, host_drives = true;
  bool awake = false;
  int wake_cells = 0, wakes = 0;
  bool in_frame = false;
  int pos = 0;
  uint8_t address = 0;
  bool write = false, header_parity = false, header_ok = false;
  uint32_t data = 0, out = 0;
  bool data_parity = false, out_parity = false;
  int frames = 0, reads = 0;
  uint32_t dmcontrol = 0, progbuf0 = 0x12345678u, abstractauto = 1, hart_bits = 3u << 10;
  uint32_t min_read_half = 0, min_write_half = 0;
  // A CH32L103's ways (section G): wake_resets - a wake pattern on a link that is up restarts the target itself (the
  // hart runs from its reset vector, havereset set; with haltreq held it stops there again); a drop after a change of
  // hart state - until drop_until_us every read gets nothing back and every write is lost, re-syncs included, and after
  // it the link needs the configuration pair again (out_of_step) before it answers
  bool wake_resets = false, out_of_step = false;
  int target_resets = 0;
  uint64_t drop_until_us = 0;
  uint32_t havereset = 0;
  bool linkUp() const { return awake && !out_of_step && micros() >= drop_until_us; }
  // Section H: the abstract registers a host's read of a register uses - DATA0, COMMAND (access register, 32 bits, on
  // the GPRs), ABSTRACTCS - and a link that comes back from a drop flickering: for flicker_us after the re-sync that
  // brought it back, one transaction in flicker_every misses (a write lost, a read answering the value of the read
  // before it), the looks around it passing
  uint32_t data0 = 0, gpr[32] = {}, dcsr = 0x40000003u, dpc = 0;
  uint32_t flicker_us = 0;
  int flicker_every = 0, flicker_n = 0, flickers = 0;
  uint64_t back_at_us = 0;
  uint32_t last_out = 0;
  bool flicker() {
    if (!flicker_every || !back_at_us || micros() >= back_at_us + flicker_us) return false;
    if (++flicker_n % flicker_every) return false;
    ++flickers;
    return true;
  }
  void tick() {
    ps += uint64_t(half_ns) * 1000u + cell_overhead_ps;
    if (ps >= 1000000) { advanceMicros(static_cast<uint32_t>(ps / 1000000)); ps %= 1000000; }
  }
  uint32_t dmstatus() const { return 0x00400082u | hart_bits | havereset; }
  uint32_t readReg(uint8_t a) {
    switch (a) {
      case 0x10: return dmcontrol;
      case 0x11: return dmstatus();
      case 0x04: return data0;
      case 0x12: return 0x0002'1000u | 0x380;
      case 0x16: return 2;
      case 0x18: return abstractauto;
      case 0x20: return progbuf0;
      case 0x7d: case 0x7e: return 0x5aa50400u;
      case 0x7f: return 0x10350000u;
      default: return 0;
    }
  }
  void writeReg(uint8_t a, uint32_t v) {
    if (half_ns < min_write_half) v ^= 0x00010000u;
    if (a == 0x10) {
      if (v & (1u << 28)) havereset = 0;                          // ackhavereset
      dmcontrol = v;
      if (v & (1u << 31)) hart_bits = 3u << 8;                    // haltreq: halted
      else if (v & (1u << 30)) hart_bits = (3u << 10) | (3u << 16);   // resumereq
    }
    if (a == 0x18) abstractauto = v;
    if (a == 0x20) progbuf0 = v;
    if (a == 0x04) data0 = v;
    if (a == 0x17 && (v & (1u << 17)) && (v & 0xffe0u) == 0x1000u && (hart_bits & (1u << 8))) {   // access a GPR
      if (v & (1u << 16)) gpr[v & 0x1f] = data0;
      else data0 = gpr[v & 0x1f];
    }
    if (a == 0x17 && (v & (1u << 17)) && ((v & 0xffff) == 0x7b0 || (v & 0xffff) == 0x7b1) && (hart_bits & (1u << 8))) {
      uint32_t &csr = (v & 0xffff) == 0x7b0 ? dcsr : dpc;   // dcsr / dpc (the probe reads dpc twice over sentinels)
      if (v & (1u << 16)) csr = data0;
      else data0 = csr;
    }
  }
  bool parityOf(uint32_t v) { bool p = false; while (v) { p ^= v & 1; v >>= 1; } return p; }
  void rise() {
    if (!driven) return;
    if (!in_frame) {
      if (!dio && wake_cells >= 100 && wake_resets && awake) {   // the wake restarts the target (a CH32L103)
        ++target_resets;
        havereset = 3u << 18;
        hart_bits = (dmcontrol & (1u << 31)) ? 3u << 8 : 3u << 10;
      }
      wake_cells = dio ? wake_cells + 1 : (wake_cells >= 100 ? (awake = true, ++wakes, 0) : 0);
      return;
    }
    const bool bit = host_drives ? dio : true;
    if (pos <= 6) { address = static_cast<uint8_t>((address << 1) | dio); header_parity ^= dio; }
    else if (pos == 7) { write = dio; header_parity ^= dio; }
    else if (pos == 8) {
      header_ok = header_parity == dio;
      if (!write) {
        ++reads;
        out = linkUp() && flicker() ? last_out : readReg(address);
        last_out = out;
        out_parity = parityOf(out);
        if (half_ns < min_read_half) out_parity = !out_parity;
      }
    } else if (pos >= 14 && pos <= 45 && write) { data = (data << 1) | bit; data_parity ^= bit; }
    else if (pos == 46 && write && header_ok && data_parity == dio) {
      if (awake && micros() >= drop_until_us && (address == 0x7d || address == 0x7e) && out_of_step) {   // re-sync
        out_of_step = false;
        if (drop_until_us) back_at_us = micros();
      }
      if (linkUp() && !flicker()) writeReg(address, data);
    }
    ++pos;
  }
  void dioChanged(bool from, bool to) {
    if (!driven || !clk || from == to) return;
    if (!to) { in_frame = true; ++frames; pos = 0; address = 0; header_parity = false; data = 0; data_parity = false; wake_cells = 0; }
    else in_frame = false;
  }
  bool targetBit() const {
    if (!linkUp() || !header_ok || write || !in_frame) return pulled;
    if (pos >= 14 && pos <= 45) return (out >> (45 - pos)) & 1;
    if (pos == 46) return out_parity;
    return pulled;
  }
};
static Target t;

void FakeRvswdIo::spin() const { t.tick(); }
void FakeRvswdIo::bothHigh() const {
  if (!t.clk) { t.clk = true; t.rise(); }
  const bool was = t.dio; t.dio = true; t.dioChanged(was, true);
}
void FakeRvswdIo::clkLowDio(bool v) const { t.clk = false; t.dio = v; }
void FakeRvswdIo::clkHigh() const { if (!t.clk) { t.clk = true; t.rise(); } }
void FakeRvswdIo::clk(bool v) const { if (v) clkHigh(); else t.clk = false; }
void FakeRvswdIo::dio(bool v) const { const bool was = t.dio; t.dio = v; t.dioChanged(was, v); }
bool FakeRvswdIo::dioRead() const { return t.host_drives ? t.dio : t.targetBit(); }
void FakeRvswdIo::hostDrives(bool yes) const { t.host_drives = yes; }
bool FakeRvswdIo::begin(int, int) const { t.begun = true; t.pulled = true; t.driven = false; return true; }
void FakeRvswdIo::pullUp(bool on) const { t.pulled = on; }
void FakeRvswdIo::clkPull(int) const {}
void FakeRvswdIo::driveBoth(bool on) const { t.driven = on; }
uint32_t FakeRvswdIo::setHalfNs(uint32_t half_ns) const { t.half_ns = half_ns; return half_ns; }

static Result call(Interface &i, uint8_t op, const Bytes &payload, Bytes &out, size_t capacity = 1024 - 5) {
  out.assign(capacity, 0);
  const Result r = i.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
// what riscv.Wire.attach_body sends: method, max_speed (critical), pins (critical), idle_clock low (critical); every
// TLV tag(u8) len(u16) value (core §2.2)
static Bytes attachRequest(uint8_t method, uint32_t hz, int swdio, int swclk, bool idle_low) {
  Bytes p = {method, uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 4, 0, uint8_t(hz), uint8_t(hz >> 8),
             uint8_t(hz >> 16), uint8_t(hz >> 24)};
  p.insert(p.end(), {uint8_t(wire::kTlvAttachPins | kTagCritical), 4, 0, uint8_t(swdio), 0, uint8_t(swclk), 0});
  if (idle_low) p.insert(p.end(), {uint8_t(wire::kTlvAttachIdleClock | kTagCritical), 1, 0, 1});
  return p;
}
static Bytes detachRequest(uint16_t number) { return {uint8_t(number), uint8_t(number >> 8)}; }
static const uint8_t *answerTlv(const Bytes &out, size_t from, uint8_t tag, size_t &len) {
  for (size_t at = from; at + 3 <= out.size(); at += 3u + (out[at + 1] | out[at + 2] << 8)) {
    const size_t n = out[at + 1] | out[at + 2] << 8;
    if (out[at] == tag && at + 3u + n <= out.size()) { len = n; return out.data() + at + 3; }
  }
  return nullptr;
}

int main() {
  static RvswdPhy phy;
  static Ch32Dm dm(phy);
  static PinTable pins((1ull << 30) - 1);
  static DebugPort port{dm, 0xfffe, 0xfffe};
  static WireRvswd w(port, 0);
  port.pin_choice = (1ull << 30) - 1;
  port.pins = &pins;
  port.reset_allowed = port.pin_choice;
  Bytes out;
  const uint32_t kHz = 1000000;   // max_speed: what the bench's L103 attach asks (any; only the slowest periods qualify)

  // ---- A: no slot, attach / detach x N with a short and a long gap ----
  for (int halt = 0; halt < 2; ++halt) {
    for (uint32_t gap_ms : {50u, 200u, 1000u, 3000u}) {
      uint32_t took_min = ~0u, took_max = 0;
      for (int k = 0; k < 5; ++k) {
        g_millis += gap_ms;
        const uint32_t t0 = micros();
        phy.beginRequest();
        Result r = call(w, WireRvswd::kOpAttach, attachRequest(uint8_t(halt), kHz, 0, 1, true), out);
        const uint32_t took = micros() - t0;
        took_min = took < took_min ? took : took_min;
        took_max = took > took_max ? took : took_max;
        CHECK(ok(r) && out.size() >= 11);
        CHECK(!(out[6] & wire::kAttachFlagsExisting));
        size_t len = 0;
        const uint8_t *v = answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len);
        CHECK(v && len == 2);
        r = call(w, WireRvswd::kOpDetach, detachRequest(port.number), out);
        CHECK(ok(r) && !port.connected);
      }
      printf("  A %s gap %4u ms: attach %u.%03u-%u.%03u ms (sim), search_retries present\n", halt ? "halt" : "run ",
             gap_ms, took_min / 1000, took_min % 1000, took_max / 1000, took_max % 1000);
    }
  }

  // ---- B: a slot shares the connection: the host's detach leaves it up ----
  {
    Result r = call(w, WireRvswd::kOpAttach, attachRequest(0, kHz, 0, 1, true), out);
    CHECK(ok(r) && !(out[6] & wire::kAttachFlagsExisting));
    size_t len = 0;
    CHECK(answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len) != nullptr);
    port.users |= DebugPort::kUserSlot;   // as a slot's automatic attach / a bind's console would have it
    r = call(w, WireRvswd::kOpDetach, detachRequest(port.number), out);
    CHECK(ok(r) && port.connected);       // the host's use goes; the link stays
    g_millis += 50;
    const uint32_t t0 = micros();
    r = call(w, WireRvswd::kOpAttach, attachRequest(0, kHz, 0, 1, true), out);
    const uint32_t took = micros() - t0;
    CHECK(ok(r) && (out[6] & wire::kAttachFlagsExisting));
    CHECK(answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len) == nullptr);
    printf("  B slot-shared: second attach flags 0x%02x (existing), no TLV 0x12, %u.%03u ms (sim)\n", out[6],
           took / 1000, took % 1000);
    r = call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);   // force
    CHECK(ok(r) && !port.connected);
  }

  // ---- C: the answer at a small capacity: where TLV 0x12 would be dropped silently (the fixed part 11 bytes, then
  // target_id 8, dpc 7 and search_retries 5 with their 3-byte headers) ----
  {
    for (size_t cap : {11u, 19u, 26u, 30u, 31u}) {
      Result r = call(w, WireRvswd::kOpAttach, attachRequest(1, kHz, 0, 1, true), out, cap);
      size_t len = 0;
      printf("  C capacity %2zu: %s, %zu bytes, TLV 0x12 %s\n", cap, ok(r) ? "ok" : "not ok", out.size(),
             answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len) ? "present" : "absent");
      if (port.connected) call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);
    }
  }

  // ---- D: the attach budget - how long one attach request takes on the probe, worst cases the sim can make ----
  {
    struct Case { const char *name; uint32_t hz; uint32_t min_read, min_write; uint8_t method; };
    const Case cases[] = {
        {"writes never land, 1 MHz", 1000000, 0, 100000, 0},
        {"writes never land, 20 kHz", 20000, 0, 100000, 0},
        {"clean, 20 kHz", 20000, 0, 0, 1},
        {"clean, 5 kHz", 5000, 0, 0, 1},
        {"fast reads fail, 10 MHz", 10000000, 400, 0, 1},
        {"writes never land, 5 kHz", 5000, 0, 100000, 0},
        {"clean, 10 MHz", 10000000, 0, 0, 0},
        {"clean, 2.5 MHz (200 ns ok)", 2500000, 0, 0, 0},
        {"10 MHz, reads ok >=100ns, writes ok >=500", 10000000, 100, 500, 0},
        {"10 MHz, reads ok >=200ns", 10000000, 200, 0, 0},
        {"clean, 2 kHz", 2000, 0, 0, 1},
        {"clean, 1 kHz", 1000, 0, 0, 1},
        {"writes never land, 10 kHz", 10000, 0, 100000, 1},
        {"clean, 50 kHz", 50000, 0, 0, 1},
        {"writes never land, 50 kHz", 50000, 0, 100000, 1},
    };
    for (const Case &c : cases) {
      t.min_read_half = c.min_read;
      t.min_write_half = c.min_write;
      phy.beginRequest();
      const uint32_t t0 = micros();
      Result r = call(w, WireRvswd::kOpAttach, attachRequest(c.method, c.hz, 0, 1, true), out);
      const uint32_t took = micros() - t0;
      size_t len = 0;
      const uint8_t *v = ok(r) && out.size() > 11 ? answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len) : nullptr;
      printf("  D %-28s %s  %4u.%03u ms (sim)%s%u\n", c.name, ok(r) && out.size() >= 11 && out[0] != 0xff ? "answer" : "status",
             took / 1000, took % 1000, v ? "  search_retries " : "", v ? unsigned(v[0] | v[1] << 8) : 0u);
      if (c.hz < RvswdPhy::kMinClockHz)   // under min_clock_hz: unsupported, the tag as received, nothing run
        CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out.size() == 1 &&
              out[0] == (wire::kTlvAttachMaxSpeed | kTagCritical) && took == 0);
      else
        CHECK(r.resolution == kResolutionCompleted && took <= limits::kAttachBudgetMs * 1000u);
      if (port.connected) call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);
    }
    t.min_read_half = t.min_write_half = 0;
  }

  // ---- E: attach with the reset TLV (channel 5, hold 20 ms), halt and run ----
  for (uint32_t hz : {1000000u, 10000000u, 50000u, 20000u}) {
    for (uint8_t method : {uint8_t(1), uint8_t(0)}) {
      Bytes req = attachRequest(method, hz, 0, 1, true);
      req.insert(req.end(), {uint8_t(wire::kTlvAttachReset | kTagCritical), 4, 0, 5, 0, 20, 0});
      phy.beginRequest();
      const uint32_t t0 = micros();
      Result r = call(w, WireRvswd::kOpAttach, req, out);
      const uint32_t took = micros() - t0;
      printf("  E reset 20 ms, %s, %8u Hz: %s %u.%03u ms (sim)\n", method ? "halt" : "run ", hz,
             ok(r) && out.size() >= 11 ? "answer" : "status/reject", took / 1000, took % 1000);
      if (hz < RvswdPhy::kMinClockHz) CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported);
      else CHECK(ok(r) && took <= (limits::kAttachBudgetMs + 20) * 1000u);   // the budget, the hold aside
      if (port.connected) call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);
    }
  }

  // ---- F: the target's power gone and back under a live connection: attach answers the same connection, its link
  // woken again. On RVSWD a read of the sleeping link fails its parity and the PHY's re-sync wakes it inside the read;
  // on SWIO the line reads all ones with no parity to fail, and 0.0.28+68d9694 answered timeout until a forced detach
  // (P4 + CH32V003 bench) - WireRvswd::attach now brings the link up afresh (test_wire's stale PHY). ----
  for (uint32_t off_ms : {300u, 2000u}) {
    phy.beginRequest();
    Result r = call(w, WireRvswd::kOpAttach, attachRequest(0, kHz, 0, 1, true), out);
    CHECK(ok(r) && port.connected);
    const uint16_t number = port.number;
    t.awake = false;                          // power gone: the line rests on its pull-up
    g_millis += off_ms;
    t.dmcontrol = 0;                          // power back: a fresh module, the hart running, the link asleep
    t.hart_bits = 3u << 10;
    for (uint8_t method : {uint8_t(0), uint8_t(1)}) {
      phy.beginRequest();
      const uint32_t t0 = micros();
      r = call(w, WireRvswd::kOpAttach, attachRequest(method, kHz, 0, 1, true), out);
      const uint32_t took = micros() - t0;
      CHECK(ok(r) && out.size() >= 11 && port.connected && port.number == number && (out[6] & wire::kAttachFlagsExisting));
      CHECK(took <= (limits::kAttachBudgetMs + 20) * 1000u);
      if (method == 1) CHECK(out[6] & wire::kAttachFlagsHalted);
      printf("  F power off %4u ms, attach %s: %s %u.%03u ms (sim)\n", off_ms, method ? "halt" : "run ",
             ok(r) ? "answer" : "status", took / 1000, took % 1000);
    }
    call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);
    CHECK(!port.connected);
  }

  // ---- G: a host's request meeting a CH32L103's drop after the hart stopped (bench, 0.0.29-dev+f594f04, tests/hw
  // test_wire on the L103 through the RP2350: 3 of 8 runs failed "read_register ... failed (cmderr 6)" - the hart the
  // test had halted was no longer halted). The link drops 0.7 - 2.1 ms after a change of hart state, not always at once,
  // so the next request - after a rest, so the PHY revives the link first - can meet it; a re-sync inside the drop leaves
  // it down, and the revive then sent the wake pattern, which restarts this part: the hart left halt under the host.
  // The revive now re-syncs until the drop is over; the target is never restarted, the hart stays halted, the request
  // answers. A link silent for longer than any such drop (the target's power gone and back) still gets the wakes. ----
  {
    static TargetRiscvDm riscv(port, 0);
    t.wake_resets = true;
    for (uint32_t hold_us : {700u, 1400u, 2100u, 5000u}) {
      phy.beginRequest();
      Result r = call(w, WireRvswd::kOpAttach, attachRequest(1, kHz, 0, 1, true), out);   // halt, idle_clock low
      CHECK(ok(r) && port.connected && t.hart_bits == 3u << 8);
      const Bytes conn = {uint8_t(port.number), uint8_t(port.number >> 8)};
      const int resets = t.target_resets, wakes = t.wakes;
      advanceMicros(400);                    // the host's next request, past the PHY's rest
      t.drop_until_us = micros() + hold_us;  // and the drop met by it
      t.out_of_step = true;
      // dmi: DMSTATUS, DMCONTROL, ABSTRACTCS read (a raw look, as RiscvDm.held starts one)
      phy.beginRequest();
      const uint32_t t0 = micros();
      r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 3, 0, 0x02, 0x11, 0x02, 0x10, 0x02, 0x16}, out);
      const uint32_t took = micros() - t0;
      const bool answered = ok(r) && out.size() >= 5 + 12 && out[2] == kStatusOk;
      const uint32_t status = answered ? getU32(out.data() + 5) : 0;
      CHECK(answered && (status & (3u << 8)) == (3u << 8) && !(status & (3u << 18)));   // halted, no havereset
      CHECK(t.target_resets == resets && t.wakes == wakes);   // no wake, the target not restarted
      CHECK(t.hart_bits == 3u << 8);
      printf("  G drop %4u us met by a request: %s, DMSTATUS %08x, %d wakes, %d target restarts, %u.%03u ms (sim)\n",
             hold_us, answered ? "answer" : "status", status, t.wakes - wakes, t.target_resets - resets, took / 1000,
             took % 1000);
      call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);
      CHECK(!port.connected);
    }
    // a link only a wake brings back (the target's power gone and back, its link asleep): on the connection, outside
    // attach and reset, no wake goes out (oep-if-debug §2: the probe's resyncs do not change the target) - the request
    // answers line; an attach on the connection may wake it (its revive) and brings it back
    phy.beginRequest();
    Result r = call(w, WireRvswd::kOpAttach, attachRequest(0, kHz, 0, 1, true), out);
    CHECK(ok(r) && port.connected);
    const Bytes conn = {uint8_t(port.number), uint8_t(port.number >> 8)};
    const int wakes = t.wakes;
    advanceMicros(400);
    t.awake = false;
    phy.beginRequest();
    r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 1, 0, 0x02, 0x11}, out);
    CHECK(!ok(r) && out.size() >= 3 && out[2] == kStatusLine && t.wakes == wakes);
    phy.beginRequest();
    r = call(w, WireRvswd::kOpAttach, attachRequest(0, kHz, 0, 1, true), out);
    CHECK(ok(r) && port.connected && t.wakes > wakes);
    printf("  G link asleep: line with no wake, back after %d wakes in an attach\n", t.wakes - wakes);
    call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);
    t.wake_resets = false;
    t.drop_until_us = 0;
    t.out_of_step = false;
    t.havereset = 0;
  }

  // ---- H: a host's read of a register (oep-client-python RiscvDm.read_register: look, clear cmderr, the access-
  // register command, wait for it, DATA0, look - one dmi request, tried again 5 ms later when it is not seen held, 4 tries)
  // meeting a CH32L103's drop after a change of hart state that comes back flickering: for a while after the re-sync that
  // brings it back, one transaction in a few misses (a write lost, a read answering the value of the read before it).
  // Bench (0.0.29-dev+bd19b00, tests/hw test_wire on the L103 through the RP2350): 5 of 8 runs read s1 / a0 wrong after
  // a read_block with the request's looks passing (a0 read as s1's value: the command lost, DATA0 still the read before).
  // The revive handed the link on at the first DMSTATUS that answered and the request ran in the flicker; now the revive
  // hands it on after kReviveLooks good looks in a row, and the dmi request answers line when the look after its steps
  // fails or the PHY revived the link between its looks (P4). Counted: values the host would take (status ok, its looks
  // passing) that are not the register's - the host's group as it is, and as a group that reads the register twice
  // over sentinels written to DATA0 (0, then all ones) and takes it only when both agree. ----
  {
    static TargetRiscvDm riscv(port, 0);
    phy.beginRequest();
    Result r = call(w, WireRvswd::kOpAttach, attachRequest(1, kHz, 0, 1, true), out);   // halt, idle_clock low
    CHECK(ok(r) && port.connected && t.hart_bits == 3u << 8);
    const Bytes conn = {uint8_t(port.number), uint8_t(port.number >> 8)};
    auto rd = [](Bytes &b, uint8_t a) { b.insert(b.end(), {0x02, a}); };
    auto wr = [](Bytes &b, uint8_t a, uint32_t v) {
      b.insert(b.end(), {0x01, a, uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)});
    };
    auto poll = [](Bytes &b) { b.insert(b.end(), {0x03, 0x16, 0, 0x10, 0, 0, 0, 0, 0, 0, 100, 0}); };   // busy clear
    auto linkHeldLook = [](uint32_t status, uint32_t control) {   // oep_client.riscv.link_held
      const uint32_t v = status & 0xf;
      return v >= 2 && v != 15 && (status & 0x80) && (control & 1) && !(control & 0x07ffffc0u);
    };
    const uint32_t kCmd = 0x00221009u;   // read s1
    for (int verified = 0; verified < 2; ++verified) {
      int cases = 0, taken = 0, wrong = 0, not_held = 0, flickers = 0;
      for (int every : {2, 3, 4, 5, 6, 8})
        for (uint32_t flicker_us : {500u, 1000u, 2000u})
          for (uint32_t hold_us : {700u, 1400u, 2100u}) {
            ++cases;
            const uint32_t value = 0x00002da8u + static_cast<uint32_t>(cases);
            t.gpr[9] = value;
            t.data0 = 0x00001000u;   // the read before (s0's)
            t.flicker_every = every;
            t.flicker_us = flicker_us;
            t.flicker_n = 0;
            t.back_at_us = 0;
            const int flickers_before = t.flickers;
            advanceMicros(400);                     // the host's next request, past the PHY's rest
            t.drop_until_us = micros() + hold_us;   // the drop it meets
            t.out_of_step = true;
            bool done = false;
            for (int attempt = 0; attempt < 4 && !done; ++attempt) {
              if (attempt) advanceMicros(5000);     // HELD_RETRY_S
              Bytes q = conn;
              const uint8_t n = verified ? 13 : 8;
              q.insert(q.end(), {n, 0});
              rd(q, 0x11); rd(q, 0x10);
              wr(q, 0x16, 0x700);
              if (verified) wr(q, 0x04, 0);
              wr(q, 0x17, kCmd); poll(q); rd(q, 0x04);
              if (verified) { wr(q, 0x04, 0xffffffffu); wr(q, 0x17, kCmd); poll(q); rd(q, 0x04); }
              rd(q, 0x11); rd(q, 0x10);
              phy.beginRequest();
              r = call(riscv, TargetRiscvDm::kOpDmi, q, out);
              const size_t nv = verified ? 8 : 6;
              if (!ok(r) || out.size() < 5 + 4 * nv || out[2] != kStatusOk) continue;
              auto v = [&](size_t i) { return getU32(out.data() + 5 + 4 * i); };
              if (!linkHeldLook(v(0), v(1)) || !linkHeldLook(v(nv - 2), v(nv - 1))) continue;
              if (verified && v(3) != v(5)) continue;
              done = true;
              ++taken;
              if (v(3) != value) ++wrong;
            }
            if (!done) ++not_held;
            flickers += t.flickers - flickers_before;
          }
      CHECK(wrong == 0);
      printf("  H a register read meeting a drop that comes back flickering (%s): %d cases, %d taken, %d wrong, "
             "%d not held after 4 tries, %d flickers\n", verified ? "read twice over sentinels" : "read once",
             cases, taken, wrong, not_held, flickers);
    }
    t.flicker_every = 0;
    // A revive between a request's own looks: a wait step longer than the PHY's rest, the link dropping before it (the
    // command written then is lost) and brought back by the revive at the next step - the look after the steps passes
    // and DATA0 is the read before. bd19b00 answered it ok with that value; now status line, done = all the steps, no
    // values (P4).
    {
      static bool armed = false;
      armed = false;
      g_on_wait = [] {
        if (!armed) return;
        armed = false;
        t.drop_until_us = micros() + 300;
        t.out_of_step = true;
      };
      t.gpr[9] = 0x0000eab0u;
      t.data0 = 0x00001000u;
      phy.beginRequest();
      r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 1, 0, 0x02, 0x11}, out);   // the link in use
      CHECK(ok(r));
      const uint32_t revives = phy.revives();
      Bytes q = conn;
      q.insert(q.end(), {10, 0});
      rd(q, 0x11); rd(q, 0x10);
      wr(q, 0x16, 0x700);
      q.insert(q.end(), {0x04, 100, 0, 0, 0});      // 100 us: the drop starts at its end
      wr(q, 0x17, kCmd);                            // lost
      q.insert(q.end(), {0x04, 0x90, 0x01, 0, 0});  // 400 us: past the PHY's rest
      poll(q); rd(q, 0x04);                         // the revive before the poll brings the link back
      rd(q, 0x11); rd(q, 0x10);
      armed = true;
      phy.beginRequest();
      r = call(riscv, TargetRiscvDm::kOpDmi, q, out);
      g_on_wait = nullptr;
      const bool line = out.size() == 5 && getU16(out.data()) == 10 && out[2] == kStatusLine && getU16(out.data() + 3) == 0;
      CHECK(phy.revives() == revives + 1 && t.gpr[9] == 0x0000eab0u);
      CHECK(line);
      printf("  H a revive between a request's looks (the command lost before it): %s, %u revive\n",
             line ? "status line, done 10, no values" : ok(r) ? "answered ok" : "other", phy.revives() - revives);
    }
    t.drop_until_us = 0;
    t.out_of_step = false;
    call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);
  }

  printf("attach-cycle: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
