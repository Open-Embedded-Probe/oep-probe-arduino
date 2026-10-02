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
//   a reset's hold and the wire_lost_ms after it not counted.
#include <stdio.h>

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

  bool attach() override {
    if (attached_flag) return true;
    ++attaches;
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
  bool readWire(uint8_t address, uint32_t &value) override {
    if (!present || !attached_flag) return false;
    if (flaky) {   // the RVSWD PHY's way: retries while the request's allowance lasts, then a failed read
      while (retryLeft()) { g_millis += 1; spentRetrying(1000); ++retry_reads; }
      return false;
    }
    switch (address) {
      case 0x04: value = data0; break;
      case 0x05: value = data1; break;
      case 0x10: value = 1; break;   // DMCONTROL: dmactive
      case 0x11:                     // DMSTATUS: version, authenticated, all/any halted or running, allresumeack
        value = version | (1u << 7) | (halted ? (3u << 8) : (3u << 10)) | (resumeack ? (3u << 16) : 0);
        break;
      case 0x12: value = 0x0002'1000u | 0x380; break;   // HARTINFO: DATA0 at 0x380 (memory-mapped), datacount 2
      case 0x16: value = 2; break;                       // ABSTRACTCS: datacount 2, not busy, no cmderr
      default: value = 0; break;
    }
    return true;
  }
  void write(uint8_t address, uint32_t value) override {
    ++writes;
    if (!present || !attached_flag) return;
    if (address == 0x04) data0 = value;
    if (address == 0x05) data1 = value;
    if (address == 0x10) {
      if (value & (1u << 31)) { halted = true; resumeack = false; }
      if (value & (1u << 30)) { halted = false; resumeack = true; }
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

static Result call(Interface &i, uint8_t op, const Bytes &payload, Bytes &out) {
  out.assign(256, 0);
  const Result r = i.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
// attach: method, max_speed (critical), [pins (critical)], [idle_clock (critical)]
static Bytes attachRequest(uint8_t method, int swdio = -1, int swclk = -1, bool idle_low = false) {
  Bytes p = {method, uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 4, 0x40, 0x42, 0x0f, 0x00};
  if (swdio >= 0) {
    p.insert(p.end(), {uint8_t(wire::kTlvAttachPins | kTagCritical), 4, uint8_t(swdio), 0, uint8_t(swclk), 0});
  }
  if (idle_low) p.insert(p.end(), {uint8_t(wire::kTlvAttachIdleClock | kTagCritical), 1, 1});
  return p;
}
static Bytes detachRequest(uint16_t number) { return {uint8_t(number), uint8_t(number >> 8)}; }
// The value of answer TLV `tag` after `from` (core §2.3), or nullptr.
static const uint8_t *answerTlv(const Bytes &out, size_t from, uint8_t tag, size_t &len) {
  for (size_t at = from; at + 2 <= out.size(); at += 2u + out[at + 1])
    if (out[at] == tag && at + 2u + out[at + 1] <= out.size()) { len = out[at + 1]; return out.data() + at + 2; }
  return nullptr;
}

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
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(0, -1, -1, true), out);
    CHECK(ok(r) && fixed.connected);
    r = call(wire_fixed, WireRvswd::kOpDetach, detachRequest(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
    CHECK(phy.state == FakePhy::kFree);   // 0.0.28: park() left SWCLK driven low
    phy.idle_low = false;
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
    // attach on 4 / 5, then detach: 4 pulled up, 5 driven low by its idle, the PHY's own drive gone
    Result r = call(wire_chosen, WireRvswd::kOpAttach, attachRequest(0, 4, 5, true), out);
    CHECK(ok(r) && chosen.connected && pins.owner(4) == chosen.pin_owner);
    g_pin_mode[4] = g_pin_mode[5] = -1;
    r = call(wire_chosen, WireRvswd::kOpDetach, detachRequest(chosen.number), out);
    CHECK(ok(r) && !chosen.connected && pins.free(4) && pins.free(5));
    CHECK(phy2.state == FakePhy::kFree);
    CHECK(g_pin_mode[4] == INPUT_PULLUP);
    CHECK(g_pin_mode[5] == OUTPUT && g_pin_level[5] == LOW);
    // a scan of 4 / 5 (found) and 6 / 7 (nothing there): each tried pair back to its free state
    g_pin_mode[4] = g_pin_mode[5] = -1;
    const Bytes scan_found = {1, 4, 0, 5, 0};
    r = call(wire_chosen, WireRvswd::kOpScan, scan_found, out);
    CHECK(ok(r) && out.size() >= 2 && out[0] == 1 && out[1] == 1);
    CHECK(phy2.state == FakePhy::kFree && !chosen.connected);
    CHECK(g_pin_mode[4] == INPUT_PULLUP && g_pin_mode[5] == OUTPUT && g_pin_level[5] == LOW);
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
    Bytes bad = {0, uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 4, 0x40, 0x42, 0x0f, 0x00,
                 uint8_t(wire::kTlvAttachPins | kTagCritical), 4, 9, 0, 5, 0};
    r = call(wire_chosen, WireRvswd::kOpAttach, bad, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out.size() >= 1 &&
          out[0] == (wire::kTlvAttachPins | kTagCritical));
    bad[7] = wire::kTlvAttachPins;   // not critical: ignored; no live connection and the host must name a pair
    r = call(wire_chosen, WireRvswd::kOpAttach, bad, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnavailable);
  }

  // ---- scan: the bring-up, no write check, nothing written through the link; attach(halt): DMSTATUS after the halt ----
  {
    phy.version = 2;
    phy.halted = false;
    phy.writes = phy.attaches = phy.bring_ups = 0;
    const Bytes scan_fixed = {0};
    Result r = call(wire_fixed, WireRvswd::kOpScan, scan_fixed, out);
    CHECK(ok(r) && out.size() >= 12 && out[0] == 1 && out[1] == 1);
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

  // ---- an undefined attach method (2+): unsupported, payload 0x00 (C-02) ----
  {
    Result r = call(wire_fixed, WireRvswd::kOpAttach, attachRequest(2), out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out.size() == 1 && out[0] == 0);
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

  printf("wire: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
