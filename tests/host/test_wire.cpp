// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.wire.rvswd / oep.target.riscv-dm over Ch32Dm on a fake DMI PHY with a small debug module behind it.
// - A version-3 (debug spec 1.0) module is worked with like a version-2 one: attach(halt) stops it and says halted,
//   riscv-dm halt answers ok (oep-if-debug §1: found = DMSTATUS.version 2 or 3).
// - The pins go to their free state - Hi-Z with no pull, or the idle the settings give them - whenever nothing holds
//   them: a connection closed (also with idle_clock low, which rests SWCLK driven low), a scan's try, a failed attach
//   (oep-core §8, oep-if-debug §1).
#include <stdio.h>

#include <vector>

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

  bool attach() override {
    if (attached_flag) return true;
    state = kDriven;
    if (!present) { state = kReleased; return false; }
    attached_flag = true;
    return true;
  }
  void release() override { state = kReleased; attached_flag = false; }
  void park() override {
    attached_flag = false;
    state = idle_low ? kDriven : kReleased;   // SWCLK low is driven
  }
  void free() override { state = kFree; attached_flag = false; }
  bool attached() const override { return attached_flag; }
  bool read(uint8_t address, uint32_t &value) override {
    if (!present || !attached_flag) return false;
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

  printf("wire: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
