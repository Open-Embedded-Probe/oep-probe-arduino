// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.wire.swio + oep.target.riscv-dm through the whole stack below the endpoint - WireRvswd -> Ch32Dm ->
// SwioPhy (OEP_HOST_FAKE_SWIO: whole frames) -> a simulated CH32V003 whose reset goes through its bootloader: the hart
// starts in the bootloader, which hands over to the application with a system reset - and a system reset drops the
// SDI configuration (the target answers nothing until the configuration pair is written again) and the debug module
// (dmactive 0, havereset). On the P4 + V003 bench a reset op right after a power-on (BOOT_MODE set) answered
// status line and lost the connection (oep-if-debug §2: a target reset does not close it); a fresh attach worked. And a
// bootloader silent for 400 ms before its hand-over (longer than wire_retry_ms): the reset op waits it out, as an
// attach with the reset TLV does.
#include <stdio.h>

#include <vector>

#include "OepCh32Dm.h"
#include "OepSwioPhy.h"
#include "OepTarget.h"
#include "fake_swio_io.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace wire = reg::wire_rvswd;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                                     \
  do {                                                                                  \
    ++checks;                                                                           \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// The target: SDI configuration, debug module, one hart that runs the bootloader for boot_us after a reset release
// (when boot_mode) and then the application.
struct V003 {
  bool sdi = false;          // the configuration pair written since the last system reset: the target answers
  bool dmactive = false, haltreq = false, ndmreset = false, halted = false, resumeack = false, havereset = false;
  bool boot_mode = false;    // a reset starts the bootloader
  bool in_boot = false;      // running the bootloader
  uint32_t boot_us = 500;    // how long it runs before its system reset into the application
  // From this long into its run until the hand-over, the bootloader answers nothing (as the P4 bench saw a V003 after a
  // power-on: silent from just after the resume for a few hundred ms). ~0u: it answers throughout.
  uint32_t dark_after_us = ~0u;
  uint32_t boot_started = 0;
  uint32_t data0 = 0, data1 = 0, progbuf0 = 0x12345678u, dpc = 0x2f6, dcsr = 0x40000003u;
  int system_resets = 0;
  void tick() {
    if (in_boot && !halted && micros() - boot_started >= boot_us) {   // the bootloader's jump: a system reset
      in_boot = false;
      sdi = dmactive = haltreq = resumeack = false;
      havereset = true;
      dpc = 0x2f6;
      ++system_resets;
    }
  }
  uint32_t pc() const { return in_boot ? 0x1ffff10cu : 0x2f6u; }
  bool dark() const { return in_boot && !halted && dark_after_us != ~0u && micros() - boot_started >= dark_after_us; }
  // The reset line (attach's reset TLV): held, nothing answers; let go, a system reset - the SDI configuration and the
  // module gone - and the core starts at its vector, running (the bootloader's, with boot_mode).
  bool line_held = false;
  void resetLine(bool held) {
    if (held) { line_held = true; sdi = dmactive = haltreq = halted = resumeack = in_boot = false; return; }
    if (!line_held) return;
    line_held = false;
    havereset = true;
    in_boot = boot_mode;
    boot_started = micros();
    ++line_resets;
  }
  int line_resets = 0;
  void releaseReset() {   // ndmreset released: the core starts at its vector (the bootloader's, with boot_mode)
    havereset = true;
    in_boot = boot_mode;
    boot_started = micros();
    if (haltreq) { halted = true; dpc = 0; } else halted = false;
  }
  bool read(uint8_t a, uint32_t &v) {
    advanceMicros(45);
    tick();
    if (!sdi || dark() || line_held) { v = 0xffffffffu; return true; }   // nobody answers: the line stays at its pull-up
    if (!dmactive && a != 0x10 && a != 0x7d && a != 0x7e) { v = 0; return true; }
    switch (a) {
      case 0x04: v = data0; break;
      case 0x05: v = data1; break;
      case 0x10: v = dmactive ? 1u : 0u; break;
      case 0x11:
        v = 2 | (1u << 7) | (halted ? (3u << 8) : (3u << 10)) | (resumeack ? (3u << 16) : 0) | (havereset ? (3u << 18) : 0);
        break;
      case 0x12: v = 0x0002'1000u | 0x380; break;
      case 0x16: v = 2; break;
      case 0x18: v = 0; break;
      case 0x20: v = progbuf0; break;
      case 0x7d: case 0x7e: v = 0x5aa50400u; break;
      default: v = 0; break;
    }
    return true;
  }
  void write(uint8_t a, uint32_t v) {
    advanceMicros(45);
    tick();
    if (dark() || line_held) return;
    if (a == 0x7d && v == 0x5aa50400u) sdi = true;
    if (!sdi) return;
    if (a == 0x10) {
      dmactive = v & 1;
      if (!dmactive) return;
      haltreq = v & (1u << 31);
      if (v & (1u << 28)) havereset = false;
      const bool was = ndmreset;
      ndmreset = v & 2;
      if (ndmreset) { halted = false; in_boot = false; return; }
      if (was) { releaseReset(); return; }
      if (haltreq && !halted) { halted = true; dpc = pc(); resumeack = false; }
      if ((v & (1u << 30)) && halted) {
        halted = false;
        resumeack = true;
        if (in_boot) boot_started = micros();   // the bootloader goes on from where it was stopped
      }
      return;
    }
    if (!dmactive) return;
    if (a == 0x04) data0 = v;
    if (a == 0x05) data1 = v;
    if (a == 0x20) progbuf0 = v;
    if (a == 0x17 && halted && (v & (1u << 17))) {   // an abstract register transfer
      const uint16_t regno = v & 0xffff;
      uint32_t *reg = regno == 0x7b1 ? &dpc : regno == 0x7b0 ? &dcsr : nullptr;
      if (reg) { if (v & (1u << 16)) *reg = data0; else data0 = *reg; }
    }
  }
};
static V003 t;

bool fakeSwioRead(uint8_t address, uint32_t &value) { return t.read(address, value); }
void fakeSwioWrite(uint8_t address, uint32_t value, bool) { t.write(address, value); }
bool fakeSwioLineHigh() { advanceMicros(2000); return true; }

constexpr int kResetPin = 8;
static void onPin(int pin) {
  if (pin == kResetPin) t.resetLine(g_pin_mode[pin] == OUTPUT_OPEN_DRAIN && g_pin_level[pin] == LOW);
}

static Result call(Interface &i, uint8_t op, const Bytes &payload, Bytes &out) {
  out.assign(1024, 0);
  const Result r = i.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }

int main() {
  static SwioPhy phy;
  static Ch32Dm dm(phy);
  static DebugPort port{dm, 19, 0xffff};
  static WireRvswd w(port, 0, "oep.wire.swio");
  static TargetRiscvDm riscv(port, 0);
  phy.begin(19);
  port.reset_allowed = 1ull << kResetPin;
  g_on_pin = onPin;
  Bytes out;
  // max_speed 1 MHz, tag(u8) len(u16) value (core §2.2)
  const Bytes attach = {0, uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 4, 0, 0x40, 0x42, 0x0f, 0x00};

  // ---- a fixed-speed wire: describe declares min_clock_hz = max_clock_hz = the speed of a zero's slot (oep-if-debug
  // §3.2), and the connections say that speed; a max_speed at or above it is taken, one under it refused unsupported.
  // speed_hz was a read's wall time over its 41 slots - 732142 / 745454 Hz on the ESP32-P4 against the 888888 declared ----
  {
    uint8_t d[128];
    const size_t n = w.describe(d, sizeof d);
    auto tagU32 = [&](uint8_t tag) -> uint32_t {
      for (size_t i = 0; i + 3 <= n;) {
        const size_t len = d[i + 1] | (d[i + 2] << 8);
        if (d[i] == tag && len == 4) return getU32(d + i + 3);
        i += 3 + len;
      }
      return 0;
    };
    CHECK(tagU32(kTagMinClockHz) == SwioPhy::kNominalHz && tagU32(kTagMaxClockHz) == SwioPhy::kNominalHz);
    for (const uint32_t hz : {0u, 1000000u, 2000000u, 900000u, SwioPhy::kNominalHz, 800000u, SwioPhy::kNominalHz - 1}) {
      t = V003{};
      Bytes a = {0};
      if (hz) {
        a.insert(a.end(), {uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 4, 0});
        for (int b = 0; b < 4; ++b) a.push_back(uint8_t(hz >> (8 * b)));
      }
      Result r = call(w, WireRvswd::kOpAttach, a, out);
      if (!hz) {   // attach needs max_speed (oep-if-debug §1)
        CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
      } else if (hz < SwioPhy::kNominalHz) {
        CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && !port.connected);
      } else {
        CHECK(ok(r) && out.size() >= 11 && getU32(out.data() + 7) == SwioPhy::kNominalHz);
        const Bytes conn = {uint8_t(port.number), uint8_t(port.number >> 8)};
        r = call(w, WireRvswd::kOpDetach, {conn[0], conn[1], uint8_t(wire::kTlvDetachForce | kTagCritical), 0, 0}, out);
        CHECK(ok(r) && !port.connected);
      }
    }
  }

  // ---- a reset through the bootloader keeps the connection (oep-if-debug §2, §4.3): the bootloader's system reset
  // dropped the SWIO configuration and the module, and every request after it answered line until the connection was
  // lost (0.0.28+ffe4eb1 on the bench); the reads now bring the link back in step. And when the bootloader is silent
  // for longer than wire_retry_ms (400 ms: the P4 bench saw a V003 silent for a few hundred ms from just after the
  // resume - 0.0.28+1c940ca answered the confirmed reset status line at 212 ms, and a mode 0 reset ok with line for the
  // requests after it), the reset op waits it out: it answers once the module answers again, within the host's wait ----
  struct Boot { uint32_t boot_us, dark_after_us; };
  // the hand-over before the confirmation's halt, after the op, and a silent bootloader (from 200 us into its run, or
  // from the resume on: the resume's acknowledgement is not seen)
  const Boot boots[] = {{500, ~0u}, {30000, ~0u}, {400000, 200}, {400000, 0}};
  for (const uint8_t mode : {uint8_t(1), uint8_t(0), uint8_t(2)}) {
    for (const Boot &boot : boots) {
      const bool silent = boot.dark_after_us != ~0u;
      t = V003{};
      Result r = call(w, WireRvswd::kOpAttach, attach, out);
      CHECK(ok(r) && port.connected);
      const Bytes conn = {uint8_t(port.number), uint8_t(port.number >> 8)};
      t.boot_mode = true;
      t.boot_us = boot.boot_us;
      t.dark_after_us = boot.dark_after_us;
      phy.beginRequest();
      const uint32_t began = millis();
      r = call(riscv, TargetRiscvDm::kOpReset, {conn[0], conn[1], mode}, out);
      const uint32_t took = millis() - began;
      CHECK(out.size() == 7);
      CHECK(ok(r) && out[0] == kStatusOk && (out[1] & reg::target_riscv_dm::kResetFlagsReached));
      if (!ok(r) || out.size() < 7 || out[0] != kStatusOk)
        printf("  mode %u boot %u us silent from %d: status %u flags %#x attempts %u, %u ms\n", mode, boot.boot_us,
               silent ? int(boot.dark_after_us) : -1, out.size() > 0 ? out[0] : 0, out.size() > 1 ? out[1] : 0,
               out.size() > 2 ? out[2] : 0, took);
      // the pc confirmed: the application's after a silent bootloader (the op waited for the hand-over), else the
      // bootloader's or the application's - whichever ran at the confirmation's halt
      if (mode == 1)
        CHECK((out[1] & reg::target_riscv_dm::kResetFlagsVerified) &&
              (silent ? getU32(out.data() + 3) == 0x2f6u : getU32(out.data() + 3) != 0));
      // a silent hand-over waited out inside the op, which still answers within the host's wait (core §4.4)
      if (silent && mode != 2) CHECK(took >= 400 && took < v1::reg::kHostWaitAddMs && t.system_resets == 1);
      if (mode == 2) {
        r = call(riscv, TargetRiscvDm::kOpResume, conn, out);
        // silent from the resume on, its acknowledgement is not seen (the hart left debug mode unobserved): line
        CHECK(boot.dark_after_us == 0 ? out.size() == 1 && out[0] == kStatusLine : ok(r));
      }
      // a mode 0 / 1 reset's own hand-over is over by its answer when it was silent; otherwise (or after a host's
      // resume of a reset-halt: the target restarting by itself, oep-if-debug §2) the host's requests may meet it
      if (!silent || mode == 2) g_millis += silent ? 500 : 100;
      // raw DMSTATUS reads on the same connection: answered (a pending havereset shows; it is the host's to see)
      for (int i = 0; i < 3; ++i) {
        r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 1, 0, 0x02, 0x11}, out);
        CHECK(ok(r) && out.size() >= 9 && out[2] == kStatusOk && (out[5] & 0x0f) == 2);
        g_millis += 200;
      }
      CHECK(port.connected);
      // and the high-level ops
      CHECK(ok(call(riscv, TargetRiscvDm::kOpHalt, conn, out)) && t.halted);
      CHECK(ok(call(riscv, TargetRiscvDm::kOpResume, conn, out)) && !t.halted);
      CHECK(t.system_resets == 1);
      r = call(w, WireRvswd::kOpDetach, {conn[0], conn[1], uint8_t(wire::kTlvDetachForce | kTagCritical), 0, 0}, out);
      CHECK(ok(r) && !port.connected);
    }
  }

  // ---- a module that stays silent past the reset's wait: status line, within the host's wait, and no redo (it would
  // start the target into the same hand-over again); the connection is kept (the reset's excuse, oep-if-debug §2) ----
  for (const uint8_t mode : {uint8_t(1), uint8_t(0)}) {
    t = V003{};
    Result r = call(w, WireRvswd::kOpAttach, attach, out);
    CHECK(ok(r) && port.connected);
    const Bytes conn = {uint8_t(port.number), uint8_t(port.number >> 8)};
    t.boot_mode = true;
    t.boot_us = 5000000;
    t.dark_after_us = 200;
    phy.beginRequest();
    const uint32_t began = millis();
    r = call(riscv, TargetRiscvDm::kOpReset, {conn[0], conn[1], mode}, out);
    const uint32_t took = millis() - began;
    CHECK(r.resolution == kResolutionCompleted && r.detail != kOutcomeSuccess && out.size() == 7);
    CHECK(out.size() == 7 && out[0] == kStatusLine && !(out[1] & 3) && out[2] == 1);
    CHECK(took >= Ch32Dm::kResetSettleMs && took < v1::reg::kHostWaitAddMs);
    CHECK(port.connected);
    g_millis += 5000;   // handed over at last
    r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 1, 0, 0x02, 0x11}, out);
    CHECK(ok(r) && out.size() >= 9 && out[2] == kStatusOk && (out[5] & 0x0f) == 2);
    call(w, WireRvswd::kOpDetach, {conn[0], conn[1], uint8_t(wire::kTlvDetachForce | kTagCritical), 0, 0}, out);
  }

  // ---- attach's reset TLV with method 0 (the line pulled and let go, the target left running) on a target whose
  // bootloader is silent for 400 ms before its hand-over: the attach goes on trying until its search's deadline (three
  // tries took a few ms and answered line); a new connection and the live one alike ----
  for (const bool live : {false, true}) {
    t = V003{};
    Result r = call(w, WireRvswd::kOpAttach, attach, out);
    CHECK(ok(r) && port.connected);
    Bytes conn = {uint8_t(port.number), uint8_t(port.number >> 8)};
    if (!live) {
      r = call(w, WireRvswd::kOpDetach, {conn[0], conn[1], uint8_t(wire::kTlvDetachForce | kTagCritical), 0, 0}, out);
      CHECK(ok(r) && !port.connected);
    }
    t.boot_mode = true;
    t.boot_us = 400000;
    t.dark_after_us = 0;
    Bytes with_reset = attach;   // reset(channel u16, hold_ms u16): 10 ms
    with_reset.insert(with_reset.end(), {uint8_t(wire::kTlvAttachReset | kTagCritical), 4, 0, kResetPin, 0, 10, 0});
    const uint32_t began = millis();
    r = call(w, WireRvswd::kOpAttach, with_reset, out);
    const uint32_t took = millis() - began;
    CHECK(ok(r) && port.connected && t.line_resets == 1 && t.system_resets == 1);
    CHECK(took >= 400 && took < v1::reg::kLimitAttachBudgetMs + 10);
    if (!ok(r)) printf("  attach with reset, live %d: %u ms, outcome %u\n", live, took, r.detail);
    conn = {uint8_t(port.number), uint8_t(port.number >> 8)};
    r = call(riscv, TargetRiscvDm::kOpDmi, {conn[0], conn[1], 1, 0, 0x02, 0x11}, out);
    CHECK(ok(r) && out.size() >= 9 && out[2] == kStatusOk && (out[5] & 0x0f) == 2);
    call(w, WireRvswd::kOpDetach, {conn[0], conn[1], uint8_t(wire::kTlvDetachForce | kTagCritical), 0, 0}, out);
  }

  // ---- a link in step pays nothing for a relink: no configuration pair is written while reads answer ----
  {
    t = V003{};
    Result r = call(w, WireRvswd::kOpAttach, attach, out);
    CHECK(ok(r) && port.connected);
    const Bytes conn = {uint8_t(port.number), uint8_t(port.number >> 8)};
    const uint32_t before = phy.retries();
    int pairs = 0;
    for (int i = 0; i < 5; ++i) {
      CHECK(ok(call(riscv, TargetRiscvDm::kOpHalt, conn, out)));
      CHECK(ok(call(riscv, TargetRiscvDm::kOpResume, conn, out)));
    }
    CHECK(phy.retries() == before && t.sdi);
    (void)pairs;
    call(w, WireRvswd::kOpDetach, {conn[0], conn[1], uint8_t(wire::kTlvDetachForce | kTagCritical), 0, 0}, out);
  }

  printf("swio: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
