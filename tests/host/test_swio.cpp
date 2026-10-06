// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.wire.swio + oep.target.riscv-dm through the whole stack below the endpoint - WireRvswd -> Ch32Dm ->
// SwioPhy (OEP_HOST_FAKE_SWIO: whole frames) -> a simulated CH32V003 whose reset goes through its bootloader: the hart
// starts in the bootloader, which hands over to the application with a system reset - and a system reset drops the
// SDI configuration (the target answers nothing until the configuration pair is written again) and the debug module
// (dmactive 0, havereset). On the P4 + V003 bench a reset op right after a power-on (BOOT_MODE set) answered
// status line and lost the connection (oep-if-debug §2: a target reset does not close it); a fresh attach worked.
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
  void releaseReset() {   // ndmreset released: the core starts at its vector (the bootloader's, with boot_mode)
    havereset = true;
    in_boot = boot_mode;
    boot_started = micros();
    if (haltreq) { halted = true; dpc = 0; } else halted = false;
  }
  bool read(uint8_t a, uint32_t &v) {
    advanceMicros(45);
    tick();
    if (!sdi) { v = 0xffffffffu; return true; }   // nobody answers: the line stays at its pull-up
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
  Bytes out;
  const Bytes attach = {0, uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 4, 0x40, 0x42, 0x0f, 0x00};   // 1 MHz

  // ---- a reset through the bootloader keeps the connection (oep-if-debug §2, §4.3): the bootloader's system reset
  // dropped the SWIO configuration and the module, and every request after it answered line until the connection was
  // lost (0.0.28+ffe4eb1 on the bench); the reads now bring the link back in step ----
  for (const uint8_t mode : {uint8_t(1), uint8_t(0), uint8_t(2)}) {
    for (const uint32_t boot_us : {500u, 30000u}) {   // the hand-over before the confirmation's halt, or after the op
      t = V003{};
      Result r = call(w, WireRvswd::kOpAttach, attach, out);
      CHECK(ok(r) && port.connected);
      const Bytes conn = {uint8_t(port.number), uint8_t(port.number >> 8)};
      t.boot_mode = true;
      t.boot_us = boot_us;
      phy.beginRequest();
      r = call(riscv, TargetRiscvDm::kOpReset, {conn[0], conn[1], mode}, out);
      CHECK(out.size() == 7);
      if (mode == 2) {
        CHECK(ok(r) && out[0] == kStatusOk && (out[1] & reg::target_riscv_dm::kResetFlagsReached));
        CHECK(ok(call(riscv, TargetRiscvDm::kOpResume, conn, out)));
      } else {
        CHECK(ok(r) && out[0] == kStatusOk && (out[1] & reg::target_riscv_dm::kResetFlagsReached));
        if (mode == 1) CHECK((out[1] & reg::target_riscv_dm::kResetFlagsVerified) && out.size() == 7);
        if (!ok(r)) printf("  mode %u boot %u us: status %u flags %#x attempts %u\n", mode, boot_us, out[0], out[1], out[2]);
      }
      g_millis += 100;   // the bootloader has handed over by now
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
      r = call(w, WireRvswd::kOpDetach, {conn[0], conn[1], uint8_t(wire::kTlvDetachForce | kTagCritical), 0}, out);
      CHECK(ok(r) && !port.connected);
    }
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
    call(w, WireRvswd::kOpDetach, {conn[0], conn[1], uint8_t(wire::kTlvDetachForce | kTagCritical), 0}, out);
  }

  printf("swio: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
