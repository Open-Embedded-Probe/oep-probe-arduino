// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.fixture.spi-target (oep-spec oep-if-fixture §4) on a fake of the ESP-IDF spi_slave driver - a transfer
// nobody armed counts in transactions and errors, an armed one is queued, a CS frame with no clock counts nothing and
// leaves the arm waiting, one arm at a time, over length counts an error (and over length with the queue full two),
// read_rx in state 0 is unavailable cause 6, configure clears the counts, releasing the plan goes back to describe's
// state. The next transaction is loaded at the CS rising edge that ended the last one, never inside a frame: the bench
// sequence of 0.0.28 (a 0-bit frame before a 64-byte one, loop() running during it), and an unarmed frame right after
// an armed one. arm does not restart the driver. MISO is driven only while CS is low. Modes 1 / 3 are refused unsupported
// (the classic), describe cs_setup_ns 15000.
#include <stdio.h>

#include <vector>

#include "OepP4SpiTarget.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

struct Status {
  uint8_t state, mode, bit_order, armed, queued;
  uint32_t transactions, errors;
};

static Result call(P4SpiTarget &t, uint8_t op, const Bytes &payload, Bytes &out) {
  out.assign(256, 0);
  const Result r = t.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length <= out.size() ? r.length : 0);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static bool unavailableCause(const Result &r, const Bytes &out, uint8_t cause) {
  return r.resolution == kResolutionRejected && r.detail == kRejectUnavailable && out.size() >= 3 && out[0] == 0x01 &&
         out[1] == 1 && out[2] == cause;
}
static Status status(P4SpiTarget &t) {
  Bytes out;
  const Result r = call(t, P4SpiTarget::kOpStatus, {}, out);
  Status s = {};
  if (!ok(r) || out.size() < 13) { CHECK(false); return s; }
  s = {out[0], out[1], out[2], out[3], out[4], getU32(out.data() + 5), getU32(out.data() + 9)};
  return s;
}
static Result arm(P4SpiTarget &t, uint16_t length, const Bytes &tx) {
  Bytes p(4);
  putU16(p.data(), length);
  putU16(p.data() + 2, static_cast<uint16_t>(tx.size()));
  p.insert(p.end(), tx.begin(), tx.end());
  Bytes out;
  return call(t, P4SpiTarget::kOpArm, p, out);
}

// The channel bitmap role_channels declares for `role` in a describe (first channel 0), 0 when absent.
static uint64_t roleMask(const uint8_t *d, size_t n, uint8_t role) {
  for (size_t i = 0; i + 1 < n; i += 2u + d[i + 1]) {
    const size_t len = d[i + 1];
    if (d[i] != kTagRoleChannels || len < 3 || d[i + 2] != role) continue;
    uint64_t m = 0;
    for (size_t k = 0; k < len - 3 && k < 8; ++k) m |= uint64_t{d[i + 5 + k]} << (8 * k);
    return m;
  }
  return 0;
}

int main() {
  static PinTable pins((1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7));
  static P4SpiTarget t(pins);
  const RoleAssignment roles[] = {{0, P4SpiTarget::kRoleSck, 4}, {0, P4SpiTarget::kRoleMosi, 5},
                                  {0, P4SpiTarget::kRoleMiso, 6}, {0, P4SpiTarget::kRoleCs, 7}};
  {   // an input-only channel (the classic ESP32's GPIO34-39): left out of MISO's role_channels only, refused there
    static PinTable in_pins((1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 34));
    in_pins.setInputOnly(1ull << 34);
    static P4SpiTarget t2(in_pins);
    uint8_t d[128];
    const size_t n = t2.describe(d, sizeof d);
    CHECK(roleMask(d, n, P4SpiTarget::kRoleMiso) == 0x70 && roleMask(d, n, P4SpiTarget::kRoleCs) == 0x400000070ull);
    const RoleAssignment miso34[] = {{0, P4SpiTarget::kRoleSck, 4}, {0, P4SpiTarget::kRoleMosi, 5},
                                     {0, P4SpiTarget::kRoleMiso, 34}, {0, P4SpiTarget::kRoleCs, 6}};
    const RoleAssignment cs34[] = {{0, P4SpiTarget::kRoleSck, 4}, {0, P4SpiTarget::kRoleMosi, 5},
                                   {0, P4SpiTarget::kRoleMiso, 6}, {0, P4SpiTarget::kRoleCs, 34}};
    CHECK(t2.planCheck(miso34, 4) == kRejectUnsupported && t2.planCheck(cs34, 4) == 0);
  }
  {   // a channel outside role_channels: unsupported; a declared one something else holds: unavailable (core §8)
    const RoleAssignment undeclared[] = {{0, P4SpiTarget::kRoleSck, 4}, {0, P4SpiTarget::kRoleMosi, 5},
                                         {0, P4SpiTarget::kRoleMiso, 6}, {0, P4SpiTarget::kRoleCs, 8}};
    CHECK(t.planCheck(undeclared, 4) == kRejectUnsupported);
    CHECK(pins.claim(7, 99));
    CHECK(t.planCheck(roles, 4) == kRejectUnavailable);
    pins.release(99);
  }
  CHECK(t.planCheck(roles, 4) == 0);
  CHECK(t.planApply(roles, 4));
  Bytes out;
  // state 0: read_rx is unavailable cause 6, as reset is
  CHECK(unavailableCause(call(t, P4SpiTarget::kOpReadRx, {}, out), out, 6));
  CHECK(unavailableCause(call(t, P4SpiTarget::kOpReset, {}, out), out, 6));
  CHECK(ok(call(t, P4SpiTarget::kOpConfigure, {0, 0}, out)));
  Status s = status(t);
  CHECK(s.state == 1 && !s.armed && s.queued == 0 && s.transactions == 0 && s.errors == 0);

  // not armed: MOSI dropped, MISO 0, counted in transactions and errors
  const uint8_t mosi[] = {0x11, 0x22, 0x33, 0x44};
  CHECK(fakeSpiTransfer(16, mosi));
  CHECK(g_fake_spi.miso == Bytes({0, 0}));
  t.service();
  s = status(t);
  CHECK(!s.armed && s.queued == 0 && s.transactions == 1 && s.errors == 1);
  // CS with no clock, not armed: noise, nothing counted
  CHECK(fakeSpiTransfer(0));
  t.service();
  s = status(t);
  CHECK(s.transactions == 1 && s.errors == 1);

  // armed: a CS frame with no clock leaves the arm waiting; one arm at a time
  CHECK(ok(arm(t, 4, {0xa5, 0x5a})));
  s = status(t);
  CHECK(s.armed && s.queued == 0 && s.transactions == 1 && s.errors == 1);
  CHECK(fakeSpiTransfer(0));
  t.service();
  s = status(t);
  CHECK(s.armed && s.queued == 0 && s.transactions == 1 && s.errors == 1);
  {
    const Result r = arm(t, 2, {});
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnavailable);
  }
  // the armed transfer: MISO from tx (then 0), queued, no error
  CHECK(fakeSpiTransfer(32, mosi));
  CHECK(g_fake_spi.miso == Bytes({0xa5, 0x5a, 0, 0}));
  t.service();
  s = status(t);
  CHECK(!s.armed && s.queued == 1 && s.transactions == 2 && s.errors == 1);
  // and the next one is not armed again
  CHECK(fakeSpiTransfer(8, mosi));
  t.service();
  s = status(t);
  CHECK(!s.armed && s.queued == 1 && s.transactions == 3 && s.errors == 2);
  CHECK(ok(call(t, P4SpiTarget::kOpReadRx, {}, out)));
  CHECK(out.size() == 11 && out[0] == 0 && getU32(out.data() + 1) == 32 && getU16(out.data() + 5) == 4);
  CHECK(Bytes(out.begin() + 7, out.end()) == Bytes(mosi, mosi + 4));

  // over length: the bits that came, data up to length, an error
  CHECK(ok(arm(t, 2, {0x01})));
  CHECK(fakeSpiTransfer(32, mosi));
  t.service();
  s = status(t);
  CHECK(!s.armed && s.queued == 1 && s.transactions == 4 && s.errors == 3);
  CHECK(ok(call(t, P4SpiTarget::kOpReadRx, {}, out)));
  CHECK(out.size() == 9 && getU32(out.data() + 1) == 32 && getU16(out.data() + 5) == 2 && out[7] == 0x11 && out[8] == 0x22);

  // reset clears the counts; unarmed transfers are seen again
  CHECK(ok(call(t, P4SpiTarget::kOpReset, {}, out)));
  s = status(t);
  CHECK(s.state == 1 && !s.armed && s.queued == 0 && s.transactions == 0 && s.errors == 0);
  CHECK(fakeSpiTransfer(8, mosi));
  t.service();
  s = status(t);
  CHECK(s.transactions == 1 && s.errors == 1);

  // over length with the queue full: counted in transactions, not queued, errors + 2
  for (int i = 0; i < 4; ++i) {
    CHECK(ok(arm(t, 4, {})));
    CHECK(fakeSpiTransfer(32, mosi));
    t.service();
  }
  s = status(t);
  CHECK(s.queued == 4 && s.transactions == 5 && s.errors == 1);
  CHECK(ok(arm(t, 2, {})));
  CHECK(fakeSpiTransfer(32, mosi));
  t.service();
  s = status(t);
  CHECK(s.queued == 4 && s.transactions == 6 && s.errors == 3);

  // configure makes the target anew: queue, wait and counts go
  CHECK(ok(arm(t, 4, {})));
  CHECK(ok(call(t, P4SpiTarget::kOpConfigure, {2, 1}, out)));
  s = status(t);
  CHECK(s.state == 1 && s.mode == 2 && s.bit_order == 1 && !s.armed && s.queued == 0 && s.transactions == 0 && s.errors == 0);
  CHECK(fakeSpiTransfer(8, mosi));
  t.service();
  s = status(t);
  CHECK(s.transactions == 1 && s.errors == 1);

  // The bench sequence that failed on 0.0.28 (ArduinoCore-CH32RV tests/bench/trace/periph_probe test_spi_peer, classic
  // ESP32): 4-byte frames over the modes, then configure(0) / arm(64) / a 64-byte frame three times. Each 64-byte frame
  // follows a CS pulse with no clock (a 0-bit frame), and loop() runs 18 bits into it. 0.0.28 loaded the armed
  // transaction again from that loop(): the slave restarted there, the DUT's MISO went wrong from byte 2 and read_rx
  // held only bits from 18 on. Now it is loaded at the pulse's CS rising edge: whole both ways, nothing loaded in a frame.
  {
    const uint8_t p4[] = {0xa5, 0x5a, 0x0f, 0x01};
    const Bytes a4 = {0x3c, 0x96, 0xc3, 0x0f};
    for (uint8_t mode : {0, 2, 0}) {
      CHECK(ok(call(t, P4SpiTarget::kOpConfigure, {mode, 0}, out)));
      CHECK(ok(arm(t, 4, a4)));
      CHECK(fakeSpiTransfer(32, p4));
      CHECK(g_fake_spi.miso == a4);
      t.service();
      CHECK(ok(call(t, P4SpiTarget::kOpReadRx, {}, out)));
      CHECK(out.size() == 11 && getU32(out.data() + 1) == 32 && Bytes(out.begin() + 7, out.end()) == Bytes(p4, p4 + 4));
    }
    Bytes payload(64), answer(64);
    uint32_t x = 7;
    for (size_t i = 0; i < 64; ++i) { x = x * 1103515245u + 12345u; payload[i] = static_cast<uint8_t>(x >> 16); }
    for (size_t i = 0; i < 64; ++i) { x = x * 1103515245u + 12345u; answer[i] = static_cast<uint8_t>(x >> 16); }
    for (int n = 0; n < 3; ++n) {
      CHECK(ok(call(t, P4SpiTarget::kOpConfigure, {0, 0}, out)));
      const int inits = g_fake_spi.inits, in_frame = g_fake_spi.loads_in_frame;
      CHECK(ok(arm(t, 64, answer)));
      CHECK(g_fake_spi.inits == inits);   // arm swaps the discard for the armed one; the driver is not restarted
      CHECK(fakeSpiTransfer(0));          // CS without a clock: the arm keeps waiting
      CHECK(fakeCsLow());
      fakeClock(18, payload.data());
      t.service();                        // loop() during the frame
      fakeClock(64 * 8 - 18, payload.data());
      fakeCsHigh();
      t.service();
      CHECK(g_fake_spi.miso == answer);
      CHECK(g_fake_spi.loads_in_frame == in_frame);
      CHECK(ok(call(t, P4SpiTarget::kOpReadRx, {}, out)));
      CHECK(out.size() == 7 + 64 && out[0] == 0 && getU32(out.data() + 1) == 512 && getU16(out.data() + 5) == 64);
      CHECK(Bytes(out.begin() + 7, out.end()) == payload);
      s = status(t);
      CHECK(!s.armed && s.queued == 0 && s.transactions == 1 && s.errors == 0);
    }
    // An unarmed frame right after the armed one, before loop() runs: the discard is already loaded, so the frame is
    // counted and its MISO is 0 (not the armed frame's MOSI left in the slave's buffer).
    CHECK(ok(call(t, P4SpiTarget::kOpConfigure, {0, 0}, out)));
    CHECK(ok(arm(t, 4, a4)));
    CHECK(fakeSpiTransfer(32, p4));
    CHECK(fakeSpiTransfer(32, p4));
    CHECK(g_fake_spi.miso == Bytes({0, 0, 0, 0}));
    t.service();
    s = status(t);
    CHECK(!s.armed && s.queued == 1 && s.transactions == 2 && s.errors == 1);
  }

  // MISO is driven only while CS is low (the classic's slave drives it from initialize to free; the target gates its
  // output enable from CS): configured, armed, after a CS cycle, armed again, during a frame, and after release.
  {
    t.planRelease();
    CHECK(t.planApply(roles, 4));
    CHECK(!fakeMisoDriven());                               // planned, not configured
    CHECK(ok(call(t, P4SpiTarget::kOpConfigure, {0, 0}, out)));
    CHECK(!fakeMisoDriven());                               // configured, CS high
    CHECK(ok(arm(t, 2, {0x81, 0x42})));
    CHECK(!fakeMisoDriven());                               // armed
    CHECK(fakeCsLow());
    CHECK(fakeMisoDriven());                                // selected
    fakeClock(16);
    CHECK(g_fake_spi.miso == Bytes({0x81, 0x42}));          // driven from the first bit on
    fakeCsHigh();
    CHECK(!fakeMisoDriven());                               // after the CS cycle
    t.service();
    CHECK(ok(arm(t, 2, {0x01})));
    CHECK(!fakeMisoDriven());                               // armed again
    CHECK(fakeSpiTransfer(16));
    CHECK(g_fake_spi.miso == Bytes({0x01, 0x00}));
    CHECK(!fakeMisoDriven());
    CHECK(ok(call(t, P4SpiTarget::kOpReset, {}, out)));
    CHECK(!fakeMisoDriven());                               // reset
    CHECK(ok(call(t, P4SpiTarget::kOpConfigure, {2, 0}, out)));
    CHECK(!fakeMisoDriven());                               // configured anew
    // configured while selected: driven at once
    CHECK(fakeCsLow() || true);
    CHECK(ok(call(t, P4SpiTarget::kOpConfigure, {0, 0}, out)));
    CHECK(fakeMisoDriven());
    fakeCsHigh();
    CHECK(!fakeMisoDriven());
    t.planRelease();
    CHECK(!fakeMisoDriven());                               // released
    CHECK(g_fake_gpio.isr[7] == nullptr && g_fake_gpio.intr[7] == GPIO_INTR_DISABLE);   // the CS handler went
    CHECK(!g_fake_gpio.oe_by_gpio[6]);                      // the pad's output enable handed back
    CHECK(t.planApply(roles, 4));
  }

  // the CS handler runs on core 0, not on loop()'s core 1 that the SWIO frames mask; describe declares cs_setup_ns
  // (tag 0x43, u32): how long MISO may stay undriven after CS falls
  {
    CHECK(g_fake_gpio.service && g_fake_gpio.service_core == 0);   // 0.0.28: installed from loop(), core 1
    // the ISR service installed once, from a task pinned to core 0 - not from the IPC task, where the install's own
    // IPC call waited for ever (0.0.28+ec38b1d: the first configure never answered) - and never again (a second
    // install logs an error, on the classic's UART0)
    CHECK(g_fake_ipc_deadlocks == 0);
    CHECK(g_fake_gpio_error_logs == 0);
    CHECK(g_fake_tasks == 1);
    uint8_t d[64];
    const size_t n = t.describe(d, sizeof d);
    bool found = false;
    for (size_t at = 0; at + 2 <= n; at += 2u + d[at + 1])
      if (d[at] == 0x43 && d[at + 1] == 4)
        found = (d[at + 2] | d[at + 3] << 8 | d[at + 4] << 16 | uint32_t(d[at + 5]) << 24) == P4SpiTarget::kCsSetupNs;
    CHECK(found && P4SpiTarget::kCsSetupNs == 15000);   // the bench's 11.9 us (CS rising to MISO undriven) with margin
  }

  // Modes 1 and 3 are not offered on the classic (its slave drives 0 on MISO before the first SCK edge, whatever the
  // first bit): configure is refused unsupported, payload tag 0x00 (core §4.3 order 6), and the running target and its
  // state are left as they were. Unsupported comes before the plan's unavailable (order 6 before 7); a mode over 3 stays
  // malformed.
  {
    CHECK(ok(call(t, P4SpiTarget::kOpConfigure, {2, 1}, out)));
    CHECK(ok(arm(t, 4, {})));
    for (uint8_t mode : {1, 3}) {
      const Result r = call(t, P4SpiTarget::kOpConfigure, {mode, 0}, out);
      CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out.size() >= 1 && out[0] == 0x00);
    }
    s = status(t);
    CHECK(s.state == 1 && s.mode == 2 && s.bit_order == 1 && s.armed);
    const Result bad = call(t, P4SpiTarget::kOpConfigure, {4, 0}, out);
    CHECK(bad.resolution == kResolutionRejected && bad.detail == kRejectMalformed);
    P4SpiTarget unplanned(pins);
    const Result r = call(unplanned, P4SpiTarget::kOpConfigure, {3, 0}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported);
    CHECK(unavailableCause(call(unplanned, P4SpiTarget::kOpConfigure, {0, 0}, out), out, 6));
  }

  // releasing the plan, then a new one: the state right after describe (state 0, mode and bit_order 0, no counts)
  t.planRelease();
  CHECK(t.planCheck(roles, 4) == 0);
  CHECK(t.planApply(roles, 4));
  s = status(t);
  CHECK(s.state == 0 && s.mode == 0 && s.bit_order == 0 && !s.armed && s.queued == 0 && s.transactions == 0 && s.errors == 0);
  CHECK(unavailableCause(call(t, P4SpiTarget::kOpReadRx, {}, out), out, 6));

  t.planRelease();
  printf("spi-target: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
