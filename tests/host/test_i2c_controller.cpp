// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.fixture.i2c-controller (oep-spec oep-if-fixture §5) on a fake bus (OEP_HOST_FAKE_I2C_BUS: a target
// at 0x50 with registers, answering the controller's bytes) - the plan's roles, configure's clock and refusals, a
// register read joined by a repeated START, the bus held without a STOP and the next transfer's repeated START, the
// address NACKed, a data byte NACKed and its place, a stretch timeout and a bus error ending the transfer, the request's
// refusals (form, reserved flags, max_length, state 0), status and the counters, a held bus at the session's end.
#include <stdio.h>

#include <string>
#include <vector>

#include "OepI2cController.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace i2c = reg::fixture_i2c_controller;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// The bus as the target sees it: S / Sr / P, each byte (W:xx a or n, R:xx), and a target at 0x50 whose register
// pointer is the first byte written; it NACKs a write past register 0x0f. `stretch` / `bus_error`: the next byte's.
struct FakeBus final : I2cHostBus {
  std::string log;
  uint8_t regs[16] = {};
  uint8_t pointer = 0;
  int address = -1;   // the addressed target's address, -1 none
  bool reading = false, first = false;
  bool stretch = false, bus_error = false;
  void add(const char *s) { if (!log.empty()) log += ' '; log += s; }
  uint8_t start(bool repeated) override { add(repeated ? "Sr" : "S"); address = -1; return I2cController::kLineOk; }
  uint8_t write(uint8_t byte, bool &ack) override {
    if (stretch) { stretch = false; add("stretch"); return I2cController::kLineStretch; }
    if (bus_error) { bus_error = false; add("lost"); return I2cController::kLineBusError; }
    char s[16];
    if (address < 0) {   // the address byte
      ack = byte >> 1 == 0x50;
      if (ack) { address = byte >> 1; reading = byte & 1; first = true; }
    } else {
      if (first) { pointer = byte; ack = true; first = false; }
      else { ack = pointer < 16; if (ack) regs[pointer++] = byte; }
    }
    snprintf(s, sizeof s, "W:%02x%c", byte, ack ? 'a' : 'n');
    add(s);
    return I2cController::kLineOk;
  }
  uint8_t read(uint8_t &byte, bool ack) override {
    byte = pointer < 16 ? regs[pointer++] : 0xff;
    char s[16];
    snprintf(s, sizeof s, "R:%02x%c", byte, ack ? 'a' : 'n');
    add(s);
    return I2cController::kLineOk;
  }
  void stop() override { add("P"); address = -1; }
  void release() override { add("let-go"); address = -1; }
};

static Result call(I2cController &c, uint8_t op, const Bytes &payload, Bytes &out) {
  out.assign(512, 0);
  const Result r = c.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static bool rejectedAs(const Result &r, uint8_t reason) { return r.resolution == kResolutionRejected && r.detail == reason; }
static bool unavailableCause(const Result &r, const Bytes &out, uint8_t cause) {
  return rejectedAs(r, kRejectUnavailable) && out.size() >= 4 && out[0] == 0x01 && out[3] == cause;
}
static Bytes configureBytes(uint32_t clock, uint32_t timeout) {
  Bytes p(8);
  putU32(p.data(), clock);
  putU32(p.data() + 4, timeout);
  return p;
}
// a segment: dir address count [data]
static void segment(Bytes &p, uint8_t dir, uint8_t address, uint16_t count, const Bytes &data = {}) {
  p.push_back(dir);
  p.push_back(address);
  p.push_back(count & 0xff);
  p.push_back(count >> 8);
  p.insert(p.end(), data.begin(), data.end());
}
struct Answer {
  uint8_t result, at;
  uint16_t index;
  Bytes data;
};
static Answer answerOf(const Bytes &out) {
  Answer a = {};
  if (out.size() < 6) { CHECK(false); return a; }
  a = {out[0], out[1], getU16(out.data() + 2), Bytes(out.begin() + 6, out.end())};
  CHECK(getU16(out.data() + 4) == a.data.size());
  return a;
}

int main() {
  static PinTable pins((1ull << 4) | (1ull << 5));
  static I2cController c(pins);
  static FakeBus bus;
  I2cController::host_bus = &bus;
  Bytes out;

  // the plan: each role once, on distinct channels
  {
    const RoleAssignment twice[] = {{0, I2cController::kRoleSda, 4}, {0, I2cController::kRoleSda, 5}};
    const RoleAssignment same[] = {{0, I2cController::kRoleSda, 4}, {0, I2cController::kRoleScl, 4}};
    CHECK(c.planCheck(twice, 2) == kRejectUnsupported);
    CHECK(c.planCheck(same, 2) == kRejectUnsupported);
    CHECK(c.planCheck(twice, 1) == kRejectUnsupported);
    CHECK(pins.claim(5, 99));
    const RoleAssignment held[] = {{0, I2cController::kRoleSda, 4}, {0, I2cController::kRoleScl, 5}};
    CHECK(c.planCheck(held, 2) == kRejectUnavailable);
    pins.release(99);
  }
  // no plan: configure is unavailable cause 6; neither is a transfer taken
  CHECK(unavailableCause(call(c, I2cController::kOpConfigure, configureBytes(100000, 1000), out), out, 6));
  const RoleAssignment roles[] = {{0, I2cController::kRoleSda, 4}, {0, I2cController::kRoleScl, 5}};
  CHECK(c.planCheck(roles, 2) == 0);
  CHECK(c.planApply(roles, 2));
  {
    Bytes p = {i2c::kTransferFlagsStop, 1};
    segment(p, i2c::kDirWrite, 0x50, 0);
    CHECK(unavailableCause(call(c, I2cController::kOpTransfer, p, out), out, 6));   // state 0
  }

  // configure: 0 is malformed; past the limits unsupported; the SCL used, at or below the one asked
  CHECK(rejectedAs(call(c, I2cController::kOpConfigure, configureBytes(0, 1000), out), kRejectMalformed));
  CHECK(rejectedAs(call(c, I2cController::kOpConfigure, configureBytes(100000, 0), out), kRejectMalformed));
  CHECK(rejectedAs(call(c, I2cController::kOpConfigure, configureBytes(I2cController::kMaxClockHz + 1, 1000), out),
                   kRejectUnsupported));
  CHECK(rejectedAs(call(c, I2cController::kOpConfigure, configureBytes(I2cController::kMinClockHz - 1, 1000), out),
                   kRejectUnsupported));
  CHECK(rejectedAs(call(c, I2cController::kOpConfigure,
                        configureBytes(100000, I2cController::kMaxStretchTimeoutUs + 1), out), kRejectUnsupported));
  CHECK(ok(call(c, I2cController::kOpConfigure, configureBytes(300000, 1000), out)));
  CHECK(out.size() == 4 && getU32(out.data()) <= 300000 && getU32(out.data()) > 290000);   // 160 MHz: 267 cycles a half
  CHECK(ok(call(c, I2cController::kOpConfigure, configureBytes(100000, 1000), out)));
  CHECK(out.size() == 4 && getU32(out.data()) == 100000);

  // a register read: write the register, repeated START, read two, STOP
  bus.regs[3] = 0xab;
  bus.regs[4] = 0xcd;
  {
    Bytes p = {i2c::kTransferFlagsStop, 2};
    segment(p, i2c::kDirWrite, 0x50, 1, {0x03});
    segment(p, i2c::kDirRead, 0x50, 2);
    CHECK(ok(call(c, I2cController::kOpTransfer, p, out)));
    const Answer a = answerOf(out);
    CHECK(a.result == i2c::kResultDone && a.at == 0 && a.index == 0 && a.data == Bytes({0xab, 0xcd}));
    CHECK(bus.log == "S W:a0a W:03a Sr W:a1a R:aba R:cdn P");   // the last byte read NACKed
  }

  // without STOP: the bus is held (status held 1); the next transfer starts with a repeated START
  bus.log.clear();
  {
    Bytes p = {0, 1};
    segment(p, i2c::kDirWrite, 0x50, 1, {0x00});
    CHECK(ok(call(c, I2cController::kOpTransfer, p, out)));
    CHECK(answerOf(out).result == i2c::kResultDone);
    CHECK(bus.log == "S W:a0a W:00a");
    CHECK(ok(call(c, I2cController::kOpStatus, {}, out)) && out.size() == 10 && out[0] == 1 && out[1] == 1);
    Bytes q = {i2c::kTransferFlagsStop, 1};
    segment(q, i2c::kDirRead, 0x50, 1);
    CHECK(ok(call(c, I2cController::kOpTransfer, q, out)));
    CHECK(bus.log == "S W:a0a W:00a Sr W:a1a R:00n P");
    CHECK(ok(call(c, I2cController::kOpStatus, {}, out)) && out[1] == 0);
  }

  // n 0 with STOP: a STOP alone, only while held; not counted. n 0 without it: malformed
  bus.log.clear();
  CHECK(ok(call(c, I2cController::kOpTransfer, {i2c::kTransferFlagsStop, 0}, out)) && out.size() == 6);
  CHECK(bus.log.empty());
  CHECK(rejectedAs(call(c, I2cController::kOpTransfer, {0, 0}, out), kRejectMalformed));

  // nobody at the address: result 1, a STOP, nothing read; the segments after it not run
  bus.log.clear();
  {
    Bytes p = {i2c::kTransferFlagsStop, 2};
    segment(p, i2c::kDirWrite, 0x51, 0);
    segment(p, i2c::kDirRead, 0x50, 1);
    CHECK(ok(call(c, I2cController::kOpTransfer, p, out)));
    const Answer a = answerOf(out);
    CHECK(a.result == i2c::kResultAddressNack && a.at == 0 && a.data.empty());
    CHECK(bus.log == "S W:a2n P");
  }
  // a data byte NACKed: result 2, its place in the segment; the second segment's
  bus.log.clear();
  {
    Bytes p = {0, 2};
    segment(p, i2c::kDirWrite, 0x50, 1, {0x00});
    segment(p, i2c::kDirWrite, 0x50, 3, {0x0f, 0x11, 0x22});   // register 0x0f: 0x11 lands, 0x22 is past it
    CHECK(ok(call(c, I2cController::kOpTransfer, p, out)));
    const Answer a = answerOf(out);
    CHECK(a.result == i2c::kResultDataNack && a.at == 1 && a.index == 2);
    CHECK(bus.log == "S W:a0a W:00a Sr W:a0a W:0fa W:11a W:22n P");   // a STOP although flags asked none
    CHECK(ok(call(c, I2cController::kOpStatus, {}, out)) && out[1] == 0);
  }
  // a stretch past the timeout: result 4 and a STOP; a bus error: result 3, the lines let go
  bus.log.clear();
  {
    Bytes p = {i2c::kTransferFlagsStop, 1};
    segment(p, i2c::kDirWrite, 0x50, 1, {0x00});
    bus.stretch = true;
    CHECK(ok(call(c, I2cController::kOpTransfer, p, out)) && answerOf(out).result == i2c::kResultStretchTimeout);
    CHECK(bus.log == "S stretch P");
    bus.log.clear();
    bus.bus_error = true;
    CHECK(ok(call(c, I2cController::kOpTransfer, p, out)) && answerOf(out).result == i2c::kResultBusError);
    CHECK(bus.log == "S lost let-go");
  }
  // the counters: every transfer answered, the errors those with a result
  CHECK(ok(call(c, I2cController::kOpStatus, {}, out)));
  CHECK(getU32(out.data() + 2) == 7 && getU32(out.data() + 6) == 4);

  // the request's refusals, nothing sent
  bus.log.clear();
  {
    Bytes p = {i2c::kTransferFlagsStop, 1};
    segment(p, i2c::kDirRead, 0x50, 0);
    CHECK(rejectedAs(call(c, I2cController::kOpTransfer, p, out), kRejectMalformed));   // a read of 0
    Bytes q = {i2c::kTransferFlagsStop, 1};
    segment(q, 2, 0x50, 1);
    CHECK(rejectedAs(call(c, I2cController::kOpTransfer, q, out), kRejectMalformed));   // dir 2
    Bytes r = {i2c::kTransferFlagsStop, 1};
    segment(r, i2c::kDirWrite, 0x80, 0);
    CHECK(rejectedAs(call(c, I2cController::kOpTransfer, r, out), kRejectMalformed));   // address past 0x7f
    Bytes s = {i2c::kTransferFlagsStop, 1};
    segment(s, i2c::kDirWrite, 0x50, 3, {1, 2});
    CHECK(rejectedAs(call(c, I2cController::kOpTransfer, s, out), kRejectMalformed));   // data short of count
    Bytes t = {i2c::kTransferFlagsStop | 0x02, 1};
    segment(t, i2c::kDirWrite, 0x50, 0);
    CHECK(rejectedAs(call(c, I2cController::kOpTransfer, t, out), kRejectUnsupported));   // reserved flags
    Bytes u = {i2c::kTransferFlagsStop, 2};
    segment(u, i2c::kDirRead, 0x50, 200);
    segment(u, i2c::kDirRead, 0x50, 57);
    CHECK(rejectedAs(call(c, I2cController::kOpTransfer, u, out), kRejectUnsupported));   // past max_length
    CHECK(bus.log.empty());
  }
  // the reserved addresses are taken (fixture §5: unlike the target's)
  {
    Bytes p = {i2c::kTransferFlagsStop, 1};
    segment(p, i2c::kDirWrite, 0x00, 0);
    CHECK(ok(call(c, I2cController::kOpTransfer, p, out)) && answerOf(out).result == i2c::kResultAddressNack);
  }

  // describe: the roles, the limits, the pull-ups, max_stretch_timeout_us
  {
    Bytes d(256);
    const size_t n = c.describe(d.data(), d.size());
    CHECK(n > 0);
    bool timeout = false;
    for (size_t at = 0; at + 3 <= n;) {
      const uint8_t tag = d[at];
      const uint16_t len = getU16(d.data() + at + 1);
      if (tag == i2c::kTlvDescribeMaxStretchTimeoutUs) timeout = len == 4 && getU32(d.data() + at + 3) == I2cController::kMaxStretchTimeoutUs;
      at += 3 + len;
    }
    CHECK(timeout);
  }

  // the session ends with the bus held: a STOP; the plan released: state 0, the counters 0
  bus.log.clear();
  {
    Bytes p = {0, 1};
    segment(p, i2c::kDirWrite, 0x50, 0);
    CHECK(ok(call(c, I2cController::kOpTransfer, p, out)));
    c.sessionOver();
    CHECK(bus.log == "S W:a0a P");
  }
  c.planRelease();
  CHECK(ok(call(c, I2cController::kOpStatus, {}, out)) && out[0] == 0 && getU32(out.data() + 2) == 0);
  // a start that fails: completed failed, state 0
  CHECK(c.planApply(roles, 2));
  I2cController::fake_fail_start = true;
  const Result failed_start = call(c, I2cController::kOpConfigure, configureBytes(100000, 1000), out);
  CHECK(failed_start.resolution == kResolutionCompleted && failed_start.detail == kOutcomeFailed);
  CHECK(ok(call(c, I2cController::kOpStatus, {}, out)) && out[0] == 0);

  printf("i2c-controller: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
