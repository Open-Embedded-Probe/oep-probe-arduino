// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the fixture UART's receive on a fake UART (OEP_HOST_FAKE_UART: platformUartBegin starts it,
// platformUartTakeOverrun reports the hardware's overruns). The UART begins on the core the sketch named
// (setInterruptCore, every begin: plan, configure, a configure refused and put back); configure takes 2000000 and
// refuses a faster rate unsupported; what the UART received is in the stream in order; every overrun the hardware
// reports is one mark lost (detail 1 overflow) at a position no earlier than the bytes taken before it was seen, and
// none is placed without one (oep-if-fixture §2, oep-if-common §1.3).
#include <stdio.h>

#include <vector>

#include "OepFixture.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
namespace ua = reg::fixture_uart;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                                       \
  do {                                                                                    \
    ++checks;                                                                             \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

struct Mark { uint32_t serial; uint64_t position; uint8_t kind, detail; };

static Result configure(FixtureUart &uart, uint32_t baud, uint8_t *out, size_t capacity) {
  uint8_t p[4];
  putU32(p, baud);
  return uart.handle(ua::kOpConfigure, p, sizeof p, out, capacity);
}

static std::vector<Mark> marks(FixtureUart &uart, uint32_t from) {
  std::vector<Mark> got;
  for (;;) {
    uint8_t p[4], out[1024];
    putU32(p, from);
    const Result r = uart.handle(ua::kOpMarks, p, sizeof p, out, sizeof out);
    if (refused(r) || r.length < 2) return got;
    for (uint8_t i = 0; i < out[1]; ++i) {
      const uint8_t *m = out + 2 + PositionStream::kMarkBytes * i;
      got.push_back({getU32(m), getU64(m + 4), m[12], m[21]});
      from = getU32(m) + 1;
    }
    if (!out[0]) return got;
  }
}

static std::vector<uint8_t> readAll(FixtureUart &uart, uint64_t from) {
  std::vector<uint8_t> got;
  for (;;) {
    uint8_t p[11], out[1100];
    p[0] = 0;   // from a position
    putU64(p + 1, from);
    putU16(p + 9, 1000);
    const Result r = uart.handle(ua::kOpRead, p, sizeof p, out, sizeof out);
    if (refused(r) || r.length < PositionStream::kReadHeader) return got;
    const uint16_t len = getU16(out + 9);
    got.insert(got.end(), out + PositionStream::kReadHeader, out + PositionStream::kReadHeader + len);
    from = getU64(out) + len;
    if (!(out[8] & reg::common::kReadFlagsMore)) return got;
  }
}

static void testCoreAndRates() {
  PinTable pins((1ull << 4) | (1ull << 5));
  HardwareSerial serial;
  FixtureUart uart(pins, serial, 0, 2);
  uart.setInterruptCore(0);
  const RoleAssignment plan[] = {{1, ua::kRoleRx, 4}, {1, ua::kRoleTx, 5}};
  CHECK(uart.planCheck(plan, 2) == 0 && uart.planApply(plan, 2));
  CHECK(serial.fake_running && serial.fake_irq_core == 0 && serial.fake_baud == FixtureUart::kDefaultBaud);
  uint8_t out[64];
  Result r = configure(uart, 2000000, out, sizeof out);
  CHECK(r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess && getU32(out) == 2000000);
  CHECK(serial.fake_irq_core == 0 && serial.fake_baud == 2000000);
  const int begins = serial.fake_begins;
  r = configure(uart, 2000001, out, sizeof out);   // over kMaxBaud: unsupported, the running UART untouched
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported);
  CHECK(serial.fake_begins == begins && serial.fake_baud == 2000000 && uart.baud() == 2000000);
  uart.planRelease();
  CHECK(!serial.fake_running);
  FixtureUart other(pins, serial, 1, 5);   // no setInterruptCore: the caller's core (-1)
  CHECK(other.planApply(plan, 2) && serial.fake_irq_core == -1);
  other.planRelease();
}

// Bytes and overruns in rounds: every byte in order, one lost mark per overrun, each at or after the bytes taken
// before the overrun was seen and at or before the bytes taken after it.
static void testOverrunMarks() {
  PinTable pins((1ull << 4) | (1ull << 5));
  HardwareSerial serial;
  FixtureUart uart(pins, serial, 0, 2);
  const RoleAssignment plan[] = {{1, ua::kRoleRx, 4}};
  CHECK(uart.planApply(plan, 1));
  uint8_t out[64];
  CHECK(!refused(configure(uart, 2000000, out, sizeof out)));
  std::vector<uint8_t> sent;
  std::vector<std::pair<uint64_t, uint64_t>> windows;   // per overrun: the stream's end before / after the poll seeing it
  uint32_t seed = 12345;
  auto rnd = [&seed](uint32_t n) { seed = seed * 1103515245u + 12345u; return (seed >> 16) % n; };
  uint64_t taken = 0;
  std::vector<Mark> got;   // read after every poll: the stream keeps 16 marks
  for (int round = 0; round < 60; ++round) {
    const uint32_t before = rnd(80), after = rnd(80);
    for (uint32_t i = 0; i < before; ++i) { serial.fakeReceive(static_cast<uint8_t>(sent.size())); sent.push_back(static_cast<uint8_t>(sent.size())); }
    const bool overrun = rnd(3) == 0;
    if (overrun) serial.fake_overrun = true;
    for (uint32_t i = 0; i < after; ++i) { serial.fakeReceive(static_cast<uint8_t>(sent.size())); sent.push_back(static_cast<uint8_t>(sent.size())); }
    const uint64_t start = taken;
    uart.poll();
    taken = sent.size();
    if (overrun) windows.push_back({start + before, taken});
    for (const Mark &m : marks(uart, got.empty() ? 0 : got.back().serial + 1)) got.push_back(m);
  }
  CHECK(sent.size() < 8192);   // within the stream's ring: no lost mark of its own
  CHECK(readAll(uart, 0) == sent);
  CHECK(!windows.empty() && got.size() == windows.size());
  size_t bad = 0;
  for (size_t i = 0; i < got.size() && i < windows.size(); ++i)
    if (got[i].kind != reg::common::kMarkKindLost || got[i].detail != reg::common::kMarkDetailLostOverflow ||
        got[i].position < windows[i].first || got[i].position > windows[i].second)
      ++bad;
  CHECK(bad == 0);
  uart.poll();   // nothing new: no mark
  CHECK(marks(uart, got.empty() ? 0 : got.back().serial + 1).empty());
  uart.planRelease();
}

int main() {
  testCoreAndRates();
  testOverrunMarks();
  printf("test_fixture_uart: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
