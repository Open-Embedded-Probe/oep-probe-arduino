// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the fixture UART's receive on a fake UART (OEP_HOST_FAKE_UART: platformUartBegin starts it). The UART
// begins on the core the sketch named (setInterruptCore, every begin: plan, configure, a configure refused and put
// back); configure takes 2000000 and refuses a faster rate unsupported; what the UART received is in the stream in
// order. A lost mark is at or before the first byte after the loss, never after it (oep-if-common §1.3, oep-if-fixture
// §2), at it where the driver tells: ESP-IDF style (the default build) - the driver's events in order, a FIFO overflow
// exactly at the gap also when the events come late, a framing / parity error at or before the bad byte, a buffer full
// not a loss, dropped events a loss at the first byte not counted, more losses than the ledger holds merged earlier;
// arduino-pico style (OEP_HOST_FAKE_UART_RP2) - the receive queue's overflow exactly at the gap, the PL011's overrun
// and a break at or before it.
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

struct Feed {
  explicit Feed(HardwareSerial &s) : serial(s) {}
  HardwareSerial &serial;
  std::vector<uint8_t> delivered;   // the bytes the UART kept, in order
  std::vector<uint64_t> gaps;       // per loss: the index in `delivered` of the first byte after it
  uint8_t next = 0;
  void bytes(size_t n, uint8_t type = 0) {
    std::vector<uint8_t> b;
    for (size_t i = 0; i < n; ++i) b.push_back(next++);
    serial.fakeChunk(b.data(), b.size(), type);
    delivered.insert(delivered.end(), b.begin(), b.end());
  }
  void lose(size_t n) {   // n bytes never reach the driver
    next = static_cast<uint8_t>(next + n);
    gaps.push_back(delivered.size());
  }
};

static std::vector<Mark> newMarks(FixtureUart &uart, std::vector<Mark> &all) {
  std::vector<Mark> got = marks(uart, all.empty() ? 0 : all.back().serial + 1);
  all.insert(all.end(), got.begin(), got.end());
  return got;
}

static bool lostAt(const Mark &m, uint64_t position, uint8_t detail = reg::common::kMarkDetailLostOverflow) {
  return m.kind == reg::common::kMarkKindLost && m.detail == detail && m.position == position;
}

#if !defined(OEP_HOST_FAKE_UART_RP2)
enum : uint8_t { kEvData = 0, kEvBufferFull = 1, kEvFifoOverflow = 2, kEvBreak = 3, kEvFrame = 4, kEvParity = 5 };

// The bench's case (X035, 2000000, a 256-byte burst): 4 bytes, 5 lost, 247 more - the mark at offset 4, whether the
// events are taken as they come or after all the bytes are in the driver's ring.
static void testOverflowAtGap() {
  for (int late = 0; late < 2; ++late) {
    PinTable pins((1ull << 4) | (1ull << 5));
    HardwareSerial serial;
    FixtureUart uart(pins, serial, 0, 2);
    const RoleAssignment plan[] = {{1, ua::kRoleRx, 4}};
    CHECK(uart.planApply(plan, 1));
    uint8_t out[64];
    CHECK(!refused(configure(uart, 2000000, out, sizeof out)));
    std::vector<Mark> all;
    const uint64_t base = readAll(uart, 0).size();
    Feed f(serial);
    serial.fake_hold_events = late;
    f.bytes(4);
    if (!late) uart.poll();
    f.lose(5);
    serial.fakeEvent(kEvFifoOverflow);
    for (int i = 0; i < 7; ++i) {
      f.bytes(i < 6 ? 32 : 55);
      if (!late) uart.poll();
    }
    uart.poll();
    if (late) {
      CHECK(readAll(uart, 0).empty());   // nothing past what the events counted (none taken yet)
      serial.fake_hold_events = false;
      uart.poll();
    }
    CHECK(readAll(uart, 0) == f.delivered && f.delivered.size() == 251);
    const std::vector<Mark> got = newMarks(uart, all);
    CHECK(got.size() == 1 && lostAt(got[0], base + 4));
    uart.planRelease();
  }
}

// Rounds of chunks, overflows, held events and polls at random: every byte in order, one mark per overflow, each at the
// gap exactly.
static void testOverflowRounds() {
  PinTable pins((1ull << 4) | (1ull << 5));
  HardwareSerial serial;
  FixtureUart uart(pins, serial, 0, 2);
  const RoleAssignment plan[] = {{1, ua::kRoleRx, 4}};
  CHECK(uart.planApply(plan, 1));
  uint8_t out[64];
  CHECK(!refused(configure(uart, 2000000, out, sizeof out)));
  Feed f(serial);
  std::vector<Mark> all;
  uint32_t seed = 12345;
  auto rnd = [&seed](uint32_t n) { seed = seed * 1103515245u + 12345u; return (seed >> 16) % n; };
  for (int round = 0; round < 400; ++round) {
    serial.fake_hold_events = rnd(4) == 0;
    const uint32_t chunks = rnd(4);
    for (uint32_t c = 0; c < chunks; ++c) f.bytes(1 + rnd(40));
    if (rnd(3) == 0) {
      f.lose(1 + rnd(130));
      serial.fakeEvent(kEvFifoOverflow);
    }
    if (rnd(2)) uart.poll();
    if (f.delivered.size() > 7000) break;   // within the stream's ring
    newMarks(uart, all);
  }
  serial.fake_hold_events = false;
  uart.poll();
  newMarks(uart, all);
  CHECK(readAll(uart, 0) == f.delivered);
  CHECK(f.gaps.size() > 20 && all.size() == f.gaps.size());
  size_t bad = 0;
  for (size_t i = 0; i < all.size() && i < f.gaps.size(); ++i) bad += !lostAt(all[i], f.gaps[i]);
  CHECK(bad == 0);
  uart.poll();   // nothing new: no mark
  CHECK(newMarks(uart, all).empty());
  uart.planRelease();
}

// A framing / parity error (or a break) queued after the chunk that brought the bad byte: at or before that byte,
// not before the chunk. A buffer full is not a loss. The driver's event queue full: a loss right after the events it
// held (what came then was dropped), every byte still taken.
static void testErrorsAndDroppedEvents() {
  PinTable pins((1ull << 4) | (1ull << 5));
  HardwareSerial serial;
  FixtureUart uart(pins, serial, 0, 2);
  const RoleAssignment plan[] = {{1, ua::kRoleRx, 4}};
  CHECK(uart.planApply(plan, 1));
  uint8_t out[64];
  CHECK(!refused(configure(uart, 2000000, out, sizeof out)));
  Feed f(serial);
  std::vector<Mark> all;
  f.bytes(10);
  uart.poll();
  serial.fake_hold_events = true;   // one batch of the interrupt: a chunk with the bad byte (its 3rd), then the error
  f.bytes(20);
  serial.fakeEvent(kEvFrame);
  serial.fake_hold_events = false;
  uart.poll();
  std::vector<Mark> got = newMarks(uart, all);
  CHECK(got.size() == 1 && lostAt(got[0], 10, reg::common::kMarkDetailLostFraming));
  f.bytes(5);
  uart.poll();   // a batch of its own: the bad byte comes after what was taken
  serial.fakeEvent(kEvParity);
  f.bytes(5);
  uart.poll();
  got = newMarks(uart, all);
  CHECK(got.size() == 1 && lostAt(got[0], 35, reg::common::kMarkDetailLostParity));
  f.bytes(40, kEvBufferFull);   // stashed by the driver, not lost
  uart.poll();
  CHECK(newMarks(uart, all).empty());
  serial.fake_event_queue = 2;   // the driver's event queue full with the task away: two chunks' events dropped
  serial.fake_hold_events = true;
  f.bytes(7);
  f.bytes(8);
  f.bytes(9);
  f.bytes(6);
  serial.fake_hold_events = false;
  uart.poll();
  serial.fake_event_queue = 20;
  f.bytes(5);   // counted after the drop
  uart.poll();
  got = newMarks(uart, all);
  CHECK(got.size() == 1 && lostAt(got[0], 95));
  CHECK(readAll(uart, 0) == f.delivered);
  uart.planRelease();
}

// More losses before a poll than the ledger holds: the first 16 at their gaps, the rest merged into one at the 17th.
// A configure takes what the UART received, and its losses, before it starts the UART again.
static void testManyLosses() {
  PinTable pins((1ull << 4) | (1ull << 5));
  HardwareSerial serial;
  FixtureUart uart(pins, serial, 0, 2);
  const RoleAssignment plan[] = {{1, ua::kRoleRx, 4}};
  CHECK(uart.planApply(plan, 1));
  uint8_t out[64];
  CHECK(!refused(configure(uart, 2000000, out, sizeof out)));
  Feed f(serial);
  std::vector<Mark> all;
  for (int i = 0; i < 24; ++i) {
    f.bytes(3);
    f.lose(2);
    serial.fakeEvent(kEvFifoOverflow);
  }
  f.bytes(3);
  for (int i = 0; i < 3; ++i) {
    uart.poll();
    newMarks(uart, all);
  }
  CHECK(readAll(uart, 0) == f.delivered);
  // 17 marks; the stream keeps 16 (the first pushed out, its serial skipped)
  CHECK(all.size() == 16 && all.front().serial == 1 && all.back().serial == 16);
  size_t bad = 0;
  for (const Mark &m : all) bad += !lostAt(m, f.gaps[m.serial]);
  CHECK(bad == 0);
  // a configure while the events of a loss wait: taken before the UART starts again, the mark at its place
  f.bytes(4);
  serial.fakeEvent(kEvFifoOverflow);
  CHECK(!refused(configure(uart, 1000000, out, sizeof out)));
  CHECK(readAll(uart, 0) == f.delivered);
  const std::vector<Mark> got = newMarks(uart, all);
  CHECK(got.size() == 1 && lostAt(got[0], f.delivered.size()));
  uart.planRelease();
}
#else
// The receive queue (64 bytes here) full and 36 more dropped: the mark right after its 64.
static void testQueueOverflowAtGap() {
  PinTable pins((1ull << 4) | (1ull << 5));
  HardwareSerial serial;
  serial.fake_cap = 64;
  FixtureUart uart(pins, serial, 0, 2);
  const RoleAssignment plan[] = {{1, ua::kRoleRx, 4}};
  CHECK(uart.planApply(plan, 1));
  Feed f(serial);
  std::vector<Mark> all;
  f.bytes(10);
  uart.poll();
  f.bytes(64);   // the queue full
  f.lose(10);
  for (int i = 0; i < 10; ++i) serial.fakeReceive(0xEE);   // dropped
  uart.poll();
  f.bytes(20);
  uart.poll();
  CHECK(readAll(uart, 0) == f.delivered);
  const std::vector<Mark> got = newMarks(uart, all);
  CHECK(got.size() == 1 && lostAt(got[0], 74));
  uart.planRelease();
}

// The PL011's overrun and a break, seen only as flags: at or before the gap, not before the bytes counted at the look
// before the one that saw them.
static void testOverrunRounds() {
  PinTable pins((1ull << 4) | (1ull << 5));
  HardwareSerial serial;
  FixtureUart uart(pins, serial, 0, 2);
  const RoleAssignment plan[] = {{1, ua::kRoleRx, 4}};
  CHECK(uart.planApply(plan, 1));
  Feed f(serial);
  std::vector<Mark> all;
  std::vector<std::pair<uint64_t, uint64_t>> windows;   // per overrun: the earliest allowed place, the gap
  std::vector<uint8_t> details;
  uint32_t seed = 4321;
  auto rnd = [&seed](uint32_t n) { seed = seed * 1103515245u + 12345u; return (seed >> 16) % n; };
  uint64_t counted_before = 0, counted = 0;   // the bytes received at the last two looks
  for (int round = 0; round < 80; ++round) {
    const uint32_t before = rnd(80), after = rnd(80);
    f.bytes(before);
    const uint32_t kind = rnd(4);
    if (kind == 0) { f.lose(1 + rnd(20)); serial.fake_overrun = true; }
    if (kind == 1) { f.gaps.push_back(f.delivered.size()); serial.fake_break = true; }
    if (kind <= 1) { windows.push_back({counted, f.delivered.size()}); details.push_back(kind ? 2 : 1); }
    f.bytes(after);
    uart.poll();
    counted_before = counted;
    counted = f.delivered.size();
    newMarks(uart, all);
  }
  (void)counted_before;
  CHECK(readAll(uart, 0) == f.delivered);
  CHECK(windows.size() > 10 && all.size() == windows.size());
  size_t bad = 0;
  for (size_t i = 0; i < all.size() && i < windows.size(); ++i)
    bad += all[i].kind != reg::common::kMarkKindLost || all[i].detail != details[i] ||
           all[i].position < windows[i].first || all[i].position > windows[i].second;
  CHECK(bad == 0);
  uart.planRelease();
}
#endif

int main() {
  testCoreAndRates();
#if !defined(OEP_HOST_FAKE_UART_RP2)
  testOverflowAtGap();
  testOverflowRounds();
  testErrorsAndDroppedEvents();
  testManyLosses();
  printf("test_fixture_uart (ESP-IDF driver): %d checks, %d failures\n", checks, failures);
#else
  testQueueOverflowAtGap();
  testOverrunRounds();
  printf("test_fixture_uart (arduino-pico): %d checks, %d failures\n", checks, failures);
#endif
  return failures ? 1 : 0;
}
