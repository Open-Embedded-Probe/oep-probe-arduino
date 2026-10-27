// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.fixture.spi-controller (oep-spec oep-if-fixture §6) on a fake bus (OEP_HOST_FAKE_SPI_BUS: a target
// that answers each byte with its complement and records the times asked in CPU cycles) - the plan's roles, configure's
// clock and refusals, transfers run back to back with the times at or above the ones asked (whole cycles, answered in
// ns rounded down; the first one's gap 0), a CS-only transfer (count 0), the request's refusals (form, n 0, past
// max_delay_ns, past max_length, state 0), status and transfers, describe's tags.
#include <stdio.h>

#include <vector>

#include "OepSpiController.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace spi = reg::fixture_spi_controller;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

struct Seen {
  bool first;
  uint32_t gap, setup, hold;
  Bytes tx;
};
struct FakeBus final : SpiHostBus {
  std::vector<Seen> seen;
  void transfer(bool first, uint32_t gap, uint32_t setup, uint32_t hold, const uint8_t *tx, uint8_t *rx,
                size_t count) override {
    seen.push_back({first, gap, setup, hold, Bytes(tx, tx + count)});
    for (size_t i = 0; i < count; ++i) rx[i] = static_cast<uint8_t>(~tx[i]);
  }
};

static Result call(SpiController &c, uint8_t op, const Bytes &payload, Bytes &out) {
  out.assign(1024, 0);
  const Result r = c.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static bool rejectedAs(const Result &r, uint8_t reason) { return r.resolution == kResolutionRejected && r.detail == reason; }
static bool unavailableCause(const Result &r, const Bytes &out, uint8_t cause) {
  return rejectedAs(r, kRejectUnavailable) && out.size() >= 4 && out[0] == 0x01 && out[3] == cause;
}
static Bytes configureBytes(uint8_t mode, uint8_t order, uint32_t clock) {
  Bytes p = {mode, order, 0, 0, 0, 0};
  putU32(p.data() + 2, clock);
  return p;
}
static void item(Bytes &p, uint32_t gap, uint32_t setup, uint32_t hold, const Bytes &tx) {
  const size_t at = p.size();
  p.resize(at + 14);
  putU32(p.data() + at, gap);
  putU32(p.data() + at + 4, setup);
  putU32(p.data() + at + 8, hold);
  putU16(p.data() + at + 12, static_cast<uint16_t>(tx.size()));
  p.insert(p.end(), tx.begin(), tx.end());
}

int main() {
  static PinTable pins(0xf0);
  static SpiController c(pins);
  static FakeBus bus;
  SpiController::host_bus = &bus;
  Bytes out;

  // the plan: roles 1-4 once each, on distinct channels
  {
    const RoleAssignment three[] = {{0, 1, 4}, {0, 2, 5}, {0, 3, 6}};
    const RoleAssignment same[] = {{0, 1, 4}, {0, 2, 5}, {0, 3, 6}, {0, 4, 6}};
    const RoleAssignment twice[] = {{0, 1, 4}, {0, 1, 5}, {0, 3, 6}, {0, 4, 7}};
    CHECK(c.planCheck(three, 3) == kRejectUnsupported);
    CHECK(c.planCheck(same, 4) == kRejectUnsupported);
    CHECK(c.planCheck(twice, 4) == kRejectUnsupported);
  }
  CHECK(unavailableCause(call(c, SpiController::kOpConfigure, configureBytes(0, 0, 1000000), out), out, 6));   // no plan
  const RoleAssignment roles[] = {{0, 1, 4}, {0, 2, 5}, {0, 3, 6}, {0, 4, 7}};
  CHECK(c.planCheck(roles, 4) == 0);
  CHECK(c.planApply(roles, 4));
  {
    Bytes p = {1};
    item(p, 0, 0, 0, {1});
    CHECK(unavailableCause(call(c, SpiController::kOpTransfer, p, out), out, 6));   // state 0
  }

  // configure: the form, the limits, the SCK used at or below the one asked
  CHECK(rejectedAs(call(c, SpiController::kOpConfigure, configureBytes(4, 0, 1000000), out), kRejectMalformed));
  CHECK(rejectedAs(call(c, SpiController::kOpConfigure, configureBytes(0, 2, 1000000), out), kRejectMalformed));
  CHECK(rejectedAs(call(c, SpiController::kOpConfigure, configureBytes(0, 0, 0), out), kRejectMalformed));
  CHECK(rejectedAs(call(c, SpiController::kOpConfigure, configureBytes(0, 0, SpiController::kMaxClockHz + 1), out),
                   kRejectUnsupported));
  CHECK(ok(call(c, SpiController::kOpConfigure, configureBytes(3, 1, 1200000), out)));
  CHECK(out.size() == 4 && getU32(out.data()) == 1194029);   // 160 MHz: 67 cycles a half
  CHECK(ok(call(c, SpiController::kOpStatus, {}, out)) && out.size() == 7 && out[0] == 1 && out[1] == 3 && out[2] == 1);
  CHECK(ok(call(c, SpiController::kOpConfigure, configureBytes(0, 0, 1000000), out)) && getU32(out.data()) == 1000000);

  // two transfers back to back: the times in whole cycles (6.25 ns each), answered rounded down, never under the ask;
  // the first one's gap is not used (0)
  {
    Bytes p = {2};
    item(p, 500, 1000, 503, {0x9f});
    item(p, 2010, 1001, 0, {0x01, 0x02});
    CHECK(ok(call(c, SpiController::kOpTransfer, p, out)));
    CHECK(bus.seen.size() == 2);
    CHECK(bus.seen[0].first && bus.seen[0].gap == 0 && bus.seen[0].setup == 160 && bus.seen[0].hold == 81);
    CHECK(!bus.seen[1].first && bus.seen[1].gap == 322 && bus.seen[1].setup == 161 && bus.seen[1].hold == 0);
    CHECK(bus.seen[1].tx == Bytes({0x01, 0x02}));
    CHECK(out.size() == 14 + 1 + 14 + 2);
    CHECK(getU32(out.data()) == 0 && getU32(out.data() + 4) == 1000 && getU32(out.data() + 8) == 506);
    CHECK(getU16(out.data() + 12) == 1 && out[14] == 0x60);
    CHECK(getU32(out.data() + 15) == 2012 && getU32(out.data() + 19) == 1006 && getU32(out.data() + 23) == 0);
    CHECK(getU16(out.data() + 27) == 2 && out[29] == 0xfe && out[30] == 0xfd);
  }
  // a CS-only transfer: count 0, nothing clocked
  bus.seen.clear();
  {
    Bytes p = {1};
    item(p, 0, 100, 100, {});
    CHECK(ok(call(c, SpiController::kOpTransfer, p, out)) && out.size() == 14 && getU16(out.data() + 12) == 0);
    CHECK(bus.seen.size() == 1 && bus.seen[0].tx.empty());
  }
  CHECK(ok(call(c, SpiController::kOpStatus, {}, out)) && getU32(out.data() + 3) == 3);

  // the request's refusals, nothing on the bus
  bus.seen.clear();
  CHECK(rejectedAs(call(c, SpiController::kOpTransfer, {0}, out), kRejectMalformed));   // n 0
  {
    Bytes p = {1};
    item(p, 0, 0, 0, {1, 2});
    p.pop_back();
    CHECK(rejectedAs(call(c, SpiController::kOpTransfer, p, out), kRejectMalformed));   // tx short of count
    Bytes q = {1};
    item(q, 0, SpiController::kMaxDelayNs + 1, 0, {1});
    CHECK(rejectedAs(call(c, SpiController::kOpTransfer, q, out), kRejectUnsupported));   // past max_delay_ns
    Bytes r = {2};
    item(r, 0, 0, 0, Bytes(200));
    item(r, 0, 0, 0, Bytes(57));
    CHECK(rejectedAs(call(c, SpiController::kOpTransfer, r, out), kRejectUnsupported));   // past max_length
    Bytes s = {101};
    for (int k = 0; k < 101; ++k) item(s, 0, SpiController::kMaxDelayNs, 0, {});
    CHECK(rejectedAs(call(c, SpiController::kOpTransfer, s, out), kRejectUnsupported));   // past max_op_ms
  }
  CHECK(bus.seen.empty());

  // describe: time_step_ns (one cycle, rounded up), modes, max_delay_ns
  {
    Bytes d(256);
    const size_t n = c.describe(d.data(), d.size());
    CHECK(n > 0);
    uint32_t step = 0, delay = 0;
    uint8_t modes = 0;
    for (size_t at = 0; at + 3 <= n;) {
      const uint8_t tag = d[at];
      const uint16_t len = getU16(d.data() + at + 1);
      if (tag == spi::kTlvDescribeTimeStepNs) step = getU32(d.data() + at + 3);
      if (tag == spi::kTlvDescribeMaxDelayNs) delay = getU32(d.data() + at + 3);
      if (tag == spi::kTlvDescribeModes) modes = d[at + 3];
      at += 3 + len;
    }
    CHECK(step == 7 && delay == SpiController::kMaxDelayNs && modes == 0x0f);
  }

  // the plan released: state 0, mode and order 0, transfers 0; a start that fails: completed failed, state 0
  c.planRelease();
  CHECK(ok(call(c, SpiController::kOpStatus, {}, out)) && out[0] == 0 && out[1] == 0 && getU32(out.data() + 3) == 0);
  CHECK(c.planApply(roles, 4));
  SpiController::fake_fail_start = true;
  const Result failed_start = call(c, SpiController::kOpConfigure, configureBytes(1, 0, 1000000), out);
  CHECK(failed_start.resolution == kResolutionCompleted && failed_start.detail == kOutcomeFailed);
  CHECK(ok(call(c, SpiController::kOpStatus, {}, out)) && out[0] == 0 && out[1] == 0);

  printf("spi-controller: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
