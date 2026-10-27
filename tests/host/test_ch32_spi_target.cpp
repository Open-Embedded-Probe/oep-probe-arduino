// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.fixture.spi-target (oep-spec oep-if-fixture §4) on a CH32's SPI slave (OEP_HOST_FAKE_CH32_SPI_SLAVE:
// a fake of the core's CH32RVSPISlave) - configure (a refused route fails to state 0), arm answering with tx then 0 and
// keeping MOSI, bits as whole bytes, a CS with no clock using nothing up, an unarmed transfer counted with an error, a
// transfer past the armed length (cut, an error), the queue full, one arm at a time, describe's modes and cs_setup_ns.
#include <stdio.h>

#include <vector>

#include "OepCh32SpiTarget.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace spi = reg::fixture_spi_target;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

static Result call(Ch32SpiTarget &t, uint8_t op, const Bytes &payload, Bytes &out) {
  out.assign(512, 0);
  const Result r = t.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static bool rejectedAs(const Result &r, uint8_t reason) { return r.resolution == kResolutionRejected && r.detail == reason; }
static Result arm(Ch32SpiTarget &t, uint16_t length, const Bytes &tx) {
  Bytes p(4), out;
  putU16(p.data(), length);
  putU16(p.data() + 2, static_cast<uint16_t>(tx.size()));
  p.insert(p.end(), tx.begin(), tx.end());
  return call(t, Ch32SpiTarget::kOpArm, p, out);
}
struct Rx {
  uint8_t pending;
  uint32_t bits;
  Bytes data;
};
static Rx readRx(Ch32SpiTarget &t) {
  Bytes out;
  Rx r = {};
  if (!ok(call(t, Ch32SpiTarget::kOpReadRx, {}, out)) || out.size() < 7) { CHECK(false); return r; }
  r = {out[0], getU32(out.data() + 1), Bytes(out.begin() + 7, out.end())};
  CHECK(getU16(out.data() + 5) == r.data.size());
  return r;
}
static Bytes status(Ch32SpiTarget &t) {
  Bytes out;
  CHECK(ok(call(t, Ch32SpiTarget::kOpStatus, {}, out)) && out.size() == 13);
  return out;
}

int main() {
  static PinTable pins(0xf0);
  static Ch32SpiTarget t(pins);
  Bytes out;
  const RoleAssignment roles[] = {{0, 1, 4}, {0, 2, 5}, {0, 3, 6}, {0, 4, 7}};
  CHECK(t.planCheck(roles, 3) == kRejectUnsupported);
  CHECK(t.planCheck(roles, 4) == 0);
  CHECK(t.planApply(roles, 4));

  // a route the core refuses: completed failed, state 0
  CH32RVSPISlave::fake_fail_begin = true;
  const Result refused = call(t, Ch32SpiTarget::kOpConfigure, {0, 0}, out);
  CHECK(refused.resolution == kResolutionCompleted && refused.detail == kOutcomeFailed);
  CHECK(status(t)[0] == 0);
  CHECK(rejectedAs(arm(t, 2, {}), kRejectUnavailable));

  // mode 3, LSB first: the pins and the mode go to the slave, the fill byte 0
  CHECK(ok(call(t, Ch32SpiTarget::kOpConfigure, {3, 1}, out)));
  CH32RVSPISlave &slave = t.hostSlave();
  CHECK(slave.begun && slave.sck == 4 && slave.mosi == 5 && slave.miso == 6 && slave.cs == 7);
  CHECK(slave.mode == SPI_MODE3 && slave.order == LSBFIRST);

  // armed: tx then 0 on MISO; MOSI kept; bits = bytes x 8
  CHECK(ok(arm(t, 4, {0xa5, 0x5a})));
  CHECK(rejectedAs(arm(t, 4, {}), kRejectUnavailable));   // one at a time
  CHECK(status(t)[3] == 1);
  slave.hostSelect(nullptr, 0);   // a CS with no clock: the arm keeps waiting
  CHECK(status(t)[3] == 1);
  const uint8_t mosi[] = {1, 2, 3, 4};
  uint8_t miso[4] = {};
  slave.hostSelect(mosi, 4, miso);
  CHECK(miso[0] == 0xa5 && miso[1] == 0x5a && miso[2] == 0 && miso[3] == 0);
  {
    const Rx r = readRx(t);
    CHECK(r.pending == 0 && r.bits == 32 && r.data == Bytes({1, 2, 3, 4}));
  }
  // nothing armed: MISO 0, counted with an error
  slave.hostSelect(mosi, 2, miso);
  CHECK(miso[0] == 0 && miso[1] == 0);
  {
    const Bytes s = status(t);
    CHECK(s[3] == 0 && s[4] == 0 && getU32(s.data() + 5) == 2 && getU32(s.data() + 9) == 1);
  }
  // past the armed length: data cut to it, bits as clocked, an error
  CHECK(ok(arm(t, 2, {})));
  slave.hostSelect(mosi, 4, miso);
  {
    const Rx r = readRx(t);
    CHECK(r.bits == 32 && r.data == Bytes({1, 2}));
    const Bytes s = status(t);
    CHECK(getU32(s.data() + 5) == 3 && getU32(s.data() + 9) == 2);
  }
  // the queue full: the fifth transfer dropped with an error
  for (int k = 0; k < 5; ++k) {
    CHECK(ok(arm(t, 1, {static_cast<uint8_t>(k)})));
    slave.hostSelect(mosi + k % 4, 1, miso);
    CHECK(miso[0] == k);
  }
  {
    const Bytes s = status(t);
    CHECK(s[4] == Ch32SpiTarget::kQueueDepth && getU32(s.data() + 5) == 8 && getU32(s.data() + 9) == 3);
    CHECK(readRx(t).pending == Ch32SpiTarget::kQueueDepth - 1);
  }
  // the form: length 0 / count past length malformed, past max_length unsupported
  CHECK(rejectedAs(arm(t, 0, {}), kRejectMalformed));
  CHECK(rejectedAs(arm(t, 1, {1, 2}), kRejectMalformed));
  CHECK(rejectedAs(arm(t, Ch32SpiTarget::kMaxFrame + 1, {}), kRejectUnsupported));
  // configure again: made anew
  CHECK(ok(call(t, Ch32SpiTarget::kOpConfigure, {0, 0}, out)));
  {
    const Bytes s = status(t);
    CHECK(s[0] == 1 && s[1] == 0 && s[3] == 0 && s[4] == 0 && getU32(s.data() + 5) == 0 && getU32(s.data() + 9) == 0);
  }

  // describe: all four modes, LSB first, cs_setup_ns
  {
    Bytes d(256);
    const size_t n = t.describe(d.data(), d.size());
    uint8_t modes = 0;
    uint32_t setup = 0;
    for (size_t at = 0; at + 3 <= n;) {
      const uint8_t tag = d[at];
      const uint16_t len = getU16(d.data() + at + 1);
      if (tag == spi::kTlvDescribeModes) modes = d[at + 3];
      if (tag == spi::kTlvDescribeCsSetupNs) setup = getU32(d.data() + at + 3);
      at += 3 + len;
    }
    CHECK(modes == 0x0f && setup == Ch32SpiTarget::kCsSetupNs);
  }

  // the plan released: the slave ended, state 0
  t.planRelease();
  CHECK(!slave.begun && status(t)[0] == 0);

  printf("ch32 spi-target: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
