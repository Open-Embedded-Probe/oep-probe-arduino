// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.fixture.spi-target (oep-spec oep-if-fixture §4) on a fake of the ESP-IDF spi_slave driver - a transfer
// nobody armed counts in transactions and errors, an armed one is queued, a CS frame with no clock counts nothing and
// leaves the arm waiting, one arm at a time, over length counts an error.
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
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
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

int main() {
  static PinTable pins((1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7));
  static P4SpiTarget t(pins);
  const RoleAssignment roles[] = {{0, P4SpiTarget::kRoleSck, 4}, {0, P4SpiTarget::kRoleMosi, 5},
                                  {0, P4SpiTarget::kRoleMiso, 6}, {0, P4SpiTarget::kRoleCs, 7}};
  CHECK(t.planCheck(roles, 4) == 0);
  CHECK(t.planApply(roles, 4));
  Bytes out;
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

  t.planRelease();
  printf("spi-target: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
