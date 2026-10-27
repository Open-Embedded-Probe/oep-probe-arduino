// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.fixture.pwm / counter / dac (oep-spec oep-if-fixture §7-§9) on fakes (OEP_HOST_FAKE_PWM / COUNTER /
// DAC): the plans (roles, a counter sharing a pin with a pwm output, a DAC only on its pins), set's checks before
// anything changes, what set and status answer, measure's window and counts, the plans' release.
#include <stdio.h>

#include <vector>

#include "OepCounter.h"
#include "OepDac.h"
#include "OepPwm.h"

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

template <typename T> static Result call(T &t, uint8_t op, const Bytes &p, Bytes &out) {
  out.assign(256, 0);
  const Result r = t.handle(op, p.data(), p.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static bool rejectedAs(const Result &r, uint8_t reason) { return r.resolution == kResolutionRejected && r.detail == reason; }
static void pwmEntry(Bytes &p, uint8_t role, uint32_t hz, uint16_t duty) {
  const size_t at = p.size();
  p.resize(at + 7);
  p[at] = role;
  putU32(p.data() + at + 1, hz);
  putU16(p.data() + at + 5, duty);
}

int main() {
  static PinTable pins(0xffull);
  Bytes out;
  // pwm: roles on distinct pins; set checks every entry first; the output made; hz 0 stops
  {
    static FixturePwm pwm(pins);
    const RoleAssignment same[] = {{0, 0, 3}, {0, 1, 3}};
    CHECK(pwm.planCheck(same, 2) == kRejectUnsupported);
    const RoleAssignment roles[] = {{0, 0, 3}, {0, 2, 5}};
    CHECK(pwm.planCheck(roles, 2) == 0 && pwm.planApply(roles, 2));
    Bytes p = {2};
    pwmEntry(p, 0, 1000, 32767);
    pwmEntry(p, 1, 1000, 1);   // role 1 not planned: unavailable, nothing changed
    CHECK(rejectedAs(call(pwm, FixturePwm::kOpSet, p, out), kRejectUnavailable));
    Bytes q = {1};
    pwmEntry(q, 0, FixturePwm::kMaxHz + 1, 1);
    CHECK(rejectedAs(call(pwm, FixturePwm::kOpSet, q, out), kRejectUnsupported));
    Bytes r = {2};
    pwmEntry(r, 0, 1000, 32767);
    pwmEntry(r, 2, 0, 0);
    CHECK(ok(call(pwm, FixturePwm::kOpSet, r, out)) && out.size() == 22);
    CHECK(out[0] == 0 && getU32(out.data() + 1) == 1000 && getU32(out.data() + 5) == 1 && getU16(out.data() + 9) == 32767);
    CHECK(out[11] == 2 && getU32(out.data() + 12) == 0 && getU32(out.data() + 16) == 1);
    CHECK(ok(call(pwm, FixturePwm::kOpStatus, {}, out)) && out[0] == 2 && getU32(out.data() + 2) == 1000);
    CHECK(pins.owner(3) == FixturePwm::kOwnerId);
    // the counter shares the pwm's pin (it only listens); measure counts the edges in its window
    static FixtureCounter counter(pins);
    const RoleAssignment c[] = {{0, 0, 3}, {0, 1, 6}};
    CHECK(counter.planCheck(c, 2) == 0 && counter.planApply(c, 2));
    Bytes m(5);
    putU32(m.data(), 0);
    CHECK(rejectedAs(call(counter, FixtureCounter::kOpMeasure, m, out), kRejectMalformed));   // gate 0
    putU32(m.data(), FixtureCounter::kMaxGateUs + 1);
    CHECK(rejectedAs(call(counter, FixtureCounter::kOpMeasure, m, out), kRejectUnsupported));
    putU32(m.data(), 100000);
    m[4] = reg::fixture_counter::kEdgeRising;
    g_on_wait = [] { for (int i = 0; i < 100; ++i) FixtureCounter::hostEdge(0); };
    for (int i = 0; i < 100; ++i) FixtureCounter::hostEdge(0);   // counted from the start of the measure: none before
    CHECK(ok(call(counter, FixtureCounter::kOpMeasure, m, out)) && out.size() == 9 + 10);
    CHECK(getU64(out.data()) == 100000000ull && out[8] == 2 && out[9] == 0 && getU32(out.data() + 10) == 100 && out[14] == 1 &&
          getU32(out.data() + 15) == 0);
    g_on_wait = nullptr;
    pwm.planRelease();
    CHECK(pins.owner(3) == 0);
  }
  // the counter without a plan: unavailable
  {
    static FixtureCounter counter(pins);
    Bytes m(5);
    putU32(m.data(), 1000);
    CHECK(rejectedAs(call(counter, FixtureCounter::kOpMeasure, m, out), kRejectUnavailable));
  }
  // dac: only its pins; set's value at most 2^bits - 1; status the value and on
  {
    static FixtureDac dac(pins, (1ull << 4) | (1ull << 5));
    const RoleAssignment wrong[] = {{0, 0, 3}};
    CHECK(dac.planCheck(wrong, 1) == kRejectUnsupported);
    const RoleAssignment roles[] = {{0, 0, 4}, {0, 1, 5}};
    CHECK(dac.planCheck(roles, 2) == 0 && dac.planApply(roles, 2));
    Bytes p = {1, 0, 0, 0, 0, 0};
    putU32(p.data() + 2, (1u << FixtureDac::kBits));
    CHECK(rejectedAs(call(dac, FixtureDac::kOpSet, p, out), kRejectUnsupported));
    putU32(p.data() + 2, 100);
    CHECK(ok(call(dac, FixtureDac::kOpSet, p, out)) && FixtureDac::host_value[4] == 100);
    CHECK(ok(call(dac, FixtureDac::kOpStatus, {}, out)) && out[0] == 2 && getU32(out.data() + 2) == 100 && out[6] == 1 && out[12] == 0);
    dac.planRelease();
    CHECK(FixtureDac::host_value[4] == 0xFFFFFFFFu && pins.owner(4) == 0);
  }
  printf("fixture out: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
