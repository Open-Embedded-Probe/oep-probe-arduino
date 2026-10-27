// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the logic capture (oep-spec oep-if-capture §3) on a CH32 (Ch32LogicCapture) with a fake of the core's
// CH32RVLogicCapture (OEP_HOST_FAKE_CH32_LOGIC): the plan on one port's pins 0-15, the rate as F_CPU / a whole divider,
// an immediate one-shot packed a byte a sample, a rising edge with its pretrigger, a level with when 0 and 1, force, and
// a half overwritten before it was taken (a hole: state 6).
#include <stdio.h>

#include <vector>

#include "OepCh32Logic.h"
#include "OepEndpoint.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace cap = reg::fixture_logic;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

class NullStream final : public Stream {
 public:
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  size_t write(uint8_t) override { return 1; }
};

static constexpr uint8_t kCrit = 0x80;
static void tlv(Bytes &p, uint8_t tag, const Bytes &v) {
  p.push_back(tag);
  p.push_back(static_cast<uint8_t>(v.size()));
  p.push_back(static_cast<uint8_t>(v.size() >> 8));
  p.insert(p.end(), v.begin(), v.end());
}
static Bytes u32(uint32_t v) { Bytes b(4); putU32(b.data(), v); return b; }
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static bool rejectedAs(const Result &r, uint8_t reason) { return r.resolution == kResolutionRejected && r.detail == reason; }

struct Rig {
  NullStream stream;
  uint8_t rx[600], tx[600];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kVendorBulk, 0};
  PinTable pins{~0ull};
  Ch32LogicCapture cap{ep, pins};
  Bytes out;
  Rig() { ep.add(cap); }
  uint8_t plan(std::initializer_list<uint16_t> channels) {
    RoleAssignment roles[8];
    size_t n = 0;
    for (uint16_t ch : channels) { roles[n] = {1, static_cast<uint8_t>(n), ch}; ++n; }
    const uint8_t r = cap.planCheck(roles, n);
    if (r == 0) cap.planApply(roles, n);
    return r;
  }
  Result op(uint8_t code, const Bytes &p = {}) {
    out.assign(600, 0);
    const Result r = cap.handle(code, p.data(), p.size(), out.data(), out.size());
    out.resize(r.length);
    return r;
  }
  Bytes segment(uint32_t generation, uint32_t samples) {
    Bytes all;
    while (all.size() < samples) {
      Bytes p(14);
      putU32(p.data(), generation);
      putU64(p.data() + 4, all.size());
      putU16(p.data() + 12, 500);
      if (!ok(op(cap::kOpRead, p)) || getU16(out.data() + 9) == 0) break;
      all.insert(all.end(), out.begin() + 11, out.end());
    }
    return all;
  }
};

static Bytes request(uint32_t rate, uint32_t samples, const Bytes &trigger = {}, uint32_t pre = 0) {
  Bytes p;
  tlv(p, kCrit | cap::kTlvConfigureMode, {cap::kModeOneShot});
  tlv(p, kCrit | cap::kTlvConfigureRate, u32(rate));
  tlv(p, cap::kTlvConfigureSamples, u32(samples));
  if (!trigger.empty()) tlv(p, kCrit | cap::kTlvConfigureTrigger, trigger);
  if (pre) tlv(p, kCrit | cap::kTlvConfigurePretrigger, u32(pre));
  return p;
}
static Bytes trig(uint8_t type, uint8_t role, uint32_t value) {
  Bytes t = {type, role};
  const Bytes v = u32(value);
  t.insert(t.end(), v.begin(), v.end());
  return t;
}

int main() {
  // the plan: one port's pins 0-15 (port x 24 + pin)
  {
    Rig rig;
    CHECK(rig.plan({24 + 3, 24 + 15}) == 0);
    CHECK(rig.plan({16}) == kRejectUnsupported);              // PA16: not sampled
    CHECK(rig.plan({3, 24 + 3}) == kRejectUnsupported);        // two ports
  }
  // the rate: F_CPU / a whole divider, at most kMaxHz; an immediate one-shot packed a byte a sample (PB3 on bit 0, PB5 on 1)
  {
    Rig rig;
    CHECK(rig.plan({24 + 3, 24 + 5}) == 0);
    CHECK(rejectedAs(rig.op(cap::kOpConfigure, request(Ch32LogicCapture::kMaxHz + 1000, 10)), kRejectUnsupported));
    CHECK(ok(rig.op(cap::kOpConfigure, request(100000, 3000))));
    CHECK(ok(rig.op(cap::kOpStart)));
    const uint32_t generation = getU32(rig.out.data());
    CH32RVLogicCapture &l = rig.cap.hostLogic();
    CHECK(l.running && l.port == PORT_B && l.sampleRate() == 100000);
    for (uint32_t h = 0; h < 3; ++h) {   // a half at a time, taken before the next
      l.hostSample(Ch32LogicCapture::kHalf, [h](uint32_t i) {
        const uint32_t k = h * Ch32LogicCapture::kHalf + i;
        return static_cast<uint16_t>(((k & 1) << 3) | ((k >> 1 & 1) << 5));
      });
      rig.cap.poll();
    }
    CHECK(ok(rig.op(cap::kOpStatus)) && rig.out[0] == cap::kStateDone);
    const Bytes s = rig.segment(generation, 3000);
    CHECK(s.size() == 3000 && s[0] == 0 && s[1] == 1 && s[2] == 2 && s[3] == 3 && s[2999] == 3);
  }
  // a rising edge on role 1 with pretrigger 10: the segment from 10 before the edge
  {
    Rig rig;
    CHECK(rig.plan({0, 1}) == 0);
    CHECK(ok(rig.op(cap::kOpConfigure, request(100000, 40, trig(cap::kTriggerEdge, 1, 0), 10))));
    CHECK(ok(rig.op(cap::kOpStart)));
    const uint32_t generation = getU32(rig.out.data());
    CHECK(ok(rig.op(cap::kOpStatus)) && rig.out[0] == cap::kStateWaiting);
    rig.cap.hostLogic().hostSample(2 * Ch32LogicCapture::kHalf, [](uint32_t i) { return static_cast<uint16_t>(i >= 500 ? 2 : 0); });
    rig.cap.poll();
    CHECK(ok(rig.op(cap::kOpStatus)) && rig.out[0] == cap::kStateDone);
    const Bytes s = rig.segment(generation, 40);
    CHECK(s.size() == 40 && s[9] == 0 && s[10] == 2 && s[39] == 2);
  }
  // a level high on role 0: when 0 waits for the edge into it, the line already high at arming
  {
    Rig rig;
    CHECK(rig.plan({7}) == 0);
    CHECK(ok(rig.op(cap::kOpConfigure, request(100000, 20, trig(cap::kTriggerLevel, 0, 1), 5))));
    CHECK(ok(rig.op(cap::kOpStart)));
    const uint32_t generation = getU32(rig.out.data());
    rig.cap.hostLogic().hostSample(2 * Ch32LogicCapture::kHalf,   // high, low from 300, high again from 600
                                   [](uint32_t i) { return static_cast<uint16_t>(i < 300 || i >= 600 ? 1u << 7 : 0); });
    rig.cap.poll();
    const Bytes s = rig.segment(generation, 20);
    CHECK(s.size() == 20 && s[4] == 0 && s[5] == 1);
  }
  // force: waiting for an edge that never comes, the trigger at the next sample looked at
  {
    Rig rig;
    CHECK(rig.plan({2}) == 0);
    CHECK(ok(rig.op(cap::kOpConfigure, request(100000, 100, trig(cap::kTriggerEdge, 0, 0), 10))));
    CHECK(ok(rig.op(cap::kOpStart)));
    CHECK(ok(rig.op(cap::kOpForce)));
    rig.cap.hostLogic().hostSample(2 * Ch32LogicCapture::kHalf, [](uint32_t) { return static_cast<uint16_t>(0); });
    rig.cap.poll();
    CHECK(ok(rig.op(cap::kOpSegments, {0, 0, 0, 0})) && rig.out[1] == 1 && getU32(rig.out.data() + 2 + 28) == 10);
  }
  // a half filled again before it was taken: a hole, state 6 error 2
  {
    Rig rig;
    CHECK(rig.plan({2}) == 0);
    CHECK(ok(rig.op(cap::kOpConfigure, request(100000, 5000))));
    CHECK(ok(rig.op(cap::kOpStart)));
    rig.cap.hostLogic().hostSample(3 * Ch32LogicCapture::kHalf, [](uint32_t) { return static_cast<uint16_t>(0); });
    rig.cap.poll();
    CHECK(ok(rig.op(cap::kOpStatus)) && rig.out[0] == cap::kStateError && rig.out[21] == cap::kErrorStorage);
  }

  printf("logic ch32: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
