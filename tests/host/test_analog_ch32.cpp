// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the analog capture (oep-spec oep-if-capture §3) in its CH32 build, on a fake of the core's CH32RVAdcStream
// (OEP_HOST_FAKE_CH32_ADC): the steps measured into rate_range, configure taking the fastest step at or below the rate
// (refusing one below the slowest), an immediate one-shot copied out half by half in pin order, a crossing with its
// pretrigger, and a half the DMA filled again before it was taken (a hole: not handed out).
#include <stdio.h>

#include <vector>

#include "OepAnalog.h"
#include "OepEndpoint.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace ana = reg::fixture_analog;

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
  PinTable pins{0xFFull};
  AnalogCapture cap{ep, 0x0Full};   // channels 0-3 are ADC inputs
  Bytes out;
  Rig() {
    ep.add(cap);
    cap.setPins(&pins, 7);
  }
  uint8_t plan(std::initializer_list<uint16_t> channels) {
    RoleAssignment roles[4];
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
};

static Bytes configureRequest(uint32_t rate, uint32_t samples) {
  Bytes p;
  tlv(p, kCrit | ana::kTlvConfigureMode, {ana::kModeOneShot});
  tlv(p, kCrit | ana::kTlvConfigureRate, u32(rate));
  tlv(p, ana::kTlvConfigureSamples, u32(samples));
  return p;
}
static const uint8_t *find(const Bytes &a, uint8_t tag) {
  for (size_t at = 0; at + kTlvHeader <= a.size(); at += kTlvHeader + getU16(a.data() + at + 1))
    if ((a[at] & 0x7f) == tag) return a.data() + at + kTlvHeader;
  return nullptr;
}
static Bytes readReq(uint32_t generation, uint64_t position, uint16_t max) {
  Bytes p(14);
  putU32(p.data(), generation);
  putU64(p.data() + 4, position);
  putU16(p.data() + 12, max);
  return p;
}

int main() {
  // the steps measured into rate_range: the slowest and the fastest
  {
    Rig rig;
    CHECK(rig.cap.measureRates());
    Bytes d(512);
    const size_t n = rig.cap.describe(d.data(), d.size());
    d.resize(n);
    const uint8_t *range = find(d, ana::kTlvDescribeRateRange);
    CHECK(range && getU32(range) == 60000 && getU32(range + 4) == 250000);
  }
  // configure: the fastest step at or below the rate a channel; two channels share it; below the slowest refused
  {
    Rig rig;
    CHECK(rig.plan({1}) == 0);
    CHECK(ok(rig.op(ana::kOpConfigure, configureRequest(120000, 100))));
    const uint8_t *rate = find(rig.out, ana::kTlvConfigureAnswerActualRate);
    CHECK(rate && getU32(rate) == 100000 && getU32(rate + 4) == 1);
    CHECK(rejectedAs(rig.op(ana::kOpConfigure, configureRequest(59999, 100)), kRejectUnsupported));
    CHECK(rig.plan({1, 2}) == 0);
    CHECK(ok(rig.op(ana::kOpConfigure, configureRequest(60000, 100))));   // 160000 / 2 > 60000, 100000 / 2 below
    rate = find(rig.out, ana::kTlvConfigureAnswerActualRate);
    CHECK(rate && getU32(rate) == 50000);
  }
  // an immediate one-shot: 100 frames of two channels, half after half (512 values = 256 frames a half)
  {
    Rig rig;
    CHECK(rig.plan({2, 3}) == 0);
    CHECK(ok(rig.op(ana::kOpConfigure, configureRequest(30000, 100))));
    CHECK(ok(rig.op(ana::kOpStart)));
    const uint32_t generation = getU32(rig.out.data());
    CH32RVAdcStream &adc = rig.cap.hostAdc();
    CHECK(adc.running && adc.count == 2 && adc.pins[0] == 2 && adc.pins[1] == 3);
    adc.hostConvert(511, 0, true);   // not a whole half yet: nothing taken
    rig.cap.poll();
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateCapturing);
    adc.hostConvert(1, 511);
    rig.cap.poll();
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateDone && getU64(rig.out.data() + 5) == 400);
    CHECK(!adc.begun);   // the ADC given back
    CHECK(ok(rig.op(ana::kOpRead, readReq(generation, 0, 1000))) && getU16(rig.out.data() + 9) == 400);
    CHECK(getU16(rig.out.data() + 11) == 0 && getU16(rig.out.data() + 13) == 1 && getU16(rig.out.data() + 11 + 398) == 199);
  }
  // a crossing up through 300 on one channel, pretrigger 10: the segment from 10 frames before it
  {
    Rig rig;
    CHECK(rig.plan({0}) == 0);
    Bytes p = configureRequest(60000, 50);
    tlv(p, kCrit | ana::kTlvConfigureTrigger, {ana::kTriggerCrossUp, 0, 0x2C, 0x01, 0, 0});   // 300
    tlv(p, kCrit | ana::kTlvConfigurePretrigger, u32(10));
    CHECK(ok(rig.op(ana::kOpConfigure, p)));
    CHECK(ok(rig.op(ana::kOpStart)));
    const uint32_t generation = getU32(rig.out.data());
    CH32RVAdcStream &adc = rig.cap.hostAdc();
    adc.hostConvert(512, 0, true);   // 0, 1, ... 511: up through 300 at frame 300
    rig.cap.poll();
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateDone && getU64(rig.out.data() + 5) == 100);
    CHECK(ok(rig.op(ana::kOpRead, readReq(generation, 0, 1000))) && getU16(rig.out.data() + 9) == 100);
    CHECK(getU16(rig.out.data() + 11) == 290 && getU16(rig.out.data() + 11 + 98) == 339);
  }
  // a half filled again before it was taken: the immediate segment has a hole, not handed out (state 6, error 2)
  {
    Rig rig;
    CHECK(rig.plan({0}) == 0);
    CHECK(ok(rig.op(ana::kOpConfigure, configureRequest(60000, 2000))));
    CHECK(ok(rig.op(ana::kOpStart)));
    rig.cap.hostAdc().hostConvert(3 * 512, 7);   // three halves, poll none: the first one overwritten
    rig.cap.poll();
    rig.cap.hostAdc().hostConvert(2 * 512, 7);
    rig.cap.poll();
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateError && rig.out[21] == ana::kErrorStorage);
  }
  // a pin the core refuses: start fails, state 6
  {
    Rig rig;
    CHECK(rig.plan({3}) == 0);
    CHECK(ok(rig.op(ana::kOpConfigure, configureRequest(60000, 100))));
    g_ch32_no_adc_pin = 3;
    const Result r = rig.op(ana::kOpStart);
    CHECK(r.resolution == kResolutionCompleted && r.detail == kOutcomeFailed);
    g_ch32_no_adc_pin = -1;
  }

  printf("analog ch32: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
