// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the analog capture (oep-spec oep-if-capture §1.2, §3.2, §3.3) in its RP2 build, on fakes of the RP2's ADC
// and DMA (OEP_HOST_FAKE_RP2_ADC): configure's refusals (core §4.3), its TLVs checked the same with or without bit 7
// (core §2.3), the answer and describe, the plan's refusals, and what start / stop / plan_release leave to read.
#include <stdio.h>

#include <algorithm>
#include <vector>

#include "OepAnalog.h"
#include "OepEndpoint.h"
#include <hardware/adc.h>
#include <hardware/dma.h>

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

// An analog capture on the RP2's four ADC inputs (GPIO26..29), the pin table its plan claims from.
struct Rig {
  NullStream stream;
  uint8_t rx[600], tx[600];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kVendorBulk, 0};
  PinTable pins{(0xFull << 26) | 0xFFull};
  AnalogCapture cap{ep, 0xFull << 26};
  Bytes out;
  Rig() {
    g_fake_dma = FakeDma{};
    ep.add(cap);   // fn 1
    cap.setPins(&pins, 1);
  }
  // roles 0..n-1 on the given inputs
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

// mode 1, rate and samples: the TLVs a one-shot requires (capture §3.3); `skip` left out
static Bytes configureRequest(uint32_t rate, uint32_t samples = 1024, uint8_t skip = 0) {
  Bytes p;
  if (skip != ana::kTlvConfigureMode) tlv(p, kCrit | ana::kTlvConfigureMode, {ana::kModeOneShot});
  if (skip != ana::kTlvConfigureRate) tlv(p, kCrit | ana::kTlvConfigureRate, u32(rate));
  if (skip != ana::kTlvConfigureSamples) tlv(p, ana::kTlvConfigureSamples, u32(samples));
  return p;
}

static bool hasTag(const Bytes &a, uint8_t tag) {
  for (size_t at = 0; at + kTlvHeader <= a.size(); at += kTlvHeader + getU16(a.data() + at + 1))
    if (a[at] == tag) return true;
  return false;
}

// core §2.3 (capture §3.3): every configure TLV is checked the same with or without bit 7. A value the probe cannot
// honour refuses the configure unsupported with the tag as received (bit 7 as sent) - nothing is ignored; a length
// other than the definition is malformed; an unknown non-critical tag is passed over.
static void testValuesRefused() {
  Rig rig;
  CHECK(rig.plan({26}) == 0);
  for (const uint8_t bit : {uint8_t(0), kCrit}) {
    Bytes p = configureRequest(10000, 1024, ana::kTlvConfigureMode);
    tlv(p, bit | ana::kTlvConfigureMode, {ana::kModeRepeat});
    Result r = rig.op(ana::kOpConfigure, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigureMode));
    p = configureRequest(10000);
    tlv(p, bit | ana::kTlvConfigureTrigger, {1, 0, 0, 0, 0, 0});   // a logic type
    r = rig.op(ana::kOpQuery, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigureTrigger));
    p = configureRequest(10000, 100);
    tlv(p, ana::kTlvConfigureTrigger, {3, 0, 0, 8, 0, 0});
    tlv(p, bit | ana::kTlvConfigurePretrigger, u32(100));         // no room left in the segment
    r = rig.op(ana::kOpConfigure, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigurePretrigger));
    p = configureRequest(10000, 100);
    tlv(p, ana::kTlvConfigureTrigger, {3, 0, 0, 8, 0, 0});
    tlv(p, bit | ana::kTlvConfigurePretrigger, u32(99));          // just inside it (the RP2's room is 1)
    CHECK(ok(rig.op(ana::kOpQuery, p)));
    p = configureRequest(10000, 100);
    tlv(p, bit | ana::kTlvConfigurePretrigger, u32(10));          // without a trigger: whatever the value
    r = rig.op(ana::kOpConfigure, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigurePretrigger));
    p = configureRequest(10000);
    tlv(p, bit | ana::kTlvConfigureSegments, u32(2));             // segments in one-shot: whatever the value
    r = rig.op(ana::kOpQuery, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigureSegments));
    p = configureRequest(10000);
    tlv(p, bit | ana::kTlvConfigureFrontend, {0, 9});              // a frontend it does not declare
    r = rig.op(ana::kOpConfigure, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigureFrontend));
    p = configureRequest(10000);
    tlv(p, bit | ana::kTlvConfigureFrontend, {1, 0});              // a role not in the plan
    r = rig.op(ana::kOpConfigure, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigureFrontend));
    // a required TLV missing, samples 0: malformed (capture §3.3)
    for (const uint8_t tag : {ana::kTlvConfigureMode, ana::kTlvConfigureRate, ana::kTlvConfigureSamples})
      CHECK(rejectedAs(rig.op(ana::kOpQuery, configureRequest(10000, 1024, tag)), kRejectMalformed));
    p = configureRequest(10000, 1024, ana::kTlvConfigureSamples);
    tlv(p, bit | ana::kTlvConfigureSamples, u32(0));
    CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectMalformed));
    // a length other than the definition: malformed, longer or shorter, any tag (none is ignored)
    for (const uint8_t tag : {ana::kTlvConfigureMode, ana::kTlvConfigureRate, ana::kTlvConfigureSamples,
                              ana::kTlvConfigureSegments, ana::kTlvConfigureTrigger, ana::kTlvConfigurePretrigger,
                              ana::kTlvConfigureFrontend}) {
      const size_t size = tag == ana::kTlvConfigureMode ? 1 : tag == ana::kTlvConfigureTrigger ? 6
                        : tag == ana::kTlvConfigureFrontend ? 2 : 4;
      for (const size_t len : {size - 1, size + 1}) {
        p = configureRequest(10000, 1024, tag);
        tlv(p, bit | tag, Bytes(len, 1));
        CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectMalformed));
      }
    }
  }
  // an unknown non-critical tag is passed over; the answer has no ignored (0x7F), timing (0x54) or rate_accuracy (0x5A),
  // and keeps skew, scale, blocking_ms, frontend_used and reference
  Bytes p = configureRequest(10000);
  tlv(p, 0x61, {1, 2, 3});
  tlv(p, 0x7F, {ana::kTlvConfigureSamples});   // no longer a special tag: unknown, non-critical
  CHECK(ok(rig.op(ana::kOpConfigure, p)));
  CHECK(!hasTag(rig.out, 0x7F) && !hasTag(rig.out, 0x54) && !hasTag(rig.out, 0x5A));
  CHECK(hasTag(rig.out, ana::kTlvConfigureAnswerActualSamples) && !hasTag(rig.out, ana::kTlvConfigureAnswerActualSegments));
  CHECK(hasTag(rig.out, ana::kTlvConfigureAnswerSkew) && hasTag(rig.out, ana::kTlvConfigureAnswerScale) &&
        hasTag(rig.out, ana::kTlvConfigureAnswerBlockingMs) && hasTag(rig.out, ana::kTlvConfigureAnswerFrontendUsed) &&
        hasTag(rig.out, ana::kTlvConfigureAnswerReference));
}

// capture §3.5: mode is mode(u8) max_samples(u32) max_segments(u32); channels is max(u8); rate_limit, max_read,
// segment_ring and frontend_shared (0x43, 0x47, 0x48, 0x49) are not declared.
static void testDescribe() {
  Rig rig;
  Bytes d(1024);
  d.resize(rig.cap.describe(d.data(), d.size()));
  CHECK(!d.empty());
  int modes = 0, frontends = 0;
  bool channels = false;
  for (size_t at = 0; at + kTlvHeader <= d.size(); at += kTlvHeader + getU16(d.data() + at + 1)) {
    const uint8_t tag = d[at];
    const uint16_t len = getU16(d.data() + at + 1);
    CHECK(len <= 512 - 9);   // core §7.3: fits the smallest max_frame (512) with the header 5, more 1, TLV 3
    if (tag == ana::kTlvDescribeMode) { ++modes; CHECK(len == 9 && d[at + kTlvHeader] == ana::kModeOneShot); }
    if (tag == ana::kTlvDescribeChannels) { channels = true; CHECK(len == 1 && d[at + kTlvHeader] == AnalogCapture::kMaxChannels); }
    if (tag == ana::kTlvDescribeFrontend) ++frontends;
    CHECK(tag != 0x42 && tag != 0x43 && tag != 0x47 && tag != 0x48 && tag != 0x49);
  }
  CHECK(modes == 1 && channels && frontends == 1);
}

// core §4.3: malformed before unsupported before unavailable, over the whole request.
static void testConfigureOrder() {
  Rig rig;   // no plan yet
  Bytes p = configureRequest(10000);
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectUnavailable));   // no plan: cause 6
  tlv(p, kCrit | 0x60, {1});                                              // an unknown critical tag
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectUnsupported) && rig.out[0] == (kCrit | 0x60));
  tlv(p, ana::kTlvConfigureSegments, {1, 0});                             // and a short segments
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectMalformed));
  p = configureRequest(10000);
  tlv(p, kCrit | ana::kTlvConfigureMode, {});                             // mode twice: the first is used (core §2.3)
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectUnavailable));
  p = configureRequest(10000);
  tlv(p, kCrit | ana::kTlvConfigureTrigger, {3, 3, 0, 8, 0, 0});          // role 3 without a plan: the plan's refusal
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectUnavailable));
  CHECK(rig.plan({26}) == 0);   // and with a plan without role 3: unavailable cause 6 (capture §3.3)
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectUnavailable) && rig.out == (Bytes{0x01, 1, 0, 6}));
}

static bool findTlv(const Bytes &a, uint8_t tag, Bytes &v) {
  for (size_t at = 0; at + kTlvHeader <= a.size(); at += kTlvHeader + getU16(a.data() + at + 1))
    if (a[at] == tag) {
      v.assign(a.begin() + at + kTlvHeader, a.begin() + at + kTlvHeader + getU16(a.data() + at + 1));
      return true;
    }
  return false;
}

// rate (capture §3.3): outside the declared rate_range unsupported, tag 0x42 as received; inside it, the nearest the
// shared ADC allows for the channel count (500 kS/s in all on the RP2; not declared).
static void testRateRange() {
  Rig rig;
  CHECK(rig.plan({26, 27}) == 0);
  for (const uint32_t rate : {uint32_t(1), uint32_t(48000000 / 65536), uint32_t(500001), uint32_t(10000000)}) {
    for (const uint8_t bit : {uint8_t(0), kCrit}) {
      Bytes p = configureRequest(rate, 1024, ana::kTlvConfigureRate);
      tlv(p, bit | ana::kTlvConfigureRate, u32(rate));
      CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectUnsupported) && rig.out == Bytes{uint8_t(bit | ana::kTlvConfigureRate)});
    }
  }
  CHECK(ok(rig.op(ana::kOpConfigure, configureRequest(400000))));   // in range, over two channels' limit (250 kHz)
  Bytes rate;
  CHECK(findTlv(rig.out, ana::kTlvConfigureAnswerActualRate, rate) && rate.size() == 8);
  CHECK(getU32(rate.data()) == 48000000 && getU32(rate.data() + 4) == 96 * 2);   // 250 kHz a channel
}

// query answers what configure would (capture §3.2): the frame's order on the RP2 follows the inputs, not the roles.
static void testQueryAsConfigure() {
  Rig rig;
  CHECK(rig.plan({29, 26, 28}) == 0);   // roles 0, 1, 2 on GPIO29, 26, 28: slots are roles 1, 2, 0
  Bytes p = configureRequest(10000, 100);
  CHECK(ok(rig.op(ana::kOpQuery, p)));
  const Bytes queried = rig.out;
  CHECK(ok(rig.op(ana::kOpConfigure, p)));
  CHECK(queried == rig.out);
  Bytes layout;
  CHECK(findTlv(rig.out, ana::kTlvConfigureAnswerLayout, layout) && layout == (Bytes{16, 0, 12, 3, 1, 2, 0}));
}

// An analog plan on a channel whose idle is an output (core §8, capture §1.2): unavailable cause 5 and the channel;
// nothing changes.
static void testOutputIdle() {
  Rig rig;
  CHECK(rig.pins.setIdle(27, PinTable::kIdleOutputHigh, false));
  const RoleAssignment roles[] = {{1, 0, 26}, {1, 1, 27}};
  const uint16_t fns[] = {1};
  CHECK(rig.ep.replacePlan(roles, 2, fns, 1) == kRejectUnavailable);
  uint8_t out[32];
  const Result r = rig.ep.planUnavailable(out, sizeof out);
  CHECK(rejectedAs(r, kRejectUnavailable) && Bytes(out, out + r.length) == (Bytes{0x01, 1, 0, 5, 0x02, 2, 0, 27, 0}));
  RoleAssignment now[2];
  CHECK(rig.ep.plan(now, 2) == 0 && rig.pins.owner(26) == 0);
  CHECK(rig.pins.setIdle(27, PinTable::kIdlePullUp, false));   // an input idle: fine
  CHECK(rig.ep.replacePlan(roles, 2, fns, 1) == 0);
}

static Bytes readReq(uint32_t generation, uint64_t position, uint32_t max) {
  Bytes p(16);
  putU32(p.data(), generation);
  putU64(p.data() + 4, position);
  putU32(p.data() + 12, max);
  return p;
}
// The DMA writes `count` values of `value` (of the transfer the capture set up).
static void convert(uint32_t count, uint16_t value) {
  for (uint32_t i = 0; i < count && g_fake_dma.written < g_fake_dma.count; ++i, ++g_fake_dma.written)   // a ring wraps
    g_fake_dma.to[g_fake_dma.ring ? g_fake_dma.written % (32768 / 2) : g_fake_dma.written] = value;
}

// capture §2.2: a triggered segment the DMA ring came round over before it was taken out (poll late) has a hole: not
// handed out - no segment, read empty, write_pos 0 - and the track stops in state 6 with error 2, status flags bit0.
static void testHole() {
  Rig rig;
  CHECK(rig.plan({26}) == 0);
  Bytes p = configureRequest(10000, 100);
  tlv(p, kCrit | ana::kTlvConfigureTrigger, {ana::kTriggerCrossUp, 0, 0xD0, 0x07, 0, 0});   // 2000
  tlv(p, kCrit | ana::kTlvConfigurePretrigger, u32(10));
  CHECK(ok(rig.op(ana::kOpConfigure, p)));
  CHECK(ok(rig.op(ana::kOpStart)));
  const uint32_t generation = getU32(rig.out.data() + 4);
  convert(20, 100);
  convert(30, 4000);
  rig.cap.poll();                 // the crossing at frame 20: the segment runs 10 - 109
  convert(20000, 4000);           // the ring (16384 values) came round over it before the next look
  rig.cap.poll();
  CHECK(ok(rig.op(ana::kOpStatus)) && rig.out.size() == 22 && rig.out[0] == ana::kStateError &&
        getU32(rig.out.data() + 1) == 0 && getU64(rig.out.data() + 5) == 0 && (rig.out[13] & ana::kStatusFlagDropped) &&
        rig.out[21] == ana::kErrorStorage);
  CHECK(ok(rig.op(ana::kOpSegments, {0, 0, 0, 0})) && rig.out[1] == 0);
  CHECK(ok(rig.op(ana::kOpRead, readReq(generation, 0, 1000))) && getU32(rig.out.data() + 9) == 0);
  CHECK(ok(rig.op(ana::kOpStart)));   // from state 6: a new generation
  CHECK(ok(rig.op(ana::kOpStatus)) && rig.out.size() == 18);
}

// The plan released or replaced (capture §3.2): state 0, the data and the segment gone - read is empty (it read past
// the buffer after a plan with more channels), status counts nothing.
static void testPlanReleaseForgets() {
  Rig rig;
  CHECK(rig.plan({26}) == 0);
  CHECK(ok(rig.op(ana::kOpConfigure, configureRequest(10000, 100))));
  CHECK(ok(rig.op(ana::kOpStart)));
  const uint32_t generation = getU32(rig.out.data() + 4);
  convert(100, 0x123);
  rig.cap.poll();
  CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateDone && getU64(rig.out.data() + 5) == 200);
  CHECK(ok(rig.op(ana::kOpRead, readReq(generation, 0, 1000))) && getU32(rig.out.data() + 9) == 200);
  CHECK(rig.plan({26, 27, 28, 29}) == 0);
  CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateUnconfigured && getU32(rig.out.data() + 1) == 0 &&
        getU64(rig.out.data() + 5) == 0);
  CHECK(ok(rig.op(ana::kOpRead, readReq(generation, 0, 1000))) && rig.out.size() == 13 && getU32(rig.out.data() + 9) == 0);
  CHECK(ok(rig.op(ana::kOpSegments, {0, 0, 0, 0})) && rig.out.size() >= 2 && rig.out[1] == 0);
  rig.cap.planRelease();
  CHECK(ok(rig.op(ana::kOpRead, readReq(generation, 0, 1000))) && getU32(rig.out.data() + 9) == 0);
}

// stop (capture §3.2): capturing (state 3) -> 1 with the segment cut short (flags bit1) holding what came in, readable;
// waiting for the trigger (state 2) -> 1 with nothing; a capture already complete stays complete (state 4).
static void testStop() {
  {   // immediate: 30 of 100 frames came
    Rig rig;
    CHECK(rig.plan({26}) == 0);
    CHECK(ok(rig.op(ana::kOpConfigure, configureRequest(10000, 100))));
    CHECK(ok(rig.op(ana::kOpStart)));
    const uint32_t generation = getU32(rig.out.data() + 4);
    convert(30, 0x321);
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateCapturing);
    CHECK(ok(rig.op(ana::kOpStop)));
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateConfigured && getU32(rig.out.data() + 1) == 1 &&
          getU64(rig.out.data() + 5) == 60);
    CHECK(ok(rig.op(ana::kOpSegments, {0, 0, 0, 0})) && rig.out.size() >= 2 + 37 && rig.out[1] == 1);
    CHECK(getU32(rig.out.data() + 2 + 12) == 30 && (rig.out[2 + 32] & ana::kSegmentFlagShort));
    CHECK(ok(rig.op(ana::kOpRead, readReq(generation, 0, 1000))) && getU32(rig.out.data() + 9) == 60 &&
          getU16(rig.out.data() + 13) == 0x321);
    CHECK(ok(rig.op(ana::kOpStart)));   // from state 1: the next generation, the segment gone
    CHECK(ok(rig.op(ana::kOpStatus)) && getU32(rig.out.data() + 1) == 0);
  }
  {   // a threshold crossed up at frame 20 (pretrigger 10), 40 of 100 frames in when stopped
    Rig rig;
    CHECK(rig.plan({26}) == 0);
    Bytes p = configureRequest(10000, 100);
    tlv(p, kCrit | ana::kTlvConfigureTrigger, {ana::kTriggerCrossUp, 0, 0xD0, 0x07, 0, 0});   // 2000
    tlv(p, kCrit | ana::kTlvConfigurePretrigger, u32(10));
    CHECK(ok(rig.op(ana::kOpConfigure, p)));
    CHECK(ok(rig.op(ana::kOpStart)));
    const uint32_t generation = getU32(rig.out.data() + 4);
    convert(20, 100);
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateWaiting);
    convert(30, 4000);
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateCapturing);
    CHECK(ok(rig.op(ana::kOpStop)));
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateConfigured && getU32(rig.out.data() + 1) == 1);
    CHECK(ok(rig.op(ana::kOpSegments, {0, 0, 0, 0})) && rig.out.size() >= 2 + 37 && rig.out[1] == 1);
    CHECK(getU32(rig.out.data() + 2 + 12) == 40 && getU32(rig.out.data() + 2 + 28) == 10 &&
          (rig.out[2 + 32] & ana::kSegmentFlagShort));
    CHECK(ok(rig.op(ana::kOpRead, readReq(generation, 0, 1000))) && getU32(rig.out.data() + 9) == 80);
    CHECK(getU16(rig.out.data() + 13 + 2 * 9) == 100 && getU16(rig.out.data() + 13 + 2 * 10) == 4000);
    // waiting, never crossed: state 1, nothing
    CHECK(ok(rig.op(ana::kOpStart)));
    convert(20, 100);
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateWaiting);
    CHECK(ok(rig.op(ana::kOpStop)));
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateConfigured && getU32(rig.out.data() + 1) == 0 &&
          getU64(rig.out.data() + 5) == 0);
  }
  {   // complete before the stop is looked at: complete
    Rig rig;
    CHECK(rig.plan({26}) == 0);
    CHECK(ok(rig.op(ana::kOpConfigure, configureRequest(10000, 100))));
    CHECK(ok(rig.op(ana::kOpStart)));
    convert(100, 1);
    CHECK(ok(rig.op(ana::kOpStop)));
    CHECK(ok(rig.op(ana::kOpStatus)) && rig.out[0] == ana::kStateDone && getU32(rig.out.data() + 1) == 1);
    CHECK(ok(rig.op(ana::kOpSegments, {0, 0, 0, 0})) && !(rig.out[2 + 32] & ana::kSegmentFlagShort));
  }
}

int main() {
  testStop();
  testHole();
  testPlanReleaseForgets();
  testOutputIdle();
  testRateRange();
  testQueryAsConfigure();
  testValuesRefused();
  testDescribe();
  testConfigureOrder();
  printf("analog: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
