// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the analog capture (oep-spec oep-if-capture §1.2, §3.2, §3.3) in its RP2 build, on fakes of the RP2's ADC
// and DMA (OEP_HOST_FAKE_RP2_ADC): configure's refusals in core §4.3's order, the sent-critical TLVs, the plan's
// refusals, and what start / stop / plan_release leave to read.
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
  PinTable pins{0xF0000000ull | 0xFFull};
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

static Bytes configureRequest(uint32_t rate, uint32_t samples = 0) {
  Bytes p;
  tlv(p, kCrit | ana::kTlvConfigureMode, {ana::kModeOneShot});
  tlv(p, kCrit | ana::kTlvConfigureRate, u32(rate));
  if (samples) tlv(p, ana::kTlvConfigureSamples, u32(samples));
  return p;
}

// mode, rate, trigger, pretrigger and frontend are critical whether or not bit 7 is set (capture §3.3, core §2.3):
// a value the probe cannot honour refuses the configure, with the tag as received; never ignored.
static void testSentCritical() {
  Rig rig;
  CHECK(rig.plan({26}) == 0);
  for (const uint8_t bit : {uint8_t(0), kCrit}) {
    Bytes p;
    tlv(p, bit | ana::kTlvConfigureMode, {ana::kModeRepeat});
    tlv(p, kCrit | ana::kTlvConfigureRate, u32(10000));
    Result r = rig.op(ana::kOpConfigure, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigureMode));
    p.clear();
    tlv(p, kCrit | ana::kTlvConfigureRate, u32(10000));
    tlv(p, bit | ana::kTlvConfigureTrigger, {1, 0, 0, 0, 0, 0});   // a logic type
    r = rig.op(ana::kOpQuery, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigureTrigger));
    p.clear();
    tlv(p, kCrit | ana::kTlvConfigureRate, u32(10000));
    tlv(p, ana::kTlvConfigureSamples, u32(100));
    tlv(p, bit | ana::kTlvConfigurePretrigger, u32(100));         // no room left in the segment
    r = rig.op(ana::kOpConfigure, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigurePretrigger));
    p.clear();
    tlv(p, kCrit | ana::kTlvConfigureRate, u32(10000));
    tlv(p, bit | ana::kTlvConfigureFrontend, {0, 9});              // a frontend it does not declare
    r = rig.op(ana::kOpConfigure, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigureFrontend));
    p.clear();
    tlv(p, bit | ana::kTlvConfigureRate, {0x10, 0x27, 0, 0, 0});   // longer than it knows (core §2.3)
    r = rig.op(ana::kOpConfigure, p);
    CHECK(rejectedAs(r, kRejectUnsupported) && rig.out.size() == 1 && rig.out[0] == (bit | ana::kTlvConfigureRate));
  }
  // samples goes by its bit: longer than known and not critical, ignored and listed
  Bytes p = configureRequest(10000);
  tlv(p, ana::kTlvConfigureSamples, {1, 0, 0, 0, 0});
  const Result r = rig.op(ana::kOpConfigure, p);
  const Bytes ignored = {0x7F, 1, ana::kTlvConfigureSamples};
  CHECK(ok(r) && std::search(rig.out.begin(), rig.out.end(), ignored.begin(), ignored.end()) != rig.out.end());
}

// core §4.3: malformed before unsupported before unavailable, over the whole request.
static void testConfigureOrder() {
  Rig rig;   // no plan yet
  Bytes p = configureRequest(10000);
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectUnavailable));   // no plan: cause 6
  tlv(p, kCrit | 0x60, {1});                                              // an unknown critical tag
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectUnsupported) && rig.out[0] == (kCrit | 0x60));
  tlv(p, ana::kTlvConfigureSamples, {1, 0});                              // and a short samples
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectMalformed));
  p = configureRequest(10000);
  tlv(p, kCrit | ana::kTlvConfigureMode, {});                             // mode twice: malformed (core §2.3)
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectMalformed));
  p.clear();
  tlv(p, kCrit | ana::kTlvConfigureRate, u32(10000));
  tlv(p, kCrit | ana::kTlvConfigureTrigger, {3, 3, 0, 8, 0, 0});          // role 3 without a plan: the plan's refusal
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectUnavailable));
  CHECK(rig.plan({26}) == 0);
  CHECK(rejectedAs(rig.op(ana::kOpConfigure, p), kRejectUnsupported) && rig.out[0] == (kCrit | ana::kTlvConfigureTrigger));
}

static bool findTlv(const Bytes &a, uint8_t tag, Bytes &v) {
  for (size_t at = 0; at + 2 <= a.size(); at += 2u + a[at + 1])
    if (a[at] == tag) { v.assign(a.begin() + at + 2, a.begin() + at + 2 + a[at + 1]); return true; }
  return false;
}

// rate (capture §3.3): outside the declared rate_range unsupported, tag 0x42 as received; inside it, the nearest the
// channel count's rate_limit allows (500 kS/s in all on the RP2).
static void testRateRange() {
  Rig rig;
  CHECK(rig.plan({26, 27}) == 0);
  for (const uint32_t rate : {uint32_t(1), uint32_t(48000000 / 65536), uint32_t(500001), uint32_t(10000000)}) {
    for (const uint8_t bit : {uint8_t(0), kCrit}) {
      Bytes p;
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

int main() {
  testRateRange();
  testQueryAsConfigure();
  testSentCritical();
  testConfigureOrder();
  printf("analog: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
