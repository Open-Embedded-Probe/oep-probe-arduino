// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the ESP32-P4 logic capture's configure (oep-spec oep-if-capture §3.3) on fakes of the PARLIO RX driver
// and the heap (OEP_HOST_FAKE_PARLIO): a samples above what the probe can hold is rounded down to its limit in every
// mode (one-shot with and without a trigger, repeat), actual_samples is what the segment holds, and a configure in
// every mode at the lowest declared rate succeeds on boards with and without PSRAM; every configure TLV is checked the
// same with or without bit 7 (core §2.3), the answer and describe carry only what capture §3.3 / §3.5 define.
#include <stdio.h>

#include <algorithm>
#include <vector>

#include <esp_heap_caps.h>
#include <freertos/queue.h>

#include <esp_timer.h>

#include "OepCapture.h"
#include "OepEndpoint.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace cap = reg::fixture_logic;
namespace grp = reg::fixture_capture_group;

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

static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }

static void tlv(Bytes &p, uint8_t tag, const Bytes &v) {
  p.push_back(tag);
  p.push_back(static_cast<uint8_t>(v.size()));
  p.push_back(static_cast<uint8_t>(v.size() >> 8));
  p.insert(p.end(), v.begin(), v.end());
}
static Bytes u32(uint32_t v) { Bytes b(4); putU32(b.data(), v); return b; }
static constexpr uint8_t kCrit = 0x80;

struct Config {
  uint8_t mode = 1;
  uint32_t rate = LogicCapture::kMinHz;
  uint32_t samples = 1000;   // 0: not sent; never sent in streaming (capture §3.3)
  uint32_t segments = 0;
  bool trigger = false;
  uint32_t pretrigger = 0;
};

static Bytes request(const Config &c) {
  Bytes p;
  tlv(p, kCrit | cap::kTlvConfigureMode, {c.mode});
  tlv(p, kCrit | cap::kTlvConfigureRate, u32(c.rate));
  if (c.samples && c.mode != 3) tlv(p, cap::kTlvConfigureSamples, u32(c.samples));
  if (c.segments) tlv(p, cap::kTlvConfigureSegments, u32(c.segments));
  if (c.trigger) {   // an edge on role 0, falling
    Bytes t = {cap::kTriggerEdge, 0};
    const Bytes v = u32(1);
    t.insert(t.end(), v.begin(), v.end());
    tlv(p, kCrit | cap::kTlvConfigureTrigger, t);
  }
  if (c.pretrigger) tlv(p, kCrit | cap::kTlvConfigurePretrigger, u32(c.pretrigger));
  return p;
}

static bool findU32(const Bytes &a, uint8_t tag, uint32_t &v) {
  for (size_t at = 0; at + kTlvHeader <= a.size(); at += kTlvHeader + getU16(a.data() + at + 1))
    if (a[at] == tag && getU16(a.data() + at + 1) == 4) { v = getU32(a.data() + at + kTlvHeader); return true; }
  return false;
}

static Result configure(LogicCapture &c, const Config &cfg, Bytes &out, bool query = false) {
  const Bytes p = request(cfg);
  out.assign(256, 0);
  const Result r = c.handle(query ? LogicCapture::kOpQuery : LogicCapture::kOpConfigure, p.data(), p.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}

static bool rejectedAs(const Result &r, uint8_t reason) { return r.resolution == kResolutionRejected && r.detail == reason; }

static Result raw(LogicCapture &c, uint8_t op, const Bytes &p, Bytes &out) {
  out.assign(256, 0);
  const Result r = c.handle(op, p.data(), p.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}

// One capture per board, its plan on `channels` lines.
struct Rig {
  NullStream stream;
  uint8_t rx[512], tx[512];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kVendorBulk, 0};
  PinTable pins{0xFFFFull};
  LogicCapture cap{ep, pins};
  explicit Rig(uint8_t channels) {
    RoleAssignment roles[16];
    for (uint8_t k = 0; k < channels; ++k) roles[k] = {1, k, k};
    cap.planApply(roles, channels);
  }
  ~Rig() { cap.planRelease(); }   // the one PARLIO RX unit back
};

static void board(size_t internal, size_t psram) {
  g_fake_heap = FakeHeap{};
  g_fake_heap.internal_free = internal;
  g_fake_heap.psram_total = g_fake_heap.psram_free = psram;
}

// capture §3.3: samples above the limit is rounded down, actual_samples authoritative - never refused, never wrapped
static void testOverLimitRoundsDown() {
  for (const size_t psram : {size_t(0), size_t(32) << 20}) {
    for (const uint8_t channels : {1, 2, 16}) {
      const uint32_t w = channels == 1 ? 1 : channels == 2 ? 2 : 16;
      const uint32_t limit = static_cast<uint32_t>(LogicCapture::kSegmentBytes * 8 / w);
      for (const uint32_t asked : {limit + 1, 1u << 28, 0x40000000u, 0xFFFFFFFFu}) {
        for (const bool trigger : {false, true}) {
          board(512 * 1024, psram);
          Rig rig(channels);
          Bytes out;
          Config c;
          c.samples = asked;
          c.trigger = trigger;
          c.pretrigger = trigger ? 100 : 0;
          const Result r = configure(rig.cap, c, out);
          uint32_t got = 0;
          CHECK(ok(r) && findU32(out, cap::kTlvConfigureAnswerActualSamples, got) && got > 0 && got <= limit && got + 1024 / w > limit);
          if (!ok(r) || got == 0 || got > limit) printf("  one-shot w=%u asked=%u trigger=%d psram=%zu: %u (res %u det %u failed allocs %d units %d)\n", w, asked, trigger, psram, got, r.resolution, r.detail, g_fake_heap.failed, g_fake_parlio_units);
          Bytes q;
          CHECK(ok(configure(rig.cap, c, q, true)) && findU32(q, cap::kTlvConfigureAnswerActualSamples, got) && got > 0 && got <= limit);
        }
        board(512 * 1024, psram);
        Rig rig(channels);
        Bytes out;
        Config c;
        c.mode = 2;
        c.samples = asked;
        const Result r = configure(rig.cap, c, out);
        uint32_t got = 0;
        CHECK(ok(r) && findU32(out, cap::kTlvConfigureAnswerActualSamples, got) && got > 0 &&
              got <= LogicCapture::kSegmentMaxRepeat * 8 / w);
      }
    }
  }
}

// Every mode at the lowest declared rate (627451 Hz) configures, on a board with PSRAM and on one without.
static void testEveryModeAtTheLowestRate() {
  for (const size_t psram : {size_t(0), size_t(32) << 20}) {
    for (const uint8_t mode : {1, 2, 3}) {
      board(512 * 1024, psram);
      Rig rig(2);
      Bytes out;
      Config c;
      c.mode = mode;
      c.samples = 6275;
      const Result r = configure(rig.cap, c, out);
      CHECK(ok(r));
      if (!ok(r)) printf("  mode %u psram=%zu: resolution %u detail %u\n", mode, psram, r.resolution, r.detail);
      uint32_t got = 0;
      if (mode != 3) CHECK(findU32(out, cap::kTlvConfigureAnswerActualSamples, got) && got >= 6275);
    }
  }
}

static bool hasTag(const Bytes &a, uint8_t tag) {
  for (size_t at = 0; at + kTlvHeader <= a.size(); at += kTlvHeader + getU16(a.data() + at + 1))
    if (a[at] == tag) return true;
  return false;
}

class FakeDirect final : public DirectTransport {
 public:
  bool queueData(const uint8_t *, size_t, Done, void *) override { return true; }
  size_t queued() const override { return 0; }
};

// Streaming through a zero-copy transport takes stages of internal RAM; with too little left beside the DMA ring (a
// trigger's ring and the firmware's own use, 0.0.28 on the bench: a streaming configure answered failed) it streams
// the copied way through the store instead, and configure succeeds.
static void testStreamingWithoutStages() {
  for (const size_t internal : {size_t(160) << 10, size_t(512) << 10}) {
    board(internal, size_t(32) << 20);
    static FakeDirect direct;
    Rig rig(2);
    rig.ep.setDirect(&direct);
    rig.cap.setFrameLimit(16 * 1024);
    Bytes out;
    Config c;
    c.mode = 3;
    const Result r = configure(rig.cap, c, out);
    CHECK(ok(r));
    if (!ok(r)) printf("  streaming internal=%zu: resolution %u detail %u\n", internal, r.resolution, r.detail);
    // streaming's answer rows (capture §3.3): no actual_samples, no actual_segments
    CHECK(hasTag(out, cap::kTlvConfigureAnswerActualRate) && hasTag(out, cap::kTlvConfigureAnswerLayout) &&
          hasTag(out, cap::kTlvConfigureAnswerBlockingMs) && !hasTag(out, cap::kTlvConfigureAnswerActualSamples) &&
          !hasTag(out, cap::kTlvConfigureAnswerActualSegments));
  }
}

// The TLVs of a valid one-shot request (mode 1, the lowest rate, 1000 samples) but `skip`, bit 7 as `bit`.
static Bytes base(uint8_t bit, uint8_t skip = 0) {
  Bytes p;
  if (skip != cap::kTlvConfigureMode) tlv(p, bit | cap::kTlvConfigureMode, {1});
  if (skip != cap::kTlvConfigureRate) tlv(p, bit | cap::kTlvConfigureRate, u32(LogicCapture::kMinHz));
  if (skip != cap::kTlvConfigureSamples) tlv(p, bit | cap::kTlvConfigureSamples, u32(1000));
  return p;
}

// core §2.3 (capture §3.3): every configure TLV is checked the same with or without bit 7. A value the probe cannot
// honour refuses configure and query unsupported with the tag as received (bit 7 as sent) - nothing is ignored; a length
// other than the definition is malformed; an unknown non-critical tag is passed over.
static void testValuesRefused() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);
  Bytes out;
  const Bytes edge = {cap::kTriggerEdge, 0, 0, 0, 0, 0};
  for (const uint8_t bit : {uint8_t(0), kCrit}) {
    for (const uint8_t op : {LogicCapture::kOpConfigure, LogicCapture::kOpQuery}) {
      Bytes p = base(bit, cap::kTlvConfigureMode);
      tlv(p, bit | cap::kTlvConfigureMode, {7});                                  // a mode it does not have
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigureMode)});
      p = base(bit, cap::kTlvConfigureRate);
      tlv(p, bit | cap::kTlvConfigureRate, u32(1000));                            // below rate_range
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigureRate)});
      p = base(bit, cap::kTlvConfigureMode);
      tlv(p, kCrit | cap::kTlvConfigureMode, {2});
      tlv(p, bit | cap::kTlvConfigureTrigger, edge);                              // an edge in repeat
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigureTrigger)});
      p = base(bit);
      tlv(p, bit | cap::kTlvConfigureTrigger, {cap::kTriggerLevel, 0, 2, 0, 0, 0}); // a level of 2
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigureTrigger)});
      p = base(bit);
      tlv(p, bit | cap::kTlvConfigureTrigger, edge);
      tlv(p, bit | cap::kTlvConfigurePretrigger, u32(0x7FFFFFFF));                // more than the ring holds
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigurePretrigger)});
      // a length other than the definition: malformed, longer or shorter, any tag (none is ignored)
      for (const uint8_t tag : {cap::kTlvConfigureMode, cap::kTlvConfigureRate, cap::kTlvConfigureSamples,
                                cap::kTlvConfigureSegments, cap::kTlvConfigureTrigger, cap::kTlvConfigurePretrigger}) {
        const size_t size = tag == cap::kTlvConfigureMode ? 1 : tag == cap::kTlvConfigureTrigger ? 6 : 4;
        for (const size_t len : {size - 1, size + 1}) {
          p = base(bit, tag);
          tlv(p, bit | tag, Bytes(len, 1));
          CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectMalformed));
        }
      }
    }
  }
  // an unknown non-critical tag is passed over; the answer has no ignored (0x7F), timing (0x54) or rate_accuracy (0x5A)
  Bytes p = base(kCrit);
  tlv(p, 0x61, {1, 2, 3});
  tlv(p, 0x7F, {cap::kTlvConfigureSamples});   // no longer a special tag: unknown, non-critical
  CHECK(ok(raw(rig.cap, LogicCapture::kOpConfigure, p, out)));
  CHECK(hasTag(out, cap::kTlvConfigureAnswerActualRate) && hasTag(out, cap::kTlvConfigureAnswerLayout) &&
        hasTag(out, cap::kTlvConfigureAnswerActualSamples) && hasTag(out, cap::kTlvConfigureAnswerBlockingMs));
  CHECK(!hasTag(out, 0x7F) && !hasTag(out, 0x54) && !hasTag(out, 0x5A));
}

// capture §3.3's contract, configure and query alike: mode and rate required, samples in modes 1 / 2, none of them 0
// (malformed); samples in streaming, segments outside repeat, a pretrigger without a trigger or with an immediate one
// unsupported whatever the value (the tag as received); a trigger role outside the plan unavailable cause 6; a
// pretrigger at or above samples (as rounded) unsupported, just below it taken; success answers every row of the mode.
static void testContract() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);
  Bytes out;
  const Bytes edge = {cap::kTriggerEdge, 0, 0, 0, 0, 0}, immediate = {cap::kTriggerImmediate, 0, 0, 0, 0, 0};
  for (const uint8_t op : {LogicCapture::kOpConfigure, LogicCapture::kOpQuery}) {
    for (const uint8_t tag : {cap::kTlvConfigureMode, cap::kTlvConfigureRate, cap::kTlvConfigureSamples})
      CHECK(rejectedAs(raw(rig.cap, op, base(kCrit, tag), out), kRejectMalformed));   // a required one missing
    for (const uint8_t mode : {2, 3}) {   // samples required in repeat; in streaming only mode and rate
      Bytes p;
      tlv(p, cap::kTlvConfigureMode, {mode});
      tlv(p, cap::kTlvConfigureRate, u32(LogicCapture::kMinHz));
      CHECK(mode == 3 ? ok(raw(rig.cap, op, p, out)) : rejectedAs(raw(rig.cap, op, p, out), kRejectMalformed));
    }
    for (const uint8_t tag : {cap::kTlvConfigureSamples, cap::kTlvConfigureSegments}) {   // 0 is excluded
      Bytes p = base(0, cap::kTlvConfigureMode);
      tlv(p, cap::kTlvConfigureMode, {2});
      if (tag == cap::kTlvConfigureSamples) p = base(0, tag), tlv(p, tag, u32(0));
      else tlv(p, tag, u32(0));
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectMalformed));
    }
    for (const uint8_t bit : {uint8_t(0), kCrit}) {
      Bytes p;   // samples in streaming
      tlv(p, cap::kTlvConfigureMode, {3});
      tlv(p, cap::kTlvConfigureRate, u32(LogicCapture::kMinHz));
      tlv(p, bit | cap::kTlvConfigureSamples, u32(1000));
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigureSamples)});
      p = base(0);   // segments in one-shot
      tlv(p, bit | cap::kTlvConfigureSegments, u32(4));
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigureSegments)});
      for (const bool with_immediate : {false, true}) {   // a pretrigger without a trigger, with an immediate one
        p = base(0);
        if (with_immediate) tlv(p, cap::kTlvConfigureTrigger, immediate);
        tlv(p, bit | cap::kTlvConfigurePretrigger, u32(10));
        CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigurePretrigger)});
      }
    }
    Bytes p = base(0);   // a trigger role outside the plan (roles 0, 1): unavailable cause 6
    tlv(p, cap::kTlvConfigureTrigger, {cap::kTriggerLevel, 2, 1, 0, 0, 0});
    CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnavailable) && out == Bytes({reg::core::kTlvUnavailablePayloadCause, 1, 0, 6}));
    // the pretrigger against samples as rounded: 1000 samples at w = 2 is 1024 (whole cache lines)
    for (const uint32_t pre : {1023u, 1024u}) {
      p = base(0);
      tlv(p, cap::kTlvConfigureTrigger, edge);
      tlv(p, cap::kTlvConfigurePretrigger, u32(pre));
      const Result r = raw(rig.cap, op, p, out);
      CHECK(pre == 1023 ? ok(r) : rejectedAs(r, kRejectUnsupported) && out == Bytes{cap::kTlvConfigurePretrigger});
    }
    // every row of the mode: one-shot rate, layout, actual_samples, blocking_ms; repeat also actual_segments
    CHECK(ok(raw(rig.cap, op, base(0), out)) && hasTag(out, cap::kTlvConfigureAnswerActualRate) &&
          hasTag(out, cap::kTlvConfigureAnswerLayout) && hasTag(out, cap::kTlvConfigureAnswerActualSamples) &&
          hasTag(out, cap::kTlvConfigureAnswerBlockingMs) && !hasTag(out, cap::kTlvConfigureAnswerActualSegments));
    p = base(0, cap::kTlvConfigureMode);
    tlv(p, cap::kTlvConfigureMode, {2});
    tlv(p, cap::kTlvConfigureSegments, u32(3));
    CHECK(ok(raw(rig.cap, op, p, out)) && hasTag(out, cap::kTlvConfigureAnswerActualSamples) &&
          hasTag(out, cap::kTlvConfigureAnswerActualSegments));
  }
}

// capture §3.5: mode is mode(u8) max_samples(u32) max_segments(u32), one per mode; channels is max(u8); rate_list,
// rate_limit, max_read and segment_ring (0x42, 0x43, 0x47, 0x48) are not declared.
static void testDescribe() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);
  Bytes d(1024);
  d.resize(rig.cap.describe(d.data(), d.size()));
  CHECK(!d.empty());
  int modes = 0;
  bool channels = false;
  for (size_t at = 0; at + kTlvHeader <= d.size(); at += kTlvHeader + getU16(d.data() + at + 1)) {
    const uint8_t tag = d[at];
    const uint16_t len = getU16(d.data() + at + 1);
    CHECK(len <= 512 - 9);   // core §7.3: fits the smallest max_frame (512) with the header 5, more 1, TLV 3
    if (tag == cap::kTlvDescribeMode) { ++modes; CHECK(len == 9 && d[at + kTlvHeader] == modes); }
    if (tag == cap::kTlvDescribeChannels) { channels = true; CHECK(len == 1 && d[at + kTlvHeader] == LogicCapture::kMaxChannels); }
    CHECK(tag != 0x42 && tag != 0x43 && tag != 0x47 && tag != 0x48);
  }
  CHECK(modes == 3 && channels);
}

// core §4.3: malformed before unsupported before unavailable, over the whole request; the plan (cause 6) last.
static void testConfigureOrder() {
  board(512 * 1024, size_t(32) << 20);
  NullStream stream;
  uint8_t rx[512], tx[512];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kVendorBulk, 0};
  PinTable pins{0xFFFFull};
  LogicCapture c{ep, pins};   // no plan
  Bytes out, p = base(kCrit);
  CHECK(rejectedAs(raw(c, LogicCapture::kOpConfigure, p, out), kRejectUnavailable));
  tlv(p, kCrit | cap::kTlvConfigureTrigger, {cap::kTriggerEdge, 15, 0, 0, 0, 0});   // a role it has: the plan decides
  CHECK(rejectedAs(raw(c, LogicCapture::kOpConfigure, p, out), kRejectUnavailable));
  tlv(p, kCrit | 0x60, {1});                                                          // an unknown critical tag
  CHECK(rejectedAs(raw(c, LogicCapture::kOpConfigure, p, out), kRejectUnsupported) && out == Bytes{kCrit | 0x60});
  tlv(p, cap::kTlvConfigureSegments, {1});                                            // and a short segments
  CHECK(rejectedAs(raw(c, LogicCapture::kOpConfigure, p, out), kRejectMalformed));
  p = base(kCrit, cap::kTlvConfigureRate);
  tlv(p, kCrit | cap::kTlvConfigureRate, u32(0));                                     // 0 Hz: excluded (1 Hz or more)
  CHECK(rejectedAs(raw(c, LogicCapture::kOpQuery, p, out), kRejectMalformed));
}

// configure §3.3: inside rate_range, a one-shot rate above what the channel count allows (8 lines 100 MHz, 16 lines
// 48 MHz; not declared) is set to the nearest it allows, not run at the rate asked (160 MHz was taken for any count).
static void testRateLimit() {
  for (const uint8_t channels : {8, 16}) {
    board(512 * 1024, size_t(32) << 20);
    Rig rig(channels);
    Bytes out;
    Config c;
    c.rate = channels == 8 ? 160000000 : 100000000;
    for (const bool query : {true, false}) {
      CHECK(ok(configure(rig.cap, c, out, query)));
      Bytes rate;
      for (size_t at = 0; at + kTlvHeader <= out.size(); at += kTlvHeader + getU16(out.data() + at + 1))
        if (out[at] == cap::kTlvConfigureAnswerActualRate)
          rate.assign(out.begin() + at + kTlvHeader, out.begin() + at + kTlvHeader + getU16(out.data() + at + 1));
      // the fake divider is a whole number: 160 / 2 = 80 MHz for 100 MHz, 160 / 3 = 53.3 MHz for 48 MHz
      CHECK(rate.size() == 8 && getU32(rate.data()) / getU32(rate.data() + 4) == (channels == 8 ? 80000000u : 53333333u));
    }
  }
}

// Bound into a capture-group (§4.1): the track's own configure / start / stop / force are refused unavailable cause 4
// (the cause alone, core §4.3) - a malformed request is malformed (the form is checked before anything changes) - and
// its plan is the group's (boundTo: the endpoint refuses plan_apply / plan_release of its fn).
static void testBound() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);
  CaptureGroup group(rig.ep);
  rig.ep.add(rig.cap);   // fn 1
  rig.ep.add(group);     // fn 2
  group.addTrack(rig.cap, rig.cap);
  Bytes out;
  Config c;
  CHECK(ok(configure(rig.cap, c, out)));
  const uint8_t bind[] = {1, 1, 0};
  uint8_t g[16];
  CHECK(ok(group.handle(grp::kOpBind, bind, sizeof bind, g, sizeof g)));
  CHECK(rig.cap.boundTo() == 2);
  const Bytes cause4 = {0x01, 1, 0, 4};
  CHECK(rejectedAs(configure(rig.cap, c, out), kRejectUnavailable) && out == cause4);
  Bytes p;
  tlv(p, kCrit | cap::kTlvConfigureRate, {1});   // short: malformed
  CHECK(rejectedAs(raw(rig.cap, LogicCapture::kOpConfigure, p, out), kRejectMalformed));
  CHECK(ok(configure(rig.cap, c, out, true)));   // query: not the group's
  const Bytes bad_tail = {0x60, 4, 0, 1};   // a TLV longer than the request (core §2.3)
  for (const uint8_t op : {LogicCapture::kOpStart, LogicCapture::kOpStop, LogicCapture::kOpForce}) {
    CHECK(rejectedAs(raw(rig.cap, op, bad_tail, out), kRejectMalformed));
    CHECK(rejectedAs(raw(rig.cap, op, {}, out), kRejectUnavailable) && out == cause4);
  }
  const uint8_t unbind[] = {0};
  CHECK(ok(group.handle(grp::kOpBind, unbind, sizeof unbind, g, sizeof g)));
  CHECK(rig.cap.boundTo() == 0);
}

// Plays the PARLIO's DMA into the ring the capture handed the driver, and runs the harvest task (deferred by the fake
// FreeRTOS) until its queue is empty.
struct Dma {
  struct Idle {};
  size_t at = 0;
  Dma() { g_fake_tasks_deferred = true; g_fake_task_fn = nullptr; }
  ~Dma() { g_fake_tasks_deferred = false; }
  void deliver(size_t n, uint8_t value, size_t chunk = 4096) {
    while (n) {
      size_t k = g_fake_parlio_size - at;
      if (k > n) k = n;
      if (k > chunk) k = chunk;
      memset(g_fake_parlio_buffer + at, value, k);
      parlio_rx_event_data_t e = {g_fake_parlio_buffer + at, k};
      g_fake_parlio_callbacks.on_partial_receive(nullptr, &e, g_fake_parlio_context);
      at = (at + k) % g_fake_parlio_size;
      n -= k;
    }
  }
  void put(const Bytes &bytes) {   // these bytes, as one chunk (or two at the ring's end)
    size_t done = 0;
    while (done < bytes.size()) {
      size_t k = g_fake_parlio_size - at;
      if (k > bytes.size() - done) k = bytes.size() - done;
      memcpy(g_fake_parlio_buffer + at, bytes.data() + done, k);
      parlio_rx_event_data_t e = {g_fake_parlio_buffer + at, k};
      g_fake_parlio_callbacks.on_partial_receive(nullptr, &e, g_fake_parlio_context);
      at = (at + k) % g_fake_parlio_size;
      done += k;
    }
  }
  void run() {
    if (!g_fake_task_fn) return;
    g_fake_queue_empty = [] { throw Idle{}; };
    try { g_fake_task_fn(g_fake_task_arg); } catch (const Idle &) {}
    g_fake_queue_empty = nullptr;
  }
};

static Bytes readReq(uint32_t generation, uint64_t position, uint32_t max) {
  Bytes p(16);
  putU32(p.data(), generation);
  putU64(p.data() + 4, position);
  putU32(p.data() + 12, max);
  return p;
}

// The plan released (capture §3.2): state 0, the data and the segments gone - read is empty, status and segments count
// none (a repeat's read after it copied from the freed store).
static void testPlanReleaseForgets() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);
  Dma dma;
  Bytes out;
  Config c;
  c.mode = 2;
  c.samples = 16384;   // 4096 bytes at w = 2
  c.segments = 2;
  CHECK(ok(configure(rig.cap, c, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)) && out.size() == 8);
  const uint32_t generation = getU32(out.data() + 4);
  dma.deliver(4096, 0x55);
  dma.run();
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateCapturing && getU32(out.data() + 1) == 1);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 0, 100), out)) && getU32(out.data() + 9) == 100 && out[13] == 0x55);
  rig.cap.planRelease();
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out.size() >= 18);
  CHECK(out[0] == cap::kStateUnconfigured && getU32(out.data() + 1) == 0 && getU64(out.data() + 5) == 0);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 0, 100), out)) && out.size() == 13 && getU32(out.data() + 9) == 0);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out.size() >= 2 && out[1] == 0);
}

// stop (capture §3.2) of a one-shot with a trigger: capturing (state 3) -> 1, the segment cut short (flags bit1) with
// what the harvest copied, readable; waiting (state 2) -> 1 with nothing.
static void testTriggeredStop() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);
  Dma dma;
  Bytes out;
  Config c;
  c.samples = 40000;
  c.trigger = true;   // falling edge on role 0
  c.pretrigger = 100;
  CHECK(ok(configure(rig.cap, c, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  const uint32_t generation = getU32(out.data() + 4);
  dma.deliver(1000, 0xFF);   // samples 0..3999 high
  dma.run();
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateWaiting);
  dma.deliver(2000, 0x00);   // the edge at sample 4000; the segment from 3900 (byte 975)
  dma.run();
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateCapturing);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStop, {}, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateConfigured && getU32(out.data() + 1) == 1);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out.size() >= 2 + 37 && out[1] == 1);
  CHECK(getU32(out.data() + 2 + 12) == 2025 * 4 && getU32(out.data() + 2 + 28) == 100 && (out[2 + 32] & cap::kSegmentFlagShort));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 0, 100), out)) && getU32(out.data() + 9) == 100);
  CHECK(out[13] == 0xFF && out[13 + 24] == 0xFF && out[13 + 25] == 0x00);   // 25 bytes (100 samples) before the edge
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 2000, 100), out)) && getU32(out.data() + 9) == 25);
  // waiting, no edge: state 1, nothing
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  dma.deliver(1000, 0xFF);
  dma.run();
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStop, {}, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateConfigured && getU32(out.data() + 1) == 0);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out[1] == 0);
}

// One position space (capture §2.2, §3.2): a repeat's segments, read and write_pos count the bytes discarded while it
// was paused (state 5); a position in them or released moves on to the next segment with the gap flag. Streaming's new
// data discarded with every segment unsent shows as status flags bit0 (§2.1 rule 1).
static void testPositions() {
  {
    board(512 * 1024, size_t(32) << 20);
    Rig rig(2);
    Dma dma;
    Bytes out;
    Config c;
    c.mode = 2;
    c.samples = 16384;   // 4096 bytes at w = 2
    c.segments = 2;
    CHECK(ok(configure(rig.cap, c, out)));
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
    const uint32_t generation = getU32(out.data() + 4);
    dma.deliver(4096, 0x11);
    dma.deliver(4096, 0x22);
    dma.deliver(4096, 0x33);   // no free segment: discarded (paused)
    dma.run();
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStatePaused && getU32(out.data() + 1) == 2);
    CHECK(getU64(out.data() + 5) == 12288 && (out[13] & cap::kStatusFlagDropped) == 0);   // a pause is not a drop
    Bytes rel(8);
    putU32(rel.data(), generation);
    putU32(rel.data() + 4, 0);
    CHECK(ok(raw(rig.cap, LogicCapture::kOpRelease, rel, out)));
    dma.deliver(4096, 0x44);
    dma.run();
    CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {2, 0, 0, 0}, out)) && out[1] == 1);
    CHECK(getU64(out.data() + 2 + 4) == 12288 && (out[2 + 32] & cap::kSegmentFlagGap));   // after the discarded bytes
    CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 4096, 10), out)) && getU64(out.data()) == 4096 &&
          out[8] == reg::common::kReadFlagsMore && out[13] == 0x22);
    CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 9000, 10), out)) && getU64(out.data()) == 12288 &&
          (out[8] & reg::common::kReadFlagsGap) && getU32(out.data() + 9) == 10 && out[13] == 0x44);
    CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 0, 10), out)) && getU64(out.data()) == 4096 &&
          (out[8] & reg::common::kReadFlagsGap));   // released
    CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 16384, 10), out)) && getU64(out.data()) == 16384 &&
          getU32(out.data() + 9) == 0);
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && getU64(out.data() + 5) == 16384);
  }
  {
    board(512 * 1024, 0);   // a store of 4 x 64 KiB
    Rig rig(2);
    Dma dma;
    rig.cap.subscribe(true);
    Bytes out;
    Config c;
    c.mode = 3;
    CHECK(ok(configure(rig.cap, c, out)));
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
    for (int k = 0; k < 4; ++k) { dma.deliver(64 * 1024, 0x55); dma.run(); }
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && (out[13] & cap::kStatusFlagDropped) == 0);
    dma.deliver(4096, 0x66);   // nobody took any: new data discarded
    dma.run();
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && (out[13] & cap::kStatusFlagDropped) &&
          getU64(out.data() + 5) == 4 * 65536 + 4096);
  }
}

// A track configured immediate that a group makes follow another's trigger opens the ring and starts (its start
// answered into 4 bytes, failed, since the start answer carries the generation). It keeps the ring; a later start of
// its own is immediate again: no trigger inside its segment (trigger_index all ones) and no triggered event.
static void testImmediateAfterFollowing() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);
  Dma dma;
  Bytes out;
  Config c;
  c.samples = 4096;     // immediate: no pretrigger (capture §3.3)
  CHECK(ok(configure(rig.cap, c, out)));
  CHECK(rig.cap.trackStartFollowing());
  rig.cap.trackStop();
  CHECK(rig.cap.trackState() == cap::kStateConfigured);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  dma.deliver(4096, 0x5A);
  dma.run();
  rig.cap.poll();
  CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out.size() >= 2 + 37 && out[1] == 1);
  CHECK(getU32(out.data() + 2 + 28) == 0xFFFFFFFFu);
}

// A follower keeps the group's pretrigger P_k (capture §4.1): its segment starts P_k samples before the group's trigger
// (trigger_index P_k), and trackCanKeep bounds it by the ring at this width and the segment's samples.
static void testFollowerKeeps() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);   // w = 2: 4 samples a byte
  Dma dma;
  Bytes out;
  Config c;
  c.rate = 1000000;
  c.samples = 4096;
  CHECK(ok(configure(rig.cap, c, out)));
  CHECK(rig.cap.trackCanKeep(0) && rig.cap.trackCanKeep(4095) && !rig.cap.trackCanKeep(4096));
  rig.cap.trackKeep(400);
  const uint64_t start_ns = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;   // the fake clock does not move
  CHECK(rig.cap.trackStartFollowing());
  dma.deliver(50, 0x55);    // samples 0 - 199: not yet the pretrigger
  dma.run();
  CHECK(!rig.cap.trackArmed());
  dma.deliver(200, 0x55);   // to sample 999
  dma.run();
  CHECK(rig.cap.trackArmed());
  rig.cap.trackTriggerAt(start_ns + 2000000);   // 2 ms at 1 MHz: sample 2000
  dma.deliver(2048, 0x55);
  dma.run();
  rig.cap.poll();
  CHECK(rig.cap.trackState() == cap::kStateDone);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out[1] == 1 && getU32(out.data() + 2 + 28) == 400);
}

// stop (capture §3.2) of an immediate one-shot while capturing: state 1 with the segment cut short (flags bit1) holding
// the DMA nodes finished before the stop, readable (it went to state 1 with done 0, write_pos 0); the next start is a
// whole capture again.
static void testImmediateStop() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);
  Dma dma;
  Bytes out;
  Config c;
  c.samples = 40000;   // 10000 bytes at w = 2
  CHECK(ok(configure(rig.cap, c, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  const uint32_t generation = getU32(out.data() + 4);
  dma.deliver(4096, 0xA5);   // one node finished
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateCapturing);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStop, {}, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out.size() >= 18);
  CHECK(out[0] == cap::kStateConfigured && getU32(out.data() + 1) == 1 && getU64(out.data() + 5) == 4096);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out.size() >= 2 + 37 && out[1] == 1);
  CHECK(getU32(out.data() + 2 + 12) == 4096 * 4 && (out[2 + 32] & cap::kSegmentFlagShort));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 4000, 200), out)) && getU32(out.data() + 9) == 96);
  CHECK(out[13] == 0xA5);
  // stopped before any node finished: state 1, nothing
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStop, {}, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateConfigured && getU32(out.data() + 1) == 0);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out[1] == 0);
}

// Every start is a clean capture, with or without a configure before it: its receive goes to a PARLIO RX unit made for
// it, as configure leaves one (a unit that had run kept what its last transaction left: after a stop of an immediate
// one-shot the next start's done came after the rest of the window only, a third to a half of the segment unwritten,
// and after a completed run each start began with 258 bytes of the previous capture - P4 bench, 0.0.28+1c940ca).
static void testEveryStartFresh() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(1);
  Bytes out;
  Config c;
  c.samples = 40000;   // 5000 bytes at w = 1
  CHECK(ok(configure(rig.cap, c, out)));
  const int reused = g_fake_parlio_reused;
  auto complete = [](uint8_t value) {   // the whole segment, then the driver's done
    Dma dma;
    dma.deliver(g_fake_parlio_size, value);
    parlio_rx_event_data_t e = {g_fake_parlio_buffer, g_fake_parlio_size};
    g_fake_parlio_callbacks.on_receive_done(nullptr, &e, g_fake_parlio_context);
  };
  // a completed run, then a start without a configure: a new unit, the segment all the new data
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  complete(0x00);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateDone);
  int made = g_fake_parlio_made;
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  CHECK(g_fake_parlio_made == made + 1 && g_fake_parlio_units == 1);
  uint32_t generation = getU32(out.data() + 4);
  complete(0xFF);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateDone);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 0, 200), out)) && getU32(out.data() + 9) == 200);
  CHECK(std::all_of(out.begin() + 13, out.begin() + 213, [](uint8_t b) { return b == 0xFF; }));
  // stopped part way, then a start without a configure: a new unit again, done only at the whole window
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  {
    Dma dma;
    dma.deliver(4096, 0x00);
  }
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStop, {}, out)));
  made = g_fake_parlio_made;
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  CHECK(g_fake_parlio_made == made + 1 && g_fake_parlio_size == 5120);
  generation = getU32(out.data() + 4);
  complete(0xFF);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateDone && getU64(out.data() + 5) == 5120);
  for (uint32_t at : {0u, 4000u, 4920u}) {   // before, across and after where the stopped run had got to
    CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, at, 200), out)) && getU32(out.data() + 9) == 200);
    CHECK(std::all_of(out.begin() + 13, out.begin() + 213, [](uint8_t b) { return b == 0xFF; }));
  }
  // a repeat and a one-shot through the ring alike: start, stop, start
  Config rep;
  rep.mode = 2;
  rep.samples = 16384;
  Config trig;
  trig.samples = 4000;
  trig.trigger = true;
  for (const Config &k : {rep, trig}) {
    Dma dma;
    CHECK(ok(configure(rig.cap, k, out)));
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
    dma.deliver(4096, 0xFF);
    dma.run();
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStop, {}, out)));
    made = g_fake_parlio_made;
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
    CHECK(g_fake_parlio_made == made + 1);
    dma.run();
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStop, {}, out)));
  }
  CHECK(g_fake_parlio_reused == reused);   // no receive on a unit that had one
}

// One large immediate one-shot breaks no later triggered configure (0.0.28: 523264 samples on 1 line freed the DMA
// ring for its segment, the rest of the firmware took a piece of the freed 128 KiB, and every triggered configure after
// it failed with an empty payload, state 6, until a reboot): the ring is taken with the plan and never freed, the
// immediate segment is the ring. A configure that finds no memory is refused unavailable cause 3, the configuration it
// would replace left as it was.
static void testRingKept() {
  Bytes out;
  Config trig;
  trig.rate = 1000000;
  trig.samples = 1024;
  trig.trigger = true;
  trig.pretrigger = 100;
  {
    board(LogicCapture::kRingBytes + 40 * 1024, size_t(32) << 20);
    Rig rig(1);
    CHECK(ok(configure(rig.cap, trig, out)));
    Config big;
    big.rate = 1000000;
    big.samples = 523264;
    CHECK(ok(configure(rig.cap, big, out)));
    void *firmware = heap_caps_malloc(30 * 1024, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);   // the rest of the firmware
    CHECK(firmware != nullptr);
    CHECK(ok(configure(rig.cap, trig, out)));
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateConfigured);
    CHECK(ok(configure(rig.cap, big, out)));
    CHECK(ok(configure(rig.cap, trig, out)));
    heap_caps_free(firmware);
  }

  // no room for the ring at all (no PSRAM, little internal RAM): immediate runs on a block of its own, triggered and
  // repeat are refused cause 3 and the immediate configuration stays
  board(100 * 1024, 0);
  Rig small(1);
  Config imm;
  imm.samples = 40000;
  CHECK(ok(configure(small.cap, imm, out)));
  Result r = configure(small.cap, trig, out);
  CHECK(rejectedAs(r, kRejectUnavailable) && out == Bytes({reg::core::kTlvUnavailablePayloadCause, 1, 0,
                                                           reg::core::kUnavailableCauseStorageFull}));
  CHECK(ok(raw(small.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateConfigured);
  Config rep;
  rep.mode = 2;
  rep.samples = 16384;
  r = configure(small.cap, rep, out);
  CHECK(rejectedAs(r, kRejectUnavailable));
  CHECK(ok(raw(small.cap, LogicCapture::kOpStart, {}, out)));   // the immediate one still runs
  CHECK(ok(raw(small.cap, LogicCapture::kOpStop, {}, out)));
}

// A chunk the harvest queue (128) had no room for: its bytes are a gap where they were - a repeat's next segment starts
// after them with the gap flag (not at their position with the bytes after shifted into it); a triggered segment
// copies them from the ring, the bytes after them in place.
static void testLostChunk() {
  {
    board(512 * 1024, size_t(32) << 20);
    Rig rig(2);
    Dma dma;
    Bytes out;
    Config c;
    c.mode = 2;
    c.samples = 16384;   // 4096 bytes at w = 2
    c.segments = 4;
    CHECK(ok(configure(rig.cap, c, out)));
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
    const uint32_t generation = getU32(out.data() + 4);
    dma.deliver(4096, 0x11, 32);   // 128 chunks: the queue full
    dma.deliver(64, 0x22, 32);     // 2 lost
    dma.run();
    for (int k = 0; k < 4; ++k) { dma.deliver(1024, 0x33, 32); dma.run(); }
    // capture §2.2: serial 0 (finished before the loss) handed out; serial 1 had the hole: not handed out, the track
    // stopped in state 6 with error 2, write_pos at serial 1's start, status flags bit0
    rig.cap.poll();
    CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out[1] == 1);
    CHECK(getU64(out.data() + 2 + 4) == 0 && !(out[2 + 32] & cap::kSegmentFlagGap));
    CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 4096, 10), out)) && getU32(out.data() + 9) == 0);
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out.size() == 22 && out[0] == cap::kStateError &&
          getU32(out.data() + 1) == 1 && getU64(out.data() + 5) == 4096 && (out[13] & cap::kStatusFlagDropped) &&
          out[21] == cap::kErrorStorage);
  }
  {   // the loss inside a segment: the bytes of it so far are never read
    board(512 * 1024, size_t(32) << 20);
    Rig rig(2);
    Dma dma;
    Bytes out;
    Config c;
    c.mode = 2;
    c.samples = 16384;
    c.segments = 4;
    CHECK(ok(configure(rig.cap, c, out)));
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
    const uint32_t generation = getU32(out.data() + 4);
    dma.deliver(4096 + 1024, 0x11);   // serial 0 and a quarter of serial 1
    dma.run();
    dma.deliver(1024, 0x22, 8);       // 128 chunks: the queue full
    dma.deliver(16, 0x22, 8);         // 2 lost, inside serial 1
    dma.run();
    dma.deliver(1024, 0x33);
    dma.run();
    rig.cap.poll();
    CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out[1] == 1);
    CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 4096, 10), out)) && getU32(out.data() + 9) == 0);
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateError && getU64(out.data() + 5) == 4096);
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));   // state 6 starts again: a new generation, no error
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateCapturing && out.size() == 18);
  }
  {
    board(512 * 1024, size_t(32) << 20);
    Rig rig(2);
    Dma dma;
    Bytes out;
    Config c;
    c.samples = 40000;   // 10000 bytes
    c.trigger = true;    // falling edge on role 0
    c.pretrigger = 100;
    CHECK(ok(configure(rig.cap, c, out)));
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
    const uint32_t generation = getU32(out.data() + 4);
    dma.deliver(1000, 0xFF);   // samples 0..3999 high
    dma.deliver(2000, 0x00);   // the edge at sample 4000; the segment from byte 975
    dma.run();
    dma.deliver(4096, 0x10, 32);   // 128 chunks: the queue full
    dma.deliver(32, 0x20, 32);     // lost
    dma.run();
    dma.deliver(4000, 0x30, 32);
    dma.run();
    rig.cap.poll();
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out[0] == cap::kStateDone);
    // segment byte i = stream byte 975 + i: 0x10 up to 3000 + 4096, the lost chunk's 0x20, then 0x30
    const uint64_t at = 3000 + 4096 - 975;
    CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, at - 1, 40), out)) && getU32(out.data() + 9) == 40);
    CHECK(out[13] == 0x10 && out[13 + 1] == 0x20 && out[13 + 32] == 0x20 && out[13 + 33] == 0x30);
  }
}

namespace oep {
struct LogicCaptureSerials {
  static void set(LogicCapture &c, uint32_t serial) { c.completed_ = c.released_ = serial; }
};
}  // namespace oep

// A repeat's serials wrap (capture §2.2): serial_done, segments (common §1.3's paging: from_serial inclusive, = next
// empty, one not kept from the oldest kept), read and release across 0xFFFFFFFF -> 0; release frees finished segments
// only, a future serial every finished one.
static void testSerialsWrap() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);
  Dma dma;
  Bytes out;
  Config c;
  c.mode = 2;
  c.samples = 16384;   // 4096 bytes at w = 2
  c.segments = 4;
  CHECK(ok(configure(rig.cap, c, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  const uint32_t generation = getU32(out.data() + 4);
  LogicCaptureSerials::set(rig.cap, 0xFFFFFFFEu);
  dma.deliver(4096, 0x11);
  dma.deliver(4096, 0x22);
  dma.deliver(4096, 0x33);
  dma.deliver(1000, 0x44);   // the fourth, filling
  dma.run();
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && getU32(out.data() + 1) == 1);   // serial_done: next
  auto segments = [&](uint32_t from) { Bytes p(4); putU32(p.data(), from); return raw(rig.cap, LogicCapture::kOpSegments, p, out); };
  CHECK(ok(segments(0xFFFFFFFEu)) && out[0] == 0 && out[1] == 3 && getU32(out.data() + 2) == 0xFFFFFFFEu &&
        getU32(out.data() + 2 + 37) == 0xFFFFFFFFu && getU32(out.data() + 2 + 74) == 0);
  CHECK(ok(segments(0)) && out[0] == 0 && out[1] == 1 && getU32(out.data() + 2) == 0 && getU64(out.data() + 2 + 4) == 8192);
  CHECK(ok(segments(1)) && out.size() == 2 && out[0] == 0 && out[1] == 0);                     // = next: none, more 0
  CHECK(ok(segments(7)) && out[1] == 3 && getU32(out.data() + 2) == 0xFFFFFFFEu);              // not given: the oldest
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 8192, 10), out)) && getU64(out.data()) == 8192 &&
        out[13] == 0x33);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 4096, 10), out)) && out[13] == 0x22 &&
        (out[8] & reg::common::kReadFlagsMore));
  Bytes rel(8);
  putU32(rel.data(), generation);
  putU32(rel.data() + 4, 0xFFFFFFFFu);   // the first two
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRelease, rel, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 0, 10), out)) && getU64(out.data()) == 8192 &&
        (out[8] & reg::common::kReadFlagsGap) && out[13] == 0x33);
  putU32(rel.data() + 4, 0xFFFFFFFEu);   // already released: nothing
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRelease, rel, out)));
  putU32(rel.data() + 4, 100);           // ahead of serial_done: every finished one, not the one filling
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRelease, rel, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 8192, 10), out)) && getU64(out.data()) == 12288 &&
        getU32(out.data() + 9) == 0);
  dma.deliver(3096, 0x44);   // the fourth finishes: readable (not released by the earlier serial 100)
  dma.run();
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 12288, 10), out)) && getU32(out.data() + 9) == 10 &&
        out[13] == 0x44);
}

// A triggered one-shot whose ring came round over its segment before the harvest copied it (capture §2.2): no
// segment, state 6, error 2, write_pos 0, stopped reason 3.
static void testTriggeredHole() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);
  Dma dma;
  Bytes out;
  Config c;
  c.samples = 200000;   // 50000 bytes
  c.trigger = true;
  c.pretrigger = 100;
  CHECK(ok(configure(rig.cap, c, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  dma.deliver(1000, 0xFF);
  dma.deliver(2000, 0x00);   // the edge: filling
  dma.run();
  dma.deliver(LogicCapture::kRingBytes + 8192, 0x00, 1024);   // over the ring before the harvest runs again
  dma.run();
  rig.cap.poll();
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && out.size() == 22 && out[0] == cap::kStateError &&
        getU32(out.data() + 1) == 0 && getU64(out.data() + 5) == 0 && (out[13] & cap::kStatusFlagDropped) &&
        out[21] == cap::kErrorStorage);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out[1] == 0);
}

// trigger_index is the pretrigger exactly (capture §2, §3.3: pretrigger samples kept before the trigger's), wherever in
// a byte the trigger falls: at w = 2 an edge on the second sample of a byte gave trigger_index 101 for a pretrigger of
// 100 (the segment started on the byte holding sample t - P; bench, P4, 1001 for 1000). The segment's data starts
// at the sample P before the edge.
static void testTriggerIndexMidByte() {
  board(512 * 1024, size_t(32) << 20);
  Bytes out;
  Config c;
  c.samples = 4096;
  c.trigger = true;   // falling edge on role 0
  c.pretrigger = 100;
  {
  Rig rig(2);   // w = 2: 4 samples a byte
  Dma dma;
  CHECK(ok(configure(rig.cap, c, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  const uint32_t generation = getU32(out.data() + 4);
  dma.deliver(1000, 0xFF);   // samples 0..3999 high
  dma.put({0x03});           // sample 4000 high, 4001 low: the edge at 4001; the segment from 3901
  dma.deliver(2000, 0x00);
  dma.run();
  rig.cap.poll();
  CHECK(rig.cap.trackState() == cap::kStateDone);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out.size() >= 2 + 37 && out[1] == 1);
  CHECK(getU32(out.data() + 2 + 12) == 4096 && getU32(out.data() + 2 + 28) == 100);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 0, 40), out)) && getU32(out.data() + 9) == 40);
  CHECK(out[13] == 0xFF && out[13 + 24] == 0xFF && out[13 + 25] == 0x00);   // samples 0..99 high, 100 (the edge) low
  }
  {   // stopped while filling: the short segment (flags bit1) starts at the same sample
  Rig rig(2);
  Dma dma;
  c.samples = 40000;
  CHECK(ok(configure(rig.cap, c, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
  const uint32_t generation = getU32(out.data() + 4);
  dma.deliver(1000, 0xFF);
  dma.put({0x03});
  dma.deliver(999, 0x00);   // stream bytes 975 (holding sample 3901) .. 1999 copied: 1025 bytes less 2 bits
  dma.run();
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStop, {}, out)));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out[1] == 1);
  CHECK(getU32(out.data() + 2 + 12) == 4099 && getU32(out.data() + 2 + 28) == 100 && (out[2 + 32] & cap::kSegmentFlagShort));
  CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 0, 40), out)) && getU32(out.data() + 9) == 40);
  CHECK(out[13] == 0xFF && out[13 + 24] == 0xFF && out[13 + 25] == 0x00);
  CHECK(ok(raw(rig.cap, LogicCapture::kOpStatus, {}, out)) && getU64(out.data() + 5) == 1025);   // (4099 x 2 + 7) / 8
  c.samples = 4096;
  }
  for (const uint8_t w : {uint8_t(1), uint8_t(4)}) {   // the other widths below a byte: 8 and 2 samples a byte
    Rig other(w);
    Dma d;
    c.pretrigger = 1000;
    CHECK(ok(configure(other.cap, c, out)));
    CHECK(ok(raw(other.cap, LogicCapture::kOpStart, {}, out)));
    d.deliver(2000, 0xFF);
    d.put({static_cast<uint8_t>(w == 1 ? 0x01 : 0x0F)});   // the edge on the byte's second sample
    d.deliver(4096, 0x00);
    d.run();
    other.cap.poll();
    CHECK(ok(raw(other.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out[1] == 1);
    CHECK(getU32(out.data() + 2 + 12) == 4096 && getU32(out.data() + 2 + 28) == 1000);
  }
}

// force (capture §3.3): triggered at the sample of that instant, trigger_index = the pretrigger when that many are in;
// a force (or an edge) before then: the segment is short - the samples there are, the trigger's and samples - pretrigger -
// 1 after it - with trigger_index the samples before it.
static void testForcedAndEarly() {
  board(512 * 1024, size_t(32) << 20);
  {
    Rig rig(1);   // w = 1
    Dma dma;
    Bytes out;
    Config c;
    c.samples = 8192;
    c.trigger = true;
    c.pretrigger = 1000;
    CHECK(ok(configure(rig.cap, c, out)));
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
    dma.deliver(4096, 0xFF);   // no edge
    dma.run();
    CHECK(ok(raw(rig.cap, LogicCapture::kOpForce, {}, out)));
    dma.deliver(4096, 0xFF);
    dma.run();
    rig.cap.poll();
    CHECK(rig.cap.trackState() == cap::kStateDone);
    CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out[1] == 1);
    CHECK(getU32(out.data() + 2 + 12) == 8192 && getU32(out.data() + 2 + 28) == 1000);
  }
  {
    Rig rig(2);
    Dma dma;
    Bytes out;
    Config c;
    c.samples = 8192;
    c.trigger = true;
    c.pretrigger = 1000;
    CHECK(ok(configure(rig.cap, c, out)));
    CHECK(ok(raw(rig.cap, LogicCapture::kOpStart, {}, out)));
    const uint32_t generation = getU32(out.data() + 4);
    dma.deliver(100, 0xFF);   // samples 0..399 high
    dma.put({0x03});          // the edge at 401, before 1000 samples are in
    dma.deliver(4096, 0x00);
    dma.run();
    rig.cap.poll();
    CHECK(rig.cap.trackState() == cap::kStateDone);
    CHECK(ok(raw(rig.cap, LogicCapture::kOpSegments, {0, 0, 0, 0}, out)) && out[1] == 1);
    CHECK(getU32(out.data() + 2 + 12) == 401 + 8192 - 1000 && getU32(out.data() + 2 + 28) == 401);
    CHECK((out[2 + 32] & cap::kSegmentFlagShort) == 0);   // bit1 is a stop's
    CHECK(ok(raw(rig.cap, LogicCapture::kOpRead, readReq(generation, 1890, 100), out)) && getU32(out.data() + 9) == 9);   // (7593 x 2 + 7) / 8 bytes
  }
}

int main() {
  testTriggerIndexMidByte();
  testForcedAndEarly();
  testFollowerKeeps();
  testSerialsWrap();
  testTriggeredHole();
  testRingKept();
  testImmediateStop();
  testEveryStartFresh();
  testImmediateAfterFollowing();
  testPositions();
  testTriggeredStop();
  testLostChunk();
  testPlanReleaseForgets();
  testBound();
  testRateLimit();
  testValuesRefused();
  testContract();
  testDescribe();
  testConfigureOrder();
  testStreamingWithoutStages();
  testOverLimitRoundsDown();
  testEveryModeAtTheLowestRate();
  printf("capture: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
