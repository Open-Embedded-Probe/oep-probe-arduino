// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the ESP32-P4 logic capture's configure (oep-spec oep-if-capture §3.3) on fakes of the PARLIO RX driver
// and the heap (OEP_HOST_FAKE_PARLIO): a samples above what the probe can hold is rounded down to its limit in every
// mode (one-shot with and without a trigger, repeat), actual_samples is what the segment holds, and a configure in
// every mode at the lowest declared rate succeeds on boards with and without PSRAM.
#include <stdio.h>

#include <algorithm>
#include <vector>

#include <esp_heap_caps.h>

#include "OepCapture.h"
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

static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }

static void tlv(Bytes &p, uint8_t tag, const Bytes &v) {
  p.push_back(tag);
  p.push_back(static_cast<uint8_t>(v.size()));
  p.insert(p.end(), v.begin(), v.end());
}
static Bytes u32(uint32_t v) { Bytes b(4); putU32(b.data(), v); return b; }
static constexpr uint8_t kCrit = 0x80;

struct Config {
  uint8_t mode = 1;
  uint32_t rate = LogicCapture::kMinHz;
  uint32_t samples = 0;   // 0: not sent
  uint32_t segments = 0;
  bool trigger = false;
  uint32_t pretrigger = 0;
};

static Bytes request(const Config &c) {
  Bytes p;
  tlv(p, kCrit | cap::kTlvConfigureMode, {c.mode});
  tlv(p, kCrit | cap::kTlvConfigureRate, u32(c.rate));
  if (c.samples) tlv(p, cap::kTlvConfigureSamples, u32(c.samples));
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
  for (size_t at = 0; at + 2 <= a.size(); at += 2u + a[at + 1])
    if (a[at] == tag && a[at + 1] == 4) { v = getU32(a.data() + at + 2); return true; }
  return false;
}

static Result configure(LogicCapture &c, const Config &cfg, Bytes &out, bool query = false) {
  const Bytes p = request(cfg);
  out.assign(256, 0);
  const Result r = c.handle(query ? LogicCapture::kOpQuery : LogicCapture::kOpConfigure, p.data(), p.size(), out.data(), out.size());
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
    uint32_t segments = 0;
    CHECK(findU32(out, cap::kTlvConfigureAnswerActualSegments, segments) && segments >= 2);
  }
}

static bool rejectedAs(const Result &r, uint8_t reason) { return r.resolution == kResolutionRejected && r.detail == reason; }

static Result raw(LogicCapture &c, uint8_t op, const Bytes &p, Bytes &out) {
  out.assign(256, 0);
  const Result r = c.handle(op, p.data(), p.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}

// mode, rate, trigger and pretrigger are critical whether or not bit 7 is set (capture §3.3, core §2.3): a value the
// probe cannot honour refuses configure and query with the tag as received; never ignored, never listed in ignored.
static void testSentCritical() {
  board(512 * 1024, size_t(32) << 20);
  Rig rig(2);
  Bytes out;
  for (const uint8_t bit : {uint8_t(0), kCrit}) {
    for (const uint8_t op : {LogicCapture::kOpConfigure, LogicCapture::kOpQuery}) {
      Bytes p;
      tlv(p, bit | cap::kTlvConfigureMode, {7});                                  // a mode it does not have
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigureMode)});
      p.clear();
      tlv(p, bit | cap::kTlvConfigureRate, u32(1000));                            // below rate_range
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigureRate)});
      p.clear();
      tlv(p, kCrit | cap::kTlvConfigureMode, {2});
      tlv(p, bit | cap::kTlvConfigureTrigger, {cap::kTriggerEdge, 0, 0, 0, 0, 0}); // an edge in repeat
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigureTrigger)});
      p.clear();
      tlv(p, bit | cap::kTlvConfigurePretrigger, u32(0x7FFFFFFF));                // more than the ring holds
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigurePretrigger)});
      p.clear();
      tlv(p, bit | cap::kTlvConfigureMode, {1, 0});                               // longer than it knows (core §2.3)
      CHECK(rejectedAs(raw(rig.cap, op, p, out), kRejectUnsupported) && out == Bytes{uint8_t(bit | cap::kTlvConfigureMode)});
    }
  }
  // samples goes by its bit: longer and not critical, ignored and listed
  Bytes p;
  tlv(p, cap::kTlvConfigureSamples, {1, 0, 0, 0, 0});
  const Bytes ignored = {0x7F, 1, cap::kTlvConfigureSamples};
  CHECK(ok(raw(rig.cap, LogicCapture::kOpConfigure, p, out)) &&
        std::search(out.begin(), out.end(), ignored.begin(), ignored.end()) != out.end());
}

// core §4.3: malformed before unsupported before unavailable, over the whole request; the plan (cause 6) last.
static void testConfigureOrder() {
  board(512 * 1024, size_t(32) << 20);
  NullStream stream;
  uint8_t rx[512], tx[512];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kVendorBulk, 0};
  PinTable pins{0xFFFFull};
  LogicCapture c{ep, pins};   // no plan
  Bytes out, p;
  tlv(p, kCrit | cap::kTlvConfigureRate, u32(1000000));
  CHECK(rejectedAs(raw(c, LogicCapture::kOpConfigure, p, out), kRejectUnavailable));
  tlv(p, kCrit | cap::kTlvConfigureTrigger, {cap::kTriggerEdge, 15, 0, 0, 0, 0});   // a role it has: the plan decides
  CHECK(rejectedAs(raw(c, LogicCapture::kOpConfigure, p, out), kRejectUnavailable));
  tlv(p, kCrit | 0x60, {1});                                                          // an unknown critical tag
  CHECK(rejectedAs(raw(c, LogicCapture::kOpConfigure, p, out), kRejectUnsupported) && out == Bytes{kCrit | 0x60});
  tlv(p, cap::kTlvConfigureSegments, {1});                                            // and a short segments
  CHECK(rejectedAs(raw(c, LogicCapture::kOpConfigure, p, out), kRejectMalformed));
  p.clear();
  tlv(p, kCrit | cap::kTlvConfigureRate, u32(0));                                     // 0 Hz: excluded (1 Hz or more)
  CHECK(rejectedAs(raw(c, LogicCapture::kOpQuery, p, out), kRejectMalformed));
}

// rate_limit (capture §3.5, configure §3.3): a one-shot rate above the declared limit for the channel count (8 lines
// 100 MHz, 16 lines 48 MHz) is set to the limit, not run at the rate asked (160 MHz was taken for any count).
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
      for (size_t at = 0; at + 2 <= out.size(); at += 2u + out[at + 1])
        if (out[at] == cap::kTlvConfigureAnswerActualRate) rate.assign(out.begin() + at + 2, out.begin() + at + 10);
      // the fake divider is a whole number: 160 / 2 = 80 MHz for 100 MHz, 160 / 3 = 53.3 MHz for 48 MHz
      CHECK(rate.size() == 8 && getU32(rate.data()) / getU32(rate.data() + 4) == (channels == 8 ? 80000000u : 53333333u));
    }
  }
}

int main() {
  testRateLimit();
  testSentCritical();
  testConfigureOrder();
  testStreamingWithoutStages();
  testOverLimitRoundsDown();
  testEveryModeAtTheLowestRate();
  printf("capture: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
