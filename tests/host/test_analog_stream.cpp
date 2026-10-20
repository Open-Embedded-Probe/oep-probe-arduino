// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host test: the analog capture's streaming (oep-spec oep-if-capture §2.1, §3.4) on the ESP build with a fake of the ADC
// continuous driver (OEP_HOST_FAKE_ESP_ADC) - configure's answer (stream_buffer, no actual_samples), start only once
// subscribed, the pushes (position, len, data, generation and segment_info; complete frames only, one segment each),
// a new segment dropped whole when the ring has no room (rule 1: the position skips it, status flags bit0, the next
// segment's gap flag), the segments op, stop's event and what is still pushed after it, read's gap.
#include <stdio.h>
#include <string.h>

#include <vector>

#include <esp_heap_caps.h>
#include <esp_timer.h>

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

static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static void tlv(Bytes &p, uint8_t tag, const Bytes &v) {
  p.push_back(tag);
  p.push_back(static_cast<uint8_t>(v.size()));
  p.push_back(static_cast<uint8_t>(v.size() >> 8));
  p.insert(p.end(), v.begin(), v.end());
}
static Bytes u32(uint32_t v) { Bytes b(4); putU32(b.data(), v); return b; }
static constexpr uint8_t kCrit = 0x80;
// the value of TLV `tag` in out[from, to), or nullptr
static const uint8_t *find(const uint8_t *out, size_t from, size_t to, uint8_t tag, size_t *len = nullptr) {
  for (size_t at = from; at + kTlvHeader <= to;) {
    const size_t n = getU16(out + at + 1);
    if ((out[at] & ~kTagCritical) == tag) { if (len) *len = n; return out + at + kTlvHeader; }
    at += kTlvHeader + n;
  }
  return nullptr;
}

struct Rig {
  NullStream stream;
  uint8_t rx[600], tx[600];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kVendorBulk, 0};
  PinTable pins{0xFFFFFFull};
  AnalogCapture analog{ep, 0xFFull << 16};
  uint8_t out[600];
  uint32_t fed = 0;   // conversions put in the pool (one channel: frames)
  Rig() {
    g_fake_heap = FakeHeap{};
    g_fake_heap.internal_free = 512 * 1024;
    g_fake_heap.psram_total = g_fake_heap.psram_free = size_t(32) << 20;
    g_fake_adc = FakeAdc{};
    ep.add(analog);   // fn 1
    analog.setPins(&pins, 9);
    RoleAssignment a = {1, 0, 16};   // role 0 on channel 16 (ADC1 channel 0)
    analog.planApply(&a, 1);
  }
  ~Rig() { analog.planRelease(); }
  Result op(uint8_t code, const Bytes &p = {}) { return analog.handle(code, p.data(), p.size(), out, sizeof out); }
  void feed(uint32_t frames) {   // frame k's value: k & 0xFFF
    for (uint32_t k = 0; k < frames; ++k, ++fed) g_fake_adc.put(0, fed & 0xFFF);
    analog.poll();
  }
  // every push now: (position, bytes) checked against the frames' values, segment_info's position and samples
  struct Push { uint64_t position; size_t len; uint64_t seg_position; uint32_t seg_samples; };
  std::vector<Push> pullAll() {
    std::vector<Push> got;
    uint8_t buf[520];
    for (size_t n; (n = analog.pull(buf, sizeof buf)) != 0;) {
      Push p{getU64(buf), getU16(buf + 8), 0, 0};
      CHECK(10 + p.len <= n);
      bool same = true;
      for (size_t k = 0; k < p.len / 2; ++k)
        if (getU16(buf + 10 + 2 * k) != ((p.position / 2 + k) & 0xFFF)) same = false;
      CHECK(same);
      const uint8_t *g = find(buf, 10 + p.len, n, ana::kTlvDataGeneration);
      CHECK(g != nullptr);
      size_t len = 0;
      const uint8_t *si = find(buf, 10 + p.len, n, ana::kTlvDataSegmentInfo, &len);
      CHECK(si != nullptr && len == 24);
      if (si) { p.seg_position = getU64(si); p.seg_samples = getU32(si + 8); }
      got.push_back(p);
    }
    return got;
  }
};

static constexpr uint32_t kSeg = 4096;   // one channel: 8192 bytes a segment

static void configure(Rig &r) {
  Bytes p;
  tlv(p, kCrit | ana::kTlvConfigureMode, {ana::kModeStreaming});
  tlv(p, kCrit | ana::kTlvConfigureRate, u32(20000));
  const Result c = r.op(ana::kOpConfigure, p);
  CHECK(ok(c));
  CHECK(find(r.out, 0, c.length, ana::kTlvConfigureAnswerActualSamples) == nullptr);
  const uint8_t *sb = find(r.out, 0, c.length, ana::kTlvConfigureAnswerStreamBuffer);
  CHECK(sb && getU32(sb) == 8 * 8192u);
}

static void testPushes() {
  Rig r;
  configure(r);
  CHECK(!ok(r.op(ana::kOpStart)));   // not subscribed: wrong state
  r.analog.subscribe(true);
  CHECK(ok(r.op(ana::kOpStart)));
  r.feed(kSeg + 100);
  CHECK(r.analog.pending() == (kSeg + 100) * 2u);
  auto got = r.pullAll();
  uint64_t at = 0;
  bool contiguous = true, within = true;
  for (auto &p : got) {
    if (p.position != at) contiguous = false;
    at += p.len;
    // one segment a push
    if (p.position / 8192 != (p.position + p.len - 1) / 8192 || p.seg_position != p.position / 8192 * 8192) within = false;
  }
  CHECK(contiguous && within);
  CHECK(at == (kSeg + 100) * 2u);
  CHECK(!got.empty() && got.back().seg_samples == 100);
  CHECK(r.analog.pending() == 0);
  // status: one segment finished, the bytes so far, nothing dropped
  CHECK(ok(r.op(ana::kOpStatus)));
  CHECK(r.out[0] == ana::kStateCapturing && getU32(r.out + 1) == 1 && getU64(r.out + 5) == (kSeg + 100) * 2u);
  CHECK((r.out[13] & ana::kStatusFlagDropped) == 0);
  // read: nothing kept to read back (gap)
  Bytes rd = u32(getU32(r.out + 14));
  rd.resize(14);
  putU16(rd.data() + 12, 256);
  CHECK(ok(r.op(ana::kOpRead, rd)));
  CHECK((r.out[8] & reg::common::kReadFlagsGap) && getU16(r.out + 9) == 0);
  // stop: the stopped event, then the frames after the last push still go
  r.feed(50);
  CHECK(ok(r.op(ana::kOpStop)));
  CHECK(ok(r.op(ana::kOpStatus)) && r.out[0] == ana::kStateConfigured);
  got = r.pullAll();
  CHECK(got.size() == 1 && got[0].position == (kSeg + 100) * 2u && got[0].len == 100);
}

static void testDropped() {
  Rig r;
  configure(r);
  r.analog.subscribe(true);
  CHECK(ok(r.op(ana::kOpStart)));
  r.feed(9 * kSeg + 10);   // nothing pushed: segments 0..7 stored, 8 dropped, 9 begun with 0 still unsent: dropped
  CHECK(r.analog.pending() == 8 * kSeg * 2u);
  CHECK(ok(r.op(ana::kOpStatus)) && (r.out[13] & ana::kStatusFlagDropped));
  auto got = r.pullAll();
  uint64_t bytes = 0;
  for (auto &p : got) bytes += p.len;
  CHECK(bytes == 8 * kSeg * 2u && got.back().position + got.back().len == 8 * kSeg * 2u);
  r.feed(2 * kSeg);   // segment 9's rest, 10 (stored: room again), 11's first 10
  got = r.pullAll();
  CHECK(!got.empty() && got[0].position == 10 * kSeg * 2u);   // 8 and 9 skipped: the position jumps
  // segments: the newest finished, the dropped ones without a record, 10's gap flag
  CHECK(ok(r.op(ana::kOpSegments, u32(0))));
  const uint8_t count = r.out[1];
  CHECK(count >= 1);
  bool saw10 = false;
  for (uint8_t k = 0; k < count; ++k) {
    const uint8_t *s = r.out + 2 + 37 * k;
    CHECK(getU32(s) != 8 && getU32(s) != 9);
    CHECK(getU64(s + 4) == static_cast<uint64_t>(getU32(s)) * 8192u && getU32(s + 12) == kSeg);
    if (getU32(s) == 10) { saw10 = true; CHECK(s[32] & ana::kSegmentFlagGap); }
    else CHECK((s[32] & ana::kSegmentFlagGap) == 0);
  }
  CHECK(saw10);
}

static void testRefusals() {
  Rig r;
  Bytes p;   // streaming takes no samples (capture §3.3)
  tlv(p, kCrit | ana::kTlvConfigureMode, {ana::kModeStreaming});
  tlv(p, kCrit | ana::kTlvConfigureRate, u32(20000));
  tlv(p, kCrit | ana::kTlvConfigureSamples, u32(1000));
  CHECK(!ok(r.op(ana::kOpConfigure, p)));
  p.clear();   // a crossing trigger: not in streaming
  tlv(p, kCrit | ana::kTlvConfigureMode, {ana::kModeStreaming});
  tlv(p, kCrit | ana::kTlvConfigureRate, u32(20000));
  tlv(p, kCrit | ana::kTlvConfigureTrigger, {ana::kTriggerCrossUp, 0, 0, 8, 0, 0});
  CHECK(!ok(r.op(ana::kOpConfigure, p)));
}

int main() {
  testPushes();
  testDropped();
  testRefusals();
  printf("analog stream: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
