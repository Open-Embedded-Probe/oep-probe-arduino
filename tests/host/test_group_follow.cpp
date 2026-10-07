// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host test: a capture-group on the ESP32-P4 (oep-spec oep-if-capture §4.1) - the logic capture as the trigger track on
// fakes of the PARLIO RX driver and the heap (OEP_HOST_FAKE_PARLIO), the analog capture following it on a fake of the
// ADC continuous driver (OEP_HOST_FAKE_ESP_ADC). The follower hears of the trigger only once the logic's DMA chunk that
// holds it is done and loop() has come round; its values run on into its ring meanwhile. The bench's case (P4,
// 87f6d40): logic 1 MHz P 1000, analog 20 kHz 1024 samples P_k 20, an edge in the logic's first chunk: the analog track
// stopped in state 6, error 2 - its ring was the segment alone (1024 frames), and by the time the trigger came (a
// chunk of 4 KiB at w = 1 is 32 ms at 1 MHz, 51 ms at the lowest rate, and loop() on top) the frames after it had
// written over the segment's first ones.
#include <stdio.h>

#include <vector>

#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/queue.h>

#include "OepAnalog.h"
#include "OepCapture.h"
#include "OepCaptureGroup.h"
#include "OepEndpoint.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace cap = reg::fixture_logic;
namespace ana = reg::fixture_analog;
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

struct Rig {
  NullStream stream;
  uint8_t rx[600], tx[600];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kVendorBulk, 0};
  PinTable pins{0xFFFFFFull};
  LogicCapture logic{ep, pins};
  AnalogCapture analog{ep, 0xFFull << 16};
  CaptureGroup group{ep};
  uint8_t out[600];
  Rig() {
    g_fake_heap = FakeHeap{};
    g_fake_heap.internal_free = 512 * 1024;
    g_fake_heap.psram_total = g_fake_heap.psram_free = size_t(32) << 20;
    g_fake_adc = FakeAdc{};
    ep.add(logic);    // fn 1
    ep.add(analog);   // fn 2
    ep.add(group);    // fn 3
    analog.setPins(&pins, 9);
    group.addTrack(logic, logic);
    group.addTrack(analog, analog);
    RoleAssignment l = {1, 0, 2};    // logic role 0 on channel 2
    logic.planApply(&l, 1);
    RoleAssignment a = {2, 0, 16};   // analog role 0 on channel 16
    analog.planApply(&a, 1);
  }
  ~Rig() {   // the one PARLIO RX unit back
    logic.planRelease();
    analog.planRelease();
  }
  Result op(Interface &i, uint8_t code, const Bytes &p = {}) { return i.handle(code, p.data(), p.size(), out, sizeof out); }
  void loop() {   // the firmware's loop(): capture, analog, group (Esp32P4.h)
    logic.poll();
    analog.poll();
    group.poll();
  }
};

// The PARLIO's DMA into the logic's ring, a descriptor (3968 bytes) a partial receive, its harvest task run by the test.
struct Dma {
  struct Idle {};
  size_t at = 0;
  Dma() { g_fake_tasks_deferred = true; g_fake_task_fn = nullptr; }
  ~Dma() { g_fake_tasks_deferred = false; }
  void put(const Bytes &bytes) {
    memcpy(g_fake_parlio_buffer + at, bytes.data(), bytes.size());
    parlio_rx_event_data_t e = {g_fake_parlio_buffer + at, bytes.size()};
    g_fake_parlio_callbacks.on_partial_receive(nullptr, &e, g_fake_parlio_context);
    at = (at + bytes.size()) % g_fake_parlio_size;
  }
  void run() {
    if (!g_fake_task_fn) return;
    g_fake_queue_empty = [] { throw Idle{}; };
    try { g_fake_task_fn(g_fake_task_arg); } catch (const Idle &) {}
    g_fake_queue_empty = nullptr;
  }
};

// Time goes on by `us`: the ADC converts at 20 kHz meanwhile (frame f's value is f & 0xFFF), loop() comes round every
// 5 ms (the driver's pool drained, the group's trigger handed on).
struct Clock {
  Rig &rig;
  uint64_t us = 0;        // since the analog's start
  uint32_t frames = 0;    // conversions put in the pool
  void run(uint64_t span_us, bool loop = true) {
    for (uint64_t end = us + span_us; us < end;) {
      const uint64_t step = end - us < 5000 ? end - us : 5000;
      advanceMicros(static_cast<uint32_t>(step));
      us += step;
      while (static_cast<uint64_t>(frames) * 50 + 3200 < us) { g_fake_adc.put(0, frames & 0xFFF); ++frames; }
      if (loop) rig.loop();
    }
  }
};

static void testFollowerHearsLate() {
  for (const bool short_ring : {false, true}) {
    Rig rig;
    Dma dma;
    Bytes p;
    tlv(p, kCrit | cap::kTlvConfigureMode, {cap::kModeOneShot});
    tlv(p, kCrit | cap::kTlvConfigureRate, u32(LogicCapture::kMinHz));   // 627451 Hz: a 3968-byte chunk is 50.6 ms
    tlv(p, cap::kTlvConfigureSamples, u32(8192));
    tlv(p, cap::kTlvConfigureTrigger, {cap::kTriggerEdge, 0, 0, 0, 0, 0});   // rising
    tlv(p, cap::kTlvConfigurePretrigger, u32(1000));
    CHECK(ok(rig.op(rig.logic, cap::kOpConfigure, p)));
    p.clear();
    tlv(p, kCrit | ana::kTlvConfigureMode, {ana::kModeOneShot});
    tlv(p, kCrit | ana::kTlvConfigureRate, u32(20000));
    tlv(p, ana::kTlvConfigureSamples, u32(1024));
    CHECK(ok(rig.op(rig.analog, ana::kOpConfigure, p)));
    // P_k = ceil(1000 x 20000 / 627451) = 32; the logic's latency a chunk (51 ms) and loop() (100 ms)
    CHECK(rig.logic.trackLatencyNs() > 150000000 && rig.logic.trackLatencyNs() < 152000000);
    CHECK(ok(rig.op(rig.group, grp::kOpBind, {2, 1, 0, 2, 0, grp::kTlvBindTriggerTrack, 2, 0, 1, 0})));
    if (short_ring) rig.analog.trackKeep(32, 0);   // a ring of the segment and the room only: about what 87f6d40 had
    CHECK(ok(rig.op(rig.group, grp::kOpStart)));
    Clock clock{rig};
    clock.run(10000);   // the analog's first frames: P_k in, the logic starts
    CHECK(rig.logic.trackState() == cap::kStateWaiting);
    // the logic's first chunk, as the bench's: the line low for its first 40 samples, then high - the rising edge at
    // sample 40, 64 us after the logic's start; the chunk is done 3968 bytes (50.6 ms) on, and the harvest finds it then
    clock.run(50600);
    Bytes chunk(3968, 0xFF);
    for (int k = 0; k < 5; ++k) chunk[k] = 0;
    dma.put(chunk);
    dma.run();
    clock.run(20000);   // loop() comes round a few times: the trigger handed on, the analog cuts its segment
    dma.put(Bytes(3968, 0xFF));
    dma.run();
    clock.run(60000);
    if (short_ring) {   // the trigger came after the segment's first frames were written over: state 6, error 2
      CHECK(rig.analog.trackState() == ana::kStateError && rig.analog.trackError() == ana::kErrorStorage);
      continue;
    }
    CHECK(rig.logic.trackState() == cap::kStateDone);
    CHECK(rig.analog.trackState() == ana::kStateDone);
    Result r = rig.op(rig.analog, ana::kOpSegments, {0, 0, 0, 0});
    CHECK(ok(r) && r.length >= 2 + 37 && rig.out[1] == 1);
    const uint32_t samples = getU32(rig.out + 2 + 12), index = getU32(rig.out + 2 + 28);
    CHECK(samples == 1024 && index == 32);
    // the analog's frame `index` into its segment is the group's trigger (within a frame), and the segment's values are
    // its frames in order - none written over while the trigger was on its way
    const uint64_t seg_ns = getU64(rig.out + 2 + 16);
    const uint32_t generation = getU32(rig.out + 2 + 33);
    r = rig.op(rig.group, grp::kOpStatus);
    CHECK(ok(r));
    const uint64_t trigger_ns = getU64(rig.out + 9);
    const int64_t d = static_cast<int64_t>(seg_ns + 32ull * 50000 - trigger_ns);
    CHECK(d > -50000 && d < 50000);
    Bytes rq(16);
    putU32(rq.data(), generation);
    putU64(rq.data() + 4, 0);
    putU32(rq.data() + 12, 2048);
    r = rig.op(rig.analog, ana::kOpRead, rq);
    CHECK(ok(r) && getU32(rig.out + 9) >= 256);
    const uint32_t n = getU32(rig.out + 9) / 2;
    bool in_order = true;
    for (uint32_t i = 1; i < n; ++i) in_order &= getU16(rig.out + 13 + 2 * i) == ((getU16(rig.out + 13) + i) & 0xFFF);
    CHECK(in_order);
  }
}

// bind (§4.1): a follower whose ring cannot hold the segment and what comes in during the trigger track's latency is
// refused unavailable cause 2 with its fn, like a P_k it cannot keep - 16384 frames at most on the P4 (32 KiB).
static void testBindRefusesLongLatency() {
  Rig rig;
  Bytes p;
  tlv(p, kCrit | cap::kTlvConfigureMode, {cap::kModeOneShot});
  tlv(p, kCrit | cap::kTlvConfigureRate, u32(LogicCapture::kMinHz));
  tlv(p, cap::kTlvConfigureSamples, u32(8192));
  tlv(p, cap::kTlvConfigureTrigger, {cap::kTriggerEdge, 0, 1, 0, 0, 0});
  tlv(p, cap::kTlvConfigurePretrigger, u32(1000));
  CHECK(ok(rig.op(rig.logic, cap::kOpConfigure, p)));
  for (const uint32_t samples : {13000u, 16000u}) {   // 151 ms at 20 kHz: 3021 frames, 129 of room
    p.clear();
    tlv(p, kCrit | ana::kTlvConfigureMode, {ana::kModeOneShot});
    tlv(p, kCrit | ana::kTlvConfigureRate, u32(20000));
    tlv(p, ana::kTlvConfigureSamples, u32(samples));
    CHECK(ok(rig.op(rig.analog, ana::kOpConfigure, p)));
    const Result r = rig.op(rig.group, grp::kOpBind, {2, 1, 0, 2, 0, grp::kTlvBindTriggerTrack, 2, 0, 1, 0});
    if (samples == 13000) {
      CHECK(ok(r));
      CHECK(ok(rig.op(rig.group, grp::kOpBind, {0})));
    } else {
      CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnavailable && r.length >= 9 &&
            rig.out[3] == reg::core::kUnavailableCauseLimit && getU16(rig.out + 7) == 2);
    }
  }
}

int main() {
  testFollowerHearsLate();
  testBindRefusesLongLatency();
  printf("group-follow: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
