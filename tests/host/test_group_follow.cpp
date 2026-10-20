// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host test: a capture-group's trigger source on the ESP32-P4 (oep-spec oep-if-capture §4.4) - the logic capture on
// fakes of the PARLIO RX driver and the heap (OEP_HOST_FAKE_PARLIO), the analog capture on a fake of the ADC continuous
// driver (OEP_HOST_FAKE_ESP_ADC), both linked. The judging track hands the event on from the task that finds it
// (eventFound: the logic's harvest, the analog's drain) - a follower cuts its segment without the group's poll, so the
// latency a follower keeps is the judging track's chunk and a task's round (GroupTrack::kTaskNs), not loop()'s 100 ms.
// The bench's case behind the follower's ring (P4, 87f6d40): an edge in the logic's first chunk, the analog's ring the
// segment alone - the frames after the trigger wrote over its first ones (state 6, error 2).
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
  bool group_polls = true;   // false: loop() leaves the group's poll out (the handoff must not need it)
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
    if (group_polls) group.poll();
  }
  // both linked, one-shot: the logic at its lowest rate, 8192 samples, pretrigger 1000; the analog at 20 kHz
  void configure(uint32_t analog_samples = 1024, uint32_t analog_pre = 32) {
    Bytes p;
    tlv(p, kCrit | cap::kTlvConfigureMode, {cap::kModeOneShot});
    tlv(p, kCrit | cap::kTlvConfigureRate, u32(LogicCapture::kMinHz));   // 627451 Hz: a 3968-byte chunk is 50.6 ms
    tlv(p, cap::kTlvConfigureSamples, u32(8192));
    tlv(p, cap::kTlvConfigureTrigger, {cap::kTriggerLinked, 0, 0, 0, 0, 0});
    tlv(p, cap::kTlvConfigurePretrigger, u32(1000));
    CHECK(ok(op(logic, cap::kOpConfigure, p)));
    p.clear();
    tlv(p, kCrit | ana::kTlvConfigureMode, {ana::kModeOneShot});
    tlv(p, kCrit | ana::kTlvConfigureRate, u32(20000));
    tlv(p, ana::kTlvConfigureSamples, u32(analog_samples));
    tlv(p, ana::kTlvConfigureTrigger, {ana::kTriggerLinked, 0, 0, 0, 0, 0});
    tlv(p, ana::kTlvConfigurePretrigger, u32(analog_pre));
    CHECK(ok(op(analog, ana::kOpConfigure, p)));
  }
  // bind both with one condition: on the logic (fn 1) or the analog (fn 2), its type and arguments
  Result bind(uint16_t fn, uint8_t type, const Bytes &args) {
    Bytes p = {2, 1, 0, 2, 0};
    tlv(p, kCrit | grp::kTlvBindTriggerSource, {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0});
    tlv(p, kCrit | grp::kTlvBindStage, {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    Bytes c = {0, 0, 0, grp::kCondSrcTrack, uint8_t(fn), uint8_t(fn >> 8), 0, type};
    c.insert(c.end(), args.begin(), args.end());
    tlv(p, kCrit | grp::kTlvBindCond, c);
    return op(group, grp::kOpBind, p);
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
    g_fake_on_delay = [] { throw Idle{}; };
    try { g_fake_task_fn(g_fake_task_arg); } catch (const Idle &) {}
    g_fake_queue_empty = nullptr;
    g_fake_on_delay = nullptr;
  }
};

// Time goes on by `us`: the ADC converts at 20 kHz meanwhile (frame f's value is f & 0xFFF); with `logic`, the PARLIO
// fills a chunk (3968 bytes, 50.6 ms at the lowest rate, w 1) of `level` whenever one is due and the harvest takes it;
// loop() comes round every 5 ms.
struct Clock {
  Rig &rig;
  Dma &dma;
  uint64_t us = 0;        // since the start
  uint32_t frames = 0;    // conversions put in the pool
  uint64_t logic_bytes = 0;
  bool logic = false;
  uint8_t level = 0;
  void run(uint64_t span_us) {
    for (uint64_t end = us + span_us; us < end;) {
      const uint64_t step = end - us < 5000 ? end - us : 5000;
      advanceMicros(static_cast<uint32_t>(step));
      us += step;
      while (static_cast<uint64_t>(frames) * 50 + 3200 < us) { g_fake_adc.put(0, frames & 0xFFF); ++frames; }
      while (logic && (logic_bytes + 3968) * 8 * 1000000ull / LogicCapture::kMinHz <= us) {
        dma.put(Bytes(3968, level));
        logic_bytes += 3968;
        dma.run();
      }
      rig.loop();
    }
  }
};

// The logic judges (a rising edge in its first chunk); the analog follows. The harvest that finds the edge hands it to
// the analog at once: with the group's poll left out of loop(), the analog still cuts its segment around it, within a
// frame of the event, its values in order; the latency kept is a chunk and a task's round (60.6 ms, was 150.6).
static void testLogicJudges() {
  for (const bool short_ring : {false, true}) {
    Rig rig;
    Dma dma;
    rig.configure();
    CHECK(rig.logic.trackLatencyNs() > 61000000 && rig.logic.trackLatencyNs() < 62000000);   // a 4032-byte chunk: 51.4 ms
    CHECK(ok(rig.bind(1, grp::kTriggerCondEdge, {0})));   // rising, on the logic's role 0
    if (short_ring) rig.analog.trackKeep(32, 0);   // a ring of the segment and the room only: about what 87f6d40 had
    CHECK(ok(rig.op(rig.group, grp::kOpStart)));
    Clock clock{rig, dma};
    clock.run(10000);   // the analog's first frames: its pretrigger in, the logic starts (the group's poll)
    CHECK(rig.logic.trackState() == cap::kStateWaiting);
    rig.group_polls = false;   // from here the handoff alone
    clock.run(50600);
    Bytes chunk(3968, 0xFF);   // the line low for its first 1600 samples (past the pretrigger: arm 1), then high
    for (int k = 0; k < 200; ++k) chunk[k] = 0;
    dma.put(chunk);
    dma.run();
    dma.put(Bytes(3968, 0xFF));
    dma.run();
    clock.run(80000);
    if (short_ring) {   // the frames after the trigger wrote over the segment's first ones: state 6, error 2
      CHECK(rig.analog.trackState() == ana::kStateError && rig.analog.trackError() == ana::kErrorStorage);
      continue;
    }
    CHECK(rig.logic.trackState() == cap::kStateDone);
    CHECK(rig.analog.trackState() == ana::kStateDone);
    Result r = rig.op(rig.analog, ana::kOpSegments, {0, 0, 0, 0});
    CHECK(ok(r) && r.length >= 2 + 37 && rig.out[1] == 1);
    const uint32_t samples = getU32(rig.out + 2 + 12), index = getU32(rig.out + 2 + 28);
    CHECK(samples == 1024 && index == 32);
    const uint64_t seg_ns = getU64(rig.out + 2 + 16);
    const uint32_t generation = getU32(rig.out + 2 + 33);
    rig.group_polls = true;
    r = rig.op(rig.group, grp::kOpStatus);   // the event as the group records it (its poll)
    CHECK(ok(r) && r.length == 13 + 17 && getU16(rig.out + 16) == 1);
    const uint64_t trigger_ns = getU64(rig.out + 18);
    const int64_t d = static_cast<int64_t>(seg_ns + 32ull * 50000 - trigger_ns);
    CHECK(d > -50000 && d < 50000);
    Bytes rq(14);   // generation(u32) position(u64) max(u16)
    putU32(rq.data(), generation);
    putU64(rq.data() + 4, 0);
    putU16(rq.data() + 12, 2048);
    r = rig.op(rig.analog, ana::kOpRead, rq);
    CHECK(ok(r) && getU16(rig.out + 9) >= 256);
    const uint32_t n = getU16(rig.out + 9) / 2;
    bool in_order = true;
    for (uint32_t i = 1; i < n; ++i) in_order &= getU16(rig.out + 11 + 2 * i) == ((getU16(rig.out + 11) + i) & 0xFFF);
    CHECK(in_order);
  }
}

// The analog judges (its ramp crosses 2000 upward at frame 2000, 100 ms in); the logic follows, all high but for a
// low stretch so its cut can be read: the drain that finds the crossing hands it to the logic at once (the group's
// poll left out), and the logic's segment has its pretrigger before the event, within a sample period of the analog
// (50 us) - its latency kept a conversion frame and a task's round (13.2 ms at 20 kHz).
static void testAnalogJudges() {
  Rig rig;
  Dma dma;
  rig.configure(1024, 32);
  CHECK(rig.analog.trackLatencyNs() == 10000000ull + 64ull * 1000000000 / 20000);
  const Bytes cross = {0xD0, 0x07, 0, 0, 0, 0, 0, 0};   // value 2000, hysteresis 0
  CHECK(ok(rig.bind(2, grp::kTriggerCondCrossUp, cross)));
  CHECK(ok(rig.op(rig.group, grp::kOpStart)));
  Clock clock{rig, dma};
  clock.logic = true;
  clock.level = 0xFF;
  clock.run(60000);   // the logic's pretrigger in: the analog starts judging (the group's poll)
  CHECK(rig.analog.trackState() == ana::kStateWaiting);
  rig.group_polls = false;
  clock.run(300000);
  CHECK(rig.analog.trackState() == ana::kStateDone);
  CHECK(rig.logic.trackState() == cap::kStateDone);
  Result r = rig.op(rig.logic, cap::kOpSegments, {0, 0, 0, 0});
  CHECK(ok(r) && r.length >= 2 + 37 && getU32(rig.out + 2 + 28) == 1000);   // trigger_index = its pretrigger
  const uint64_t logic_ns = getU64(rig.out + 2 + 16) + 1000ull * 1000000000 / LogicCapture::kMinHz;
  r = rig.op(rig.analog, ana::kOpSegments, {0, 0, 0, 0});
  CHECK(ok(r) && getU32(rig.out + 2 + 28) == 32);
  const uint64_t analog_ns = getU64(rig.out + 2 + 16) + 32ull * 50000;
  const int64_t d = static_cast<int64_t>(logic_ns - analog_ns);
  CHECK(d > -50000 && d < 50000);
}

// bind (§4.1, §4.4): a follower whose ring cannot hold its segment and what comes in during the judging track's latency
// is refused unavailable cause 2 with its fn - 16384 frames at most for the P4's analog (32 KiB); the logic's 60.6 ms
// (1212 frames at 20 kHz) and the room (129) leave 15043 samples.
static void testBindRefusesLongLatency() {
  for (const uint32_t samples : {15000u, 16000u}) {
    Rig rig;
    rig.configure(samples, 32);
    const Result r = rig.bind(1, grp::kTriggerCondEdge, {0});
    if (samples == 15000) {
      CHECK(ok(r));
    } else {
      CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnavailable && r.length >= 9 &&
            rig.out[3] == reg::core::kUnavailableCauseLimit && getU16(rig.out + 7) == 2);
    }
  }
}

int main() {
  testLogicJudges();
  testAnalogJudges();
  testBindRefusesLongLatency();
  printf("group-follow: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
