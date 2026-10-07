// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the gate between the classic ESP32's core-0 sampler and the SWIO frames of loop()'s core (OepWireGate.h),
// with two threads standing for the two cores, and SwioPhy's frames through it (OEP_HOST_FAKE_SWIO).
// - the gate's rules: with no window a frame goes at once; a window opens at once with no frame; a frame announced
//   while a window is open waits for the sampler's stop, and the sampler stops until the frame has ended;
// - no frame and no sample at once, under a stress of both sides;
// - a request's frames (DmiPhy::read / write) go on while a window stays open, and none meets a sample;
// - the console (DmConsole, dmseq, on Ch32Dm and SwioPhy against a simulated target) delivers a line the host wrote
//   while a window stays open the whole time: 0.0.29-dev f32a3ef paused the console's reading for a whole window, so a
//   command sent after a capture's start reached the target only once the capture had ended (the classic ESP32 bench
//   recorded nothing the command or a debug reset caused).
#include <stdio.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "OepCh32Dm.h"
#include "OepDmConsole.h"
#include "OepSwioPhy.h"
#include "OepWireGate.h"
#include "fake_swio_io.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;

static std::atomic<int> failures{0}, checks{0};
#define CHECK(cond)                                                                     \
  do {                                                                                  \
    ++checks;                                                                           \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// A sample being taken (the sampler thread between yieldFrame() and the end of its GPIO read); frames counted, and
// those met during a sample.
static std::atomic<int> g_sampling{0}, g_frames{0}, g_overlaps{0};
static void frameSeen() {
  ++g_frames;
  if (g_sampling.load()) ++g_overlaps;
}

// The target behind the fake wire: the SDI configuration, a debug module with a running hart, DATA0 / DATA1 and
// PROGBUF0 (the PHY's write check). Touched by the main thread only (the frames run there).
struct Target {
  bool sdi = false, dmactive = false;
  uint32_t data0 = 0, data1 = 0, progbuf0 = 0;
  bool read(uint8_t a, uint32_t &v) {
    advanceMicros(45);
    if (!sdi) { v = 0xffffffffu; return true; }
    if (!dmactive && a != 0x10 && a != 0x7d && a != 0x7e) { v = 0; return true; }
    switch (a) {
      case 0x04: v = data0; break;
      case 0x05: v = data1; break;
      case 0x10: v = dmactive ? 1u : 0u; break;
      case 0x11: v = 2 | (1u << 7) | (3u << 10); break;   // version 2, authenticated, all running
      case 0x12: v = 0x0002'1000u | 0x380; break;
      case 0x16: v = 2; break;
      case 0x20: v = progbuf0; break;
      case 0x7d: case 0x7e: v = 0x5aa50400u; break;
      default: v = 0; break;
    }
    return true;
  }
  void write(uint8_t a, uint32_t v) {
    advanceMicros(45);
    if (a == 0x7d && v == 0x5aa50400u) sdi = true;
    if (!sdi) return;
    if (a == 0x10) dmactive = v & 1;
    if (a == 0x04) data0 = v;
    if (a == 0x05) data1 = v;
    if (a == 0x20) progbuf0 = v;
  }
} g_target;

bool fakeSwioRead(uint8_t address, uint32_t &value) {
  frameSeen();
  return g_target.read(address, value);
}
void fakeSwioWrite(uint8_t address, uint32_t value, bool) {
  frameSeen();
  g_target.write(address, value);
}
bool fakeSwioLineHigh() { advanceMicros(2000); return true; }

// A dmseq target as the reference one (oep-spec experiments/dm-console-seq DmSeq.h): its sketch calls available() now
// and then, which takes an answer and, idle, posts an empty frame; input rides on the answers.
struct SeqTarget {
  uint32_t &d0;
  uint8_t s = 0, last_h = 1;
  bool posted = false, syn = true;
  uint32_t w0 = 0;
  std::vector<uint8_t> rx;
  explicit SeqTarget(uint32_t &data0) : d0(data0) {}
  void post() {
    uint8_t b[2] = {uint8_t(0x80 | (s << 5) | (last_h << 4) | (syn ? 0x08 : 0)), 0};
    b[1] = DmConsole::crc8(b, 1);
    w0 = d0 = b[0] | uint32_t(b[1]) << 8;
    posted = true;
  }
  void service() {
    if (!posted) return;
    const uint32_t w = d0;
    if (w & 0x80u) { if (w != w0) d0 = w0; return; }   // target rule 0
    if (w == 0) return;
    const uint8_t a[4] = {uint8_t(w), uint8_t(w >> 8), uint8_t(w >> 16), uint8_t(w >> 24)};
    const uint8_t k = (a[0] >> 5) & 1, h = (a[0] >> 4) & 1, m = a[0] & 7;
    if (k != s || m > 2 || DmConsole::crc8(a, 1 + m) != a[1 + m]) { d0 = w0; return; }
    posted = syn = false;
    s ^= 1;
    if (m && h != last_h) { rx.insert(rx.end(), a + 1, a + 1 + m); last_h = h; }
  }
  void available() { service(); if (!posted) post(); }
};

// The sampler: one window open until `stop`, sampling as SamplerCapture::run does (yieldFrame, then the read).
struct Sampler {
  WireGate &gate;
  std::atomic<bool> stop{false}, open{false};
  std::atomic<long> samples{0}, stops{0};
  std::thread thread;
  explicit Sampler(WireGate &g) : gate(g) {
    thread = std::thread([this] {
      gate.beginWindow();
      open = true;
      while (!stop.load()) {
        if (gate.yieldFrame()) ++stops;
        g_sampling = 1;
        for (volatile int i = 0; i < 20; ++i) {}   // the GPIO.in read
        g_sampling = 0;
        ++samples;
      }
      gate.endWindow();
      open = false;
    });
    while (!open.load()) std::this_thread::yield();
  }
  void end() { stop = true; thread.join(); }
};

int main() {
  // ---- the rules, one thread ----
  {
    WireGate g;
    CHECK(!g.windowOpen() && !g.framing());
    g.frameBegin();                              // no window: at once
    CHECK(g.framing());
    g.frameEnd();
    CHECK(!g.framing());
    g.beginWindow();                             // no frame: at once
    CHECK(g.windowOpen() && !g.yieldFrame());
    g.endWindow();
    CHECK(!g.windowOpen());
  }

  // ---- a frame announced in a window waits for the sampler's stop; the sampler waits for its end ----
  {
    WireGate g;
    std::atomic<int> phase{0};   // 1: the window open
    std::atomic<bool> frame_started{false}, frame_done{false}, sampler_back{false};
    std::thread sampler([&] {
      g.beginWindow();
      phase = 1;
      while (!g.framing()) std::this_thread::yield();   // the frame has been announced (one sample more)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      CHECK(!frame_started.load());              // not before the sampler stopped for it
      CHECK(g.yieldFrame());                     // returns once the frame has ended
      CHECK(frame_done.load());
      sampler_back = true;
      g.endWindow();
    });
    while (phase.load() < 1) std::this_thread::yield();
    std::thread frame([&] {
      g.frameBegin();
      frame_started = true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      CHECK(!sampler_back.load());               // the sampler is stopped while the frame is on the line
      frame_done = true;
      g.frameEnd();
    });
    frame.join();
    sampler.join();
    CHECK(sampler_back.load());
  }

  // ---- no frame and no sample at once: both sides as fast as they go, windows opening and closing ----
  {
    WireGate g;
    std::atomic<int> in_frame{0}, in_sample{0}, bad{0}, windows{0}, frames{0}, in_window_frames{0};
    std::atomic<bool> core1_done{false};
    std::thread core0([&] {
      while (!core1_done.load() || windows.load() < 2000) {
        g.beginWindow();
        for (int s = 0; s < 50; ++s) {
          g.yieldFrame();
          in_sample = 1;
          if (in_frame.load()) ++bad;
          for (volatile int i = 0; i < 10; ++i) {}
          if (in_frame.load()) ++bad;
          in_sample = 0;
        }
        g.endWindow();
        ++windows;
        if (!core1_done.load()) std::this_thread::yield();
      }
    });
    for (int i = 0; i < 20000; ++i) {
      g.frameBegin();
      in_frame = 1;
      if (in_sample.load()) ++bad;
      if (g.windowOpen()) ++in_window_frames;
      for (volatile int k = 0; k < 20; ++k) {}
      if (in_sample.load()) ++bad;
      in_frame = 0;
      g.frameEnd();
      ++frames;
    }
    core1_done = true;
    core0.join();
    printf("  stress: %d frames (%d inside a window), %d windows, %d at once\n", frames.load(), in_window_frames.load(),
           windows.load(), bad.load());
    CHECK(bad.load() == 0 && frames.load() == 20000 && windows.load() >= 2000 && in_window_frames.load() > 0);
  }

  // ---- SwioPhy: a request's frames go on while one window stays open, and none meets a sample ----
  static SwioPhy phy;
  CHECK(phy.begin(4));
  {
    g_target.sdi = g_target.dmactive = true;
    Sampler sampler(gWireGate);
    const int before = g_frames.load();
    for (int i = 0; i < 3000; ++i) {
      uint32_t v = 0;
      if (i & 1) phy.write(0x20, 0x1234u + i);
      else CHECK(phy.read(0x11, v) && v == (2 | (1u << 7) | (3u << 10)));
    }
    CHECK(gWireGate.windowOpen() && sampler.open.load());   // all of it inside the one window
    sampler.end();
    printf("  requests: %d frames in one window, the sampler stopped %ld times (%ld samples), %d met a sample\n",
           g_frames.load() - before, sampler.stops.load(), sampler.samples.load(), g_overlaps.load());
    CHECK(g_frames.load() - before >= 3000 && sampler.stops.load() > 0 && g_overlaps.load() == 0);
    g_target = Target{};
  }

  // ---- the console during a capture: a line written while a window stays open reaches the target inside it ----
  {
    static Ch32Dm dm(phy);
    static DmConsole console(dm, phy);
    CHECK(console.start(2));                     // dmseq; attached (no window open yet)
    SeqTarget target(g_target.data0);
    for (int i = 0; i < 200; ++i) { advanceMicros(500); console.poll(); target.available(); }   // synced
    Sampler sampler(gWireGate);                  // a capture starts, and its window stays open
    const int before = g_frames.load(), overlaps = g_overlaps.load();
    const uint8_t line[] = {'T', 'O', 'G', 'G', 'L', 'E', ' ', '0', ' ', '2', '0', '\n'};
    CHECK(console.queue(line, sizeof line) == sizeof line);
    const uint32_t start = millis();
    for (int i = 0; i < 2000 && target.rx.size() < sizeof line; ++i) { advanceMicros(500); console.poll(); target.available(); }
    const uint32_t took = millis() - start;
    CHECK(gWireGate.windowOpen() && sampler.open.load());   // delivered with the window still open
    sampler.end();
    printf("  console: the line delivered in %u ms of probe time inside one window (%d frames, the sampler stopped %ld "
           "times)\n", took, g_frames.load() - before, sampler.stops.load());
    CHECK(target.rx == std::vector<uint8_t>(line, line + sizeof line));
    CHECK(took <= 100 && g_overlaps.load() == overlaps);
  }

  printf("wire-gate: %d checks, %d failures\n", checks.load(), failures.load());
  return failures.load() ? 1 : 0;
}
