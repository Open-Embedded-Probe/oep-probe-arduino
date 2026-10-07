// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the gate between the classic ESP32's core-0 sampler and the SWIO frames of loop()'s core (OepWireGate.h),
// with two threads standing for the two cores, and SwioPhy's frames through it (OEP_HOST_FAKE_SWIO). A sampler window
// and the SWIO wire are mutually exclusive (docs/implementation-limits §4.1):
// - the gate's rules: a take holds until release, a window waits for the holder, a take is refused while a window is
//   open or waiting (and noted for the sampler's turn unless asked not to);
// - no frame and no window at once, under a stress of both sides;
// - a request's frames (DmiPhy::read / write) wait out a window and never run inside one;
// - the console (DmConsole, dmseq, on Ch32Dm and SwioPhy against a simulated target): an immediate capture's window
//   delivers no console traffic at all - no frame while it is open, the line goes after it has closed; a trigger
//   search's bursts leave the wire a turn between them (kWireTurnMs), so a line written after arm reaches the target
//   while the search goes on, and no frame runs inside a burst. (0.0.29-dev 589acd5 let frames into a window one at a
//   time: on the bench fast signals slipped and broke, and the suite took twice as long.)
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

// The sampler thread is inside a window (interrupts off on the core it stands for); frames counted, and those met inside.
static std::atomic<int> g_inside{0}, g_frames{0}, g_overlaps{0};
static void frameSeen() {
  ++g_frames;
  if (g_inside.load()) ++g_overlaps;
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

static void spinFor(std::chrono::microseconds us) {
  const auto end = std::chrono::steady_clock::now() + us;
  while (std::chrono::steady_clock::now() < end) {}
}

int main() {
  // ---- the rules, one thread ----
  {
    WireGate g;
    CHECK(g.tryHold() && g.held());
    CHECK(g.tryHold());                          // held already: still the holder's
    g.release();
    CHECK(!g.held() && !g.windowOpen());
    CHECK(g.tryHold());
    int waited = 0;
    g.beginWindow([&] { ++waited; g.release(); });   // the holder lets go while the window waits
    CHECK(waited == 1 && g.windowOpen() && !g.held());
    CHECK(!g.tryHold() && !g.held());            // refused inside the window
    CHECK(g.takeWanted() && !g.takeWanted());    // noted once
    CHECK(!g.tryHold(false) && !g.takeWanted()); // a reader that goes on anyway asks for no turn
    g.endWindow();
    CHECK(g.tryHold() && g.held());
    g.release();
  }

  // ---- no frame and no window at once: both sides as fast as they go ----
  {
    WireGate g;
    std::atomic<int> in_frame{0}, in_window{0}, bad{0}, windows{0}, frames{0};
    std::atomic<bool> core1_done{false};
    std::thread core0([&] {
      while (!core1_done.load() || windows.load() < 2000) {
        g.beginWindow([] { std::this_thread::yield(); });
        in_window = 1;
        if (in_frame.load()) ++bad;
        for (volatile int i = 0; i < 50; ++i) {}
        if (in_frame.load()) ++bad;
        in_window = 0;
        g.endWindow();
        ++windows;
        if (core1_done.load()) continue;
        std::this_thread::yield();
      }
    });
    for (int i = 0; i < 20000; ++i) {
      g.hold([] { std::this_thread::yield(); });
      in_frame = 1;
      if (in_window.load()) ++bad;
      for (volatile int k = 0; k < 20; ++k) {}
      if (in_window.load()) ++bad;
      in_frame = 0;
      ++frames;
      if (i % 3 == 2) g.release();               // loop() came round
    }
    g.release();
    core1_done = true;
    core0.join();
    printf("  stress: %d frames, %d windows, %d at once\n", frames.load(), windows.load(), bad.load());
    CHECK(bad.load() == 0 && frames.load() == 20000 && windows.load() >= 2000);
  }

  // ---- SwioPhy: a request's frames wait out the sampler's windows ----
  static SwioPhy phy;
  CHECK(phy.begin(4));
  {
    g_target.sdi = g_target.dmactive = true;
    std::atomic<bool> stop{false};
    std::atomic<int> windows{0};
    std::thread sampler([&] {
      while (!stop.load()) {
        gWireGate.beginWindow([] { std::this_thread::yield(); });
        g_inside = 1;
        spinFor(std::chrono::microseconds(300));
        g_inside = 0;
        gWireGate.endWindow();
        ++windows;
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      }
    });
    const int before = g_frames.load();
    for (int i = 0; i < 3000; ++i) {
      uint32_t v = 0;
      if (i & 1) phy.write(0x20, 0x1234u + i);
      else CHECK(phy.read(0x11, v) && v == (2 | (1u << 7) | (3u << 10)));
      if (i % 4 == 3) {                         // loop() came round (the rest of it takes a while)
        gWireGate.release();
        std::this_thread::sleep_for(std::chrono::microseconds(50));
      }
    }
    gWireGate.release();
    stop = true;
    sampler.join();
    printf("  requests: %d frames, %d windows, %d inside a window\n", g_frames.load() - before, windows.load(),
           g_overlaps.load());
    CHECK(g_frames.load() - before >= 3000 && windows.load() > 10 && g_overlaps.load() == 0);
  }

  // ---- the console's turn: refused while a window is open, noted for the sampler, given once it has closed ----
  {
    std::atomic<bool> open_now{false}, close{false};
    std::thread sampler([&] {
      gWireGate.beginWindow([] { std::this_thread::yield(); });
      g_inside = 1;
      open_now = true;
      while (!close.load()) std::this_thread::yield();
      g_inside = 0;
      gWireGate.endWindow();
    });
    while (!open_now.load()) std::this_thread::yield();
    gWireGate.takeWanted();                      // the notes the requests above left
    CHECK(!phy.backgroundTurn());                // paused: nothing read inside the window
    CHECK(gWireGate.takeWanted());               // and a turn asked for between bursts
    close = true;
    sampler.join();
    CHECK(phy.backgroundTurn());                 // the window closed: the turn is the console's
    phy.backgroundDone();
    gWireGate.release();
    g_target = Target{};
  }

  static Ch32Dm dm(phy);
  static DmConsole console(dm, phy);
  CHECK(console.start(2));                       // dmseq; attached (no window open)
  SeqTarget target(g_target.data0);
  auto loopOnce = [&] {                          // one time round loop(): the console's poll, then the wire let go
    advanceMicros(500);
    console.poll();
    target.available();
    gWireGate.release();
  };
  for (int i = 0; i < 200; ++i) loopOnce();      // synced
  CHECK(target.rx.empty());

  // ---- an immediate capture's window: no console traffic inside it; the line goes after it ----
  {
    std::atomic<bool> open_now{false}, close{false};
    std::thread sampler([&] {
      gWireGate.beginWindow([] { std::this_thread::yield(); });
      g_inside = 1;
      open_now = true;
      while (!close.load()) std::this_thread::yield();
      g_inside = 0;
      gWireGate.endWindow();
    });
    while (!open_now.load()) std::this_thread::yield();
    const int before = g_frames.load(), overlaps = g_overlaps.load();
    const uint8_t line[] = {'T', 'O', 'G', 'G', 'L', 'E', '\n'};
    CHECK(console.queue(line, sizeof line) == sizeof line);
    for (int i = 0; i < 400; ++i) loopOnce();    // 200 ms of probe time with the window open
    CHECK(g_frames.load() == before && target.rx.empty());   // not one frame: the window has the GPIO bus alone
    close = true;
    sampler.join();
    for (int i = 0; i < 2000 && target.rx.size() < sizeof line; ++i) loopOnce();
    printf("  immediate window: 0 frames inside, the line delivered after it (%d frames)\n", g_frames.load() - before);
    CHECK(target.rx == std::vector<uint8_t>(line, line + sizeof line) && g_overlaps.load() == overlaps);
    target.rx.clear();
  }

  // ---- a trigger search: bursts with the wire's turn between them; the line arrives while the search goes on ----
  {
    std::atomic<bool> stop{false};
    std::atomic<int> bursts{0}, turns{0};
    std::thread sampler([&] {                    // as SamplerCapture::run's search: burst, then the wire's turn if asked
      while (!stop.load()) {
        gWireGate.beginWindow([] { std::this_thread::yield(); });
        g_inside = 1;
        spinFor(std::chrono::microseconds(2000));
        g_inside = 0;
        gWireGate.endWindow();
        ++bursts;
        if (gWireGate.takeWanted()) { ++turns; std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
        else std::this_thread::yield();
      }
    });
    while (bursts.load() < 1) std::this_thread::yield();
    const int overlaps = g_overlaps.load();
    const uint8_t line[] = {'A', 'R', 'M', 'E', 'D', '\n'};
    CHECK(console.queue(line, sizeof line) == sizeof line);
    for (int i = 0; i < 20000 && target.rx.size() < sizeof line; ++i) {
      loopOnce();
      std::this_thread::sleep_for(std::chrono::microseconds(100));   // the rest of loop()
    }
    const bool searching = !stop.load();
    stop = true;
    sampler.join();
    printf("  trigger search: the line delivered between bursts (%d bursts, %d turns), %d frames inside a burst\n",
           bursts.load(), turns.load(), g_overlaps.load() - overlaps);
    CHECK(searching && target.rx == std::vector<uint8_t>(line, line + sizeof line));
    CHECK(turns.load() > 0 && g_overlaps.load() == overlaps);
  }

  printf("wire-gate: %d checks, %d failures\n", checks.load(), failures.load());
  return failures.load() ? 1 : 0;
}
