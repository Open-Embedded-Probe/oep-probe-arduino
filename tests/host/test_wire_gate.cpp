// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the gate between the classic ESP32's core-0 sampler and the SWIO frames of loop()'s core (OepWireGate.h),
// with two threads standing for the two cores, and SwioPhy's frames through it (OEP_HOST_FAKE_SWIO).
// - the gate's rules: a take holds until release, a window waits for the holder, a take is refused while a window is
//   open or waiting (and noted for the sampler's turn unless asked not to);
// - no frame and no window at once, under a stress of both sides;
// - a request's frames (DmiPhy::read / write) wait out a window and never run inside one;
// - the console's turn (backgroundTurn): refused while a window is open (the console pauses, oep-if-console §3 allows
//   it; docs/implementation-limits §4.1), and given once it has closed.
#include <stdio.h>

#include <atomic>
#include <chrono>
#include <thread>

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
static uint32_t g_progbuf0 = 0;
static void frameSeen() {
  ++g_frames;
  if (g_inside.load()) ++g_overlaps;
}
bool fakeSwioRead(uint8_t address, uint32_t &value) {
  frameSeen();
  switch (address) {
    case 0x10: value = 1; break;                 // DMCONTROL: dmactive
    case 0x11: value = 0x00400382u; break;       // DMSTATUS: a running module
    case 0x20: value = g_progbuf0; break;
    case 0x7d: value = 0x5aa50400u; break;
    default: value = 0x12345678u; break;
  }
  return true;
}
void fakeSwioWrite(uint8_t address, uint32_t value, bool) {
  frameSeen();
  if (address == 0x20) g_progbuf0 = value;
}
bool fakeSwioLineHigh() { return true; }

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
      if (i & 1) phy.write(0x04, 0x1234u + i);
      else CHECK(phy.read(0x11, v) && v == 0x00400382u);
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

  // ---- the console's turn while a window is open ----
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
    const int overlaps = g_overlaps.load();
    gWireGate.takeWanted();                      // the notes the requests above left
    const bool turn = phy.backgroundTurn();
    CHECK(!turn);                                // paused: nothing read inside the window
    CHECK(gWireGate.takeWanted());               // and a turn asked for between bursts
    close = true;
    sampler.join();
    CHECK(phy.backgroundTurn());                 // the window closed: the turn is the console's
    uint32_t v = 0;
    CHECK(phy.read(0x04, v) && g_overlaps.load() == overlaps);
    phy.backgroundDone();
    gWireGate.release();
  }

  printf("wire-gate: %d checks, %d failures\n", checks.load(), failures.load());
  return failures.load() ? 1 : 0;
}
