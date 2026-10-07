// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the gate between the classic ESP32's core-0 sampler and the SWIO frames of loop()'s core (OepWireGate.h),
// with two threads standing for the two cores, SwioPhy's frames through it (OEP_HOST_FAKE_SWIO), and the sampler's own
// loop (OepSamplerRun.h) on a host Io that samples a simulated target's marker line in real time:
// - the gate's rules: a take holds until release, an exclusive window waits for the holder and refuses takes (noted:
//   a request's as needed, the console's as wanted), a search's turn lets them take, a frame waits for the sampler to
//   stop reading;
// - no frame and no GPIO read at once, under a stress of both sides (exclusive windows and turns);
// - a request's frames (DmiPhy::read / write) wait out an exclusive window and never run inside one;
// - the console (DmConsole, dmseq, on Ch32Dm and SwioPhy against a simulated target): an immediate capture's window
//   delivers no console traffic at all - no frame while it is open, the line goes after it has closed;
// - a trigger search (sampler::run): a line sent after arm reaches the target inside a burst's turn, the target acts on
//   it and the edge trigger fires on what it did - a 570 us low (a software reset's marker), 20 toggles a few us apart;
//   no frame meets a read, and the segment's rest after the trigger has no new frame (0.0.29-dev 51360ea gave the turn
//   between two bursts, when nothing sampled: the target acted then and the trigger never fired - the bench's
//   reset_probe "no capture for the software reset"); a request waiting for the wire gets a turn inside the burst; a
//   trigger before `pre` samples of a burst gives a shorter segment with the trigger index smaller (capture §3.3).
#include <stdio.h>
#include <string.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <utility>
#include <vector>

#include "OepCh32Dm.h"
#include "OepDmConsole.h"
#include "OepSamplerRun.h"
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
static std::atomic<int> g_inside{0}, g_frames{0}, g_overlaps{0}, g_in_frame{0}, g_read_overlaps{0};
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

static void spinFor(std::chrono::microseconds us);
// a frame takes a few microseconds of real time, so that a sample read during it would be seen (g_read_overlaps)
bool fakeSwioRead(uint8_t address, uint32_t &value) {
  g_in_frame = 1;
  frameSeen();
  const bool ok = g_target.read(address, value);
  spinFor(std::chrono::microseconds(20));
  g_in_frame = 0;
  return ok;
}
void fakeSwioWrite(uint8_t address, uint32_t value, bool) {
  g_in_frame = 1;
  frameSeen();
  g_target.write(address, value);
  spinFor(std::chrono::microseconds(20));
  g_in_frame = 0;
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

// The sampler's Io on the host: cycles are steady-clock nanoseconds; a sample reads the target's marker line (bit 0).
static std::atomic<uint8_t> g_marker{1};
static std::atomic<int> g_post_frames{0};
static std::atomic<bool> g_post{false};
static uint32_t nowCycles() {
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
}
struct HostIo {
  uint32_t cycles() const { return nowCycles(); }
  uint8_t read() const {
    if (g_in_frame.load()) ++g_read_overlaps;
    return g_marker.load();
  }
  void interruptsOff() const {}
  void interruptsOn() const {}
  uint64_t nowNs() const { return nowCycles(); }
  void yield() const { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
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
    g.beginWindow([&] { ++waited; g.release(); });   // the holder lets go while the exclusive window waits
    CHECK(waited == 1 && g.window() == WireGate::kExclusive && !g.held());
    CHECK(!g.tryHold() && !g.held());            // refused inside it
    CHECK(g.turnWanted() == 1);                  // noted for a search's turn: wanted (the console's poll)
    CHECK(!g.tryHold(true) && g.turnWanted() == 2);   // a request waiting: needed
    g.endWindow();
    CHECK(g.tryHold() && g.held());
    g.release();
    // a search's burst: starts with a turn when the wire was asked for (the notes above), and the turn lets takes in
    CHECK(g.beginBurst() && g.window() == WireGate::kShared && g.turnWanted() == 0);
    CHECK(g.tryHold());
    g.endTurn();
    CHECK(g.turnDone());                         // the console has sent what it had
    g.closeTurn();
    CHECK(g.tryHold());                          // a holder from the turn goes on
    g.release();
    CHECK(!g.tryHold() && g.turnWanted() == 1);  // a new take does not
    g.endWindow();
    CHECK(!g.beginBurst() || g.window() == WireGate::kShared);
    g.endWindow();
    CHECK(!g.beginBurst() && g.window() == WireGate::kExclusive);   // nothing asked: exclusive from the start
    g.endWindow();
  }

  // ---- no frame meets a read: the sampler reading through exclusive windows and turns, frames as fast as they go ----
  {
    WireGate &g = gWireGate;
    std::atomic<int> in_frame{0}, reading{0}, bad{0}, windows{0}, frames{0}, inside_exclusive{0};
    std::atomic<bool> core1_done{false};
    std::thread core0([&] {
      while (!core1_done.load() || windows.load() < 2000) {
        const bool exclusive = windows.load() % 2 == 0;
        if (exclusive) g.beginWindow([] { std::this_thread::yield(); });
        else { g.beginBurst(); g.openTurn(); }
        g.beginReading();
        for (int i = 0; i < 40; ++i) {
          g.yieldFrame();
          reading = 1;
          if (in_frame.load()) ++bad;
          for (volatile int k = 0; k < 5; ++k) {}
          if (in_frame.load()) ++bad;
          reading = 0;
        }
        g.endReading();
        g.endWindow();
        ++windows;
        std::this_thread::yield();
      }
    });
    for (int i = 0; i < 20000; ++i) {
      g.hold([] { std::this_thread::yield(); });
      g.frameBegin();
      in_frame = 1;
      if (reading.load()) ++bad;
      if (g.window() == WireGate::kExclusive && g.held()) {}   // a holder from a turn may go on
      for (volatile int k = 0; k < 20; ++k) {}
      if (reading.load()) ++bad;
      in_frame = 0;
      g.frameEnd();
      ++frames;
      if (i % 3 == 2) g.release();               // loop() came round
    }
    g.release();
    core1_done = true;
    core0.join();
    (void)inside_exclusive;
    printf("  stress: %d frames, %d windows, %d at once with a read\n", frames.load(), windows.load(), bad.load());
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

  // ---- the console's turn: refused while a window is exclusive, noted for a search, given once it has closed ----
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
    CHECK(!phy.backgroundTurn());                // paused: nothing read inside the window
    CHECK(gWireGate.turnWanted() != 0);          // and a turn asked for
    close = true;
    sampler.join();
    CHECK(phy.backgroundTurn());                 // the window closed: the turn is the console's
    phy.backgroundDone();
    gWireGate.release();
    gWireGate.openTurn();                        // the notes cleared
    gWireGate.endWindow();
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

  // ---- a trigger search (sampler::run): the line goes in a turn inside a burst, the target acts, the trigger fires ----
  // 100 kHz (10 us a sample), a 4000-sample segment with 1000 before the trigger, bursts of 20000 samples (200 ms), a
  // turn of 5 ms. The target: once the line is complete it acts at once (`act`).
  auto search = [&](const char *what, const uint8_t *line, size_t length, uint8_t value, uint32_t pre,
                    void (*act)()) {
    static uint8_t buffer[4000];
    memset(buffer, 0xEE, sizeof buffer);
    sampler::Plan plan{buffer, sizeof buffer, 10000, 1000000000u, sampler::kEdge, 0, value, pre, 20000, 500};
    volatile uint8_t control = 0;
    std::atomic<bool> triggered{false};
    std::atomic<uint32_t> kept_at{0};
    sampler::Outcome outcome;
    const int read_overlaps = g_read_overlaps.load();
    g_post_frames = 0;
    std::thread core0([&] {
      HostIo io;
      outcome = sampler::run(io, plan, control, [&](uint32_t, uint32_t kept, uint64_t) {
        kept_at = kept;
        g_post = true;
        triggered = true;
      });
      g_post = false;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(30));   // the search under way (the arm, then the command)
    CHECK(console.queue(line, length) == length);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    size_t had = target.rx.size();
    bool acted = false;
    while (!triggered.load() && std::chrono::steady_clock::now() < deadline) {
      const int before = g_frames.load();
      loopOnce();
      if (g_post.load()) g_post_frames += g_frames.load() - before;
      if (!acted && target.rx.size() >= had + length) { acted = true; act(); }   // the target acts on its line
      spinFor(std::chrono::microseconds(100));   // the rest of loop()
    }
    if (!triggered.load()) control = sampler::kControlAbort;
    while (g_post.load() || !triggered.load()) {   // the segment's rest: loop() goes on (its takes are refused)
      if (!triggered.load() && control) break;
      const int before = g_frames.load();
      loopOnce();
      if (g_post.load()) g_post_frames += g_frames.load() - before;
      spinFor(std::chrono::microseconds(100));
    }
    core0.join();
    uint32_t low = 0, edges = 0;
    for (uint32_t i = 0; i < outcome.samples; ++i) {
      low += !(buffer[i] & 1);
      if (i && (buffer[i] & 1) != (buffer[i - 1] & 1)) ++edges;
    }
    printf("  %s: delivered %s, triggered %s, segment %u samples (trigger at %u), %u low, %u edges, slipped %d, "
           "%d frames after the trigger, %d reads met a frame\n",
           what, acted ? "yes" : "no", triggered.load() ? "yes" : "no", outcome.samples, outcome.trigger_index, low,
           edges, outcome.slipped, g_post_frames.load(), g_read_overlaps.load() - read_overlaps);
    CHECK(acted && triggered.load() && !outcome.aborted);
    CHECK(g_read_overlaps.load() == read_overlaps);   // no frame met a read
    CHECK(g_post_frames.load() == 0);                  // nothing new on the wire in the segment's rest
    CHECK(outcome.trigger_index == kept_at.load() && outcome.trigger_index <= pre);
    CHECK(outcome.samples == outcome.trigger_index + 1 + (sizeof buffer - pre - 1));
    return std::make_pair(low, edges);
  };
  {
    static const uint8_t reboot[] = {'R', 'E', 'B', 'O', 'O', 'T', '\n'};
    const auto r = search("software reset (a 570 us low)", reboot, sizeof reboot, 1, 1000, [] {
      g_marker = 0;
      spinFor(std::chrono::microseconds(570));
      g_marker = 1;
    });
    CHECK(r.first >= 50 && r.first <= 70 && r.second == 2);   // the whole low run, 57 samples give or take a few
    target.rx.clear();
    static const uint8_t toggle[] = {'T', 'O', 'G', 'G', 'L', 'E', ' ', '0', ' ', '2', '0', '\n'};
    const auto t = search("20 toggles 2 us apart", toggle, sizeof toggle, 2, 1000, [] {
      for (int i = 0; i < 20; ++i) {
        g_marker = g_marker.load() ^ 1;
        spinFor(std::chrono::microseconds(25));   // 2.5 samples a level (the bench's are faster against its rate too)
      }
    });
    CHECK(t.second >= 15);   // the toggles in the segment
    target.rx.clear();
  }

  // ---- a request waiting for the wire gets a turn inside a burst, not only between bursts ----
  {
    static uint8_t buffer[4000];
    sampler::Plan plan{buffer, sizeof buffer, 10000, 1000000000u, sampler::kEdge, 0, 1, 100, 100000, 500};
    volatile uint8_t control = 0;
    sampler::Outcome outcome;
    std::thread core0([&] {
      HostIo io;
      outcome = sampler::run(io, plan, control, [](uint32_t, uint32_t, uint64_t) {});
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(30));   // a burst of 1 s is sampling
    gWireGate.release();
    const auto t0 = std::chrono::steady_clock::now();
    uint32_t v = 0;
    CHECK(phy.read(0x11, v));
    const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
    gWireGate.release();
    control = sampler::kControlAbort;
    core0.join();
    printf("  a request during a search: answered after %lld ms (bursts of 1000 ms)\n", static_cast<long long>(waited));
    CHECK(waited < 50 && outcome.aborted);
  }

  // ---- an edge in the first samples of a burst: a shorter segment, the trigger index what came before it ----
  {
    static uint8_t buffer[4000];
    sampler::Plan plan{buffer, sizeof buffer, 10000, 1000000000u, sampler::kEdge, 0, 1, 1000, 20000, 500};
    volatile uint8_t control = 0;
    sampler::Outcome outcome;
    g_marker = 1;
    std::thread toggler([] {
      std::this_thread::sleep_for(std::chrono::microseconds(1500));   // 150 samples into the first burst
      g_marker = 0;
    });
    HostIo io;
    outcome = sampler::run(io, plan, control, [](uint32_t, uint32_t, uint64_t) {});
    toggler.join();
    g_marker = 1;
    printf("  an early edge: trigger index %u (pretrigger 1000), segment %u samples\n", outcome.trigger_index,
           outcome.samples);
    CHECK(outcome.trigger_index > 50 && outcome.trigger_index < 1000 && outcome.samples == outcome.trigger_index + 3000);
    CHECK((buffer[outcome.trigger_index] & 1) == 0 && (buffer[outcome.trigger_index - 1] & 1) == 1);
  }

  printf("wire-gate: %d checks, %d failures\n", checks.load(), failures.load());
  return failures.load() ? 1 : 0;
}
