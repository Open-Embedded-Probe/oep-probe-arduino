// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The classic ESP32's two users of the GPIO registers kept apart: the sampler (SamplerCapture, on the core loop() is not
// on), which reads
// GPIO.in back to back with interrupts off, and the SWIO frames of loop()'s core (SwioPhy), whose bit times are made of
// GPIO register writes and a GPIO.in read. Both cores reach those registers over the same peripheral bus: an access of
// one core waits for the access of the other in flight. The sampler sees it as samples taken late (its slipped flag,
// 2026-09-29); a SWIO frame sees it as pulses moved by up to a GPIO.in read (two of them did not fit in the 120 cycles of
// a 2 MHz sample, so one is tens of cycles) against a 262.5 ns short pulse made of the same accesses - a one read as a
// zero, a pulse lost, a frame the target parses as another register or another value. SWIO has no parity and a DMI
// write is not read back, so nothing on the wire tells.
//
// The gate. Two rules keep the two apart:
// - Frame by frame, always: while the sampler reads (reading_, interrupts off on its core), a SWIO frame announces itself
//   (frame_ odd) and starts only once the sampler has stopped reading for it (ack_ = that frame); the sampler, between
//   two samples, sees it, stops, and reads again once the frame has ended (frame_ even) - the samples due meanwhile are
//   taken late (its slipped flag). No frame meets a GPIO.in read, whatever the window.
// - Who may take the wire (held_: from a frame until loop() comes round, release() in SamplerCapture::poll): while a
//   window is exclusive (window_ kExclusive) a take is refused, so a window samples with no frame in it; a holder from
//   before goes on, frame by frame. An immediate capture's window and the part of a triggered segment after its trigger
//   are exclusive, and the immediate window first waits for the holder to let go (beginWindow). A trigger search runs
//   exclusive too, but gives the wire turns inside its bursts (kShared): one when a take was refused (wanted_; the
//   console's polls, at most one turn a burst), and one whenever a request waits for the wire (needed_) - the search
//   samples on through a turn, so what a command or a reset sent in it makes the target do is seen. The console ends a
//   turn early once it has handed the target the last byte it had to send (endTurn): the target then acts with the
//   sampler alone on the bus.
// Each side sets its own flag before it looks at the other's (Dekker's order), so neither misses the other. One gate on
// a probe (one sampler, one SWIO wire): gWireGate. Elsewhere nothing opens a window and every take succeeds at once.
#pragma once

#include <stdint.h>

#include <atomic>

namespace oep {

// inlined where they are used: the SWIO frames' callers are IRAM code on the classic
#define OEP_GATE_INLINE inline __attribute__((always_inline))

class WireGate {
 public:
  enum : uint32_t { kNone = 0, kExclusive = 1, kShared = 2 };

  // ---- loop()'s core (the wire's user) ----
  // Take the wire unless a window is exclusive; true also when this core holds it already. A refusal is noted for the
  // sampler: `need` (a request waiting for it, hold) gets a turn in any burst of a search, a plain refusal (the console's
  // poll) one a burst.
  OEP_GATE_INLINE bool tryHold(bool need = false) {
    if (held_.load()) return true;
    held_.store(1);
    if (window_.load() != kExclusive) return true;
    held_.store(0);
    (need ? needed_ : wanted_).store(1);
    return false;
  }
  // Take the wire, waiting while a window is exclusive (a search gives a waiting request a turn at its next sample; an
  // immediate window or a segment's rest after its trigger is waited out: up to 164 ms).
  template <class Spin> OEP_GATE_INLINE void hold(Spin spin) {
    while (!tryHold(true)) {
      while (window_.load() == kExclusive && needed_.load()) spin();
    }
  }
  OEP_GATE_INLINE void hold() { hold([] {}); }
  // loop() came round (or the wire's user waits for the sampler, SamplerCapture::waitIdle): no frame of it in flight.
  OEP_GATE_INLINE void release() { held_.store(0); }
  bool held() const { return held_.load() != 0; }
  // Around each frame (after hold), nothing between them touching the GPIO registers but the frame itself.
  OEP_GATE_INLINE void frameBegin() {
    const uint32_t v = frame_.load(std::memory_order_relaxed) + 1;   // odd: this frame (only this core writes frame_)
    frame_.store(v);
    while (reading_.load() && ack_.load() != v) {}
  }
  OEP_GATE_INLINE void frameEnd() { frame_.store(frame_.load(std::memory_order_relaxed) + 1); }
  // The console handed the target what it had to send: a search's turn may end (the target acts with no frame about).
  OEP_GATE_INLINE void endTurn() { if (window_.load() == kShared) turn_done_.store(1); }

  // ---- the sampler's core ----
  // An exclusive window, its holder waited out first (`wait` yields, so the core's idle task and watchdog run).
  template <class Wait> OEP_GATE_INLINE void beginWindow(Wait wait) {
    window_.store(kExclusive);
    while (held_.load()) wait();
  }
  // A search's burst: exclusive, or at once a turn when the wire is held or asked for (no waiting: frames go one by one).
  // true: it starts with a turn.
  OEP_GATE_INLINE bool beginBurst() {
    window_.store(kExclusive);
    if (!held_.load() && !wanted_.load() && !needed_.load()) return false;
    openTurn();
    return true;
  }
  OEP_GATE_INLINE void openTurn() {
    wanted_.store(0);
    needed_.store(0);
    turn_done_.store(0);
    window_.store(kShared);
  }
  OEP_GATE_INLINE void closeTurn() { window_.store(kExclusive); }
  // A turn asked for: 2 a request waits, 1 a refused poll, 0 none (cheap looks: once a sample).
  OEP_GATE_INLINE uint32_t turnWanted() const {
    return needed_.load(std::memory_order_relaxed) ? 2 : wanted_.load(std::memory_order_relaxed) ? 1 : 0;
  }
  OEP_GATE_INLINE bool turnDone() const { return turn_done_.load(std::memory_order_relaxed) != 0; }
  OEP_GATE_INLINE void endWindow() { window_.store(kNone); }
  bool windowOpen() const { return window_.load() != kNone; }
  uint32_t window() const { return window_.load(); }
  // Interrupts off first. From here until endReading() the sampler calls yieldFrame() before each GPIO read; a frame
  // already on the line is waited out here.
  OEP_GATE_INLINE void beginReading() {
    reading_.store(1);
    yieldFrame();
  }
  OEP_GATE_INLINE void endReading() { reading_.store(0); }
  // Between two samples: a frame announced has the bus to itself until it ends. true: the sampler stopped for one.
  OEP_GATE_INLINE bool yieldFrame() {
    const uint32_t v = frame_.load(std::memory_order_relaxed);   // the cheap look, every sample
    if (!(v & 1)) return false;
    ack_.store(v);
    while (frame_.load() == v) {}
    return true;
  }

 private:
  // words, loads and stores only (no read-modify-write: nothing the Xtensa core lacks); frame_ and ack_ count frames
  // (frame_ odd while one is on the line), so an old frame's ack never lets a new one go
  std::atomic<uint32_t> window_{0}, held_{0}, wanted_{0}, needed_{0}, turn_done_{0}, reading_{0}, frame_{0}, ack_{0};
};

#undef OEP_GATE_INLINE

inline WireGate gWireGate;   // the probe's one gate (C++17 inline variable: one object for every translation unit)

}  // namespace oep
