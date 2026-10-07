// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The classic ESP32's two users of the GPIO registers kept apart: the core-0 sampler (SamplerCapture), which reads
// GPIO.in back to back with interrupts off, and the SWIO frames of loop()'s core (SwioPhy), whose bit times are made of
// GPIO register writes and a GPIO.in read. Both cores reach those registers over the same peripheral bus: an access of
// one core waits for the access of the other in flight. The sampler sees it as samples taken late (its slipped flag,
// 2026-09-29); a SWIO frame sees it as pulses moved by up to a GPIO.in read (two of them did not fit in the 120 cycles of
// a 2 MHz sample, so one is tens of cycles) against a 262.5 ns short pulse made of the same accesses - a one read as a
// zero, a pulse lost, a frame the target parses as another register or another value. SWIO has no parity and a DMI
// write is not read back, so nothing on the wire tells.
//
// The gate: the sampler opens a window (window_) before it turns interrupts off and waits for the wire to be let go of
// (held_); loop()'s core takes the wire (held_) before a frame and keeps it until loop() comes round
// (release(), SamplerCapture::poll) - a request's frames are never split by a window, and a window never starts while
// a frame is on the line. Each side sets its own flag before it looks at the other's (Dekker's order), so they cannot
// both go on. One gate on a probe (one sampler, one SWIO wire): gWireGate. Elsewhere nothing opens a window and every
// take succeeds at once.
#pragma once

#include <stdint.h>

#include <atomic>

namespace oep {

// inlined where they are used: the SWIO frames' callers are IRAM code on the classic
#define OEP_GATE_INLINE inline __attribute__((always_inline))

class WireGate {
 public:
  // ---- loop()'s core (the wire's user) ----
  // Take the wire if no window is open or waiting; true also when this core holds it already. A refusal is noted for
  // the sampler when `note` (wanted): between the bursts of a trigger search it then leaves the wire a turn
  // (SamplerCapture) - a reader that goes on through the window anyway does not ask for one.
  OEP_GATE_INLINE bool tryHold(bool note = true) {
    if (held_.load()) return true;
    held_.store(1);
    if (!window_.load()) return true;
    held_.store(0);
    if (note) wanted_.store(1);
    return false;
  }
  // Take the wire, waiting out a window (one burst at most: a waiting window lets the holder go on, see beginWindow).
  template <class Spin> OEP_GATE_INLINE void hold(Spin spin) {
    while (!tryHold()) {
      while (window_.load()) spin();
    }
  }
  OEP_GATE_INLINE void hold() { hold([] {}); }
  // loop() came round (or the wire's user waits for the sampler, SamplerCapture::waitIdle): a window may open now.
  OEP_GATE_INLINE void release() { held_.store(0); }
  bool held() const { return held_.load() != 0; }

  // ---- the sampler's core ----
  // Before interrupts go off: the window is announced, then the wire's current holder is waited for (`wait` yields,
  // so the core's idle task and watchdog run). From here until endWindow() loop()'s core starts no frame.
  template <class Wait> OEP_GATE_INLINE void beginWindow(Wait wait) {
    window_.store(1);
    while (held_.load()) wait();
  }
  OEP_GATE_INLINE void endWindow() { window_.store(0); }
  bool windowOpen() const { return window_.load() != 0; }
  // A take was refused since the last call (and the note cleared).
  OEP_GATE_INLINE bool takeWanted() {
    if (!wanted_.load()) return false;
    wanted_.store(0);
    return true;
  }

 private:
  // words, sequentially consistent loads and stores only (no read-modify-write: nothing the Xtensa core lacks)
  std::atomic<uint32_t> window_{0}, held_{0}, wanted_{0};
};

#undef OEP_GATE_INLINE

inline WireGate gWireGate;   // the probe's one gate (C++17 inline variable: one object for every translation unit)

}  // namespace oep
