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
// The gate works frame by frame: while the sampler has a window open (window_), a SWIO frame announces itself (frame_
// odd) and starts only once the sampler has stopped reading for it (ack_ = that frame); the sampler, between two
// samples, sees the frame, stops, and reads again once the frame has ended (frame_ even) - the samples due meanwhile are
// taken late (the slipped flag), the frame meets no GPIO.in read. Neither side waits for more than one frame or one
// sample: a request and the console's reading go on during a window (a console command sent during a capture reaches
// the target inside it; waiting out whole windows kept every command and reset sent after a capture's start out of it,
// 0.0.29-dev f32a3ef), and a window opens at once. The frame announces itself before it looks at window_, and the
// sampler opens window_ before it looks at frame_ (Dekker's order): a frame that saw no window is waited out by the
// window's start. One gate on a probe (one sampler, one SWIO wire): gWireGate. Elsewhere nothing opens a window and every
// frame goes at once.
#pragma once

#include <stdint.h>

#include <atomic>

namespace oep {

// inlined where they are used: the SWIO frames and the sampling loop are IRAM code on the classic
#define OEP_GATE_INLINE inline __attribute__((always_inline))

class WireGate {
 public:
  // ---- loop()'s core (the SWIO wire): around each frame, nothing between them touching the GPIO registers ----
  // Announced; with a window open, the sampler's stop for this frame waited for (one sample at most).
  OEP_GATE_INLINE void frameBegin() {
    const uint32_t v = frame_.load(std::memory_order_relaxed) + 1;   // odd: this frame (only this core writes frame_)
    frame_.store(v);
    while (window_.load() && ack_.load() != v) {}
  }
  // The frame is off the line (its writes done first: a sequentially consistent store): the sampler reads again.
  OEP_GATE_INLINE void frameEnd() { frame_.store(frame_.load(std::memory_order_relaxed) + 1); }
  bool framing() const { return (frame_.load() & 1) != 0; }

  // ---- the sampler's core ----
  // Interrupts off first: from here until endWindow() the sampler calls yieldFrame() before each GPIO read. A frame
  // already on the line (it saw no window) is waited out here.
  OEP_GATE_INLINE void beginWindow() {
    window_.store(1);
    yieldFrame();
  }
  OEP_GATE_INLINE void endWindow() { window_.store(0); }
  bool windowOpen() const { return window_.load() != 0; }
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
  std::atomic<uint32_t> window_{0}, frame_{0}, ack_{0};
};

#undef OEP_GATE_INLINE

inline WireGate gWireGate;   // the probe's one gate (C++17 inline variable: one object for every translation unit)

}  // namespace oep
