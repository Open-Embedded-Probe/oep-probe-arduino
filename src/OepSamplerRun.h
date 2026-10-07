// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The classic ESP32 sampler's loop (SamplerCapture, OepSampler.cpp), apart from the chip so that the host tests run the
// same code: an `Io` gives the cycle counter, one sample's read, interrupts off / on, the clock in ns and a yield
// (SamplerCapture: esp_cpu_get_cycle_count, GPIO.in, portDISABLE_INTERRUPTS, esp_timer, vTaskDelay(1)).
//
// Immediate: one exclusive window (OepWireGate.h), interrupts off throughout (<= 164 ms at the 400 kHz floor, inside the
// 300 ms interrupt watchdog). Triggered: bursts, each with interrupts off for at most `burst` samples of search and
// then the rest of the segment after the trigger, and on between them (a gap of one tick: the search starts over; the
// wire is not taken in a gap - a take waits for the next burst's turn, at its first sample).
// During the search the buffer is a ring; the segment is turned to start at 0 afterwards. A search burst gives the
// SWIO wire its turns inside it (WireGate::openTurn: the console's polls once a burst, for up to `turn` samples or until
// the console has sent what it had; a waiting request whenever it waits) and samples on through them, frame by frame -
// what a command or a reset sent during a search makes the target do is sampled (0.0.29-dev 51360ea gave the turn
// between two bursts, when nothing sampled: the target acted right then, and an edge trigger on what it did never
// fired). After the trigger the window is exclusive (a holder already on goes on frame by frame).
// A trigger early in a burst, before `pre` samples of it are in, gives a shorter segment (capture §3.3: the segment is
// short, trigger_index smaller) - no part of a burst is blind to the trigger but its first sample for an edge.
// Every sample taken one period or more late (a frame, or the other core holding the bus) is counted; the segment is
// slipped when one of its own samples was.
#pragma once

#include <stdint.h>

#include <algorithm>

#include "OepWireGate.h"

namespace oep {
namespace sampler {

#define OEP_SAMPLER_INLINE inline __attribute__((always_inline))

constexpr uint8_t kControlForce = 1, kControlAbort = 2;
constexpr uint8_t kImmediate = 0, kLevel = 1, kEdge = 2;   // capture §3.3's trigger types

struct Plan {
  uint8_t *out;
  uint32_t size;        // the segment's samples at most (the buffer)
  uint32_t step;        // CPU cycles per sample
  uint32_t cpu_hz;
  uint8_t trig_type, trig_role, trig_value;
  uint32_t pre;         // pretrigger (< size)
  uint32_t burst;       // search samples per burst at most (+ the rest of the segment: kOffNs)
  uint32_t turn;        // samples of a wire turn
};

struct Outcome {
  bool aborted = false;
  uint32_t samples = 0;           // the segment's
  uint32_t trigger_index = 0;     // its samples before the trigger's (immediate: 0xFFFFFFFF)
  uint64_t start_ns = 0;          // its first sample's time
  bool slipped = false;           // one of its samples taken a period or more late
  uint32_t late_cycles = 0;       // the most a sample of the window was behind
};

template <class Io>
struct Pace {
  Io &io;
  uint8_t *out;
  uint32_t size, at, step, next, n, late_n;   // n: samples taken in this window; late_n: the last late one's n
  int32_t late;
};

template <bool kRing, class Io>
OEP_SAMPLER_INLINE uint8_t take(Pace<Io> &p) {
  gWireGate.yieldFrame();   // a SWIO frame announced has the bus first
  const int32_t behind = static_cast<int32_t>(p.io.cycles() - p.next);
  ++p.n;
  if (behind > p.late) p.late = behind;
  if (behind >= static_cast<int32_t>(p.step)) p.late_n = p.n;
  while (static_cast<int32_t>(p.io.cycles() - p.next) < 0) {}
  p.next += p.step;
  const uint8_t byte = p.io.read();
  p.out[p.at] = byte;
  if (++p.at == p.size && kRing) p.at = 0;   // one window (not a ring): no wrap to pay for
  return byte;
}

// onTrigger(count, kept_pre, burst_ns): the trigger's sample in its burst, the samples kept before it, the burst's start
// - called as the rest of the segment starts coming in (SamplerCapture::poll sends triggered meanwhile).
template <class Io, class OnTrigger>
OEP_SAMPLER_INLINE Outcome run(Io &io, const Plan &plan, const volatile uint8_t &control, OnTrigger onTrigger) {
  Outcome r;
  Pace<Io> p{io, plan.out, plan.size, 0, plan.step, 0, 0, 0, 0};
  if (plan.trig_type == kImmediate) {
    gWireGate.beginWindow([&] { io.yield(); });
    io.interruptsOff();
    gWireGate.beginReading();
    r.start_ns = io.nowNs();   // the first sample's time (the start op's was earlier)
    p.next = io.cycles();
    for (uint32_t i = 0; i < p.size; ++i) take<false>(p);
    gWireGate.endReading();
    io.interruptsOn();
    gWireGate.endWindow();
    r.samples = p.size;
    r.trigger_index = 0xFFFFFFFFu;
    r.slipped = p.late_n != 0;
    r.late_cycles = static_cast<uint32_t>(p.late);
    return r;
  }
  const uint32_t pre = plan.pre, post = p.size - pre - 1;   // samples after the trigger's one
  const bool edge = plan.trig_type == kEdge;
  const uint8_t role = plan.trig_role, value = plan.trig_value;
  const uint32_t arm = edge ? 1 : 0;                         // an edge needs the sample before it
  for (;;) {
    p.at = p.n = p.late_n = 0;
    p.late = 0;
    if (control & kControlAbort) { gWireGate.endWindow(); r.aborted = true; return r; }
    bool turned = gWireGate.beginBurst();                     // the wire held or asked for: a turn at once
    uint32_t turn_left = turned ? plan.turn : 0;
    io.interruptsOff();
    gWireGate.beginReading();
    const uint64_t burst_ns = io.nowNs();
    p.next = io.cycles();
    uint8_t last = 0, stop = 0;
    bool hit = false;
    uint32_t c = 0;
    for (; c < plan.burst; ++c) {
      if (turn_left) {
        if (--turn_left == 0 || gWireGate.turnDone()) { turn_left = 0; gWireGate.closeTurn(); }
      } else if (const uint32_t want = gWireGate.turnWanted()) {
        if (want == 2 || !turned) { gWireGate.openTurn(); turned = true; turn_left = plan.turn; }
      }
      const uint8_t bit = (take<true>(p) >> role) & 1;
      if (c >= arm) {
        hit = edge ? bit != last && (value == 2 || bit == (value == 0 ? 1 : 0)) : bit == value;
        if (hit || (stop = control) != 0) break;
      }
      last = bit;
    }
    if (hit || (stop & kControlForce)) {
      gWireGate.closeTurn();                                  // the segment's rest: no new frame
      const uint32_t kept = c < pre ? c : pre, first = c - kept;
      onTrigger(c, kept, burst_ns);
      for (uint32_t i = 0; i < post; ++i) take<true>(p);
      gWireGate.endReading();
      io.interruptsOn();
      gWireGate.endWindow();
      if (c >= pre) std::rotate(p.out, p.out + p.at, p.out + p.size);   // the ring's oldest kept sample (c - pre) first
      r.samples = kept + 1 + post;                                       // c < pre: from 0, no wrap
      r.trigger_index = kept;
      r.start_ns = burst_ns + static_cast<uint64_t>(first) * p.step * 1000000000ull / plan.cpu_hz;
      r.slipped = p.late_n > first;                                      // late_n counts from 1: index late_n - 1
      r.late_cycles = static_cast<uint32_t>(p.late);
      return r;
    }
    gWireGate.endReading();
    io.interruptsOn();
    if (stop & kControlAbort) { gWireGate.endWindow(); r.aborted = true; return r; }
    // the core's own tasks (and its watchdog) run; the window stays exclusive (no take: a command sent now would have the
    // target act while nothing samples), and the next burst starts with the turn it asked for
    io.yield();
  }
}

#undef OEP_SAMPLER_INLINE

}  // namespace sampler
}  // namespace oep
