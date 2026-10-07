// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The classic ESP32 sampler's loop (SamplerCapture, OepSampler.cpp), apart from the chip so that the host tests run the
// same code: an `Io` gives the cycle counter, one sample's read, interrupts off / on, the clock in ns and a yield
// (SamplerCapture: esp_cpu_get_cycle_count, GPIO.in, portDISABLE_INTERRUPTS, esp_timer, vTaskDelay(1)).
//
// Immediate: one exclusive window (OepWireGate.h), interrupts off throughout (<= 200 ms: below 327 kHz samples round down to 200 ms, inside the
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
// Every interrupts-off span ends by the clock, not by its count of samples (`span` cycles; 0 = no bound): at the fastest
// rates the loop cannot catch up on samples taken late, so the SWIO frames of a turn, a request already on the wire when
// the trigger came, or the other core and the radio's DMA holding the bus stretch a span by their own time - a search
// burst counted in samples ran past the 300 ms interrupt watchdog and reset the probe (bench, the V003 jig over TCP,
// 3169721: "int-wdt" during test_timing). A burst's search ends at `span` less the nominal time of the segment's rest;
// a segment still reading at `span` ends there (fewer samples than configured, slipped: its samples were late), and so
// does an immediate window. The clock is looked at every kSpanCheck samples.
#pragma once

#include <stdint.h>

#include <algorithm>

#include "OepWireGate.h"

namespace oep {
namespace sampler {

#define OEP_SAMPLER_INLINE inline __attribute__((always_inline))

constexpr uint8_t kControlForce = 1, kControlAbort = 2;
constexpr uint8_t kImmediate = 0, kLevel = 1, kEdge = 2;   // capture §3.3's trigger types
constexpr uint32_t kSpanCheck = 64;   // samples between two looks at the clock against the span (a power of two)

// One sample's byte from the GPIO input registers (bit l: line l's pin, GPIO0..31 in `in0`, GPIO32..39 in bits 0..7 of
// `in1`), through a table per register byte: a few loads and ors whatever the line count. The loop over the lines it
// replaces compiled to about a dozen instructions and two jumps a line (0.0.29-dev d520847, runLow) inside the 120
// cycles a sample has at 2 MHz and 240 MHz; with the register read and the loop's own work, a few lines left the pace no
// margin, and a sample held up by the other core's bus traffic was not caught up on before the next one fell late too.
struct Packer {
  uint8_t lut[5][256];   // [register byte][its value] -> the lines' bits it holds
  // pins[l] of line l (0..39; another value: no pin, the bit stays 0)
  void build(const int *pins, uint8_t lines) {
    for (int j = 0; j < 5; ++j)
      for (int v = 0; v < 256; ++v) {
        uint8_t b = 0;
        for (uint8_t l = 0; l < lines && l < 8; ++l)
          if (pins[l] >= 0 && pins[l] < 40 && pins[l] / 8 == j && ((v >> (pins[l] % 8)) & 1)) b |= static_cast<uint8_t>(1u << l);
        lut[j][v] = b;
      }
  }
  OEP_SAMPLER_INLINE uint8_t pack(uint32_t in0) const {
    return static_cast<uint8_t>(lut[0][in0 & 0xff] | lut[1][(in0 >> 8) & 0xff] | lut[2][(in0 >> 16) & 0xff] | lut[3][in0 >> 24]);
  }
  OEP_SAMPLER_INLINE uint8_t pack(uint32_t in0, uint32_t in1) const { return static_cast<uint8_t>(pack(in0) | lut[4][in1 & 0xff]); }
};

struct Plan {
  uint8_t *out;
  uint32_t size;        // the segment's samples at most (the buffer)
  uint32_t step;        // CPU cycles per sample
  uint32_t cpu_hz;
  uint8_t trig_type, trig_role, trig_value;
  uint32_t pre;         // pretrigger (< size)
  uint32_t burst;       // search samples per burst at most (+ the rest of the segment: kOffNs)
  uint32_t turn;        // samples of a wire turn
  uint32_t span;        // CPU cycles an interrupts-off span lasts at most (0: no bound)
};

struct Outcome {
  bool aborted = false;
  uint32_t samples = 0;           // the segment's
  uint32_t trigger_index = 0;     // its samples before the trigger's (immediate: 0xFFFFFFFF)
  uint64_t start_ns = 0;          // its first sample's time
  bool slipped = false;           // one of its samples taken a period or more late
};

template <class Io>
struct Pace {
  Io &io;
  uint8_t *out;
  uint32_t size, at, step, next, n, late_n;   // n: samples taken in this window; late_n: the last late one's n
};

// One sample at its time. The same few operations in an immediate window and in a trigger search: a look at the gate,
// the clock against the sample's time (a late one noted - a branch not taken), the wait, the read, the store.
template <bool kRing, class Io>
OEP_SAMPLER_INLINE uint8_t take(Pace<Io> &p) {
  gWireGate.yieldFrame();   // a SWIO frame announced has the bus first
  ++p.n;
  if (static_cast<int32_t>(p.io.cycles() - p.next) >= static_cast<int32_t>(p.step)) p.late_n = p.n;
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
  Pace<Io> p{io, plan.out, plan.size, 0, plan.step, 0, 0, 0};
  if (plan.trig_type == kImmediate) {
    gWireGate.beginWindow([&] { io.yield(); });
    io.interruptsOff();
    gWireGate.beginReading();
    r.start_ns = io.nowNs();   // the first sample's time (the start op's was earlier)
    p.next = io.cycles();
    const uint32_t span_start = p.next;
    uint32_t i = 0;
    bool cut = false;
    while (i < p.size) {
      take<false>(p);
      if ((++i & (kSpanCheck - 1)) == 0 && plan.span && io.cycles() - span_start >= plan.span) { cut = i < p.size; break; }
    }
    gWireGate.endReading();
    io.interruptsOn();
    gWireGate.endWindow();
    r.samples = i;
    r.trigger_index = 0xFFFFFFFFu;
    r.slipped = p.late_n != 0 || cut;
    return r;
  }
  const uint32_t pre = plan.pre, post = p.size - pre - 1;   // samples after the trigger's one
  const bool edge = plan.trig_type == kEdge;
  const uint8_t mask = static_cast<uint8_t>(1u << plan.trig_role), value = plan.trig_value;
  const uint8_t level = value ? mask : 0;                    // a level trigger's
  const bool rise = value != 1, fall = value != 0;           // an edge trigger's directions
  // the search's part of a span: what the segment's rest leaves of it at its nominal pace (at least one check's worth)
  const uint64_t rest = static_cast<uint64_t>(post) * plan.step;
  const uint32_t search_end = !plan.span ? 0
                              : rest + static_cast<uint64_t>(kSpanCheck) * plan.step < plan.span ? static_cast<uint32_t>(plan.span - rest)
                                                                                                    : kSpanCheck * plan.step;
  for (;;) {
    p.at = p.n = p.late_n = 0;
    if (control & kControlAbort) { gWireGate.endWindow(); r.aborted = true; return r; }
    bool turned = gWireGate.beginBurst();                     // the wire held or asked for: a turn at once
    uint32_t turn_left = turned ? plan.turn : 0;
    io.interruptsOff();
    gWireGate.beginReading();
    const uint64_t burst_ns = io.nowNs();
    p.next = io.cycles();
    const uint32_t span_start = p.next;
    uint8_t stop = 0;
    bool hit = false;
    // The burst's first sample: a level there is the trigger; an edge needs the sample before it, so it is only its level.
    uint8_t last = take<true>(p) & mask;
    uint32_t c = 0;
    if (!edge && last == level) hit = true;
    else c = 1;
    // Blocks of up to kSpanCheck samples, each sample a take and the trigger's test alone (as lean as an immediate
    // window's); between blocks the clock against the span, the wire's turns and the control word (force, abort): a
    // turn or a force waits at most a block (32 us at 2 MHz, 64 at 1 MHz). Looking at those every sample took several
    // loads of shared words (each behind a memory barrier) and spilled the loop's state to the stack: the search ran
    // late where an immediate window kept pace (bench, 87f6d40: forced captures slipped 64 / 64 at 500 kHz to 2 MHz,
    // immediate ones from 750 kHz).
    while (!hit && c < plan.burst) {
      if (search_end && io.cycles() - span_start >= search_end) break;
      if (turn_left) {
        turn_left = turn_left > kSpanCheck ? turn_left - kSpanCheck : 0;
        if (turn_left == 0 || gWireGate.turnDone()) { turn_left = 0; gWireGate.closeTurn(); }
      } else if (const uint32_t want = gWireGate.turnWanted()) {
        if (want == 2 || !turned) { gWireGate.openTurn(); turned = true; turn_left = plan.turn; }
      }
      if ((stop = control) != 0) {   // force: the trigger is the next sample; abort: no more
        if (stop & kControlForce) take<true>(p);
        break;
      }
      const uint32_t end = plan.burst - c < kSpanCheck ? plan.burst : c + kSpanCheck;
      if (edge) {
        for (; c < end; ++c) {
          const uint8_t now = take<true>(p) & mask;
          if (now != last && (now ? rise : fall)) { hit = true; break; }
          last = now;
        }
      } else {
        for (; c < end; ++c) {
          if ((take<true>(p) & mask) == level) { hit = true; break; }
        }
      }
      if (hit) break;
    }
    if (hit || (stop & kControlForce)) {
      gWireGate.closeTurn();                                  // the segment's rest: no new frame
      const uint32_t kept = c < pre ? c : pre, first = c - kept;
      onTrigger(c, kept, burst_ns);
      uint32_t got = 0;
      bool cut = false;
      while (got < post) {
        take<true>(p);
        if ((++got & (kSpanCheck - 1)) == 0 && plan.span && io.cycles() - span_start >= plan.span) { cut = got < post; break; }
      }
      gWireGate.endReading();
      io.interruptsOn();
      gWireGate.endWindow();
      // the segment is the ring's last kept + 1 + got samples: its oldest first (c < pre: from 0, no wrap, nothing to turn)
      const uint32_t seg = kept + 1 + got;
      const uint32_t oldest = (p.at + p.size - seg) % p.size;
      if (oldest) std::rotate(p.out, p.out + oldest, p.out + p.size);
      r.samples = seg;
      r.trigger_index = kept;
      r.start_ns = burst_ns + static_cast<uint64_t>(first) * p.step * 1000000000ull / plan.cpu_hz;
      r.slipped = p.late_n > first || cut;                               // late_n counts from 1: index late_n - 1
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
