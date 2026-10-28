// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the classic ESP32 sampler's external-clock window (OepSamplerRun.h runClocked, capture §3.8) on a
// scripted line - each read is 10 cycles; the clock (bit 7) toggles every 3 reads, the data lines count the clock's
// rises, a qualifier on bit 1. A sample at each edge asked, its byte without the clock, format 3's deltas in cycles
// (the first from the window's start), the window ended by its span with fewer samples.
#include <stdio.h>

#include <vector>

#include "OepSamplerRun.h"

using namespace oep;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// read k: the clock high in reads 3..5, 9..11, ... (low first); the data the number of rises before (low 7 bits)
struct ScriptIo {
  mutable uint32_t reads = 0;
  uint32_t cycles() const { return reads * 10; }
  uint8_t read() const {
    const uint32_t k = reads++;
    const uint8_t clock = (k / 3) & 1 ? sampler::kClockBit : 0;
    return static_cast<uint8_t>(clock | (((k + 3) / 6) & 0x7F));
  }
  void interruptsOff() const {}
  void interruptsOn() const {}
  uint64_t nowNs() const { return 0; }
  void yield() const {}
};

static uint32_t u32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | static_cast<uint32_t>(p[3]) << 24; }

int main() {
  const volatile uint8_t control = 0;
  {   // rising, the bytes alone: the rise at read 3, 9, 15, ... holds the count so far
    ScriptIo io;
    uint8_t out[16] = {};
    sampler::ClockPlan plan{out, 5, 0, 0, 1, false, 0};
    const sampler::ClockOutcome r = sampler::runClocked(io, plan, control);
    CHECK(r.samples == 5 && !r.aborted);
    for (uint8_t i = 0; i < 5; ++i) CHECK(out[i] == static_cast<uint8_t>(i + 1));
    CHECK(r.first_cycles == 40);   // read 3 has the rise; the cycles are read after it (4 reads x 10)
  }
  {   // both edges, format 3: a sample every 3 reads (30 cycles), the first delta from the window's start
    ScriptIo io;
    uint8_t out[64] = {};
    sampler::ClockPlan plan{out, 6, 2, 0, 1, true, 0};
    const sampler::ClockOutcome r = sampler::runClocked(io, plan, control);
    CHECK(r.samples == 6);
    CHECK(u32(out) == r.first_cycles);
    for (uint8_t i = 1; i < 6; ++i) CHECK(u32(out + 5 * i) == 30);
  }
  {   // a qualifier on bit 1 at level 1: only the rises whose count has bit 1 set
    ScriptIo io;
    uint8_t out[16] = {};
    sampler::ClockPlan plan{out, 4, 0, 0x02, 1, false, 0};
    sampler::runClocked(io, plan, control);
    CHECK(out[0] == 2 && out[1] == 3 && out[2] == 6 && out[3] == 7);
  }
  {   // the span ends the window: fewer samples than asked
    ScriptIo io;
    uint8_t out[1024] = {};
    sampler::ClockPlan plan{out, 1000, 0, 0, 1, false, 2000};
    const sampler::ClockOutcome r = sampler::runClocked(io, plan, control);
    CHECK(r.samples > 0 && r.samples < 1000 && !r.aborted);
  }
  printf("sampler clock: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
