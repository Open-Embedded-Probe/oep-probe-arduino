// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the analog capture's scale (oep-spec oep-if-capture §1.2 rule 4, §3.3 tag 0x55) from a frontend's
// declared range (§3.5 tag 0x46): voltage = (value - zero) x scale_nv gives range_min_mv at value 0 and range_max_mv
// at full scale. The classic ESP32's ranges start above 0 V (100 / 150 mV): its zero is below 0 (it was sent as 0,
// which read 150 mV low at 12 dB); the P4's and the RP2's start at 0 V, zero stays 0.
#include <stdio.h>

#include "OepAnalog.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

static double volts_mv(uint32_t value, int32_t zero, int32_t scale_nv) {
  return (static_cast<double>(value) - zero) * scale_nv / 1e6;
}

int main() {
  struct Case { int32_t min_mv, max_mv; int32_t zero; };
  // classic ESP32 (0 / 2.5 / 6 / 12 dB), ESP32-P4, RP2 (ADC_VREF)
  const Case cases[] = {{100, 950, -482}, {100, 1250, -356}, {150, 1750, -384}, {150, 2450, -267},
                        {0, 950, 0}, {0, 1250, 0}, {0, 1750, 0}, {0, 3100, 0}, {0, 3300, 0}};
  for (const Case &c : cases) {
    int32_t zero = 1, scale = 0;
    oep::analogScale(c.min_mv, c.max_mv, 4095, zero, scale);
    CHECK(zero == c.zero);
    CHECK(scale > 0);
    // the ends of the declared range within half a value (and the rounding of zero)
    const double step = scale / 1e6;
    CHECK(volts_mv(0, zero, scale) > c.min_mv - step && volts_mv(0, zero, scale) < c.min_mv + step);
    CHECK(volts_mv(4095, zero, scale) > c.max_mv - step && volts_mv(4095, zero, scale) < c.max_mv + step);
    if (zero != c.zero) printf("  %d..%d mV: zero %d\n", c.min_mv, c.max_mv, zero);
  }
  int32_t zero = 0, scale = 0;
  oep::analogScale(150, 2450, 4095, zero, scale);
  CHECK(scale == 561661);   // 2300 mV / 4095, nearest nV
  printf("analog-scale: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
