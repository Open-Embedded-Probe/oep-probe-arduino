// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: HID's output reports (transports §1, the HID rules) - a report whose count passes what it carries is
// dropped, and so is every report up to a pause of probe_frame_gap_ms with none; then reports go on again.
#include <stdio.h>

#include "OepFrame.h"
#include "OepRegistry.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;
static int failures = 0, checks = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

int main() {
  const uint32_t gap = oep::v1::reg::kProbeFrameGapMs;
  CHECK(oep::FrameReader::kIdleResyncMs == gap);   // the guard's pause is the registry's
  oep::HidCountGuard g;
  CHECK(g.accept(false, 0));                       // good reports go on
  CHECK(g.accept(false, 1));
  CHECK(!g.accept(true, 10));                      // a bad count: dropped
  CHECK(!g.accept(false, 11));                     // the good report right after: dropped too
  CHECK(!g.accept(false, 11 + gap - 1));           // each report inside the pause starts it again
  CHECK(!g.accept(false, 11 + 2 * gap - 2));
  CHECK(g.accept(false, 11 + 3 * gap));            // a whole pause with none: on again
  CHECK(g.accept(false, 11 + 3 * gap + 1));
  CHECK(!g.accept(true, 0xFFFFFFF0u));             // across the millis wrap
  CHECK(!g.accept(false, 0xFFFFFFF0u + 5));
  CHECK(g.accept(false, 0xFFFFFFF0u + 5 + gap));
  printf("hid-count: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
