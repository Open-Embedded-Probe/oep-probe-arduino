// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host-test stand-in for ESP-IDF's esp_cpu.h: a cycle counter that counts calls (a cost the tests do not measure).
#pragma once
#include <stdint.h>

inline uint32_t esp_cpu_get_cycle_count() {
  static uint32_t n = 0;
  return ++n;
}
