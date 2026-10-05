// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's esp_timer.h for host tests: the shim's clock.
#pragma once
#include <Arduino.h>
inline int64_t esp_timer_get_time() { return static_cast<int64_t>(g_millis) * 1000 + g_micros_part; }
