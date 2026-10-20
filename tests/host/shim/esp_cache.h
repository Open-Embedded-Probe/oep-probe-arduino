// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's esp_cache.h for host tests (OEP_HOST_FAKE_PARLIO): no cache, nothing to sync.
#pragma once
#include <stddef.h>
#define ESP_CACHE_MSYNC_FLAG_INVALIDATE (1 << 0)
#define ESP_CACHE_MSYNC_FLAG_DIR_C2M (1 << 2)
#define ESP_CACHE_MSYNC_FLAG_DIR_M2C (1 << 3)
inline int esp_cache_msync(void *, size_t, int) { return 0; }
