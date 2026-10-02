// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's esp_ipc_call_blocking (OEP_HOST_FAKE_SPI_SLAVE): the call runs at once, with g_fake_core set to
// the core it was sent to, so a test can see where something was set up (the SPI target's CS handler: core 0).
#pragma once
#include <stdint.h>

#include <driver/gpio.h>

typedef void (*esp_ipc_func_t)(void *arg);
inline esp_err_t esp_ipc_call_blocking(uint32_t cpu, esp_ipc_func_t func, void *arg) {
  const int was = g_fake_core;
  g_fake_core = static_cast<int>(cpu);
  func(arg);
  g_fake_core = was;
  return ESP_OK;
}
