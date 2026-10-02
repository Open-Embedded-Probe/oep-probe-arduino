// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's esp_ipc_call_blocking (OEP_HOST_FAKE_SPI_SLAVE): the call runs at once, with g_fake_core set to
// the core it was sent to, so a test can see where something was set up (the SPI target's CS handler: core 0).
// As on the chip, a core's IPC task does one call at a time and the caller holds that core's IPC lock until it returns:
// a call to a core whose IPC task is busy - made from inside an IPC call, or from a task that call waits for - never
// returns there. Here it is counted in g_fake_ipc_deadlocks and fails.
#pragma once
#include <stdint.h>

#include <fake_core.h>

#ifndef ESP_OK
#define ESP_OK 0
#define ESP_FAIL -1
#endif
typedef int esp_err_t;
typedef void (*esp_ipc_func_t)(void *arg);
inline esp_err_t esp_ipc_call_blocking(uint32_t cpu, esp_ipc_func_t func, void *arg) {
  if (g_fake_ipc_busy[cpu]) { ++g_fake_ipc_deadlocks; return ESP_FAIL; }
  const int was = g_fake_core;
  g_fake_core = static_cast<int>(cpu);
  g_fake_ipc_busy[cpu] = true;
  func(arg);
  g_fake_ipc_busy[cpu] = false;
  g_fake_core = was;
  return ESP_OK;
}
