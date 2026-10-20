// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's async memcpy (esp_async_memcpy.h) for host tests (OEP_HOST_FAKE_PARLIO): a copy is done at
// once and its callback called, or (g_fake_async.defer) held until fakeAsyncFinish() - in order, as one GDMA channel
// does them. The bytes are read at the finish, as the DMA reads them when it gets there.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <deque>
#include <functional>

#ifndef ESP_OK
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#endif
#ifndef ESP_ERR_INVALID_ARG
#define ESP_ERR_INVALID_ARG 0x102
#endif
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

struct FakeMcp { int unused; };
typedef FakeMcp *async_memcpy_handle_t;
typedef struct { void *data; } async_memcpy_event_t;
typedef bool (*async_memcpy_isr_cb_t)(async_memcpy_handle_t, async_memcpy_event_t *, void *);
typedef struct { uint32_t backlog; size_t dma_burst_size; uint32_t flags; } async_memcpy_config_t;
#define ASYNC_MEMCPY_DEFAULT_CONFIG() {8, 16, 0}

struct FakeAsync {
  bool defer = false, fail_install = false;
  int calls = 0;
  std::deque<std::function<void()>> pending;
};
inline FakeAsync g_fake_async;

inline esp_err_t esp_async_memcpy_install_gdma_axi(const async_memcpy_config_t *, async_memcpy_handle_t *mcp) {
  if (g_fake_async.fail_install) return ESP_FAIL;
  *mcp = new FakeMcp{};
  return ESP_OK;
}
inline esp_err_t esp_async_memcpy_uninstall(async_memcpy_handle_t mcp) { delete mcp; return ESP_OK; }
inline esp_err_t esp_async_memcpy(async_memcpy_handle_t mcp, void *dst, void *src, size_t n, async_memcpy_isr_cb_t cb,
                                  void *args) {
  if (!mcp || !dst || !src || !n) return ESP_ERR_INVALID_ARG;
  ++g_fake_async.calls;
  auto run = [=] {
    memcpy(dst, src, n);
    if (cb) cb(mcp, nullptr, args);
  };
  if (g_fake_async.defer) g_fake_async.pending.push_back(run);
  else run();
  return ESP_OK;
}
inline void fakeAsyncFinish() {   // every copy held, oldest first
  while (!g_fake_async.pending.empty()) {
    auto f = g_fake_async.pending.front();
    g_fake_async.pending.pop_front();
    f();
  }
}
