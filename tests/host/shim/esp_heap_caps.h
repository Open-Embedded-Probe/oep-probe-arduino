// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's heap_caps for host tests: two pools, internal RAM and PSRAM, each one free block of g_fake_heap's
// size (largest free block = what is left of it; a test sets the sizes to model a board, e.g. no PSRAM and little
// internal RAM). Allocations are real (the code under test writes into them) and are counted against their pool.
#pragma once
#include <stdlib.h>

#include <map>

#define MALLOC_CAP_DMA (1 << 3)
#define MALLOC_CAP_INTERNAL (1 << 11)
#define MALLOC_CAP_8BIT (1 << 2)
#define MALLOC_CAP_SPIRAM (1 << 10)

struct FakeHeap {
  size_t internal_free = 512 * 1024;   // the largest free internal block
  size_t psram_total = 0, psram_free = 0;
  std::map<void *, std::pair<size_t, bool>> live;   // pointer -> size, in PSRAM
  int failed = 0;                                   // allocations refused
};
inline FakeHeap g_fake_heap;

inline void *fakeHeapTake(size_t align, size_t size, int caps) {
  const bool psram = caps & MALLOC_CAP_SPIRAM;
  size_t &pool = psram ? g_fake_heap.psram_free : g_fake_heap.internal_free;
  if (size > pool) { ++g_fake_heap.failed; return nullptr; }
  void *p = aligned_alloc(align, (size + align - 1) / align * align);
  if (!p) return nullptr;
  pool -= size;
  g_fake_heap.live[p] = {size, psram};
  return p;
}
inline void *heap_caps_aligned_alloc(size_t align, size_t size, int caps) { return fakeHeapTake(align, size, caps); }
inline void *heap_caps_malloc(size_t size, int caps) { return fakeHeapTake(16, size, caps); }
inline void heap_caps_free(void *p) {
  auto it = g_fake_heap.live.find(p);
  if (it == g_fake_heap.live.end()) return;
  (it->second.second ? g_fake_heap.psram_free : g_fake_heap.internal_free) += it->second.first;
  g_fake_heap.live.erase(it);
  free(p);
}
inline size_t heap_caps_get_largest_free_block(int caps) {
  return caps & MALLOC_CAP_SPIRAM ? g_fake_heap.psram_free : g_fake_heap.internal_free;
}
inline size_t heap_caps_get_total_size(int caps) { return caps & MALLOC_CAP_SPIRAM ? g_fake_heap.psram_total : 0; }
// esp_memory_utils.h's: whether a pointer is in PSRAM
inline bool esp_ptr_external_ram(const void *p) {
  auto it = g_fake_heap.live.find(const_cast<void *>(p));
  return it != g_fake_heap.live.end() && it->second.second;
}
