// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of FreeRTOS queues for host tests (OEP_HOST_FAKE_PARLIO): a bounded FIFO of fixed-size items.
#pragma once
#include <string.h>

#include <deque>
#include <vector>

#include <freertos/task.h>

typedef uint32_t TickType_t;
#ifndef portMAX_DELAY
#define portMAX_DELAY 0xffffffffu
#endif
struct FakeQueue {
  size_t depth, size;
  std::deque<std::vector<uint8_t>> items;
};
typedef FakeQueue *QueueHandle_t;
inline QueueHandle_t xQueueCreate(size_t depth, size_t size) { return new FakeQueue{depth, size, {}}; }
inline void vQueueDelete(QueueHandle_t q) { delete q; }
inline BaseType_t xQueueReset(QueueHandle_t q) { q->items.clear(); return pdPASS; }
inline BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item, BaseType_t *woken) {
  if (woken) *woken = pdFALSE;
  if (q->items.size() >= q->depth) return pdFALSE;
  const uint8_t *b = static_cast<const uint8_t *>(item);
  q->items.emplace_back(b, b + q->size);
  return pdTRUE;
}
#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(ms) (ms)
#endif
inline UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q) { return static_cast<UBaseType_t>(q->items.size()); }
inline BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t) {
  if (q->items.empty()) return pdFALSE;
  memcpy(item, q->items.front().data(), q->size);
  q->items.pop_front();
  return pdTRUE;
}
