// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of FreeRTOS's binary semaphore (OEP_HOST_FAKE_SPI_SLAVE). Tasks run at once (freertos/task.h), so a take finds
// the give already made or none ever comes: then it fails (on the chip it would wait for ever).
#pragma once
#include <stdint.h>

#include <freertos/task.h>

struct StaticSemaphore_t { int count; };
typedef StaticSemaphore_t *SemaphoreHandle_t;
#ifndef portMAX_DELAY
#define portMAX_DELAY 0xffffffffu
#endif
inline SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *buffer) { buffer->count = 0; return buffer; }
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t s) { if (s->count) return pdFALSE; s->count = 1; return pdTRUE; }
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, uint32_t) { if (!s->count) return pdFALSE; s->count = 0; return pdTRUE; }
