// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of the little of FreeRTOS's task API the SPI target uses (OEP_HOST_FAKE_SPI_SLAVE): a task pinned to a core
// runs at once, to its end (vTaskDelete), with g_fake_core set to that core.
#pragma once
#include <stdint.h>

#include <fake_core.h>
#include <freertos/FreeRTOS.h>

typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
typedef int BaseType_t;
typedef unsigned UBaseType_t;
#ifndef pdPASS
#define pdPASS 1
#define pdFAIL 0
#define pdTRUE 1
#define pdFALSE 0
#endif
inline int g_fake_tasks = 0;   // tasks created
// A test that drives a task itself (the logic capture's harvest, which loops until stopped): created tasks are only
// kept (g_fake_task_fn / arg) for the test to run when it wants.
inline bool g_fake_tasks_deferred = false;
inline TaskFunction_t g_fake_task_fn = nullptr;
inline void *g_fake_task_arg = nullptr;
inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *, uint32_t, void *arg, UBaseType_t,
                                          TaskHandle_t *, int core) {
  ++g_fake_tasks;
  if (g_fake_tasks_deferred) {
    g_fake_task_fn = fn;
    g_fake_task_arg = arg;
    return pdPASS;
  }
  const int was = g_fake_core;
  g_fake_core = core;
  fn(arg);
  g_fake_core = was;
  return pdPASS;
}
inline UBaseType_t uxTaskPriorityGet(TaskHandle_t) { return 1; }
inline void vTaskDelete(TaskHandle_t) {}
