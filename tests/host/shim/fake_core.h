// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests (OEP_HOST_FAKE_SPI_SLAVE): the core the code under test runs on - loop() on core 1; esp_ipc.h and
// freertos/task.h move it for the call or the task they run - and which core's IPC task is busy with a call.
#pragma once

inline int g_fake_core = 1;
inline bool g_fake_ipc_busy[2] = {};
inline int g_fake_ipc_deadlocks = 0;   // IPC calls that would never have returned on the chip
