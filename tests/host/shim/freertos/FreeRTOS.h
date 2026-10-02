// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#pragma once
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m) ((void)(m))
#define portENTER_CRITICAL_ISR(m) ((void)(m))
#define portEXIT_CRITICAL_ISR(m) ((void)(m))
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
