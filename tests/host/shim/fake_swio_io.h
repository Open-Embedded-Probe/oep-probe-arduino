// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The frame primitives of SwioPhy's host backend (OEP_HOST_FAKE_SWIO, OepSwioPhy.cpp): a test defines them against its
// simulated target (test_swio.cpp). A read the target leaves unanswered reads all ones (the line at its pull-up);
// false: the line never came back high.
#pragma once
#include <stdint.h>

bool fakeSwioRead(uint8_t address, uint32_t &value);
void fakeSwioWrite(uint8_t address, uint32_t value, bool free_after);
bool fakeSwioLineHigh();   // the line, left to its pull-up, reads high (a target is there)
