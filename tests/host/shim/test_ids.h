// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: each test interface object its own instance - an endpoint adds one (name, revision, instance) once
// (oep-spec core §7.2), and the tests add several objects of one test class.
#pragma once

#include <stdint.h>

inline uint16_t nextTestInstance() {
  static uint16_t next = 0;
  return next++;
}
