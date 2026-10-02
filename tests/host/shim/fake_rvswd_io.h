// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The pin primitives of RvswdPhy's host backend (OEP_HOST_FAKE_RVSWD, OepRvswdPhy.cpp): a test defines them against
// its simulated target (test_rvswd_phy.cpp), which decodes the frames from the edges and keeps the time.
#pragma once
#include <stdint.h>

struct FakeRvswdIo {
  void spin() const;
  void bothHigh() const;
  void clkLowDio(bool v) const;
  void clkHigh() const;
  void clk(bool v) const;
  void dio(bool v) const;
  bool dioRead() const;
  void hostDrives(bool yes) const;
  // the backend's pad setup
  bool begin(int dio, int clk) const;
  void pullUp(bool on) const;
  void clkPull(int dir) const;   // SWCLK's pull: 1 up, -1 down, 0 none
  void driveBoth(bool on) const;
  uint32_t setHalfNs(uint32_t half_ns) const;
};
