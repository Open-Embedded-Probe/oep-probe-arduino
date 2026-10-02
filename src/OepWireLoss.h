// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The wire-loss clock of one connection (oep-spec docs/oep-if-debug.md §2, limits.wire_lost_ms): the wire is lost when
// its exchanges have failed with no answer for wire_lost_ms of real time with no good exchange in between. One failed
// request (status line after its wire_retry_ms of retries) is not wire loss by itself; the clock starts at the first
// failure after a good exchange and stops at the next good one. The time the probe holds a reset line or asserts reset,
// and the wire_lost_ms after it lets go, is not counted (excuseReset at the release).
#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "OepRegistry.h"

namespace oep {

class WireLossClock {
 public:
  static constexpr uint32_t kLostMs = v1::reg::kLimitWireLostMs;
  void answered() { failing_ = false; }   // a good exchange
  void silent() {                         // an exchange that got nothing back
    if (failing_) return;
    if (excused_ && static_cast<int32_t>(millis() - excused_until_ms_) >= 0) excused_ = false;   // long over
    failing_ = true;
    since_ms_ = millis();
  }
  // A reset line was let go of (or a reset asserted through the wire ended) now: failures until kLostMs from now are
  // not counted.
  void excuseReset() {
    excused_ = true;
    excused_until_ms_ = millis() + kLostMs;
  }
  bool failing() const { return failing_; }
  // Failing for kLostMs of counted time (from the first failure, or from the end of a reset's excuse if later).
  bool lost() const {
    if (!failing_) return false;
    uint32_t from = since_ms_;
    if (excused_ && static_cast<int32_t>(excused_until_ms_ - from) > 0) from = excused_until_ms_;
    return static_cast<int32_t>(millis() - from) >= static_cast<int32_t>(kLostMs);
  }
  void clear() { failing_ = false; excused_ = false; }   // a new connection

 private:
  bool failing_ = false, excused_ = false;
  uint32_t since_ms_ = 0, excused_until_ms_ = 0;
};

}  // namespace oep
