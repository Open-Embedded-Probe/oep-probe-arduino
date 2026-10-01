// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The probe channels a sketch may hand to interfaces, who holds each one, and the state a free channel rests in
// (oep-spec oep-core §8: a released pin goes to its idle state, Hi-Z unless set otherwise). Shared by every
// interface that takes pins, so two of them never drive the same channel. Per-core pin modes live in OepPlatform.h.
#pragma once

#include <Arduino.h>

#include "OepPlatform.h"

namespace oep {

class PinTable {
 public:
  static constexpr uint8_t kChannels = 64;
  explicit PinTable(const uint8_t *allowed, size_t count) {
    for (size_t i = 0; i < count && allowed[i] < kChannels; ++i) allowed_ |= uint64_t{1} << allowed[i];
  }
  explicit constexpr PinTable(uint64_t allowed) : allowed_(allowed) {}
  bool allowed(uint16_t channel) const { return channel < kChannels && (allowed_ >> channel) & 1; }
  // Channels taken away at start-up (pins the chip in this package uses itself: platformUnusablePins).
  void forbid(uint64_t mask) { allowed_ &= ~mask; }
  bool free(uint16_t channel) const { return allowed(channel) && owner_[channel] == 0; }
  bool claim(uint16_t channel, uint8_t owner) {
    if (!free(channel)) return false;
    owner_[channel] = owner;
    return true;
  }
  // A released channel goes to its idle state (oep-core §8): Hi-Z unless the probe's settings (or its fixed wiring)
  // say pull-up / pull-down. Nothing keeps driving a pin nobody owns.
  void release(uint8_t owner) {
    for (uint8_t c = 0; c < kChannels; ++c)
      if (owner_[c] == owner) { owner_[c] = 0; applyIdle(c); }
  }
  // The same without touching the pads: for an owner that leaves its pins in a safe state itself (a debug wire's PHY
  // releases them Hi-Z; on the ESP32-P4 a pinMode on its pins would take them out of the dedicated GPIO bundle).
  void releaseQuiet(uint8_t owner) {
    for (uint8_t c = 0; c < kChannels; ++c)
      if (owner_[c] == owner) owner_[c] = 0;
  }
  uint64_t allowedMask() const { return allowed_; }
  uint8_t owner(uint16_t channel) const { return channel < kChannels ? owner_[channel] : 0xff; }
  // Idle states (oep.probe.config idle: 0 Hi-Z, 1 pull-up, 2 pull-down; kIdleUnset = Hi-Z). Applied now to a free
  // channel, and at every release. false: not a channel of this table, or a mode it does not know.
  static constexpr uint8_t kIdleHiZ = 0, kIdlePullUp = 1, kIdlePullDown = 2, kIdleUnset = 0xff;
  bool setIdle(uint16_t channel, uint8_t mode) {
    if (!allowed(channel) || (mode > kIdlePullDown && mode != kIdleUnset)) return false;
    idle_[channel] = mode;
    if (owner_[channel] == 0) applyIdle(static_cast<uint8_t>(channel));
    return true;
  }
  uint8_t idle(uint16_t channel) const { return channel < kChannels ? idle_[channel] : kIdleUnset; }

 private:
  uint64_t allowed_ = 0;
  uint8_t owner_[kChannels] = {};
  uint8_t idle_[kChannels] = {kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset};
  void applyIdle(uint8_t c) const {
    const uint8_t m = idle_[c];
    platformGpio(c, m == kIdlePullUp ? kGpioInputPullUp : m == kIdlePullDown ? kGpioInputPullDown : kGpioInputFloating);
  }
};

}  // namespace oep
