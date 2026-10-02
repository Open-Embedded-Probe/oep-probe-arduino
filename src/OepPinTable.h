// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The probe channels a sketch may hand to interfaces, who holds each one, and the state a free channel rests in
// (oep-spec oep-core §8: a released pin goes to its idle state, Hi-Z unless set otherwise). Shared by every
// interface that takes pins, so two of them never drive the same channel. Per-core pin modes live in OepPlatform.h.
//
// Two ways a channel is taken away: forbid (the firmware's: a pin the chip in this package uses itself; permanent,
// describe never offers it) and setDisabled (oep.probe.config's disable item: the user's board does not wire it; the
// settings can give it back). A channel taken by a plan is not touched by the claim: it keeps its idle state (an output
// idle keeps driving) until its owner sets it (oep-if-fixture §1). A disabled channel is never claimed, parked or set to its idle state - the probe leaves
// it as the reset left it - and describe still offers it (declarations only, core §7.3).
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
  // The settings' disabled channels (probe.config §1 disable). A channel enabled again is free: it goes to its idle
  // state now. One disabled is left as it is (the settings refuse to disable a channel in use).
  void setDisabled(uint64_t mask) {
    const uint64_t back = disabled_ & ~mask;
    disabled_ = mask;
    for (uint8_t c = 0; c < kChannels; ++c)
      if (((back >> c) & 1) && allowed(c) && owner_[c] == 0) applyIdle(c);
  }
  bool disabled(uint16_t channel) const { return channel < kChannels && (disabled_ >> channel) & 1; }
  uint64_t disabledMask() const { return disabled_; }
  bool free(uint16_t channel) const { return allowed(channel) && !disabled(channel) && owner_[channel] == 0; }
  bool claim(uint16_t channel, uint8_t owner) {
    if (!free(channel)) return false;
    owner_[channel] = owner;
    return true;
  }
  // A released channel goes to its idle state (oep-core §8): Hi-Z unless the probe's settings (or its fixed wiring)
  // say pull-up / pull-down / output low / output high. Nothing but the settings' idle drives a pin nobody owns.
  void release(uint8_t owner) {
    for (uint8_t c = 0; c < kChannels; ++c)
      if (owner_[c] == owner) { owner_[c] = 0; if (!disabled(c)) applyIdle(c); }
  }
  // The same for an owner whose own driver already left its pins Hi-Z (a debug wire's PHY): only a channel with an idle
  // set is touched, so a channel without one keeps the PHY's state (on the ESP32-P4 a pinMode takes a pin out of the
  // dedicated GPIO bundle; the PHY routes it back at its next attach).
  void releaseToIdle(uint8_t owner) {
    for (uint8_t c = 0; c < kChannels; ++c)
      if (owner_[c] == owner) { owner_[c] = 0; if (!disabled(c) && idle_[c] != kIdleUnset) applyIdle(c); }
  }
  // Without touching the pads: for an owner taking its own pins again (holdPins, a plan applied over its old one).
  void releaseQuiet(uint8_t owner) {
    for (uint8_t c = 0; c < kChannels; ++c)
      if (owner_[c] == owner) owner_[c] = 0;
  }
  // A free channel something used without claiming it (attach's reset line) back to its idle state: as releaseToIdle.
  void rest(uint16_t channel) {
    if (free(channel) && idle_[channel] != kIdleUnset) applyIdle(static_cast<uint8_t>(channel));
  }
  uint64_t allowedMask() const { return allowed_; }
  uint8_t owner(uint16_t channel) const { return channel < kChannels ? owner_[channel] : 0xff; }
  // Idle states (oep.probe.config idle: 0 Hi-Z, 1 pull-up, 2 pull-down, 3 output low, 4 output high; kIdleUnset =
  // Hi-Z). Applied now to a free channel (apply false: only kept, for a channel about to be disabled), and at every
  // release; an output idle drives its level for as long as the channel is free. false: not a channel of this table, a
  // mode it does not know, or an output idle on a channel that cannot drive (setInputOnly).
  static constexpr uint8_t kIdleHiZ = 0, kIdlePullUp = 1, kIdlePullDown = 2, kIdleOutputLow = 3, kIdleOutputHigh = 4,
                           kIdleUnset = 0xff;
  bool setIdle(uint16_t channel, uint8_t mode, bool apply = true) {
    if (!allowed(channel) || (mode > kIdleOutputHigh && mode != kIdleUnset)) return false;
    if ((mode == kIdleOutputLow || mode == kIdleOutputHigh) && !canOutput(channel)) return false;
    idle_[channel] = mode;
    if (apply && owner_[channel] == 0 && !disabled(channel)) applyIdle(static_cast<uint8_t>(channel));
    return true;
  }
  uint8_t idle(uint16_t channel) const { return channel < kChannels ? idle_[channel] : kIdleUnset; }
  // Channels the probe can only read (the classic ESP32's GPIO34-39): no output idle on them (probe.config §1).
  void setInputOnly(uint64_t mask) { input_only_ = mask; }
  bool canOutput(uint16_t channel) const { return allowed(channel) && !((input_only_ >> channel) & 1); }

 private:
  uint64_t allowed_ = 0;
  uint64_t disabled_ = 0;   // the settings' disable items
  uint64_t input_only_ = 0;   // setInputOnly
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
    platformGpio(c, m == kIdlePullUp       ? kGpioInputPullUp
                    : m == kIdlePullDown   ? kGpioInputPullDown
                    : m == kIdleOutputLow  ? kGpioOutputLow
                    : m == kIdleOutputHigh ? kGpioOutputHigh
                                           : kGpioInputFloating);
  }
};

}  // namespace oep
