// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The probe channels a sketch may hand to interfaces, who holds each one, and the state a free channel rests in
// (oep-spec oep-core §8: a released pin goes to its idle state, Hi-Z unless set otherwise). Shared by every
// interface that takes pins, so two of them never drive the same channel. Per-core pin modes live in OepPlatform.h.
//
// Two ways a channel is taken away: forbid (the firmware's: a pin the chip in this package uses itself; permanent,
// describe never offers it) and setDisabled (the settings' disable item: the user's board does not wire it; the
// settings can give it back). A channel taken by a plan is not touched by the claim: it keeps its idle state (an output
// idle keeps driving) until its owner sets it (oep-if-fixture §1). A disabled channel is never claimed, parked or set to its idle state - the probe leaves
// it as the reset left it - and describe still offers it (declarations only, core §7.3).
#pragma once

#include <Arduino.h>

#include "OepPlatform.h"

namespace oep {

// Who holds a channel (PinTable::claim's owner; 0 = nobody): one list for every part of this library, so no two parts in
// one firmware share an id. A part of the sketch's own takes kSketch and up (to 0xef).
namespace owners {
constexpr uint8_t kGpio = 1;            // oep.fixture.gpio
constexpr uint8_t kUart = 2;            // oep.fixture.uart (the first; kUart2 a second instance)
constexpr uint8_t kI2cTarget = 3;       // oep.fixture.i2c-target
constexpr uint8_t kUart2 = 5;
constexpr uint8_t kSpiTarget = 6;       // oep.fixture.spi-target
constexpr uint8_t kAnalog = 7;          // oep.fixture.analog (its pads go analog)
constexpr uint8_t kI2cController = 8;   // oep.fixture.i2c-controller
constexpr uint8_t kSpiController = 9;   // oep.fixture.spi-controller
constexpr uint8_t kPwm = 10;            // oep.fixture.pwm
constexpr uint8_t kDac = 11;            // oep.fixture.dac
constexpr uint8_t kSketch = 0x80;       // the sketch's own: 0x80 - 0xef
constexpr uint8_t kRiscvWire = 0xf0;    // a debug wire's live connection (DebugPort::pin_owner's default)
constexpr uint8_t kSwdWire = 0xf1;      // SwdPort::pin_owner's default
constexpr uint8_t kSecondWire = 0xf2;   // a second DebugPort beside the first (the SWIO wire next to RVSWD)
}  // namespace owners

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
  // The settings' disabled channels (core §13.8 disable). A channel enabled again is free: it goes to its idle
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
  // Free, or held by `owner` itself: a plan_apply's check of the plan that replaces `owner`'s own (oep-if-plan §2.1)
  bool freeFor(uint16_t channel, uint8_t owner) const {
    return allowed(channel) && !disabled(channel) && (owner_[channel] == 0 || owner_[channel] == owner);
  }
  bool claim(uint16_t channel, uint8_t owner) {
    if (!free(channel)) return false;
    owner_[channel] = owner;
    pending_ &= ~(uint64_t{1} << channel);   // taken again within a replacement: its pad untouched
    return true;
  }
  // A plan replacement (oep-core §8, oep-if-fixture §1): between deferIdle and settleIdle a released channel is only
  // marked; settleIdle puts the ones nobody took again in their idle state. A channel in both the old and the new plan
  // keeps its current state and drive; one leaving goes to its idle state; a new one stays as it was (its idle state).
  void deferIdle() { ++deferring_; }
  void settleIdle() {
    if (deferring_ && --deferring_) return;
    const uint64_t pending = pending_;
    pending_ = 0;
    for (uint8_t c = 0; c < kChannels; ++c)
      if (((pending >> c) & 1) && owner_[c] == 0 && !disabled(c)) applyIdle(c);
  }
  // A released channel goes to its idle state (oep-core §8): Hi-Z unless the probe's settings (or its fixed wiring)
  // say pull-up / pull-down / output low / output high. Nothing but the settings' idle drives a pin nobody owns.
  void release(uint8_t owner) {
    for (uint8_t c = 0; c < kChannels; ++c)
      if (owner_[c] == owner) {
        owner_[c] = 0;
        if (deferring_) pending_ |= uint64_t{1} << c;
        else if (!disabled(c)) applyIdle(c);
      }
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
  // Idle states (the settings (core §13) idle: 0 Hi-Z, 1 pull-up, 2 pull-down, 3 output low, 4 output high; kIdleUnset =
  // Hi-Z). Applied now to a free channel (apply false: only kept, for a channel about to be disabled), and at every
  // release; an output idle drives its level for as long as the channel is free, at its strength `drive` (a level of
  // platformDriveLevels; kDriveDefault: the default level) - level and strength together. false: not a channel of this
  // table, a mode it does not know, an output idle on a channel that cannot drive (setInputOnly), or a pull-up /
  // pull-down idle on a channel without that pull (setNoPull).
  static constexpr uint8_t kIdleHiZ = 0, kIdlePullUp = 1, kIdlePullDown = 2, kIdleOutputLow = 3, kIdleOutputHigh = 4,
                           kIdleUnset = 0xff;
  static constexpr uint8_t kDriveDefault = 0xff;   // no strength given: the default level (the strength is the probe's, oep-if-fixture §1)
  bool setIdle(uint16_t channel, uint8_t mode, bool apply = true, uint8_t drive = kDriveDefault) {
    if (!allowed(channel) || (mode > kIdleOutputHigh && mode != kIdleUnset)) return false;
    if ((mode == kIdleOutputLow || mode == kIdleOutputHigh) && !canOutput(channel)) return false;
    if ((mode == kIdlePullUp || mode == kIdlePullDown) && !canPull(channel)) return false;
    idle_[channel] = mode;
    idle_drive_[channel] = drive == kDriveDefault ? 0 : static_cast<uint8_t>(drive + 1);
    if (apply && owner_[channel] == 0 && !disabled(channel)) applyIdle(static_cast<uint8_t>(channel));
    return true;
  }
  uint8_t idle(uint16_t channel) const { return channel < kChannels ? idle_[channel] : kIdleUnset; }
  uint8_t idleDrive(uint16_t channel) const {
    return channel < kChannels && idle_drive_[channel] ? static_cast<uint8_t>(idle_drive_[channel] - 1) : kDriveDefault;
  }

  // Output drive strength (oep-if-fixture §1.1). A pad set to `mode` (the platform's, OepPlatform.h); output low / high
  // at `level` (kDriveDefault or out of range: the default level), everything else without one - a pad an earlier
  // output left at another strength goes back to the default, so whoever takes the pin next starts from the pad's own.
  // Used by the idle states and the gpio fixture's set; never by the wires or the other fixtures.
  void setPad(uint8_t c, uint8_t mode, uint8_t level) {
    const DriveLevels d = platformDriveLevels();
    const uint64_t bit = uint64_t{1} << c;
    if (d.count && (mode == kGpioOutputLow || mode == kGpioOutputHigh)) {
      const uint8_t l = level < d.count ? level : d.default_level;
      platformGpioDriven(c, mode, l);
      level_[c] = static_cast<uint8_t>(l + 1);
      if (l != d.default_level) strong_ |= bit;
      else strong_ &= ~bit;
      return;
    }
    platformGpio(c, mode);
    level_[c] = 0;
    if (strong_ & bit) {
      platformDrive(c, d.default_level);
      strong_ &= ~bit;
    }
  }
  // For a fixture that takes a channel for its own peripheral (UART, I2C / SPI target): a pad an output idle left at
  // another strength goes back to the default, as setPad does when a pad leaves mode 3 / 4. Not for a debug wire (its
  // PHY sets the weakest itself, and claims its pins only once they run) nor the gpio fixture (it keeps the idle state).
  void ownStrength(uint16_t channel) {
    if (channel >= kChannels) return;
    const uint64_t bit = uint64_t{1} << channel;
    level_[channel] = 0;
    if (!(strong_ & bit)) return;
    platformDrive(channel, platformDriveLevels().default_level);
    strong_ &= ~bit;
  }
  // A drive level (fixture §1.1: a level number of drive_levels, 0xFF its default level) as a level of `d`. false: a
  // level past drive_levels, or any level on a probe without drive_levels (refused unsupported).
  static bool driveLevelOf(const DriveLevels &d, uint8_t value, uint8_t &level) {
    if (!d.count) return false;
    if (value == kDriveDefault) { level = d.default_level; return true; }
    if (value >= d.count) return false;
    level = value;
    return true;
  }
  // Channels the probe can only read (the classic ESP32's GPIO34-39): no output idle on them (core §13.8), and left
  // out of the role_channels of the roles that drive a line (outputMask: UART TX, I2C SDA / SCL, SPI MISO).
  void setInputOnly(uint64_t mask) { input_only_ = mask; }
  bool canOutput(uint16_t channel) const { return allowed(channel) && !((input_only_ >> channel) & 1); }
  uint64_t outputMask() const { return allowed_ & ~input_only_; }
  // Channels without internal pull-up / pull-down (the classic ESP32's GPIO34-39): no idle with mode 1 / 2 on them
  // (core §13.8, rejected unsupported).
  void setNoPull(uint64_t mask) { no_pull_ = mask; }
  bool canPull(uint16_t channel) const { return allowed(channel) && !((no_pull_ >> channel) & 1); }

 private:
  uint64_t allowed_ = 0;
  uint64_t disabled_ = 0;   // the settings' disable items
  uint64_t input_only_ = 0;   // setInputOnly
  uint64_t no_pull_ = 0;      // setNoPull
  uint64_t pending_ = 0;      // released during a replacement, not yet settled (deferIdle)
  uint64_t strong_ = 0;       // pads setPad left at another strength than the default
  uint8_t deferring_ = 0;
  uint8_t owner_[kChannels] = {};
  uint8_t idle_[kChannels] = {kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset,
                              kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset, kIdleUnset};
  // per channel, one more than the level (0: none) - zero-initialised: the idle without a strength, not driven
  uint8_t idle_drive_[kChannels] = {};   // the idle's strength (setIdle)
  uint8_t level_[kChannels] = {};        // the level a mode 3 / 4 output is driven at now (setPad)
  void applyIdle(uint8_t c) {
    const uint8_t m = idle_[c];
    setPad(c, m == kIdlePullUp       ? kGpioInputPullUp
              : m == kIdlePullDown   ? kGpioInputPullDown
              : m == kIdleOutputLow  ? kGpioOutputLow
              : m == kIdleOutputHigh ? kGpioOutputHigh
                                     : kGpioInputFloating,
           idleDrive(c));
  }
};

// oep-if-debug §2: whether a debug wire holds its lines at their rest after a close that is not a lost line - the
// setting hold.{} of the port's SWDIO (the settings (core §13) sets `on`; none: on) and no idle / disable item on its pins.
// `check` (set by the wire) ends a hold whose pins were taken or whose settings changed; Settings::poll calls it.
struct WireHold {
  bool (*on)(const void *, uint16_t swdio) = nullptr;
  const void *context = nullptr;
  bool allows(const PinTable *pins, uint16_t swdio, uint16_t swclk) const {
    if (on && !on(context, swdio)) return false;
    if (!pins) return true;
    const uint16_t both[2] = {swdio, swclk};
    for (uint16_t ch : both)
      if (ch != 0xffff && (pins->disabled(ch) || pins->idle(ch) != PinTable::kIdleUnset)) return false;
    return true;
  }
};

}  // namespace oep
