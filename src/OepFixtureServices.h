// fixture.gpio / fixture.uart (owner 0, ids 0x20 / 0x21) on the probe's own GPIO. Pins
// are probe channels; the DUT wiring is the host's manifest. A shared pin table keeps
// GPIO and UART from driving the same channel. Per-core differences (pin modes, UART
// pin assignment) live in OepPlatform.h.
#pragma once

#include <Arduino.h>

#include "OepPlatform.h"
#include "OepService.h"

namespace oep {

class PinTable {
 public:
  static constexpr uint8_t kChannels = 64;
  explicit PinTable(const uint8_t *allowed, size_t count) {
    for (size_t i = 0; i < count && allowed[i] < kChannels; ++i) allowed_ |= uint64_t{1} << allowed[i];
  }
  explicit constexpr PinTable(uint64_t allowed) : allowed_(allowed) {}
  bool allowed(uint16_t channel) const { return channel < kChannels && (allowed_ >> channel) & 1; }
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

class FixtureGpio final : public Service {
 public:
  static constexpr uint8_t kOwner = 1;
  explicit FixtureGpio(PinTable &pins) : pins_(pins) {}
  uint16_t owner() const override { return OEP_V0_DEF_FIXTURE_GPIO_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_FIXTURE_GPIO_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_FIXTURE_GPIO_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t first, uint8_t *out, size_t capacity) override;
  void abandon() override;  // every configured channel back to floating input

 private:
  PinTable &pins_;
  uint64_t configured_ = 0;
};

class FixtureUart final : public Service {
 public:
  enum Role : uint8_t { kRoleRx = 1, kRoleTx = 2 };
  // `owner` is this instance's PinTable owner id (each UART instance needs its own so a
  // release only returns its own pins); 2 keeps the historical value for the first one.
  FixtureUart(PinTable &pins, OepUart &serial, uint8_t owner = 2) : pins_(pins), serial_(serial), kOwner(owner) {}
  uint16_t owner() const override { return OEP_V0_DEF_FIXTURE_UART_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_FIXTURE_UART_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_FIXTURE_UART_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t first, uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;

 private:
  PinTable &pins_;
  OepUart &serial_;
  const uint8_t kOwner;
  int rx_ = -1, tx_ = -1;
  bool configured_ = false;
};

}  // namespace oep
