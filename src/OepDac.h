// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.dac revision 1 (oep-spec oep-if-fixture §9): an analog value on each output.
//   0x10 set(n u8, n x (role u8, value u32))   0x11 status -> n u8, n x (role u8, value u32, on u8)   (no lock)
// Role k is output k, on any of the chip's DAC pins (the sketch says which: `dac_pins`, a channel mask).
//
//   classic ESP32   its DAC on GPIO25 / 26: 8 bits, 0 - the supply (dacWrite / dacDisable)
//   WCH CH32        the DAC on PA4 / PA5 where the core has one (V307, H417): 12 bits, 0 - VDDA (analogWrite at 12 bits)
// The pad goes to its analog function while set, so a DAC channel is shared with nothing (the pin table claims it).
// OEP_HOST_FAKE_DAC: a host test's fake (the values recorded).
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

namespace oep {

class FixtureDac final : public Interface {
 public:
  static constexpr uint8_t kOwnerId = owners::kDac;
  static constexpr uint8_t kMaxChannels = 2;
#if defined(ARDUINO_ARCH_ESP32)
  static constexpr uint8_t kBits = 8;
#else
  static constexpr uint8_t kBits = 12;
#endif
  static constexpr int32_t kMinMv = 0, kMaxMv = 3300;   // nominal: the supply
  enum : uint8_t { kOpSet = reg::fixture_dac::kOpSet, kOpStatus = reg::fixture_dac::kOpStatus };

  FixtureDac(PinTable &pins, uint64_t dac_pins, uint16_t instance = 0) : pins_(pins), dac_pins_(dac_pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_dac::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_dac::kRevision; }
  bool lockFree(uint8_t op) const override { return op == kOpStatus; }
  bool offers(uint8_t op) const override { return op == kOpSet || op == kOpStatus; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t *out, size_t capacity) override;
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  uint8_t planRefusalCause() const override { return refusal_cause_; }
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  bool planShares() const override { return false; }

#if defined(OEP_HOST_FAKE_DAC)
  static inline uint32_t host_value[64] = {};   // by channel: the last value written, 0xFFFFFFFF off
#endif

 private:
  struct Out { int channel = -1; uint32_t value = 0; bool on = false; };
  PinTable &pins_;
  uint64_t dac_pins_;
  uint16_t instance_;
  uint8_t refusal_cause_ = reg::core::kUnavailableCauseWrongState;
  Out out_[kMaxChannels];
  void off(Out &o);
};

}  // namespace oep
