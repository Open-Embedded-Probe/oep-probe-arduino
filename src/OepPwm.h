// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.pwm revision 1 (oep-spec oep-if-fixture §7): a square wave of a frequency and duty on each output.
//   0x10 set(n u8, n x (role u8, hz u32, duty u16)) -> n x (role u8, num u32, den u32, duty u16): the output made
//   0x11 status -> n u8, n x (role, num, den, duty)   (no lock)
// Role k is output k (0 .. kMaxChannels - 1). hz 0 stops an output and drives its line low. The frequency and duty made
// are the nearest the hardware takes, answered as a fraction (Hz) and the duty on the request's scale.
//
//   ESP32 (classic, P4)   LEDC through arduino-esp32 (ledcAttach / ledcWrite / ledcReadFreq): the resolution the most
//                         bits the 80 MHz clock leaves at that frequency (14 at most), the frequency as the driver reads it
//   RP2040 / RP2350       a PWM slice a pin: clk_sys / (divider x (wrap + 1)), the divider in 1/16 steps; two roles on
//                         one slice are refused at the plan (a slice has one frequency)
// OEP_HOST_FAKE_PWM: a host test's fake (the frequency as asked, the duty on 8 bits).
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

namespace oep {

class FixturePwm final : public Interface {
 public:
  static constexpr uint8_t kOwnerId = 10;
  static constexpr uint8_t kMaxChannels = 4;
  static constexpr uint32_t kMaxHz = 10000000;
  static constexpr uint32_t kMinHz = 10;
  enum : uint8_t { kOpSet = reg::fixture_pwm::kOpSet, kOpStatus = reg::fixture_pwm::kOpStatus };

  FixturePwm(PinTable &pins, uint16_t instance = 0) : pins_(pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_pwm::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_pwm::kRevision; }
  bool lockFree(uint8_t op) const override { return op == kOpStatus; }
  bool offers(uint8_t op) const override { return op == kOpSet || op == kOpStatus; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t *out, size_t capacity) override;
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  uint8_t planRefusalCause() const override { return refusal_cause_; }
  void planRefusalDetail(uint16_t &channel) const override { channel = refusal_channel_; }
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;

 private:
  struct Out { int channel = -1; uint32_t num = 0, den = 1; uint16_t duty = 0; bool running = false; };
  PinTable &pins_;
  uint16_t instance_;
  uint8_t refusal_cause_ = reg::core::kUnavailableCauseWrongState;
  uint16_t refusal_channel_ = 0xffff;
  Out out_[kMaxChannels];
  void stop(Out &o);
  size_t entry(uint8_t role, uint8_t *at) const;   // role num den duty: 11 bytes
};

}  // namespace oep
