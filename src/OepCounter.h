// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.counter revision 1 (oep-spec oep-if-fixture §8): the edges on each input within a gate.
//   0x10 measure(gate_us u32, edge u8) -> window_ns(u64) n(u8) n x (role u8, count u32)
// Role k is input k (0 .. kMaxChannels - 1). The counter only listens: it shares its pins with any other function (an
// output too: a PWM's line counted on itself), takes none in the pin table and changes no pin's drive - an ESP32's /
// RP2's pad has its input turned on, nothing else.
//
// An ESP32 with a pulse counter (PCNT: classic ESP32, P4 / P4X, S2 / S3 / C6 / H2) counts in it, a unit an input: the
// pad's input routed to the unit through the GPIO matrix, its 16-bit count carried into an accumulator at a watch
// point. kMaxHz is what was counted without a miss on the bench (a square wave on the same pad). Other boards take a GPIO
// interrupt an edge (attachInterrupt), so the rate counted is the interrupt's (kMaxHz, a cautious figure; faster edges
// may be missed). measure blocks for the gate (inside max_op_ms: kMaxGateUs); the window answered is the probe's clock
// from the counting on to off. OEP_HOST_FAKE_COUNTER: a host test's fake (the test calls hostEdge).
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

#if defined(ARDUINO_ARCH_ESP32) && !defined(OEP_HOST_FAKE_COUNTER)
#include <soc/soc_caps.h>
#if SOC_PCNT_SUPPORTED
#define OEP_COUNTER_PCNT 1
#endif
#endif

namespace oep {

class FixtureCounter final : public Interface {
 public:
  static constexpr uint8_t kMaxChannels = 4;
#if defined(OEP_COUNTER_PCNT) && defined(CONFIG_IDF_TARGET_ESP32P4)
  static constexpr uint32_t kMaxHz = 40000000;   // the bench, P4X: LEDC on the same pad, 100 kHz .. 40 MHz, no miss
#elif defined(OEP_COUNTER_PCNT)
  static constexpr uint32_t kMaxHz = 10000000;   // the bench, classic ESP32: this probe's PWM on the same pad, up to its 10 MHz
#else
  static constexpr uint32_t kMaxHz = 100000;
#endif
  static constexpr uint32_t kMaxGateUs = 5000000;   // 5 s
  enum : uint8_t { kOpMeasure = reg::fixture_counter::kOpMeasure };

  FixtureCounter(const PinTable &pins, uint16_t instance = 0) : pins_(pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_counter::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_counter::kRevision; }
  bool offers(uint8_t op) const override { return op == kOpMeasure; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t *out, size_t capacity) override;
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;

#if defined(OEP_HOST_FAKE_COUNTER)
  static void hostEdge(uint8_t role);   // an edge the counter is set to count, on role's input
  static inline uint32_t host_gate_us = 0;   // the gate the last measure waited
#endif

 private:
  const PinTable &pins_;
  uint16_t instance_;
  int channel_[kMaxChannels] = {-1, -1, -1, -1};
};

}  // namespace oep
