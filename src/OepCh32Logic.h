// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 logic capture (oep.fixture.logic revision 1, oep-spec oep-if-capture §3) on a WCH CH32 (not the X035), on
// ArduinoCore-CH32RV 0.4.0's CH32RVLogicCapture: one GPIO port's low 16 inputs sampled by a timer and DMA into two halves
// in turn, taken out by poll(). One-shot: immediate, or an edge / a level on one channel (looked for sample by sample as
// the halves come out, after the pretrigger's samples: arm 1) with a pretrigger. Up to 8 channels, all on one port's
// pins 0-15 (the core samples a whole port); packed one byte a sample (channel k on bit k).
//
// The CPU takes every sample out, so the rate is kept low (kMaxHz): a half the DMA comes back to before poll() took it
// is lost - the segment has a hole, not handed out (state 6, error 2). OEP_HOST_FAKE_CH32_LOGIC: a host test's fake of
// the core's class (tests/host/shim).
//
//   0x10 configure(TLV) -> TLV   0x11 start -> generation u32   0x12 stop   0x13 force   0x14 status
//   0x15 read(generation, position, max)   0x16 segments   0x17 release (nothing to do in one-shot)   0x18 query (no lock)
#pragma once

#include <Arduino.h>

#include "Oep.h"
#include "OepCaptureGroup.h"
#include "OepPinTable.h"

#if (defined(ARDUINO_ARCH_CH32RV) && !defined(ARDUINO_CH32X035)) || defined(OEP_HOST_FAKE_CH32_LOGIC)
#define OEP_LOGIC_CH32 1
#include <CH32RVLogicCapture.h>

namespace oep {

class Endpoint;

class Ch32LogicCapture final : public Interface {
 public:
  static constexpr uint8_t kMaxChannels = 8;
  static constexpr uint8_t kTriggerOpts = kTriggerOptsWhen1;   // arm 1 only; when 0 or 1
  static constexpr uint32_t kMaxSamples = 8192;   // a byte a sample from the heap
  static constexpr uint32_t kMaxHz = 250000;      // the CPU takes every sample out of the halves
  static constexpr uint32_t kMinHz = 1000;
  static constexpr uint32_t kHalf = 1024;         // samples a half: 4 ms at kMaxHz between two polls

  Ch32LogicCapture(Endpoint &endpoint, const PinTable &pins, uint16_t instance = 0)
      : endpoint_(endpoint), table_(pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_logic::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_logic::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::fixture_logic::kLockFreeOps, op); }
  bool offers(uint8_t op) const override { return opIn(op, reg::fixture_logic::kOpConfigure, reg::fixture_logic::kOpQuery); }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  uint8_t planRefusalCause() const override { return plan_cause_; }
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 16 ? max_frame - 16 : 0; }
  void setFrameLimits(size_t, size_t largest) override { setFrameLimit(largest); }
  bool notifies() const override { return true; }
  bool subscribe(bool on) override { subscribed_ = on; return true; }
  void poll();   // from loop(): the halves taken out, the trigger looked for, a finished capture its segment and events

#if defined(OEP_HOST_FAKE_CH32_LOGIC)
  CH32RVLogicCapture &hostLogic() { return logic_; }
#endif

 private:
  Endpoint &endpoint_;
  const PinTable &table_;
  uint16_t instance_;
  uint8_t plan_cause_ = reg::core::kUnavailableCauseLimit;
  int pins_[kMaxChannels] = {};
  uint8_t channels_ = 0;
  uint8_t port_ = 0;
  size_t max_read_ = 1000;
  bool subscribed_ = false;
  uint8_t state_ = reg::fixture_logic::kStateUnconfigured;
  uint8_t error_ = reg::fixture_logic::kErrorPeripheral;
  uint32_t generation_ = 0;
  uint32_t samples_ = 0, divider_ = 0;   // the rate: F_CPU / divider_
  uint8_t trig_type_ = 0, trig_role_ = 0;
  uint32_t trig_value_ = 0, pretrigger_ = 0;
  uint8_t when_ = 0;
  uint8_t *ring_ = nullptr;   // samples_ bytes, round while the trigger is looked for
  uint32_t capacity_ = 0;
  uint32_t got_ = 0;          // samples taken out since the start
  uint32_t trig_at_ = 0, end_ = 0;   // the trigger's sample; the segment's end (the stream's samples)
  bool triggered_ = false, armed_ = false, prev_ = false, have_prev_ = false, force_ = false, slipped_ = false;
  uint32_t overruns_ = 0;
  uint64_t start_ns_ = 0, seg_start_ns_ = 0;
  uint32_t rate_ = 0;         // what the core's timer gives
  CH32RVLogicCapture logic_;

  Result configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query);
  uint64_t periodNsOf() const { return rate_ ? 1000000000ull / rate_ : 0; }
  bool begin();
  void halt();
  void take(uint16_t sample);   // one sample out of a half: packed into the ring, the trigger looked for
  void finish();                // the segment turned to start at its first sample
  void fail(uint8_t error);
  size_t segmentInfo(uint8_t *out) const;
  void stopped(uint8_t reason, uint8_t error = 0);
};

}  // namespace oep

#endif
