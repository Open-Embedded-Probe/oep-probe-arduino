// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 logic capture (oep.fixture.logic revision 1, oep-spec logic-capture.ja.md §5) on the classic ESP32: the
// software GPIO sampler of the v0 FixtureCapture, one-shot only. Core 0 does nothing else on this probe, so it samples
// with interrupts off, paced by the cycle counter (one register read per sample); OEP keeps answering on core 1.
// A sample is one byte (w = 8), channel k on bit k (logic-capture §3.0), up to 8 channels; bits of unused channels are 0.
// Triggers: immediate, level and edge on one channel, with a pretrigger inside the segment; the search runs in bursts
// with interrupts on between them (see run()).
//
//   0x01 configure(TLV) -> TLV   0x02 start -> blocking_ms u32, generation u32   0x03 stop   0x04 force   0x05 status
//   0x06 read(generation, position, max)   0x07 segments   0x08 release (nothing to do in one-shot)   0x09 query (no lock)
#pragma once

#include <Arduino.h>

#include "Oep.h"
#include "OepCaptureGroup.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace oep {

class Endpoint;

class SamplerCapture final : public Interface, public GroupTrack {
 public:
  static constexpr uint8_t kMaxChannels = 8;
  static constexpr size_t kBufferBytes = 65408;
  // one sample per 120 cycles at 240 MHz is the tested ceiling; the floor keeps a full window under the 300 ms
  // interrupt watchdog while interrupts are off on the sampling core
  static constexpr uint32_t kMaxHz = 2000000, kMinHz = 400000;
  static constexpr uint32_t kMaxHzHighBank = 1000000;   // with any channel on GPIO32..39

  SamplerCapture(Endpoint &endpoint, uint64_t reserved_pins, uint16_t instance = 0)
      : endpoint_(endpoint), reserved_(reserved_pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_logic::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_logic::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::fixture_logic::kLockFreeOps, op); }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 16 ? max_frame - 16 : 0; }
  bool subscribe(bool on) override { subscribed_ = on; return true; }
  void poll();   // from loop(): a finished window becomes the segment and stopped events
  // GroupTrack (oep.fixture.capture-group): the group drives start / stop through handle(); bound, the host cannot
  bool trackReady() const override { return state_ == reg::fixture_logic::kStateConfigured || state_ == reg::fixture_logic::kStateDone; }
  uint8_t trackMode() const override { return reg::fixture_logic::kModeOneShot; }
  bool trackTriggered() const override { return trig_type_ != 0; }
  // the trigger track of a group (it cannot follow one: its search runs in bursts, with gaps)
  bool trackTriggerNs(uint64_t &ns) const override {
    if (!trig_type_ || !trig_seen_) return false;
    ns = trig_burst_ns_ + static_cast<uint64_t>(trig_count_) * cycles_ * 1000000000ull / cpu_hz_;
    return true;
  }
  void trackForce() override { groupOp(reg::fixture_logic::kOpForce); }
  uint32_t trackLoad() const override { return cycles_ ? static_cast<uint32_t>(static_cast<uint64_t>(channels_) * cpu_hz_ / cycles_) : 0; }
  bool trackStart() override { return groupOp(reg::fixture_logic::kOpStart); }
  void trackStop() override { groupOp(reg::fixture_logic::kOpStop); }
  uint8_t trackState() const override { return state_; }
  uint32_t trackGeneration() const override { return generation_; }

 private:
  bool group_op_ = false;
  uint32_t generation_ = 0;   // one up at every start (oep-if-capture: generation)
  bool groupOp(uint8_t op) {
    uint8_t out[8];
    group_op_ = true;
    const Result r = handle(op, nullptr, 0, out, sizeof out);
    group_op_ = false;
    return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess;
  }
  Endpoint &endpoint_;
  uint64_t reserved_;
  uint16_t instance_;
  int pins_[kMaxChannels];
  uint8_t channels_ = 0;
  size_t max_read_ = 1000;
  bool subscribed_ = false;
  uint8_t state_ = reg::fixture_logic::kStateUnconfigured;
  uint32_t samples_ = 0, cycles_ = 0, cpu_hz_ = 0;
  uint64_t start_ns_ = 0;   // the probe's clock (esp_timer x 1000)
  uint8_t *buffer_ = nullptr;
  uint32_t masks0_[kMaxChannels] = {}, masks1_[kMaxChannels] = {};
  TaskHandle_t sampler_ = nullptr;
  volatile bool done_ = false, reported_ = true;
  volatile bool slipped_ = false;         // the last window had a sample more than one period late
  volatile uint32_t late_cycles_ = 0;     // the most it was behind, in CPU cycles
  // the trigger, as configured; what the search found (written by the sampler task, read by poll)
  static constexpr uint64_t kOffNs = 250000000;   // the longest a burst keeps interrupts off (watchdog: 300 ms)
  static constexpr uint8_t kControlForce = 1, kControlAbort = 2;
  uint8_t trig_type_ = 0, trig_role_ = 0;
  uint32_t trig_value_ = 0;
  uint32_t pretrigger_ = 0;
  volatile uint8_t control_ = 0;          // from core 1: force, abort the search
  volatile bool trig_seen_ = false, aborted_ = false;
  volatile uint32_t trig_count_ = 0;      // the trigger's sample in its burst
  volatile uint64_t trig_burst_ns_ = 0, seg_start_ns_ = 0;

  Result configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query);
  static void samplerTask(void *context);
  template <bool kHigh> void run();
  void runLow();
  void runHigh();
  void waitIdle();
  size_t segmentInfo(uint8_t *out) const;
};

}  // namespace oep

#endif
