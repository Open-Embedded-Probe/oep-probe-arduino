// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 logic capture (oep.fixture.logic revision 1, oep-spec logic-capture.ja.md §5) on an RP2040 / RP2350: one-shot,
// and repeat with rearm 1 (capture §2.1: every segment its own trigger, the state machine started again for each once the
// last is packed into its slot; up to kMaxSegments slots, a full set waits for release in state 5), paced by a PIO state
// machine and copied by DMA, so the CPU only sets it up and packs each segment afterwards. Repeat with rearm 0 (gapless
// segments) is refused unsupported: the state machine stops between segments.
//
// The state machine reads all 32 GPIO inputs a sample (`in pins, 32`, autopush) into a DMA ring of `samples` words (a
// second channel puts the first back to the ring's start each round); every loop of its program takes one sample in
// three instructions, at clk_sys / (3 x a whole divider) - the rate answered as that fraction. The trigger is the
// program's: the pretrigger's samples taken first (arm 1: capture §3.3), then `jmp pin` on the trigger line - an edge
// (rising, falling, either: the first change after arming) or a level (when 0: after a sample not at it since arming,
// the edge into it; when 1: one already at it too) - then the samples after it, an IRQ flag, and a halt. Once halted the ring is turned
// to start at the segment's first sample and packed to one byte a sample (channel k on bit k, up to 8 channels, any of
// the probe's GPIOs: the capture only listens, and claims none). A sample is late only if the state machine stalled
// on a full FIFO (the DMA behind): the segment is then slipped.
//
//   0x01 configure(TLV) -> TLV   0x02 start -> generation u32   0x03 stop   0x04 force   0x05 status
//   0x06 read(generation, position, max)   0x07 segments   0x08 release (nothing to do in one-shot)   0x09 query (no lock)
#pragma once

#include <Arduino.h>

#include "Oep.h"
#include "OepCaptureGroup.h"
#include "OepPinTable.h"

#if defined(ARDUINO_ARCH_RP2040)

#include <hardware/pio.h>

namespace oep {

class Endpoint;

class Rp2LogicCapture final : public Interface, public GroupTrack {
 public:
  static constexpr uint8_t kMaxChannels = 8;
  static constexpr uint8_t kTriggerOpts = kTriggerOptsWhen1 | kTriggerOptsRearm;   // arm 1 only; when 0 / 1; rearm 1
  static constexpr uint32_t kMaxSegments = 16;      // repeat's slots
  static constexpr uint16_t kTriggerRecords = 32;   // describe trigger_records: rearm 1's records kept (capture §3.10)
  // The most samples a segment holds: its ring (4 bytes a sample) and the packed segment (1) from the heap.
#if defined(PICO_RP2350)
  static constexpr uint32_t kMaxSamples = 65536;
#else
  static constexpr uint32_t kMaxSamples = 24576;
#endif
  static constexpr uint32_t kCyclesPerSample = 3;   // the program's every loop
  static constexpr uint32_t kMinDivider = 2;        // 1 / 6 of clk_sys at most: the DMA keeps up with room to spare
  static constexpr uint32_t kMinHz = 1000;

  Rp2LogicCapture(Endpoint &endpoint, const PinTable &pins, uint16_t instance = 0)
      : endpoint_(endpoint), table_(pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_logic::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_logic::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::fixture_logic::kLockFreeOps, op); }
  bool offers(uint8_t op) const override {
    return opIn(op, reg::fixture_logic::kOpConfigure, reg::fixture_logic::kOpQuery) || op == reg::fixture_logic::kOpTriggers;
  }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  uint8_t planRefusalCause() const override { return plan_cause_; }
  uint8_t plan_cause_ = reg::core::kUnavailableCauseLimit;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 16 ? max_frame - 16 : 0; }
  void setFrameLimits(size_t, size_t largest) override { setFrameLimit(largest); }
  bool notifies() const override { return true; }
  bool subscribe(bool on) override { subscribed_ = on; return true; }
  void poll();   // from loop(): the trigger seen, a finished capture becomes the segment and its events
  uint16_t boundTo() const override { return groupFn(); }   // bound: plan changes refused (capture §4.1)
  // GroupTrack (oep.fixture.capture-group): the group starts and stops it; a triggered one is the group's trigger
  // track (its trigger's time is known once the samples after it are in: their time plus a loop round late)
  bool trackReady() const override { return state_ == reg::fixture_logic::kStateConfigured || state_ == reg::fixture_logic::kStateDone; }
  bool trackCanStart() const override { return (trackReady() || state_ == reg::fixture_logic::kStateError) && ring_; }
  uint8_t trackMode() const override { return mode_; }
  bool trackTriggered() const override { return trig_type_ != 0; }
  bool trackTriggerNs(uint64_t &ns) const override;
  uint64_t trackLatencyNs() const override;
  void trackForce() override { groupOp(reg::fixture_logic::kOpForce); }
  uint32_t trackLoad() const override { return divider_ ? channels_ * (clk_hz_ / (kCyclesPerSample * divider_)) : 0; }
  bool trackStart() override { return groupOp(reg::fixture_logic::kOpStart); }
  void trackStop() override { groupOp(reg::fixture_logic::kOpStop); }
  uint8_t trackState() const override { return state_; }
  uint32_t trackGeneration() const override { return generation_; }
  bool trackRate(uint32_t &num, uint32_t &den) const override {
    num = clk_hz_;
    den = kCyclesPerSample * divider_;
    return divider_ != 0;
  }
  uint32_t trackPretrigger() const override { return pretrigger_; }

 private:
  bool group_op_ = false;
  bool groupOp(uint8_t op) {
    uint8_t out[8];
    group_op_ = true;
    const Result r = handle(op, nullptr, 0, out, sizeof out);
    group_op_ = false;
    return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess;
  }
  Endpoint &endpoint_;
  const PinTable &table_;
  uint16_t instance_;
  int pins_[kMaxChannels] = {};
  uint8_t channels_ = 0;
  size_t max_read_ = 1000;
  bool subscribed_ = false;
  uint8_t state_ = reg::fixture_logic::kStateUnconfigured;
  uint32_t generation_ = 0;
  uint32_t samples_ = 0, divider_ = 0, clk_hz_ = 0;
  uint8_t trig_type_ = 0, trig_role_ = 0;
  uint32_t trig_value_ = 0, pretrigger_ = 0;
  uint8_t when_ = 0;   // trigger_opts' when (capture §3.3)
  uint8_t mode_ = reg::fixture_logic::kModeOneShot;
  uint32_t seg_count_ = 1;   // the slots: 1 in one-shot
  uint32_t *ring_ = nullptr;   // samples_ words
  uint8_t *segment_ = nullptr;   // seg_count_ x samples_ bytes, packed: segment serial s in slot s % seg_count_
  uint32_t capacity_ = 0;   // the samples ring_ holds
  uint32_t seg_capacity_ = 0;   // the bytes segment_ holds
  uint64_t start_ns_ = 0, seg_start_ns_ = 0;
  bool slipped_ = false, trig_seen_ = false;
  // repeat: the segments finished and released since the start (u32 serials, core §2.6), each slot's info
  uint32_t completed_ = 0, released_ = 0;
  struct SlotInfo { uint64_t start_ns; bool slipped; };
  SlotInfo slots_[kMaxSegments] = {};
  bool rep_wait_ = false;   // between segments, waiting for the next one's trigger (status flags bit2)
  struct TriggerRecord { uint64_t sample; uint64_t ns; uint32_t serial; uint32_t uncertainty; };
  TriggerRecord records_[kTriggerRecords] = {};
  uint32_t record_next_ = 0;
  bool repeat() const { return mode_ == reg::fixture_logic::kModeRepeat; }
  bool rearm() { return completed_ - released_ < seg_count_ && begin(); }   // the next segment's trigger, when a slot is free
  void segmentDone();   // a segment in: packed, its info, then the next armed (repeat) or done
  // the PIO side (claimed at the first start, kept)
  PIO pio_ = nullptr;
  int sm_ = -1, offset_ = -1, dma_data_ = -1, dma_ctrl_ = -1;
  uint32_t *ring_base_ = nullptr;   // the control channel's source: the ring's start

  Result configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query);
  bool claim();
  bool begin();      // the state machine and the DMA running from the program's entry for the trigger
  void halt();       // both stopped (nothing kept)
  bool finished() const;
  bool inPost() const;   // the trigger seen: the program is taking the samples after it
  void collect();    // the halted ring turned and packed into segment_
  size_t segmentInfo(uint8_t *out, uint32_t serial) const;
  void stopped(uint8_t reason, uint8_t error = 0);
};

}  // namespace oep

#endif
