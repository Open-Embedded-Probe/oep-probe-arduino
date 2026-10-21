// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 logic capture (oep.fixture.logic revision 1, oep-spec logic-capture.ja.md §5) on an RP2040 / RP2350: one-shot,
// paced by a PIO state machine and copied by DMA, so the CPU only sets it up and packs the segment afterwards.
//
// The state machine reads all 32 GPIO inputs a sample (`in pins, 32`, autopush) into a DMA ring of `samples` words (a
// second channel puts the first back to the ring's start each round); every loop of its program takes one sample in
// three instructions, at clk_sys / (3 x a whole divider) - the rate answered as that fraction. The trigger is the
// program's: the pretrigger's samples taken first (arm 1: capture §3.3), then `jmp pin` on the trigger line - an edge
// (rising, falling, either: the first change after arming) or a level as when 0 has it (after a sample not at it,
// since arming: the edge into it) - then the samples after it, an IRQ flag, and a halt. Once halted the ring is turned
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

class Rp2LogicCapture final : public Interface {
 public:
  static constexpr uint8_t kMaxChannels = 8;
  static constexpr uint8_t kTriggerOpts = 0;   // the defaults only: arm 1, when 0 (describe trigger opts)
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
  bool offers(uint8_t op) const override { return opIn(op, reg::fixture_logic::kOpConfigure, reg::fixture_logic::kOpQuery); }
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

 private:
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
  uint32_t *ring_ = nullptr;   // samples_ words
  uint8_t *segment_ = nullptr;   // samples_ bytes, packed
  uint32_t capacity_ = 0;   // what ring_ / segment_ hold
  uint64_t start_ns_ = 0, seg_start_ns_ = 0;
  bool slipped_ = false, trig_seen_ = false;
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
  size_t segmentInfo(uint8_t *out) const;
  void stopped(uint8_t reason, uint8_t error = 0);
};

}  // namespace oep

#endif
