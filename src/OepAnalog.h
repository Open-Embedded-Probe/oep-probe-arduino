// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.analog revision 1 (oep-spec docs/oep-if-capture.ja.md §1.2, §3.3, §3.8): one ADC's channels in turn,
// one-shot, immediate or a threshold crossed up / down on one channel, with a pretrigger. Raw values only: the answer gives the probe's nominal 1st-order scale, the reference, the
// frontend (attenuation) of each channel and how sure the rate is; calibration gives the factory data as read.
//
//   ESP32-P4 / classic ESP32   ADC1 in continuous (DMA) mode (esp_adc adc_continuous), a pattern entry per channel with
//                              its own attenuation. The P4's driver leaves out its first conversion frame: the first
//                              value is one frame after the start (logic-capture §7.4), and the time says so. Above
//                              46 kHz in all the P4 gives each value twice, so that is the ceiling declared.
//   RP2040 / RP2350            the ADC's round robin into its FIFO, copied by DMA: 48 MHz / 96 cycles a conversion at
//                              most (500 kS/s in all), one frontend (0 - ADC_VREF = the supply).
//
// A trigger keeps the ADC converting into a ring while it is looked for (ESP32: the segment's buffer, filled by the
// driver's reads; RP2: a 32 KiB DMA write ring) and takes the segment from pretrigger frames before the crossing.
//
//   0x01 configure(TLV) -> TLV   0x02 start -> blocking_ms u32   0x03 stop   0x04 force   0x05 status   0x06 read
//   0x07 segments   0x09 query(TLV) -> TLV (no lock)   0x0A calibration -> TLV (no lock)   (0x08 release: unknown)
//
// Channel k is plan role k (0..3); a pin is an ADC input the sketch offers. A capture only listens: its pins are not
// claimed in the pin table, but the pads go to their analog function while it runs.
#pragma once
#include <Arduino.h>

#include "Oep.h"
#include "OepCaptureGroup.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_adc/adc_continuous.h>
#endif

namespace oep {

class Endpoint;

class AnalogCapture final : public Interface, public GroupTrack {
 public:
  static constexpr uint8_t kMaxChannels = 4;
  static constexpr size_t kMaxBytes = 32768;   // one segment: samples x channels x 2
  AnalogCapture(Endpoint &endpoint, uint64_t adc_pins, uint16_t instance = 0)
      : endpoint_(endpoint), adc_pins_(adc_pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_analog::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_analog::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::fixture_analog::kLockFreeOps, op); }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 16 ? max_frame - 16 : 0; }
  bool subscribe(bool on) override { subscribed_ = on; return true; }
  void poll();   // from loop(): collects the conversions, a finished capture becomes the segment and stopped events

  // GroupTrack (oep.fixture.capture-group)
  bool trackReady() const override;
  uint8_t trackMode() const override { return reg::fixture_analog::kModeOneShot; }
  bool trackTriggered() const override { return trig_type_ != 0; }
  bool trackCanFollow() const override;
  bool trackStartFollowing() override;
  void trackTriggerAt(uint64_t ns) override;
  bool trackTriggerNs(uint64_t &ns) const override;
  void trackForce() override { if (state_ == reg::fixture_analog::kStateWaiting) force_ = true; }
  uint32_t trackLoad() const override { return total_hz_; }
  bool trackStart() override { follow_ = false; return startNow(); }
  void trackStop() override { stopNow(); }
  uint8_t trackState() const override { return state_; }

 private:
  Endpoint &endpoint_;
  uint64_t adc_pins_;
  uint16_t instance_;
  int pins_[kMaxChannels] = {-1, -1, -1, -1};   // by role
  uint8_t channels_ = 0;
  uint8_t frontend_[kMaxChannels] = {};           // by role
  uint8_t order_[kMaxChannels] = {0, 1, 2, 3};    // frame slot m -> role
  size_t max_read_ = 1000;
  bool subscribed_ = false, reported_ = true;
  uint8_t state_ = reg::fixture_analog::kStateUnconfigured;
  uint32_t total_hz_ = 0, rate_num_ = 0, rate_den_ = 1;   // the conversions a second; the rate a channel = num / den
  uint32_t samples_ = 0, frames_ = 0;                     // asked; complete frames captured
  uint16_t *buffer_ = nullptr;                            // samples_ x channels_ values, frame after frame
  uint64_t start_ns_ = 0;                                 // the first frame's time on the probe's clock
  uint32_t uncertainty_ns_ = 0;
  bool short_ = false;                                    // stopped before all the samples came
  // the trigger (configure) and the search (start .. the crossing): frame f's value of frame slot m is at
  // ring[(f x channels + m) % ring_len_]
  uint8_t trig_type_ = 0, trig_slot_ = 0;
  uint16_t trig_value_ = 0;
  uint32_t pretrigger_ = 0, arm_ = 0;
  uint32_t ring_len_ = 0;                                 // values
  uint32_t got_ = 0, searched_ = 0;                       // complete frames so far; frames looked at
  uint32_t trig_frame_ = 0, end_frame_ = 0;               // the crossing; the segment's end (the stream's frames)
  uint32_t seg_first_ = 0;                                // the segment's first frame
  bool follow_ = false, ext_ready_ = false;               // following a group's trigger, at frame ext_frame_
  uint32_t ext_frame_ = 0;
  bool ringMode() const { return trig_type_ != 0 || follow_; }
  uint8_t phase_ = 0;                                     // 0 searching, 1 filling after the crossing, 2 done
  uint16_t prev_ = 0;
  bool have_prev_ = false, force_ = false, trig_reported_ = true, trig_slipped_ = false;
  uint64_t seg_start_ns_ = 0;
  uint16_t *ring() const;
  uint64_t framesNs(uint64_t frames) const;               // frames x the frame period, in ns
  void search();
  void hitAt(uint32_t t);
  void pollTriggered();
  void finishTriggered();
  Result configure(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity, bool query);
  Result calibration(uint8_t *out, size_t capacity) const;
  bool startNow();
  void stopNow();
  void finish();
  size_t segmentInfo(uint8_t *out) const;
#if defined(ARDUINO_ARCH_ESP32)
  adc_continuous_handle_t handle_ = nullptr;
  volatile bool overflow_ = false;           // the driver's pool overflowed: conversions were lost (flags bit2)
  static bool IRAM_ATTR onOverflow(adc_continuous_handle_t, const adc_continuous_evt_data_t *, void *context);
  uint8_t adc_channel_[kMaxChannels] = {};   // by frame slot
  uint32_t counts_[kMaxChannels] = {};       // values stored, by frame slot
  bool overflow_seen_ = false;               // triggered: an overflow was seen ...
  uint32_t overflow_frame_ = 0;              // ... when this many frames had come (the loss was before)
  void drain();
#elif defined(ARDUINO_ARCH_RP2040)
  int dma_ = -1;
  uint32_t cycles_ = 96;                     // 48 MHz ADC clock cycles a conversion
  static constexpr uint32_t kRingBytes = 32768, kRingCount = 0x0FFFFFFF;   // the write ring; transfers a run
  uint16_t *ring_ = nullptr;                 // triggered: the DMA ring (aligned to its size), kept once allocated
  void armDma(uint16_t *to, uint32_t count, bool ring);
#endif
};

}  // namespace oep
