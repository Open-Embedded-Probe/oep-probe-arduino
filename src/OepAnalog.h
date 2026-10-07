// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.analog revision 1 (oep-spec docs/oep-if-capture.ja.md §1.2, §3.3, §3.8): one ADC's channels in turn,
// one-shot, immediate or a threshold crossed up / down on one channel, with a pretrigger. Raw values only: the answer gives the probe's nominal 1st-order scale, the reference and the
// frontend (attenuation) of each channel; calibration gives the factory data as read.
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
//   0x01 configure(TLV) -> TLV   0x02 start -> blocking_ms u32, generation u32   0x03 stop   0x04 force   0x05 status
//   0x06 read(generation, position, max)   0x07 segments   0x08 release (nothing to do in one-shot)   0x09 query (no lock)
//   0x0A calibration -> TLV (no lock)
//
// Channel k is plan role k (0..3); a pin is an ADC input the sketch offers. The pads go to their analog function while
// it runs, which cuts their digital input and output (the classic ESP32: GPIO32 as logic and analog at once gave 0
// edges against 129 alone, 2026-09-30), so a planned channel is shared with nothing (oep-if-capture §1.2): no other
// fn's plan (planShares: the endpoint refuses the overlap, a logic capture's too), and with setPins no wire connection
// or setting either (it is claimed in the pin table).
#pragma once
#include <Arduino.h>

#include "Oep.h"
#include "OepCaptureGroup.h"
#include "OepPinTable.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_adc/adc_continuous.h>
#endif
// The RP2's build (its ADC round robin and DMA); OEP_HOST_FAKE_RP2_ADC: a host test with fakes of them (tests/host/shim)
#if defined(ARDUINO_ARCH_RP2040) || (defined(OEP_HOST_FAKE_RP2_ADC) && !defined(ARDUINO_ARCH_ESP32))
#define OEP_ANALOG_RP2 1
#endif

namespace oep {

class Endpoint;

// The answer's scale (oep-if-capture §1.2 rule 4, §3.3 tag 0x55) for a frontend whose values 0 .. full span
// min_mv .. max_mv at the input pin: voltage = (value - zero) x scale_nv, so zero is the value that would read 0 V -
// below 0 for a range that starts above 0 V (the classic ESP32's ADC reads 0 up to about 150 mV at 12 dB), rounded
// to the nearest value; scale_nv rounded to the nearest nV.
inline void analogScale(int32_t min_mv, int32_t max_mv, uint32_t full, int32_t &zero, int32_t &scale_nv) {
  const int64_t span_nv = (static_cast<int64_t>(max_mv) - min_mv) * 1000000;
  scale_nv = static_cast<int32_t>((span_nv + full / 2) / full);
  const int64_t num = -static_cast<int64_t>(min_mv) * full, den = static_cast<int64_t>(max_mv) - min_mv;
  zero = static_cast<int32_t>(num >= 0 ? (num + den / 2) / den : -((-num + den / 2) / den));
}

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
  // every op of capture §3.2, calibration included (query and force optional, offered: the ops tag declares them)
  bool offers(uint8_t op) const override { return opIn(op, reg::fixture_analog::kOpConfigure, reg::fixture_analog::kOpCalibration); }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  uint8_t planRefusalCause() const override { return refusal_cause_; }   // capturing: 6; the roles' count: 2; idle: 5
  void planRefusalDetail(uint16_t &channel) const override { channel = refusal_channel_; }
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  uint16_t boundTo() const override { return groupFn(); }   // bound: plan changes refused (capture §4.1)
  // read returns at most this much (capture §3.2: the probe's choice, within max and max_frame)
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 16 ? max_frame - 16 : 0; }
  bool notifies() const override { return true; }   // subscribe / unsubscribe in its ops (core §11.3)
  bool subscribe(bool on) override { subscribed_ = on; return true; }
  void poll();   // from loop(): collects the conversions, a finished capture becomes the segment and stopped events
  bool planShares() const override { return false; }
  void setPins(PinTable *pins, uint8_t owner) { table_ = pins; owner_ = owner; }

  // GroupTrack (oep.fixture.capture-group)
  bool trackReady() const override;
  bool trackCanStart() const override { return (trackReady() || state_ == reg::fixture_analog::kStateError) && buffer_; }
  uint8_t trackMode() const override { return reg::fixture_analog::kModeOneShot; }
  bool trackTriggered() const override { return trig_type_ != 0; }
  bool trackCanFollow() const override;
  bool trackStartFollowing() override;
  void trackTriggerAt(uint64_t ns) override;
  bool trackTriggerNs(uint64_t &ns) const override;
  bool trackArmed() const override { return got_ >= pre(); }
  bool trackRate(uint32_t &num, uint32_t &den) const override { num = rate_num_; den = rate_den_; return rate_num_ != 0; }
  uint32_t trackPretrigger() const override { return pretrigger_; }
  bool trackCanKeep(uint32_t p) const override;   // a follower's P_k within its pretrigger limits (capture §4.1)
  void trackKeep(uint32_t p) override { follow_pre_ = p; }
  void trackForce() override { if (state_ == reg::fixture_analog::kStateWaiting) force_ = true; }
  uint32_t trackLoad() const override { return total_hz_; }
  bool trackStart() override { follow_ = false; return startNow(); }
  void trackStop() override { stopNow(); }
  uint8_t trackState() const override { return state_; }
  uint8_t trackError() const override { return error_; }
  uint32_t trackGeneration() const override { return generation_; }

 private:
  static uint32_t maxPretrigger();
  uint8_t error_ = reg::fixture_analog::kErrorPeripheral;   // status's error TLV in state 6
  bool lost_ = false;          // the segment had a hole and was not handed out (capture §2.2)
  bool hole() const;           // the segment just finished has values lost or overwritten inside it
  void failHole();             // not handed out: state 6, stopped reason 3, error 2
  void fail(uint8_t error);    // state 6 with stopped reason 3 and `error` (capture §3.2)
  Endpoint &endpoint_;
  uint32_t generation_ = 0;   // one up at every start (nextGeneration); 0 before the first
  uint64_t adc_pins_;
  uint16_t instance_;
  int pins_[kMaxChannels] = {-1, -1, -1, -1};   // by role
  uint8_t channels_ = 0;
  PinTable *table_ = nullptr;
  uint8_t owner_ = 0;
  uint8_t frontend_[kMaxChannels] = {};           // by role
  uint8_t order_[kMaxChannels] = {0, 1, 2, 3};    // frame slot m -> role
  size_t max_read_ = 1000;
  bool subscribed_ = false, reported_ = true;
  uint8_t state_ = reg::fixture_analog::kStateUnconfigured;
  uint8_t refusal_cause_ = reg::core::kUnavailableCauseWrongState;   // planRefusalCause
  uint16_t refusal_channel_ = 0xFFFF;                                  // planRefusalDetail
  uint32_t total_hz_ = 0, rate_num_ = 0, rate_den_ = 1;   // the conversions a second; the rate a channel = num / den
  uint32_t samples_ = 0, frames_ = 0;                     // asked; complete frames captured
  uint16_t *buffer_ = nullptr;                            // samples_ x channels_ values, frame after frame
  uint64_t start_ns_ = 0;                                 // the first frame's time on the probe's clock
  uint32_t uncertainty_ns_ = 0;
  bool short_ = false;                                    // stopped before all the samples came
  bool segment_ = false;                                  // this generation's segment is kept (complete, or cut by stop)
  // the trigger (configure) and the search (start .. the crossing): frame f's value of frame slot m is at
  // ring[(f x channels + m) % ring_len_]
  uint8_t trig_type_ = 0, trig_slot_ = 0;
  uint32_t trig_value_ = 0;
  uint32_t pretrigger_ = 0, arm_ = 0;
  uint32_t follow_pre_ = 0;   // following a group's trigger: the group's pretrigger here (P_k, capture §4.1)
  uint32_t pre() const { return follow_ ? follow_pre_ : pretrigger_; }
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
  void finishTriggered(bool cut);
  Result configure(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity, bool query);
  Result calibration(uint8_t *out, size_t capacity) const;
  bool startNow();
  void stopNow();
  void finish();
  void forget();   // the plan released or replaced: no configuration, data or segment
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
#elif defined(OEP_ANALOG_RP2)
  int dma_ = -1;
  uint32_t cycles_ = 96;                     // 48 MHz ADC clock cycles a conversion
  static constexpr uint32_t kRingBytes = 32768, kRingCount = 0x0FFFFFFF;   // the write ring; transfers a run
  uint16_t *ring_ = nullptr;                 // triggered: the DMA ring (aligned to its size), kept once allocated
  void armDma(uint16_t *to, uint32_t count, bool ring);
#endif
};

}  // namespace oep
