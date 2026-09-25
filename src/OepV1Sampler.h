// OEP v1 logic capture (oep.fixture.capture revision 1, oep-spec logic-capture.ja.md §5) on the classic ESP32: the
// software GPIO sampler of the v0 FixtureCapture, one-shot only. Core 0 does nothing else on this probe, so it samples
// with interrupts off, paced by the cycle counter (one register read per sample); OEP keeps answering on core 1.
// A sample is one byte (w = 8), channel k on bit k (logic-capture §3.0), up to 8 channels; bits of unused channels are 0.
//
//   0x01 configure(TLV) -> TLV   0x02 start -> blocking_ms u32   0x03 stop   0x05 status   0x06 read   0x07 segments
//   0x09 query(TLV) -> TLV, no lock          (0x04 force, 0x08 release: not in one-shot, unknown operation)
#pragma once

#include <Arduino.h>

#include "OepV1.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace oep {
namespace v1 {

class Endpoint;

class SamplerCapture final : public Interface {
 public:
  static constexpr uint8_t kMaxChannels = 8;
  static constexpr size_t kBufferBytes = 65408;
  // one sample per 120 cycles at 240 MHz is the tested ceiling; the floor keeps a full window under the 300 ms
  // interrupt watchdog while interrupts are off on the sampling core
  static constexpr uint32_t kMaxHz = 2000000, kMinHz = 400000;

  SamplerCapture(Endpoint &endpoint, uint64_t reserved_pins, uint16_t instance = 0)
      : endpoint_(endpoint), reserved_(reserved_pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_capture::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_capture::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::fixture_capture::kLockFreeOps, op); }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 16 ? max_frame - 16 : 0; }
  bool subscribe(bool on) override { subscribed_ = on; return true; }
  void poll();   // from loop(): a finished window becomes the segment and stopped events

 private:
  Endpoint &endpoint_;
  uint64_t reserved_;
  uint16_t instance_;
  int pins_[kMaxChannels];
  uint8_t channels_ = 0;
  size_t max_read_ = 1000;
  bool subscribed_ = false;
  uint8_t state_ = reg::fixture_capture::kStateUnconfigured;
  uint32_t samples_ = 0, cycles_ = 0, cpu_hz_ = 0, start_us_ = 0;
  uint8_t *buffer_ = nullptr;
  uint32_t masks0_[kMaxChannels] = {}, masks1_[kMaxChannels] = {};
  TaskHandle_t sampler_ = nullptr;
  volatile bool done_ = false, reported_ = true;

  Result configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query);
  static void samplerTask(void *context);
  void waitIdle();
  size_t segmentInfo(uint8_t *out) const;
};

}  // namespace v1
}  // namespace oep

#endif
