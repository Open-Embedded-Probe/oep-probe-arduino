// fixture.capture (owner 0, id 0x22): sampled logic capture, read-only observer.
// ESP32-P4 backend: PARLIO RX, one finite transaction (soft delimiter, <= 65535
// bytes, E016/E018), internal DMA RAM. Other architectures get a stub that
// reports Unavailable so the same example compiles everywhere.
#pragma once

#include <Arduino.h>

#include "OepFixtureServices.h"
#include "OepService.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)
#include <driver/parlio_rx.h>
#define OEP_CAPTURE_PARLIO 1
#endif

namespace oep {

class FixtureCapture final : public Service {
 public:
  static constexpr uint8_t kMaxLines = 8;
  static constexpr size_t kBufferBytes = 65408;  // 511 x 128-byte cache lines, below the 65535 delimiter limit
  static constexpr uint32_t kMaxSampleRateHz = 20000000;  // PARLIO ran 8 lines at 80 MHz (E022); 20 MHz keeps 3.2 ms in the 64 KiB buffer at 1 line
  explicit FixtureCapture(PinTable &pins) : pins_(pins) {}
  uint16_t owner() const override { return OEP_V0_DEF_FIXTURE_CAPTURE_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_FIXTURE_CAPTURE_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_FIXTURE_CAPTURE_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t first, uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;

 private:
  PinTable &pins_;
  int lines_[kMaxLines] = {-1, -1, -1, -1, -1, -1, -1, -1};
  uint8_t line_count_ = 0;
  uint8_t width_ = 0;         // PARLIO data width in bits (1, 2, 4, 8)
  uint32_t sample_rate_ = 0;
  uint32_t samples_ = 0;
  size_t bytes_ = 0;
  bool configured_ = false, armed_ = false, error_ = false;
  volatile bool done_ = false;
  uint8_t *buffer_ = nullptr;
#if OEP_CAPTURE_PARLIO
  parlio_rx_unit_handle_t unit_ = nullptr;
  parlio_rx_delimiter_handle_t delimiter_ = nullptr;
  static bool receiveDone(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context);
#endif
  bool setup(uint32_t sample_rate_hz, uint32_t samples);
  bool arm();
  void teardown();
  uint8_t sampleAt(uint32_t index) const;
};

}  // namespace oep
