// OEP v1 draft logic capture, the basic set of oep-spec docs/logic-capture.ja.md (§3.0, §4, §5), on the ESP32-P4
// PARLIO RX. This implementation picks: one-shot only, immediate trigger only, pushes events when subscribed.
//
//   0x01 configure(TLV)  -> TLV     0x02 start -> blocking_ms u32     0x03 stop     0x05 status     0x06 read
//   0x07 segments(from u32)   0x09 query(TLV) -> TLV, no lock           (0x04 force, 0x08 release: not in one-shot)
//
// Channel k is plan role k (0..15), taken in role order. Unused bits of a sample are left as captured (undefined).
#pragma once

#include <Arduino.h>

#include "OepV1.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)
#include <driver/parlio_rx.h>

namespace oep {
namespace v1 {

class Endpoint;

class LogicCapture final : public Interface {
 public:
  enum : uint8_t { kOpConfigure = 0x01, kOpStart = 0x02, kOpStop = 0x03, kOpForce = 0x04, kOpStatus = 0x05,
                   kOpRead = 0x06, kOpSegments = 0x07, kOpRelease = 0x08, kOpQuery = 0x09 };
  enum : uint8_t { kStateUnconfigured = 0, kStateConfigured = 1, kStateWaiting = 2, kStateCapturing = 3,
                   kStateDone = 4, kStatePaused = 5, kStateError = 6 };
  static constexpr uint8_t kMaxChannels = 16;
  static constexpr size_t kSegmentBytes = 65408;   // 511 cache lines, inside the driver's 65535-byte frame
  static constexpr uint32_t kSourceHz = 160000000, kMinHz = 627451;   // PLL_F160M / 255 (256 silently fails)

  LogicCapture(Endpoint &endpoint, uint64_t reserved_pins, uint16_t instance = 0)
      : endpoint_(endpoint), reserved_(reserved_pins), instance_(instance) {}
  const char *name() const override { return "oep.fixture.capture"; }
  uint16_t instance() const override { return instance_; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override {
    return op == kOpStatus || op == kOpRead || op == kOpSegments || op == kOpQuery;
  }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 16 ? max_frame - 16 : 0; }
  bool subscribe(bool on) override { subscribed_ = on; return true; }
  void poll();   // from loop(): turns a finished capture into events

 private:
  Endpoint &endpoint_;
  uint64_t reserved_;
  uint16_t instance_;
  int pins_[kMaxChannels];
  uint8_t channels_ = 0;
  size_t max_read_ = 1000;
  bool subscribed_ = false;
  // configured
  uint8_t state_ = kStateUnconfigured;
  uint8_t width_ = 1;
  uint32_t samples_ = 0, bytes_ = 0, rate_num_ = 0, rate_den_ = 1;
  // running
  parlio_rx_unit_handle_t unit_ = nullptr;
  parlio_rx_delimiter_handle_t delimiter_ = nullptr;
  uint8_t *buffer_ = nullptr;
  volatile bool done_ = false;
  bool reported_ = false;
  uint32_t start_us_ = 0;

  Result configure(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity, bool query);
  bool open(uint32_t rate_hz, uint8_t width, size_t bytes, uint32_t &num, uint32_t &den);
  void close();
  size_t segmentInfo(uint8_t *out) const;
  static bool receiveDone(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context);
};

}  // namespace v1
}  // namespace oep

#endif
