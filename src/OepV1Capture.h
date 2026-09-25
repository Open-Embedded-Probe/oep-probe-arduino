// OEP v1 draft logic capture, the basic set of oep-spec docs/logic-capture.ja.md (§3.0, §4, §5), on the ESP32-P4
// PARLIO RX. This implementation picks: one-shot, repeat and streaming, immediate trigger only, pushes events when
// subscribed. Repeat (as wch-protocols E078): PARLIO fills a 64 KiB internal DMA ring by partial receive; the ISR
// queues each finished chunk; a harvest task on core 0 copies it into K segments in PSRAM before the ring comes round
// again. Streaming uses the same segments, sends them as data pushes (role 0x06) and reuses a segment once it is sent;
// with every segment unsent, new bytes are dropped and the stream position skips them (the host sees the jump).
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
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

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
  static constexpr size_t kRingBytes = 128 * 1024;                    // repeat / streaming: the DMA ring (internal)
  // repeat / streaming segments: in PSRAM when there is some (all of the largest free block but a reserve for the
  // rest of the firmware; boards come with 0, 16 or 32 MB), else a small store in internal RAM
  static constexpr size_t kPsramReserve = 1024 * 1024, kInternalReserve = 96 * 1024, kInternalStoreMax = 256 * 1024;
  static constexpr size_t kSegmentMin = 4096, kSegmentMaxRepeat = 1024 * 1024;
  static constexpr size_t kInfos = 256;                               // segment infos kept (>= segments in store)

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
  size_t pending() override;
  size_t pull(uint32_t &position, uint8_t *out, size_t capacity) override;
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
  uint32_t start_us_ = 0;

  Result configure(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity, bool query);
  bool open(uint32_t rate_hz, uint8_t width, size_t bytes, uint32_t &num, uint32_t &den);
  void close();
  size_t segmentInfo(uint8_t *out) const;
  size_t storeBudget(uint32_t &caps) const;   // bytes the segments may take now (counting the store already held)
  size_t store_bytes_ = 0;
  // repeat
  struct Chunk { const uint8_t *data; size_t length; };
  struct Info { uint32_t serial, position, samples, start_us; uint8_t flags; };
  uint8_t mode_ = 1;
  uint32_t segment_bytes_ = 0, segment_count_ = 0;
  uint8_t *ring_ = nullptr, *store_ = nullptr;
  QueueHandle_t queue_ = nullptr;
  TaskHandle_t task_ = nullptr;
  volatile bool harvesting_ = false;
  volatile uint32_t completed_ = 0, released_ = 0, fill_ = 0, queue_overflow_ = 0;
  volatile uint64_t captured_ = 0, dropped_ = 0;   // bytes the DMA delivered / bytes not stored (no free segment)
  volatile uint32_t produced_ = 0;                  // bytes the ISR has seen finished (the DMA write position, wraps)
  volatile uint32_t overruns_ = 0;                  // chunks the DMA rewrote before the harvest had copied them
  volatile bool gap_pending_ = false, paused_ = false;
  uint32_t reported_ = 0;
  // streaming: the next byte to push is sent_off_ into segment sent_seg_
  volatile uint32_t sent_seg_ = 0;
  uint32_t sent_off_ = 0;
  uint32_t segmentLength(uint32_t serial) const;
  bool findSegment(uint32_t position, uint32_t &serial, uint32_t &offset) const;
  bool paused_reported_ = false;
  Info infos_[kInfos];
  static bool partialReceive(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context);
  static void harvestTask(void *context);
  void harvest(const Chunk &chunk);
  void finishSegment(uint32_t bytes, uint8_t flags);
  bool openRepeat(uint32_t rate_hz, uint8_t width, uint32_t samples, uint32_t segments, uint32_t &num, uint32_t &den,
                  uint32_t &actual_samples, uint32_t &actual_segments);
  Result startRepeat(uint8_t *out, size_t capacity);
  void stopRepeat();
  size_t infoBytes(const Info &info, uint8_t *out) const;
  static bool receiveDone(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context);
};

}  // namespace v1
}  // namespace oep

#endif
