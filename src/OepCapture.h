// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.logic revision 1 (oep-spec docs/oep-if-capture.ja.md), on the ESP32-P4
// PARLIO RX. This implementation picks: one-shot, repeat and streaming, pushes events when subscribed. Triggers (one-shot
// only): level or edge on a channel, with a pretrigger - the samples go through the repeat's DMA ring and the harvest task
// looks for the condition as they come (a byte at a time, the channel's bits masked), then keeps the pretrigger from
// the ring and fills the segment. Repeat (as wch-protocols E078): PARLIO fills a 64 KiB internal DMA ring by partial receive; the ISR
// queues each finished chunk; a harvest task on core 0 copies it into K segments in PSRAM before the ring comes round
// again. Streaming uses the same segments, sends them as data pushes (role 0x06) and reuses a segment once it is sent;
// with every segment unsent, new bytes are dropped and the stream position skips them (the host sees the jump).
//
//   0x01 configure(TLV)  -> TLV     0x02 start -> blocking_ms u32     0x03 stop     0x05 status     0x06 read
//   0x07 segments(from u32)   0x08 release(serial u32) (repeat)   0x09 query(TLV) -> TLV, no lock   (0x04 force: no)
// Every request takes a TLV tail after its fixed part (core §2.3); configure / query answer unhandled non-critical
// TLVs in ignored (0x7F) and refuse unhandled critical ones (rejected unsupported, the tag).
//
// Channel k is plan role k (0..15), taken in role order. Unused bits of a sample are left as captured (undefined).
#pragma once

#include <Arduino.h>

#include "Oep.h"
#include "OepCaptureGroup.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)
#include <driver/parlio_rx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace oep {

class Endpoint;

class LogicCapture final : public Interface, public GroupTrack {
 public:
  enum : uint8_t {
    kOpConfigure = reg::fixture_logic::kOpConfigure, kOpStart = reg::fixture_logic::kOpStart,
    kOpStop = reg::fixture_logic::kOpStop, kOpForce = reg::fixture_logic::kOpForce,
    kOpStatus = reg::fixture_logic::kOpStatus, kOpRead = reg::fixture_logic::kOpRead,
    kOpSegments = reg::fixture_logic::kOpSegments, kOpRelease = reg::fixture_logic::kOpRelease,
    kOpQuery = reg::fixture_logic::kOpQuery,
  };
  enum : uint8_t {
    kStateUnconfigured = reg::fixture_logic::kStateUnconfigured, kStateConfigured = reg::fixture_logic::kStateConfigured,
    kStateWaiting = reg::fixture_logic::kStateWaiting, kStateCapturing = reg::fixture_logic::kStateCapturing,
    kStateDone = reg::fixture_logic::kStateDone, kStatePaused = reg::fixture_logic::kStatePaused,
    kStateError = reg::fixture_logic::kStateError,
  };
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
  size_t pending() override;
  size_t pull(uint8_t *out, size_t capacity) override;
  void poll();   // from loop(): turns a finished capture into events
  // GroupTrack (oep.fixture.capture-group): the group drives start / stop through handle(); bound, the host cannot
  bool trackReady() const override { return state_ == kStateConfigured || state_ == kStateDone; }
  uint8_t trackMode() const override { return mode_; }
  bool trackTriggered() const override { return trig_type_ != 0; }
  bool trackCanFollow() const override { return mode_ == reg::fixture_logic::kModeOneShot && trackReady(); }
  bool trackStartFollowing() override;
  void trackTriggerAt(uint64_t ns) override;
  bool trackTriggerNs(uint64_t &ns) const override;
  void trackForce() override { groupOp(kOpForce); }
  bool trackArmed() const override { return static_cast<uint64_t>(produced_) * 8 / (width_ ? width_ : 1) >= pretrigger_; }
  uint32_t trackLoad() const override { return rate_den_ ? static_cast<uint32_t>(static_cast<uint64_t>(channels_) * rate_num_ / rate_den_) : 0; }
  bool trackStart() override { return groupOp(kOpStart); }
  void trackStop() override { groupOp(kOpStop); }
  uint8_t trackState() const override { return state_; }

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
  uint64_t start_ns_ = 0;   // the probe's clock: ns since boot (esp_timer x 1000), u64 like the positions
  // PARLIO's first sample against the start's clock read: an edge landed within 5 us of the GPIO write that made it
  // (logic-capture §7.4), so +-5 us
  static constexpr uint32_t kStartUncertaintyNs = 5000;
  uint64_t nsOf(uint64_t samples) const;   // samples at the actual rate, in ns (no 128-bit arithmetic on the P4)

  Result configure(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity, bool query);
  bool open(uint32_t rate_hz, uint8_t width, size_t bytes, uint32_t &num, uint32_t &den);
  void close();
  size_t segmentInfo(uint8_t *out) const;
  size_t storeBudget(uint32_t &caps) const;   // bytes the segments may take now (counting the store already held)
  // streaming through the endpoint's zero-copy transport: the harvest copies straight from the DMA ring into stages
  // (internal RAM, one whole push frame each: length prefix, push header, data) and hands full ones to the transport
  static constexpr size_t kStagesMax = 8, kStageFrameMax = 27136, kPushHead = 2 + kPushHeader + 8;   // length, core push header, position(u64)
  bool direct_ = false;
  uint8_t *stage_[kStagesMax] = {};
  uint8_t stage_count_ = 0;
  volatile uint32_t stage_free_ = 0;   // bit per stage the transport has given back
  int stage_cur_ = -1;
  uint32_t stage_fill_ = 0, stage_since_ = 0, stage_data_ = 0;
  uint64_t stage_pos_ = 0;
  volatile uint32_t stage_drops_ = 0;  // bytes dropped because every stage was queued or in flight
  bool carry_ = false;                 // a byte held back from the last frame (so that it ended with a short packet)
  uint8_t carry_byte_ = 0;
  bool takeStage();
  bool openStages();
  void freeStages();
  void sendStage();
  static void stageDone(void *context, const uint8_t *buffer);
  size_t store_bytes_ = 0;
  // repeat
  struct Chunk { const uint8_t *data; size_t length; };
  // one-shot with a trigger (or a pretrigger): the ring and the harvest task, searching
  static constexpr size_t kPretriggerBytes = 64 * 1024;   // history the ring can give back (half of it)
  bool triggered_ = false;           // this one-shot goes through the ring
  uint8_t trig_type_ = 0, trig_role_ = 0;
  uint16_t trig_value_ = 0;
  uint32_t pretrigger_ = 0;
  volatile uint8_t trig_phase_ = 0;  // 0 waiting, 1 filling, 2 done
  volatile bool force_ = false;      // force: the trigger is now
  bool have_level_ = false;
  uint8_t last_level_ = 0;
  uint32_t filled_ = 0;              // bytes in buffer_
  uint64_t seg_first_sample_ = 0;
  uint32_t trigger_index_ = 0xFFFFFFFFu;
  volatile bool trig_overrun_ = false;   // the DMA came round while filling: the segment is not contiguous
  bool reported_trigger_ = true;
  bool follow_ = false;                      // started by a group to follow another track's trigger
  volatile bool ext_ready_ = false;          // ... whose sample is known (ext_sample_ written first)
  uint64_t ext_sample_ = 0;
  uint32_t rate_hz_ = 0;                     // as asked (a follower opens the ring at it)
  void harvestTriggered(const Chunk &chunk);
  bool findTrigger(const uint8_t *data, size_t length, uint64_t first_sample, uint64_t &at, uint64_t min_at);
  Result startTriggered(uint8_t *out, size_t capacity);
  bool openTriggered(uint32_t rate_hz, uint8_t width, uint32_t bytes, uint32_t &num, uint32_t &den);
  void pollTriggered();
  struct Info { uint32_t serial; uint64_t position; uint32_t samples; uint64_t start_ns; uint8_t flags; };
  // serial u32, position u64, samples u32, start_ns u64, start_uncertainty_ns u32, trigger_index u32, flags u8
  static constexpr size_t kInfoBytes = 33;
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
  bool findSegment(uint64_t position, uint32_t &serial, uint32_t &offset) const;
  bool paused_reported_ = false;
  Info infos_[kInfos];
  static bool partialReceive(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context);
  static void harvestTask(void *context);
  void harvest(const Chunk &chunk);
  void harvestDirect(const Chunk &chunk);
  void finishSegment(uint32_t bytes, uint8_t flags);
  bool openRepeat(uint32_t rate_hz, uint8_t width, uint32_t samples, uint32_t segments, uint32_t &num, uint32_t &den,
                  uint32_t &actual_samples, uint32_t &actual_segments);
  Result startRepeat(uint8_t *out, size_t capacity);
  void stopRepeat();
  size_t infoBytes(const Info &info, uint8_t *out) const;
  static bool receiveDone(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context);
};

}  // namespace oep

#endif
