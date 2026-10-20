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
//   0x01 configure(TLV) -> TLV   0x02 start -> generation u32   0x03 stop   0x04 force   0x05 status
//   0x06 read(generation, position, max u32) -> position flags len(u32) data   0x07 segments(from u32) -> more count ...
//   0x08 release(generation, serial) (repeat)   0x09 query(TLV) -> TLV, no lock
// Every start makes a new generation; read and release name it (another one is rejected unavailable cause 6), the
// segment records and the streaming data frames (TLV 0x01) carry it.
// Every request takes a TLV tail after its fixed part (core §2.3); configure / query refuse a value they cannot honour,
// critical or not, and an unknown critical tag (rejected unsupported, the tag as received).
//
// Channel k is plan role k (0..15), taken in role order. Unused bits of a sample are left as captured (undefined).
#pragma once

#include <Arduino.h>

#include "Oep.h"
#include "OepCaptureGroup.h"
#include "OepLogicBlocks.h"
#include "OepPinTable.h"

// OEP_HOST_FAKE_PARLIO: a host test with fakes of the PARLIO RX driver and the heap (tests/host/shim)
#if (defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)) || defined(OEP_HOST_FAKE_PARLIO)
#include <driver/parlio_rx.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace oep {

class Endpoint;

class LogicCapture final : public Interface, public GroupTrack {
 public:
  // A segment's start_uncertainty_ns (capture §2.2): PARLIO's first sample against the start's clock read - an edge
  // landed within 5 us of the GPIO write that made it (logic-capture §7.4), so +-5 us
  static constexpr uint32_t kStartUncertaintyNs = 5000;
  enum : uint8_t {
    kOpConfigure = reg::fixture_logic::kOpConfigure, kOpStart = reg::fixture_logic::kOpStart,
    kOpStop = reg::fixture_logic::kOpStop, kOpForce = reg::fixture_logic::kOpForce,
    kOpStatus = reg::fixture_logic::kOpStatus, kOpRead = reg::fixture_logic::kOpRead,
    kOpSegments = reg::fixture_logic::kOpSegments, kOpRelease = reg::fixture_logic::kOpRelease,
    kOpQuery = reg::fixture_logic::kOpQuery, kOpTriggers = reg::fixture_logic::kOpTriggers,
  };
  enum : uint8_t {
    kStateUnconfigured = reg::fixture_logic::kStateUnconfigured, kStateConfigured = reg::fixture_logic::kStateConfigured,
    kStateWaiting = reg::fixture_logic::kStateWaiting, kStateCapturing = reg::fixture_logic::kStateCapturing,
    kStateDone = reg::fixture_logic::kStateDone, kStatePaused = reg::fixture_logic::kStatePaused,
    kStateError = reg::fixture_logic::kStateError,
  };
  static constexpr uint8_t kMaxChannels = 16;
  // describe trigger opts: arm 0, when 1, and rearm 1 (a repeat's every segment its own trigger, capture §2.1)
  static constexpr uint8_t kTriggerOpts = kTriggerOptsArm0 | kTriggerOptsRearm | kTriggerOptsWhen1;
  static constexpr uint16_t kTriggerRecords = 64;   // describe trigger_records: rearm 1's records kept (capture §3.10)
  static constexpr size_t kSegmentBytes = 65408;   // 511 cache lines, inside the driver's 65535-byte frame
  // a triggered one-shot's segment, copied out of the ring: up to this (and the store's room) at this rate or below
  static constexpr size_t kTriggeredSegmentMax = 16 * 1024 * 1024;
  static constexpr uint32_t kTriggeredLongMaxBytes = 20000000;
  static constexpr uint32_t kSourceHz = 160000000, kMinHz = 627451;   // PLL_F160M / 255 (256 silently fails)
  // one-shot's ceiling by channel count (configure takes the nearest it allows): up to 8 lines 100 MHz, 16 lines 48 MHz
  static constexpr uint8_t kLimitLines8 = 8;
  static constexpr uint32_t kLimitHz8 = 100000000, kLimitHz16 = 48000000;
  // streaming over the zero-copy stages (formats 0 and 4): the PARLIO's DMA ring in PSRAM, the stream's buffer - the
  // harvest copies from it as the transport gives stages back, so the host's pauses up to its length cost nothing
  // (26 ms at 40 MB/s); without PSRAM the internal ring (kRingBytes). A power of 2; 2 MiB panicked in the driver's
  // receive (parlio_rx_unit_receive, the P4X, ESP32 core 3.3.12), 1 MiB runs. Its DMA descriptors take 17 KB of
  // internal RAM
  static constexpr size_t kStreamRingBytes = 1024 * 1024;
  static constexpr size_t kSpillBytes = 8u * 1024 * 1024;   // the spill behind it (below: spill_)
  static constexpr uint32_t kStreamMaxBytes = 40000000;   // streaming: what the harvest and the port keep up with (bytes a second)
  // format 1 streaming: the samples' bytes a second the encoding keeps up with, the worst case counted (every events
  // line changing every sample); 13.3 MHz at w 8 until measured further
  static constexpr uint32_t kStreamBlocksMaxBytes = 13400000;
  // streaming the copied way (the store, pushed through the endpoint's loop): what it keeps up with - the P4 v1.3 (no
  // internal RAM for the stages) kept 8 lines at 8.42 MHz w 8 with no loss and lost 11 % at 16 MHz (wireskein,
  // 2026-10-21); the nearest whole divider at or below 8.5 MB/s
  static constexpr uint32_t kStreamCopiedMaxBytes = 8500000;
  // repeat / streaming: the DMA ring (internal). 64 KiB: at 40 MB/s 1.6 ms of the harvest's lag, and the internal RAM it
  // leaves goes to the zero-copy stages (27 KiB each)
  static constexpr size_t kRingBytes = 64 * 1024;
  // repeat / streaming segments: in PSRAM when there is some (all of the largest free block but a reserve for the
  // rest of the firmware; boards come with 0, 16 or 32 MB), else a small store in internal RAM
  static constexpr size_t kPsramReserve = 1024 * 1024, kInternalReserve = 96 * 1024, kInternalStoreMax = 256 * 1024;
  static constexpr size_t kSegmentMin = 4096, kSegmentMaxRepeat = 1024 * 1024;
  static constexpr size_t kInfos = 256;                               // segment infos kept (>= segments in store)

  // pins: the probe's channels (any allowed one may be a line: the capture only listens, and claims none)
  LogicCapture(Endpoint &endpoint, const PinTable &pins, uint16_t instance = 0)
      : endpoint_(endpoint), table_(pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_logic::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_logic::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::fixture_logic::kLockFreeOps, op); }
  // every op of capture §3.2 (query and force optional, in the ops tag; calibration is analog's)
  // and triggers (rearm 1's records, §3.10: required with the opts' rearm bit)
  bool offers(uint8_t op) const override { return opIn(op, kOpConfigure, kOpQuery) || op == kOpTriggers; }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  // its unavailable plans are a role twice, more roles than channels, a role skipped: the roles' count limit
  uint8_t planRefusalCause() const override { return plan_cause_; }   // capturing: 6; the roles: 2
  uint8_t plan_cause_ = reg::core::kUnavailableCauseLimit;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  uint16_t boundTo() const override { return groupFn(); }   // bound: plan changes refused (capture §4.1)
  // read returns at most this much (capture §3.2: the probe's choice, within max and max_frame)
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 16 ? max_frame - 16 : 0; }
  // a read's answer is sized by its route's room as well: the largest route's (the zero-copy stages too)
  void setFrameLimits(size_t, size_t largest) override { setFrameLimit(largest); }
  bool notifies() const override { return true; }   // subscribe / unsubscribe in its ops (core §11.3)
  bool subscribe(bool on) override { subscribed_ = on; return true; }
  size_t pending() override;
  size_t pull(uint8_t *out, size_t capacity) override;
  void poll();   // from loop(): turns a finished capture into events
  // GroupTrack (oep.fixture.capture-group): the group drives start / stop through handle(); bound, the host cannot
  bool trackReady() const override { return state_ == kStateConfigured || state_ == kStateDone; }
  bool trackCanStart() const override {   // start's own conditions (capture §3.2): streaming needs the subscription
    return (state_ == kStateConfigured || state_ == kStateDone || state_ == kStateError) &&
           (mode_ != reg::fixture_logic::kModeStreaming || subscribed_);
  }
  uint8_t trackMode() const override { return mode_; }
  bool trackTriggered() const override { return trig_type_ != 0; }
  bool trackCanFollow() const override { return mode_ == reg::fixture_logic::kModeOneShot && trackReady(); }
  bool trackStartFollowing() override;
  void trackTriggerAt(uint64_t ns) override;
  bool trackTriggerNs(uint64_t &ns) const override;
  void trackForce() override { groupOp(kOpForce); }
  bool trackArmed() const override { return static_cast<uint64_t>(produced_) * 8 / (width_ ? width_ : 1) >= pre(); }
  bool trackRate(uint32_t &num, uint32_t &den) const override { num = rate_num_; den = rate_den_; return rate_num_ != 0; }
  uint32_t trackPretrigger() const override { return pretrigger_; }
  // a follower's P_k: what the ring gives back at this width, below the segment's samples (capture §4.1)
  // ... and the ring holds it and what comes in while the trigger is on its way (late_ns), with a DMA chunk and the
  // DMA's lead (kDmaAhead) to spare
  bool trackCanKeep(uint32_t p, uint64_t late_ns) const override {
    const uint8_t w = width_ ? width_ : 1;
    const size_t history = linked_ && streamRingAvailable() ? kStreamRingBytes / 2 : kPretriggerBytes;
    if (p && (p >= samples_ || p > static_cast<uint32_t>(history * 8 / w))) return false;
    const uint64_t late = samplesIn(late_ns, rate_num_, rate_den_) + 1;
    const size_t ring = streamRingAvailable() ? kStreamRingBytes : kRingBytes;   // a follower takes the PSRAM one
    return rate_num_ != 0 && (p + late) * w / 8 + kChunkMax + kDmaAhead <= ring;
  }
  void trackKeep(uint32_t p, uint64_t late_ns) override { follow_pre_ = p; (void)late_ns; }
  bool trackLinked() const override { return linked_ && mode_ == reg::fixture_logic::kModeOneShot; }
  uint8_t trackCanJudge(uint8_t type, uint8_t role, const uint8_t *args) const override;
  void trackJudge(uint8_t type, uint8_t role, const uint8_t *args, uint8_t arm) override;
  void trackUnjudge() override;
  // As the judging track: an edge is found when its DMA chunk (a descriptor, at most kChunkMax bytes) is done, and the
  // harvest hands it on at once (eventFound); a force is known at once.
  static constexpr uint32_t kChunkMax = 4032;
  uint64_t trackLatencyNs() const override { return latencyNs(rate_num_, rate_den_, width_); }
  // as the judging track at this rate and width (configure's judge_latency_ns, §3.3)
  // (a DMA chunk to be done, and the harvest task to come round - it hands the event on itself, eventFound)
  static uint64_t latencyNs(uint32_t num, uint32_t den, uint8_t width) {
    const uint64_t samples = static_cast<uint64_t>(kChunkMax) * 8 / (width ? width : 1);
    return kTaskNs + (num ? samples * den * 1000000000ull / num : 0);
  }
  uint32_t trackLoad() const override { return rate_den_ ? static_cast<uint32_t>(static_cast<uint64_t>(channels_) * rate_num_ / rate_den_) : 0; }
  bool trackStart() override { return groupOp(kOpStart); }
  void trackStop() override { groupOp(kOpStop); }
  uint8_t trackState() const override { return state_; }
  uint8_t trackError() const override { return error_; }
  uint32_t trackGeneration() const override { return generation_; }

 private:
#if defined(OEP_HOST_FAKE_PARLIO)
  friend struct LogicCaptureSerials;   // the host test moves the serials near their wrap
#endif
  bool group_op_ = false;
  uint32_t generation_ = 0;   // one up at every start (nextGeneration); 0 before the first
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
  int pins_[kMaxChannels];
  uint8_t channels_ = 0;
  size_t max_read_ = 1000;
  bool subscribed_ = false;
  // configured
  uint8_t state_ = kStateUnconfigured;
  uint8_t width_ = 1;
  uint32_t samples_ = 0, bytes_ = 0, rate_num_ = 0, rate_den_ = 1;
  // one-shot: the samples of this generation's segment (0: none yet), and whether stop cut it short
  uint32_t kept_samples_ = 0;
  bool kept_short_ = false;
  // running
  parlio_rx_unit_handle_t unit_ = nullptr;
  parlio_rx_delimiter_handle_t delimiter_ = nullptr;
  uint8_t *buffer_ = nullptr;
  volatile bool done_ = false;
  uint64_t start_ns_ = 0;   // the probe's clock: ns since boot (esp_timer x 1000), u64 like the positions
  uint64_t nsOf(uint64_t samples) const;   // samples at the actual rate, in ns (no 128-bit arithmetic on the P4)

  Result configure(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity, bool query);
  bool open(uint32_t rate_hz, uint8_t width, size_t bytes, uint32_t &num, uint32_t &den);
  bool openUnit(uint32_t rate_hz, uint8_t width, bool ring, uint32_t bytes, uint32_t &num, uint32_t &den);
  void closeUnit();
  bool reopenUnit();   // every start: the unit made anew for its receive, as configure leaves it
  void close();
  void forget();   // the plan released or replaced: no configuration, data or segments
  size_t segmentInfo(uint8_t *out) const;
  size_t storeBudget(uint32_t &caps) const;   // bytes the segments may take now (counting the store already held)
  // streaming through the endpoint's zero-copy transport: the harvest copies straight from the DMA ring into stages
  // (internal RAM, one whole push frame each: length prefix, push header, data) and hands full ones to the transport
  // length, core push header, position(u64), len(u16); after the data the generation TLV (kPushTail: tag len(u16) u32, oep-if-capture §3.4)
  static constexpr size_t kStagesMax = 8, kStageFrameMax = 27136, kPushHead = 2 + kPushHeader + 8 + 2,
                          kSegmentInfoBytes = 24,   // position(u64) samples(u32) start_ns(u64) start_uncertainty_ns(u32)
                          kPushTail = kTlvHeader + 4 + kTlvHeader + kSegmentInfoBytes;   // TLV generation, TLV segment_info
  bool direct_ = false;
  uint8_t *stage_[kStagesMax] = {};
  uint8_t stage_count_ = 0;
  volatile uint32_t stage_free_ = 0;   // bit per stage the transport has given back
  int stage_cur_ = -1;
  uint32_t stage_fill_ = 0, stage_since_ = 0, stage_data_ = 0;
  uint64_t stage_pos_ = 0;
  // data format 1 (capture §1.3), streaming over the stages only: the DMA bytes gather into a block of L samples, which
  // is encoded straight into the stage; a stage (one segment) holds whole blocks, none held back
  blocks::Plan plan_;
  blocks::Encoder encoder_;
  uint8_t *block_buf_ = nullptr;        // one block's samples as the capture took them (L x w / 8 bytes)
  uint32_t block_fill_ = 0;             // its bytes so far
  uint64_t enc_pos_ = 0;                // the format 1 stream's position (dropped blocks jump it: their raw bytes)
  uint64_t sample_pos_ = 0;             // samples taken since start
  uint64_t stage_first_sample_ = 0;     // the stage's (segment's) first sample
  uint32_t stage_samples_ = 0;          // its samples
  // the encoding's cost, since start (named diagnostics capture-encode): cycles of the longest block, of all, blocks,
  // and the channels carried raw (fallback) summed over the blocks
  uint32_t enc_max_cycles_ = 0, enc_blocks_ = 0, enc_fallbacks_ = 0;
  uint64_t enc_cycles_ = 0;
  volatile uint32_t stage_drops_ = 0;  // bytes dropped because every stage was queued or in flight
  volatile uint32_t stage_unsent_ = 0; // bytes of stages the transport would not queue (given back unsent)
  bool open_transfer_ = false;         // the last frame queued was whole packets: the host's read is still open
  void closeTransfer();
  bool takeStage();
  bool openStages();
  void freeStages();
  uint32_t streamBuffer(uint32_t samples, uint8_t width, uint32_t segments) const;
  size_t stageFrame() const;
  size_t stagesPossible() const;
  void releaseStages();
  void sendStage();
  static void stageDone(void *context, const uint8_t *buffer);
  size_t store_bytes_ = 0;
  // repeat
  // end: produced_ after it. A chunk the queue had no room for (queue_overflow_) is seen at the next one: its start is
  // past captured_ - the bytes between are dropped with the gap flag where they were (not shifted into the positions
  // after them).
  // ring: where in the ring the chunk begins, counted on from the start round the ring (u32, wraps with it: kRingBytes
  // divides 2^32). The ring is not the stream laid out byte for byte: at every eof_data_len (kSegmentBytes) the DMA closes
  // its descriptor early and goes on at the next descriptor's start, so the rest of that descriptor - up to one
  // descriptor's bytes - keeps an older lap between two chunks of the stream (partialReceive: ring_end_).
  struct Chunk { const uint8_t *data; size_t length; uint32_t end; uint32_t ring; };
  Chunk held_chunk_ = {};
  void harvestBlocks(const Chunk &chunk);   // format 1 streaming (above)
  void emitBlock(uint32_t samples);
  // the chunks lost to the queue before this one: its start past the stream byte taken last (the stages', the spill's)
  uint32_t missed(const Chunk &chunk) const {
    return chunk.end - static_cast<uint32_t>(chunk.length) - static_cast<uint32_t>(captured_ + spill_used_ + spill_drop_);
  }
  // The DMA's ring position past the last chunk it finished (the ISR, partialReceive), counted as Chunk::ring. The DMA
  // writes on within kDmaAhead of it (the descriptor being filled, and one skipped after an early close).
  volatile uint32_t ring_end_ = 0;
  static constexpr uint32_t kDmaAhead = 8192;
  // Byte `ring` (counted as Chunk::ring) is still what the DMA wrote there on that lap: the DMA has not come round to it.
  bool ringIntact(uint32_t ring) const { return ring_end_ + kDmaAhead - ring <= rx_ring_bytes_; }
  // the ring the DMA fills now: ring_ (kRingBytes, internal), or a direct stream's in PSRAM (stream_ring_)
  uint8_t *rx_ring_ = nullptr, *stream_ring_ = nullptr;
  uint32_t rx_ring_bytes_ = kRingBytes;
  void useRing(bool stream);
  bool takeStreamRing();   // the PSRAM ring, taken now if there is room (kept, as the stages'); false: none
  // the stream's ring held, or PSRAM to take it - asked of MALLOC_CAP_SPIRAM alone: the largest free block with
  // MALLOC_CAP_DMA too is 0 on the P4 though heap_caps_aligned_alloc takes it with those caps (the P4X, core 3.3.12)
  bool streamRingAvailable() const {
    return stream_ring_ || heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) >= kStreamRingBytes;
  }
  static constexpr uint32_t kStopDrainMs = 500;   // at stop the stream ring goes out within this (or the rest is dropped)
  // one-shot with a trigger (or a pretrigger): the ring and the harvest task, searching
  static constexpr size_t kPretriggerBytes = kRingBytes / 2;   // history the ring can give back (half of it)
  // the harvest's core: 0, away from loop() (core 1), which answers the host while it streams - on core 1 the harvest
  // kept loop() from the keepalive and the host's link resynced (P4X, 2026-10-20); the USB task is unpinned
  static constexpr BaseType_t kHarvestCore = 0;
  bool triggered_ = false;           // this one-shot goes through the ring
  uint8_t trig_type_ = 0, trig_role_ = 0;
  uint32_t trig_value_ = 0;
  uint32_t pretrigger_ = 0;
  uint8_t arm_ = 1, when_ = 0;   // trigger_opts (capture §3.3)
  bool linked_ = false;          // configured with the linked trigger (§4.4): started only by its group
  uint8_t judged_arm_ = 1, judged_when_ = 0;   // the configured trigger_opts while the group's condition is judged here
  uint8_t format_ = 0;            // the data format (capture §1): 0, or 4 the test sequence
  bool level_false_ = false;      // a level, when 0: a sample not at it seen since arming
  uint32_t follow_pre_ = 0;          // following a group's trigger: the group's pretrigger here (P_k, capture §4.1)
  uint32_t pre() const { return follow_ ? follow_pre_ : pretrigger_; }
  volatile uint8_t trig_phase_ = 0;  // 0 waiting, 1 filling, 2 done
  volatile bool force_ = false;      // force: the trigger is force_sample_ (capture §3.3: the sample at that instant)
  bool have_level_ = false;
  uint8_t last_level_ = 0;
  uint32_t filled_ = 0;              // bytes in buffer_ (from the byte holding the segment's first sample)
  uint64_t seg_first_sample_ = 0;
  // The segment's first sample need not start a byte (w < 8): it is copied from the byte holding it, seg_shift_ bits
  // in, seg_raw_ bytes in all, and shifted down to bit 0 once it is in (alignSegment). seg_samples_: the segment's
  // samples - fewer than samples_ when the trigger came before pretrigger samples were in (capture §3.3).
  uint8_t seg_shift_ = 0;
  bool seg_aligned_ = false;
  uint32_t seg_samples_ = 0, seg_raw_ = 0;
  void alignSegment(uint32_t raw);
  uint32_t filledSamples() const;    // the segment's samples in buffer_ so far
  uint32_t trigger_index_ = 0xFFFFFFFFu;
  volatile bool trig_overrun_ = false;   // the DMA came round while filling: the segment is not contiguous
  bool reported_trigger_ = true;
  bool follow_ = false;                      // started by a group to follow another track's trigger
  volatile bool ext_ready_ = false;          // ... whose sample is known (ext_sample_ written first)
  uint64_t ext_sample_ = 0;
  uint32_t rate_hz_ = 0;                     // as asked (a follower opens the ring at it)
  void harvestTriggered(const Chunk &chunk);
  // A triggered one-shot's map of the stream onto the ring: the recent chunks in order (stream byte, ring position,
  // length; ring kUnmapped for chunks lost to the queue, whose bytes are somewhere in the ring but not known where).
  // The pretrigger is copied through it, chunk by chunk.
  struct Span { uint64_t stream; uint32_t ring; uint32_t length; };
  static constexpr uint32_t kUnmapped = 0xFFFFFFFFu;
  static constexpr size_t kSpans = 64;   // a lap of the ring is 33 descriptors and the early closes
  Span spans_[kSpans];
  uint32_t span_count_ = 0;   // spans recorded (the newest kSpans kept)
  void addSpan(uint64_t stream, uint32_t ring, uint32_t length);
  void mapLost(uint64_t stream, uint32_t length, uint32_t next_ring);
  uint64_t oldestKept() const;   // the oldest stream byte the ring still holds behind the newest span, contiguous to it
  bool copyStream(uint64_t from, uint64_t to);   // stream bytes [from, to) into buffer_ + filled_; false: not all held
  volatile uint64_t force_sample_ = 0;       // force: the sample at that instant (written before force_)
  // What an open could not get: storage (refused unavailable cause 3) or the peripheral (failed, state 6).
  enum class Open : uint8_t { kOk, kNoMemory, kFailed };
  bool findTrigger(const uint8_t *data, size_t length, uint64_t first_sample, uint64_t &at);
  Result startTriggered(uint8_t *out, size_t capacity);
  Open openTriggered(uint32_t rate_hz, uint8_t width, uint32_t bytes, uint32_t &num, uint32_t &den);
  void pollTriggered();
  // slot: the store slot holding it (the serial wraps at 2^32, capture §2.2: not a multiple of segment_count_)
  // (in this order: 40 bytes, no padding added by trigger_index - kInfos of them in internal RAM, the stages' room)
  struct Info { uint32_t serial; uint32_t trigger_index; uint64_t position; uint32_t samples; uint32_t slot; uint64_t start_ns;
                uint8_t flags; };   // trigger_index: a repeat with a trigger's (0xFFFFFFFF: none)
  // serial u32, position u64, samples u32, start_ns u64, start_uncertainty_ns u32, trigger_index u32, flags u8, generation u32
  static constexpr size_t kInfoBytes = 37;
  size_t storeMax() const;   // the most bytes the segment store can ever take (describe mode: the maximum, not the free)
  uint8_t mode_ = 1;
  uint32_t segment_bytes_ = 0, segment_count_ = 0;
  uint8_t *ring_ = nullptr, *store_ = nullptr;
  // The DMA ring is taken once and never freed (takeRing): giving it up to the one-shot's segment let the rest of the
  // firmware take a piece of its 128 KiB, and no later triggered or repeat configure found a block again. An immediate
  // one-shot's segment is the ring itself (buffer_in_ring_).
  bool buffer_in_ring_ = false;
  bool takeRing();
  Result noStorage(uint8_t *out, size_t capacity);
  QueueHandle_t queue_ = nullptr;
  TaskHandle_t task_ = nullptr;
  volatile bool harvesting_ = false;
  // completed_: the next serial to finish (serial_done), released_: the oldest not released; both wrap (core §2.6)
  volatile uint32_t completed_ = 0, released_ = 0, fill_ = 0, queue_overflow_ = 0;
  uint32_t fill_slot_ = 0;                // the store slot being filled (harvest task)
  volatile uint32_t kept_infos_ = 0;      // finished segments whose info is kept: up to kInfos - 1 (one is being filled)
  volatile uint64_t captured_ = 0, dropped_ = 0;   // bytes the DMA delivered / bytes not stored (no free segment)
  volatile uint32_t produced_ = 0;                  // bytes the ISR has seen finished (the DMA write position, wraps)
  volatile uint32_t overruns_ = 0;                  // chunks the DMA rewrote before the harvest had copied them
  volatile bool gap_pending_ = false, paused_ = false;
  // capture §2.2: a segment that could not be kept seamless (a chunk lost to the queue, the DMA ring come round over
  // bytes not yet copied) is never handed out: the harvest drops it and stops taking data (lost_), poll() stops the
  // track in state 6 (stopped reason 3, error 2). lost_pos_: where that segment began (status write_pos stays there).
  volatile bool lost_ = false;
  uint64_t lost_pos_ = 0;
  uint8_t error_ = reg::fixture_logic::kErrorPeripheral;   // status's error TLV in state 6
  void loseSegment();          // harvest task: the segment being filled (or the stage) goes, nothing more is taken
  void failLost();             // loop: the track stops on it (state 6, stopped reason 3 error 2)
  void fail(uint8_t error);    // state 6 with stopped reason 3 and `error` (capture §3.2)
  uint32_t reported_ = 0;
  // streaming: the next byte to push is sent_off_ into segment sent_seg_
  volatile uint32_t sent_seg_ = 0;
  uint32_t sent_off_ = 0;
  uint32_t segmentLength(uint32_t serial) const;
  bool findSegment(uint64_t position, uint32_t &serial, uint32_t &offset) const;
  Info infos_[kInfos];
  static bool partialReceive(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context);
  static void harvestTask(void *context);
  void harvest(const Chunk &chunk);
  // A repeat with a trigger (capture §2.1): the first segment starts pretrigger samples before the trigger; rearm 0 the
  // segments after it follow on, rearm 1 each waits for its own trigger from the last one's end (rep_wait_; arm 1:
  // once its pretrigger is in). The samples are whole bytes (w 8 or 16), so a segment starts at a byte. Its positions
  // are the segments' bytes one after another (data_pos_: the time waited for a trigger takes none; the bytes discarded
  // while paused count, as a plain repeat's). The ring's recent chunks are mapped (spans_) for the pretrigger.
  bool rep_trig_ = false;
  uint8_t rearm_ = 0;
  volatile bool rep_wait_ = false;             // looking for the next segment's trigger (harvest task)
  volatile bool rep_started_ = false;          // the first segment's trigger found (state 2 -> 3)
  uint64_t arm_from_ = 0;                      // the first sample a trigger may be at (arming: the floor and arm 1's pretrigger)
  uint64_t floor_sample_ = 0;                  // the first sample the next segment may hold (the last one's end)
  volatile uint64_t data_pos_ = 0;             // the next segment's position
  uint32_t seg_trigger_index_ = 0xFFFFFFFFu;   // the segment being filled's (an Info's when it starts)
  void harvestRepeatTriggered(const Chunk &chunk);
  bool harvestRepeatTrigger(const Chunk &chunk, uint64_t base, size_t &at);   // waiting: the trigger found -> the segment begun
  void beginTriggeredSegment(uint64_t trigger, uint64_t base);
  // rearm 1's records (§3.10): serial (from 0 in the generation), the original sample, ns, uncertainty
  struct TriggerRecord { uint64_t sample; uint64_t ns; uint32_t serial; uint32_t uncertainty; };   // 24 bytes
  TriggerRecord *records_ = nullptr;   // kTriggerRecords of them, taken by the first rearm 1 configure (PSRAM when there is some)
  volatile uint32_t record_next_ = 0;
  // the triggered events found by the harvest, sent from loop() (poll): serial index record sample
  struct TriggerFound { uint64_t sample; uint32_t serial, index, record; };
  static constexpr size_t kFound = 16;
  TriggerFound found_[kFound] = {};
  volatile uint32_t found_head_ = 0;
  uint32_t found_tail_ = 0;
  void reportFound();
  bool copyStreamTo(uint8_t *dst, uint64_t from, uint64_t to);   // stream bytes [from, to) to dst; false: not all held
  size_t harvestDirect(const Chunk &chunk, size_t at);   // -> where it got to (short: no stage free yet)
  uint32_t ring_peak_ = 0;   // the most the stream ring held unharvested since the start (bytes; capture-drops)
  // Streaming over the stages with PSRAM: the spill, a FIFO behind the DMA ring (which cannot grow past 1 MiB). With
  // no stage free the harvest moves what the ring holds into it, and the stages take from it first, so the host may
  // pause about kSpillBytes / rate (200 ms at 40 MB/s) instead of the ring's 26 ms. Its bytes are the stream's
  // [captured_, captured_ + spill_used_); a drop while it holds any (full, or the DMA came round) is put after it
  // (spill_drop_: the stream bytes after it dropped, every one until it has gone - no marks inside it). Harvest task
  // only. Taken with the stream ring, given back with the stages (releaseStages); none: the ring alone, as before.
  uint8_t *spill_ = nullptr;
  size_t spill_head_ = 0, spill_used_ = 0;
  uint64_t spill_drop_ = 0;
  uint32_t spill_peak_ = 0;   // the most it held since the start (bytes; capture-drops)
  // capture-spill (status, named): the spill's copies in and out (bytes, CPU cycles), the DMA overruns while it had room,
  // the longest between two passes of the harvest's loop (cycles) - since the start
  uint64_t spill_in_bytes_ = 0, spill_in_cycles_ = 0, spill_out_bytes_ = 0, spill_out_cycles_ = 0;
  uint32_t overrun_room_ = 0, harvest_gap_max_ = 0, harvest_last_ = 0;
  void countOverrun() {
    overruns_ = overruns_ + 1;
    if (spill_ && spill_used_ < kSpillBytes) overrun_room_ = overrun_room_ + 1;
  }
  // what one drain moves at most before the harvest looks at the ring again: a few chunks' worth, more than comes in
  // meanwhile (one chunk a pass), so the spill still empties
  static constexpr size_t kSpillDrainStep = 16 * 1024;
  size_t spillIn(const Chunk &chunk, size_t at);   // -> where it got to (short: the spill is full)
  bool drainSpill();                               // into the stages while one is free; -> moved any
  void dropAfter(size_t n);                        // n stream bytes dropped, after what the spill holds
  bool held_ = false;        // the harvest's chunk not all taken (held_chunk_ from held_at_): no stage was free
  size_t held_at_ = 0;
  void dropDirect(size_t n);
  void finishSegment(uint32_t bytes, uint8_t flags);
  Open openRepeat(uint32_t rate_hz, uint8_t width, uint32_t samples, uint32_t segments, uint32_t &num, uint32_t &den,
                  uint32_t &actual_samples, uint32_t &actual_segments);
  Result startRepeat(uint8_t *out, size_t capacity);
  void stopRepeat();
  size_t infoBytes(const Info &info, uint8_t *out) const;
  static bool receiveDone(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context);
  // An immediate one-shot's DMA nodes as they finish (produced_: the bytes of the segment written so far, for stop).
  static bool oneShotProgress(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *e, void *context);
};

}  // namespace oep

#endif
