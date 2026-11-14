// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 logic capture (oep.fixture.logic revision 1, oep-spec oep-if-capture) on a WCH CH32H415 / H416 / H417's UHSIF
// (ArduinoCore-CH32RV's CH32RVUhsif): the 16 data lines latched every clock and written by the UHSIF's own DMA, streamed
// over the USB vendor bulk port without a copy. Streaming only (mode 3), immediate, format 0, w 16 (DATAn on bit n).
//
// The data lines and their pads: DATA0-5 PD10-PD15, DATA6-8 PF0-PF2, DATA9-12 PC6-PC9, DATA13-15 PA13-PA15. A role
// goes on one of those channels; the sample is the UHSIF's 16 bits whatever the plan (pos[k] = the role's DATA line).
// The rates: SYSCLK / n (n 2-64) and the USBHS PLL (480 MHz, from the crystal) / n (n 3-64), up to 200 MS/s; configure
// takes the nearest.
//
// The ring: buffers of kBufferBytes in the order the UHSIF writes them (two threads taking turns, each a ring of its
// own with a stride of two buffers), each with kGapBytes before it. A buffer is a segment and one data frame (capture
// §3.4): its push header goes at the end of the gap before it, and the frame's TLV tail (generation, segment_info, a
// padding TLV, core §10.2) at the start of the gap after it - so the finished buffers next to each other are one
// contiguous run of whole frames, handed to the transport as one transfer from the gap's start (the padding makes tail +
// header the gap's 64 bytes: 4-byte aligned). A run starts with the tail of the last frame of the run before; the first
// frame's header (no tail before it) and the last tail at the stop go through a small aligned scratch. A run leaves its
// last frame open (DirectTransport::queueDataOpen): results wait for the frame's end (core §10.4), so when some wait
// the open frame's tail goes alone and the next run starts with its header alone.
// Behind the writer by more than half the ring, the oldest finished buffers are dropped (the next frame's position
// jumps, status flags bit0); a buffer the UHSIF wrote again while it was being sent stops the capture (state 6, error
// 2, capture §2.2: the host drops data from the write position on).
//
// OEP_HOST_FAKE_UHSIF: a host test's fake of CH32RVUhsif (tests/host/shim).
#pragma once

#include <Arduino.h>

#include "Oep.h"
#include "OepCaptureGroup.h"
#include "OepPinTable.h"

#if defined(CH32RV_SERIES_CH32H417) || defined(CH32RV_SERIES_CH32H416) || defined(CH32RV_SERIES_CH32H415) || \
    defined(OEP_HOST_FAKE_UHSIF)
#define OEP_LOGIC_UHSIF 1
#include <CH32RVUhsif.h>

namespace oep {

class Endpoint;

class H41xLogicCapture final : public Interface {
 public:
  static constexpr uint8_t kMaxChannels = 16;
  static constexpr uint32_t kMaxHz = 200000000, kMinHz = 6250000;   // SYSCLK / 64 at the bottom
  static constexpr size_t kGapBytes = 64;          // before each buffer: a frame's tail, padding, the next header
  static constexpr size_t kMaxBufferBytes = 16384;  // a buffer's most (smaller when the route's frame is): 41 us at 200 MS/s
  // The ring: 16 buffers (8 a thread; the UHSIF takes 16 a thread), or 8 or 4 when the heap has no room for more (the
  // H41x runs its code from RAM). A run is a quarter of it at most; the oldest go past half of it.
  static constexpr uint8_t kMaxBuffers = 16, kMinBuffers = 4;

  H41xLogicCapture(Endpoint &endpoint, const PinTable &pins, uint16_t instance = 0)
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
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void setFrameLimit(size_t max_frame) override { max_frame_ = max_frame; }
  void setFrameLimits(size_t, size_t largest) override { setFrameLimit(largest); }
  bool notifies() const override { return true; }
  bool subscribe(bool on) override { subscribed_ = on; return true; }
  void poll();   // from loop(): the finished buffers handed to the transport

  // The UHSIF data line a channel's pad is (0-15), or -1
  static int dataLineOf(uint16_t channel);
  // The rate nearest `rate` the UHSIF makes: its clock source and divider (n - 1), and the rate as num / den
  static bool rateOf(uint32_t rate, CH32RVUhsifClock &source, uint8_t &divider, uint32_t &num, uint32_t &den);

#if defined(OEP_HOST_FAKE_UHSIF)
  uint8_t hostBuffers() const { return buffers_; }
  CH32RVUhsif &hostUhsif() { return uhsif_; }
  uint8_t *hostRing() const { return ring_; }
  size_t hostBufferBytes() const { return buffer_bytes_; }
#endif

 private:
  Endpoint &endpoint_;
  const PinTable &table_;
  uint16_t instance_;
  uint8_t plan_cause_ = reg::core::kUnavailableCauseLimit;
  int8_t line_[kMaxChannels] = {};   // role k's DATA line
  uint8_t channels_ = 0;
  size_t max_frame_ = 1024;
  bool subscribed_ = false;
  uint8_t state_ = reg::fixture_logic::kStateUnconfigured;
  uint8_t error_ = 0;
  uint32_t generation_ = 0;
  CH32RVUhsifClock source_ = CH32RVUhsifClock::Sysclk;
  uint8_t divider_ = 0;
  uint32_t num_ = 0, den_ = 1;   // the rate
  CH32RVUhsif uhsif_;
  uint8_t *ring_ = nullptr;      // buffers_ x (kGapBytes + buffer_bytes_), 32-byte aligned
  uint8_t buffers_ = 0;          // in the ring (configure: as many as fit, kMaxBuffers down to kMinBuffers)
  size_t buffer_bytes_ = 0;      // at configure, from the host's frame
  uint64_t start_ns_ = 0;
  // buffers by number since the start (buffer b is ring slot b % buffers_, stream position b x buffer_bytes_)
  uint32_t finished_ = 0;        // the UHSIF finished these
  uint32_t next_ = 0;            // the next to send
  uint32_t run_first_ = 0;      // the run out's first buffer
  bool run_out_ = false, head_out_ = false, tail_out_ = false;
  bool frame_open_ = false;      // the last frame sent still needs its tail (at the next run's start, or alone)
  bool head_sent_ = false;       // next_'s header went alone (no frame open before it)
  uint32_t tail_of_ = 0;         // the open frame's buffer
  uint32_t raw_[2] = {};         // each thread's bytesWritten as last read (it wraps at 2^32)
  uint64_t bytes_[2] = {};       // and since the start
  bool running_ = false;         // the UHSIF runs (state 3)
  bool draining_ = false;        // stopped: the finished buffers still going out
  uint64_t dropped_bytes_ = 0;
  uint32_t overwritten_ = 0;
  uint32_t lost_first_ = 0, lost_at_ = 0;   // the first buffer written again while it went out
  uint8_t head_scratch_[32] __attribute__((aligned(4))) = {};   // a header with no frame open before it
  uint8_t tail_scratch_[64] __attribute__((aligned(4))) = {};   // a tail that closes the stream for results or the end

  Result configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query);
  size_t bufferBytesFor(size_t max_frame) const;
  uint8_t *slot(uint32_t b) const { return ring_ + (b % buffers_) * (kGapBytes + buffer_bytes_) + kGapBytes; }
  uint32_t runMax() const { return buffers_ / 4; }
  uint32_t finishedNow();
  bool begin();
  void halt();
  void fail(uint8_t error);
  void stopped(uint8_t reason, uint8_t error = 0);
  void putHeader(uint8_t *at, uint32_t b);   // 17 bytes before buffer b's data
  void putTail(uint8_t *at, uint32_t b);     // 39 bytes after it (with the padding)
  void sendRun();
  void closeFrame();   // the open frame's tail alone: results may go, or the stream ends
  static void runDone(void *context, const uint8_t *buffer);
  static void headDone(void *context, const uint8_t *buffer);
  static void tailDone(void *context, const uint8_t *buffer);
};

}  // namespace oep

#endif
