// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepH41xLogic.h"

#if defined(OEP_LOGIC_UHSIF)

#include <stdlib.h>
#include <string.h>

#include "OepEndpoint.h"

namespace oep {
namespace {

namespace cap = reg::fixture_logic;
enum : uint8_t { kTagMode = cap::kTlvConfigureMode, kTagRate = cap::kTlvConfigureRate,
                 kTagSamples = cap::kTlvConfigureSamples, kTagSegments = cap::kTlvConfigureSegments,
                 kTagTrigger = cap::kTlvConfigureTrigger, kTagPretrigger = cap::kTlvConfigurePretrigger,
                 kTagFormat = cap::kTlvConfigureFormat };

constexpr uint32_t kChannelsPerPort = 24;   // a channel is port x 24 + pin (OepPlatform.h, platformPad)
constexpr size_t kHead = 2 + kPushHeader + 8 + 2;   // length, push header, position(u64), len(u16): 17
constexpr size_t kInfoBytes = 24;                   // segment_info: position(u64) samples(u32) start_ns(u64) uncertainty(u32)
// The padding TLV's value: a frame's tail and the next header fill the gap between two buffers exactly (a run is the
// buffers and their gaps end to end), 64 bytes - a 4-byte multiple, so a run from the gap's start is aligned too
constexpr size_t kPadValue = H41xLogicCapture::kGapBytes - kHead - (kTlvHeader + 4 + kTlvHeader + kInfoBytes + kTlvHeader);
constexpr size_t kTail = kTlvHeader + 4 + kTlvHeader + kInfoBytes + kTlvHeader + kPadValue;
static_assert(kTail + kHead == H41xLogicCapture::kGapBytes, "a tail and a header fill the gap");
static_assert(H41xLogicCapture::kGapBytes % 32 == 0, "the buffers stay 32-byte aligned");
constexpr uint32_t kStartUncertaintyNs = 20000;     // the UHSIF's start against the clock read after it
constexpr size_t kTransportRoom = 16;              // data entries the transport queues (Ch32DirectBulkStream: 32)

uint32_t gcd(uint32_t a, uint32_t b) {
  while (b) { const uint32_t t = a % b; a = b; b = t; }
  return a;
}

// UHSIF DATAn's pad as (port, pin): DATA0-5 PD10-15, DATA6-8 PF0-2, DATA9-12 PC6-9, DATA13-15 PA13-15
constexpr uint8_t kLinePort[16] = {3, 3, 3, 3, 3, 3, 5, 5, 5, 2, 2, 2, 2, 0, 0, 0};
constexpr uint8_t kLinePin[16] = {10, 11, 12, 13, 14, 15, 0, 1, 2, 6, 7, 8, 9, 13, 14, 15};

}  // namespace

int H41xLogicCapture::dataLineOf(uint16_t channel) {
  for (int n = 0; n < 16; ++n)
    if (channel == kLinePort[n] * kChannelsPerPort + kLinePin[n]) return n;
  return -1;
}

// The nearest of SYSCLK / n and the USBHS PLL / n (n 1-64) up to kMaxHz; the PLL (from the crystal) when both are as near
bool H41xLogicCapture::rateOf(uint32_t rate, CH32RVUhsifClock &source, uint8_t &divider, uint32_t &num, uint32_t &den) {
  const CH32RVUhsifClock sources[2] = {CH32RVUhsifClock::UsbhsPll, CH32RVUhsifClock::Sysclk};
  uint32_t best_err = UINT32_MAX;
  bool found = false;
  for (const CH32RVUhsifClock src : sources) {
    const uint32_t hz = CH32RVUhsif::sourceHz(src);
    for (uint32_t n = 1; n <= 64; ++n) {
      if (hz / n > kMaxHz) continue;
      const uint64_t f = hz / n;   // the clock / n is exact in Hz for both sources (400 and 480 MHz), compared as such
      const uint64_t err = f > rate ? f - rate : rate - f;
      if (hz % n == 0 ? err < best_err : err + 1 < best_err) {
        const uint32_t g = gcd(hz, n);
        best_err = static_cast<uint32_t>(err);
        source = src;
        divider = static_cast<uint8_t>(n - 1);
        num = hz / g;
        den = n / g;
        found = true;
      }
    }
  }
  return found;
}

size_t H41xLogicCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  uint8_t roles[kMaxChannels];
  for (uint8_t k = 0; k < kMaxChannels; ++k) roles[k] = k;
  ChannelMask lines = {};
  for (int n = 0; n < 16; ++n) lines = lines | channelBit(static_cast<uint16_t>(kLinePort[n] * kChannelsPerPort + kLinePin[n]));
  w.roleChannels(roles, kMaxChannels, table_.allowedMask() & lines);
  uint8_t mode[9] = {cap::kModeStreaming};   // segments of a buffer each, pushed (needs a subscription)
  putU32(mode + 1, kMaxBufferBytes / 2);
  putU32(mode + 5, kMaxBuffers);
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  uint8_t range[8];
  putU32(range, kMinHz);
  putU32(range + 4, kMaxHz);
  w.put(cap::kTlvDescribeRateRange, range, sizeof range);
  w.u8(cap::kTlvDescribeChannels, kMaxChannels);
  uint8_t trig[9];
  putTriggerDecl(trig, 1u << cap::kTriggerImmediate, 0, 0);
  w.put(cap::kTlvDescribeTrigger, trig, sizeof trig);
  return w.ok() ? w.length() : 0;
}

// Roles 0 .. n-1, each once, each on a UHSIF data line
uint8_t H41xLogicCapture::planCheck(const RoleAssignment *roles, size_t count) {
  plan_cause_ = reg::core::kUnavailableCauseWrongState;
  if (running_ || draining_ || paused_) return kRejectUnavailable;
  plan_cause_ = reg::core::kUnavailableCauseLimit;
  uint32_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role >= kMaxChannels || !table_.allowed(roles[i].channel) || dataLineOf(roles[i].channel) < 0)
      return kRejectUnsupported;
    if (seen & (1u << roles[i].role)) return kRejectUnavailable;
    seen |= 1u << roles[i].role;
  }
  if (count > kMaxChannels) return kRejectUnavailable;
  return (count == 0 || seen == (1u << count) - 1) ? 0 : kRejectUnavailable;
}

// Taking the plan changes no pin: the UHSIF reads its lines whatever their mode (capture §1.2, core §8)
bool H41xLogicCapture::planApply(const RoleAssignment *roles, size_t count) {
  halt();
  for (size_t i = 0; i < count; ++i) line_[roles[i].role] = static_cast<int8_t>(dataLineOf(roles[i].channel));
  channels_ = static_cast<uint8_t>(count);
  state_ = cap::kStateUnconfigured;
  return true;
}

void H41xLogicCapture::planRelease() {
  halt();
  paused_ = false;
  draining_ = false;
  channels_ = 0;
  state_ = cap::kStateUnconfigured;
}

// A buffer: whole 64-byte units (the UHSIF's), its frame (length, message) within the host's frame, at most
// kMaxBufferBytes
size_t H41xLogicCapture::bufferBytesFor(size_t max_frame) const {
  const size_t over = 2 + (kHead - 2) + kTail;
  if (max_frame <= over + 64) return 0;
  size_t b = (max_frame - over) / 64 * 64;
  return b > kMaxBufferBytes ? kMaxBufferBytes : b;
}

// Buffers the UHSIF finished: each thread's bytes since the start (bytesWritten wraps at 2^32, ~20 s a thread at
// 200 MS/s; poll() reads far more often); thread 0 writes the even buffers, thread 1 the odd ones
uint32_t H41xLogicCapture::finishedNow() {
  if (!running_ || !buffer_bytes_) return finished_;
  for (uint8_t n = 0; n < 2; ++n) {
    const uint32_t now = uhsif_.bytesWritten(n);
    bytes_[n] += now - raw_[n];
    raw_[n] = now;
  }
  return epoch_b_ + static_cast<uint32_t>(bytes_[0] / buffer_bytes_ + bytes_[1] / buffer_bytes_);
}

void H41xLogicCapture::halt() {
  if (running_) uhsif_.stop();
  running_ = false;
}

// The finished buffers not yet sent go; the open frame is still closed (draining) so the host's framing holds.
void H41xLogicCapture::fail(uint8_t error) {
  halt();
  paused_ = false;
  next_ = finished_;
  draining_ = frame_open_ || run_out_ || head_out_;
  state_ = cap::kStateError;
  error_ = error;
  stopped(cap::kStoppedReasonError, error);
}

void H41xLogicCapture::stopped(uint8_t reason, uint8_t error) {
  if (!subscribed_) return;
  uint8_t e[6] = {reason, error};
  putU32(e + 2, generation_);
  endpoint_.event(*this, cap::kEventStopped, e, sizeof e);
}

// Every clock the machine latches the lines and pushes the sample; two threads take turns a buffer each (as
// CH32RVLogicCapture's UHSIF backend, docs there): thread 0 the even buffers, thread 1 the odd ones, each its own ring of
// buffers_ / 2 with a stride of two buffers and their gaps.
bool H41xLogicCapture::begin() {
  constexpr CH32RVUhsifInput hit = CH32RVUhsifInput::DataCountHit, one = CH32RVUhsifInput::One;
  constexpr uint8_t S = CH32RVUhsif::SampleData;
  constexpr uint32_t P = CH32RVUhsif::Push, C = CH32RVUhsif::CountData,
                     R = CH32RVUhsif::ResetDataCount | CH32RVUhsif::Interrupt;
  const uint32_t t0 = CH32RVUhsif::thread(0), t1 = CH32RVUhsif::thread(1);
  const CH32RVUhsifState states[4] = {
      {{hit, one, one, one}, 0x0000, {1, S, P | C | t0}, {1, S, P | C | t0}},
      {{hit, one, one, one}, 0xAAAA, {0, S, P | C | t0}, {2, S, P | R | t1}},
      {{hit, one, one, one}, 0x0000, {3, S, P | C | t1}, {3, S, P | C | t1}},
      {{hit, one, one, one}, 0xAAAA, {2, S, P | C | t1}, {0, S, P | R | t0}},
  };
  const CH32RVUhsifTransition first = {0, S, P | CH32RVUhsif::ResetDataCount | t0};
  const uint32_t stride = 2 * (kGapBytes + buffer_bytes_);
  const uint32_t samples = static_cast<uint32_t>(buffer_bytes_ / 2);
  if (!uhsif_.begin(divider_, source_) || !uhsif_.setStates(states, 4, first) || !uhsif_.setDataCounter(samples - 2) ||
      !uhsif_.setRing(0, slot(0), static_cast<uint16_t>(buffer_bytes_), buffers_ / 2, stride) ||
      !uhsif_.setRing(1, slot(1), static_cast<uint16_t>(buffer_bytes_), buffers_ / 2, stride))
    return false;
  raw_[0] = raw_[1] = 0;
  bytes_[0] = bytes_[1] = 0;
  if (!uhsif_.start()) return false;
  epoch_ns_ = nowNs();
  running_ = true;
  return true;
}

// Buffer b's stream position and its first sample's time: from the epoch it was taken in (the start, or the restart
// after a pause)
uint64_t H41xLogicCapture::positionOf(uint32_t b) const {
  return epoch_pos_ + static_cast<uint64_t>(b - epoch_b_) * buffer_bytes_;
}
uint64_t H41xLogicCapture::nsOf(uint32_t b) const {
  const uint64_t first = static_cast<uint64_t>(b - epoch_b_) * (buffer_bytes_ / 2);
  const uint64_t q = first / num_, r = first % num_;
  return epoch_ns_ + q * den_ * 1000000000ull + static_cast<uint64_t>(static_cast<double>(r * den_) * 1e9 / num_);
}

// The writer about to come round onto the run going out (the host is not taking IN as fast as the UHSIF writes): the
// UHSIF stops rather than write over it. Once every finished buffer is out and the frame closed, it starts again at
// the ring's first buffer; the samples it did not take are a drop - the stream position jumps by the time it stood
// still (capture §2.1 rule 1: the next frame's position shows the gap, status flags bit0).
void H41xLogicCapture::pause() {
  halt();
  paused_ = true;
  ++pauses_;
}
void H41xLogicCapture::resume() {
  const uint64_t was = positionOf(finished_);
  const uint64_t now = nowNs();
  // the samples since the epoch's start at the rate, in whole samples (2 bytes): where the new epoch starts
  const uint64_t dns = now - epoch_ns_;
  const uint64_t taken = static_cast<uint64_t>(static_cast<double>(dns) * num_ / den_ / 1e9);
  uint64_t at = epoch_pos_ + taken * 2;
  if (at < was) at = was;
  epoch_b_ = (finished_ + buffers_ - 1) / buffers_ * buffers_;   // the ring starts again at slot 0
  epoch_pos_ = at;
  finished_ = next_ = epoch_b_;
  dropped_bytes_ += at - was;
  paused_ = false;
  if (!begin()) fail(cap::kErrorPeripheral);
}

// Frame b's push header: the 17 bytes ending at its data
void H41xLogicCapture::putHeader(uint8_t *s, uint32_t b) {
  uint16_t fn = 0, min_bytes = 0;
  uint32_t max_delay = 0;
  endpoint_.directPush(*this, fn, min_bytes, max_delay);
  putU16(s, static_cast<uint16_t>(kHead - 2 + buffer_bytes_ + kTail));
  s[2] = kRolePush;
  putU16(s + 3, fn);
  putU16(s + 5, endpoint_.takeSeq(fn));
  putU64(s + 7, positionOf(b));
  putU16(s + 15, static_cast<uint16_t>(buffer_bytes_));
}

// Frame b's TLV tail: generation, segment_info (the segment is the buffer: its start the start's time plus its samples
// at the rate), the padding
void H41xLogicCapture::putTail(uint8_t *t, uint32_t b) {
  putTlvHeader(t, cap::kTlvDataGeneration, 4);
  putU32(t + kTlvHeader, generation_);
  uint8_t *info = t + kTlvHeader + 4;
  putTlvHeader(info, cap::kTlvDataSegmentInfo, kInfoBytes);
  putU64(info + kTlvHeader, positionOf(b));
  putU32(info + kTlvHeader + 8, static_cast<uint32_t>(buffer_bytes_ / 2));
  putU64(info + kTlvHeader + 12, nsOf(b));
  putU32(info + kTlvHeader + 20, kStartUncertaintyNs);
  uint8_t *pad = info + kTlvHeader + kInfoBytes;
  putTlvHeader(pad, reg::kTagDataPadding, kPadValue);
  memset(pad + kTlvHeader, 0, kPadValue);
}

// The next run: the finished buffers from next_, up to runMax() and the ring's end, in one transfer - from the gap
// before the first (the open frame's tail, then this one's header) when a frame is open, else from the data after the
// header went alone. The run's last frame stays open. Results waiting: the open frame is closed instead.
void H41xLogicCapture::sendRun() {
  if (run_out_ || head_out_ || tail_out_) return;
  DirectTransport *t = endpoint_.direct();
  uint16_t fn = 0, min_bytes = 0;
  uint32_t max_delay = 0;
  if (!t || !endpoint_.directPush(*this, fn, min_bytes, max_delay)) {   // nobody may receive it: the buffers go
    // a frame begun is finished all the same (core §10.3: the subscription's end does not cut a frame), or the port's
    // results would wait for its end for ever: a header that went alone gets its buffer, then the open frame its tail
    if (head_sent_) {
      if (run_out_ || head_out_ || tail_out_ || !t || !t->queueDataOpen(slot(next_), buffer_bytes_, runDone, this)) return;
      run_first_ = next_;
      run_out_ = frame_open_ = true;
      head_sent_ = false;
      tail_of_ = next_++;
    }
    if (finished_ > next_) {
      dropped_bytes_ += static_cast<uint64_t>(finished_ - next_) * buffer_bytes_;
      next_ = finished_;
    }
    closeFrame();
    return;
  }
  if (frame_open_ && t->resultsWaiting()) { closeFrame(); return; }
  if (finished_ <= next_) return;
  uint32_t end = finished_;
  if (end - next_ > runMax()) end = next_ + runMax();
  const uint32_t wrap = (next_ / buffers_ + 1) * buffers_;   // contiguous only up to the ring's end
  if (end > wrap) end = wrap;
  if (t->queued() + 2 > kTransportRoom) return;   // the header alone and the run both go, or neither (below)
  const uint8_t *from = slot(next_);
  if (frame_open_) {   // the headers in frame order: their seq (core §10.2)
    putTail(slot(next_) - kHead - kTail, tail_of_);
    putHeader(slot(next_) - kHead, next_);
    from = slot(next_) - kHead - kTail;
  } else if (!head_sent_) {   // no frame open: the header alone (17 bytes: not 4-aligned before the data)
    putHeader(head_scratch_, next_);
    if (!t->queueDataOpen(head_scratch_, kHead, headDone, this)) return;
    head_out_ = head_sent_ = true;
  }
  for (uint32_t b = next_ + 1; b < end; ++b) {
    putTail(slot(b) - kHead - kTail, b - 1);
    putHeader(slot(b) - kHead, b);
  }
  const uint8_t *to = slot(end - 1) + buffer_bytes_;
  if (!t->queueDataOpen(from, static_cast<size_t>(to - from), runDone, this)) return;   // again at the next poll
  run_first_ = next_;
  run_out_ = true;
  frame_open_ = true;
  head_sent_ = false;
  tail_of_ = end - 1;
  next_ = end;
}

void H41xLogicCapture::runDone(void *context, const uint8_t *) {
  auto *self = static_cast<H41xLogicCapture *>(context);
  self->run_out_ = false;
  // the writer back on the run's first buffer while it went out: what the host got from there on is not this run's
  if (self->running_ && self->finishedNow() >= self->run_first_ + self->buffers_ && !self->overwritten_++)
    self->lost_first_ = self->run_first_;
}

void H41xLogicCapture::headDone(void *context, const uint8_t *) { static_cast<H41xLogicCapture *>(context)->head_out_ = false; }
void H41xLogicCapture::tailDone(void *context, const uint8_t *) { static_cast<H41xLogicCapture *>(context)->tail_out_ = false; }

void H41xLogicCapture::closeFrame() {
  DirectTransport *t = endpoint_.direct();
  if (!frame_open_ || run_out_ || head_out_ || tail_out_ || !t) return;
  putTail(tail_scratch_, tail_of_);
  if (!t->queueData(tail_scratch_, kTail, tailDone, this)) return;
  tail_out_ = true;
  frame_open_ = false;
}

void H41xLogicCapture::poll() {
  if (!running_ && !draining_ && !paused_) return;
  if (running_) finished_ = finishedNow();
  // two buffers from the run going out: stop the writer before it gets there
  if (running_ && run_out_ && finished_ + 2 >= run_first_ + buffers_) pause();
  if (overwritten_ && running_) {   // capture §2.2: the data from the write position on is not to be trusted
    lost_at_ = lost_first_;     // status's write_pos: the first buffer written again while it went out
    fail(cap::kErrorStorage);   // (the open frame still closes: draining)
    return;
  }
  // far behind: the oldest go (the next frame's position jumps) - not one whose header already went alone (its frame
  // is open on the wire: its data must follow)
  if (finished_ - next_ > buffers_ / 2u && !head_sent_) {
    const uint32_t keep = finished_ - buffers_ / 2u;
    dropped_bytes_ += static_cast<uint64_t>(keep - next_) * buffer_bytes_;
    next_ = keep;
  }
  sendRun();
  if (paused_ && next_ >= finished_ && !run_out_ && !head_out_) {   // all out: close the frame, then start again
    closeFrame();
    if (!frame_open_ && !tail_out_) resume();
  }
  if (draining_ && next_ >= finished_ && !run_out_ && !head_out_) {
    closeFrame();
    if (!frame_open_ && !tail_out_) draining_ = false;
  }
}

Result H41xLogicCapture::configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query) {
  static const uint8_t kKnown[] = {kTagMode, kTagRate, kTagSamples, kTagSegments, kTagTrigger, kTagPretrigger,
                                   capture_tag::kTriggerOpts, kTagFormat};
  Tail tail;
  Result unknown_critical;
  const Result parsed = tail.parse(p, n, kKnown, out, capacity, &unknown_critical);
  if (refused(parsed)) return parsed;
  struct Item { uint8_t tag; size_t size; const uint8_t *v; bool critical; };
  Item items[] = {{kTagMode, 1, nullptr, false},     {kTagRate, 4, nullptr, false},     {kTagSamples, 4, nullptr, false},
                  {kTagSegments, 4, nullptr, false},  {kTagTrigger, 6, nullptr, false},  {kTagPretrigger, 4, nullptr, false},
                  {capture_tag::kTriggerOpts, kTriggerOptsBytes, nullptr, false}, {kTagFormat, 1, nullptr, false}};
  Item &mode_tlv = items[0], &rate_tlv = items[1], &samples_tlv = items[2], &segments_tlv = items[3],
       &trigger_tlv = items[4], &pretrigger_tlv = items[5], &opts_tlv = items[6], &format_tlv = items[7];
  for (Item &it : items) {
    const Result r = tail.fixed(it.tag, it.size, it.v, out, capacity, &it.critical);
    if (refused(r)) return r;
  }
  auto ask = [](const Item &it) { return CaptureAsk{it.v, it.critical}; };
  if (captureMalformed(ask(mode_tlv), ask(rate_tlv), ask(samples_tlv), ask(segments_tlv))) return rejected(kRejectMalformed);
  if (refused(unknown_critical)) return unknown_critical;
  if (mode_tlv.v[0] != cap::kModeStreaming) return Tail::refuse(kTagMode, mode_tlv.critical, out, capacity);
  const uint32_t rate = getU32(rate_tlv.v);
  if (rate < kMinHz || rate > kMaxHz) return Tail::refuse(kTagRate, rate_tlv.critical, out, capacity);
  const Result misplaced = captureMisplaced(mode_tlv.v[0], ask(samples_tlv), ask(segments_tlv), ask(trigger_tlv),
                                            ask(pretrigger_tlv), out, capacity);
  if (refused(misplaced)) return misplaced;
  if (trigger_tlv.v && trigger_tlv.v[0] != cap::kTriggerImmediate) return Tail::refuse(kTagTrigger, trigger_tlv.critical, out, capacity);
  TriggerOpts opts;
  if (!triggerOpts(opts_tlv.v, 0, opts)) return Tail::refuse(capture_tag::kTriggerOpts, opts_tlv.critical, out, capacity);
  if (format_tlv.v && format_tlv.v[0] != cap::kDataFormatPacked) return Tail::refuse(kTagFormat, format_tlv.critical, out, capacity);
  CH32RVUhsifClock source;
  uint8_t divider;
  uint32_t num, den;
  if (!rateOf(rate, source, divider, num, den)) return Tail::refuse(kTagRate, rate_tlv.critical, out, capacity);
  const size_t bytes = bufferBytesFor(max_frame_);
  if (!bytes) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);   // the host's frame holds no buffer
  if (channels_ == 0) return wrongState(out, capacity);
  if ((running_ || draining_ || paused_) && !query) return wrongState(out, capacity);
  if (!query) {
    halt();
    if (!ring_ || buffer_bytes_ != bytes || buffers_ != kMaxBuffers) {   // as many buffers as the heap has room for
      free(ring_);
      ring_ = nullptr;
      buffer_bytes_ = 0;
      buffers_ = 0;
      for (uint8_t n = kMaxBuffers; n >= kMinBuffers && !ring_; n /= 2) {
        ring_ = static_cast<uint8_t *>(aligned_alloc(32, n * (kGapBytes + bytes)));
        if (ring_) buffers_ = n;
      }
      if (!ring_) return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
      buffer_bytes_ = bytes;
    }
    source_ = source;
    divider_ = divider;
    num_ = num;
    den_ = den;
    state_ = cap::kStateConfigured;
  }
  TlvWriter w(out, capacity);
  uint8_t r[8];
  putU32(r, num);
  putU32(r + 4, den);
  w.put(cap::kTlvConfigureAnswerActualRate, r, sizeof r);
  uint8_t layout[2 + kMaxChannels] = {16, channels_};
  for (uint8_t k = 0; k < channels_; ++k) layout[2 + k] = static_cast<uint8_t>(line_[k]);
  w.put(cap::kTlvConfigureAnswerLayout, layout, 2 + channels_);
  // the stream's bytes a second (2 a sample, rounded up) and what waits to be sent at most (half the ring: poll drops
  // past it)
  const uint64_t data_rate = (uint64_t{2} * num + den - 1) / den;
  w.u32(cap::kTlvConfigureAnswerDataRate, data_rate > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(data_rate));
  w.u32(cap::kTlvConfigureAnswerStreamBuffer, static_cast<uint32_t>((query && !buffers_ ? kMaxBuffers : buffers_) / 2 * bytes));
  return w.ok() ? completed(w.length()) : failed();
}

Result H41xLogicCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case cap::kOpConfigure: return configure(p, n, out, capacity, false);
    case cap::kOpQuery: return configure(p, n, out, capacity, true);
    case cap::kOpStart: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (state_ != cap::kStateConfigured && state_ != cap::kStateError) return wrongState(out, capacity);
      if (draining_) return wrongState(out, capacity);   // the last stream's data still going out
      if (capacity < 4) return failed();
      halt();
      finished_ = next_ = run_first_ = 0;
      head_sent_ = paused_ = false;
      epoch_b_ = 0;
      epoch_pos_ = 0;
      pauses_ = 0;
      dropped_bytes_ = 0;
      overwritten_ = 0;
      error_ = 0;
      generation_ = nextGeneration(generation_);
      if (!begin()) {
        fail(cap::kErrorPeripheral);
        return failed();
      }
      state_ = cap::kStateCapturing;
      putU32(out, generation_);
      return completed(4);
    }
    case cap::kOpStop: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (running_ || paused_) {
        if (running_) finished_ = finishedNow();   // the buffers finished before the stop still go (capture §2.1 rule 2)
        halt();
        paused_ = false;
        draining_ = true;
        state_ = cap::kStateConfigured;
        stopped(cap::kStoppedReasonHost);
        poll();
      }
      return completed();
    }
    case cap::kOpStatus: {   // -> state(u8) serial_done(u32) write_pos(u64) flags(u8) generation(u32) [TLV]
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 22) return failed();
      poll();
      out[0] = state_;
      putU32(out + 1, finished_);
      // write_pos (capture §3.2): the finished buffers' end; stopped on data written again while it went out, that
      // buffer's start (§2.2: the host drops what it got from there on)
      putU64(out + 5, positionOf(state_ == cap::kStateError && error_ == cap::kErrorStorage ? lost_at_ : finished_));
      out[13] = dropped_bytes_ ? cap::kStatusFlagDropped : 0;
      putU32(out + 14, generation_);
      size_t used = 18;
      if (state_ == cap::kStateError) {
        putTlvHeader(out + 18, cap::kTlvStatusAnswerError, 1);
        out[21] = error_;
        used = 22;
      }
      return completed(used);
    }
    case cap::kOpRead: {   // zero-copy streaming keeps nothing to read back: position = write_pos, gap, len 0 (§2.1 rule 4)
      const Result parsed = plainTail(tail, p, n, 14, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);
      if (capacity < 11) return failed();
      poll();
      putU64(out, positionOf(finished_));
      out[8] = reg::common::kReadFlagsGap;
      putU16(out + 9, 0);
      return completed(11);
    }
    case cap::kOpSegments: {   // streaming sends no segment records (§2.1 rule 3)
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 2) return failed();
      out[0] = 0;
      out[1] = 0;
      return completed(2);
    }
    case cap::kOpRelease: {
      const Result parsed = plainTail(tail, p, n, 8, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);
      return completed();
    }
    case cap::kOpForce: {   // immediate only: nothing to force
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      return completed();
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
