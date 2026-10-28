// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepCh32Logic.h"

#if defined(OEP_LOGIC_CH32)

#include <stdlib.h>
#include <string.h>

#include "OepEndpoint.h"

namespace oep {
namespace {

namespace cap = reg::fixture_logic;
enum : uint8_t { kTagMode = cap::kTlvConfigureMode, kTagRate = cap::kTlvConfigureRate,
                 kTagSamples = cap::kTlvConfigureSamples, kTagSegments = cap::kTlvConfigureSegments,
                 kTagTrigger = cap::kTlvConfigureTrigger, kTagPretrigger = cap::kTlvConfigurePretrigger };

constexpr uint32_t kChannelsPerPort = 24;   // a channel is port x 24 + pin (OepPlatform.h, platformPad)
constexpr uint32_t kSampledPins = 16;       // the core samples a port's low 16 pins
// The DMA's two halves: in RAM the DMA reaches (a static), the core's buffer for the whole run
uint16_t gStage[2 * Ch32LogicCapture::kHalf];

uint32_t gcd(uint32_t a, uint32_t b) {
  while (b) { const uint32_t t = a % b; a = b; b = t; }
  return a;
}

// The divider of F_CPU nearest the rate asked, at or above the one kMaxHz takes
uint32_t dividerOf(uint32_t rate) {
  const uint32_t least = (F_CPU + Ch32LogicCapture::kMaxHz - 1) / Ch32LogicCapture::kMaxHz;
  uint32_t d = (F_CPU + rate / 2) / rate;
  return d < least ? least : d;
}

}  // namespace

size_t Ch32LogicCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  uint8_t roles[kMaxChannels];
  for (uint8_t k = 0; k < kMaxChannels; ++k) roles[k] = k;
  uint64_t sampled = 0;   // every port's pins 0-15
  for (uint32_t c = 0; c < 64; ++c) if (c % kChannelsPerPort < kSampledPins) sampled |= uint64_t{1} << c;
  w.roleChannels(roles, kMaxChannels, table_.allowedMask() & sampled);
  uint8_t mode[9] = {cap::kModeOneShot};
  putU32(mode + 1, kMaxSamples);
  putU32(mode + 5, 1);
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  uint8_t range[8];   // min_hz(u32) max_hz(u32) (capture §3.5)
  putU32(range, kMinHz);
  putU32(range + 4, F_CPU / dividerOf(kMaxHz));
  w.put(cap::kTlvDescribeRateRange, range, sizeof range);
  w.u8(cap::kTlvDescribeChannels, kMaxChannels);
  uint8_t trig[9];
  putTriggerDecl(trig, (1u << cap::kTriggerImmediate) | (1u << cap::kTriggerLevel) | (1u << cap::kTriggerEdge),
                 kMaxSamples - 1, kTriggerOpts);
  w.put(cap::kTlvDescribeTrigger, trig, sizeof trig);
  return w.ok() ? w.length() : 0;
}

// Roles 0 .. n-1, each once, all on one port's pins 0-15 (the core samples one port)
uint8_t Ch32LogicCapture::planCheck(const RoleAssignment *roles, size_t count) {
  plan_cause_ = reg::core::kUnavailableCauseWrongState;
  if (state_ == cap::kStateWaiting || state_ == cap::kStateCapturing) return kRejectUnavailable;
  plan_cause_ = reg::core::kUnavailableCauseLimit;
  uint32_t seen = 0;
  int port = -1;
  for (size_t i = 0; i < count; ++i) {
    const uint16_t ch = roles[i].channel;
    if (roles[i].role >= kMaxChannels || !table_.allowed(ch) || ch % kChannelsPerPort >= kSampledPins)
      return kRejectUnsupported;
    if (port >= 0 && static_cast<uint32_t>(port) != ch / kChannelsPerPort) return kRejectUnsupported;   // one port
    port = static_cast<int>(ch / kChannelsPerPort);
    if (seen & (1u << roles[i].role)) return kRejectUnavailable;
    seen |= 1u << roles[i].role;
  }
  if (count > kMaxChannels) return kRejectUnavailable;
  return (count == 0 || seen == (1u << count) - 1) ? 0 : kRejectUnavailable;
}

// Taking the plan changes no pin: the capture only listens (capture §1.2, core §8)
bool Ch32LogicCapture::planApply(const RoleAssignment *roles, size_t count) {
  halt();
  for (size_t i = 0; i < count; ++i) pins_[roles[i].role] = roles[i].channel;
  channels_ = static_cast<uint8_t>(count);
  port_ = count ? static_cast<uint8_t>(roles[0].channel / kChannelsPerPort) : 0;
  state_ = cap::kStateUnconfigured;
  return true;
}

void Ch32LogicCapture::planRelease() {
  halt();
  channels_ = 0;
  state_ = cap::kStateUnconfigured;
}

void Ch32LogicCapture::halt() {
  logic_.end();
}

bool Ch32LogicCapture::begin() {
  if (!logic_.begin(static_cast<CH32RVLogicPort>(port_), F_CPU / divider_)) return false;
  rate_ = logic_.sampleRate();
  if (!logic_.startContinuous(gStage, 2 * kHalf)) { logic_.end(); return false; }
  overruns_ = logic_.overruns();
  start_ns_ = nowNs();
  return true;
}

void Ch32LogicCapture::fail(uint8_t error) {
  halt();
  state_ = cap::kStateError;
  error_ = error;
  stopped(cap::kStoppedReasonError, error);
}

// One sample: packed (channel l on bit l) into the ring, then - waiting, armed once the pretrigger's samples are in -
// the trigger looked for on it: an edge, the first change after arming (way 0 rising, 1 falling, 2 either); a level,
// the sample into it after one not at it (when 1: one already at it as well); force, this sample.
void Ch32LogicCapture::take(uint16_t sample) {
  uint8_t b = 0;
  for (uint8_t l = 0; l < channels_; ++l)
    b |= static_cast<uint8_t>(((sample >> (pins_[l] % kChannelsPerPort)) & 1u) << l);
  ring_[got_ % samples_] = b;
  const uint32_t k = got_++;
  if (triggered_ || !trig_type_ || k < pretrigger_) return;
  const bool bit = (b >> trig_role_) & 1u;
  bool hit = force_;
  if (!hit && trig_type_ == cap::kTriggerEdge) {
    hit = have_prev_ && bit != prev_ && (trig_value_ == 2 || bit == (trig_value_ == 0));
  } else if (!hit && trig_type_ == cap::kTriggerLevel) {
    const bool at = bit == (trig_value_ != 0);
    hit = at && (armed_ || when_);
    if (!at) armed_ = true;
  }
  prev_ = bit;
  have_prev_ = true;
  if (!hit) return;
  triggered_ = true;
  trig_at_ = k;
  end_ = k + (samples_ - pretrigger_);
  state_ = cap::kStateCapturing;
  if (subscribed_) {   // capture §3.4: no record (rearm 0); the sample from the generation's start
    uint8_t e[kTriggeredBytes];
    putTriggered(e, 0, pretrigger_, k, start_ns_ + k * periodNsOf(), static_cast<uint32_t>(periodNsOf()), generation_);
    endpoint_.event(*this, cap::kEventTriggered, e, sizeof e);
  }
}

// The segment's samples all in: the ring turned to start at its first
void Ch32LogicCapture::finish() {
  halt();
  const uint32_t first = trig_type_ ? trig_at_ - pretrigger_ : 0;
  if (first % samples_) {
    uint8_t *turned = static_cast<uint8_t *>(malloc(samples_));
    if (turned) {
      for (uint32_t i = 0; i < samples_; ++i) turned[i] = ring_[(first + i) % samples_];
      memcpy(ring_, turned, samples_);
      free(turned);
    } else {
      fail(cap::kErrorStorage);
      return;
    }
  }
  seg_start_ns_ = start_ns_ + first * periodNsOf();
  state_ = cap::kStateDone;
  if (subscribed_) {
    uint8_t seg[37];
    endpoint_.event(*this, cap::kEventSegment, seg, segmentInfo(seg));
  }
  stopped(cap::kStoppedReasonComplete);
}

void Ch32LogicCapture::poll() {
  if (state_ != cap::kStateWaiting && state_ != cap::kStateCapturing) return;
  while (const uint16_t *half = logic_.readBuffer()) {
    for (uint32_t i = 0; i < kHalf && got_ < end_; ++i) take(half[i]);
    if (got_ >= end_) break;
  }
  if (logic_.overruns() != overruns_ && got_ < end_) {   // a half lost before it was taken: a hole (capture §2.2)
    fail(cap::kErrorStorage);
    return;
  }
  if (got_ >= end_) finish();
}

Result Ch32LogicCapture::configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query) {
  // as the RP2's (OepRp2Logic.cpp): the form, then the values, then the plan and the state
  static const uint8_t kKnown[] = {kTagMode, kTagRate, kTagSamples, kTagSegments, kTagTrigger, kTagPretrigger,
                                   capture_tag::kTriggerOpts};
  Tail tail;
  Result unknown_critical;
  const Result parsed = tail.parse(p, n, kKnown, out, capacity, &unknown_critical);
  if (refused(parsed)) return parsed;
  struct Item { uint8_t tag; size_t size; const uint8_t *v; bool critical; };
  Item items[] = {{kTagMode, 1, nullptr, false},    {kTagRate, 4, nullptr, false},    {kTagSamples, 4, nullptr, false},
                  {kTagSegments, 4, nullptr, false}, {kTagTrigger, 6, nullptr, false}, {kTagPretrigger, 4, nullptr, false},
                  {capture_tag::kTriggerOpts, kTriggerOptsBytes, nullptr, false}};
  Item &mode_tlv = items[0], &rate_tlv = items[1], &samples_tlv = items[2], &segments_tlv = items[3],
       &trigger_tlv = items[4], &pretrigger_tlv = items[5], &opts_tlv = items[6];
  for (Item &it : items) {
    const Result r = tail.fixed(it.tag, it.size, it.v, out, capacity, &it.critical);
    if (refused(r)) return r;
  }
  auto ask = [](const Item &it) { return CaptureAsk{it.v, it.critical}; };
  if (captureMalformed(ask(mode_tlv), ask(rate_tlv), ask(samples_tlv), ask(segments_tlv))) return rejected(kRejectMalformed);
  if (refused(unknown_critical)) return unknown_critical;
  if (mode_tlv.v[0] != cap::kModeOneShot) return Tail::refuse(kTagMode, mode_tlv.critical, out, capacity);
  const uint32_t rate = getU32(rate_tlv.v);
  if (rate < kMinHz || rate > F_CPU / dividerOf(kMaxHz)) return Tail::refuse(kTagRate, rate_tlv.critical, out, capacity);
  const Result misplaced = captureMisplaced(mode_tlv.v[0], ask(samples_tlv), ask(segments_tlv), ask(trigger_tlv),
                                            ask(pretrigger_tlv), out, capacity);
  if (refused(misplaced)) return misplaced;
  uint32_t samples = getU32(samples_tlv.v);
  uint8_t trig_type = cap::kTriggerImmediate, trig_role = 0;
  uint32_t trig_value = 0;
  if (const uint8_t *v = trigger_tlv.v) {   // type(u8) role(u8) value(u32)
    const uint32_t value = getU32(v + 2);
    const bool ok = v[0] == cap::kTriggerImmediate ||
                    (v[0] == cap::kTriggerLevel && value <= 1) || (v[0] == cap::kTriggerEdge && value <= 2);
    if (!ok) return Tail::refuse(kTagTrigger, trigger_tlv.critical, out, capacity);
    trig_type = v[0];
    trig_role = v[1];
    trig_value = value;
  }
  TriggerOpts opts;
  if (!triggerOpts(opts_tlv.v, kTriggerOpts, opts)) return Tail::refuse(capture_tag::kTriggerOpts, opts_tlv.critical, out, capacity);
  if (samples > kMaxSamples) samples = kMaxSamples;
  uint32_t pretrigger = 0;
  if (const uint8_t *v = pretrigger_tlv.v) {
    pretrigger = getU32(v);
    if (pretrigger >= samples) return Tail::refuse(kTagPretrigger, pretrigger_tlv.critical, out, capacity);
  }
  if (channels_ == 0 || (trig_type != 0 && trig_role >= channels_)) return wrongState(out, capacity);
  if ((state_ == cap::kStateCapturing || state_ == cap::kStateWaiting) && !query) return wrongState(out, capacity);
  const uint32_t divider = dividerOf(rate), g = gcd(F_CPU, divider);
  if (!query) {
    halt();
    if (capacity_ < samples) {
      uint8_t *ring = static_cast<uint8_t *>(realloc(ring_, samples));
      if (!ring) return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
      ring_ = ring;
      capacity_ = samples;
    }
    samples_ = samples;
    trig_type_ = trig_type;
    trig_role_ = trig_role;
    trig_value_ = trig_value;
    pretrigger_ = pretrigger;
    when_ = opts.when;
    divider_ = divider;
    state_ = cap::kStateConfigured;
  }
  TlvWriter w(out, capacity);
  uint8_t r[8];
  putU32(r, F_CPU / g);
  putU32(r + 4, divider / g);
  w.put(cap::kTlvConfigureAnswerActualRate, r, sizeof r);
  uint8_t layout[2 + kMaxChannels] = {8, channels_};
  for (uint8_t k = 0; k < channels_; ++k) layout[2 + k] = k;
  w.put(cap::kTlvConfigureAnswerLayout, layout, 2 + channels_);
  w.u32(cap::kTlvConfigureAnswerActualSamples, samples);
  return w.ok() ? completed(w.length()) : failed();
}

size_t Ch32LogicCapture::segmentInfo(uint8_t *out) const {   // the segment record (oep-if-capture §2), 37 bytes
  putU32(out, 0);
  putU64(out + 4, 0);
  putU32(out + 12, samples_);
  putU64(out + 16, seg_start_ns_);
  putU32(out + 24, static_cast<uint32_t>(periodNsOf() + 20000));   // the run's start against the clock read
  putU32(out + 28, trig_type_ ? pretrigger_ : 0xFFFFFFFFu);
  out[32] = slipped_ ? cap::kSegmentFlagSlipped : 0;
  putU32(out + 33, generation_);
  return 37;
}

void Ch32LogicCapture::stopped(uint8_t reason, uint8_t error) {
  if (!subscribed_) return;
  uint8_t e[6] = {reason, error};
  putU32(e + 2, generation_);
  endpoint_.event(*this, cap::kEventStopped, e, sizeof e);
}

Result Ch32LogicCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case cap::kOpConfigure: return configure(p, n, out, capacity, false);
    case cap::kOpQuery: return configure(p, n, out, capacity, true);
    case cap::kOpStart: {   // -> generation(u32) [TLV]
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (state_ != cap::kStateConfigured && state_ != cap::kStateDone && state_ != cap::kStateError) return wrongState(out, capacity);
      if (capacity < 4) return failed();
      halt();
      got_ = trig_at_ = 0;
      end_ = trig_type_ ? UINT32_MAX : samples_;
      triggered_ = !trig_type_;
      armed_ = have_prev_ = prev_ = force_ = slipped_ = false;
      generation_ = nextGeneration(generation_);
      if (!begin()) {
        fail(cap::kErrorPeripheral);
        return failed();
      }
      state_ = trig_type_ ? cap::kStateWaiting : cap::kStateCapturing;
      putU32(out, generation_);
      return completed(4);
    }
    case cap::kOpStop: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      poll();
      if (state_ == cap::kStateWaiting || state_ == cap::kStateCapturing) {   // no segment: what came in is dropped
        halt();
        state_ = cap::kStateConfigured;
        stopped(cap::kStoppedReasonHost);
      }
      return completed();
    }
    case cap::kOpStatus: {   // -> state(u8) serial_done(u32) write_pos(u64) flags(u8) generation(u32) [TLV]
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 22) return failed();
      poll();
      out[0] = state_;
      const bool done = state_ == cap::kStateDone;
      putU32(out + 1, done ? 1 : 0);
      putU64(out + 5, done ? samples_ : 0);
      out[13] = state_ == cap::kStateError && error_ == cap::kErrorStorage ? cap::kStatusFlagDropped : 0;
      putU32(out + 14, generation_);
      size_t used = 18;
      if (state_ == cap::kStateError) {
        putTlvHeader(out + 18, cap::kTlvStatusAnswerError, 1);
        out[21] = error_;
        used = 22;
      }
      return completed(used);
    }
    case cap::kOpRead: {   // generation(u32) position(u64) max(u16) -> position(u64) flags(u8) len(u16) data [TLV]
      const Result parsed = plainTail(tail, p, n, 14, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);
      if (capacity < 11) return failed();
      poll();
      const uint64_t have = state_ == cap::kStateDone ? samples_ : 0;
      uint64_t position = getU64(p + 4);
      uint32_t max = getU16(p + 12);
      if (position > have) position = have;
      uint32_t count = static_cast<uint32_t>(have - position);
      size_t room = capacity - 11;
      if (room > max_read_) room = max_read_;
      if (max > room) max = static_cast<uint32_t>(room);
      uint8_t flags = 0;
      if (count > max) { count = max; flags |= reg::common::kReadFlagsMore; }
      putU64(out, position);
      out[8] = flags;
      putU16(out + 9, static_cast<uint16_t>(count));
      if (count) memcpy(out + 11, ring_ + position, count);
      return completed(11 + count);
    }
    case cap::kOpSegments: {   // from_serial(u32) -> more(u8) count(u8) count x segment [TLV]
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 3 + 37) return failed();
      poll();
      const bool one = state_ == cap::kStateDone && serialPageStart(getU32(p), 0, 1) == 0;
      out[0] = 0;
      out[1] = one ? 1 : 0;
      const size_t info = one ? segmentInfo(out + 2) : 0;
      return completed(2u + info);
    }
    case cap::kOpRelease: {   // generation(u32) serial(u32): nothing to release in one-shot
      const Result parsed = plainTail(tail, p, n, 8, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);
      return completed();
    }
    case cap::kOpForce: {   // waiting: the trigger at the next sample looked at
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (state_ == cap::kStateWaiting) force_ = true;
      return completed();
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
