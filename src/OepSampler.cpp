// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepSampler.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32)

#include <esp_cpu.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <soc/gpio_struct.h>
#include <string.h>

#include <algorithm>

#include "OepEndpoint.h"

namespace oep {
namespace {

namespace cap = reg::fixture_logic;
enum : uint8_t { kTagMode = cap::kTlvConfigureMode, kTagRate = cap::kTlvConfigureRate,
                 kTagSamples = cap::kTlvConfigureSamples, kTagSegments = cap::kTlvConfigureSegments,
                 kTagTrigger = cap::kTlvConfigureTrigger, kTagPretrigger = cap::kTlvConfigurePretrigger };


uint32_t gcd(uint32_t a, uint32_t b) {
  while (b) { const uint32_t t = a % b; a = b; b = t; }
  return a;
}

}  // namespace

namespace {

// The sampling loop's state, local to the task so that it stays in registers.
struct Pace {
  uint8_t *out;
  uint32_t size, at, step, next, lines;
  int32_t late;
  uint32_t m0[SamplerCapture::kMaxChannels], m1[SamplerCapture::kMaxChannels];
};

// One sample, due at p.next: channel l on bit l, into the buffer used as a ring of p.size.
// A sample due more than one period ago when the loop comes round was taken late: the core was held up (the other
// core stalls this one now and then even with interrupts off, 2026-09-29), and the samples after it catch up faster
// than the rate. The segment says so (flags bit2) rather than passing a bent time base as even.
// GPIO0..31 only (kHigh false): one register read per sample. Reading GPIO.in1 as well cost enough that 2 MHz fell
// behind (1.92 MHz actually sampled, periods spread +-5 %, 2026-09-29); 1 MHz and below kept pace either way.
template <bool kHigh, bool kRing>
inline __attribute__((always_inline)) uint8_t takeSample(Pace &p) {
  const int32_t behind = static_cast<int32_t>(esp_cpu_get_cycle_count() - p.next);
  if (behind > p.late) p.late = behind;
  while (static_cast<int32_t>(esp_cpu_get_cycle_count() - p.next) < 0) {}
  p.next += p.step;
  const uint32_t in0 = GPIO.in, in1 = kHigh ? GPIO.in1.val : 0;
  uint8_t byte = 0;
  for (uint32_t l = 0; l < p.lines; ++l)
    byte |= static_cast<uint8_t>(((in0 & p.m0[l]) | (kHigh ? in1 & p.m1[l] : 0)) != 0) << l;
  p.out[p.at] = byte;
  if (++p.at == p.size && kRing) p.at = 0;   // one window (not a ring): no wrap to pay for
  return byte;
}

}  // namespace

// Immediate: one window, interrupts off throughout (<= 164 ms at the 400 kHz floor, inside the 300 ms interrupt
// watchdog). Triggered: bursts, each with interrupts off for at most kOffNs - the search, then the rest of the
// segment after the trigger - and on between them (a gap: the search starts over, the pretrigger fills again). The
// buffer is a ring during the search; the segment is turned to start at 0 afterwards.
// (A template cannot be IRAM_ATTR - its literals land after their use - so it is inlined into runLow / runHigh.)
template <bool kHigh>
inline __attribute__((always_inline)) void SamplerCapture::run() {
  Pace p = {};
  p.out = buffer_;
  p.size = samples_;
  p.step = cycles_;
  p.lines = channels_;
  for (uint32_t l = 0; l < p.lines; ++l) { p.m0[l] = masks0_[l]; p.m1[l] = masks1_[l]; }
  if (trig_type_ == cap::kTriggerImmediate) {
    portDISABLE_INTERRUPTS();
    p.next = esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < p.size; ++i) takeSample<kHigh, false>(p);
    portENABLE_INTERRUPTS();
    late_cycles_ = static_cast<uint32_t>(p.late);
    slipped_ = p.late > static_cast<int32_t>(p.step);
    return;
  }
  const uint32_t pre = pretrigger_, post = p.size - pre - 1;   // samples after the trigger's one
  const uint64_t limit = static_cast<uint64_t>(cpu_hz_) / 1000u * (kOffNs / 1000000u) / p.step;
  const uint32_t burst = static_cast<uint32_t>(limit - post);  // > pre: samples <= kBufferBytes < limit
  const bool edge = trig_type_ == cap::kTriggerEdge;
  const uint8_t role = trig_role_, value = static_cast<uint8_t>(trig_value_);
  const uint32_t arm = edge && pre == 0 ? 1 : pre;             // an edge needs the sample before it
  for (;;) {
    p.at = 0;
    const uint64_t burst_ns = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
    portDISABLE_INTERRUPTS();
    p.next = esp_cpu_get_cycle_count();
    uint8_t last = 0, stop = 0;
    bool hit = false;
    uint32_t c = 0;
    for (; c < burst; ++c) {
      const uint8_t bit = (takeSample<kHigh, true>(p) >> role) & 1;
      if (c >= arm) {
        hit = edge ? bit != last && (value == 2 || bit == (value == 0 ? 1 : 0)) : bit == value;
        if (hit || (stop = control_) != 0) break;
      }
      last = bit;
    }
    if (hit || (stop & kControlForce)) {
      trig_count_ = c;
      trig_burst_ns_ = burst_ns;
      trig_seen_ = true;                                        // poll sends triggered while the rest comes in
      for (uint32_t i = 0; i < post; ++i) takeSample<kHigh, true>(p);
      portENABLE_INTERRUPTS();
      std::rotate(p.out, p.out + p.at, p.out + p.size);         // the ring's oldest sample (c - pre) first
      seg_start_ns_ = burst_ns + static_cast<uint64_t>(c - pre) * p.step * 1000000000ull / cpu_hz_;
      late_cycles_ = static_cast<uint32_t>(p.late);
      slipped_ = p.late > static_cast<int32_t>(p.step);
      return;
    }
    portENABLE_INTERRUPTS();
    if (stop & kControlAbort) { aborted_ = true; return; }
    vTaskDelay(1);                                              // the core's own tasks (and its watchdog) run
  }
}

void IRAM_ATTR SamplerCapture::runLow() { run<false>(); }
void IRAM_ATTR SamplerCapture::runHigh() { run<true>(); }

void IRAM_ATTR SamplerCapture::samplerTask(void *context) {
  auto *self = static_cast<SamplerCapture *>(context);
  bool high = false;
  for (uint32_t l = 0; l < self->channels_; ++l) high |= self->masks1_[l] != 0;
  if (high) self->runHigh();
  else self->runLow();
  self->done_ = true;
  self->sampler_ = nullptr;
  vTaskDelete(nullptr);
}

void SamplerCapture::waitIdle() {
  // a search for the trigger ends at its next sample; a window cannot be cut short (interrupts are off on core 0)
  if (sampler_) control_ = kControlAbort;
  while (sampler_) delay(1);
}

size_t SamplerCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.u32(kTagFeatures, 0b111);                      // bit0 query, bit1 force, bit2 notifications
  uint8_t mode[10] = {cap::kModeOneShot, 1};       // one-shot, in the background (core 0 samples, OEP on core 1)
  putU32(mode + 2, kBufferBytes);                  // one byte per sample
  putU32(mode + 6, 1);
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  uint8_t range[9];
  putU32(range, kMinHz);
  putU32(range + 4, kMaxHz);
  range[8] = 0;                                    // not any value: whole CPU cycles per sample
  w.put(cap::kTlvDescribeRateRange, range, sizeof range);
  const uint32_t list[] = {400000, 500000, 1000000, 2000000};
  uint8_t l[sizeof list];
  for (size_t i = 0; i < 4; ++i) putU32(l + 4 * i, list[i]);
  w.put(cap::kTlvDescribeRateList, l, sizeof l);
  uint8_t ch[2] = {kMaxChannels, 0b1000};          // w = 8 only
  w.put(cap::kTlvDescribeChannels, ch, sizeof ch);
  uint8_t trig[5] = {(1u << cap::kTriggerImmediate) | (1u << cap::kTriggerLevel) | (1u << cap::kTriggerEdge)};
  putU32(trig + 1, kBufferBytes - 1);              // the pretrigger: less than the segment's samples
  w.put(cap::kTlvDescribeTrigger, trig, sizeof trig);
  w.u32(cap::kTlvDescribeMaxRead, static_cast<uint32_t>(max_read_));
  w.u16(cap::kTlvDescribeSegmentRing, 1);
  return w.ok() ? w.length() : 0;
}

uint8_t SamplerCapture::planCheck(const RoleAssignment *roles, size_t count) {
  if (count > kMaxChannels) return kRejectUnavailable;
  uint32_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role >= kMaxChannels || roles[i].channel >= 40) return kRejectUnavailable;
    if ((reserved_ >> roles[i].channel) & 1) return kRejectUnavailable;
    if (seen & (1u << roles[i].role)) return kRejectUnavailable;
    seen |= 1u << roles[i].role;
  }
  return (count == 0 || seen == (1u << count) - 1) ? 0 : kRejectUnavailable;   // roles 0..count-1, no holes
}

bool SamplerCapture::planApply(const RoleAssignment *roles, size_t count) {
  waitIdle();
  for (size_t i = 0; i < count; ++i) {
    pins_[roles[i].role] = roles[i].channel;
    pinMode(roles[i].channel, INPUT);            // never driven
  }
  channels_ = static_cast<uint8_t>(count);
  state_ = cap::kStateUnconfigured;
  return true;
}

void SamplerCapture::planRelease() {
  waitIdle();
  channels_ = 0;
  state_ = cap::kStateUnconfigured;
}

Result SamplerCapture::configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query) {
  static const uint8_t kKnown[] = {kTagMode, kTagRate, kTagSamples, kTagSegments, kTagTrigger, kTagPretrigger};
  Tail tail;
  const Result parsed = tail.parse(p, n, kKnown, out, capacity);
  if (refused(parsed)) return parsed;
  uint32_t rate = 1000000, samples = 0;
  uint8_t len = 0;
  bool critical = false;
  if (const uint8_t *v = tail.find(kTagMode, len, &critical)) {   // one-shot only
    if (len != 1) return rejected(kRejectMalformed);
    if (v[0] != cap::kModeOneShot) {
      const Result r = tail.refuse(kTagMode, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  if (const uint8_t *v = tail.find(kTagRate, len, &critical)) {
    if (len != 4) return rejected(kRejectMalformed);
    if (getU32(v) >= kMinHz && getU32(v) <= kMaxHz) rate = getU32(v);
    else {
      const Result r = tail.refuse(kTagRate, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  if (const uint8_t *v = tail.find(kTagSamples, len)) {
    if (len != 4) return rejected(kRejectMalformed);
    samples = getU32(v);
  }
  if (tail.find(kTagSegments, len) && len != 4) return rejected(kRejectMalformed);
  if (channels_ == 0) return rejected(kRejectUnavailable);   // plan first
  uint8_t trig_type = cap::kTriggerImmediate, trig_role = 0;
  uint16_t trig_value = 0;
  if (const uint8_t *v = tail.find(kTagTrigger, len, &critical)) {   // type(u8) role(u8) value(u16)
    if (len != 4) return rejected(kRejectMalformed);
    const uint16_t value = static_cast<uint16_t>(v[2] | v[3] << 8);
    const bool ok = v[0] == cap::kTriggerImmediate ||
                    (v[1] < channels_ && ((v[0] == cap::kTriggerLevel && value <= 1) || (v[0] == cap::kTriggerEdge && value <= 2)));
    if (ok) { trig_type = v[0]; trig_role = v[1]; trig_value = value; }
    else {
      const Result r = tail.refuse(kTagTrigger, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  if (state_ == cap::kStateCapturing && !query) return rejected(kRejectBusy);
  if (samples == 0 || samples > kBufferBytes) samples = kBufferBytes;
  uint32_t pretrigger = 0;
  if (const uint8_t *v = tail.find(kTagPretrigger, len, &critical)) {   // only with a trigger, inside the segment
    if (len != 4) return rejected(kRejectMalformed);
    pretrigger = getU32(v);
    if (pretrigger && (trig_type == cap::kTriggerImmediate || pretrigger >= samples)) {
      const Result r = tail.refuse(kTagPretrigger, critical, out, capacity);
      if (refused(r)) return r;
      pretrigger = 0;
    }
  }
  // Paced in software, the loop keeps up to kMaxHz only while every channel is on GPIO0..31 (one register read per
  // sample); with GPIO32..39 in the plan it keeps 1 MHz, not 2 (1.52 MHz actually sampled, 2026-09-29). The answer
  // carries the rate that is really paced.
  bool high = false;
  for (uint8_t l = 0; l < channels_; ++l) high |= pins_[l] >= 32;
  if (high && rate > kMaxHzHighBank) rate = kMaxHzHighBank;
  const uint32_t cpu_hz = getCpuFrequencyMhz() * 1000000u;
  const uint32_t cycles = cpu_hz / rate;
  const uint32_t g = gcd(cpu_hz, cycles);
  const uint32_t num = cpu_hz / g, den = cycles / g;   // the rate actually paced: whole cycles per sample
  if (!query) {
    waitIdle();
    if (!buffer_) buffer_ = static_cast<uint8_t *>(heap_caps_malloc(kBufferBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!buffer_) return failed();
    for (uint8_t l = 0; l < channels_; ++l) {
      masks0_[l] = pins_[l] < 32 ? 1u << pins_[l] : 0;
      masks1_[l] = pins_[l] >= 32 ? 1u << (pins_[l] - 32) : 0;
    }
    samples_ = samples;
    trig_type_ = trig_type;
    trig_role_ = trig_role;
    trig_value_ = trig_value;
    pretrigger_ = pretrigger;
    cycles_ = cycles;
    cpu_hz_ = cpu_hz;
    state_ = cap::kStateConfigured;
  }
  TlvWriter w(out, capacity);
  uint8_t r[8];
  putU32(r, num);
  putU32(r + 4, den);
  w.put(cap::kTlvConfigureAnswerActualRate, r, sizeof r);
  uint8_t layout[2 + kMaxChannels] = {8, channels_};
  for (uint8_t k = 0; k < channels_; ++k) layout[2 + k] = k;
  w.put(cap::kTlvConfigureAnswerLayout, layout, 2 + channels_);
  w.u32(cap::kTlvConfigureAnswerActualSamples, samples);
  w.u32(cap::kTlvConfigureAnswerActualSegments, 1);
  uint8_t timing[5] = {2};                         // software pacing: the cycle counter, polled
  putU32(timing + 1, 1000000000u / cpu_hz * 8 + 1);   // a few cycles of polling, rounded up (ns)
  w.put(cap::kTlvConfigureAnswerTiming, timing, sizeof timing);
  w.u32(cap::kTlvConfigureAnswerBlockingMs, 0);    // core 1 keeps answering
  return w.ok() ? tail.finish(completed(w.length()), out, capacity) : failed();
}

size_t SamplerCapture::segmentInfo(uint8_t *out) const {
  putU32(out, 0);                  // serial
  putU64(out + 4, 0);              // position
  putU32(out + 12, samples_);
  putU64(out + 16, trig_type_ ? seg_start_ns_ : start_ns_);
  putU32(out + 24, 2000);          // the software pace's first sample against the clock read: +-2 us
  putU32(out + 28, trig_type_ ? pretrigger_ : 0xFFFFFFFFu);   // immediate: no trigger inside
  out[32] = slipped_ ? cap::kSegmentFlagSlipped : 0;   // bit2: the time base bent (see samplerTask)
  return 33;
}

void SamplerCapture::poll() {
  if (state_ == cap::kStateWaiting && trig_seen_) {
    state_ = cap::kStateCapturing;
    if (subscribed_) {   // serial(u32) trigger_index(u32) trigger_ns(u64)
      uint8_t e[16];
      putU32(e, 0);
      putU32(e + 4, pretrigger_);
      putU64(e + 8, trig_burst_ns_ + static_cast<uint64_t>(trig_count_) * cycles_ * 1000000000ull / cpu_hz_);
      endpoint_.event(*this, cap::kEventTriggered, e, sizeof e);
    }
  }
  if (state_ != cap::kStateCapturing || !done_) return;
  state_ = cap::kStateDone;
  if (subscribed_) {
    uint8_t seg[33];
    endpoint_.event(*this, cap::kEventSegment, seg, segmentInfo(seg));
    const uint8_t reason = cap::kStoppedReasonComplete;
    endpoint_.event(*this, cap::kEventStopped, &reason, 1);
  }
}

Result SamplerCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  if (bound() && !group_op_ && (op == cap::kOpConfigure || op == cap::kOpStart || op == cap::kOpStop || op == cap::kOpForce))
    return rejected(kRejectUnavailable);   // bound in a capture-group: the group starts and stops it
  Tail tail;
  switch (op) {
    case cap::kOpConfigure: return configure(p, n, out, capacity, false);
    case cap::kOpQuery: return configure(p, n, out, capacity, true);
    case cap::kOpStart: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (state_ != cap::kStateConfigured && state_ != cap::kStateDone) return rejected(kRejectUnavailable);
      if (capacity < 4) return failed();
      waitIdle();
      done_ = trig_seen_ = aborted_ = false;
      control_ = 0;
      memset(buffer_, 0, samples_);
      start_ns_ = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
      if (xTaskCreatePinnedToCore(samplerTask, "oep_sampler", 4096, this, configMAX_PRIORITIES - 1, &sampler_, 0) != pdPASS) {
        sampler_ = nullptr;
        state_ = cap::kStateError;
        return failed();
      }
      state_ = trig_type_ ? cap::kStateWaiting : cap::kStateCapturing;
      putU32(out, 0);              // blocking_ms: the probe keeps answering
      return tail.finish(completed(4), out, capacity);
    }
    case cap::kOpStop: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      poll();
      if (state_ == cap::kStateWaiting) {   // no trigger yet: nothing captured
        waitIdle();
        poll();                             // it may have come while the search ended
      }
      if (state_ == cap::kStateWaiting) {
        state_ = cap::kStateConfigured;
        if (subscribed_) { const uint8_t reason = cap::kStoppedReasonHost; endpoint_.event(*this, cap::kEventStopped, &reason, 1); }
      } else if (state_ == cap::kStateCapturing) {
        waitIdle();                // the window ends by itself; its data stay readable
        state_ = cap::kStateDone;
        if (subscribed_) { const uint8_t reason = cap::kStoppedReasonHost; endpoint_.event(*this, cap::kEventStopped, &reason, 1); }
      }
      return tail.finish(completed(), out, capacity);
    }
    case cap::kOpStatus: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 14) return failed();
      poll();
      out[0] = state_;
      const bool finished = state_ == cap::kStateDone;
      putU32(out + 1, finished ? 1 : 0);
      putU64(out + 5, finished ? samples_ : 0);
      out[13] = slipped_ ? 2 : 0;   // bit1 the time base bent (a sample taken late)
      return tail.finish(completed(14), out, capacity);
    }
    case cap::kOpRead: {           // position(u64) max(u32) -> position(u64) flags data (closed tail)
      if (n < 12) return rejected(kRejectMalformed);
      const Result parsed = plainTail(tail, p, n, 12, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 9) return failed();
      poll();
      const uint64_t have = state_ == cap::kStateDone ? samples_ : 0;
      uint64_t position = getU64(p);
      uint32_t max = getU32(p + 8);
      if (position > have) position = have;
      uint32_t count = static_cast<uint32_t>(have - position);
      size_t room = capacity - 9;
      if (room > max_read_) room = max_read_;
      if (max > room) max = static_cast<uint32_t>(room);
      uint8_t flags = 0;
      if (count > max) { count = max; flags |= 1; }   // more
      putU64(out, position);
      out[8] = flags;
      if (count) memcpy(out + 9, buffer_ + position, count);
      return completed(9 + count);
    }
    case cap::kOpSegments: {
      if (n < 4) return rejected(kRejectMalformed);
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 2 + 33) return failed();
      poll();
      const bool one = state_ == cap::kStateDone && getU32(p) == 0;
      out[0] = one ? 1 : 0;
      if (one) out[1] = static_cast<uint8_t>(segmentInfo(out + 2));   // len(u8) then the info (core §2.3)
      return tail.finish(completed(one ? 2u + out[1] : 1u), out, capacity);
    }
    case cap::kOpForce: {          // waiting for the trigger: take the segment from the next sample on
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (state_ == cap::kStateWaiting) control_ = kControlForce;
      return tail.finish(completed(), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);   // release: not in one-shot
  }
}

}  // namespace oep

#endif
