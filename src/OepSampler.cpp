// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepSampler.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32)

#include <esp_cpu.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>
#include <string.h>

#include <algorithm>

#include "OepEndpoint.h"
#include "OepSamplerRun.h"

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

// The chip's side of the sampling loop (OepSamplerRun.h). GPIO0..31 only (kHigh false): one register read per sample.
// Reading GPIO.in1 as well cost enough that 2 MHz fell behind (1.92 MHz actually sampled, periods spread +-5 %,
// 2026-09-29); 1 MHz and below kept pace either way. A sample is channel l on bit l.
template <bool kHigh>
struct EspIo {
  uint32_t lines;
  uint32_t m0[SamplerCapture::kMaxChannels], m1[SamplerCapture::kMaxChannels];
  inline __attribute__((always_inline)) uint32_t cycles() const { return esp_cpu_get_cycle_count(); }
  inline __attribute__((always_inline)) uint8_t read() const {
    const uint32_t in0 = GPIO.in, in1 = kHigh ? GPIO.in1.val : 0;
    uint8_t byte = 0;
    for (uint32_t l = 0; l < lines; ++l)
      byte |= static_cast<uint8_t>(((in0 & m0[l]) | (kHigh ? in1 & m1[l] : 0)) != 0) << l;
    return byte;
  }
  inline __attribute__((always_inline)) void interruptsOff() const { portDISABLE_INTERRUPTS(); }
  inline __attribute__((always_inline)) void interruptsOn() const { portENABLE_INTERRUPTS(); }
  inline __attribute__((always_inline)) uint64_t nowNs() const { return static_cast<uint64_t>(esp_timer_get_time()) * 1000u; }
  inline __attribute__((always_inline)) void yield() const { vTaskDelay(1); }
};

static_assert(sampler::kImmediate == reg::fixture_logic::kTriggerImmediate && sampler::kLevel == reg::fixture_logic::kTriggerLevel &&
                  sampler::kEdge == reg::fixture_logic::kTriggerEdge, "capture §3.3's trigger types");

}  // namespace

// One capture (OepSamplerRun.h: the windows, the bursts of a search and the SWIO wire's turns in them). A sample taken
// one period or more late (the core held up: the other core stalls this one now and then even with interrupts off,
// 2026-09-29; a SWIO frame during a turn) makes the segment slipped (flags bit2, status flags bit1, capture §2.2,
// §3.2) rather than passing a bent time base as even.
// (A template cannot be IRAM_ATTR - its literals land after their use - so it is inlined into runLow / runHigh.)
template <bool kHigh>
inline __attribute__((always_inline)) void SamplerCapture::run() {
  EspIo<kHigh> io;
  io.lines = channels_;
  for (uint32_t l = 0; l < io.lines; ++l) { io.m0[l] = masks0_[l]; io.m1[l] = masks1_[l]; }
  sampler::Plan plan;
  plan.out = buffer_;
  plan.size = samples_;
  plan.step = cycles_;
  plan.cpu_hz = cpu_hz_;
  plan.trig_type = trig_type_;
  plan.trig_role = trig_role_;
  plan.trig_value = static_cast<uint8_t>(trig_value_);
  plan.pre = pretrigger_;
  const uint64_t off_ms = (radio_cfg_ ? kOffNsRadio : kOffNs) / 1000000u;
  const uint64_t limit = static_cast<uint64_t>(cpu_hz_) / 1000u * off_ms / cycles_;
  plan.burst = static_cast<uint32_t>(limit - (samples_ - pretrigger_ - 1));   // > pre: samples < limit (configure)
  plan.turn = static_cast<uint32_t>(static_cast<uint64_t>(cpu_hz_) / 1000u * kWireTurnMs / cycles_);
  plan.span = static_cast<uint32_t>(static_cast<uint64_t>(cpu_hz_) / 1000u * off_ms);   // by the clock (OepSamplerRun.h)
  const sampler::Outcome r = sampler::run(io, plan, control_, [this](uint32_t c, uint32_t kept, uint64_t burst_ns) {
    trig_count_ = c;
    trig_kept_ = kept;
    trig_burst_ns_ = burst_ns;
    trig_seen_ = true;   // poll sends triggered while the rest comes in
  });
  if (r.aborted) { aborted_ = true; return; }
  seg_samples_ = r.samples;
  seg_start_ns_ = r.start_ns;
  if (!trig_type_) start_ns_ = r.start_ns;
  late_cycles_ = r.late_cycles;
  slipped_ = r.slipped;
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
  // a search for the trigger ends at its next sample; a window cannot be cut short (interrupts are off on core 0).
  // The wire is let go of first: a window waiting for it would otherwise wait for this loop, which waits for it.
  gWireGate.release();
  if (sampler_) control_ = sampler::kControlAbort;
  while (sampler_) delay(1);
}

size_t SamplerCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  // role k = channel k (oep-if-capture: roles 0..), each on any of the probe's channels
  uint8_t roles[kMaxChannels];
  for (uint8_t k = 0; k < kMaxChannels; ++k) roles[k] = k;
  w.roleChannels(roles, kMaxChannels, table_.allowedMask());
  // no features: revision 1 defines no bit (query, force, subscribe and unsubscribe are in the ops tag)
  uint8_t mode[9] = {cap::kModeOneShot};           // mode(u8) max_samples(u32) max_segments(u32): one-shot
  putU32(mode + 1, kBufferBytes);                  // one byte per sample (core 0 samples, OEP answers on core 1)
  putU32(mode + 5, 1);
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  uint8_t range[9];
  putU32(range, kMinHz);
  putU32(range + 4, kMaxHz);
  range[8] = 0;                                    // not any value: whole CPU cycles per sample
  w.put(cap::kTlvDescribeRateRange, range, sizeof range);
  w.u8(cap::kTlvDescribeChannels, kMaxChannels);   // max(u8)
  uint8_t trig[8];                                 // types(u32) max_pretrigger(u32)
  putU32(trig, (1u << cap::kTriggerImmediate) | (1u << cap::kTriggerLevel) | (1u << cap::kTriggerEdge));
  putU32(trig + 4, kBufferBytes - 1);              // the pretrigger: less than the segment's samples
  w.put(cap::kTlvDescribeTrigger, trig, sizeof trig);
  return w.ok() ? w.length() : 0;
}

uint8_t SamplerCapture::planCheck(const RoleAssignment *roles, size_t count) {
  uint32_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role >= kMaxChannels) return kRejectUnsupported;   // a role this capture does not have (core §8)
    if (!table_.allowed(roles[i].channel)) return kRejectUnsupported;   // not in role_channels (core §8)
    if (seen & (1u << roles[i].role)) return kRejectUnavailable;
    seen |= 1u << roles[i].role;
  }
  if (count > kMaxChannels) return kRejectUnavailable;   // more channels than it has (a role twice)
  return (count == 0 || seen == (1u << count) - 1) ? 0 : kRejectUnavailable;   // roles 0..count-1, no holes
}

// Taking the plan changes no pin (a logic capture only listens, capture §1.2, core §8): an output idle or another fn's
// output on the channel keeps driving. Only the pad's input buffer is switched on (an ESP32 output pad has it off,
// and GPIO.in then reads 0), again at each start in case something set the pad up since. Nothing to restore at release.
static void listen(const int *pins, uint8_t count) {
  for (uint8_t l = 0; l < count; ++l) if (pins[l] >= 0) gpio_ll_input_enable(&GPIO, static_cast<uint32_t>(pins[l]));
}

bool SamplerCapture::planApply(const RoleAssignment *roles, size_t count) {
  waitIdle();
  for (size_t i = 0; i < count; ++i) pins_[roles[i].role] = roles[i].channel;
  channels_ = static_cast<uint8_t>(count);
  listen(pins_, channels_);
  state_ = cap::kStateUnconfigured;
  return true;
}

void SamplerCapture::planRelease() {
  waitIdle();
  channels_ = 0;
  state_ = cap::kStateUnconfigured;
}

Result SamplerCapture::configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query) {
  // core §2.3: each TLV this configure implements is checked the same with or without bit 7 - a length other than its
  // definition or a value the definition excludes is malformed; a value the definition leaves unused or this probe
  // cannot honour refuses the whole configure unsupported, the tag as received. An unknown critical tag is unsupported,
  // an unknown non-critical one passed over. Everything is checked before anything changes (core §4.3): the form, then
  // the values, then the plan, the group and the state (unavailable).
  static const uint8_t kKnown[] = {kTagMode, kTagRate, kTagSamples, kTagSegments, kTagTrigger, kTagPretrigger};
  Tail tail;
  Result unknown_critical;
  const Result parsed = tail.parse(p, n, kKnown, out, capacity, &unknown_critical);
  if (refused(parsed)) return parsed;
  struct Item { uint8_t tag; size_t size; const uint8_t *v; bool critical; };
  Item items[] = {{kTagMode, 1, nullptr, false},    {kTagRate, 4, nullptr, false},    {kTagSamples, 4, nullptr, false},
                  {kTagSegments, 4, nullptr, false}, {kTagTrigger, 6, nullptr, false}, {kTagPretrigger, 4, nullptr, false}};
  Item &mode_tlv = items[0], &rate_tlv = items[1], &samples_tlv = items[2], &segments_tlv = items[3],
       &trigger_tlv = items[4], &pretrigger_tlv = items[5];
  for (Item &it : items) {   // a length other than its definition: malformed (core §2.3)
    const Result r = tail.fixed(it.tag, it.size, it.v, out, capacity, &it.critical);
    if (refused(r)) return r;
  }
  // capture §3.3's contract: mode and rate required, samples in mode 1 / 2; none of them 0 (malformed)
  auto ask = [](const Item &it) { return CaptureAsk{it.v, it.critical}; };
  if (captureMalformed(ask(mode_tlv), ask(rate_tlv), ask(samples_tlv), ask(segments_tlv))) return rejected(kRejectMalformed);
  if (refused(unknown_critical)) return unknown_critical;
  if (mode_tlv.v[0] != cap::kModeOneShot)   // one-shot only
    return Tail::refuse(kTagMode, mode_tlv.critical, out, capacity);
  uint32_t rate = getU32(rate_tlv.v);   // outside rate_range (capture §3.3)
  if (rate < kMinHz || rate > kMaxHz) return Tail::refuse(kTagRate, rate_tlv.critical, out, capacity);
  // segments outside repeat, a pretrigger without a trigger: unsupported whatever the value
  const Result misplaced = captureMisplaced(mode_tlv.v[0], ask(samples_tlv), ask(segments_tlv), ask(trigger_tlv),
                                            ask(pretrigger_tlv), out, capacity);
  if (refused(misplaced)) return misplaced;
  uint32_t samples = getU32(samples_tlv.v);
  // a role outside the plan is refused unavailable cause 6 below, after every unsupported value (core §4.3)
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
  if (samples > kBufferBytes) samples = kBufferBytes;
  // the radio on: a segment of kSegmentNsRadio at most at the rate paced (below), rounded down - see after the rate
  uint32_t pretrigger = 0;
  if (const uint8_t *v = pretrigger_tlv.v) {   // only with a trigger (above), inside the segment: less than samples
    pretrigger = getU32(v);
    if (pretrigger >= samples) return Tail::refuse(kTagPretrigger, pretrigger_tlv.critical, out, capacity);
  }
  // no plan, or a trigger's role not in it (type 0's role is not looked at): unavailable cause 6 (capture §3.2, §3.3)
  if (channels_ == 0 || (trig_type != 0 && trig_role >= channels_)) return wrongState(out, capacity);
  if (!query && bound()) return boundInGroup(*this, out, capacity);   // the group's now (cause 4)
  if ((state_ == cap::kStateCapturing || state_ == cap::kStateWaiting) && !query) return wrongState(out, capacity);
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
  const bool radio = radio_ && radio_();
  if (radio) {
    const uint64_t most = static_cast<uint64_t>(cpu_hz) / 1000u * (kSegmentNsRadio / 1000000u) / cycles;
    if (samples > most) samples = static_cast<uint32_t>(most);
    if (pretrigger >= samples) return Tail::refuse(kTagPretrigger, pretrigger_tlv.critical, out, capacity);   // inside the segment
  }
  if (!query) {
    waitIdle();
    if (!buffer_) buffer_ = static_cast<uint8_t *>(heap_caps_malloc(kBufferBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!buffer_) return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);   // the resources (core §4.3)
    for (uint8_t l = 0; l < channels_; ++l) {
      masks0_[l] = pins_[l] < 32 ? 1u << pins_[l] : 0;
      masks1_[l] = pins_[l] >= 32 ? 1u << (pins_[l] - 32) : 0;
    }
    samples_ = samples;
    trig_type_ = trig_type;
    trig_role_ = trig_role;
    trig_value_ = trig_value;
    pretrigger_ = pretrigger;
    radio_cfg_ = radio;
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
  w.u32(cap::kTlvConfigureAnswerActualSamples, samples);   // one-shot: no actual_segments (capture §3.3)
  w.u32(cap::kTlvConfigureAnswerBlockingMs, 0);    // core 1 keeps answering
  return w.ok() ? completed(w.length()) : failed();
}

size_t SamplerCapture::segmentInfo(uint8_t *out) const {   // the segment record (oep-if-capture §2), 37 bytes
  putU32(out, 0);                  // serial
  putU64(out + 4, 0);              // position
  putU32(out + 12, seg_samples_);
  putU64(out + 16, trig_type_ ? seg_start_ns_ : start_ns_);
  putU32(out + 24, 2000);          // the software pace's first sample against the clock read: +-2 us
  putU32(out + 28, trig_type_ ? trig_kept_ : 0xFFFFFFFFu);   // immediate: no trigger inside; early: fewer before it
  out[32] = slipped_ ? cap::kSegmentFlagSlipped : 0;   // bit2: a sample taken a period or more late (OepSamplerRun.h)
  putU32(out + 33, generation_);
  return 37;
}

void SamplerCapture::poll() {
  gWireGate.release();   // loop() came round: the SWIO wire's request has ended, a window may open (OepWireGate.h)
  if (state_ == cap::kStateWaiting && trig_seen_) {
    state_ = cap::kStateCapturing;
    if (subscribed_) {   // serial(u32) trigger_index(u32) trigger_ns(u64) generation(u32)
      uint8_t e[20];
      putU32(e, 0);
      putU32(e + 4, trig_kept_);
      putU64(e + 8, trig_burst_ns_ + static_cast<uint64_t>(trig_count_) * cycles_ * 1000000000ull / cpu_hz_);
      putU32(e + 16, generation_);
      endpoint_.event(*this, cap::kEventTriggered, e, sizeof e);
    }
  }
  if (state_ != cap::kStateCapturing || !done_) return;
  state_ = cap::kStateDone;
  if (subscribed_) {
    uint8_t seg[37];
    endpoint_.event(*this, cap::kEventSegment, seg, segmentInfo(seg));
    uint8_t stopped[6] = {cap::kStoppedReasonComplete, 0};   // reason(u8) error(u8) generation(u32) (capture §3.4)
    putU32(stopped + 2, generation_);
    endpoint_.event(*this, cap::kEventStopped, stopped, sizeof stopped);
  }
}

Result SamplerCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  // bound in a capture-group: the group starts and stops it (unavailable cause 4, after the request's form, core §4.3)
  auto boundRefused = [&]() { return bound() && !group_op_; };
  Tail tail;
  switch (op) {
    case cap::kOpConfigure: return configure(p, n, out, capacity, false);
    case cap::kOpQuery: return configure(p, n, out, capacity, true);
    case cap::kOpStart: {   // -> blocking_ms(u32) generation(u32) [TLV]
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (boundRefused()) return boundInGroup(*this, out, capacity);
      if (state_ != cap::kStateConfigured && state_ != cap::kStateDone && state_ != cap::kStateError) return wrongState(out, capacity);
      if (capacity < 8) return failed();
      // the radio came on since configure: its segment and spans are too long for it (configure again)
      if (!radio_cfg_ && radio_ && radio_()) return wrongState(out, capacity);
      waitIdle();
      listen(pins_, channels_);
      done_ = trig_seen_ = aborted_ = false;
      control_ = 0;
      memset(buffer_, 0, samples_);
      start_ns_ = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
      if (xTaskCreatePinnedToCore(samplerTask, "oep_sampler", 4096, this, kTaskPriority, &sampler_, 0) != pdPASS) {
        sampler_ = nullptr;
        state_ = cap::kStateError;   // every entry to state 6 sends stopped reason 3 (capture §3.2)
        if (subscribed_) {
          uint8_t stopped[6] = {cap::kStoppedReasonError, cap::kErrorPeripheral};
          putU32(stopped + 2, generation_);
          endpoint_.event(*this, cap::kEventStopped, stopped, sizeof stopped);
        }
        return failed();
      }
      state_ = trig_type_ ? cap::kStateWaiting : cap::kStateCapturing;
      generation_ = nextGeneration(generation_);   // 1 after 0xFFFFFFFF, never 0 (capture §3.2)
      putU32(out, 0);              // blocking_ms: the probe keeps answering
      putU32(out + 4, generation_);
      return completed(8);
    }
    case cap::kOpStop: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (boundRefused()) return boundInGroup(*this, out, capacity);
      poll();
      if (state_ == cap::kStateWaiting) {   // no trigger yet: nothing captured
        waitIdle();
        poll();                             // it may have come while the search ended
      }
      uint8_t stopped[6] = {cap::kStoppedReasonHost, 0};   // reason(u8) error(u8) generation(u32) (capture §3.4)
      putU32(stopped + 2, generation_);
      if (state_ == cap::kStateWaiting) {
        state_ = cap::kStateConfigured;
        if (subscribed_) endpoint_.event(*this, cap::kEventStopped, stopped, sizeof stopped);
      } else if (state_ == cap::kStateCapturing) {
        // the window cannot be cut short (interrupts are off on core 0) and ends by itself: the capture completes
        // (state 4, its segment and stopped 0 - capture §3.2's "complete -> 4"), not a short segment in state 1
        waitIdle();
        poll();
      }
      return completed();
    }
    case cap::kOpStatus: {   // -> state(u8) serial_done(u32) write_pos(u64) flags(u8) generation(u32) [TLV]
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 21) return failed();
      poll();
      out[0] = state_;
      const bool finished = state_ == cap::kStateDone;
      putU32(out + 1, finished ? 1 : 0);
      putU64(out + 5, finished ? seg_samples_ : 0);
      out[13] = slipped_ ? cap::kStatusFlagSlipped : 0;   // bit1: a sample taken a period or more late (OepSamplerRun.h)
      putU32(out + 14, generation_);
      size_t used = 18;
      if (state_ == cap::kStateError) {   // why (TLV 0x01 error, capture §3.2): the sampling task would not start
        putTlvHeader(out + 18, cap::kTlvStatusAnswerError, 1);
        out[21] = cap::kErrorPeripheral;
        used = 22;
      }
      return completed(used);
    }
    case cap::kOpRead: {   // generation(u32) position(u64) max(u32) -> position(u64) flags(u8) len(u32) data [TLV]
      const Result parsed = plainTail(tail, p, n, 16, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);   // another generation (cause 6)
      if (capacity < 13) return failed();
      poll();
      const uint64_t have = state_ == cap::kStateDone ? seg_samples_ : 0;
      uint64_t position = getU64(p + 4);
      uint32_t max = getU32(p + 12);
      if (position > have) position = have;
      uint32_t count = static_cast<uint32_t>(have - position);
      size_t room = capacity - 13;   // how much read returns is the probe's (capture §3.2): max_read_ and the frame
      if (room > max_read_) room = max_read_;
      if (max > room) max = static_cast<uint32_t>(room);
      uint8_t flags = 0;
      if (count > max) { count = max; flags |= reg::common::kReadFlagsMore; }
      putU64(out, position);
      out[8] = flags;
      putU32(out + 9, count);
      if (count) memcpy(out + 13, buffer_ + position, count);
      return completed(13 + count);
    }
    case cap::kOpSegments: {   // from_serial(u32) -> more(u8) count(u8) count x segment [TLV]
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 3 + 37) return failed();
      poll();
      // common §1.3's serial paging over the one segment (serial 0, once there): every from_serial but next (1) gets it
      const bool one = state_ == cap::kStateDone && serialPageStart(getU32(p), 0, 1) == 0;
      out[0] = 0;
      out[1] = one ? 1 : 0;
      const size_t info = one ? segmentInfo(out + 2) : 0;   // no element length (core §2.3)
      return completed(2u + info);
    }
    case cap::kOpRelease: {   // generation(u32) serial(u32): nothing to release in one-shot (success)
      const Result parsed = plainTail(tail, p, n, 8, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);
      return completed();
    }
    case cap::kOpForce: {          // waiting for the trigger: take the segment from the next sample on
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (boundRefused()) return boundInGroup(*this, out, capacity);
      if (state_ == cap::kStateWaiting) control_ = sampler::kControlForce;
      return completed();
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
