#include "OepV1Sampler.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32)

#include <esp_cpu.h>
#include <esp_heap_caps.h>
#include <soc/gpio_struct.h>
#include <string.h>

#include "OepV1Endpoint.h"

namespace oep {
namespace v1 {
namespace {

namespace cap = reg::fixture_capture;
enum : uint8_t { kTagMode = cap::kTlvConfigureMode, kTagRate = cap::kTlvConfigureRate,
                 kTagSamples = cap::kTlvConfigureSamples, kTagSegments = cap::kTlvConfigureSegments,
                 kTagTrigger = cap::kTlvConfigureTrigger, kTagPretrigger = cap::kTlvConfigurePretrigger };

inline bool refused(const Result &r) { return r.resolution != kResolutionCompleted; }

uint32_t gcd(uint32_t a, uint32_t b) {
  while (b) { const uint32_t t = a % b; a = b; b = t; }
  return a;
}

}  // namespace

// Interrupts stay off for the whole window (<= 164 ms at the 400 kHz floor), inside the 300 ms interrupt watchdog.
void IRAM_ATTR SamplerCapture::samplerTask(void *context) {
  auto *self = static_cast<SamplerCapture *>(context);
  uint8_t *out = self->buffer_;
  const uint32_t n = self->samples_, step = self->cycles_, lines = self->channels_;
  uint32_t m0[kMaxChannels], m1[kMaxChannels];
  for (uint32_t l = 0; l < lines; ++l) { m0[l] = self->masks0_[l]; m1[l] = self->masks1_[l]; }
  portDISABLE_INTERRUPTS();
  uint32_t next = esp_cpu_get_cycle_count();
  for (uint32_t i = 0; i < n; ++i) {
    while (static_cast<int32_t>(esp_cpu_get_cycle_count() - next) < 0) {}
    next += step;
    const uint32_t in0 = GPIO.in, in1 = GPIO.in1.val;
    uint8_t byte = 0;
    for (uint32_t l = 0; l < lines; ++l) byte |= static_cast<uint8_t>(((in0 & m0[l]) | (in1 & m1[l])) != 0) << l;
    out[i] = byte;
  }
  portENABLE_INTERRUPTS();
  self->done_ = true;
  self->sampler_ = nullptr;
  vTaskDelete(nullptr);
}

void SamplerCapture::waitIdle() {
  while (sampler_) delay(1);   // a window cannot be cut short (interrupts are off on core 0); it ends by itself
}

size_t SamplerCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.u32(kTagFeatures, 0b101);                      // bit0 query, bit2 notifications (no force: immediate trigger only)
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
  uint8_t trig[5] = {0b1};                         // immediate only; no pretrigger
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
  if (const uint8_t *v = tail.find(kTagSegments, len)) {
    if (len != 4) return rejected(kRejectMalformed);
  }
  if (const uint8_t *v = tail.find(kTagTrigger, len, &critical)) {
    if (len != 4) return rejected(kRejectMalformed);
    if (v[0] != cap::kTriggerImmediate) {
      const Result r = tail.refuse(kTagTrigger, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  if (const uint8_t *v = tail.find(kTagPretrigger, len, &critical)) {
    if (len != 4) return rejected(kRejectMalformed);
    if (getU32(v)) {
      const Result r = tail.refuse(kTagPretrigger, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  if (channels_ == 0) return rejected(kRejectUnavailable);   // plan first
  if (state_ == cap::kStateCapturing && !query) return rejected(kRejectBusy);
  if (samples == 0 || samples > kBufferBytes) samples = kBufferBytes;
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
  putU32(out + 4, 0);              // position
  putU32(out + 8, samples_);
  putU32(out + 12, start_us_);
  putU32(out + 16, 0xFFFFFFFFu);   // no trigger inside (immediate start)
  out[20] = 0;
  return 21;
}

void SamplerCapture::poll() {
  if (state_ != cap::kStateCapturing || !done_) return;
  state_ = cap::kStateDone;
  if (subscribed_) {
    uint8_t seg[21];
    endpoint_.event(*this, cap::kEventSegment, seg, segmentInfo(seg));
    const uint8_t reason = cap::kStoppedReasonComplete;
    endpoint_.event(*this, cap::kEventStopped, &reason, 1);
  }
}

Result SamplerCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
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
      done_ = false;
      memset(buffer_, 0, samples_);
      start_us_ = micros();
      if (xTaskCreatePinnedToCore(samplerTask, "oep_sampler", 4096, this, configMAX_PRIORITIES - 1, &sampler_, 0) != pdPASS) {
        sampler_ = nullptr;
        state_ = cap::kStateError;
        return failed();
      }
      state_ = cap::kStateCapturing;
      putU32(out, 0);              // blocking_ms: the probe keeps answering
      return tail.finish(completed(4), out, capacity);
    }
    case cap::kOpStop: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (state_ == cap::kStateCapturing) {
        waitIdle();                // the window ends by itself; its data stay readable
        state_ = cap::kStateDone;
        if (subscribed_) { const uint8_t reason = cap::kStoppedReasonHost; endpoint_.event(*this, cap::kEventStopped, &reason, 1); }
      }
      return tail.finish(completed(), out, capacity);
    }
    case cap::kOpStatus: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 10) return failed();
      poll();
      out[0] = state_;
      const bool finished = state_ == cap::kStateDone;
      putU32(out + 1, finished ? 1 : 0);
      putU32(out + 5, finished ? samples_ : 0);
      out[9] = 0;
      return tail.finish(completed(10), out, capacity);
    }
    case cap::kOpRead: {           // position(u32) max(u32) -> position flags data (closed tail)
      if (n < 8) return rejected(kRejectMalformed);
      const Result parsed = plainTail(tail, p, n, 8, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 5) return failed();
      poll();
      const uint32_t have = state_ == cap::kStateDone ? samples_ : 0;
      uint32_t position = getU32(p), max = getU32(p + 4);
      if (position > have) position = have;
      uint32_t count = have - position;
      size_t room = capacity - 5;
      if (room > max_read_) room = max_read_;
      if (max > room) max = static_cast<uint32_t>(room);
      uint8_t flags = 0;
      if (count > max) { count = max; flags |= 1; }   // more
      putU32(out, position);
      out[4] = flags;
      if (count) memcpy(out + 5, buffer_ + position, count);
      return completed(5 + count);
    }
    case cap::kOpSegments: {
      if (n < 4) return rejected(kRejectMalformed);
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 1 + 21) return failed();
      poll();
      const bool one = state_ == cap::kStateDone && getU32(p) == 0;
      out[0] = one ? 1 : 0;
      return tail.finish(completed(1 + (one ? segmentInfo(out + 1) : 0)), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);   // force, release: not in one-shot with an immediate trigger
  }
}

}  // namespace v1
}  // namespace oep

#endif
