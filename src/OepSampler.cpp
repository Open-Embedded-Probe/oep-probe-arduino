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
// 2026-09-29); 1 MHz and below kept pace either way (rate_range ends at 300 kHz: SamplerCapture::kMaxHz). A sample is
// channel l on bit l, packed by table (sampler::Packer).
template <bool kHigh>
struct EspIo {
  const sampler::Packer *packer;
  inline __attribute__((always_inline)) uint32_t cycles() const { return esp_cpu_get_cycle_count(); }
  inline __attribute__((always_inline)) uint8_t read() const {
    return kHigh ? packer->pack(GPIO.in, GPIO.in1.val) : packer->pack(GPIO.in);
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
  io.packer = &packer_;
  if (clocked_) {   // the external clock: one window, a sample at each edge (OepSamplerRun.h runClocked)
    sampler::ClockPlan plan;
    plan.out = out_;
    plan.size = samples_;
    plan.edge = clock_edge_;
    plan.qual_bit = qual_bit_;
    plan.qual_level = qual_level_;
    plan.timed = timed_;
    plan.span = static_cast<uint32_t>(static_cast<uint64_t>(cpu_hz_) / 1000u * (kWindowNs / 1000000u));
    const sampler::ClockOutcome r = sampler::runClocked(io, plan, control_);
    if (r.aborted) { aborted_ = true; return; }
    seg_samples_ = r.samples;
    // the first edge's time; format 3's first delta counted from the start, not the window's (capture §3.8)
    const uint64_t lead = r.window_ns > start_ns_ ? (r.window_ns - start_ns_) * (cpu_hz_ / 1000000u) / 1000u : 0;
    seg_start_ns_ = r.window_ns + static_cast<uint64_t>(r.first_cycles) * 1000u / (cpu_hz_ / 1000000u);
    if (timed_ && r.samples) putU32(out_, getU32(out_) + static_cast<uint32_t>(lead));
    slipped_ = false;
    return;
  }
  sampler::Plan plan;
  plan.out = out_;
  plan.size = samples_;
  plan.step = cycles_;
  plan.cpu_hz = cpu_hz_;
  plan.trig_type = trig_type_;
  plan.trig_role = trig_role_;
  plan.trig_value = static_cast<uint8_t>(trig_value_);
  plan.pre = pretrigger_;
  plan.arm = arm_;
  plan.when = when_;
  const uint64_t off_ms = kOffNs / 1000000u;
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
  // a search for the trigger ends at its next sample; a window cannot be cut short (interrupts are off on the sampler's core).
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
  putU32(mode + 1, kBufferBytes);                  // one byte per sample (one core samples, OEP answers on the other)
  putU32(mode + 5, 1);
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  mode[0] = cap::kModeRepeat;                      // rearm 1 only (configure refuses rearm 0): the slots share the buffer
  putU32(mode + 1, kBufferBytes / 2);
  putU32(mode + 5, kMaxSegments);
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  uint8_t range[8];   // min_hz(u32) max_hz(u32) (capture §3.5)
  putU32(range, kMinHz);
  putU32(range + 4, kMaxHz);
  w.put(cap::kTlvDescribeRateRange, range, sizeof range);
  w.u8(cap::kTlvDescribeChannels, kMaxChannels);   // max(u8)
  uint8_t trig[9];                                 // types(u32) max_pretrigger(u32) opts(u8)
  putTriggerDecl(trig, (1u << cap::kTriggerImmediate) | (1u << cap::kTriggerLevel) | (1u << cap::kTriggerEdge),
                 kBufferBytes - 1, kTriggerOpts);  // the pretrigger: less than the segment's samples
  w.put(cap::kTlvDescribeTrigger, trig, sizeof trig);
  w.u16(cap::kTlvDescribeTriggerRecords, kTriggerRecords);   // rearm 1's records kept (§3.10)
  const uint8_t formats[] = {cap::kDataFormatStateTimed};    // format 3, on the external clock (§3.8)
  w.put(cap::kTlvDescribeFormats, formats, sizeof formats);
  uint8_t clock[6];   // external_clock: max_hz(u32) edges(u8: rising, falling, both) qualifier(u8)
  putU32(clock, kExtClockMaxHz);
  clock[4] = 0b111;
  clock[5] = 1;
  w.put(cap::kTlvDescribeExternalClock, clock, sizeof clock);
  return w.ok() ? w.length() : 0;
}

uint8_t SamplerCapture::planCheck(const RoleAssignment *roles, size_t count) {
  // capture §3.2: a changed plan while waiting, capturing or paused (2 / 3 / 5): cause 6
  plan_cause_ = reg::core::kUnavailableCauseWrongState;
  if (state_ == reg::fixture_logic::kStateWaiting || state_ == reg::fixture_logic::kStateCapturing || state_ == reg::fixture_logic::kStatePaused) return kRejectUnavailable;
  plan_cause_ = reg::core::kUnavailableCauseLimit;
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
  static const uint8_t kKnown[] = {kTagMode, kTagRate, kTagSamples, kTagSegments, kTagTrigger, kTagPretrigger,
                                   capture_tag::kTriggerOpts, cap::kTlvConfigureClock, cap::kTlvConfigureFormat};
  Tail tail;
  Result unknown_critical;
  const Result parsed = tail.parse(p, n, kKnown, out, capacity, &unknown_critical);
  if (refused(parsed)) return parsed;
  struct Item { uint8_t tag; size_t size; const uint8_t *v; bool critical; };
  Item items[] = {{kTagMode, 1, nullptr, false},    {kTagRate, 4, nullptr, false},    {kTagSamples, 4, nullptr, false},
                  {kTagSegments, 4, nullptr, false}, {kTagTrigger, 6, nullptr, false}, {kTagPretrigger, 4, nullptr, false},
                  {capture_tag::kTriggerOpts, kTriggerOptsBytes, nullptr, false}, {cap::kTlvConfigureClock, 4, nullptr, false},
                  {cap::kTlvConfigureFormat, 1, nullptr, false}};
  Item &mode_tlv = items[0], &rate_tlv = items[1], &samples_tlv = items[2], &segments_tlv = items[3],
       &trigger_tlv = items[4], &pretrigger_tlv = items[5], &opts_tlv = items[6], &clock_tlv = items[7],
       &format_tlv = items[8];
  for (Item &it : items) {   // a length other than its definition: malformed (core §2.3)
    const Result r = tail.fixed(it.tag, it.size, it.v, out, capacity, &it.critical);
    if (refused(r)) return r;
  }
  // capture §3.3's contract: mode and rate required, samples in mode 1 / 2; none of them 0 (malformed)
  auto ask = [](const Item &it) { return CaptureAsk{it.v, it.critical}; };
  if (captureMalformed(ask(mode_tlv), ask(rate_tlv), ask(samples_tlv), ask(segments_tlv))) return rejected(kRejectMalformed);
  if (refused(unknown_critical)) return unknown_critical;
  const uint8_t mode = mode_tlv.v[0];
  if (mode != cap::kModeOneShot && mode != cap::kModeRepeat) return Tail::refuse(kTagMode, mode_tlv.critical, out, capacity);
  // the external clock (§3.8): role edge qualifier_role(0xFF none) qualifier_level; a one-shot with an immediate trigger
  const uint8_t *ext = clock_tlv.v;
  if (ext && (ext[1] > 2 || ext[3] > 1 || ext[2] == ext[0] || mode != cap::kModeOneShot ||
              (trigger_tlv.v && trigger_tlv.v[0] != cap::kTriggerImmediate)))
    return Tail::refuse(cap::kTlvConfigureClock, clock_tlv.critical, out, capacity);
  // the data format: 0, or 3 (the timed states) on the external clock
  const uint8_t format = format_tlv.v ? format_tlv.v[0] : cap::kDataFormatPacked;
  if (format != cap::kDataFormatPacked && !(format == cap::kDataFormatStateTimed && ext))
    return Tail::refuse(cap::kTlvConfigureFormat, format_tlv.critical, out, capacity);
  uint32_t rate = getU32(rate_tlv.v);   // outside rate_range (capture §3.3); with the clock the fastest that may come
  if (ext ? rate > kExtClockMaxHz : (rate < kMinHz || rate > kMaxHz))
    return Tail::refuse(kTagRate, rate_tlv.critical, out, capacity);
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
  TriggerOpts opts;   // arm 0 and when 1 as well as the defaults (kTriggerOpts)
  if (!triggerOpts(opts_tlv.v, kTriggerOpts, opts)) return Tail::refuse(capture_tag::kTriggerOpts, opts_tlv.critical, out, capacity);
  // repeat: every segment a window of its own trigger, its pretrigger in first (arm 1: every segment samples long);
  // gapless segments (rearm 0, no trigger) a window at a time cannot give
  if (mode == cap::kModeRepeat && (trig_type == cap::kTriggerImmediate || !opts.rearm || !opts.arm))
    return Tail::refuse(kTagMode, mode_tlv.critical, out, capacity);
  // The answer carries the rate that is really paced (whole cycles a sample). GPIO32..39 in the plan add a register read
  // a sample: it kept 1 MHz (1.52 MHz actually sampled at 2 MHz, 2026-09-29), far above kMaxHz.
  const uint32_t cpu_hz = getCpuFrequencyMhz() * 1000000u;
  const uint32_t cycles = cpu_hz / rate;
  const uint32_t per = format == cap::kDataFormatStateTimed ? 5 : 1;   // delta(u32) and the byte, or the byte
  if (samples > kBufferBytes / per) samples = kBufferBytes / per;
  // A window lasts samples x the period with interrupts off on the sampling core, and every such span ends by the
  // clock at kOffNs: samples are rounded down to what kWindowNs holds at this rate (actual_samples, capture §3.3), so a
  // window is never cut short by the clock at its nominal pace and a search keeps kOffNs - kWindowNs or more of a
  // burst. At kMaxHz that is the whole buffer.
  const uint64_t window = static_cast<uint64_t>(cpu_hz) / 1000u * (kWindowNs / 1000000u) / cycles;
  if (samples > window) samples = static_cast<uint32_t>(window);
  uint32_t segs = 1;
  if (mode == cap::kModeRepeat) {   // the slots share the buffer
    if (samples > kBufferBytes / 2) samples = kBufferBytes / 2;
    segs = segments_tlv.v ? getU32(segments_tlv.v) : kMaxSegments;
    if (segs > kMaxSegments) segs = kMaxSegments;
    if (segs > kBufferBytes / samples) segs = static_cast<uint32_t>(kBufferBytes / samples);
    if (segs < 1) segs = 1;
  }
  uint32_t pretrigger = 0;
  if (const uint8_t *v = pretrigger_tlv.v) {   // only with a trigger (above), inside the segment: less than samples
    pretrigger = getU32(v);
    if (pretrigger >= samples) return Tail::refuse(kTagPretrigger, pretrigger_tlv.critical, out, capacity);
  }
  // no plan, or a trigger's role not in it (type 0's role is not looked at): unavailable cause 6 (capture §3.2, §3.3)
  if (channels_ == 0 || (trig_type != 0 && trig_role >= channels_)) return wrongState(out, capacity);
  if (ext && (ext[0] >= channels_ || (ext[2] != 0xFF && ext[2] >= channels_))) return wrongState(out, capacity);
  const uint8_t lines = ext ? channels_ - 1 : channels_;   // the clock's role is no sample line
  if (ext && (lines == 0 || lines > 7)) return Tail::refuse(cap::kTlvConfigureClock, clock_tlv.critical, out, capacity);
  if (!query && bound()) return boundInGroup(*this, out, capacity);   // the group's now (cause 4)
  if ((state_ == cap::kStateCapturing || state_ == cap::kStateWaiting) && !query) return wrongState(out, capacity);
  const uint32_t g = gcd(cpu_hz, cycles);
  const uint32_t num = cpu_hz / g, den = cycles / g;   // the rate actually paced: whole cycles per sample
  if (!query) {
    waitIdle();
    if (!buffer_) buffer_ = static_cast<uint8_t *>(heap_caps_malloc(kBufferBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!buffer_) return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);   // the resources (core §4.3)
    for (uint8_t l = 0; l < channels_; ++l) {
      masks0_[l] = pins_[l] < 32 ? 1u << pins_[l] : 0;
      masks1_[l] = pins_[l] >= 32 ? 1u << (pins_[l] - 32) : 0;
    }
    clocked_ = ext != nullptr;
    timed_ = format == cap::kDataFormatStateTimed;
    lines_ = lines;
    if (ext) {   // the data lines in role order, the clock's line on bit 7 (sampler::kClockBit)
      int packed[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
      uint8_t l = 0;
      for (uint8_t k = 0; k < channels_; ++k) {
        if (k == ext[0]) continue;
        if (k == ext[2]) qual_bit_ = static_cast<uint8_t>(1u << l);
        packed[l++] = pins_[k];
      }
      packed[7] = pins_[ext[0]];
      if (ext[2] == 0xFF) qual_bit_ = 0;
      clock_edge_ = ext[1];
      qual_level_ = ext[3];
      packer_.build(packed, 8);
    } else {
      packer_.build(pins_, channels_);
    }
    samples_ = samples;
    mode_ = mode;
    seg_count_ = segs;
    trig_type_ = trig_type;
    trig_role_ = trig_role;
    trig_value_ = trig_value;
    pretrigger_ = pretrigger;
    arm_ = opts.arm;
    when_ = opts.when;
    cycles_ = cycles;
    cpu_hz_ = cpu_hz;
    state_ = cap::kStateConfigured;
  }
  TlvWriter w(out, capacity);
  uint8_t r[8];
  putU32(r, ext ? 0 : num);   // the clock's: 0 / 1, no time (§3.8)
  putU32(r + 4, ext ? 1 : den);
  w.put(cap::kTlvConfigureAnswerActualRate, r, sizeof r);
  uint8_t layout[2 + kMaxChannels] = {8, lines};
  for (uint8_t k = 0; k < lines; ++k) layout[2 + k] = k;
  w.put(cap::kTlvConfigureAnswerLayout, layout, 2 + lines);
  w.u32(cap::kTlvConfigureAnswerActualSamples, samples);
  if (format == cap::kDataFormatStateTimed) {   // format 3 and its delta's tick: the CPU's cycles
    w.u8(cap::kTlvConfigureAnswerFormat, format);
    uint8_t tb[8];
    putU32(tb, cpu_hz);
    putU32(tb + 4, 1);
    w.put(cap::kTlvConfigureAnswerTimebase, tb, sizeof tb);
  }
  if (mode == cap::kModeRepeat) w.u32(cap::kTlvConfigureAnswerActualSegments, segs);
  return w.ok() ? completed(w.length()) : failed();
}

// The segment record (oep-if-capture §2), 37 bytes. Repeat: every segment started by its own trigger (bit3), and after
// the first not straight after the one before (bit0: rearm 1's wait, §2.2).
size_t SamplerCapture::segmentInfo(uint8_t *out, uint32_t serial) const {
  const SlotInfo &slot = slots_[serial % seg_count_];
  putU32(out, serial);
  putU64(out + 4, static_cast<uint64_t>(serial) * samples_);
  putU32(out + 12, slot.samples);
  putU64(out + 16, slot.start_ns);
  putU32(out + 24, 2000);          // the software pace's first sample against the clock read: +-2 us
  putU32(out + 28, trig_type_ ? slot.kept : 0xFFFFFFFFu);   // immediate: no trigger inside; early: fewer before it
  uint8_t flags = slot.slipped ? cap::kSegmentFlagSlipped : 0;   // bit2: a sample a period or more late (OepSamplerRun.h)
  if (clocked_ && slot.samples < samples_) flags |= cap::kSegmentFlagShort;   // the window ended before every edge came
  if (repeat()) flags |= cap::kSegmentFlagRearm | (serial ? cap::kSegmentFlagGap : 0);
  out[32] = flags;
  putU32(out + 33, generation_);
  return 37;
}

// A window (or a trigger's search, then the window) into the next segment's slot, on the core loop() is not on (this
// call's): its windows mask that core, and loop()'s keeps answering
bool SamplerCapture::startRun() {
  done_ = trig_seen_ = aborted_ = false;
  control_ = 0;
  out_ = buffer_ + static_cast<size_t>(completed_ % seg_count_) * bytesOf(samples_);
  memset(out_, 0, bytesOf(samples_));
  if (xTaskCreatePinnedToCore(samplerTask, "oep_sampler", 4096, this, kTaskPriority, &sampler_,
                              static_cast<BaseType_t>(xPortGetCoreID() ^ 1)) != pdPASS) {
    sampler_ = nullptr;
    return false;
  }
  return true;
}

// The window's segment kept (its slot's info, the event); then one-shot is done, repeat arms the next segment when a slot
// is free (`rearm`: not when stopping) or waits for release in state 5
void SamplerCapture::segmentDone(bool rearm) {
  slots_[completed_ % seg_count_] = {trig_type_ || clocked_ ? seg_start_ns_ : start_ns_, seg_samples_, trig_kept_, slipped_};
  const uint32_t serial = completed_++;
  if (subscribed_) {
    uint8_t seg[37];
    endpoint_.event(*this, cap::kEventSegment, seg, segmentInfo(seg, serial));
  }
  if (!repeat()) {
    state_ = cap::kStateDone;
    if (subscribed_) {
      uint8_t stopped[6] = {cap::kStoppedReasonComplete, 0};   // reason(u8) error(u8) generation(u32) (capture §3.4)
      putU32(stopped + 2, generation_);
      endpoint_.event(*this, cap::kEventStopped, stopped, sizeof stopped);
    }
    return;
  }
  if (!rearm) return;
  if (completed_ - released_ < seg_count_ && startRun()) rep_wait_ = true;   // state 3, flags bit2 (§2.1)
  else state_ = cap::kStatePaused;   // every slot waits for the host (no stopped event)
}

void SamplerCapture::poll() {
  gWireGate.release();   // loop() came round: the SWIO wire's request has ended, a window may open (OepWireGate.h)
  if (state_ == cap::kStatePaused && completed_ - released_ < seg_count_ && startRun()) {   // a slot released
    state_ = cap::kStateCapturing;
    rep_wait_ = true;
  }
  if ((state_ == cap::kStateWaiting || (state_ == cap::kStateCapturing && rep_wait_)) && trig_seen_) {
    state_ = cap::kStateCapturing;
    rep_wait_ = false;
    // +- a sample. The search runs in bursts with gaps between them, so the sample from the generation's start is its
    // time from start at the rate (what a sampler that never paused would have counted)
    const uint64_t ns = trig_burst_ns_ + static_cast<uint64_t>(trig_count_) * cycles_ * 1000000000ull / cpu_hz_;
    const uint64_t sample = ns > start_ns_ ? (ns - start_ns_) * (cpu_hz_ / 1000u) / cycles_ / 1000000u : 0;
    const uint32_t record = repeat() ? record_next_++ : 0;
    if (repeat()) records_[record % kTriggerRecords] = {sample, ns, record, periodNs(cpu_hz_, cycles_)};   // §3.10
    if (subscribed_) {   // capture §3.4
      uint8_t e[kTriggeredBytes];
      putTriggered(e, record, trig_kept_, sample, ns, periodNs(cpu_hz_, cycles_), generation_);
      endpoint_.event(*this, cap::kEventTriggered, e, sizeof e);
    }
  }
  if (state_ != cap::kStateCapturing || rep_wait_ || !done_ || aborted_) return;
  segmentDone(true);
}

Result SamplerCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  // bound in a capture-group: the group starts and stops it (unavailable cause 4, after the request's form, core §4.3)
  auto boundRefused = [&]() { return bound() && !group_op_; };
  Tail tail;
  switch (op) {
    case cap::kOpConfigure: return configure(p, n, out, capacity, false);
    case cap::kOpQuery: return configure(p, n, out, capacity, true);
    case cap::kOpStart: {   // -> generation(u32) [TLV]
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (boundRefused()) return boundInGroup(*this, out, capacity);
      if (state_ != cap::kStateConfigured && state_ != cap::kStateDone && state_ != cap::kStateError) return wrongState(out, capacity);
      if (capacity < 4) return failed();
      waitIdle();
      listen(pins_, channels_);
      completed_ = released_ = record_next_ = 0;
      rep_wait_ = false;
      start_ns_ = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
      generation_ = nextGeneration(generation_);   // 1 after 0xFFFFFFFF, never 0 (capture §3.2)
      if (!startRun()) {
        state_ = cap::kStateError;   // every entry to state 6 sends stopped reason 3 (capture §3.2)
        if (subscribed_) {
          uint8_t stopped[6] = {cap::kStoppedReasonError, cap::kErrorPeripheral};
          putU32(stopped + 2, generation_);
          endpoint_.event(*this, cap::kEventStopped, stopped, sizeof stopped);
        }
        return failed();
      }
      state_ = trig_type_ ? cap::kStateWaiting : cap::kStateCapturing;
      putU32(out, generation_);
      return completed(4);
    }
    case cap::kOpStop: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (boundRefused()) return boundInGroup(*this, out, capacity);
      poll();
      uint8_t stopped[6] = {cap::kStoppedReasonHost, 0};   // reason(u8) error(u8) generation(u32) (capture §3.4)
      putU32(stopped + 2, generation_);
      const bool searching = state_ == cap::kStateWaiting || (state_ == cap::kStateCapturing && rep_wait_);
      if (searching || state_ == cap::kStateCapturing) {
        // a search ends at its next sample; a window cannot be cut short (interrupts are off on the sampler's core) and
        // ends by itself: its segment is kept (one-shot: complete, state 4 - capture §3.2's "complete -> 4")
        waitIdle();
        if (done_ && !aborted_ && (trig_seen_ || !trig_type_)) segmentDone(false);
      }
      if (state_ == cap::kStateWaiting || state_ == cap::kStateCapturing || state_ == cap::kStatePaused) {
        state_ = cap::kStateConfigured;
        rep_wait_ = false;
        if (subscribed_) endpoint_.event(*this, cap::kEventStopped, stopped, sizeof stopped);
      }
      return completed();
    }
    case cap::kOpStatus: {   // -> state(u8) serial_done(u32) write_pos(u64) flags(u8) generation(u32) [TLV]
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 22) return failed();
      poll();
      out[0] = state_;
      putU32(out + 1, completed_);
      // one-shot: the segment's samples (fewer after an early trigger); repeat: every slot samples long (arm 1)
      putU64(out + 5, repeat() ? static_cast<uint64_t>(completed_) * samples_ : completed_ ? bytesOf(slots_[0].samples) : 0);
      out[13] = slipped_ ? cap::kStatusFlagSlipped : 0;   // bit1: a sample taken a period or more late (OepSamplerRun.h)
      if (repeat() && rep_wait_ && state_ == cap::kStateCapturing) out[13] |= cap::kStatusFlagRearmWaiting;   // §3.2
      putU32(out + 14, generation_);
      size_t used = 18;
      if (state_ == cap::kStateError) {   // why (TLV 0x01 error, capture §3.2): the sampling task would not start
        putTlvHeader(out + 18, cap::kTlvStatusAnswerError, 1);
        out[21] = cap::kErrorPeripheral;
        used = 22;
      }
      return completed(used);
    }
    case cap::kOpRead: {   // generation(u32) position(u64) max(u16) -> position(u64) flags(u8) len(u16) data [TLV]
      const Result parsed = plainTail(tail, p, n, 14, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);   // another generation (cause 6)
      if (capacity < 11) return failed();
      poll();
      // the segments kept: one-shot's, or repeat's last seg_count_ (a released one readable until its slot is used again)
      const uint64_t have = repeat() ? static_cast<uint64_t>(completed_) * samples_ : completed_ ? bytesOf(slots_[0].samples) : 0;
      const uint64_t low = repeat() && completed_ > seg_count_ ? static_cast<uint64_t>(completed_ - seg_count_) * samples_ : 0;
      uint64_t position = getU64(p + 4);
      uint8_t flags = 0;
      if (position < low) { position = low; flags |= reg::common::kReadFlagsGap; }   // used again (§3.2)
      if (position > have) position = have;
      uint32_t max = getU16(p + 12);
      uint32_t count = static_cast<uint32_t>(have - position);
      size_t room = capacity - 11;   // how much read returns is the probe's (capture §3.2): max_read_ and the frame
      if (room > max_read_) room = max_read_;
      if (max > room) max = static_cast<uint32_t>(room);
      if (count > max) { count = max; flags |= reg::common::kReadFlagsMore; }
      putU64(out, position);
      out[8] = flags;
      putU16(out + 9, static_cast<uint16_t>(count));
      for (uint32_t i = 0; i < count;) {   // across the slots
        const uint64_t at = position + i;
        const uint32_t slot = bytesOf(samples_);   // one-shot's format 3: 5 bytes a sample
        const uint32_t serial = static_cast<uint32_t>(at / slot), off = static_cast<uint32_t>(at % slot);
        const uint32_t take = slot - off < count - i ? slot - off : count - i;
        memcpy(out + 11 + i, buffer_ + static_cast<size_t>(serial % seg_count_) * slot + off, take);
        i += take;
      }
      return completed(11 + count);
    }
    case cap::kOpSegments: {   // from_serial(u32) -> more(u8) count(u8) count x segment [TLV]
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 2) return failed();
      poll();
      const uint32_t next = completed_, oldest = completed_ > seg_count_ ? completed_ - seg_count_ : 0;
      uint32_t k = serialPageStart(getU32(p), oldest, next);   // common §1.3's serial paging
      uint8_t count = 0;
      size_t used = 2;
      for (; k != next && used + 37 <= capacity && count < 255; ++k, ++count) used += segmentInfo(out + used, k);
      out[0] = k != next ? 1 : 0;
      out[1] = count;
      return completed(used);
    }
    case cap::kOpRelease: {   // generation(u32) serial(u32): the finished segments up to serial may be reused
      const Result parsed = plainTail(tail, p, n, 8, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);
      if (!repeat()) return completed();   // one-shot: nothing to release
      const uint32_t serial = getU32(p + 4);
      if (!serialBefore(serial, released_)) released_ = serialBefore(serial, completed_) ? serial + 1 : completed_;
      poll();   // state 5 with a slot free again: the next segment armed
      return completed();
    }
    case cap::kOpForce: {          // waiting for a trigger (the first, or rearm 1's between segments): the next sample
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (boundRefused()) return boundInGroup(*this, out, capacity);
      if (state_ == cap::kStateWaiting || (state_ == cap::kStateCapturing && rep_wait_)) control_ = sampler::kControlForce;
      return completed();
    }
    case cap::kOpTriggers: {   // generation(u32) from_serial(u32) [TLV] -> more(u8) count(u8) count x record [TLV] (§3.10)
      const Result parsed = plainTail(tail, p, n, 8, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);
      if (capacity < 2) return failed();
      poll();
      constexpr size_t kRecordBytes = 24;   // serial(u32) sample(u64) trigger_ns(u64) uncertainty_ns(u32)
      const uint32_t next = repeat() ? record_next_ : 0;
      const uint32_t kept = next < kTriggerRecords ? next : kTriggerRecords;
      uint32_t k = serialPageStart(getU32(p + 4), next - kept, next);
      uint8_t count = 0;
      size_t used = 2;
      for (; k != next && used + kRecordBytes <= capacity && count < 255; ++k, ++count) {
        const TriggerRecord &r = records_[k % kTriggerRecords];
        putU32(out + used, r.serial);
        putU64(out + used + 4, r.sample);
        putU64(out + used + 12, r.ns);
        putU32(out + used + 20, r.uncertainty);
        used += kRecordBytes;
      }
      out[0] = k != next ? 1 : 0;
      out[1] = count;
      return completed(used);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
