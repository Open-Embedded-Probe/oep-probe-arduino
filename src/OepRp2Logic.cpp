// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepRp2Logic.h"

#if defined(ARDUINO_ARCH_RP2040)

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/pio_instructions.h>
#include <pico/time.h>
#include <stdlib.h>
#include <string.h>

#include "OepEndpoint.h"

namespace oep {
namespace {

namespace cap = reg::fixture_logic;
enum : uint8_t { kTagMode = cap::kTlvConfigureMode, kTagRate = cap::kTlvConfigureRate,
                 kTagSamples = cap::kTlvConfigureSamples, kTagSegments = cap::kTlvConfigureSegments,
                 kTagTrigger = cap::kTlvConfigureTrigger, kTagPretrigger = cap::kTlvConfigurePretrigger };

// The program, every loop three cycles a sample (the comments: the label, what the loop waits for). X: the samples
// after the trigger's less one; Y: the pretrigger's less one. The trigger's sample is the one before the jmp that saw
// it, one cycle before the look on both the rise's and the fall's (the fall's two put its index a sample late about
// twice as often: bench, 4385db8); from it to the next sample is three cycles on every path (an extra jmp or nop where
// a taken jmp would be early).
enum : uint8_t {
  kPreRise = 0, kLowRise = 2, kHighRise = 4, kRiseHit = 7,   // rising: the pretrigger, then low, then high
  kPreFall = 8, kHighFall = 10, kFallTurn = 13, kLowFall = 14,   // falling: the pretrigger, then high, then low
  kPost = 16, kEnd = 19,                                         // the samples after it; the IRQ flag, a halt
  kPreEither = 20, kEither = 22,                                 // either edge: the pretrigger, then the first change
  kPreHigh = 24, kPreLow = 27,   // a level with when 1: the pretrigger, then at it (hi_r / lo_f: at once if it is)
  kLength = 30,
};
uint16_t gProgram[kLength];
void build() {
  const uint16_t in = pio_encode_in(pio_pins, 32);
  uint16_t *p = gProgram;
  p[0] = in | pio_encode_delay(1);        // pre_r
  p[1] = pio_encode_jmp_y_dec(kPreRise);
  p[2] = in | pio_encode_delay(1);        // lo_r: while high
  p[3] = pio_encode_jmp_pin(kLowRise);
  p[4] = in;                              // hi_r: while low
  p[5] = pio_encode_jmp_pin(kRiseHit);
  p[6] = pio_encode_jmp(kHighRise);
  p[7] = pio_encode_jmp(kPost);           // the hit's third cycle
  p[8] = in | pio_encode_delay(1);        // pre_f
  p[9] = pio_encode_jmp_y_dec(kPreFall);
  p[10] = in;                             // hi_f: while low
  p[11] = pio_encode_jmp_pin(kFallTurn);
  p[12] = pio_encode_jmp(kHighFall);
  p[13] = pio_encode_nop();               // the turn's third cycle
  p[14] = in;                             // lo_f: while high; low falls through to the samples after it
  p[15] = pio_encode_jmp_pin(kLowFall) | pio_encode_delay(1);   // the look one cycle after the sample, as hi_r's
  p[16] = in | pio_encode_delay(1);       // post
  p[17] = pio_encode_jmp_x_dec(kPost);
  p[18] = pio_encode_irq_set(true, 0);    // the flag of this state machine (rel)
  p[19] = pio_encode_jmp(kEnd);
  p[20] = in | pio_encode_delay(1);       // pre_a
  p[21] = pio_encode_jmp_y_dec(kPreEither);
  p[22] = pio_encode_jmp_pin(kFallTurn);  // high now: the first change is a fall
  p[23] = pio_encode_jmp(kHighRise);      // low: a rise
  p[24] = in | pio_encode_delay(1);       // pre_h (its last sample to hi_r's first: four cycles, once)
  p[25] = pio_encode_jmp_y_dec(kPreHigh);
  p[26] = pio_encode_jmp(kHighRise);
  p[27] = in | pio_encode_delay(1);       // pre_l
  p[28] = pio_encode_jmp_y_dec(kPreLow);
  p[29] = pio_encode_jmp(kLowFall);
}

uint32_t gcd(uint32_t a, uint32_t b) {
  while (b) { const uint32_t t = a % b; a = b; b = t; }
  return a;
}

// The halt's time (the PIO flag's interrupt): the last sample's, within the interrupt's latency
PIO gIrqPio = nullptr;
int gIrqSm = -1;
volatile uint64_t gEndUs = 0;
void onFlag() {
  if (gIrqPio && gIrqSm >= 0 && pio_interrupt_get(gIrqPio, static_cast<uint>(gIrqSm))) {
    gEndUs = time_us_64();
    pio_interrupt_clear(gIrqPio, static_cast<uint>(gIrqSm));
  }
}

}  // namespace

bool Rp2LogicCapture::claim() {
  if (sm_ >= 0) return true;
  build();
  pio_program_t program = {};
  program.instructions = gProgram;
  program.length = kLength;
  program.origin = -1;
  uint sm = 0, offset = 0;
  PIO pio = nullptr;
  if (!pio_claim_free_sm_and_add_program(&program, &pio, &sm, &offset)) return false;
  const int data = dma_claim_unused_channel(false), ctrl = dma_claim_unused_channel(false);
  if (data < 0 || ctrl < 0) {
    if (data >= 0) dma_channel_unclaim(data);
    if (ctrl >= 0) dma_channel_unclaim(ctrl);
    pio_remove_program_and_unclaim_sm(&program, pio, sm, offset);
    return false;
  }
  pio_ = pio;
  sm_ = static_cast<int>(sm);
  offset_ = static_cast<int>(offset);
  dma_data_ = data;
  dma_ctrl_ = ctrl;
  gIrqPio = pio_;
  gIrqSm = sm_;
  const uint irq = pio_get_irq_num(pio_, 0);
  pio_set_irqn_source_enabled(pio_, 0, static_cast<pio_interrupt_source_t>(pis_interrupt0 + sm), true);
  irq_add_shared_handler(irq, onFlag, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(irq, true);
  return true;
}

void Rp2LogicCapture::halt() {
  if (sm_ < 0) return;
  pio_sm_set_enabled(pio_, static_cast<uint>(sm_), false);
  // Both channels' enable off first, then both aborted at once: aborting the data channel alone set off its chain
  // (RP2040-E13), the control channel started it again, and dma_channel_abort's wait for it to stop never ended - the
  // probe's watchdog reset it (bench, 4385db8: every triggered capture, its ring mid-round)
  hw_clear_bits(&dma_hw->ch[dma_ctrl_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
  hw_clear_bits(&dma_hw->ch[dma_data_].al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
  const uint32_t both = (1u << dma_data_) | (1u << dma_ctrl_);
  dma_hw->abort = both;
  for (uint32_t t = micros(); (dma_hw->abort & both) && micros() - t < 1000;) {}
  pio_sm_clear_fifos(pio_, static_cast<uint>(sm_));
  pio_sm_restart(pio_, static_cast<uint>(sm_));
  pio_interrupt_clear(pio_, static_cast<uint>(sm_));
}

// The ring's length: the segment, and one more sample when nothing comes after the trigger's (the post loop takes one
// at least: that last one is not the segment's)
static uint32_t ringOf(uint32_t samples, uint8_t trig_type, uint32_t pre) {
  return samples + (trig_type && samples - pre - 1 == 0 ? 1 : 0);
}

bool Rp2LogicCapture::begin() {
  const uint sm = static_cast<uint>(sm_);
  const uint32_t ring = ringOf(samples_, trig_type_, pretrigger_);
  const uint32_t post = trig_type_ ? samples_ - pretrigger_ - 1 : samples_;
  const uint32_t x = post ? post - 1 : 0;
  const uint32_t y = pretrigger_ ? pretrigger_ - 1 : 0;
  uint8_t entry = kPost;
  if (trig_type_) {
    const bool level = trig_type_ == cap::kTriggerLevel;
    // a level high (when 0) is the rise into it, a level low the fall; when 1 one already at it counts (the wait for
    // it alone); an edge 0 rising, 1 falling, 2 either
    const uint8_t way = level ? (trig_value_ ? 0 : 1) : static_cast<uint8_t>(trig_value_);
    if (level && when_) entry = way == 0 ? (pretrigger_ ? kPreHigh : kHighRise) : (pretrigger_ ? kPreLow : kLowFall);
    else if (way == 0) entry = pretrigger_ ? kPreRise : kLowRise;
    else if (way == 1) entry = pretrigger_ ? kPreFall : kHighFall;
    else entry = pretrigger_ ? kPreEither : kEither;
  }
  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, static_cast<uint>(offset_), static_cast<uint>(offset_ + kLength - 1));
  sm_config_set_in_pins(&c, 0);   // GPIO0..31: a sample is the whole input register
  sm_config_set_in_shift(&c, false, true, 32);   // autopush every sample
  sm_config_set_jmp_pin(&c, trig_type_ ? static_cast<uint>(pins_[trig_role_]) : 0);
  sm_config_set_clkdiv_int_frac(&c, static_cast<uint16_t>(divider_), 0);
  pio_sm_init(pio_, sm, static_cast<uint>(offset_ + entry), &c);
  // X and Y through the TX FIFO, before the RX FIFO takes the TX one's room (joined: eight words for the DMA to lag by)
  pio_sm_put_blocking(pio_, sm, x);
  pio_sm_exec(pio_, sm, pio_encode_pull(false, true));
  pio_sm_exec(pio_, sm, pio_encode_mov(pio_x, pio_osr));
  pio_sm_put_blocking(pio_, sm, y);
  pio_sm_exec(pio_, sm, pio_encode_pull(false, true));
  pio_sm_exec(pio_, sm, pio_encode_mov(pio_y, pio_osr));
  pio_->sm[sm].shiftctrl |= PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS;
  pio_sm_clear_fifos(pio_, sm);
  pio_sm_exec(pio_, sm, pio_encode_jmp(static_cast<uint>(offset_ + entry)));
  pio_->fdebug = 1u << (PIO_FDEBUG_RXSTALL_LSB + sm);   // a stall from here on: a sample late
  pio_interrupt_clear(pio_, sm);
  // the data channel round the ring; the control channel puts it back at the start each time it ends
  ring_base_ = ring_;
  dma_channel_config d = dma_channel_get_default_config(static_cast<uint>(dma_data_));
  channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
  channel_config_set_read_increment(&d, false);
  channel_config_set_write_increment(&d, true);
  channel_config_set_dreq(&d, pio_get_dreq(pio_, sm, false));
  channel_config_set_chain_to(&d, static_cast<uint>(dma_ctrl_));
  dma_channel_config k = dma_channel_get_default_config(static_cast<uint>(dma_ctrl_));
  channel_config_set_transfer_data_size(&k, DMA_SIZE_32);
  channel_config_set_read_increment(&k, false);
  channel_config_set_write_increment(&k, false);
  dma_channel_configure(static_cast<uint>(dma_ctrl_), &k, &dma_hw->ch[dma_data_].al2_write_addr_trig, &ring_base_, 1,
                        false);
  dma_channel_configure(static_cast<uint>(dma_data_), &d, ring_, &pio_->rxf[sm], ring, true);
  gEndUs = 0;
  pio_sm_set_enabled(pio_, sm, true);
  return true;
}

bool Rp2LogicCapture::finished() const {
  return sm_ >= 0 && pio_sm_get_pc(pio_, static_cast<uint>(sm_)) == static_cast<uint>(offset_ + kEnd);
}

bool Rp2LogicCapture::inPost() const {
  if (sm_ < 0) return false;
  const uint pc = pio_sm_get_pc(pio_, static_cast<uint>(sm_)) - static_cast<uint>(offset_);
  return pc >= kPost && pc <= kEnd;
}

// The ring turned to start at the segment's first sample (the oldest: the ring is the segment, plus the one after it
// when there is one), packed to a byte a sample
void Rp2LogicCapture::collect() {
  const uint sm = static_cast<uint>(sm_);
  for (uint32_t t = micros(); !pio_sm_is_rx_fifo_empty(pio_, sm) && micros() - t < 1000;) {}
  busy_wait_us(2);   // the DMA's last word in
  const uint32_t ring = ringOf(samples_, trig_type_, pretrigger_);
  const uint32_t written = static_cast<uint32_t>(
      (reinterpret_cast<uint32_t *>(dma_hw->ch[dma_data_].write_addr) - ring_));
  const uint32_t oldest = written >= ring ? 0 : written;   // the next write's place: the oldest sample's
  slipped_ = (pio_->fdebug & (1u << (PIO_FDEBUG_RXSTALL_LSB + sm))) != 0;
  halt();
  for (uint32_t i = 0; i < samples_; ++i) {
    const uint32_t w = ring_[(oldest + i) % ring];
    uint8_t b = 0;
    for (uint8_t l = 0; l < channels_; ++l) b |= static_cast<uint8_t>(((w >> pins_[l]) & 1u) << l);
    segment_[i] = b;
  }
  const uint64_t period_ns = static_cast<uint64_t>(divider_) * kCyclesPerSample * 1000000000ull / clk_hz_;
  const uint64_t end_ns = (gEndUs ? gEndUs : time_us_64()) * 1000ull;   // the last sample the ring took
  seg_start_ns_ = end_ns - static_cast<uint64_t>(ring - 1) * period_ns;
}

// The trigger's time: its sample's, once the capture is in (the halt's time less the samples after it)
bool Rp2LogicCapture::trackTriggerNs(uint64_t &ns) const {
  if (!trig_type_ || state_ != cap::kStateDone) return false;
  ns = seg_start_ns_ + static_cast<uint64_t>(pretrigger_) * divider_ * kCyclesPerSample * 1000000000ull / clk_hz_;
  return true;
}

uint64_t Rp2LogicCapture::trackLatencyNs() const {
  const uint64_t post = samples_ > pretrigger_ ? samples_ - pretrigger_ - 1 : 0;
  return (clk_hz_ ? post * divider_ * kCyclesPerSample * 1000000000ull / clk_hz_ : 0) + kLoopNs;
}

size_t Rp2LogicCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  uint8_t roles[kMaxChannels];
  for (uint8_t k = 0; k < kMaxChannels; ++k) roles[k] = k;
  w.roleChannels(roles, kMaxChannels, table_.allowedMask());
  uint8_t mode[9] = {cap::kModeOneShot};
  putU32(mode + 1, kMaxSamples);
  putU32(mode + 5, 1);
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  const uint32_t clk = clock_get_hz(clk_sys);
  uint8_t range[9];
  putU32(range, kMinHz);
  putU32(range + 4, clk / (kCyclesPerSample * kMinDivider));
  range[8] = 0;   // not any value: clk_sys / (3 x a whole divider)
  w.put(cap::kTlvDescribeRateRange, range, sizeof range);
  w.u8(cap::kTlvDescribeChannels, kMaxChannels);
  uint8_t trig[9];
  putTriggerDecl(trig, (1u << cap::kTriggerImmediate) | (1u << cap::kTriggerLevel) | (1u << cap::kTriggerEdge),
                 kMaxSamples - 1, kTriggerOpts);
  w.put(cap::kTlvDescribeTrigger, trig, sizeof trig);
  return w.ok() ? w.length() : 0;
}

uint8_t Rp2LogicCapture::planCheck(const RoleAssignment *roles, size_t count) {
  plan_cause_ = reg::core::kUnavailableCauseWrongState;
  if (state_ == cap::kStateWaiting || state_ == cap::kStateCapturing) return kRejectUnavailable;
  plan_cause_ = reg::core::kUnavailableCauseLimit;
  uint32_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role >= kMaxChannels) return kRejectUnsupported;
    if (!table_.allowed(roles[i].channel) || roles[i].channel > 31) return kRejectUnsupported;
    if (seen & (1u << roles[i].role)) return kRejectUnavailable;
    seen |= 1u << roles[i].role;
  }
  if (count > kMaxChannels) return kRejectUnavailable;
  return (count == 0 || seen == (1u << count) - 1) ? 0 : kRejectUnavailable;
}

// Taking the plan changes no pin (a logic capture only listens, capture §1.2, core §8); the pads' inputs are on.
bool Rp2LogicCapture::planApply(const RoleAssignment *roles, size_t count) {
  halt();
  for (size_t i = 0; i < count; ++i) pins_[roles[i].role] = roles[i].channel;
  channels_ = static_cast<uint8_t>(count);
  for (uint8_t l = 0; l < channels_; ++l) gpio_set_input_enabled(static_cast<uint>(pins_[l]), true);
  state_ = cap::kStateUnconfigured;
  return true;
}

void Rp2LogicCapture::planRelease() {
  halt();
  channels_ = 0;
  state_ = cap::kStateUnconfigured;
}

Result Rp2LogicCapture::configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query) {
  // as the classic ESP32's sampler (OepSampler.cpp): the form, then the values, then the plan and the state
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
  const uint32_t clk = clock_get_hz(clk_sys);
  const uint32_t max_hz = clk / (kCyclesPerSample * kMinDivider);
  const uint32_t rate = getU32(rate_tlv.v);
  if (rate < kMinHz || rate > max_hz) return Tail::refuse(kTagRate, rate_tlv.critical, out, capacity);
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
  if (!query && bound()) return boundInGroup(*this, out, capacity);   // the group's now (cause 4)
  if ((state_ == cap::kStateCapturing || state_ == cap::kStateWaiting) && !query) return wrongState(out, capacity);
  // a whole divider: the rate paced is clk_sys / (3 x it), the nearest to the one asked at or above kMinDivider
  uint32_t divider = (clk + kCyclesPerSample * rate / 2) / (kCyclesPerSample * rate);
  if (divider < kMinDivider) divider = kMinDivider;
  if (divider > 65535) divider = 65535;
  const uint32_t den_full = kCyclesPerSample * divider, g = gcd(clk, den_full);
  if (!query) {
    halt();
    if (!claim()) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);   // no PIO state machine / DMA
    const uint32_t ring = samples + 1;   // the most a ring of this segment takes
    if (capacity_ < ring) {
      free(ring_);
      free(segment_);
      ring_ = static_cast<uint32_t *>(malloc(ring * sizeof(uint32_t)));
      segment_ = static_cast<uint8_t *>(malloc(ring));
      capacity_ = ring_ && segment_ ? ring : 0;
      if (!capacity_) {
        free(ring_);
        free(segment_);
        ring_ = nullptr;
        segment_ = nullptr;
        return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
      }
    }
    samples_ = samples;
    trig_type_ = trig_type;
    trig_role_ = trig_role;
    trig_value_ = trig_value;
    pretrigger_ = pretrigger;
    when_ = opts.when;
    divider_ = divider;
    clk_hz_ = clk;
    state_ = cap::kStateConfigured;
  }
  TlvWriter w(out, capacity);
  uint8_t r[8];
  putU32(r, clk / g);
  putU32(r + 4, den_full / g);
  w.put(cap::kTlvConfigureAnswerActualRate, r, sizeof r);
  uint8_t layout[2 + kMaxChannels] = {8, channels_};
  for (uint8_t k = 0; k < channels_; ++k) layout[2 + k] = k;
  w.put(cap::kTlvConfigureAnswerLayout, layout, 2 + channels_);
  w.u32(cap::kTlvConfigureAnswerActualSamples, samples);
  return w.ok() ? completed(w.length()) : failed();
}

size_t Rp2LogicCapture::segmentInfo(uint8_t *out) const {   // the segment record (oep-if-capture §2), 37 bytes
  putU32(out, 0);
  putU64(out + 4, 0);
  putU32(out + 12, samples_);
  putU64(out + 16, seg_start_ns_);
  putU32(out + 24, 2000);   // the halt's interrupt against the clock read: a few us
  putU32(out + 28, trig_type_ ? pretrigger_ : 0xFFFFFFFFu);
  out[32] = slipped_ ? cap::kSegmentFlagSlipped : 0;
  putU32(out + 33, generation_);
  return 37;
}

void Rp2LogicCapture::stopped(uint8_t reason, uint8_t error) {
  if (!subscribed_) return;
  uint8_t e[6] = {reason, error};
  putU32(e + 2, generation_);
  endpoint_.event(*this, cap::kEventStopped, e, sizeof e);
}

void Rp2LogicCapture::poll() {
  if (state_ == cap::kStateWaiting && (inPost() || finished())) {
    state_ = cap::kStateCapturing;
    trig_seen_ = true;
    if (subscribed_) {   // capture §3.4: no record (rearm 0); its time from now, the samples after it still to come
      const uint64_t period_ns = static_cast<uint64_t>(divider_) * kCyclesPerSample * 1000000000ull / clk_hz_;
      const uint64_t ns = time_us_64() * 1000ull;
      const uint64_t sample = ns > start_ns_ ? (ns - start_ns_) / period_ns : 0;
      uint8_t e[kTriggeredBytes];
      putTriggered(e, 0, pretrigger_, sample, ns, periodNs(clk_hz_, divider_ * kCyclesPerSample), generation_);
      endpoint_.event(*this, cap::kEventTriggered, e, sizeof e);
    }
  }
  if (state_ != cap::kStateCapturing || !finished()) return;
  collect();
  state_ = cap::kStateDone;
  if (subscribed_) {
    uint8_t seg[37];
    endpoint_.event(*this, cap::kEventSegment, seg, segmentInfo(seg));
  }
  stopped(cap::kStoppedReasonComplete);
}

Result Rp2LogicCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
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
      halt();
      for (uint8_t l = 0; l < channels_; ++l) gpio_set_input_enabled(static_cast<uint>(pins_[l]), true);
      trig_seen_ = slipped_ = false;
      start_ns_ = time_us_64() * 1000ull;
      generation_ = nextGeneration(generation_);
      if (!begin()) {
        state_ = cap::kStateError;
        stopped(cap::kStoppedReasonError, cap::kErrorPeripheral);
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
      if (state_ == cap::kStateWaiting) {   // no trigger yet: nothing captured
        halt();
        state_ = cap::kStateConfigured;
        stopped(cap::kStoppedReasonHost);
      } else if (state_ == cap::kStateCapturing) {
        // the samples after the trigger are a few ms at most: wait them out, the capture completes (capture §3.2)
        for (uint32_t t = millis(); !finished() && millis() - t < 2000;) delay(1);
        poll();
        if (state_ == cap::kStateCapturing) {
          halt();
          state_ = cap::kStateConfigured;
          stopped(cap::kStoppedReasonHost);
        }
      }
      return completed();
    }
    case cap::kOpStatus: {   // -> state(u8) serial_done(u32) write_pos(u64) flags(u8) generation(u32) [TLV]
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 21) return failed();
      poll();
      out[0] = state_;
      const bool done = state_ == cap::kStateDone;
      putU32(out + 1, done ? 1 : 0);
      putU64(out + 5, done ? samples_ : 0);
      out[13] = slipped_ ? cap::kStatusFlagSlipped : 0;
      putU32(out + 14, generation_);
      size_t used = 18;
      if (state_ == cap::kStateError) {
        putTlvHeader(out + 18, cap::kTlvStatusAnswerError, 1);
        out[21] = cap::kErrorPeripheral;
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
      if (count) memcpy(out + 11, segment_ + position, count);
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
    case cap::kOpForce: {   // waiting for the trigger: the samples after it from the next one on
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (boundRefused()) return boundInGroup(*this, out, capacity);
      if (state_ == cap::kStateWaiting && sm_ >= 0 && !inPost())
        pio_sm_exec(pio_, static_cast<uint>(sm_), pio_encode_jmp(static_cast<uint>(offset_ + kPost)));
      return completed();
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
