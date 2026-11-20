// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepPwm.h"

#include <string.h>

#if defined(OEP_HOST_FAKE_PWM)
#elif defined(ARDUINO_ARCH_ESP32)
#define OEP_PWM_ESP 1
#include <driver/ledc.h>
#include <esp_timer.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/ledc_periph.h>
#include <soc/soc_caps.h>
#elif defined(ARDUINO_ARCH_RP2040)
#define OEP_PWM_RP2 1
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#endif

namespace oep {

namespace {

#if defined(OEP_PWM_RP2) || defined(OEP_PWM_ESP)
uint64_t gcd(uint64_t a, uint64_t b) {
  while (b) { const uint64_t t = a % b; a = b; b = t; }
  return a;
}
#endif

#if defined(OEP_PWM_ESP)
// LEDC's clock: the crystal where LEDC can take it (P4: 40 MHz), else the APB's 80 MHz (as arduino-esp32 picks it)
#if defined(SOC_LEDC_SUPPORT_XTAL_CLOCK)
constexpr uint32_t kClockHz = 40000000;
constexpr ledc_clk_cfg_t kClockSource = LEDC_USE_XTAL_CLK;
#else
constexpr uint32_t kClockHz = 80000000;
constexpr ledc_clk_cfg_t kClockSource = LEDC_AUTO_CLK;
#endif
constexpr uint8_t kMaxBits = 14;
constexpr uint32_t kFractionBits = 8;   // the divider's fraction (ESP-IDF's LEDC_LL_FRACTIONAL_BITS)

// LEDC's low-speed channels take a new duty, a stop or a start at their timer's next overflow, not at once: a
// channel connected to the line before then shows what it last ran (its old duty, for up to a period), and a timer
// set again while its channel runs cuts that period short. So output k has timer k and two channels (2k, 2k + 1): a
// start sets up the one that has stopped for certain (its stop landed: a period passed), the line held low by the
// GPIO meanwhile, and connects the line to it - the channel stays low until the overflow, then runs the new duty
// from the head of a period. A new duty at the same frequency goes to the running channel (from its next period).
constexpr ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
constexpr uint32_t kSettleMarginUs = 5;
struct Ledc {
  int8_t live = -1, last = 1;              // the channel (0 / 1) driving the line, -1 none; the one used last
  uint32_t hz = 0;
  uint8_t bits = 0;
  bool stopping[2] = {false, false};       // a stop not landed for certain before settled[k] (esp_timer, us)
  int64_t settled[2] = {0, 0};             // 64 bits: a stop kept for hours is never read as in the future
};
Ledc ledc[4];

uint32_t periodUs(uint32_t hz) { return (1000000u + hz - 1) / hz + kSettleMarginUs; }
ledc_channel_t ledcChannel(int role, int k) { return static_cast<ledc_channel_t>(2 * role + k); }

// The line low from the GPIO at once; the channel driving it stops at its period's end
void retire(int role, int pad) {
  Ledc &l = ledc[role];
  if (l.live < 0) return;
  gpio_set_level(static_cast<gpio_num_t>(pad), 0);
  esp_rom_gpio_connect_out_signal(pad, SIG_GPIO_OUT_IDX, false, false);
  ledc_stop(kMode, ledcChannel(role, l.live), 0);
  l.stopping[l.live] = true;
  l.settled[l.live] = esp_timer_get_time() + periodUs(l.hz);
  l.live = -1;
}
#elif defined(OEP_PWM_RP2)
#else
constexpr uint8_t kFakeBits = 8;
#endif

// The duty on the request's scale (0 .. 65535) as `steps` steps of a period, and back
uint32_t stepsOf(uint16_t duty, uint32_t steps) { return (static_cast<uint32_t>(duty) * steps + 32767) / 65535; }
uint16_t dutyOf(uint32_t level, uint32_t steps) { return static_cast<uint16_t>(level >= steps ? 65535 : level * 65535ull / steps); }

// Start (or change) an output at the nearest frequency and duty; what it made. false: the hardware refused it.
bool startOutput(int role, int pad, uint32_t hz, uint16_t duty, uint32_t &num, uint32_t &den, uint16_t &made) {
#if defined(OEP_PWM_ESP)
  uint8_t bits = 1;
  while (bits < kMaxBits && (static_cast<uint64_t>(hz) << (bits + 1)) <= kClockHz) ++bits;
  const uint32_t steps = 1u << bits, level = stepsOf(duty, steps);   // steps itself: always high (LEDC's full duty)
  Ledc &l = ledc[role];
  if (l.live >= 0 && l.hz == hz && l.bits == bits) {
    const ledc_channel_t ch = ledcChannel(role, l.live);
    if (ledc_set_duty_with_hpoint(kMode, ch, level, 0) != ESP_OK || ledc_update_duty(kMode, ch) != ESP_OK) return false;
  } else {
    retire(role, pad);
    const int k = 1 - l.last;
    if (l.stopping[k]) {                     // set twice within a period: wait for that channel's stop to land
      while (esp_timer_get_time() < l.settled[k]) {}
      l.stopping[k] = false;
    }
    digitalWrite(pad, LOW);                  // the line low from the GPIO until the channel takes it
    pinMode(pad, OUTPUT);
    digitalWrite(pad, LOW);
    ledc_timer_config_t t = {};
    t.speed_mode = kMode;
    t.timer_num = static_cast<ledc_timer_t>(role);
    t.duty_resolution = static_cast<ledc_timer_bit_t>(bits);
    t.freq_hz = hz;
    t.clk_cfg = kClockSource;
    if (ledc_timer_config(&t) != ESP_OK) return false;
    const int64_t now = esp_timer_get_time();
    if (l.stopping[1 - k]) l.settled[1 - k] = now + periodUs(hz);   // the timer started again: one new period
    const ledc_channel_t ch = ledcChannel(role, k);
    if (ledc_bind_channel_timer(kMode, ch, t.timer_num) != ESP_OK ||
        ledc_set_duty_with_hpoint(kMode, ch, level, 0) != ESP_OK || ledc_update_duty(kMode, ch) != ESP_OK)
      return false;
    esp_rom_gpio_connect_out_signal(pad, ledc_periph_signal[kMode].sig_out0_idx + ch, false, false);
    l.live = l.last = static_cast<int8_t>(k);
    l.hz = hz;
    l.bits = bits;
  }
  // the frequency made, as ESP-IDF divides (ledc_timer_config: the divider rounded, 8 fraction bits) - not
  // ledcReadFreq, which gives whole Hz (and 0 on the P4)
  const uint64_t clk = static_cast<uint64_t>(kClockHz) << kFractionBits, per = static_cast<uint64_t>(hz) * steps;
  const uint64_t div = (clk + per / 2) / per, n = clk, d = div * steps, g = gcd(n, d);
  if (!div || n / g > 0xFFFFFFFFu || d / g > 0xFFFFFFFFu) return false;
  num = static_cast<uint32_t>(n / g);
  den = static_cast<uint32_t>(d / g);
  made = dutyOf(level, steps);
  return true;
#elif defined(OEP_PWM_RP2)
  (void)role;
  const uint32_t clk = clock_get_hz(clk_sys);
  // the smallest divider (1/16 steps) whose wrap fits 16 bits, then the wrap nearest the frequency
  uint64_t div16 = (static_cast<uint64_t>(clk) * 16 + static_cast<uint64_t>(hz) * 65536 - 1) / (static_cast<uint64_t>(hz) * 65536);
  if (div16 < 16) div16 = 16;
  if (div16 > 4095) div16 = 4095;
  uint64_t top = (static_cast<uint64_t>(clk) * 16 + div16 * hz / 2) / (div16 * hz);
  if (top < 2) top = 2;
  if (top > 65536) top = 65536;
  const uint slice = pwm_gpio_to_slice_num(static_cast<uint>(pad)), chan = pwm_gpio_to_channel(static_cast<uint>(pad));
  pwm_config c = pwm_get_default_config();
  pwm_config_set_clkdiv_int_frac(&c, static_cast<uint8_t>(div16 / 16), static_cast<uint8_t>(div16 % 16));
  pwm_config_set_wrap(&c, static_cast<uint16_t>(top - 1));
  pwm_init(slice, &c, false);
  const uint32_t level = stepsOf(duty, static_cast<uint32_t>(top));   // top itself: always high
  pwm_set_chan_level(slice, chan, static_cast<uint16_t>(level > 65535 ? 65535 : level));
  gpio_set_function(static_cast<uint>(pad), GPIO_FUNC_PWM);
  pwm_set_enabled(slice, true);
  const uint32_t n = clk * 16u, d = static_cast<uint32_t>(div16 * top);   // clk_sys x 16 fits 32 bits
  const uint32_t g = static_cast<uint32_t>(gcd(n, d));
  num = n / g;
  den = d / g;
  made = dutyOf(level, static_cast<uint32_t>(top));
  return true;
#else
  (void)role;
  (void)pad;
  num = hz;
  den = 1;
  made = dutyOf(stepsOf(duty, 1u << kFakeBits), 1u << kFakeBits);
  return true;
#endif
}

// An output stopped: its line driven low
void stopOutput(int role, int pad) {
#if defined(OEP_PWM_ESP)
  retire(role, pad);
#elif defined(OEP_PWM_RP2)
  pwm_set_enabled(pwm_gpio_to_slice_num(static_cast<uint>(pad)), false);
  gpio_set_function(static_cast<uint>(pad), GPIO_FUNC_SIO);
#endif
#if !defined(OEP_PWM_ESP)
  (void)role;
#endif
#if !defined(OEP_HOST_FAKE_PWM)
  digitalWrite(pad, LOW);
  pinMode(pad, OUTPUT);
  digitalWrite(pad, LOW);
#else
  (void)pad;
#endif
}

}  // namespace

uint8_t FixturePwm::planCheck(const RoleAssignment *roles, size_t count) {
  uint8_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    const uint8_t role = roles[i].role;
    if (role >= kMaxChannels || ((seen >> role) & 1) || !pins_.canOutput(roles[i].channel)) return kRejectUnsupported;
    seen |= 1u << role;
    for (size_t j = i + 1; j < count; ++j) {
      if (roles[j].channel == roles[i].channel) return kRejectUnsupported;
#if defined(OEP_PWM_RP2)
      if (pwm_gpio_to_slice_num(platformPad(roles[j].channel)) == pwm_gpio_to_slice_num(platformPad(roles[i].channel)))
        return kRejectUnsupported;   // one slice, one frequency
#endif
    }
  }
  refusal_cause_ = reg::core::kUnavailableCauseWrongState;
  refusal_channel_ = 0xffff;
  for (size_t i = 0; i < count; ++i) {
    const uint8_t idle = pins_.idle(roles[i].channel);
    if (idle == PinTable::kIdleOutputLow || idle == PinTable::kIdleOutputHigh) {
      refusal_cause_ = reg::core::kUnavailableCauseHeldBySettings;
      refusal_channel_ = roles[i].channel;
      return kRejectUnavailable;
    }
    if (!pins_.freeFor(roles[i].channel, kOwnerId)) return kRejectUnavailable;
  }
  return 0;
}

// The plan taken: no line changes until the first set (fixture §7, as the gpio's)
bool FixturePwm::planApply(const RoleAssignment *roles, size_t count) {
  for (Out &o : out_) if (o.running) stop(o);
  pins_.release(kOwnerId);
  for (Out &o : out_) o = Out{};
  for (size_t i = 0; i < count; ++i) {
    if (!pins_.claim(roles[i].channel, kOwnerId)) { planRelease(); return false; }
    out_[roles[i].role].channel = roles[i].channel;
  }
  return true;
}

void FixturePwm::planRelease() {
  for (Out &o : out_) if (o.running) stop(o);
  pins_.release(kOwnerId);   // each line to its idle state
  for (Out &o : out_) o = Out{};
}

void FixturePwm::stop(Out &o) {
  stopOutput(static_cast<int>(&o - out_), platformPad(o.channel));
  o.running = false;
  o.num = 0;
  o.den = 1;
  o.duty = 0;
}

size_t FixturePwm::entry(uint8_t role, uint8_t *at) const {
  const Out &o = out_[role];
  at[0] = role;
  putU32(at + 1, o.num);
  putU32(at + 5, o.den);
  putU16(at + 9, o.duty);
  return 11;
}

size_t FixturePwm::describe(uint8_t *out, size_t capacity) {
  static const uint8_t kRoles[] = {0, 1, 2, 3};
  TlvWriter w(out, capacity);
  w.roleChannels(kRoles, kMaxChannels, pins_.outputMask());
  w.u32(kTagMaxClockHz, kMaxHz);
  w.u32(kTagMinClockHz, kMinHz);
  w.u8(reg::fixture_pwm::kTlvDescribeChannels, kMaxChannels);
#if defined(OEP_PWM_ESP)
  uint8_t bits = 1;
  while (bits < kMaxBits && (static_cast<uint64_t>(kMaxHz) << (bits + 1)) <= kClockHz) ++bits;
  w.u8(reg::fixture_pwm::kTlvDescribeDutyBits, bits);
#elif defined(OEP_PWM_RP2)
  uint8_t bits = 0;
  while ((clock_get_hz(clk_sys) / kMaxHz) >> (bits + 1)) ++bits;
  w.u8(reg::fixture_pwm::kTlvDescribeDutyBits, bits);
#else
  w.u8(reg::fixture_pwm::kTlvDescribeDutyBits, kFakeBits);
#endif
  return w.ok() ? w.length() : 0;
}

Result FixturePwm::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (operation) {
    case kOpSet: {   // n(u8) n x (role(u8) hz(u32) duty(u16)) [TLV] -> n(u8) n x (role num den duty) [TLV]
      if (length < 1) return rejected(kRejectMalformed);
      const uint8_t n = payload[0];
      const size_t fixed = 1u + 7u * n;
      const Result parsed = plainTail(tail, payload, length, fixed, out, capacity);
      if (refused(parsed)) return parsed;
      if (n == 0) return rejected(kRejectMalformed);
      uint8_t seen = 0;
      for (uint8_t i = 0; i < n; ++i) {   // every check before anything changes (core §4.3)
        const uint8_t *e = payload + 1 + 7 * i;
        const uint32_t hz = getU32(e + 1);
        if (e[0] >= kMaxChannels || ((seen >> e[0]) & 1) || (hz && (hz < kMinHz || hz > kMaxHz)))
          return unsupportedValue(out, capacity);
        seen |= 1u << e[0];
      }
      for (uint8_t i = 0; i < n; ++i) if (out_[payload[1 + 7 * i]].channel < 0) return wrongState(out, capacity);
      if (capacity < 1 + 11u * n) return failed();
      out[0] = n;
      for (uint8_t i = 0; i < n; ++i) {
        const uint8_t *e = payload + 1 + 7 * i;
        Out &o = out_[e[0]];
        const uint32_t hz = getU32(e + 1);
        if (!hz) {
          stop(o);
        } else if (startOutput(e[0], platformPad(o.channel), hz, getU16(e + 5), o.num, o.den, o.duty)) {
          o.running = true;
        } else {
          stop(o);
          return failed();
        }
        entry(e[0], out + 1 + 11 * i);
      }
      return completed(1 + 11u * n);
    }
    case kOpStatus: {   // [TLV] -> n(u8) n x (role num den duty) [TLV]: the planned roles
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 1 + 11u * kMaxChannels) return failed();
      size_t at = 1;
      uint8_t n = 0;
      for (uint8_t r = 0; r < kMaxChannels; ++r) {
        if (out_[r].channel < 0) continue;
        at += entry(r, out + at);
        ++n;
      }
      out[0] = n;
      return completed(at);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep
