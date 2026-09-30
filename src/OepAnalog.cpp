// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepAnalog.h"

#include "OepEndpoint.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_timer.h>
#if defined(CONFIG_IDF_TARGET_ESP32)
#include <esp_efuse.h>
#include <esp_efuse_table.h>
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
#include <esp_efuse_rtc_calib.h>
#endif
#elif defined(ARDUINO_ARCH_RP2040)
#include <hardware/adc.h>
#include <hardware/dma.h>
#endif

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)

namespace oep {
namespace ana = reg::fixture_analog;

namespace {

struct Frontend { uint8_t number; int32_t min_mv, max_mv; uint32_t attenuation_mdb; };

// The input ranges (nominal, from the chip's documentation) and the conversions a second the ADC takes in all.
#if defined(CONFIG_IDF_TARGET_ESP32)
constexpr Frontend kFrontends[] = {{0, 100, 950, 0}, {1, 100, 1250, 2500}, {2, 150, 1750, 6000}, {3, 150, 2450, 12000}};
constexpr uint32_t kMinTotalHz = 20000, kMaxTotalHz = 100000;    // the driver's floor; a ceiling kept low (DMA to RAM)
constexpr uint32_t kRatePpm = 20000;                             // not measured on this chip: a wide guess
constexpr size_t kRecordBytes = 2;
constexpr bool kFirstFrameLost = false;                          // not measured here: the time is not corrected
#elif defined(ARDUINO_ARCH_ESP32)
constexpr Frontend kFrontends[] = {{0, 0, 950, 0}, {1, 0, 1250, 2500}, {2, 0, 1750, 6000}, {3, 0, 3100, 12000}};
constexpr uint32_t kMinTotalHz = 611, kMaxTotalHz = 46000;       // above 46 kHz the P4 gives each value twice
constexpr uint32_t kRatePpm = 12000;                             // measured: +0.15 % mostly, +1.2 % at 44.1 kHz
constexpr size_t kRecordBytes = 4;
constexpr bool kFirstFrameLost = true;                           // measured on the P4 (logic-capture §7.4)
#else
constexpr Frontend kFrontends[] = {{0, 0, 3300, 0}};             // ADC_VREF: the supply (3.3 V on a Pico)
constexpr uint32_t kAdcClockHz = 48000000;
constexpr uint32_t kMinTotalHz = kAdcClockHz / 65536 + 1, kMaxTotalHz = kAdcClockHz / 96;
constexpr uint32_t kRatePpm = 100;                               // the crystal's
#endif
constexpr size_t kFrontendCount = sizeof kFrontends / sizeof kFrontends[0];
constexpr uint8_t kWidest = kFrontends[kFrontendCount - 1].number;
constexpr uint16_t kFull = 4095;
#if defined(ARDUINO_ARCH_ESP32)
constexpr size_t kFrameBytes = 256;                              // a conversion frame: the time's correction is one
constexpr uint32_t kFrameConversions = kFrameBytes / kRecordBytes;
#endif

const Frontend *frontendOf(uint8_t number) {
  for (const Frontend &f : kFrontends) if (f.number == number) return &f;
  return nullptr;
}

uint64_t nowNs() {
#if defined(ARDUINO_ARCH_ESP32)
  return static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
#else
  return time_us_64() * 1000u;
#endif
}

#if defined(ARDUINO_ARCH_RP2040)
bool gAdcReady = false;
#endif

}  // namespace

size_t AnalogCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  static const uint8_t kRoles[] = {0, 1, 2, 3};
  w.roleChannels(kRoles, kMaxChannels, adc_pins_);
  uint8_t mode[10] = {ana::kModeOneShot, 1};   // background: the probe keeps answering
  putU32(mode + 2, kMaxBytes / 2);
  putU32(mode + 6, 1);
  w.put(ana::kTlvDescribeMode, mode, sizeof mode);
  uint8_t range[9];
  putU32(range, kMinTotalHz);
  putU32(range + 4, kMaxTotalHz);
  range[8] = 0;
  w.put(ana::kTlvDescribeRateRange, range, sizeof range);   // one channel; more share the ADC:
  for (uint8_t c = 2; c <= kMaxChannels; ++c) {
    uint8_t limit[6] = {ana::kModeOneShot, c};
    putU32(limit + 2, kMaxTotalHz / c);
    w.put(ana::kTlvDescribeRateLimit, limit, sizeof limit);
  }
  const uint8_t channels[2] = {kMaxChannels, 1u << 4};   // 16-bit slots (bit i: s = 2^i)
  w.put(ana::kTlvDescribeChannels, channels, sizeof channels);
  uint8_t trig[5] = {1u << ana::kTriggerImmediate};      // immediate only, no pretrigger
  w.put(ana::kTlvDescribeTrigger, trig, sizeof trig);
  for (const Frontend &f : kFrontends) {
    uint8_t v[13];
    v[0] = f.number;
    putU32(v + 1, static_cast<uint32_t>(f.min_mv));
    putU32(v + 5, static_cast<uint32_t>(f.max_mv));
    putU32(v + 9, f.attenuation_mdb);
    w.put(ana::kTlvDescribeFrontend, v, sizeof v);
  }
  w.u32(ana::kTlvDescribeMaxRead, static_cast<uint32_t>(max_read_));
  w.u16(ana::kTlvDescribeSegmentRing, 1);
  w.u32(kTagFeatures, 1);                                // bit0 query
  w.u8(kTagImplementation, 3);                           // ADC + DMA
  return w.ok() ? w.length() : 0;
}

// ---- the plan: roles 0 .. C-1 on ADC inputs, each once ------------------------------------------------------------

uint8_t AnalogCapture::planCheck(const RoleAssignment *roles, size_t count) {
  if (state_ == ana::kStateCapturing || count > kMaxChannels) return kRejectUnavailable;
  uint8_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    const uint8_t role = roles[i].role;
    const uint16_t ch = roles[i].channel;
    if (role >= count || ((seen >> role) & 1) || ch > 63 || !((adc_pins_ >> ch) & 1)) return kRejectUnavailable;
    seen |= 1u << role;
  }
  return 0;
}

bool AnalogCapture::planApply(const RoleAssignment *roles, size_t count) {
  for (size_t i = 0; i < kMaxChannels; ++i) pins_[i] = -1;
  for (size_t i = 0; i < count; ++i) pins_[roles[i].role] = roles[i].channel;
  channels_ = static_cast<uint8_t>(count);
  state_ = ana::kStateUnconfigured;   // the plan changed: configure again
  return true;
}

void AnalogCapture::planRelease() {
  if (state_ == ana::kStateCapturing) stopNow();
  channels_ = 0;
  state_ = ana::kStateUnconfigured;
}

// ---- configure ----------------------------------------------------------------------------------------------------

Result AnalogCapture::configure(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity, bool query) {
  if (!query && bound()) return rejected(kRejectUnavailable);   // the group's now
  if (!query && state_ == ana::kStateCapturing) return rejected(kRejectUnavailable);
  static const uint8_t kKnown[] = {ana::kTlvConfigureMode, ana::kTlvConfigureRate, ana::kTlvConfigureSamples,
                                   ana::kTlvConfigureSegments, ana::kTlvConfigureTrigger, ana::kTlvConfigurePretrigger,
                                   ana::kTlvConfigureFrontend};
  Tail tail;
  const Result parsed = tail.parse(payload, length, kKnown, out, capacity);
  if (refused(parsed)) return parsed;
  if (!channels_) return rejected(kRejectUnavailable);   // plan the channels first
  uint8_t len = 0;
  bool critical = false;
  if (const uint8_t *v = tail.find(ana::kTlvConfigureMode, len, &critical)) {
    if (len != 1) return rejected(kRejectMalformed);
    if (v[0] != ana::kModeOneShot) {
      const Result r = tail.refuse(ana::kTlvConfigureMode, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  const uint8_t *rv = tail.find(ana::kTlvConfigureRate, len);
  if (!rv || len != 4 || getU32(rv) == 0) return rejected(kRejectMalformed);
  uint64_t total = static_cast<uint64_t>(getU32(rv)) * channels_;
  if (total < kMinTotalHz) total = kMinTotalHz;
  if (total > kMaxTotalHz) total = kMaxTotalHz;
  uint32_t samples = 1024;
  if (const uint8_t *v = tail.find(ana::kTlvConfigureSamples, len)) {
    if (len != 4 || getU32(v) == 0) return rejected(kRejectMalformed);
    samples = getU32(v);
  }
  const uint32_t most = kMaxBytes / (2u * channels_);
  if (samples > most) samples = most;
  if (const uint8_t *v = tail.find(ana::kTlvConfigureTrigger, len, &critical)) {
    if (len != 4) return rejected(kRejectMalformed);
    if (v[0] != ana::kTriggerImmediate) {
      const Result r = tail.refuse(ana::kTlvConfigureTrigger, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  if (const uint8_t *v = tail.find(ana::kTlvConfigurePretrigger, len, &critical)) {
    if (len != 4) return rejected(kRejectMalformed);
    if (getU32(v)) {
      const Result r = tail.refuse(ana::kTlvConfigurePretrigger, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  uint8_t chosen[kMaxChannels];
  for (uint8_t k = 0; k < kMaxChannels; ++k) chosen[k] = kWidest;
  {   // frontend(role u8, frontend u8), once per channel
    size_t at = 0;
    uint8_t raw = 0, vlen = 0;
    const uint8_t *v = nullptr;
    while (tail.next(at, raw, v, vlen)) {
      if ((raw & ~kTagCritical) != ana::kTlvConfigureFrontend) continue;
      if (vlen != 2) return rejected(kRejectMalformed);
      if (v[0] >= channels_ || !frontendOf(v[1])) return rejected(kRejectUnavailable);
      chosen[v[0]] = v[1];
    }
  }
  // the actual rate
  uint32_t total_hz, num, den;
#if defined(ARDUINO_ARCH_RP2040)
  uint32_t cycles = static_cast<uint32_t>((kAdcClockHz + total / 2) / total);   // whole ADC clock cycles
  if (cycles < 96) cycles = 96;
  total_hz = kAdcClockHz / cycles;
  num = kAdcClockHz;
  den = cycles * channels_;
#else
  total_hz = static_cast<uint32_t>(total);
  num = total_hz;
  den = channels_;
#endif
  if (!query) {
    const size_t bytes = static_cast<size_t>(samples) * channels_ * 2u;
    uint16_t *buffer = static_cast<uint16_t *>(realloc(buffer_, bytes));
    if (!buffer) return failed();
    buffer_ = buffer;
    samples_ = samples;
    total_hz_ = total_hz;
    rate_num_ = num;
    rate_den_ = den;
    memcpy(frontend_, chosen, sizeof frontend_);
#if defined(ARDUINO_ARCH_RP2040)
    cycles_ = cycles;
    // the round robin goes through the inputs in ascending order: frame slot m is the role on the m-th lowest input
    uint8_t sorted[kMaxChannels];
    for (uint8_t k = 0; k < channels_; ++k) sorted[k] = k;
    for (uint8_t a = 0; a < channels_; ++a)
      for (uint8_t b = a + 1; b < channels_; ++b)
        if (pins_[sorted[b]] < pins_[sorted[a]]) { const uint8_t t = sorted[a]; sorted[a] = sorted[b]; sorted[b] = t; }
    memcpy(order_, sorted, sizeof order_);
#else
    for (uint8_t k = 0; k < kMaxChannels; ++k) order_[k] = k;   // the pattern is in role order
#endif
    frames_ = 0;
    state_ = ana::kStateConfigured;
  }
  // the answer
  TlvWriter w(out, capacity);
  uint8_t rate[8];
  putU32(rate, num);
  putU32(rate + 4, den);
  w.put(ana::kTlvConfigureAnswerActualRate, rate, sizeof rate);
  uint8_t layout[4 + kMaxChannels] = {16, 0, 12, channels_};
  for (uint8_t m = 0; m < channels_; ++m) layout[4 + m] = query ? m : order_[m];
  w.put(ana::kTlvConfigureAnswerLayout, layout, 4u + channels_);
  w.u32(ana::kTlvConfigureAnswerActualSamples, samples);
  w.u32(ana::kTlvConfigureAnswerActualSegments, 1);
  const uint8_t timing[5] = {0, 0, 0, 0, 0};   // the ADC's own clock: no jitter to speak of
  w.put(ana::kTlvConfigureAnswerTiming, timing, sizeof timing);
  uint8_t accuracy[5] = {0};                   // computed from the divider
  putU32(accuracy + 1, kRatePpm);
  w.put(ana::kTlvConfigureAnswerRateAccuracy, accuracy, sizeof accuracy);
  w.u32(ana::kTlvConfigureAnswerBlockingMs, 0);
  for (uint8_t k = 0; k < channels_; ++k) {
    const Frontend &f = *frontendOf(chosen[k]);
    uint8_t scale[9] = {k};
    putU32(scale + 1, 0);                                                          // zero
    putU32(scale + 5, static_cast<uint32_t>(static_cast<uint64_t>(f.max_mv) * 1000000u / kFull));   // nV a value
    w.put(ana::kTlvConfigureAnswerScale, scale, sizeof scale);
    uint8_t skew[5] = {k};
    uint8_t slot = 0;
    for (uint8_t m = 0; m < channels_; ++m) if ((query ? m : order_[m]) == k) slot = m;
    putU32(skew + 1, static_cast<uint32_t>(static_cast<uint64_t>(slot) * 1000000000u / total_hz));   // in turn
    w.put(ana::kTlvConfigureAnswerSkew, skew, sizeof skew);
    const uint8_t used[2] = {k, chosen[k]};
    w.put(ana::kTlvConfigureAnswerFrontendUsed, used, sizeof used);
  }
#if defined(ARDUINO_ARCH_ESP32)
  uint8_t ref[6] = {ana::kReferenceSourceInternal};   // the ADC's internal reference, about 1100 mV
  putU32(ref + 1, 1100);
#else
  uint8_t ref[6] = {ana::kReferenceSourceSupply};     // ADC_VREF is the supply
  putU32(ref + 1, 3300);
#endif
  ref[5] = 0;   // nominal
  w.put(ana::kTlvConfigureAnswerReference, ref, sizeof ref);
  return w.ok() ? tail.finish(completed(w.length()), out, capacity) : failed();
}

// ---- calibration: the factory data, raw (§3.8) --------------------------------------------------------------------

Result AnalogCapture::calibration(uint8_t *out, size_t capacity) const {
  TlvWriter w(out, capacity);
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  // ESP-IDF's eFuse ADC calibration of ADC1: the version, then per attenuation the initial code and a reference point
  static const char kScheme[] = "com.espressif.esp32p4.rtc-calib";
  const int ver = esp_efuse_rtc_calib_get_ver();
  for (const Frontend &f : kFrontends) {
    uint32_t digi = 0, mv = 0;
    if (esp_efuse_rtc_calib_get_cal_voltage(ver, 0, f.number, &digi, &mv) != ESP_OK) continue;
    uint8_t v[2 + sizeof kScheme - 1 + 13];
    v[0] = f.number;
    v[1] = sizeof kScheme - 1;
    memcpy(v + 2, kScheme, sizeof kScheme - 1);
    uint8_t *raw = v + 2 + sizeof kScheme - 1;   // version(u8) init_code(u32) digi(u32) mv(u32)
    raw[0] = static_cast<uint8_t>(ver);
    putU32(raw + 1, esp_efuse_rtc_calib_get_init_code(ver, 0, f.number));
    putU32(raw + 5, digi);
    putU32(raw + 9, mv);
    w.put(ana::kTlvCalibrationAnswerFactory, v, sizeof v);
  }
#elif defined(CONFIG_IDF_TARGET_ESP32)
  // the eFuse fields as they are: ADC_VREF (5 bits), ADC1_TP_LOW (7), ADC1_TP_HIGH (9); 0 = not burnt
  static const char kScheme[] = "com.espressif.esp32.adc-efuse";
  uint8_t vref = 0, low = 0;
  uint16_t high = 0;
  esp_efuse_read_field_blob(ESP_EFUSE_ADC_VREF, &vref, 5);
  esp_efuse_read_field_blob(ESP_EFUSE_ADC1_TP_LOW, &low, 7);
  esp_efuse_read_field_blob(ESP_EFUSE_ADC1_TP_HIGH, &high, 9);
  uint8_t v[2 + sizeof kScheme - 1 + 4];
  v[0] = 0xff;   // not per frontend
  v[1] = sizeof kScheme - 1;
  memcpy(v + 2, kScheme, sizeof kScheme - 1);
  uint8_t *raw = v + 2 + sizeof kScheme - 1;     // vref(u8) tp_low(u8) tp_high(u16)
  raw[0] = vref;
  raw[1] = low;
  putU16(raw + 2, high);
  w.put(ana::kTlvCalibrationAnswerFactory, v, sizeof v);
#endif
  // no Vrefint: the continuous mode here cannot measure the internal reference in the same capture
  return w.ok() ? completed(w.length()) : failed();
}

// ---- running ------------------------------------------------------------------------------------------------------

bool AnalogCapture::trackReady() const {
  return state_ == ana::kStateConfigured || state_ == ana::kStateDone;
}

bool AnalogCapture::startNow() {
  if (!trackReady() || !buffer_) return false;
  frames_ = 0;
  short_ = false;
  reported_ = false;
#if defined(ARDUINO_ARCH_ESP32)
  adc_continuous_handle_cfg_t hc = {};
  hc.max_store_buf_size = 32 * kFrameBytes;   // 8 KiB: tens of ms of conversions between two polls
  hc.conv_frame_size = kFrameBytes;
  if (adc_continuous_new_handle(&hc, &handle_) != ESP_OK) { handle_ = nullptr; state_ = ana::kStateError; return false; }
  adc_digi_pattern_config_t pattern[kMaxChannels] = {};
  for (uint8_t m = 0; m < channels_; ++m) {
    adc_unit_t unit;
    adc_channel_t channel;
    if (adc_continuous_io_to_channel(pins_[order_[m]], &unit, &channel) != ESP_OK || unit != ADC_UNIT_1) {
      adc_continuous_deinit(handle_);
      handle_ = nullptr;
      state_ = ana::kStateError;
      return false;
    }
    pattern[m].atten = frontend_[order_[m]];
    pattern[m].channel = channel;
    pattern[m].unit = ADC_UNIT_1;
    pattern[m].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    adc_channel_[m] = static_cast<uint8_t>(channel);
    counts_[m] = 0;
  }
  adc_continuous_config_t config = {};
  config.pattern_num = channels_;
  config.adc_pattern = pattern;
  config.sample_freq_hz = total_hz_;
  config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  config.format = kRecordBytes == 2 ? ADC_DIGI_OUTPUT_FORMAT_TYPE1 : ADC_DIGI_OUTPUT_FORMAT_TYPE2;
  adc_continuous_evt_cbs_t callbacks = {};
  callbacks.on_pool_ovf = onOverflow;
  overflow_ = false;
  if (adc_continuous_config(handle_, &config) != ESP_OK ||
      adc_continuous_register_event_callbacks(handle_, &callbacks, this) != ESP_OK) {
    adc_continuous_deinit(handle_);
    handle_ = nullptr;
    state_ = ana::kStateError;
    return false;
  }
  const uint64_t frame_ns = static_cast<uint64_t>(kFrameConversions) * 1000000000u / total_hz_;
  start_ns_ = nowNs();
  if (adc_continuous_start(handle_) != ESP_OK) {
    adc_continuous_deinit(handle_);
    handle_ = nullptr;
    state_ = ana::kStateError;
    return false;
  }
  // The P4's driver leaves out the first conversion frame: its first value is one frame after the start. Corrected,
  // it matched to about 0.1 ms (logic-capture §7.4). Uncorrected (the classic ESP32, not measured), up to a frame off.
  if (kFirstFrameLost) {
    start_ns_ += frame_ns;
    uncertainty_ns_ = 200000;
  } else {
    uncertainty_ns_ = static_cast<uint32_t>(frame_ns + 100000);
  }
#else
  if (!gAdcReady) { adc_init(); gAdcReady = true; }
  uint32_t mask = 0;
  int lowest = 99;
  for (uint8_t k = 0; k < channels_; ++k) {
    adc_gpio_init(pins_[k]);
    const int input = pins_[k] - ADC_BASE_PIN;
    mask |= 1u << input;
    if (input < lowest) lowest = input;
  }
  adc_run(false);
  adc_fifo_drain();
  adc_select_input(lowest);
  adc_set_round_robin(channels_ > 1 ? mask : 0);
  adc_fifo_setup(true, true, 1, false, false);
  adc_set_clkdiv(static_cast<float>(cycles_ - 1));
  if (dma_ < 0) dma_ = dma_claim_unused_channel(false);
  if (dma_ < 0) { state_ = ana::kStateError; return false; }
  dma_channel_config c = dma_channel_get_default_config(dma_);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, DREQ_ADC);
  dma_channel_configure(dma_, &c, buffer_, &adc_hw->fifo, samples_ * channels_, true);
  start_ns_ = nowNs() + 2000;   // the first conversion ends about 96 cycles (2 us) after the run starts
  uncertainty_ns_ = 5000;
  adc_run(true);
#endif
  state_ = ana::kStateCapturing;
  return true;
}

void AnalogCapture::finish() {
#if defined(ARDUINO_ARCH_ESP32)
  if (handle_) {
    adc_continuous_stop(handle_);
    adc_continuous_deinit(handle_);
    handle_ = nullptr;
  }
#else
  adc_run(false);
  if (dma_ >= 0) {
    const uint32_t left = dma_channel_hw_addr(dma_)->transfer_count;
    dma_channel_abort(dma_);
    frames_ = (samples_ * channels_ - left) / channels_;
  }
  adc_fifo_drain();
  adc_set_round_robin(0);
#endif
  state_ = ana::kStateDone;
}

void AnalogCapture::stopNow() {
  if (state_ != ana::kStateCapturing) return;
#if defined(ARDUINO_ARCH_ESP32)
  drain();
#endif
  finish();
  short_ = frames_ < samples_;
  if (subscribed_ && !reported_) {
    const uint8_t reason = ana::kStoppedReasonHost;
    endpoint_.event(*this, ana::kEventStopped, &reason, 1);
    reported_ = true;
  }
}

#if defined(ARDUINO_ARCH_ESP32)
bool IRAM_ATTR AnalogCapture::onOverflow(adc_continuous_handle_t, const adc_continuous_evt_data_t *, void *context) {
  static_cast<AnalogCapture *>(context)->overflow_ = true;
  return false;
}

void AnalogCapture::drain() {
  uint8_t records[kFrameBytes];
  uint32_t got = 0;
  while (handle_ && adc_continuous_read(handle_, records, sizeof records, &got, 0) == ESP_OK && got) {
    for (uint32_t at = 0; at + kRecordBytes <= got; at += kRecordBytes) {
      const adc_digi_output_data_t *r = reinterpret_cast<const adc_digi_output_data_t *>(records + at);
#if defined(CONFIG_IDF_TARGET_ESP32)
      const uint8_t channel = r->type1.channel;
      const uint16_t value = r->type1.data;
#else
      if (r->type2.unit != 0) continue;
      const uint8_t channel = r->type2.channel;
      const uint16_t value = r->type2.data;
#endif
      for (uint8_t m = 0; m < channels_; ++m) {
        if (adc_channel_[m] != channel) continue;
        if (counts_[m] < samples_) buffer_[static_cast<size_t>(counts_[m]) * channels_ + m] = value;
        ++counts_[m];
        break;
      }
    }
    uint32_t low = samples_;
    for (uint8_t m = 0; m < channels_; ++m) if (counts_[m] < low) low = counts_[m];
    frames_ = low;
    if (frames_ >= samples_) break;
  }
}
#endif

void AnalogCapture::poll() {
  if (state_ == ana::kStateCapturing) {
#if defined(ARDUINO_ARCH_ESP32)
    drain();
    if (frames_ >= samples_) finish();
#else
    if (!dma_channel_is_busy(dma_)) {
      frames_ = samples_;
      finish();
    }
#endif
  }
  if (state_ == ana::kStateDone && !reported_) {
    reported_ = true;
    if (subscribed_) {
      uint8_t seg[33];
      endpoint_.event(*this, ana::kEventSegment, seg, segmentInfo(seg));
      const uint8_t reason = ana::kStoppedReasonComplete;
      endpoint_.event(*this, ana::kEventStopped, &reason, 1);
    }
  }
}

size_t AnalogCapture::segmentInfo(uint8_t *out) const {
  putU32(out, 0);                  // serial
  putU64(out + 4, 0);              // position
  putU32(out + 12, frames_);
  putU64(out + 16, start_ns_);
  putU32(out + 24, uncertainty_ns_);
  putU32(out + 28, 0xFFFFFFFFu);   // no trigger inside (immediate start)
  uint8_t flags = short_ ? ana::kSegmentFlagShort : 0;
#if defined(ARDUINO_ARCH_ESP32)
  if (overflow_) flags |= ana::kSegmentFlagSlipped;   // conversions were lost: the values after it come late
#endif
  out[32] = flags;
  return 33;
}

// ---- the operations -----------------------------------------------------------------------------------------------

Result AnalogCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case ana::kOpConfigure: return configure(p, n, out, capacity, false);
    case ana::kOpQuery: return configure(p, n, out, capacity, true);
    case ana::kOpStart: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (bound()) return rejected(kRejectUnavailable);   // the group starts it
      if (!trackReady() || capacity < 4) return rejected(kRejectUnavailable);
      if (!startNow()) return failed();
      putU32(out, 0);   // blocking_ms: DMA, the probe keeps answering
      return tail.finish(completed(4), out, capacity);
    }
    case ana::kOpStop: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (bound()) return rejected(kRejectUnavailable);
      stopNow();
      return tail.finish(completed(), out, capacity);
    }
    case ana::kOpStatus: {   // -> state(u8) serial_done(u32) write_pos(u64) flags(u8)
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 14) return failed();
      poll();
      out[0] = state_;
      putU32(out + 1, state_ == ana::kStateDone ? 1 : 0);
      putU64(out + 5, static_cast<uint64_t>(frames_) * channels_ * 2u);
      out[13] = 0;
      return tail.finish(completed(14), out, capacity);
    }
    case ana::kOpRead: {   // position(u64) max(u32) -> position(u64) flags(u8: bit0 more) data
      if (n < 12) return rejected(kRejectMalformed);
      const uint64_t position = getU64(p);
      const uint32_t most = getU32(p + 8);
      const uint64_t end = static_cast<uint64_t>(frames_) * channels_ * 2u;
      if (capacity < 9) return failed();
      size_t take = position < end ? static_cast<size_t>(end - position) : 0;
      if (take > most) take = most;
      if (take > max_read_) take = max_read_;
      if (take > capacity - 9) take = capacity - 9;
      putU64(out, position);
      out[8] = position + take < end ? 1 : 0;
      if (take) memcpy(out + 9, reinterpret_cast<const uint8_t *>(buffer_) + position, take);   // little endian
      return completed(9 + take);
    }
    case ana::kOpSegments: {   // from_serial(u32) -> count(u8) segments
      if (n < 4) return rejected(kRejectMalformed);
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 1 + 33) return failed();
      poll();
      const bool one = state_ == ana::kStateDone && getU32(p) == 0;
      out[0] = one ? 1 : 0;
      return tail.finish(completed(1 + (one ? segmentInfo(out + 1) : 0)), out, capacity);
    }
    case ana::kOpCalibration: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      const Result r = calibration(out, capacity);
      return refused(r) ? r : tail.finish(r, out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);   // force, release: not in one-shot with an immediate trigger
  }
}

}  // namespace oep

#endif
