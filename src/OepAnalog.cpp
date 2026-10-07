// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepAnalog.h"

#include "OepEndpoint.h"

#include <algorithm>

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_timer.h>
#if defined(CONFIG_IDF_TARGET_ESP32)
#include <esp_efuse.h>
#include <esp_efuse_table.h>
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
#include <esp_efuse_rtc_calib.h>
#endif
#elif defined(OEP_ANALOG_RP2)
#include <hardware/adc.h>
#include <hardware/dma.h>
#endif

#if defined(ARDUINO_ARCH_ESP32) || defined(OEP_ANALOG_RP2)

namespace oep {
namespace ana = reg::fixture_analog;

namespace {

struct Frontend { uint8_t number; int32_t min_mv, max_mv; uint32_t attenuation_mdb; };

// The input ranges (nominal, from the chip's documentation) and the conversions a second the ADC takes in all.
#if defined(CONFIG_IDF_TARGET_ESP32)
constexpr Frontend kFrontends[] = {{0, 100, 950, 0}, {1, 100, 1250, 2500}, {2, 150, 1750, 6000}, {3, 150, 2450, 12000}};
constexpr uint32_t kMinTotalHz = 20000, kMaxTotalHz = 100000;    // the driver's floor; a ceiling kept low (DMA to RAM)
constexpr size_t kRecordBytes = 2;
constexpr bool kFirstFrameLost = false;                          // the first value is kept (see startNow)
constexpr uint32_t kStartLagNs = 100000;                         // measured: the first value ~100 us after the start
constexpr uint32_t kPretriggerRoom = 129;                        // see configure
#elif defined(ARDUINO_ARCH_ESP32)
constexpr Frontend kFrontends[] = {{0, 0, 950, 0}, {1, 0, 1250, 2500}, {2, 0, 1750, 6000}, {3, 0, 3100, 12000}};
constexpr uint32_t kMinTotalHz = 611, kMaxTotalHz = 46000;       // above 46 kHz the P4 gives each value twice
constexpr size_t kRecordBytes = 4;
constexpr bool kFirstFrameLost = true;                           // measured on the P4 (logic-capture §7.4)
// Triggered, the segment's buffer is the ring: one channel's values can run up to a driver read (128 records) ahead
// of the frame being looked at, into the ring's slots of older frames; the pretrigger leaves them out.
constexpr uint32_t kPretriggerRoom = 129;
#else
constexpr Frontend kFrontends[] = {{0, 0, 3300, 0}};             // ADC_VREF: the supply (3.3 V on a Pico)
constexpr uint32_t kAdcClockHz = 48000000;
constexpr uint32_t kMinTotalHz = kAdcClockHz / 65536 + 1, kMaxTotalHz = kAdcClockHz / 96;
constexpr uint32_t kPretriggerRoom = 1;
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

#if defined(OEP_ANALOG_RP2)
bool gAdcReady = false;
#endif

}  // namespace

// describe's max_pretrigger: a triggered segment of one channel less kPretriggerRoom (configure also keeps it that far
// below the segment's samples)
uint32_t AnalogCapture::maxPretrigger() {
#if defined(OEP_ANALOG_RP2)
  return kRingBytes / 4 - kPretriggerRoom;   // a triggered segment is at most half the ring (one channel)
#else
  return static_cast<uint32_t>(kMaxBytes / 2) - kPretriggerRoom;
#endif
}

size_t AnalogCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  static const uint8_t kRoles[] = {0, 1, 2, 3};
  w.roleChannels(kRoles, kMaxChannels, adc_pins_);
  uint8_t mode[9] = {ana::kModeOneShot};       // mode(u8) max_samples(u32) max_segments(u32)
  putU32(mode + 1, kMaxBytes / 2);
  putU32(mode + 5, 1);
  w.put(ana::kTlvDescribeMode, mode, sizeof mode);
  uint8_t range[9];
  putU32(range, kMinTotalHz);
  putU32(range + 4, kMaxTotalHz);
  range[8] = 0;
  w.put(ana::kTlvDescribeRateRange, range, sizeof range);   // one channel (more share the ADC: configure says)
  w.u8(ana::kTlvDescribeChannels, kMaxChannels);             // max(u8)
  uint8_t trig[8];   // types(u32) max_pretrigger(u32)
  putU32(trig, (1u << ana::kTriggerImmediate) | (1u << ana::kTriggerCrossUp) | (1u << ana::kTriggerCrossDown));
  putU32(trig + 4, maxPretrigger());
  w.put(ana::kTlvDescribeTrigger, trig, sizeof trig);
  for (const Frontend &f : kFrontends) {
    uint8_t v[13];
    v[0] = f.number;
    putU32(v + 1, static_cast<uint32_t>(f.min_mv));
    putU32(v + 5, static_cast<uint32_t>(f.max_mv));
    putU32(v + 9, f.attenuation_mdb);
    w.put(ana::kTlvDescribeFrontend, v, sizeof v);
  }
  // no features: revision 1 defines no bit (query, force, subscribe and unsubscribe are in the ops tag)
  return w.ok() ? w.length() : 0;
}

// ---- the plan: roles 0 .. C-1 on ADC inputs, each once ------------------------------------------------------------

uint8_t AnalogCapture::planCheck(const RoleAssignment *roles, size_t count) {
  refusal_cause_ = reg::core::kUnavailableCauseWrongState;
  refusal_channel_ = 0xFFFF;
  if (state_ == ana::kStateCapturing || state_ == ana::kStateWaiting) return kRejectUnavailable;
  refusal_cause_ = reg::core::kUnavailableCauseLimit;   // the roles: more than its channels, one twice, one skipped
  if (count > kMaxChannels) return kRejectUnavailable;
  uint8_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    const uint8_t role = roles[i].role;
    const uint16_t ch = roles[i].channel;
    // a role it does not have, a pin without an ADC channel (outside role_channels): unsupported (core §8, capture §3.1)
    if (role >= kMaxChannels || ch > 63 || !((adc_pins_ >> ch) & 1)) return kRejectUnsupported;
    if (role >= count || ((seen >> role) & 1)) return kRejectUnavailable;
    if (table_ && table_->owner(ch) != 0 && table_->owner(ch) != owner_) return kRejectUnavailable;   // a wire, a slot
    seen |= 1u << role;
  }
  // a channel whose idle is an output (mode 3 / 4): the ADC would take the pad off its drive (core §8, capture §1.2)
  for (size_t i = 0; i < count && table_; ++i) {
    const uint8_t idle = table_->idle(roles[i].channel);
    if (idle != PinTable::kIdleOutputLow && idle != PinTable::kIdleOutputHigh) continue;
    refusal_cause_ = reg::core::kUnavailableCauseHeldBySettings;
    refusal_channel_ = roles[i].channel;
    return kRejectUnavailable;
  }
  return 0;
}

bool AnalogCapture::planApply(const RoleAssignment *roles, size_t count) {
  for (size_t i = 0; i < kMaxChannels; ++i) pins_[i] = -1;
  if (table_) table_->releaseQuiet(owner_);
  for (size_t i = 0; i < count; ++i) {
    pins_[roles[i].role] = roles[i].channel;
    if (table_ && table_->allowed(roles[i].channel)) table_->claim(roles[i].channel, owner_);
  }
  channels_ = static_cast<uint8_t>(count);
  forget();   // the plan changed: configure again
  return true;
}

void AnalogCapture::planRelease() {
  stopNow();
  if (table_) table_->release(owner_);   // to their idle state, now that the ADC is off them
  channels_ = 0;
  forget();
}

// The plan released or replaced (capture §3.2): state 0, the configuration, the data and the segment gone - a read
// finds nothing (the buffer was sized for the old channels), status and segments count none.
void AnalogCapture::forget() {
  state_ = ana::kStateUnconfigured;
  frames_ = samples_ = 0;
  short_ = trig_slipped_ = follow_ = segment_ = lost_ = false;
  trig_type_ = 0;
  pretrigger_ = 0;
  phase_ = 0;
#if defined(ARDUINO_ARCH_ESP32)
  overflow_ = overflow_seen_ = false;
#endif
}

// ---- configure ----------------------------------------------------------------------------------------------------

Result AnalogCapture::configure(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity, bool query) {
  // core §2.3: each TLV this configure implements is checked the same with or without bit 7 - a length other than its
  // definition, a value the definition excludes or two frontends for one role is malformed; a value the definition
  // leaves unused or this probe cannot honour refuses the whole configure unsupported, the tag as received. An unknown
  // critical tag is unsupported, an unknown non-critical one passed over. Everything is checked before anything
  // changes (core §4.3): the form, then the values, then the plan, the group and the state (unavailable).
  static const uint8_t kKnown[] = {ana::kTlvConfigureMode, ana::kTlvConfigureRate, ana::kTlvConfigureSamples,
                                   ana::kTlvConfigureSegments, ana::kTlvConfigureTrigger, ana::kTlvConfigurePretrigger,
                                   ana::kTlvConfigureFrontend};
  Tail tail;
  Result unknown_critical;
  const Result parsed = tail.parse(payload, length, kKnown, out, capacity, &unknown_critical);
  if (refused(parsed)) return parsed;
  struct Item { uint8_t tag; size_t size; const uint8_t *v; bool critical; };
  Item items[] = {{ana::kTlvConfigureMode, 1, nullptr, false},     {ana::kTlvConfigureRate, 4, nullptr, false},
                  {ana::kTlvConfigureSamples, 4, nullptr, false},  {ana::kTlvConfigureSegments, 4, nullptr, false},
                  {ana::kTlvConfigureTrigger, 6, nullptr, false},  {ana::kTlvConfigurePretrigger, 4, nullptr, false}};
  Item &mode_tlv = items[0], &rate_tlv = items[1], &samples_tlv = items[2], &segments_tlv = items[3],
       &trigger_tlv = items[4], &pretrigger_tlv = items[5];
  for (Item &it : items) {   // a length other than its definition: malformed (core §2.3)
    const Result r = tail.fixed(it.tag, it.size, it.v, out, capacity, &it.critical);
    if (refused(r)) return r;
  }
  // capture §3.3's contract: mode and rate required, samples in mode 1 / 2; none of them 0 (malformed)
  auto ask = [](const Item &it) { return CaptureAsk{it.v, it.critical}; };
  if (captureMalformed(ask(mode_tlv), ask(rate_tlv), ask(samples_tlv), ask(segments_tlv))) return rejected(kRejectMalformed);
  {   // frontend(role u8, frontend u8), repeated: one per role (capture §3.3)
    size_t at = 0, vlen = 0;
    uint8_t raw = 0;
    const uint8_t *v = nullptr;
    uint32_t roles = 0;
    while (tail.next(at, raw, v, vlen)) {
      if ((raw & ~kTagCritical) != ana::kTlvConfigureFrontend) continue;
      if (vlen != 2) return rejected(kRejectMalformed);
      if (v[0] < 32 && ((roles >> v[0]) & 1)) return rejected(kRejectMalformed);   // two for the same role
      if (v[0] < 32) roles |= 1u << v[0];
    }
  }
  if (refused(unknown_critical)) return unknown_critical;
  // a channel count for what follows; without a plan, one (the plan's refusal follows)
  const uint8_t count = channels_ ? channels_ : 1;
  const uint8_t planned = channels_ ? channels_ : kMaxChannels;
  if (mode_tlv.v[0] != ana::kModeOneShot)
    return Tail::refuse(ana::kTlvConfigureMode, mode_tlv.critical, out, capacity);
  // a rate outside rate_range is refused (capture §3.3); inside it, more channels share the ADC: the nearest its
  // conversions a second allow for the channel count
  const uint32_t asked = getU32(rate_tlv.v);
  if (asked < kMinTotalHz || asked > kMaxTotalHz)
    return Tail::refuse(ana::kTlvConfigureRate, rate_tlv.critical, out, capacity);
  uint64_t total = static_cast<uint64_t>(asked) * count;
  if (total > kMaxTotalHz) total = kMaxTotalHz;
  // segments outside repeat, a pretrigger without a trigger: unsupported whatever the value
  const Result misplaced = captureMisplaced(mode_tlv.v[0], ask(samples_tlv), ask(segments_tlv), ask(trigger_tlv),
                                            ask(pretrigger_tlv), out, capacity);
  if (refused(misplaced)) return misplaced;
  uint32_t samples = getU32(samples_tlv.v);
  // type(u8) role(u8) value(u32): the ADC value crossed up (from below to at or above) or down (above to at or below).
  // A role outside the plan is refused unavailable cause 6 below, after every unsupported value (core §4.3).
  uint8_t trig_type = ana::kTriggerImmediate, trig_role = 0;
  uint32_t trig_value = 0;
  if (const uint8_t *v = trigger_tlv.v) {
    const uint32_t value = getU32(v + 2);
    const bool ok = v[0] == ana::kTriggerImmediate ||
                    ((v[0] == ana::kTriggerCrossUp || v[0] == ana::kTriggerCrossDown) && value <= kFull);
    if (!ok) return Tail::refuse(ana::kTlvConfigureTrigger, trigger_tlv.critical, out, capacity);
    trig_type = v[0];
    trig_role = v[1];
    trig_value = value;
  }
  uint32_t most = kMaxBytes / (2u * count);
#if defined(OEP_ANALOG_RP2)
  if (trig_type) most = kRingBytes / 4 / count;   // half the ring: the DMA runs on while poll sees it is full
#endif
  if (samples > most) samples = most;
  uint32_t pretrigger = 0;
  if (const uint8_t *v = pretrigger_tlv.v) {   // only with a trigger (above), in the segment: less than samples and
    pretrigger = getU32(v);                    // max_pretrigger, and (ESP32) kPretriggerRoom short of the segment's end
    if (pretrigger > maxPretrigger() || pretrigger + kPretriggerRoom > samples)
      return Tail::refuse(ana::kTlvConfigurePretrigger, pretrigger_tlv.critical, out, capacity);
  }
  uint8_t chosen[kMaxChannels];
  for (uint8_t k = 0; k < kMaxChannels; ++k) chosen[k] = kWidest;
  {   // frontend(role u8, frontend u8): a role it does not have or not planned, a frontend it does not declare
    size_t at = 0, vlen = 0;
    uint8_t raw = 0;
    const uint8_t *v = nullptr;
    while (tail.next(at, raw, v, vlen)) {
      if ((raw & ~kTagCritical) != ana::kTlvConfigureFrontend) continue;
      if (v[0] >= planned || !frontendOf(v[1])) return unsupportedTag(out, capacity, raw);   // the tag as received
      chosen[v[0]] = v[1];
    }
  }
  // no plan, or a trigger's role not in it (type 0's role is not looked at): unavailable cause 6 (capture §3.2, §3.3)
  if (!channels_ || (trig_type != 0 && trig_role >= channels_)) return wrongState(out, capacity);
  if (!query && bound()) return boundInGroup(*this, out, capacity);   // the group's now (cause 4)
  if (!query && (state_ == ana::kStateCapturing || state_ == ana::kStateWaiting)) return wrongState(out, capacity);
  // the actual rate
  uint32_t total_hz, num, den;
#if defined(OEP_ANALOG_RP2)
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
  // the frame's order (layout order[m]: the role in slot m), the same for query as configure would answer
  uint8_t order[kMaxChannels] = {0, 1, 2, 3};   // the ESP32's pattern is in role order
#if defined(OEP_ANALOG_RP2)
  // the round robin goes through the inputs in ascending order: frame slot m is the role on the m-th lowest input
  for (uint8_t a = 0; a < channels_; ++a)
    for (uint8_t b = a + 1; b < channels_; ++b)
      if (pins_[order[b]] < pins_[order[a]]) { const uint8_t t = order[a]; order[a] = order[b]; order[b] = t; }
#endif
  if (!query) {
    // Memory it cannot get is refused unavailable cause 3 (core §4.3, the resources; it answered failed with an empty
    // payload), with the earlier configuration left as it was: the ring first (taken once, kept), then the segment's
    // buffer (realloc keeps the old one when it fails). The ring's failure after a resize left the old samples_ over a
    // smaller buffer.
#if defined(OEP_ANALOG_RP2)
    if (trig_type && !ring_) ring_ = static_cast<uint16_t *>(aligned_alloc(kRingBytes, kRingBytes));   // the DMA's ring
    if (trig_type && !ring_) return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
#endif
    const size_t bytes = static_cast<size_t>(samples) * channels_ * 2u;
    uint16_t *buffer = static_cast<uint16_t *>(realloc(buffer_, bytes));
    if (!buffer) return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
    buffer_ = buffer;
    trig_type_ = trig_type;
    trig_value_ = trig_value;
    pretrigger_ = pretrigger;
    arm_ = pretrigger ? pretrigger : 1;   // a crossing needs the value before it
    samples_ = samples;
    total_hz_ = total_hz;
    rate_num_ = num;
    rate_den_ = den;
    memcpy(frontend_, chosen, sizeof frontend_);
#if defined(OEP_ANALOG_RP2)
    cycles_ = cycles;
#endif
    memcpy(order_, order, sizeof order_);
    for (uint8_t m = 0; m < channels_; ++m) if (order_[m] == trig_role) trig_slot_ = m;
    frames_ = 0;
    segment_ = false;   // the buffer is the new configuration's
    state_ = ana::kStateConfigured;
  }
  // the answer
  TlvWriter w(out, capacity);
  uint8_t rate[8];
  putU32(rate, num);
  putU32(rate + 4, den);
  w.put(ana::kTlvConfigureAnswerActualRate, rate, sizeof rate);
  uint8_t layout[4 + kMaxChannels] = {16, 0, 12, channels_};
  for (uint8_t m = 0; m < channels_; ++m) layout[4 + m] = order[m];
  w.put(ana::kTlvConfigureAnswerLayout, layout, 4u + channels_);
  w.u32(ana::kTlvConfigureAnswerActualSamples, samples);   // one-shot: no actual_segments (capture §3.3)
  w.u32(ana::kTlvConfigureAnswerBlockingMs, 0);
  for (uint8_t k = 0; k < channels_; ++k) {
    const Frontend &f = *frontendOf(chosen[k]);
    uint8_t scale[9] = {k};
    int32_t zero = 0, scale_nv = 0;   // the frontend's whole range over 0 .. kFull (min_mv may be above 0 V)
    analogScale(f.min_mv, f.max_mv, kFull, zero, scale_nv);
    putU32(scale + 1, static_cast<uint32_t>(zero));
    putU32(scale + 5, static_cast<uint32_t>(scale_nv));
    w.put(ana::kTlvConfigureAnswerScale, scale, sizeof scale);
    uint8_t skew[5] = {k};
    uint8_t slot = 0;
    for (uint8_t m = 0; m < channels_; ++m) if (order[m] == k) slot = m;
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
  return w.ok() ? completed(w.length()) : failed();
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
    uint8_t v[2 + sizeof kScheme - 1 + 2 + 13];   // frontend scheme_len scheme raw_len(u16) raw
    v[0] = f.number;
    v[1] = sizeof kScheme - 1;
    memcpy(v + 2, kScheme, sizeof kScheme - 1);
    putU16(v + 2 + sizeof kScheme - 1, 13);
    uint8_t *raw = v + 2 + sizeof kScheme - 1 + 2;   // version(u8) init_code(u32) digi(u32) mv(u32)
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
  uint8_t v[2 + sizeof kScheme - 1 + 2 + 4];   // frontend scheme_len scheme raw_len(u16) raw
  v[0] = 0xff;   // not per frontend
  v[1] = sizeof kScheme - 1;
  memcpy(v + 2, kScheme, sizeof kScheme - 1);
  putU16(v + 2 + sizeof kScheme - 1, 4);
  uint8_t *raw = v + 2 + sizeof kScheme - 1 + 2;     // vref(u8) tp_low(u8) tp_high(u16)
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
  if (!(trackReady() || state_ == ana::kStateError) || !buffer_) return false;
  generation_ = nextGeneration(generation_);   // 1 after 0xFFFFFFFF, never 0 (capture §3.2)
  segment_ = false;   // the last generation's segment goes
  frames_ = got_ = searched_ = 0;
  short_ = have_prev_ = force_ = trig_slipped_ = lost_ = false;
  error_ = ana::kErrorPeripheral;
  reported_ = false;
  trig_reported_ = trig_type_ == 0 || follow_;   // a follower's trigger is the group's event
  ext_ready_ = false;
  phase_ = 0;
  end_frame_ = ringMode() ? UINT32_MAX : samples_;
#if defined(OEP_ANALOG_RP2)
  ring_len_ = ringMode() ? kRingBytes / 2 : samples_ * channels_;
#else
  ring_len_ = samples_ * channels_;
#endif
#if defined(ARDUINO_ARCH_ESP32)
  adc_continuous_handle_cfg_t hc = {};
  hc.max_store_buf_size = 32 * kFrameBytes;   // 8 KiB: tens of ms of conversions between two polls
  hc.conv_frame_size = kFrameBytes;
  if (adc_continuous_new_handle(&hc, &handle_) != ESP_OK) { handle_ = nullptr; fail(ana::kErrorPeripheral); return false; }
  adc_digi_pattern_config_t pattern[kMaxChannels] = {};
  for (uint8_t m = 0; m < channels_; ++m) {
    adc_unit_t unit;
    adc_channel_t channel;
    if (adc_continuous_io_to_channel(pins_[order_[m]], &unit, &channel) != ESP_OK || unit != ADC_UNIT_1) {
      adc_continuous_deinit(handle_);
      handle_ = nullptr;
      fail(ana::kErrorPeripheral);
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
  overflow_ = overflow_seen_ = false;
  if (adc_continuous_config(handle_, &config) != ESP_OK ||
      adc_continuous_register_event_callbacks(handle_, &callbacks, this) != ESP_OK) {
    adc_continuous_deinit(handle_);
    handle_ = nullptr;
    fail(ana::kErrorPeripheral);
    return false;
  }
  const uint64_t frame_ns = static_cast<uint64_t>(kFrameConversions) * 1000000000u / total_hz_;
  start_ns_ = nowNs();
  if (adc_continuous_start(handle_) != ESP_OK) {
    adc_continuous_deinit(handle_);
    handle_ = nullptr;
    fail(ana::kErrorPeripheral);
    return false;
  }
  // The P4's driver leaves out the first conversion frame: its first value is one frame after the start. Corrected,
  // it matched to about 0.1 ms (logic-capture §7.4). The classic ESP32 keeps it, but its values came about 100 us
  // before the logic's for the same edge, at 10, 20 and 40 kS/s alike (50-200 us: a time, not a count of samples,
  // within one sample either way; a capture-group on the V003 jig, 0.0.16, 2026-09-30).
  if (kFirstFrameLost) {
    start_ns_ += frame_ns;
    uncertainty_ns_ = 200000;
  } else {
#if defined(CONFIG_IDF_TARGET_ESP32)
    start_ns_ += kStartLagNs;
    uncertainty_ns_ = static_cast<uint32_t>(kStartLagNs + static_cast<uint64_t>(channels_) * 1000000000u / total_hz_);
#else
    uncertainty_ns_ = static_cast<uint32_t>(frame_ns + 100000);
#endif
  }
#else
  if (!gAdcReady) { adc_init(); gAdcReady = true; }
  for (uint8_t k = 0; k < channels_; ++k) adc_gpio_init(pins_[k]);
  if (dma_ < 0) dma_ = dma_claim_unused_channel(false);
  if (dma_ < 0) { fail(ana::kErrorPeripheral); return false; }
  if (ringMode()) armDma(ring_, kRingCount, true);
  else armDma(buffer_, samples_ * channels_, false);
#endif
  state_ = ringMode() ? ana::kStateWaiting : ana::kStateCapturing;
  return true;
}

#if defined(OEP_ANALOG_RP2)
// The round robin from the lowest input, its FIFO copied by DMA into `to`: `count` values, or on and on round a
// kRingBytes ring (to aligned to it).
void AnalogCapture::armDma(uint16_t *to, uint32_t count, bool ring) {
  uint32_t mask = 0;
  int lowest = 99;
  for (uint8_t k = 0; k < channels_; ++k) {
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
  dma_channel_config c = dma_channel_get_default_config(dma_);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  if (ring) channel_config_set_ring(&c, true, 15);   // the write address wraps at 2^15 bytes
  channel_config_set_dreq(&c, DREQ_ADC);
  dma_channel_configure(dma_, &c, to, &adc_hw->fifo, count, true);
  start_ns_ = nowNs() + 2000;   // the first conversion ends about 96 cycles (2 us) after the run starts
  uncertainty_ns_ = 5000;
  adc_run(true);
}
#endif

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
  segment_ = true;
}

// stop (capture §3.2): waiting for the trigger (state 2) -> 1 with nothing captured; capturing (state 3) -> 1 with the
// segment cut short (flags bit1) holding what came in; stopped 1 either way. A capture that completed meanwhile is
// complete (state 4, stopped 0).
void AnalogCapture::stopNow() {
  if (state_ != ana::kStateWaiting && state_ != ana::kStateCapturing) return;
  poll();   // what has come in
  if (state_ != ana::kStateWaiting && state_ != ana::kStateCapturing) return;
  if (state_ == ana::kStateWaiting) {   // no trigger yet: no segment
#if defined(ARDUINO_ARCH_ESP32)
    finish();
#else
    adc_run(false);
    if (dma_ >= 0) dma_channel_abort(dma_);
    adc_fifo_drain();
    adc_set_round_robin(0);
#endif
    frames_ = 0;
    segment_ = false;
  } else if (ringMode()) {
    finishTriggered(true);
  } else {
#if defined(ARDUINO_ARCH_ESP32)
    drain();
#endif
    finish();
    segment_ = frames_ > 0;
  }
  if (segment_ && hole()) { failHole(); return; }   // stopped on a hole: not handed out (capture §2.2)
  short_ = segment_;
  state_ = ana::kStateConfigured;
  reported_ = true;
  if (subscribed_) {
    if (segment_) {
      uint8_t seg[37];
      endpoint_.event(*this, ana::kEventSegment, seg, segmentInfo(seg));
    }
    uint8_t stopped[6] = {ana::kStoppedReasonHost, 0};   // reason(u8) error(u8) generation(u32) (capture §3.4)
    putU32(stopped + 2, generation_);
    endpoint_.event(*this, ana::kEventStopped, stopped, sizeof stopped);
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
  // triggered, the values go round the buffer (end_frame_: none after the segment's last frame); immediate, the
  // ring is the segment and end_frame_ its samples, so the index is the plain one
  while (handle_ && adc_continuous_read(handle_, records, sizeof records, &got, 0) == ESP_OK && got) {
    for (uint32_t k = 0; (k + 1) * kRecordBytes <= got; ++k) {
#if defined(CONFIG_IDF_TARGET_ESP32)
      // The I2S DMA puts the two 16-bit records of each 32-bit word the other way round: conversion 2j + 1 comes
      // before 2j. A 1 kHz square wave read one-channel showed a 0 inside the high run, just after the edge
      // (0.0.13, the V003 jig, 2026-09-30: 0 0 0 4095 0 4095 4095); swapped back pairwise it is 0 0 0 0 4095 4095 4095.
      const uint32_t at = ((k ^ 1) + 1) * kRecordBytes <= got ? (k ^ 1) * kRecordBytes : k * kRecordBytes;
#else
      const uint32_t at = k * kRecordBytes;
#endif
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
        if (counts_[m] < end_frame_) buffer_[(static_cast<size_t>(counts_[m]) * channels_ + m) % ring_len_] = value;
        ++counts_[m];
        break;
      }
    }
    uint32_t low = UINT32_MAX;
    for (uint8_t m = 0; m < channels_; ++m) if (counts_[m] < low) low = counts_[m];
    got_ = low;
    if (ringMode()) {
      if (overflow_) { overflow_ = false; overflow_seen_ = true; overflow_frame_ = got_; }
      if (phase_ == 0) search();   // after every read: no channel gets further ahead than one read
      if (phase_ == 1 && got_ >= end_frame_) break;
    } else {
      frames_ = low < samples_ ? low : samples_;
      if (frames_ >= samples_) break;
    }
  }
}
#endif

uint16_t *AnalogCapture::ring() const {
#if defined(OEP_ANALOG_RP2)
  if (ringMode()) return ring_;
#endif
  return buffer_;
}

uint64_t AnalogCapture::framesNs(uint64_t frames) const {   // frames x den / num seconds
  const uint64_t fd = frames * rate_den_;
  return fd / rate_num_ * 1000000000ull + fd % rate_num_ * 1000000000ull / rate_num_;
}

// Frames [searched_, got_) are in the ring: the first at or after arm_ where the value crosses (or force) is the
// trigger - following a group, the frame nearest the group's trigger, once it has come; the segment then runs from
// pretrigger frames before it (from frame 0 when there are fewer).
void AnalogCapture::search() {
  if (follow_) {
    searched_ = got_;
    if (!ext_ready_ || ext_frame_ >= got_) return;
    hitAt(ext_frame_);
    return;
  }
  const volatile uint16_t *r = ring();   // the RP2's DMA writes it under us
  const bool up = trig_type_ == ana::kTriggerCrossUp;
  for (; searched_ < got_; ++searched_) {
    const uint32_t f = searched_;
    const uint16_t v = r[(static_cast<size_t>(f) * channels_ + trig_slot_) % ring_len_];
    bool hit = false;
    if (f >= arm_)
      hit = force_ || (have_prev_ && (up ? prev_ < trig_value_ && v >= trig_value_ : prev_ > trig_value_ && v <= trig_value_));
    prev_ = v;
    have_prev_ = true;
    if (hit) {
      ++searched_;
      hitAt(f);
      return;
    }
  }
}

void AnalogCapture::hitAt(uint32_t t) {
  trig_frame_ = t;
  seg_first_ = t > pre() ? t - pre() : 0;
  end_frame_ = seg_first_ + samples_;
#if defined(ARDUINO_ARCH_ESP32)
  // the ring is the segment: frames already come past its end have taken the slots of its first ones
  if (static_cast<uint64_t>(got_) + kPretriggerRoom > end_frame_) trig_slipped_ = true;
#endif
  phase_ = 1;
}

bool AnalogCapture::trackCanKeep(uint32_t p) const {
  return p == 0 || (p <= maxPretrigger() && static_cast<uint64_t>(p) + kPretriggerRoom <= samples_);
}

bool AnalogCapture::trackCanFollow() const {
#if defined(OEP_ANALOG_RP2)
  if (static_cast<size_t>(samples_) * channels_ > kRingBytes / 4) return false;   // a triggered segment's most
#endif
  return trackReady();
}

bool AnalogCapture::trackStartFollowing() {
  if (!trackCanFollow()) return false;
#if defined(OEP_ANALOG_RP2)
  if (!ring_) ring_ = static_cast<uint16_t *>(aligned_alloc(kRingBytes, kRingBytes));
  if (!ring_) return false;
#endif
  follow_ = true;
  return startNow();
}

void AnalogCapture::trackTriggerAt(uint64_t ns) {
  ext_frame_ = static_cast<uint32_t>(samplesIn(ns > start_ns_ ? ns - start_ns_ : 0, rate_num_, rate_den_));
  ext_ready_ = true;
}

bool AnalogCapture::trackTriggerNs(uint64_t &ns) const {
  if (!trig_type_ || follow_ || phase_ < 1) return false;
  ns = start_ns_ + framesNs(trig_frame_);
  return true;
}

void AnalogCapture::pollTriggered() {
#if defined(ARDUINO_ARCH_ESP32)
  drain();
#else
  const bool running = dma_channel_is_busy(dma_);
  got_ = (kRingCount - (dma_channel_hw_addr(dma_)->transfer_count & 0x0FFFFFFFu)) / channels_;
  if (phase_ == 0) {
    const uint32_t ring_frames = ring_len_ / channels_;
    if (got_ > searched_ + ring_frames / 2) {   // poll came late: what was not looked at is going; look at the newest
      searched_ = got_ - ring_frames / 4;
      have_prev_ = false;
    }
    search();
    if (phase_ == 0 && !running) {              // the run's transfers are used up (minutes): again from the start
      armDma(ring_, kRingCount, true);
      got_ = searched_ = 0;
      have_prev_ = false;
    }
  }
#endif
  if (phase_ >= 1 && state_ == ana::kStateWaiting) state_ = ana::kStateCapturing;
  if (phase_ >= 1 && !trig_reported_) {
    trig_reported_ = true;
    if (subscribed_) {   // serial(u32) trigger_index(u32) trigger_ns(u64) generation(u32)
      uint8_t e[20];
      putU32(e, 0);
      putU32(e + 4, trig_frame_ - seg_first_);
      putU64(e + 8, start_ns_ + framesNs(trig_frame_));
      putU32(e + 16, generation_);
      endpoint_.event(*this, ana::kEventTriggered, e, sizeof e);
    }
  }
  if (phase_ == 1 && got_ >= end_frame_) finishTriggered(false);
}

// The segment's frames are all in (cut: stopped, those that came in): stop, and put them at the buffer's start in
// order.
void AnalogCapture::finishTriggered(bool cut) {
  const uint32_t s0 = seg_first_;
  uint32_t n = samples_;
#if defined(ARDUINO_ARCH_ESP32)
  if (cut) n = got_ > s0 ? std::min(got_ - s0, samples_) : 0;   // drained just before (poll)
  const bool lost = overflow_;
  finish();
  std::rotate(buffer_, buffer_ + static_cast<size_t>(s0 % samples_) * channels_, buffer_ + ring_len_);
  trig_slipped_ |= lost || (overflow_seen_ && overflow_frame_ >= s0);   // values were lost after the segment began
#else
  adc_run(false);
  const uint32_t written = kRingCount - (dma_channel_hw_addr(dma_)->transfer_count & 0x0FFFFFFFu);
  dma_channel_abort(dma_);
  adc_fifo_drain();
  adc_set_round_robin(0);
  if (cut) n = written / channels_ > s0 ? std::min(written / channels_ - s0, samples_) : 0;
  const size_t first = static_cast<size_t>(s0) * channels_, values = static_cast<size_t>(n) * channels_;
  trig_slipped_ |= written > first + ring_len_;   // the DMA came round over the segment's start before it stopped
  for (size_t i = 0; i < values; ++i) buffer_[i] = ring_[(first + i) % ring_len_];
  state_ = ana::kStateDone;
#endif
  phase_ = 2;
  frames_ = n;
  segment_ = n > 0;
  seg_start_ns_ = start_ns_ + framesNs(s0);
}

// capture §2.2: values lost (the driver's overflow) or overwritten (the ring come round over the segment) inside the
// segment - the bits that marked it slipped (bit2) before.
bool AnalogCapture::hole() const {
  if (ringMode()) return trig_slipped_;
#if defined(ARDUINO_ARCH_ESP32)
  return overflow_;
#else
  return false;   // the RP2's immediate segment: the DMA writes it in place
#endif
}

// The segment with a hole is not handed out (no segment, read empty, write_pos 0 - its start); the track stops in
// state 6 with stopped reason 3, error 2 (the conversions' queue or the ring overflowed).
void AnalogCapture::failHole() {
  segment_ = false;
  frames_ = 0;
  short_ = false;
  lost_ = true;
  reported_ = true;
  fail(ana::kErrorStorage);
}

// State 6 (capture §3.2): every entry sends stopped reason 3 with the error status answers (1 the ADC driver or its
// DMA, 2 a segment with a hole).
void AnalogCapture::fail(uint8_t error) {
  const bool entering = state_ != ana::kStateError;
  state_ = ana::kStateError;
  error_ = error;
  if (entering && subscribed_) {
    uint8_t stopped[6] = {ana::kStoppedReasonError, error};   // reason(u8) error(u8) generation(u32)
    putU32(stopped + 2, generation_);
    endpoint_.event(*this, ana::kEventStopped, stopped, sizeof stopped);
  }
}

void AnalogCapture::poll() {
  if (ringMode() && (state_ == ana::kStateWaiting || state_ == ana::kStateCapturing)) pollTriggered();
  else if (state_ == ana::kStateCapturing) {
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
  if (state_ == ana::kStateDone && !reported_ && hole()) failHole();
  if (state_ == ana::kStateDone && !reported_) {
    reported_ = true;
    if (subscribed_) {
      uint8_t seg[37];
      endpoint_.event(*this, ana::kEventSegment, seg, segmentInfo(seg));
      uint8_t stopped[6] = {ana::kStoppedReasonComplete, 0};   // reason(u8) error(u8) generation(u32) (capture §3.4)
      putU32(stopped + 2, generation_);
      endpoint_.event(*this, ana::kEventStopped, stopped, sizeof stopped);
    }
  }
}

size_t AnalogCapture::segmentInfo(uint8_t *out) const {   // the segment record (oep-if-capture §2), 37 bytes
  putU32(out, 0);                  // serial
  putU64(out + 4, 0);              // position
  putU32(out + 12, frames_);
  putU64(out + 16, ringMode() ? seg_start_ns_ : start_ns_);
  putU32(out + 24, uncertainty_ns_);
  putU32(out + 28, ringMode() ? trig_frame_ - seg_first_ : 0xFFFFFFFFu);   // immediate: no trigger inside
  uint8_t flags = short_ ? ana::kSegmentFlagShort : 0;
  // bit2 (capture §2.2): a value taken one sample period or more after its time - conversions lost or overwritten
  // inside the segment put the values after them later than their count says
  if (ringMode() && trig_slipped_) flags |= ana::kSegmentFlagSlipped;
#if defined(ARDUINO_ARCH_ESP32)
  if (!ringMode() && overflow_) flags |= ana::kSegmentFlagSlipped;   // conversions were lost
#endif
  out[32] = flags;
  putU32(out + 33, generation_);
  return 37;
}

// ---- the operations -----------------------------------------------------------------------------------------------

Result AnalogCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case ana::kOpConfigure: return configure(p, n, out, capacity, false);
    case ana::kOpQuery: return configure(p, n, out, capacity, true);
    case ana::kOpStart: {   // -> blocking_ms(u32) generation(u32) [TLV]
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (bound()) return boundInGroup(*this, out, capacity);   // the group starts it (cause 4)
      if (!trackReady() && state_ != ana::kStateError) return wrongState(out, capacity);
      if (capacity < 8) return failed();
      follow_ = false;
      if (!startNow()) return failed();
      putU32(out, 0);   // blocking_ms: DMA, the probe keeps answering
      putU32(out + 4, generation_);
      return completed(8);
    }
    case ana::kOpStop: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (bound()) return boundInGroup(*this, out, capacity);
      stopNow();
      return completed();
    }
    case ana::kOpStatus: {   // -> state(u8) serial_done(u32) write_pos(u64) flags(u8) generation(u32) [TLV error]
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 21) return failed();
      poll();
      out[0] = state_;
      putU32(out + 1, segment_ ? 1 : 0);   // the segment, complete or cut short by stop
      putU64(out + 5, static_cast<uint64_t>(frames_) * channels_ * 2u);
#if defined(ARDUINO_ARCH_ESP32)
      out[13] = (overflow_ || overflow_seen_ || trig_slipped_ || lost_) ? ana::kStatusFlagDropped : 0;   // bit0 conversions were lost
#else
      out[13] = (trig_slipped_ || lost_) ? ana::kStatusFlagDropped : 0;
#endif
      putU32(out + 14, generation_);
      size_t used = 18;
      if (state_ == ana::kStateError) {   // why (TLV 0x01 error): the driver would not start
        putTlvHeader(out + 18, ana::kTlvStatusAnswerError, 1);
        out[21] = error_;   // 1 the driver would not start / run, 2 a segment with a hole (§2.2)
        used = 22;
      }
      return completed(used);
    }
    case ana::kOpRead: {   // generation(u32) position(u64) max(u32) -> position(u64) flags(u8: bit0 more) len(u32) data [TLV]
      const Result parsed = plainTail(tail, p, n, 16, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);   // another generation (cause 6)
      const uint64_t position = getU64(p + 4);
      const uint32_t most = getU32(p + 12);
      // no plan or configuration (released: the data went with it): nothing to read
      const uint64_t end = state_ == ana::kStateUnconfigured || !buffer_ ? 0 : static_cast<uint64_t>(frames_) * channels_ * 2u;
      if (capacity < 13) return failed();
      // how much read returns is the probe's (capture §3.2): within max, max_read_ and the frame
      size_t take = position < end ? static_cast<size_t>(end - position) : 0;
      if (take > most) take = most;
      if (take > max_read_) take = max_read_;
      if (take > capacity - 13) take = capacity - 13;
      putU64(out, position);
      out[8] = position + take < end ? reg::common::kReadFlagsMore : 0;
      putU32(out + 9, static_cast<uint32_t>(take));
      if (take) memcpy(out + 13, reinterpret_cast<const uint8_t *>(buffer_) + position, take);   // little endian
      return completed(13 + take);
    }
    case ana::kOpSegments: {   // from_serial(u32) -> more(u8) count(u8) count x segment [TLV]
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 3 + 37) return failed();
      poll();
      // common §1.3's serial paging over the one segment (serial 0, once there): every from_serial but next (1) gets it
      const bool one = segment_ && serialPageStart(getU32(p), 0, 1) == 0;
      out[0] = 0;
      out[1] = one ? 1 : 0;
      const size_t info = one ? segmentInfo(out + 2) : 0;   // no element length (core §2.3)
      return completed(2u + info);
    }
    case ana::kOpRelease: {   // generation(u32) serial(u32): nothing to release in one-shot (success)
      const Result parsed = plainTail(tail, p, n, 8, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);
      return completed();
    }
    case ana::kOpCalibration: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      const Result r = calibration(out, capacity);
      return refused(r) ? r : r;
    }
    case ana::kOpForce: {   // waiting for the crossing: take the segment from the next frame on (after the pretrigger)
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (bound()) return boundInGroup(*this, out, capacity);
      if (state_ == ana::kStateWaiting) force_ = true;
      return completed();
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
