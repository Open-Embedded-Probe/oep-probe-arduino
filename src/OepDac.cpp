// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepDac.h"

// Not on the X035 (kRoomy in the Firmware): its interrupts / DAC code would be linked by this file alone
#if !defined(ARDUINO_CH32X035)

#if defined(ARDUINO_ARCH_CH32RV) && !defined(OEP_HOST_FAKE_DAC)
#include "ch32rv_registers.h"
#elif defined(ARDUINO_ARCH_ESP32)
#include <soc/soc_caps.h>   // SOC_DAC_SUPPORTED: the classic ESP32 (and S2) only
#endif

namespace oep {

namespace {

constexpr uint32_t kFull = (1u << FixtureDac::kBits) - 1;

void write(int channel, uint32_t value) {
#if defined(OEP_HOST_FAKE_DAC)
  FixtureDac::host_value[channel] = value;
#elif defined(ARDUINO_ARCH_ESP32) && SOC_DAC_SUPPORTED
  dacWrite(static_cast<uint8_t>(platformPad(channel)), static_cast<uint8_t>(value));
#elif defined(ARDUINO_ARCH_CH32RV) && (defined(CH32RV_DAC1_PIN) || defined(CH32RV_DAC2_PIN))
  analogWriteResolution(12);
  analogWrite(platformPad(channel), static_cast<int>(value));
#else
  (void)channel;
  (void)value;
#endif
}

// The converter off its pad, the pad back to core §8's free state
void stopWriting(int channel) {
#if defined(OEP_HOST_FAKE_DAC)
  FixtureDac::host_value[channel] = 0xFFFFFFFFu;
#else
#if defined(ARDUINO_ARCH_ESP32) && SOC_DAC_SUPPORTED
  dacDisable(static_cast<uint8_t>(platformPad(channel)));
#elif defined(ARDUINO_ARCH_CH32RV) && defined(CH32RV_DAC1_PIN)
  const int pad = platformPad(channel);
  if (pad == CH32RV_DAC1_PIN) CH32RV_DAC_CTLR &= ~CH32RV_DAC_CTLR_EN1;
#if defined(CH32RV_DAC2_PIN)
  if (pad == CH32RV_DAC2_PIN) CH32RV_DAC_CTLR &= ~CH32RV_DAC_CTLR_EN2;
#endif
#endif
  platformGpio(channel, kGpioInputFloating);
#endif
}

}  // namespace

uint8_t FixtureDac::planCheck(const RoleAssignment *roles, size_t count) {
  uint8_t seen = 0;
  refusal_cause_ = reg::core::kUnavailableCauseWrongState;
  for (size_t i = 0; i < count; ++i) {
    const uint16_t ch = roles[i].channel;
    if (roles[i].role >= kMaxChannels || ((seen >> roles[i].role) & 1) || ch > 63 || !((dac_pins_ >> ch) & 1))
      return kRejectUnsupported;
    seen |= 1u << roles[i].role;
    for (size_t j = i + 1; j < count; ++j) if (roles[j].channel == ch) return kRejectUnsupported;
    const uint8_t idle = pins_.idle(ch);
    if (idle == PinTable::kIdleOutputLow || idle == PinTable::kIdleOutputHigh) {
      refusal_cause_ = reg::core::kUnavailableCauseHeldBySettings;
      return kRejectUnavailable;
    }
    if (!pins_.freeFor(ch, kOwnerId)) return kRejectUnavailable;
  }
  return 0;
}

// The plan taken: nothing driven until the first set (fixture §9)
bool FixtureDac::planApply(const RoleAssignment *roles, size_t count) {
  planRelease();
  for (size_t i = 0; i < count; ++i) {
    if (!pins_.claim(roles[i].channel, kOwnerId)) { planRelease(); return false; }
    out_[roles[i].role].channel = roles[i].channel;
  }
  return true;
}

void FixtureDac::planRelease() {
  for (Out &o : out_) if (o.on) off(o);
  pins_.release(kOwnerId);
  for (Out &o : out_) o = Out{};
}

void FixtureDac::off(Out &o) {
  stopWriting(o.channel);
  o.on = false;
}

size_t FixtureDac::describe(uint8_t *out, size_t capacity) {
  static const uint8_t kRoles[] = {0, 1};
  namespace dac = reg::fixture_dac;
  TlvWriter w(out, capacity);
  w.roleChannels(kRoles, kMaxChannels, dac_pins_ & pins_.allowedMask());
  w.u8(dac::kTlvDescribeChannels, kMaxChannels);
  w.u8(dac::kTlvDescribeBits, kBits);
  uint8_t range[8];
  putU32(range, static_cast<uint32_t>(kMinMv));
  putU32(range + 4, static_cast<uint32_t>(kMaxMv));
  w.put(dac::kTlvDescribeRange, range, sizeof range);
  return w.ok() ? w.length() : 0;
}

Result FixtureDac::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (operation) {
    case kOpSet: {   // n(u8) n x (role(u8) value(u32)) [TLV]
      if (length < 1) return rejected(kRejectMalformed);
      const uint8_t n = payload[0];
      const Result parsed = plainTail(tail, payload, length, 1u + 5u * n, out, capacity);
      if (refused(parsed)) return parsed;
      if (n == 0) return rejected(kRejectMalformed);
      for (uint8_t i = 0; i < n; ++i) {
        const uint8_t *e = payload + 1 + 5 * i;
        if (e[0] >= kMaxChannels || getU32(e + 1) > kFull) return unsupportedValue(out, capacity);
      }
      for (uint8_t i = 0; i < n; ++i) if (out_[payload[1 + 5 * i]].channel < 0) return wrongState(out, capacity);
      for (uint8_t i = 0; i < n; ++i) {
        const uint8_t *e = payload + 1 + 5 * i;
        Out &o = out_[e[0]];
        o.value = getU32(e + 1);
        write(o.channel, o.value);
        o.on = true;
      }
      return completed();
    }
    case kOpStatus: {   // [TLV] -> n(u8) n x (role(u8) value(u32) on(u8)) [TLV]
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 1u + 6u * kMaxChannels) return failed();
      size_t at = 1;
      uint8_t n = 0;
      for (uint8_t r = 0; r < kMaxChannels; ++r) {
        if (out_[r].channel < 0) continue;
        out[at] = r;
        putU32(out + at + 1, out_[r].value);
        out[at + 5] = out_[r].on ? 1 : 0;
        at += 6;
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

#endif
