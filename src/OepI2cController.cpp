// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepI2cController.h"

#include <string.h>

#include <initializer_list>

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_cpu.h>
// ESP-IDF's GPIO matrix initializer leaves fields zero; silence its aggregate warning only in the SDK header.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include <hal/gpio_ll.h>
#pragma GCC diagnostic pop
#include <soc/gpio_struct.h>
#endif

namespace oep {

namespace {
// The clock the halves are counted on: the CPU's cycles on an ESP32, micros() elsewhere; the host tests' a 160 MHz CPU
#if defined(OEP_HOST_FAKE_I2C_BUS)
constexpr uint32_t tickHz() { return 160000000u; }
#elif defined(ARDUINO_ARCH_ESP32)
uint32_t tickHz() { return getCpuFrequencyMhz() * 1000000u; }
inline uint32_t ticks() { return esp_cpu_get_cycle_count(); }
// open drain: the pad's output enable stays on, its level 1 lets go
inline void line(int pad, bool high) { gpio_ll_set_level(&GPIO, static_cast<uint32_t>(pad), high ? 1 : 0); }
inline bool level(int pad) { return gpio_ll_get_level(&GPIO, static_cast<uint32_t>(pad)) != 0; }
#else
constexpr uint32_t tickHz() { return 1000000u; }
inline uint32_t ticks() { return micros(); }
// open drain from a GPIO without it (the X035 has none): let go = an input with its pull-up, low = an output at 0 (the
// level set before the output is on)
inline void line(int pad, bool high) {
  if (high) { pinMode(pad, INPUT_PULLUP); return; }
  digitalWrite(pad, LOW);
  pinMode(pad, OUTPUT);
  digitalWrite(pad, LOW);
}
inline bool level(int pad) { return digitalRead(pad) != 0; }
#endif
// An I2C bit on the bus: SCL at least twice, the segments' START and the ACK bit counted in
constexpr uint32_t kBitsPerByte = 9;
}  // namespace

uint8_t I2cController::planCheck(const RoleAssignment *roles, size_t count) {
  // roles 1 and 2 exactly once each, on two channels (fixture §5): another set is unsupported (oep-if-plan §2.4)
  if (count != 2) return kRejectUnsupported;
  int sda = -1, scl = -1;
  for (size_t i = 0; i < count; ++i) {
    int &pin = roles[i].role == kRoleSda ? sda : scl;
    if ((roles[i].role != kRoleSda && roles[i].role != kRoleScl) || pin >= 0) return kRejectUnsupported;
    pin = roles[i].channel;
  }
  if (sda < 0 || scl < 0 || sda == scl) return kRejectUnsupported;
  if (!pins_.canOutput(sda) || !pins_.canOutput(scl)) return kRejectUnsupported;   // both are pulled low
  // a channel whose idle is an output: not taken (core §8), cause 5
  refusal_cause_ = reg::core::kUnavailableCauseWrongState;
  refusal_channel_ = 0xffff;
  for (size_t i = 0; i < count; ++i) {
    const uint8_t idle = pins_.idle(roles[i].channel);
    if (idle == PinTable::kIdleOutputLow || idle == PinTable::kIdleOutputHigh) {
      refusal_cause_ = reg::core::kUnavailableCauseHeldBySettings;
      refusal_channel_ = roles[i].channel;
      return kRejectUnavailable;
    }
  }
  if (!pins_.freeFor(sda, kOwnerId) || !pins_.freeFor(scl, kOwnerId)) return kRejectUnavailable;
  if (sda_ >= 0 && started_) return kRejectUnavailable;   // running (state 1): no change, cause 6 (fixture §5)
  return 0;
}

bool I2cController::planApply(const RoleAssignment *roles, size_t count) {
  int sda = -1, scl = -1;
  for (size_t i = 0; i < count; ++i) (roles[i].role == kRoleSda ? sda : scl) = roles[i].channel;
  if (!pins_.claim(sda, kOwnerId)) return false;
  if (!pins_.claim(scl, kOwnerId)) { pins_.release(kOwnerId); return false; }
  sda_ = sda; scl_ = scl;
  pins_.ownStrength(sda);
  pins_.ownStrength(scl);
  return true;
}

// Releasing or replacing the plan: a held bus gets its STOP, then the state right after describe (fixture §5).
void I2cController::planRelease() {
  stop();
  pins_.release(kOwnerId);
  sda_ = scl_ = -1;
  clock_hz_ = stretch_timeout_us_ = 0;
  transfers_ = errors_ = 0;
}

size_t I2cController::describe(uint8_t *out, size_t capacity) {
  static const uint8_t kRoles[] = {kRoleSda, kRoleScl};
  namespace i2c = reg::fixture_i2c_controller;
  TlvWriter w(out, capacity);
  w.roleChannels(kRoles, sizeof kRoles, pins_.outputMask());   // both lines are pulled low: no input-only pin
  w.u16(kTagMaxLength, kMaxLength);
  w.u32(kTagMaxClockHz, kMaxClockHz);
  w.u32(kTagMinClockHz, kMinClockHz);
  w.u32(kTagFeatures, i2c::kFeaturesInternalPullups);
  w.u32(i2c::kTlvDescribeMaxStretchTimeoutUs, kMaxStretchTimeoutUs);
  return w.ok() ? w.length() : 0;
}

bool I2cController::start() {
#if defined(OEP_HOST_FAKE_I2C_BUS)
  if (fake_fail_start) { fake_fail_start = false; return false; }
  started_ = host_bus != nullptr;
  return started_;
#else
  sda_pad_ = platformPad(sda_);
  scl_pad_ = platformPad(scl_);
  for (const int pad : {sda_pad_, scl_pad_}) {
#if defined(ARDUINO_ARCH_ESP32)
    line(pad, true);   // let go before the output is on
    pinMode(pad, OUTPUT_OPEN_DRAIN | PULLUP);   // open drain, its input on, the pad's pull-up (features bit0)
#endif
    line(pad, true);
  }
  const uint32_t hz = tickHz();
  half_ = (hz + 2 * clock_hz_ - 1) / (2 * clock_hz_);
  timeout_ = static_cast<uint32_t>(static_cast<uint64_t>(stretch_timeout_us_) * (hz / 1000000u));
  started_ = true;
  return true;
#endif
}

// state 0: a held bus gets its STOP, the lines are neither driven nor pulled up (core §8's free state)
void I2cController::stop() {
  if (held_) lineStop();
  held_ = false;
  if (started_) {
#if !defined(OEP_HOST_FAKE_I2C_BUS)
    platformGpio(sda_, kGpioInputFloating);
    platformGpio(scl_, kGpioInputFloating);
#endif
  }
  started_ = false;
}

#if defined(OEP_HOST_FAKE_I2C_BUS)

uint8_t I2cController::lineStart(bool repeated) { return host_bus->start(repeated); }
uint8_t I2cController::lineWrite(uint8_t byte, bool &ack) { return host_bus->write(byte, ack); }
uint8_t I2cController::lineRead(uint8_t &byte, bool ack) { return host_bus->read(byte, ack); }
void I2cController::lineStop() { host_bus->stop(); }
void I2cController::lineRelease() { host_bus->release(); }

#else

void I2cController::sda(bool high) const { line(sda_pad_, high); }
void I2cController::scl(bool high) const { line(scl_pad_, high); }
bool I2cController::sdaHigh() const { return level(sda_pad_); }
bool I2cController::sclHigh() const { return level(scl_pad_); }

// Until half a period after the last edge; a half an interrupt stretched starts the next one from now
void I2cController::waitHalf() {
  const uint32_t due = edge_ + half_;
  while (static_cast<int32_t>(ticks() - due) < 0) {}
  const uint32_t now = ticks();
  edge_ = now - due > half_ ? now : due;
}

bool I2cController::sclRelease() {
  scl(true);
  const uint32_t from = ticks();
  while (!sclHigh()) {
    if (ticks() - from > timeout_) return false;
  }
  edge_ = ticks();
  return true;
}

// SCL low on entry and on return. A 1 the bus does not show while SCL is high: another controller, or a stuck line.
uint8_t I2cController::bitOut(bool bit) {
  sda(bit);
  waitHalf();
  if (!sclRelease()) return kLineStretch;
  if (bit && !sdaHigh()) return kLineBusError;
  waitHalf();
  scl(false);
  return kLineOk;
}

uint8_t I2cController::bitIn(bool &bit) {
  sda(true);
  waitHalf();
  if (!sclRelease()) return kLineStretch;
  bit = sdaHigh();
  waitHalf();
  scl(false);
  return kLineOk;
}

// A START from an idle bus (both lines high), or a repeated START with SCL held low
uint8_t I2cController::lineStart(bool repeated) {
  if (repeated) {
    sda(true);
    waitHalf();
    if (!sclRelease()) return kLineStretch;
    waitHalf();
  } else {
    edge_ = ticks();
    if (!sclRelease()) return kLineStretch;   // a target still holding SCL low
  }
  if (!sdaHigh()) return kLineBusError;   // SDA held low: no START can be made
  sda(false);
  waitHalf();
  scl(false);
  return kLineOk;
}

uint8_t I2cController::lineWrite(uint8_t byte, bool &ack) {
  for (int k = 7; k >= 0; --k) {
    const uint8_t r = bitOut((byte >> k) & 1);
    if (r != kLineOk) return r;
  }
  bool nack = true;
  const uint8_t r = bitIn(nack);
  ack = !nack;
  return r;
}

uint8_t I2cController::lineRead(uint8_t &byte, bool ack) {
  byte = 0;
  for (int k = 0; k < 8; ++k) {
    bool bit = false;
    const uint8_t r = bitIn(bit);
    if (r != kLineOk) return r;
    byte = static_cast<uint8_t>(byte << 1 | bit);
  }
  return bitOut(!ack);
}

void I2cController::lineStop() {
  sda(false);
  waitHalf();
  sclRelease();   // a target holding SCL past the timeout: SDA rises anyway, the bus let go
  waitHalf();
  sda(true);
  waitHalf();
}

void I2cController::lineRelease() {
  sda(true);
  scl(true);
}

#endif

// flags(u8) n(u8) n x (dir(u8) address(u8) count(u16) [data]) [TLV] -> result(u8) at(u8) index(u16) count(u16) data [TLV]
Result I2cController::transfer(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  namespace i2c = reg::fixture_i2c_controller;
  if (length < 2) return rejected(kRejectMalformed);
  const uint8_t flags = payload[0], n = payload[1];
  // the segments' form first: where the fixed part ends, the bytes written and read
  size_t at = 2, total = 0, reads = 0, bits = 0;
  for (uint8_t s = 0; s < n; ++s) {
    if (at + 4 > length) return rejected(kRejectMalformed);
    const uint8_t dir = payload[at], address = payload[at + 1];
    const uint16_t count = getU16(payload + at + 2);
    if (dir > i2c::kDirRead || address > 0x7f || (dir == i2c::kDirRead && count == 0)) return rejected(kRejectMalformed);
    at += 4 + (dir == i2c::kDirWrite ? count : 0);
    if (at > length) return rejected(kRejectMalformed);
    total += count;
    if (dir == i2c::kDirRead) reads += count;
    bits += kBitsPerByte * (1u + count) + 2;   // the address byte, the data, a START's and a STOP's share
  }
  Tail tail;
  const Result parsed = plainTail(tail, payload, length, at, out, capacity);
  if (refused(parsed)) return parsed;
  if (n == 0 && !(flags & i2c::kTransferFlagsStop)) return rejected(kRejectMalformed);
  if (flags & ~i2c::kTransferFlagsStop) return unsupportedValue(out, capacity);   // reserved bits (core §2.5)
  if (total > kMaxLength) return unsupportedValue(out, capacity);
  if (!started_) return wrongState(out, capacity);
  // the clock alone past max_op_ms (fixture §5): refused before anything is sent
  if (static_cast<uint64_t>(bits) * 1000u / clock_hz_ > kMaxOpMs) return unsupportedValue(out, capacity);
  if (capacity < 6 + reads) return failed();
  if (n == 0) {   // a STOP alone, if the bus is held (not counted)
    if (held_) lineStop();
    held_ = false;
    memset(out, 0, 6);
    return completed(6);
  }
  uint8_t result = i2c::kResultDone, stopped_at = 0;
  uint16_t index = 0;
  size_t got = 0;
  at = 2;
  for (uint8_t s = 0; s < n && result == i2c::kResultDone; ++s) {
    const uint8_t dir = payload[at], address = payload[at + 1];
    const uint16_t count = getU16(payload + at + 2);
    const uint8_t *data = payload + at + 4;
    at += 4 + (dir == i2c::kDirWrite ? count : 0);
    uint8_t line = lineStart(held_ || s > 0);
    held_ = true;
    bool ack = false;
    if (line == kLineOk) line = lineWrite(static_cast<uint8_t>(address << 1 | dir), ack);
    if (line == kLineOk && !ack) result = i2c::kResultAddressNack;
    for (uint16_t k = 0; line == kLineOk && result == i2c::kResultDone && k < count; ++k) {
      if (dir == i2c::kDirWrite) {
        line = lineWrite(data[k], ack);
        if (line == kLineOk && !ack) { result = i2c::kResultDataNack; index = k; }
      } else {
        line = lineRead(out[6 + got], k + 1 < count);   // the last byte NACKed
        if (line == kLineOk) ++got;
      }
    }
    if (line != kLineOk) result = line;
    if (result != i2c::kResultDone) stopped_at = s;
  }
  if (result == i2c::kResultBusError) {
    lineRelease();
    held_ = false;
  } else if (result != i2c::kResultDone || (flags & i2c::kTransferFlagsStop)) {
    lineStop();
    held_ = false;
  }
  if (transfers_ != 0xffffffffu) ++transfers_;
  if (result != i2c::kResultDone && errors_ != 0xffffffffu) ++errors_;
  out[0] = result;
  out[1] = stopped_at;
  putU16(out + 2, index);
  putU16(out + 4, static_cast<uint16_t>(got));
  return completed(6 + got);
}

Result I2cController::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (operation) {
    case kOpConfigure: {   // clock_hz(u32) stretch_timeout_us(u32) [TLV] -> clock_hz(u32) [TLV]
      const Result parsed = plainTail(tail, payload, length, 8, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t clock = getU32(payload), timeout = getU32(payload + 4);
      if (clock == 0 || timeout == 0) return rejected(kRejectMalformed);
      if (clock < kMinClockHz || clock > kMaxClockHz || timeout > kMaxStretchTimeoutUs) return unsupportedValue(out, capacity);
      if (sda_ < 0) return wrongState(out, capacity);   // needs a plan (fixture §5: unavailable cause 6)
      if (capacity < 4) return failed();
      stop();   // a held bus gets its STOP first
      transfers_ = errors_ = 0;
      clock_hz_ = clock;
      stretch_timeout_us_ = timeout;
      if (!start()) { stop(); return failed(); }
      // the SCL used: the nearest at or below the one asked, whole CPU cycles a half
      const uint32_t cpu = tickHz(), half = (cpu + 2 * clock - 1) / (2 * clock);
      putU32(out, cpu / (2 * half));
      return completed(4);
    }
    case kOpTransfer:
      return transfer(payload, length, out, capacity);
    case kOpStatus: {   // [TLV] -> state(u8) held(u8) transfers(u32) errors(u32) [TLV]
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 10) return failed();
      out[0] = started_ ? reg::fixture_i2c_controller::kStateRunning : reg::fixture_i2c_controller::kStateUnconfigured;
      out[1] = held_ ? 1 : 0;
      putU32(out + 2, transfers_);
      putU32(out + 6, errors_);
      return completed(10);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep
