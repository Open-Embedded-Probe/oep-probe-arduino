// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepSpiController.h"

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
// The clock the times are counted on: the CPU's cycles on an ESP32, micros() elsewhere; the host tests' a 160 MHz CPU
#if defined(OEP_HOST_FAKE_SPI_BUS)
constexpr uint32_t tickHz() { return 160000000u; }
#elif defined(ARDUINO_ARCH_ESP32)
uint32_t tickHz() { return getCpuFrequencyMhz() * 1000000u; }
inline uint32_t ticks() { return esp_cpu_get_cycle_count(); }
inline void line(int pad, bool high) { gpio_ll_set_level(&GPIO, static_cast<uint32_t>(pad), high ? 1 : 0); }
inline bool level(int pad) { return gpio_ll_get_level(&GPIO, static_cast<uint32_t>(pad)) != 0; }
#else
constexpr uint32_t tickHz() { return 1000000u; }
inline uint32_t ticks() { return micros(); }
inline void line(int pad, bool high) { digitalWrite(pad, high ? HIGH : LOW); }
inline bool level(int pad) { return digitalRead(pad) != 0; }
#endif
// A time asked as whole ticks at or above it, and those ticks as ns rounded down: what the line kept at least
uint32_t cyclesOf(uint32_t ns) { return static_cast<uint32_t>((static_cast<uint64_t>(ns) * tickHz() + 999999999u) / 1000000000u); }
uint32_t nsOf(uint32_t cycles) { return static_cast<uint32_t>(static_cast<uint64_t>(cycles) * 1000000000u / tickHz()); }
constexpr size_t kItem = 14;   // gap_ns setup_ns hold_ns count: a transfer's fixed part, asked and answered
}  // namespace

uint8_t SpiController::planCheck(const RoleAssignment *roles, size_t count) {
  // roles 1 to 4 exactly once each, on four channels (fixture §6): another set is unsupported (oep-if-plan §2.4)
  if (count != 4) return kRejectUnsupported;
  int pin[5] = {-1, -1, -1, -1, -1};
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role < kRoleSck || roles[i].role > kRoleCs || pin[roles[i].role] >= 0) return kRejectUnsupported;
    pin[roles[i].role] = roles[i].channel;
  }
  for (int r = kRoleSck; r <= kRoleCs; ++r) {
    for (int q = r + 1; q <= kRoleCs; ++q) if (pin[r] == pin[q]) return kRejectUnsupported;
  }
  for (int r = kRoleSck; r <= kRoleCs; ++r) if (!pins_.allowed(pin[r])) return kRejectUnsupported;
  // the driven lines: not an input-only pin
  if (!pins_.canOutput(pin[kRoleSck]) || !pins_.canOutput(pin[kRoleMosi]) || !pins_.canOutput(pin[kRoleCs]))
    return kRejectUnsupported;
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
  for (int r = kRoleSck; r <= kRoleCs; ++r) if (!pins_.freeFor(pin[r], kOwnerId)) return kRejectUnavailable;
  if (sck_ >= 0 && started_) return kRejectUnavailable;   // running (state 1): no change, cause 6 (fixture §6)
  return 0;
}

bool SpiController::planApply(const RoleAssignment *roles, size_t count) {
  int pin[5] = {-1, -1, -1, -1, -1};
  for (size_t i = 0; i < count; ++i) pin[roles[i].role] = roles[i].channel;
  for (int r = kRoleSck; r <= kRoleCs; ++r) {
    if (!pins_.claim(pin[r], kOwnerId)) { pins_.release(kOwnerId); return false; }
  }
  sck_ = pin[kRoleSck]; mosi_ = pin[kRoleMosi]; miso_ = pin[kRoleMiso]; cs_ = pin[kRoleCs];
  for (int r = kRoleSck; r <= kRoleCs; ++r) pins_.ownStrength(static_cast<uint16_t>(pin[r]));
  return true;
}

// Releasing or replacing the plan: back to the state right after describe (fixture §6).
void SpiController::planRelease() {
  stop();
  pins_.release(kOwnerId);
  sck_ = mosi_ = miso_ = cs_ = -1;
  mode_ = bit_order_ = 0;
  transfers_ = 0;
}

size_t SpiController::describe(uint8_t *out, size_t capacity) {
  static const uint8_t kDriven[] = {kRoleSck, kRoleMosi, kRoleCs}, kMiso[] = {kRoleMiso};
  namespace spi = reg::fixture_spi_controller;
  TlvWriter w(out, capacity);
  w.roleChannels(kDriven, sizeof kDriven, pins_.outputMask());
  w.roleChannels(kMiso, sizeof kMiso, pins_.allowedMask());   // MISO is only read
  w.u16(kTagMaxLength, kMaxLength);
  w.u32(kTagMaxClockHz, kMaxClockHz);
  w.u32(kTagMinClockHz, kMinClockHz);
  w.u32(kTagFeatures, spi::kFeaturesLsbFirst);
  w.u32(spi::kTlvDescribeTimeStepNs, (1000000000u + tickHz() - 1) / tickHz());   // one tick, rounded up
  w.u8(spi::kTlvDescribeModes, kModes);
  w.u32(spi::kTlvDescribeMaxDelayNs, kMaxDelayNs);
  return w.ok() ? w.length() : 0;
}

bool SpiController::start() {
#if defined(OEP_HOST_FAKE_SPI_BUS)
  if (fake_fail_start) { fake_fail_start = false; return false; }
  started_ = host_bus != nullptr;
  return started_;
#else
  sck_pad_ = platformPad(sck_); mosi_pad_ = platformPad(mosi_); miso_pad_ = platformPad(miso_); cs_pad_ = platformPad(cs_);
  // the levels before the outputs are on: CS inactive (high), SCK at CPOL, MOSI low
  set(cs_pad_, true);
  set(sck_pad_, mode_ >> 1);
  set(mosi_pad_, false);
  for (const int pad : {cs_pad_, sck_pad_, mosi_pad_}) pinMode(pad, OUTPUT);
  set(cs_pad_, true);
  set(sck_pad_, mode_ >> 1);
  set(mosi_pad_, false);
  pinMode(miso_pad_, INPUT);   // no pull (fixture §6)
  started_ = true;
  return true;
#endif
}

// state 0: the channels in core §8's free state
void SpiController::stop() {
  if (started_) {
#if !defined(OEP_HOST_FAKE_SPI_BUS)
    for (const int c : {cs_, sck_, mosi_, miso_}) platformGpio(c, kGpioInputFloating);
#endif
  }
  started_ = false;
}

#if defined(OEP_HOST_FAKE_SPI_BUS)

void SpiController::lineTransfer(bool first, uint32_t gap, uint32_t setup, uint32_t hold, const uint8_t *tx, uint8_t *rx,
                                 size_t count) {
  host_bus->transfer(first, gap, setup, hold, tx, rx, count);
}

#else

void SpiController::set(int pad, bool high) const { line(pad, high); }
bool SpiController::miso() const { return level(miso_pad_); }

// Until `cycles` ticks after the last edge
void SpiController::waitFor(uint32_t cycles) {
  const uint32_t due = edge_ + cycles;
  while (static_cast<int32_t>(ticks() - due) < 0) {}
}

// One CS-framed transfer. Mode = CPOL x 2 + CPHA (fixture §4): with CPHA 0 a bit is on MOSI before its leading edge
// (the first one right at CS), MISO read at the leading edge; with CPHA 1 a bit goes out at its leading edge and MISO is
// read at the trailing one. The last edge is a trailing one either way; hold counts from it.
void SpiController::lineTransfer(bool first, uint32_t gap, uint32_t setup, uint32_t hold, const uint8_t *tx, uint8_t *rx,
                                 size_t count) {
  const bool cpol = mode_ >> 1, cpha = mode_ & 1, lsb = bit_order_;
  const size_t bits = count * 8;
  auto bitOf = [&](size_t k) { return ((tx[k / 8] >> (lsb ? k % 8 : 7 - k % 8)) & 1) != 0; };
  if (!first) waitFor(gap);   // edge_: the CS rise that ended the transfer before
  set(cs_pad_, false);
  edge_ = ticks();
  if (!bits) {
    waitFor(setup + hold);
  } else {
    memset(rx, 0, count);
    if (!cpha) set(mosi_pad_, bitOf(0));
    waitFor(setup);
    for (size_t k = 0; k < bits; ++k) {
      if (k) waitFor(half_);
      set(sck_pad_, !cpol);   // the leading edge
      edge_ = ticks();
      if (cpha) set(mosi_pad_, bitOf(k));
      bool in = cpha ? false : miso();
      waitFor(half_);
      set(sck_pad_, cpol);   // the trailing edge
      edge_ = ticks();
      if (cpha) in = miso();
      if (in) rx[k / 8] |= static_cast<uint8_t>(1u << (lsb ? k % 8 : 7 - k % 8));
      if (!cpha && k + 1 < bits) set(mosi_pad_, bitOf(k + 1));
    }
    waitFor(hold);
  }
  set(cs_pad_, true);
  edge_ = ticks();
}

#endif

// n(u8) n x (gap_ns setup_ns hold_ns count tx) [TLV] -> n x (gap_ns setup_ns hold_ns count rx) [TLV]
Result SpiController::transfer(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 1) return rejected(kRejectMalformed);
  const uint8_t n = payload[0];
  size_t at = 1, total = 0;
  uint64_t ns = 0;   // the whole request's line time: the times asked and the clock
  bool too_long = false;
  for (uint8_t t = 0; t < n; ++t) {
    if (at + kItem > length) return rejected(kRejectMalformed);
    const uint16_t count = getU16(payload + at + 12);
    for (int k = 0; k < 3; ++k) {
      const uint32_t v = getU32(payload + at + 4 * k);
      if (v > kMaxDelayNs) too_long = true;
      if (k || t) ns += v;
    }
    at += kItem + count;
    if (at > length) return rejected(kRejectMalformed);
    total += count;
  }
  Tail tail;
  const Result parsed = plainTail(tail, payload, length, at, out, capacity);
  if (refused(parsed)) return parsed;
  if (n == 0) return rejected(kRejectMalformed);
  if (too_long || total > kMaxLength) return unsupportedValue(out, capacity);
  if (!started_) return wrongState(out, capacity);
  ns += static_cast<uint64_t>(total) * 16 * nsOf(half_);
  if (ns > static_cast<uint64_t>(kMaxOpMs) * 1000000u) return unsupportedValue(out, capacity);   // past max_op_ms
  if (capacity < kItem * n + total) return failed();
  at = 1;
  size_t put = 0;
  for (uint8_t t = 0; t < n; ++t) {
    const uint32_t gap = t ? cyclesOf(getU32(payload + at)) : 0;
    const uint32_t setup = cyclesOf(getU32(payload + at + 4)), hold = cyclesOf(getU32(payload + at + 8));
    const uint16_t count = getU16(payload + at + 12);
    lineTransfer(t == 0, gap, setup, hold, payload + at + kItem, out + put + kItem, count);
    putU32(out + put, nsOf(gap));
    putU32(out + put + 4, nsOf(setup));
    putU32(out + put + 8, nsOf(hold));
    putU16(out + put + 12, count);
    at += kItem + count;
    put += kItem + count;
    if (transfers_ != 0xffffffffu) ++transfers_;
  }
  return completed(put);
}

Result SpiController::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (operation) {
    case kOpConfigure: {   // mode(u8) bit_order(u8) clock_hz(u32) [TLV] -> clock_hz(u32) [TLV]
      const Result parsed = plainTail(tail, payload, length, 6, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t clock = getU32(payload + 2);
      if (payload[0] > 3 || payload[1] > 1 || clock == 0) return rejected(kRejectMalformed);
      if (!(kModes >> payload[0] & 1) || clock < kMinClockHz || clock > kMaxClockHz) return unsupportedValue(out, capacity);
      if (sck_ < 0) return wrongState(out, capacity);   // needs a plan (fixture §6: unavailable cause 6)
      if (capacity < 4) return failed();
      stop();
      mode_ = payload[0]; bit_order_ = payload[1]; transfers_ = 0;
      const uint32_t cpu = tickHz();
      half_ = (cpu + 2 * clock - 1) / (2 * clock);   // the SCK used: the nearest at or below the one asked
      if (!start()) { stop(); mode_ = bit_order_ = 0; return failed(); }
      putU32(out, cpu / (2 * half_));
      return completed(4);
    }
    case kOpTransfer:
      return transfer(payload, length, out, capacity);
    case kOpStatus: {   // [TLV] -> state(u8) mode(u8) bit_order(u8) transfers(u32) [TLV]
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 7) return failed();
      out[0] = started_ ? reg::fixture_spi_controller::kStateRunning : reg::fixture_spi_controller::kStateUnconfigured;
      out[1] = mode_;
      out[2] = bit_order_;
      putU32(out + 3, transfers_);
      return completed(7);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep
