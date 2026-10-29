// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepCounter.h"

// Not on the X035 (kRoomy in the Firmware): its interrupts / DAC code would be linked by this file alone
#if !defined(ARDUINO_CH32X035)

#if defined(OEP_HOST_FAKE_COUNTER)
#elif defined(ARDUINO_ARCH_ESP32)
// ESP-IDF's GPIO matrix initializer leaves fields zero; silence its aggregate warning only in the SDK header.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include <hal/gpio_ll.h>
#pragma GCC diagnostic pop
#include <soc/gpio_struct.h>
#if defined(OEP_COUNTER_PCNT)
#include <driver/pulse_cnt.h>
#endif
#elif defined(ARDUINO_ARCH_RP2040)
#include <hardware/gpio.h>
#endif

namespace oep {

namespace {

volatile uint32_t gCounts[FixtureCounter::kMaxChannels];
void edge0() { gCounts[0] = gCounts[0] + 1; }
void edge1() { gCounts[1] = gCounts[1] + 1; }
void edge2() { gCounts[2] = gCounts[2] + 1; }
void edge3() { gCounts[3] = gCounts[3] + 1; }
void (*const kEdges[FixtureCounter::kMaxChannels])() = {edge0, edge1, edge2, edge3};

#if !defined(OEP_HOST_FAKE_COUNTER)
// The pad's input on, its output (if any) left as it is
void listen(int pad) {
#if defined(ARDUINO_ARCH_ESP32)
  gpio_ll_input_enable(&GPIO, static_cast<uint32_t>(pad));
#elif defined(ARDUINO_ARCH_RP2040)
  gpio_set_input_enabled(static_cast<uint>(pad), true);
#else
  (void)pad;   // a CH32's input register follows the pad in any mode
#endif
}
#endif

#if defined(OEP_COUNTER_PCNT)
// One unit an input, made for one measure and freed after it: the 16-bit count carried at a watch point (accum_count)
constexpr int kLimit = 30000;
struct Unit {
  pcnt_unit_handle_t unit = nullptr;
  pcnt_channel_handle_t channel = nullptr;
};
bool unitUp(Unit &u, int pad, uint8_t edge) {
  pcnt_unit_config_t uc = {};
  uc.low_limit = -1;
  uc.high_limit = kLimit;
  uc.flags.accum_count = 1;
  if (pcnt_new_unit(&uc, &u.unit) != ESP_OK) return false;
  pcnt_chan_config_t cc = {};
  cc.edge_gpio_num = pad;
  cc.level_gpio_num = -1;
  namespace cnt = reg::fixture_counter;
  const auto on = PCNT_CHANNEL_EDGE_ACTION_INCREASE, off = PCNT_CHANNEL_EDGE_ACTION_HOLD;
  if (pcnt_unit_add_watch_point(u.unit, kLimit) != ESP_OK || pcnt_new_channel(u.unit, &cc, &u.channel) != ESP_OK ||
      pcnt_channel_set_edge_action(u.channel, edge == cnt::kEdgeFalling ? off : on, edge == cnt::kEdgeRising ? off : on) != ESP_OK ||
      pcnt_unit_enable(u.unit) != ESP_OK || pcnt_unit_clear_count(u.unit) != ESP_OK)
    return false;
  listen(pad);   // the channel turned the pad's input on; an output on it (a PWM counted on itself) is left as it is
  return true;
}
void unitDown(Unit &u) {
  if (u.unit) pcnt_unit_disable(u.unit);
  if (u.channel) pcnt_del_channel(u.channel);
  if (u.unit) pcnt_del_unit(u.unit);
  u = Unit{};
}
#endif

}  // namespace

#if defined(OEP_HOST_FAKE_COUNTER)
uint8_t gHostEdge = 0xff;   // the edges counted: rising / falling / both, set by measure
void FixtureCounter::hostEdge(uint8_t role) { if (role < kMaxChannels) kEdges[role](); }
#endif

uint8_t FixtureCounter::planCheck(const RoleAssignment *roles, size_t count) {
  uint8_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role >= kMaxChannels || ((seen >> roles[i].role) & 1) || !pins_.allowed(roles[i].channel))
      return kRejectUnsupported;
    seen |= 1u << roles[i].role;
  }
  return 0;   // it only listens: shared with anything (fixture §8)
}

bool FixtureCounter::planApply(const RoleAssignment *roles, size_t count) {
  for (int &c : channel_) c = -1;
  for (size_t i = 0; i < count; ++i) channel_[roles[i].role] = roles[i].channel;
  return true;
}

void FixtureCounter::planRelease() {
  for (int &c : channel_) c = -1;
}

size_t FixtureCounter::describe(uint8_t *out, size_t capacity) {
  static const uint8_t kRoles[] = {0, 1, 2, 3};
  TlvWriter w(out, capacity);
  w.roleChannels(kRoles, kMaxChannels, pins_.allowedMask());
  w.u32(kTagMaxClockHz, kMaxHz);
  w.u8(reg::fixture_counter::kTlvDescribeChannels, kMaxChannels);
  w.u32(reg::fixture_counter::kTlvDescribeMaxGateUs, kMaxGateUs);
  return w.ok() ? w.length() : 0;
}

Result FixtureCounter::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  namespace cnt = reg::fixture_counter;
  if (operation != kOpMeasure) return rejected(kRejectUnknownOperation);
  Tail tail;   // gate_us(u32) edge(u8) [TLV] -> window_ns(u64) n(u8) n x (role(u8) count(u32)) [TLV]
  const Result parsed = plainTail(tail, payload, length, 5, out, capacity);
  if (refused(parsed)) return parsed;
  const uint32_t gate = getU32(payload);
  const uint8_t edge = payload[4];
  if (gate == 0 || edge > cnt::kEdgeBoth) return rejected(kRejectMalformed);
  if (gate > kMaxGateUs) return unsupportedValue(out, capacity);
  uint8_t n = 0;
  for (const int c : channel_) n += c >= 0;
  if (!n) return wrongState(out, capacity);   // no plan (fixture §8: unavailable cause 6)
  if (capacity < 9u + 5u * n) return failed();
  for (uint8_t r = 0; r < kMaxChannels; ++r) gCounts[r] = 0;
#if defined(OEP_HOST_FAKE_COUNTER)
  gHostEdge = edge;
  host_gate_us = gate;
  const uint64_t t0 = nowNs();
  if (g_on_wait) g_on_wait();   // the test's edges, within the gate
  const uint64_t t1 = t0 + static_cast<uint64_t>(gate) * 1000u;
#elif defined(OEP_COUNTER_PCNT)
  Unit units[kMaxChannels];
  bool up = true;
  for (uint8_t r = 0; r < kMaxChannels && up; ++r)
    if (channel_[r] >= 0) up = unitUp(units[r], platformPad(channel_[r]), edge);
  if (!up) {
    for (Unit &u : units) unitDown(u);
    return failed();
  }
  const uint64_t t0 = nowNs();
  for (Unit &u : units) if (u.unit) pcnt_unit_start(u.unit);
  for (const uint32_t from = micros(); micros() - from < gate;) delay(gate - (micros() - from) > 2000 ? 1 : 0);
  for (Unit &u : units) if (u.unit) pcnt_unit_stop(u.unit);
  const uint64_t t1 = nowNs();
  for (uint8_t r = 0; r < kMaxChannels; ++r) {
    int value = 0;
    if (units[r].unit && pcnt_unit_get_count(units[r].unit, &value) == ESP_OK) gCounts[r] = value < 0 ? 0 : static_cast<uint32_t>(value);
    unitDown(units[r]);
  }
#else
  const auto mode = edge == cnt::kEdgeRising ? RISING : edge == cnt::kEdgeFalling ? FALLING : CHANGE;
  for (uint8_t r = 0; r < kMaxChannels; ++r) {
    if (channel_[r] < 0) continue;
    const int pad = platformPad(channel_[r]);
    listen(pad);
    attachInterrupt(digitalPinToInterrupt(pad), kEdges[r], mode);
  }
  const uint64_t t0 = nowNs();
  for (const uint32_t from = micros(); micros() - from < gate;) delay(gate - (micros() - from) > 2000 ? 1 : 0);
  for (uint8_t r = 0; r < kMaxChannels; ++r) if (channel_[r] >= 0) detachInterrupt(digitalPinToInterrupt(platformPad(channel_[r])));
  const uint64_t t1 = nowNs();
#endif
  putU64(out, t1 - t0);
  out[8] = n;
  size_t at = 9;
  for (uint8_t r = 0; r < kMaxChannels; ++r) {
    if (channel_[r] < 0) continue;
    out[at] = r;
    putU32(out + at + 1, gCounts[r]);
    at += 5;
  }
  return completed(at);
}

}  // namespace oep

#endif
