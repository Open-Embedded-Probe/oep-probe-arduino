// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.wire.swd / oep.target.arm-adi (OepSwd.cpp) over a simulated SWD target (shim/fake_swd_io.h).
// - Idle items (oep-if-debug §1): count = 0 and an attach without pins leave out every channel with an idle item; a
//   request naming a channel whose idle is an output is refused unavailable cause 5, holder_kind 7; an input idle named
//   is accepted.
// - Wire loss (oep-if-debug §2): a request that gets nothing back answers status line and keeps the connection; it closes
//   only after wire_lost_ms of failures with no answer between.
#include <stdio.h>

#include <vector>

#include "OepSwd.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace sw = reg::wire_swd;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

static Result call(Interface &i, uint8_t op, const Bytes &payload, Bytes &out) {
  out.assign(512, 0);
  const Result r = i.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static Bytes attachRequest(int swdio = -1, int swclk = -1) {
  Bytes p = {0, uint8_t(sw::kTlvAttachMaxSpeed | kTagCritical), 4, 0x40, 0x42, 0x0f, 0x00};   // 1 MHz
  if (swdio >= 0) p.insert(p.end(), {uint8_t(sw::kTlvAttachPins | kTagCritical), 4, uint8_t(swdio), 0, uint8_t(swclk), 0});
  return p;
}
static Bytes u16(uint16_t v) { return {uint8_t(v), uint8_t(v >> 8)}; }
static const uint8_t *tlv(const Bytes &out, size_t from, uint8_t tag, size_t &len) {
  for (size_t at = from; at + 2 <= out.size(); at += 2u + out[at + 1])
    if (out[at] == tag && at + 2u + out[at + 1] <= out.size()) { len = out[at + 1]; return out.data() + at + 2; }
  return nullptr;
}
static bool isSettingsIdle(const Result &r, const Bytes &out, uint16_t channel) {
  if (r.resolution != kResolutionRejected || r.detail != kRejectUnavailable) return false;
  size_t len = 0;
  const uint8_t *cause = tlv(out, 0, reg::core::kTlvUnavailablePayloadCause, len);
  const uint8_t *ch = tlv(out, 0, reg::core::kTlvUnavailablePayloadChannel, len);
  const uint8_t *kind = tlv(out, 0, reg::core::kTlvUnavailablePayloadHolderKind, len);
  return cause && cause[0] == reg::core::kUnavailableCauseHeldBySettings && ch && (ch[0] | ch[1] << 8) == channel &&
         kind && kind[0] == reg::core::kHolderKindSettingsIdle;
}

int main() {
  Bytes out;

  // ---- a fixed pair: attach, a DP read through arm-adi, detach ----
  static PinTable fixed_pins((1ull << 2) | (1ull << 3));
  static SwdPort fixed{2, 3};
  fixed.pins = &fixed_pins;
  static WireSwd wire(fixed, 0);
  static TargetArmAdi adi(fixed, 0);
  Result r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
  CHECK(ok(r) && out.size() >= 11 && fixed.connected);
  CHECK((out[2] | out[3] << 8 | out[4] << 16 | uint32_t(out[5]) << 24) == g_swd.dpidr);
  {
    Bytes t = u16(fixed.number);
    t.insert(t.end(), {1, 0, 0x06});   // one DP read of CTRL/STAT (A = 1)
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(ok(r) && out.size() >= 10 && out[2] == kStatusOk);
  }
  r = call(wire, WireSwd::kOpDetach, u16(fixed.number), out);
  CHECK(ok(r) && !fixed.connected);

  // ---- idle items on the fixed pair (debug §1) ----
  {
    CHECK(fixed_pins.setIdle(3, PinTable::kIdlePullUp));
    const Bytes scan_all = {0};
    const uint32_t before = g_swd.requests;
    r = call(wire, WireSwd::kOpScan, scan_all, out);
    CHECK(ok(r) && out.size() >= 2 && out[0] == 0 && g_swd.requests == before);   // not in count = 0
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);                     // not a candidate: none left
    CHECK(isSettingsIdle(r, out, 3) && !fixed.connected);
    r = call(wire, WireSwd::kOpAttach, attachRequest(2, 3), out);                 // named, an input idle: accepted
    CHECK(ok(r) && fixed.connected);
    r = call(wire, WireSwd::kOpScan, scan_all, out);                              // the live pair is listed
    CHECK(ok(r) && out[0] == 1 && out[1] == 1);
    r = call(wire, WireSwd::kOpDetach, u16(fixed.number), out);
    CHECK(ok(r));
    CHECK(fixed_pins.setIdle(2, PinTable::kIdleOutputLow));
    r = call(wire, WireSwd::kOpAttach, attachRequest(2, 3), out);                 // named, an output idle: refused
    CHECK(isSettingsIdle(r, out, 2));
    const Bytes scan_named = {1, 2, 0, 3, 0};
    r = call(wire, WireSwd::kOpScan, scan_named, out);
    CHECK(isSettingsIdle(r, out, 2));
    CHECK(fixed_pins.setIdle(2, PinTable::kIdleUnset) && fixed_pins.setIdle(3, PinTable::kIdleUnset));
  }

  // ---- host-chosen pins 4-7: count = 0 leaves out the channels with an idle item ----
  {
    static PinTable pins((1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7));
    static SwdPort chosen{0xfffe, 0xfffe};
    chosen.pins = &pins;
    chosen.pin_choice = (1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7);
    static WireSwd wire2(chosen, 1);
    CHECK(pins.setIdle(4, PinTable::kIdlePullDown));
    const Bytes scan_all = {0};
    r = call(wire2, WireSwd::kOpScan, scan_all, out);
    CHECK(ok(r) && out.size() >= 2 && out[0] == 6 && out[1] == 6);   // 5, 6, 7 in order: no pair with 4
    for (size_t at = 2; at + 10 <= out.size(); at += 10) CHECK(out[at + 2] != 4 && out[at + 4] != 4);
    CHECK(pins.setIdle(5, PinTable::kIdleOutputHigh));
    r = call(wire2, WireSwd::kOpAttach, attachRequest(5, 6), out);
    CHECK(isSettingsIdle(r, out, 5) && !chosen.connected);
    r = call(wire2, WireSwd::kOpAttach, attachRequest(4, 6), out);   // the pull-down idle on 4, named: accepted
    CHECK(ok(r) && chosen.connected);
    r = call(wire2, WireSwd::kOpDetach, u16(chosen.number), out);
    CHECK(ok(r));
  }

  // ---- wire loss: status line keeps the connection until wire_lost_ms of failures ----
  {
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected);
    Bytes t = u16(fixed.number);
    t.insert(t.end(), {1, 0, 0x06});
    g_swd.absent = true;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine && fixed.connected);   // 0.0.28: closed at once
    g_millis += 600;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(out[2] == kStatusLine && fixed.connected);
    g_millis += 600;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(out[2] == kStatusLine && !fixed.connected);                     // 1000 ms of failures: closed
    g_swd.absent = false;
  }

  printf("swd: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
