// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.wire.swd / oep.target.arm-adi (OepSwd.cpp) over a simulated SWD target (shim/fake_swd_io.h).
// - attach's fixed part method swdio swclk max_speed, a new connection only (oep-if-debug §1, §5); scan lists its pairs
//   (count 1 or more); connections as §2.1 (tid scheme 2, the DPIDR; the selector in TLV selected_target).
// - Idle items (oep-if-debug §1): a request naming a channel whose idle is an output is refused unavailable cause 5; an
//   input idle is accepted.
// - Wire loss (oep-if-debug §2): a request that gets nothing back answers status line and keeps the connection; it closes
//   only after wire_lost_ms of failures with no answer between.
// - Wire retries (oep-if-debug §2, §5): a transfer that got nothing back is tried again after the line reset (and the
//   dormant wake, TARGETSEL, the DPIDR read) within wire_retry_ms; an AP read with a bad parity is not repeated. attach
//   tries the wake again within it.
// - Idle cycles and the lines while the wire does not answer (oep-if-debug §2, §5): 8 idle cycles after a read whose
//   parity failed and before a detach lets the lines go; from a transfer with no answer until one answers, the lines
//   are free between exchanges (released, no pulls) and the rest state comes back with the answer.
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
// attach: method 0 (run), swdio(u16) swclk(u16) max_speed(u32 Hz) (oep-if-debug §1, §5)
static Bytes attachRequest(uint16_t swdio = 2, uint16_t swclk = 3, uint32_t hz = 1000000) {
  return {sw::kAttachMethodRun, uint8_t(swdio), uint8_t(swdio >> 8), uint8_t(swclk), uint8_t(swclk >> 8),
          uint8_t(hz), uint8_t(hz >> 8), uint8_t(hz >> 16), uint8_t(hz >> 24)};
}
static Bytes u32(uint32_t v) { return {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)}; }
static uint32_t at32(const Bytes &b, size_t at) {
  return b[at] | b[at + 1] << 8 | b[at + 2] << 16 | uint32_t(b[at + 3]) << 24;
}
// one DP read of CTRL/STAT (A = 1) on `connection`
static Bytes dpRead(uint32_t connection) {
  Bytes t = u32(connection);
  t.insert(t.end(), {1, 0, 0x06});
  return t;
}
static uint8_t causeOf(const Bytes &out);
// The value of answer TLV `tag` after `from` (core §2.2: tag(u8) len(u16) value), or nullptr.
static const uint8_t *tlv(const Bytes &out, size_t from, uint8_t tag, size_t &len) {
  for (size_t at = from; at + 3 <= out.size(); at += 3u + (out[at + 1] | out[at + 2] << 8)) {
    const size_t n = out[at + 1] | out[at + 2] << 8;
    if (out[at] == tag && at + 3u + n <= out.size()) { len = n; return out.data() + at + 3; }
  }
  return nullptr;
}
static bool isSettingsIdle(const Result &r, const Bytes &out, uint16_t channel) {
  if (r.resolution != kResolutionRejected || r.detail != kRejectUnavailable) return false;
  size_t len = 0;
  const uint8_t *cause = tlv(out, 0, reg::core::kTlvUnavailablePayloadCause, len);
  const uint8_t *ch = tlv(out, 0, reg::core::kTlvUnavailablePayloadChannel, len);
  return cause && cause[0] == reg::core::kUnavailableCauseHeldBySettings && ch && (ch[0] | ch[1] << 8) == channel &&
         out.size() == 4 + 5;   // cause and channel only (core §4.3: no holder_kind)
}

static bool isHeld(const Result &r, const Bytes &out, uint16_t channel) {
  if (r.resolution != kResolutionRejected || r.detail != kRejectUnavailable) return false;
  size_t len = 0;
  const uint8_t *cause = tlv(out, 0, reg::core::kTlvUnavailablePayloadCause, len);
  const uint8_t *ch = tlv(out, 0, reg::core::kTlvUnavailablePayloadChannel, len);
  return cause && cause[0] == reg::core::kUnavailableCausePinInUse && ch && (ch[0] | ch[1] << 8) == channel &&
         out.size() == 4 + 5;
}

static uint8_t causeOf(const Bytes &out) {
  size_t len = 0;
  const uint8_t *cause = tlv(out, 0, reg::core::kTlvUnavailablePayloadCause, len);
  return cause ? cause[0] : 0;
}

int main() {
  Bytes out;

  // ---- a fixed pair: attach, a DP read through arm-adi, connections, detach ----
  static PinTable fixed_pins((1ull << 2) | (1ull << 3));
  static SwdPort fixed{2, 3};
  fixed.pins = &fixed_pins;
  static WireSwd wire(fixed, 0);
  static TargetArmAdi adi(fixed, 0);
  Result r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
  CHECK(ok(r) && out.size() == 13 && fixed.connected && at32(out, 0) == fixed.number);
  CHECK(at32(out, 4) == g_swd.dpidr);   // connection(u32) DPIDR(u32) flags(u8) speed_hz(u32)
  {
    r = call(adi, TargetArmAdi::kOpTransfer, dpRead(fixed.number), out);
    CHECK(ok(r) && out.size() >= 10 && out[2] == kStatusOk);
    // n = 0 (oep-spec vector "arm-adi transfer: n = 0", debug §6): success, done 0, status ok, ack 0, nvals 0
    Bytes none = u32(fixed.number);
    none.insert(none.end(), {0, 0});
    r = call(adi, TargetArmAdi::kOpTransfer, none, out);
    CHECK(ok(r) && out == (Bytes{0, 0, kStatusOk, 0, 0, 0}));
    // connections: connection swdio swclk speed_hz tid_scheme 2 tid_len 4 DPIDR, no selector TLV (§2.1)
    r = call(wire, WireSwd::kOpConnections, {0}, out);
    CHECK(ok(r) && out.size() == 20 && out[1] == 1 && at32(out, 2) == fixed.number && out[6] == 2 && out[8] == 3);
    CHECK(out[14] == reg::common::kTargetIdSchemeDpidr && out[15] == 4 && at32(out, 16) == g_swd.dpidr);
    // a new connection only: the live pair again is in use (cause 1, oep-if-debug §1)
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(r.detail == kRejectUnavailable && causeOf(out) == reg::core::kUnavailableCausePinInUse);
  }
  r = call(wire, WireSwd::kOpDetach, u32(fixed.number), out);
  CHECK(ok(r) && !fixed.connected);
  // arm-adi on a connection it does not know: the form first, no_connection last (core §4.3 order 8)
  {
    Bytes t = u32(fixed.number + 100);
    t.insert(t.end(), {1, 0, 0x16});   // req with bit 4 set: unsupported, the fixed part (oep-if-debug §6)
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out == Bytes({0}));
    t.back() = 0x06;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectNoResource);
  }

  // ---- core §4.3's order: every format error before anything unsupported ----
  {
    r = call(wire, WireSwd::kOpAttach, {1}, out);   // method 1 (unsupported) with a short fixed part (malformed)
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    Bytes a = attachRequest();   // the reset TLV (not offered: an unknown critical tag) and a TLV cut short
    a.insert(a.end(), {uint8_t(sw::kTlvAttachReset | kTagCritical), 4, 0, 8, 0, 10, 0, uint8_t(sw::kTlvAttachTargetsel), 4, 0, 1});
    r = call(wire, WireSwd::kOpAttach, a, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    a.resize(a.size() - 4);   // the reset TLV alone: unsupported, the tag as received
    r = call(wire, WireSwd::kOpAttach, a, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported &&
          out == Bytes({uint8_t(sw::kTlvAttachReset | kTagCritical)}));
    r = call(wire, WireSwd::kOpScan, {0}, out);   // count 0: malformed (oep-if-debug §1)
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    r = call(wire, WireSwd::kOpScan, {2, 2, 0, 3, 0, 2, 0, 3, 0}, out);   // the same pair twice: malformed
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    Bytes zero = attachRequest(2, 3, 0);   // max_speed 0: malformed
    r = call(wire, WireSwd::kOpAttach, zero, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    r = call(wire, WireSwd::kOpAttach, {1, 2, 0, 3, 0, 0x40, 0x42, 0x0f, 0}, out);   // method 1: unsupported, 0x00
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out == Bytes({0}));
  }

  // ---- idle items on the fixed pair (debug §1) ----
  {
    CHECK(fixed_pins.setIdle(3, PinTable::kIdlePullUp));
    r = call(wire, WireSwd::kOpAttach, attachRequest(2, 3), out);                 // an input idle: accepted
    CHECK(ok(r) && fixed.connected);
    r = call(wire, WireSwd::kOpScan, {1, 2, 0, 3, 0}, out);                       // the live pair is never scanned
    CHECK(r.detail == kRejectUnavailable && causeOf(out) == reg::core::kUnavailableCausePinInUse);
    r = call(wire, WireSwd::kOpDetach, u32(fixed.number), out);
    CHECK(ok(r));
    CHECK(fixed_pins.setIdle(2, PinTable::kIdleOutputLow));
    r = call(wire, WireSwd::kOpAttach, attachRequest(2, 3), out);                 // an output idle: refused
    CHECK(isSettingsIdle(r, out, 2));
    r = call(wire, WireSwd::kOpScan, {1, 2, 0, 3, 0}, out);
    CHECK(isSettingsIdle(r, out, 2));
    CHECK(fixed_pins.setIdle(2, PinTable::kIdleUnset) && fixed_pins.setIdle(3, PinTable::kIdleUnset));
  }

  // ---- host-chosen pins 4-7 ----
  {
    static PinTable pins((1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7));
    static SwdPort chosen{0xfffe, 0xfffe};
    chosen.pins = &pins;
    chosen.pin_choice = (1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 7);
    static WireSwd wire2(chosen, 1);
    r = call(wire2, WireSwd::kOpScan, {2, 5, 0, 6, 0, 6, 0, 7, 0}, out);
    // tried 2, found 2: count x (kind swdio(u16) swclk(u16) id(u32)), 9 bytes each, no element length
    CHECK(ok(r) && out.size() == 2u + 9u * 2u && out[0] == 2 && out[1] == 2 && out[2] == sw::kScanKindArmAdi);
    CHECK(pins.setIdle(5, PinTable::kIdleOutputHigh));
    r = call(wire2, WireSwd::kOpAttach, attachRequest(5, 6), out);
    CHECK(isSettingsIdle(r, out, 5) && !chosen.connected);
    CHECK(pins.setIdle(5, PinTable::kIdleUnset) && pins.setIdle(4, PinTable::kIdlePullDown));
    r = call(wire2, WireSwd::kOpAttach, attachRequest(4, 6), out);   // the pull-down idle on 4: accepted
    CHECK(ok(r) && chosen.connected);
    r = call(wire2, WireSwd::kOpAttach, attachRequest(5, 7), out);   // the one seat taken: cause 2
    CHECK(r.detail == kRejectUnavailable && causeOf(out) == reg::core::kUnavailableCauseLimit);
    r = call(wire2, WireSwd::kOpAttach, attachRequest(6, 7), out);   // the live connection's pin: cause 1
    CHECK(r.detail == kRejectUnavailable && causeOf(out) == reg::core::kUnavailableCausePinInUse);
    r = call(wire2, WireSwd::kOpDetach, u32(chosen.number), out);
    CHECK(ok(r));
    // a channel a plan holds: unavailable cause 1, the channel (core §4.3)
    CHECK(pins.claim(7, 0x01));
    r = call(wire2, WireSwd::kOpAttach, attachRequest(6, 7), out);
    CHECK(isHeld(r, out, 7) && !chosen.connected);
    r = call(wire2, WireSwd::kOpScan, {1, 6, 0, 7, 0}, out);
    CHECK(isHeld(r, out, 7));
    // a combination the declaration does not allow, listed after the held one: unsupported, tag 0x00 and TLV 0x40 its
    // index, before any held channel (oep-if-debug §1, core §4.3 order 6 before 7)
    r = call(wire2, WireSwd::kOpScan, {2, 6, 0, 7, 0, 7, 0, 7, 0}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out == Bytes({0, 0x40, 1, 0, 1}));
    pins.release(0x01);
  }

  // ---- TARGETSEL: kept apart from the identity read, shown in connections (oep-if-debug §2.1, §5) ----
  {
    Bytes a = attachRequest();
    a.insert(a.end(), {uint8_t(sw::kTlvAttachTargetsel | kTagCritical), 4, 0, 0x21, 0x43, 0x65, 0x07});
    r = call(wire, WireSwd::kOpAttach, a, out);
    CHECK(ok(r) && fixed.connected);
    r = call(wire, WireSwd::kOpConnections, {0}, out);
    size_t len = 0;
    const uint8_t *v = tlv(out, 20, sw::kTlvConnectionsAnswerSelectedTarget, len);
    CHECK(ok(r) && v && len == 5 && v[0] == 0 && (v[1] | v[2] << 8 | v[3] << 16 | uint32_t(v[4]) << 24) == 0x07654321u);
    r = call(wire, WireSwd::kOpDetach, u32(fixed.number), out);
    CHECK(ok(r));
  }

  // ---- wire loss: status line keeps the connection until wire_lost_ms of failures ----
  {
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected);
    const Bytes t = dpRead(fixed.number);
    g_swd.absent = true;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine && fixed.connected);
    g_millis += 600;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(out[2] == kStatusLine && fixed.connected);
    g_millis += 600;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(out[2] == kStatusLine && !fixed.connected);                     // 1000 ms of failures: closed
    g_swd.absent = false;
  }

  // ---- wire retries (oep-if-debug §2, §5): a transfer that got nothing back, retried after the line reset ----
  {
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected);
    const Bytes t = dpRead(fixed.number);
    g_swd.ctrl = 0xf0000000u;
    g_swd.drop_requests = 3;           // three requests get no reply (the DPIDR reads of the retries count too)
    const uint32_t resets = g_swd.line_resets;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(ok(r) && out.size() >= 10 && out[2] == kStatusOk && fixed.connected);
    CHECK(at32(out, 6) == 0xf0000000u);
    CHECK(g_swd.line_resets > resets);
    // a DP read with a bad data parity: read again after the relink
    g_swd.parity_reads = 1;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(ok(r) && out[2] == kStatusOk);
    // an AP read with a bad data parity is not repeated (the target took it): status line, one AP read, still connected
    Bytes ap = u32(fixed.number);
    ap.insert(ap.end(), {1, 0, 0x03});   // AP read, A = 0
    g_swd.parity_reads = 1;
    const uint32_t ap_reads = g_swd.ap_reads;
    r = call(adi, TargetArmAdi::kOpTransfer, ap, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine && g_swd.ap_reads == ap_reads + 1 && fixed.connected);
    // nothing answers: the retries stop inside wire_retry_ms (plus the first try and one round's estimate)
    g_swd.absent = true;
    const uint32_t before = micros();
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    const uint32_t took = micros() - before;
    printf("  a transfer with nothing there: %u ms\n", took / 1000);
    CHECK(out[2] == kStatusLine && took >= 150000u && took <= limits::kWireRetryMs * 1000u + 2000u);
    g_swd.absent = false;
    r = call(wire, WireSwd::kOpDetach, u32(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
  }

  // ---- the idle cycles (oep-if-debug §5): after a read whose data parity failed, and before a detach lets the lines
  //      go ----
  {
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected);
    Bytes ap = u32(fixed.number);
    ap.insert(ap.end(), {1, 0, 0x03});   // an AP read: a bad parity is not retried
    g_swd.parity_reads = 1;
    r = call(adi, TargetArmAdi::kOpTransfer, ap, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine && g_swd.idle_run >= 8);
    r = call(adi, TargetArmAdi::kOpTransfer, dpRead(fixed.number), out);   // answers: the rest state again
    CHECK(ok(r) && fixed.io.driven && !fixed.rest_free);
    const uint32_t before = g_swd.idle_run;
    r = call(wire, WireSwd::kOpDetach, u32(fixed.number), out);
    // oep-if-debug §2: the 8 idle cycles, then the lines held driven at the rest (not a lost line); holds lists the port
    CHECK(ok(r) && g_swd.idle_run >= before + 8 && fixed.io.driven && fixed.holding);
    r = call(wire, reg::wire_swd::kOpHolds, Bytes{0}, out);
    CHECK(ok(r) && out.size() == 7 && out[1] == 1);
  }

  // ---- the lines while the wire does not answer (oep-if-debug §2, §5): from a transfer with no answer until one
  //      answers, released between exchanges, SWDIO without its pull-up; an answer restores the rest state ----
  {
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected && fixed.io.driven);
    const Bytes t = dpRead(fixed.number);
    g_swd.absent = true;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine && fixed.connected);
    CHECK(!fixed.io.driven && fixed.rest_free && g_swd_pull[fixed.swdio] == 0 && g_swd_pull[fixed.swclk] == 0);
    CHECK(g_swd.idle_run >= 8);   // the last exchange's idle cycles before the release
    g_swd.absent = false;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(ok(r) && fixed.io.driven && !fixed.rest_free);
    r = call(wire, WireSwd::kOpDetach, u32(fixed.number), out);
    CHECK(ok(r));
  }

  // ---- attach: the wake tried again within wire_retry_ms; the slowest clock inside the attach budget ----
  {
    g_swd.drop_requests = 2;   // the first wake's two DPIDR reads (after JTAG-to-SWD, after the dormant wake)
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected);
    r = call(wire, WireSwd::kOpDetach, u32(fixed.number), out);
    CHECK(ok(r));
    // a lower max_speed (100 kHz): the connection at or under it, and transfers work at that speed
    r = call(wire, WireSwd::kOpAttach, attachRequest(2, 3, 100000), out);
    CHECK(ok(r) && at32(out, 9) <= 100000u && fixed.io.half_ns >= 5000);
    r = call(adi, TargetArmAdi::kOpTransfer, dpRead(fixed.number), out);
    CHECK(ok(r) && out[2] == kStatusOk);
    r = call(wire, WireSwd::kOpDetach, u32(fixed.number), out);
    CHECK(ok(r));
    g_swd.absent = true;
    const uint32_t before = micros();
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    const uint32_t took = micros() - before;
    printf("  an attach with nothing there: %u ms\n", took / 1000);
    CHECK(r.resolution == kResolutionCompleted && out.size() >= 1 && out[0] == kStatusLine && !fixed.connected);
    CHECK(took >= 150000u && took <= limits::kWireRetryMs * 1000u + 2000u);
    // at the slowest clock this wire takes (min_clock_hz 10 kHz): still well inside the attach budget
    const uint32_t before_slow = micros();
    r = call(wire, WireSwd::kOpAttach, attachRequest(2, 3, 10000), out);
    const uint32_t took_slow = micros() - before_slow;
    printf("  an attach with nothing there at 10 kHz: %u ms\n", took_slow / 1000);
    CHECK(r.resolution == kResolutionCompleted && out.size() >= 1 && out[0] == kStatusLine);
    CHECK(took_slow <= limits::kAttachBudgetMs * 1000u);
    r = call(wire, WireSwd::kOpAttach, attachRequest(2, 3, 9999), out);   // under min_clock_hz: unsupported, 0x00
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out == Bytes({0}));
    g_swd.absent = false;
  }

  // ---- the target's power gone and back under a live connection: the transfer's wire retries bring the port back
  //      (line reset, wake, the DPIDR read first) while the wire is not yet lost (oep-if-debug §2) ----
  {
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected);
    const uint32_t number = fixed.number;
    const Bytes t = dpRead(fixed.number);
    g_swd.absent = true;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine && fixed.connected);
    g_millis += 500;
    g_swd.absent = false;
    g_swd.locked = true;   // powered up again: DPIDR first
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(ok(r) && out[2] == kStatusOk && fixed.connected && fixed.number == number);
    r = call(wire, WireSwd::kOpDetach, u32(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
  }

  printf("swd: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
