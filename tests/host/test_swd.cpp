// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.wire.swd / oep.target.arm-adi (OepSwd.cpp) over a simulated SWD target (shim/fake_swd_io.h).
// - Idle items (oep-if-debug §1): count = 0 and an attach without pins leave out every channel with an idle item; a
//   request naming a channel whose idle is an output is refused unavailable cause 5, holder_kind 7; an input idle named
//   is accepted.
// - Wire loss (oep-if-debug §2): a request that gets nothing back answers status line and keeps the connection; it closes
//   only after wire_lost_ms of failures with no answer between.
// - Wire retries (oep-if-debug §2, §5): a transfer that got nothing back is tried again after the line reset (and the
//   dormant wake, TARGETSEL, the DPIDR read) within wire_retry_ms; an AP read with a bad parity is not repeated. attach
//   tries the wake again within it, and search_retries counts the failed wakes.
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
// attach: method 0, max_speed 1 MHz (critical), [pins (critical)]; every TLV tag(u8) len(u16) value (core §2.2)
static Bytes attachRequest(int swdio = -1, int swclk = -1) {
  Bytes p = {0, uint8_t(sw::kTlvAttachMaxSpeed | kTagCritical), 4, 0, 0x40, 0x42, 0x0f, 0x00};
  if (swdio >= 0)
    p.insert(p.end(), {uint8_t(sw::kTlvAttachPins | kTagCritical), 4, 0, uint8_t(swdio), 0, uint8_t(swclk), 0});
  return p;
}
static Bytes u16(uint16_t v) { return {uint8_t(v), uint8_t(v >> 8)}; }
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
  const uint8_t *kind = tlv(out, 0, reg::core::kTlvUnavailablePayloadHolderKind, len);
  return cause && cause[0] == reg::core::kUnavailableCauseHeldBySettings && ch && (ch[0] | ch[1] << 8) == channel &&
         kind && kind[0] == reg::core::kHolderKindSettingsIdle;
}

static bool isHeld(const Result &r, const Bytes &out, uint16_t channel, uint8_t holder_kind) {
  if (r.resolution != kResolutionRejected || r.detail != kRejectUnavailable) return false;
  size_t len = 0;
  const uint8_t *cause = tlv(out, 0, reg::core::kTlvUnavailablePayloadCause, len);
  const uint8_t *ch = tlv(out, 0, reg::core::kTlvUnavailablePayloadChannel, len);
  const uint8_t *kind = tlv(out, 0, reg::core::kTlvUnavailablePayloadHolderKind, len);
  return cause && cause[0] == reg::core::kUnavailableCausePinInUse && ch && (ch[0] | ch[1] << 8) == channel && kind &&
         kind[0] == holder_kind;
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
  // arm-adi on a connection it does not know: the form first, no_connection last (core §4.3 order 8)
  {
    Bytes t = u16(uint16_t(fixed.number + 100));
    t.insert(t.end(), {1, 0, 0x16});   // req with bit 4 set: malformed
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    t.back() = 0x06;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectNoConnection);
  }

  // ---- core §4.3's order: every format error before anything unsupported (it answered unsupported first) ----
  {
    r = call(wire, WireSwd::kOpAttach, {1}, out);   // method 1 (unsupported) without max_speed (malformed)
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    // the reset TLV (not offered: an unknown critical tag) with a pins TLV shorter than its form
    Bytes a = attachRequest();
    a.insert(a.end(), {uint8_t(sw::kTlvAttachReset | kTagCritical), 4, 0, 8, 0, 10, 0,
                       uint8_t(sw::kTlvAttachPins | kTagCritical), 2, 0, 2, 0});
    r = call(wire, WireSwd::kOpAttach, a, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
    a.resize(a.size() - 5);   // the reset TLV alone: unsupported, the tag as received
    r = call(wire, WireSwd::kOpAttach, a, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out == Bytes({0x85}));
    r = call(wire, WireSwd::kOpScan, {0, 0xbf, 0, 0, uint8_t(sw::kTlvScanSkip), 1, 0, 0}, out);   // skip too short
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
  }

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
    // count x (kind swdio(u16) swclk(u16) id(u32)), 9 bytes each, no element length
    CHECK(out.size() == 2u + 9u * 6u);
    for (size_t at = 2; at + 9 <= out.size(); at += 9) CHECK(out[at + 1] != 4 && out[at + 3] != 4);
    CHECK(pins.setIdle(5, PinTable::kIdleOutputHigh));
    r = call(wire2, WireSwd::kOpAttach, attachRequest(5, 6), out);
    CHECK(isSettingsIdle(r, out, 5) && !chosen.connected);
    r = call(wire2, WireSwd::kOpAttach, attachRequest(4, 6), out);   // the pull-down idle on 4, named: accepted
    CHECK(ok(r) && chosen.connected);
    // the one seat taken: another pair is a count limit (cause 2; 0.0.28: cause 1)
    r = call(wire2, WireSwd::kOpAttach, attachRequest(6, 7), out);
    size_t clen = 0;
    const uint8_t *cause = tlv(out, 0, reg::core::kTlvUnavailablePayloadCause, clen);
    CHECK(r.detail == kRejectUnavailable && cause && cause[0] == reg::core::kUnavailableCauseLimit);
    r = call(wire2, WireSwd::kOpDetach, u16(chosen.number), out);
    CHECK(ok(r));
    // a channel a plan holds: unavailable cause 1, the channel, holder_kind 1 (core §4.3; 0.0.28: the cause alone)
    CHECK(pins.claim(7, 0x01));
    r = call(wire2, WireSwd::kOpAttach, attachRequest(6, 7), out);
    CHECK(isHeld(r, out, 7, reg::core::kHolderKindPlan) && !chosen.connected);
    const Bytes scan67 = {1, 6, 0, 7, 0};
    r = call(wire2, WireSwd::kOpScan, scan67, out);
    CHECK(isHeld(r, out, 7, reg::core::kHolderKindPlan));
    // a combination the declaration does not allow, listed after the held one: unsupported, tag 0x00 and TLV 0x40 its
    // index, before any held channel (oep-if-debug §1, core §4.3 order 6 before 7; it answered unavailable)
    r = call(wire2, WireSwd::kOpScan, {2, 6, 0, 7, 0, 7, 0, 7, 0}, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out == Bytes({0, 0x40, 1, 0, 1}));
    pins.release(0x01);
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

  // ---- wire retries (oep-if-debug §2, §5): a transfer that got nothing back, retried after the line reset ----
  {
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected);
    Bytes t = u16(fixed.number);
    t.insert(t.end(), {1, 0, 0x06});   // DP read CTRL/STAT
    g_swd.ctrl = 0xf0000000u;
    g_swd.drop_requests = 3;           // three requests get no reply (the DPIDR reads of the retries count too)
    const uint32_t resets = g_swd.line_resets;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(ok(r) && out.size() >= 10 && out[2] == kStatusOk && fixed.connected);   // 0.0.28: status line, closed
    CHECK((out[6] | out[7] << 8 | out[8] << 16 | uint32_t(out[9]) << 24) == 0xf0000000u);
    CHECK(g_swd.line_resets > resets);
    // a DP read with a bad data parity: read again after the relink
    g_swd.parity_reads = 1;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(ok(r) && out[2] == kStatusOk);
    // an AP read with a bad data parity is not repeated (the target took it): status line, one AP read, still connected
    Bytes ap = u16(fixed.number);
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
    CHECK(out[2] == kStatusLine && took >= 150000u && took <= reg::kLimitWireRetryMs * 1000u + 2000u);
    g_swd.absent = false;
    r = call(wire, WireSwd::kOpDetach, u16(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
  }

  // ---- the idle cycles (oep-if-debug §5): after a read whose data parity failed (0.0.28: none), and before a detach
  //      lets the lines go ----
  {
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected);
    Bytes ap = u16(fixed.number);
    ap.insert(ap.end(), {1, 0, 0x03});   // an AP read: a bad parity is not retried
    g_swd.parity_reads = 1;
    r = call(adi, TargetArmAdi::kOpTransfer, ap, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine && g_swd.idle_run >= 8);
    Bytes t = u16(fixed.number);
    t.insert(t.end(), {1, 0, 0x06});
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);   // answers: the rest state again
    CHECK(ok(r) && fixed.io.driven && !fixed.rest_free);
    const uint32_t before = g_swd.idle_run;
    r = call(wire, WireSwd::kOpDetach, u16(fixed.number), out);
    CHECK(ok(r) && g_swd.idle_run >= before + 8 && !fixed.io.driven);   // 0.0.28: released at once
  }

  // ---- the lines while the wire does not answer (oep-if-debug §2, §5; oep-spec 975d88c, 8d91db0): from a transfer
  //      with no answer until one answers, released between exchanges, SWDIO without its pull-up; an answer restores
  //      the rest state (0.0.28: driven throughout) ----
  {
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected && fixed.io.driven);
    Bytes t = u16(fixed.number);
    t.insert(t.end(), {1, 0, 0x06});
    g_swd.absent = true;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine && fixed.connected);
    CHECK(!fixed.io.driven && fixed.rest_free && g_swd_pull[fixed.swdio] == 0 && g_swd_pull[fixed.swclk] == 0);
    CHECK(g_swd.idle_run >= 8);   // the last exchange's idle cycles before the release
    g_swd.absent = false;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(ok(r) && fixed.io.driven && !fixed.rest_free);
    r = call(wire, WireSwd::kOpDetach, u16(fixed.number), out);
    CHECK(ok(r));
  }

  // ---- attach: the wake tried again within wire_retry_ms; search_retries counts the failed wakes ----
  {
    g_swd.drop_requests = 2;   // the first wake's two DPIDR reads (after JTAG-to-SWD, after the dormant wake)
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected);
    size_t len = 0;
    const uint8_t *v = tlv(out, 11, sw::kTlvAttachAnswerSearchRetries, len);
    CHECK(v && len == 2 && v[0] == 1 && v[1] == 0);
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);   // joining: no search, no search_retries
    CHECK(ok(r) && !tlv(out, 11, sw::kTlvAttachAnswerSearchRetries, len));
    // joining with a lower max_speed (100 kHz): the connection slowed to it and returned (oep-if-debug §1; it refused)
    const uint16_t number = fixed.number;
    Bytes slower = {0, uint8_t(sw::kTlvAttachMaxSpeed | kTagCritical), 4, 0, 0xa0, 0x86, 0x01, 0x00};
    r = call(wire, WireSwd::kOpAttach, slower, out);
    CHECK(ok(r) && out.size() >= 11 && (out[6] & sw::kAttachFlagsExisting) && fixed.number == number);
    CHECK((out[7] | out[8] << 8 | out[9] << 16 | uint32_t(out[10]) << 24) <= 100000u && fixed.io.half_ns >= 5000);
    CHECK(tlv(out, 11, sw::kTlvAttachAnswerSearchRetries, len) != nullptr);
    Bytes dp = u16(fixed.number);
    dp.insert(dp.end(), {1, 0, 0x06});
    r = call(adi, TargetArmAdi::kOpTransfer, dp, out);   // and works at that speed
    CHECK(ok(r) && out[2] == kStatusOk);
    r = call(wire, WireSwd::kOpDetach, u16(fixed.number), out);
    CHECK(ok(r));
    g_swd.absent = true;
    const uint32_t before = micros();
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    const uint32_t took = micros() - before;
    printf("  an attach with nothing there: %u ms\n", took / 1000);
    CHECK(r.resolution == kResolutionCompleted && out.size() >= 1 && out[0] == kStatusLine && !fixed.connected);
    CHECK(took >= 150000u && took <= reg::kLimitWireRetryMs * 1000u + 2000u);
    // at the slowest clock this wire takes (min_clock_hz 10 kHz): still well inside the attach budget
    Bytes slow = {0, uint8_t(sw::kTlvAttachMaxSpeed | kTagCritical), 4, 0, 0x10, 0x27, 0, 0};   // 10000 Hz
    const uint32_t before_slow = micros();
    r = call(wire, WireSwd::kOpAttach, slow, out);
    const uint32_t took_slow = micros() - before_slow;
    printf("  an attach with nothing there at 10 kHz: %u ms\n", took_slow / 1000);
    CHECK(r.resolution == kResolutionCompleted && out.size() >= 1 && out[0] == kStatusLine);
    CHECK(took_slow <= reg::kLimitAttachBudgetMs * 1000u);
    slow[4] = 0x0f;   // 9999 Hz: under min_clock_hz, unsupported
    slow[5] = 0x27;
    r = call(wire, WireSwd::kOpAttach, slow, out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported);
    g_swd.absent = false;
  }

  // ---- the target's power gone and back under a live connection: attach answers it as it is (oep-if-debug §1), the
  // DPIDR read's wire retries bring the port back (line reset, wake); a scan through the live connection runs the
  // wire-loss clock and closes it once lost (§2) ----
  for (uint32_t gone_ms : {500u, 1500u}) {
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected);
    const uint16_t number = fixed.number;
    Bytes t = u16(fixed.number);
    t.insert(t.end(), {1, 0, 0x06});
    g_swd.absent = true;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(out.size() >= 3 && out[2] == kStatusLine && fixed.connected);
    g_millis += gone_ms;
    g_swd.absent = false;
    g_swd.locked = true;   // powered up again: DPIDR first
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected && fixed.number == number && out.size() >= 11 && (out[6] & sw::kAttachFlagsExisting));
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(ok(r) && out[2] == kStatusOk);
    r = call(wire, WireSwd::kOpDetach, u16(fixed.number), out);
    CHECK(ok(r) && !fixed.connected);
  }
  {
    r = call(wire, WireSwd::kOpAttach, attachRequest(), out);
    CHECK(ok(r) && fixed.connected);
    Bytes t = u16(fixed.number);
    t.insert(t.end(), {1, 0, 0x06});
    g_swd.absent = true;
    r = call(adi, TargetArmAdi::kOpTransfer, t, out);
    CHECK(out[2] == kStatusLine && fixed.connected);
    g_millis += reg::kLimitWireLostMs;
    r = call(wire, WireSwd::kOpScan, {0}, out);
    CHECK(ok(r) && out.size() >= 2 && out[1] == 0 && !fixed.connected);   // 68d9694: kept
    g_swd.absent = false;
  }

  printf("swd: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
