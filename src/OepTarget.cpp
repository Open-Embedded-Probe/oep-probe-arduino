// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepTarget.h"

#include <Arduino.h>

#include "OepPinTable.h"
#include "OepPlatform.h"

namespace oep {
namespace {

namespace wire = reg::wire_rvswd;

constexpr uint8_t kDmStatus = 0x11;

// A cold CH32 ignores the first wake now and then (the CH32L103 answered on the fifth, 2026-09-23), and one that
// sat idle past its link timeout has dropped the link: bring the bus up afresh and try again before saying no - while
// the attach budget lasts (the PHY's deadline). Each try again counts in search_retries.
bool attachAndRead(Ch32Dm &dm, uint32_t &status) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (attempt) {
      if (dm.phy().pastDeadline()) break;
      dm.phy().countSearchRetry();
    }
    if (dm.attach() && dm.readDmi(kDmStatus, status) && status != 0 && status != 0xffffffffu) return true;
    dm.detach();
  }
  return false;
}

// The max_speed TLV (0x01, u32 Hz) of attach / scan: 0 when absent. false = malformed (not 4 bytes, or 0).
bool maxSpeed(const Tail &tail, uint8_t tag, uint32_t &hz, bool &critical) {
  hz = 0;
  critical = false;
  size_t len = 0;
  const uint8_t *v = tail.find(tag, len, &critical);
  if (!v) return true;
  if (len != 4 || getU32(v) == 0) return false;
  hz = getU32(v);
  return true;
}

// The idle_clock TLV (0x04, u8 0 high / 1 low; rvswd only) of attach / scan: absent = high (oep-if-debug §3).
// false = malformed.
bool idleClock(const Tail &tail, uint8_t tag, bool &low, bool &critical) {
  low = false;
  critical = false;
  size_t len = 0;
  const uint8_t *v = tail.find(tag, len, &critical);
  if (!v) return true;
  if (len != 1 || v[0] > wire::kIdleClockLow) return false;
  low = v[0] == wire::kIdleClockLow;
  return true;
}

// The reset line, open drain: pulled low, then released - never driven high (oep-if-debug §3) - and back to its idle
// state when the settings give it one (oep-core §8; without one it stays released, Hi-Z).
struct ResetLine {
  int channel;
  PinTable *pins;
  uint64_t *held_at_ns;   // nullptr, or where the time the pull started goes
  WireLossClock *loss;    // the connection's wire-loss clock: the hold and the wire_lost_ms after it do not count (§2)
};
void holdReset(void *ctx) {
  const ResetLine &line = *static_cast<ResetLine *>(ctx);
  if (line.held_at_ns) *line.held_at_ns = nowNs();
  platformGpio(line.channel, kGpioOpenDrainLow);
}
void releaseReset(void *ctx) {
  const ResetLine &line = *static_cast<ResetLine *>(ctx);
  platformGpio(line.channel, kGpioOpenDrainRelease);
  if (line.pins) line.pins->rest(static_cast<uint16_t>(line.channel));
  if (line.loss) line.loss->excuseReset();
}

// The halted hart's dpc for the attach answer's TLV 0x11 (DATA0 is used and put back).
bool readDpc(Ch32Dm &dm, uint32_t &dpc) {
  dm.keepMailbox();
  const bool ok = dm.readRegister(0x7b1, dpc);
  dm.giveMailbox();
  return ok;
}

}  // namespace

// ---- oep.wire.rvswd / oep.wire.swio ------------------------------------------------------------------

// The attach result's target_id (oep-if-debug §1): scheme wch_dmi_7f, the u32 at DMI 0x7F; 0 and all ones = none.
size_t targetId(DebugPort &port, uint8_t *out, size_t room) {
  uint32_t id = 0;
  port.has_tid = port.dm.readDmi(0x7f, id) && id != 0 && id != 0xffffffffu;   // kept for connections / the slots
  port.tid = port.has_tid ? id : 0;
  if (room < 7 || !port.has_tid) return 0;
  out[0] = wire::kTlvAttachAnswerTargetId;
  out[1] = 5;
  out[2] = reg::common::kTargetIdSchemeWchDmi7f;
  putU32(out + 3, id);
  return 7;
}

bool attachRunning(DebugPort &port, uint8_t user, uint32_t &dmstatus, uint32_t max_hz, bool idle_low, AttachReset *reset,
                   bool *no_answer) {
  if (no_answer) *no_answer = false;
  if (port.connected) {
    if (!port.dm.readDmi(kDmStatus, dmstatus)) { if (no_answer) *no_answer = true; return false; }
  } else {
    DmiPhy &phy = port.dm.phy();
    if (!phy.setMaxHz(max_hz)) phy.setMaxHz(0);   // the slot's settings were checked when it was set
    phy.setIdleClockLow(idle_low);
    if (max_hz && port.dm.attached() && phy.clockHz() > max_hz) port.dm.detach();
    AttachDeadline budget(phy, reset ? reset->hold_ms : 0);   // oep-if-debug §1, as a host's attach
    if (reset) {   // as attach's reset TLV with method 0: the line pulled and released, then the attach
      ResetLine line{reset->channel, port.pins, &reset->held_at_ns, &phy.loss()};
      port.dm.pulseReset(holdReset, releaseReset, &line, reset->hold_ms);
    }
    // a failed try holds nothing: the pins go free until the slot tries again
    auto fail = [&](bool silent) {
      if (no_answer) *no_answer = silent;
      port.dm.detach();
      freeWire(port);
      return false;
    };
    if (!attachAndRead(port.dm, dmstatus)) return fail(true);
    port.dm.ackHaveReset();
    if (!port.dm.readDmi(kDmStatus, dmstatus)) return fail(true);
    const uint16_t number = ResourceNumbers::take(ResourceNumbers::kConnection);
    if (!number) return fail(false);
    port.connected = true;
    port.number = number;
    port.lost = false;
    phy.loss().clear();   // a new connection: its own wire-loss clock (oep-if-debug §2)
    holdPins(port);
    uint8_t tlv[8];
    targetId(port, tlv, sizeof tlv);   // what a slot's lock is checked against
  }
  port.users |= user;
  return true;
}

// Nothing holds the wire's pins any more (a connection closed, a scan's try, a failed attach): the PHY lets them go
// to Hi-Z with no pull, then a channel with an idle set goes to it (oep-core §8: a released pin, whichever way; the
// PHY's resting - SWCLK low, SWDIO's pull-up - is only how a live link waits). The pin table's owner is cleared first.
void freeWire(DebugPort &port) {
  port.dm.phy().free();
  if (!port.pins) return;
  if (port.pin_choice && !port.connected) port.pins->releaseQuiet(port.pin_owner);
  port.pins->rest(port.swdio);
  if (port.swclk != 0xffff) port.pins->rest(port.swclk);
}

void releaseConnection(DebugPort &port, uint8_t user, bool force, bool lost) {
  port.users &= static_cast<uint8_t>(~user);
  if (!port.connected || (port.users && !force)) return;
  port.dm.detach();   // haltreq lowered, dmactive kept (oep-if-debug §4.6)
  port.connected = false;
  port.users = 0;
  port.has_tid = false;
  port.lost = lost;
  ++port.closes;
  ResourceNumbers::close(port.number);
  freeWire(port);
}

// One DMSTATUS read, with the wire retries of one request (oep-if-debug §2). A check that gets nothing back is not wire
// loss by itself: the connection closes only once the wire has failed for wire_lost_ms with no good exchange between
// (this check, the host's requests, the console's reads all run the same clock).
bool checkConnection(DebugPort &port) {
  if (!port.connected) return true;
  DmiPhy &phy = port.dm.phy();
  phy.beginRequest();
  uint32_t status = 0;
  if (port.dm.readDmi(kDmStatus, status) && status != 0 && status != 0xffffffffu) return true;
  phy.loss().silent();   // all zeros / ones: no module behind the answer
  if (!phy.loss().lost()) return true;
  releaseConnection(port, 0xff, true, true);
  return false;
}

bool pairAllowed(const DebugPort &port, uint16_t swdio, uint16_t swclk) {
  if (!port.pin_choice) return swdio == port.swdio && swclk == port.swclk;
  const bool one_wire = port.swclk == 0xffff;
  if (swdio > 63 || !((port.pin_choice >> swdio) & 1)) return false;
  if (one_wire) return swclk == 0xffff;
  return swclk <= 63 && swclk != swdio && ((port.pin_choice >> swclk) & 1);
}

// A channel of the pair the settings disable (probe.config §1), or 0xFFFF.
uint16_t pairDisabled(const DebugPort &port, uint16_t swdio, uint16_t swclk) {
  if (!port.pins) return 0xffff;
  if (port.pins->disabled(swdio)) return swdio;
  if (swclk != 0xffff && port.pins->disabled(swclk)) return swclk;
  return 0xffff;
}

// A channel of the pair that has an idle item in the settings (probe.config §1), or 0xFFFF. outputs: only an output
// idle (mode 3 / 4). oep-if-debug §1: count = 0 and an attach without pins leave out every channel with an idle item;
// a request naming one is refused (cause 5, holder_kind 7) only when the idle is an output.
uint16_t pairIdle(const DebugPort &port, uint16_t swdio, uint16_t swclk, bool outputs) {
  if (!port.pins) return 0xffff;
  auto has = [&](uint16_t c) {
    if (c == 0xffff) return false;
    const uint8_t mode = port.pins->idle(c);
    return outputs ? mode == PinTable::kIdleOutputLow || mode == PinTable::kIdleOutputHigh : mode != PinTable::kIdleUnset;
  };
  return has(swdio) ? swdio : has(swclk) ? swclk : 0xffff;
}

bool pairFree(const DebugPort &port, uint16_t swdio, uint16_t swclk) {
  if (pairDisabled(port, swdio, swclk) != 0xffff) return false;   // never used, the fixed pair included
  if (!port.pin_choice || !port.pins) return true;   // a fixed pair is the wire's own (kept out of the pin table)
  auto freeFor = [&](uint16_t c) {
    if (c == 0xffff) return true;
    const uint8_t owner = port.pins->owner(c);
    return owner == 0 || (owner == port.pin_owner && port.connected && swdio == port.swdio && swclk == port.swclk);
  };
  return freeFor(swdio) && freeFor(swclk);
}

bool usePair(DebugPort &port, uint16_t swdio, uint16_t swclk) {
  if (!port.connected && pairIdle(port, swdio, swclk, true) != 0xffff) return false;   // an output idle: never driven
  if (swdio == port.swdio && swclk == port.swclk) return true;
  if (!port.pin_choice || port.connected || !pairAllowed(port, swdio, swclk) || !pairFree(port, swdio, swclk)) return false;
  if (port.dm.attached()) port.dm.detach();
  if (!port.dm.phy().usePins(swdio, swclk == 0xffff ? -1 : swclk)) return false;
  port.swdio = swdio;
  port.swclk = swclk;
  return true;
}

void holdPins(DebugPort &port) {
  if (!port.pin_choice || !port.pins || !port.connected) return;
  port.pins->releaseQuiet(port.pin_owner);
  port.pins->claim(port.swdio, port.pin_owner);
  if (port.swclk != 0xffff) port.pins->claim(port.swclk, port.pin_owner);
}

// connections (oep-if-debug §2.1): first(u8) -> more(u8) count(u8), per entry len(u8) then connection(u16) swdio(u16)
// swclk(u16) speed_hz(u32) users(u8) slot(u8) tid_scheme(u8) tid_len(u8) tid. One place per wire here: at most one entry.
Result connectionsOf(DebugPort &port, uint8_t first, uint32_t speed_hz, uint8_t *out, size_t capacity) {
  if (capacity < 3 + 20) return failed();
  out[0] = 0;   // more: never (one entry at most)
  out[1] = 0;
  if (!port.connected || first > 0) return completed(2);
  out[1] = 1;
  uint8_t *e = out + 3;   // after the entry's len(u8) (core §2.3)
  putU16(e, port.number);
  putU16(e + 2, port.swdio);
  putU16(e + 4, port.swclk);
  putU32(e + 6, speed_hz);
  e[10] = port.users;
  e[11] = port.slot;
  e[12] = port.has_tid ? reg::common::kTargetIdSchemeWchDmi7f : 0;
  e[13] = port.has_tid ? reg::common::kTargetIdLenWchDmi7f : 0;
  if (port.has_tid) putU32(e + 14, port.tid);
  out[2] = port.has_tid ? 18 : 14;
  return completed(3u + out[2]);
}

size_t WireRvswd::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  if (port_.pin_choice) {                     // any free pair of these (oep-if-debug §1)
    static const uint8_t kPair[] = {wire::kPinRoleSwdio, wire::kPinRoleSwclk};
    w.roleChannels(kPair, port_.swclk == 0xffff ? 1 : 2, port_.pin_choice);
  } else {
    w.pinGroup(port_.swdio, port_.swclk);     // fixed on this probe
  }
  static const uint8_t kReset[] = {wire::kPinRoleReset};
  if (port_.reset_allowed) w.roleChannels(kReset, 1, port_.reset_allowed);   // attach's reset TLV: its channels, no default
  DmiPhy &phy = port_.dm.phy();
  if (phy.minClockHz()) w.u32(kTagMinClockHz, phy.minClockHz());   // a max_speed under it is unsupported
  w.u8(kTagImplementation, 1);                // bit-bang
  return w.ok() ? w.length() : 0;
}

// The attach pins TLV (oep-if-debug §1): the pair to attach on, the link moved there. Absent: the live connection's pair
// (join it), else the fixed pair, else - pins the host chooses, nothing live - refused (the host names one). Another pair
// than the live connection's takes its seat only when a slot alone uses it (the seat rule); else refused. 0, or a
// reject reason.
uint8_t WireRvswd::choosePair(const uint8_t *pins, size_t len) {
  if (!pins) return port_.pin_choice && !port_.connected ? kRejectUnavailable : 0;
  if (len != 4) return kRejectMalformed;
  const uint16_t d = getU16(pins), c = getU16(pins + 2);
  if (!pairAllowed(port_, d, c)) return kRejectUnsupported;   // not a pair this wire declares (core §4.3 order 6)
  if (d == port_.swdio && c == port_.swclk) return 0;          // the pair the link is on (live or not)
  if (!port_.pins) return kRejectUnavailable;
  const uint16_t chs[2] = {d, c};
  for (uint16_t ch : chs) {                                    // held by anything but this wire's own connection
    if (ch == 0xffff) continue;
    const uint8_t owner = port_.pins->owner(ch);
    if (owner != 0 && owner != port_.pin_owner) return kRejectUnavailable;
  }
  if (port_.connected) {
    if (port_.users != DebugPort::kUserSlot) return kRejectUnavailable;   // no seat: the host's connection is on it
    releaseConnection(port_, DebugPort::kUserSlot, true);     // the seat rule: a slot-only connection makes room
  }
  return usePair(port_, d, c) ? 0 : kRejectUnavailable;
}

// count(u8) pairs [TLV 0x01 max_speed, 0x02 skip (count 0), 0x04 idle_clock (rvswd)]
//   ->  tried(u8) count(u8), then kind(u8) swdio(u16) swclk(u16) raw DMSTATUS(u32) per answer, each after its len(u8)
// (oep-if-debug §1, §3). Only dmactive is written; a pair without a module goes back to Hi-Z.
Result WireRvswd::scan(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 1 || length < 1u + 4u * payload[0]) return rejected(kRejectMalformed);
  const uint8_t count = payload[0];
  const size_t fixed = 1u + 4u * count;
  static const uint8_t kScanTags[] = {wire::kTlvScanMaxSpeed, wire::kTlvScanSkip, wire::kTlvScanIdleClock};
  Tail tail;
  const Result parsed = tail.parse(payload + fixed, length - fixed, kScanTags, isRvswd() ? 3 : 2, out, capacity);
  if (refused(parsed)) return parsed;
  DmiPhy &phy = port_.dm.phy();
  const bool one_wire = port_.swclk == 0xffff;
  for (uint8_t k = 0; k < count && one_wire; ++k)
    if (getU16(payload + 3 + 4 * k) != 0xffff) return rejected(kRejectMalformed);   // swio: swclk is always 0xFFFF
  uint16_t skip = 0;
  {
    size_t len = 0;
    if (const uint8_t *v = tail.find(wire::kTlvScanSkip, len)) {
      if (count || len != 2) return rejected(kRejectMalformed);   // skip goes with count 0 only
      skip = getU16(v);
    }
  }
  uint32_t max_hz = 0;
  bool critical = false, idle_low = false, idle_critical = false;
  if (!maxSpeed(tail, wire::kTlvScanMaxSpeed, max_hz, critical)) return rejected(kRejectMalformed);
  if (isRvswd() && !idleClock(tail, wire::kTlvScanIdleClock, idle_low, idle_critical)) return rejected(kRejectMalformed);
  if (max_hz && phy.minClockHz() && max_hz < phy.minClockHz()) return unsupportedTag(out, capacity, wire::kTlvScanMaxSpeed | (critical ? kTagCritical : 0));
  // every pair listed: one this wire allows, whose pins nothing else holds, and - the one seat taken - the live one
  for (uint8_t k = 0; k < count; ++k) {
    const uint16_t d = getU16(payload + 1 + 4 * k), c = getU16(payload + 3 + 4 * k);
    if (!pairAllowed(port_, d, c)) return unsupportedValue(out, capacity);
    const uint16_t off = pairDisabled(port_, d, c);   // the settings disable it: cause 5 (probe.config §1)
    if (off != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, off, 0xFFFF,
                                           reg::core::kHolderKindDisabled);
    const uint16_t idle = pairIdle(port_, d, c, true);   // an output idle: cause 5, holder_kind 7 (debug §1)
    if (idle != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, idle, 0xFFFF,
                                            reg::core::kHolderKindSettingsIdle);
    if (!pairFree(port_, d, c) || (port_.connected && (d != port_.swdio || c != port_.swclk)))
      return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, port_.pins && port_.pins->owner(d) ? d : c);
  }
  if (capacity < 2) return failed();
  if (!port_.connected) {   // the scan's line settings (a connection keeps its own)
    if (!phy.setMaxHz(max_hz)) phy.setMaxHz(0);
    phy.setIdleClockLow(idle_low);
    if (max_hz && port_.dm.attached() && phy.clockHz() > max_hz) port_.dm.detach();
  }
  size_t at = 2;
  uint8_t tried = 0, found = 0;
  // No pair starts later than scan_budget_ms after the request (oep-if-debug §1): the host goes on with the rest (count 0
  // with skip, or the pairs after tried). 26 free pins bit-banged pair by pair kept an RP2350 from answering for seconds
  // (0.0.18). At least one pair is tried while any is left (tried 0 means the list is used up); one try is bounded by
  // the attach budget.
  const uint32_t began = millis();
  auto tryPair = [&](uint16_t d, uint16_t c) {   // false: the answer is full, or its time is up
    if (at + 10 > capacity || (tried && millis() - began >= reg::kLimitScanBudgetMs)) return false;
    uint32_t status = 0;
    bool ok = false;
    if (port_.connected) {
      // the live connection: look through it - re-attaching (and detaching on a miss) would pull the link out
      // from under the host that holds it
      ok = port_.dm.readDmi(kDmStatus, status) && status != 0 && status != 0xffffffffu;
    } else if (usePair(port_, d, c)) {
      {
        AttachDeadline budget(phy);
        ok = port_.dm.probe(status);                     // wake / configuration and dmactive only (debug §1)
      }
      port_.dm.detach();                                 // nothing held: dmactive stays
      freeWire(port_);                                   // found or not, the pair to its free state (debug §1)
    }
    if (ok) {
      out[at] = 9;   // the element's length (core §2.3)
      out[at + 1] = wire::kScanKindRiscvDm;
      putU16(out + at + 2, d);
      putU16(out + at + 4, c);
      putU32(out + at + 6, status);
      at += 10;
      ++found;
    }
    ++tried;
    return true;
  };
  if (count) {
    for (uint8_t k = 0; k < count && tryPair(getU16(payload + 1 + 4 * k), getU16(payload + 3 + 4 * k)); ) ++k;
  } else if (port_.connected || !port_.pin_choice) {   // the count-0 list: the live pair, or the fixed one
    // a fixed pair with an idle item on a channel is not in the list (debug §1); the live pair is
    if (skip == 0 && pairDisabled(port_, port_.swdio, port_.swclk) == 0xffff &&
        (port_.connected || pairIdle(port_, port_.swdio, port_.swclk, false) == 0xffff))
      tryPair(port_.swdio, port_.swclk);
  } else {                                            // swdio ascending, then swclk; held pairs and idle items left out
    uint32_t index = 0;
    bool more = true;
    auto consider = [&](uint16_t d, uint16_t c) {
      if (more && tried < 255 && pairAllowed(port_, d, c) && pairFree(port_, d, c) &&
          pairIdle(port_, d, c, false) == 0xffff && index++ >= skip)
        more = tryPair(d, c);
    };
    for (uint16_t d = 0; d < 64; ++d) {
      if (one_wire) consider(d, 0xffff);
      else for (uint16_t c = 0; c < 64; ++c) consider(d, c);
    }
  }
  out[0] = tried;
  out[1] = found;
  return tail.finish(completed(at), out, capacity);
}

// method(u8: 0 leave it running, 1 halt) [TLV 0x01 max_speed (required), 0x03 pins, 0x04 idle_clock, 0x05 reset]
//   ->  connection(u16) DMSTATUS(u32, after the attach: halted when it halted) flags(u8: bit0 acknowledged a pending
//       havereset, bit1 existing, bit3 halted) speed_hz(u32) [TLV 0x10 target_id, 0x11 dpc while halted, 0x12
//       search_retries when a speed search ran]
// A failed attach answers status(u8) alone: line (no answer), timeout (the hart did not stop / start).
Result WireRvswd::attach(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // idle_clock is rvswd's (oep-if-debug §3): on swio it is an unknown tag (critical: rejected unsupported)
  static const uint8_t kAttachTags[] = {wire::kTlvAttachMaxSpeed, wire::kTlvAttachPins, wire::kTlvAttachReset,
                                        wire::kTlvAttachIdleClock};
  if (length < 1) return rejected(kRejectMalformed);
  Tail tail;
  const Result parsed = tail.parse(payload + 1, length - 1, kAttachTags, isRvswd() ? 4 : 3, out, capacity);
  if (refused(parsed)) return parsed;
  // not a method of the table: unsupported, payload 0x00 (oep-if-debug §3; a later revision may define it, core §2.5)
  if (payload[0] > wire::kAttachMethodHalt) return unsupportedValue(out, capacity);
  const bool halt = payload[0] == wire::kAttachMethodHalt;
  DmiPhy &phy = port_.dm.phy();
  uint32_t max_hz = 0;
  bool critical = false, idle_low = false, idle_critical = false;
  if (!maxSpeed(tail, wire::kTlvAttachMaxSpeed, max_hz, critical) ||
      (isRvswd() && !idleClock(tail, wire::kTlvAttachIdleClock, idle_low, idle_critical)))
    return rejected(kRejectMalformed);
  if (!max_hz) return rejected(kRejectMalformed);   // max_speed is required (oep-if-debug §1)
  // reset(channel u16, hold_ms u16): the line to hold first; one this probe allows (unsupported otherwise) and nobody
  // holds (unavailable), held at most max_op_ms (core §7.5)
  bool with_reset = false;
  int reset_channel = -1;
  uint16_t hold_ms = 0;
  {
    size_t len = 0;
    bool reset_critical = false;
    if (const uint8_t *v = tail.find(wire::kTlvAttachReset, len, &reset_critical)) {
      if (len != 4) return rejected(kRejectMalformed);
      const uint8_t raw = wire::kTlvAttachReset | (reset_critical ? kTagCritical : 0);
      reset_channel = getU16(v);
      hold_ms = getU16(v + 2);
      // a value it cannot take: unsupported with the tag as received when critical, else ignored (core §2.3)
      if (reset_channel > 63 || !((port_.reset_allowed >> reset_channel) & 1) || hold_ms > kMaxOpMs) {
        if (reset_critical) return unsupportedTag(out, capacity, raw);
        tail.ignore(wire::kTlvAttachReset);
      } else {
        with_reset = true;
      }
    }
  }
  if (phy.minClockHz() && max_hz < phy.minClockHz())
    return unsupportedTag(out, capacity, wire::kTlvAttachMaxSpeed | (critical ? kTagCritical : 0));
  {
    size_t plen = 0;
    bool pins_critical = false;
    const uint8_t *pins = tail.find(wire::kTlvAttachPins, plen, &pins_critical);
    // a pair this wire does not declare: unsupported with the tag as received when critical, else ignored (core §2.3)
    if (pins && plen == 4 && !pairAllowed(port_, getU16(pins), getU16(pins + 2)) && !pins_critical) {
      tail.ignore(wire::kTlvAttachPins);
      pins = nullptr;
    }
    // a channel the settings disable - the pins asked for, the fixed pair, the reset line: cause 5 with the channel
    // (probe.config §1), before choosePair moves anything
    uint16_t off = 0xffff;
    if (pins && plen == 4 && pairAllowed(port_, getU16(pins), getU16(pins + 2)))
      off = pairDisabled(port_, getU16(pins), getU16(pins + 2));
    else if (!pins && !port_.pin_choice)
      off = pairDisabled(port_, port_.swdio, port_.swclk);
    if (off == 0xffff && with_reset && port_.pins && port_.pins->disabled(static_cast<uint16_t>(reset_channel)))
      off = static_cast<uint16_t>(reset_channel);
    if (off != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, off, 0xFFFF,
                                           reg::core::kHolderKindDisabled);
    // an idle item (debug §1): pins naming a channel whose idle is an output, or - no pins, no live connection - the
    // fixed pair with any idle item (the candidates leave it out, so none is left): cause 5, holder_kind 7
    uint16_t idle = 0xffff;
    if (pins && plen == 4 && pairAllowed(port_, getU16(pins), getU16(pins + 2)))
      idle = pairIdle(port_, getU16(pins), getU16(pins + 2), true);
    else if (!pins && !port_.pin_choice && !port_.connected)
      idle = pairIdle(port_, port_.swdio, port_.swclk, false);
    if (idle != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, idle, 0xFFFF,
                                            reg::core::kHolderKindSettingsIdle);
    if (const uint8_t bad = choosePair(pins, plen)) {
      if (bad == kRejectUnsupported) return unsupportedTag(out, capacity, wire::kTlvAttachPins | kTagCritical);   // critical, above
      if (bad == kRejectUnavailable) return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse);
      return rejected(bad);
    }
  }
  if (with_reset && port_.pins && port_.pins->owner(static_cast<uint16_t>(reset_channel)))
    return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, static_cast<uint16_t>(reset_channel));
  if (!phy.setIdleClockLow(idle_low)) {   // an existing connection takes the new rest level too
    const Result r = tail.refuse(wire::kTlvAttachIdleClock, idle_critical, out, capacity);
    if (refused(r)) return r;
    phy.setIdleClockLow(false);
  }
  if (capacity < 11) return failed();
  ResetLine reset_line{reset_channel, port_.pins, nullptr, &phy.loss()};
  uint32_t status = 0, dpc = 0;
  uint8_t flags = 0;
  uint8_t failure = kStatusOk;
  bool have_dpc = false;
  // The speed search and its retries take at most the attach budget, a reset's hold_ms aside (oep-if-debug §1). What
  // failed on the way is the answer's search_retries.
  AttachDeadline budget(phy, with_reset ? hold_ms : 0);
  phy.clearSearchRetries();
  bool searched = !port_.connected || with_reset;   // a search ran (search_retries goes in the answer)
  if (port_.connected) {
    // Already attached: the same connection, nothing redone (a one-command-per-process host gets its link back).
    // A running link over the new ceiling is slowed to it (going slower is safe); only a link that cannot keep
    // the ceiling refuses it.
    if (phy.clockHz() > max_hz) {
      searched = true;
      if (!(phy.setMaxHz(max_hz) && phy.retune() && phy.clockHz() <= max_hz)) {
        const Result r = tail.refuse(wire::kTlvAttachMaxSpeed, critical, out, capacity);
        if (refused(r)) return r;
      }
    }
    flags |= wire::kAttachFlagsExisting;
    if (with_reset) {
      // the reset op's NRST on this connection (mark reset 3), then stopped at the vector or left running
      if (halt) {
        if (!port_.dm.attachUnderReset(holdReset, releaseReset, &reset_line, hold_ms, dpc)) failure = kStatusTimeout;
        else have_dpc = true;
      } else {
        port_.dm.pulseReset(holdReset, releaseReset, &reset_line, hold_ms);
        if (!attachAndRead(port_.dm, status)) failure = kStatusLine;
        else if (port_.dm.ackHaveReset()) flags |= wire::kAttachFlagsHaveresetAcked;
      }
      if (failure == kStatusOk) { ++port_.resets; port_.reset_detail = reg::common::kMarkDetailResetAttachReset; }
    }
    if (failure == kStatusOk) {
      // halt() is idempotent and also brings this driver's own halted state in line with the hart: a hart left
      // halted (by an earlier process or a reset-halt) must count as halted here, or block reads refuse it
      if (!port_.dm.readDmi(kDmStatus, status)) failure = kStatusLine;
      else if ((halt || (status & (1u << 9))) && !(port_.dm.halt() && port_.dm.readDmi(kDmStatus, status)))
        failure = kStatusTimeout;
    }
    // nothing back: status line; the connection closes after the answer only once the wire is lost (oep-if-debug §2)
    if (failure == kStatusLine && phy.loss().lost()) releaseConnection(port_, 0xff, true, true);
  } else {
    if (!phy.setMaxHz(max_hz)) {   // a ceiling this link cannot keep (a fixed speed above it)
      const Result r = tail.refuse(wire::kTlvAttachMaxSpeed, critical, out, capacity);
      if (refused(r)) return r;
      phy.setMaxHz(0);
    }
    // A scan may have left the link up at a speed over the new ceiling: search again under it.
    if (port_.dm.attached() && phy.clockHz() > max_hz) port_.dm.detach();
    if (with_reset && halt) {
      if (!port_.dm.attachUnderReset(holdReset, releaseReset, &reset_line, hold_ms, dpc)) {
        // the module answers but the hart never stopped: timeout; no answer: line
        const bool answers = port_.dm.readDmi(kDmStatus, status) && status != 0 && status != 0xffffffffu;
        failure = answers ? kStatusTimeout : kStatusLine;
      } else {
        have_dpc = true;
        if (!port_.dm.readDmi(kDmStatus, status)) failure = kStatusLine;
      }
    } else {
      if (with_reset) port_.dm.pulseReset(holdReset, releaseReset, &reset_line, hold_ms);
      if (!attachAndRead(port_.dm, status)) {
        failure = kStatusLine;
      } else {
        // A pending havereset freezes a V00x's DMSTATUS halt / run bits at their reset values (ch32rv 0.8.0):
        // acknowledge it first so the DMSTATUS returned is current, and say that it was there (flags bit0).
        if (port_.dm.ackHaveReset()) flags |= wire::kAttachFlagsHaveresetAcked;
        if (!port_.dm.readDmi(kDmStatus, status)) failure = kStatusLine;
        else if (halt && !port_.dm.halt()) failure = kStatusTimeout;
        else if (halt && !port_.dm.readDmi(kDmStatus, status)) failure = kStatusLine;   // the answer: DMSTATUS after the halt
      }
    }
    if (failure == kStatusOk) {
      const uint16_t number = ResourceNumbers::take(ResourceNumbers::kConnection);
      if (!number) {
        port_.dm.detach();
        freeWire(port_);
        return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
      }
      port_.connected = true;
      port_.number = number;
      port_.lost = false;
      phy.loss().clear();   // a new connection: its own wire-loss clock (oep-if-debug §2)
      holdPins(port_);
      if (with_reset) { ++port_.resets; port_.reset_detail = reg::common::kMarkDetailResetAttachReset; }
    } else {
      port_.dm.detach();   // a failed attach consumes no number and holds no pins: they go free
      freeWire(port_);
    }
  }
  if (failure != kStatusOk) return tail.finish(failedStatus(failure, out, capacity), out, capacity);
  port_.users |= DebugPort::kUserHost;
  if (port_.dm.halted()) {
    flags |= wire::kAttachFlagsHalted;
    if (!have_dpc) have_dpc = readDpc(port_.dm, dpc);
  }
  putU16(out, port_.number);
  putU32(out + 2, status);
  out[6] = flags;
  putU32(out + 7, phy.clockHz());
  size_t n = 11 + targetId(port_, out + 11, capacity - 11);
  if (have_dpc && (flags & wire::kAttachFlagsHalted) && n + 6 <= capacity) {
    out[n] = wire::kTlvAttachAnswerDpc;
    out[n + 1] = 4;
    putU32(out + n + 2, dpc);
    n += 6;
  }
  if (searched && n + 4 <= capacity) {   // search_retries (oep-if-debug §1): u16, 0xFFFF = that many or more
    const uint32_t retries = phy.searchRetries();
    out[n] = wire::kTlvAttachAnswerSearchRetries;
    out[n + 1] = 2;
    putU16(out + n + 2, static_cast<uint16_t>(retries < 0xffff ? retries : 0xffff));
    n += 4;
  }
  return tail.finish(completed(n), out, capacity);
}

Result WireRvswd::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  port_.dm.phy().beginRequest();   // the request's allowance for wire retries (oep-if-debug §2)
  switch (op) {
    case kOpScan: return scan(payload, length, out, capacity);
    case kOpAttach: return attach(payload, length, out, capacity);
    case kOpConnections: {   // first(u8) [TLV] -> more count entries: the live connections from the first-th (no lock)
      const Result parsed = plainTail(tail, payload, length, 1, out, capacity);
      if (refused(parsed)) return parsed;
      return tail.finish(connectionsOf(port_, payload[0], port_.dm.phy().clockHz(), out, capacity), out, capacity);
    }
    case kOpDetach: {   // connection(u16) [TLV 0x01 force]
      // The host's use goes; the link stays while another user (a bind's console) has it, unless forced.
      static const uint8_t kDetachTags[] = {wire::kTlvDetachForce};
      if (length < 2) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 2, length - 2, kDetachTags, out, capacity);
      if (refused(parsed)) return parsed;
      if (!port_.connected || getU16(payload) != port_.number)
        return ResourceNumbers::refuse(getU16(payload), ResourceNumbers::kConnection, out, capacity);
      size_t len = 0;
      const bool force = tail.find(wire::kTlvDetachForce, len) != nullptr;
      releaseConnection(port_, DebugPort::kUserHost, force);
      return tail.finish(completed(), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

// ---- oep.target.riscv-dm -------------------------------------------------------------------------

size_t TargetRiscvDm::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.u32(kTagFeatures, reg::target_riscv_dm::kFeaturesBlock | reg::target_riscv_dm::kFeaturesRun |
                          reg::target_riscv_dm::kFeaturesReset | reg::target_riscv_dm::kFeaturesStep);
  // block read / write use a0, a1, s0, s1 and put them back before answering (oep-if-debug §4.5)
  w.u8(kTagImplementation, 1);
  // One block operation's data in bytes (oep-if-debug §4.5): what fits the endpoint's frame - write_block's request
  // and read_block's answer both, max_frame - 24 - and the word buffer. The host takes count from this, not from max_frame.
  w.u16(kTagMaxLength, maxLength());
  return w.ok() ? w.length() : 0;
}

// The status of an op that did not go through: `otherwise` when the module still answers, line when it does not
// (oep-if-debug §2). A request that gets nothing back within its wire_retry_ms answers status line and keeps the
// connection; the connection is closed once the answer is out only when the wire has now failed for wire_lost_ms of
// real time with no good exchange between (the wire-loss clock, shared with the console's reads).
uint8_t TargetRiscvDm::failure(uint8_t otherwise) {
  uint32_t status = 0;
  WireLossClock &loss = port_->dm.phy().loss();
  if (port_->dm.readDmi(kDmStatus, status) && status != 0 && status != 0xffffffffu) return otherwise;
  loss.silent();   // all zeros / ones: no module behind the answer
  if (loss.lost()) line_lost_ = true;
  return kStatusLine;
}

Result TargetRiscvDm::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 2) return rejected(kRejectMalformed);
  // The wire whose live connection the request names (one riscv-dm serves every wire's connections).
  port_ = nullptr;
  for (DebugPort *port : ports_)
    if (port && port->connected && getU16(payload) == port->number) port_ = port;
  if (!port_) return ResourceNumbers::refuse(getU16(payload), ResourceNumbers::kConnection, out, capacity);
  line_lost_ = false;
  port_->dm.phy().beginRequest();   // the request's allowance for wire retries (oep-if-debug §2)
  const Result r = dispatch(op, payload + 2, length - 2, out, capacity);
  if (line_lost_) releaseConnection(*port_, 0xff, true, true);   // the answer says line; the connection goes with it
  return r;
}

Result TargetRiscvDm::dispatch(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  Ch32Dm &dm = port_->dm;
  Tail tail;
  switch (op) {
    case kOpDmi: return dmi(p, n, out, capacity);
    case kOpHalt:     // [TLV] -> status(u8); already halted = ok, nothing done
    case kOpResume: { // [TLV] -> status(u8); ok = the hart left debug mode once (re-issue rules in Ch32Dm::resume)
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 1) return failed();
      const bool ok = op == kOpHalt ? dm.halt() : dm.resume();
      out[0] = ok ? kStatusOk : failure(op == kOpHalt ? kStatusTimeout : kStatusState);
      return tail.finish(outcome(out[0], 0, 1), out, capacity);
    }
    case kOpReset: {
      // mode(u8: 0 run, 1 run + confirm, 2 stop before the first instruction) [TLV 0x01 method]
      //   ->  status(u8) flags(u8) attempts(u8) pc(u32) (mode 2: dpc)
      static const uint8_t kKnown[] = {reg::target_riscv_dm::kTlvResetMethod};
      if (n < 1) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(p + 1, n - 1, kKnown, out, capacity);
      if (refused(parsed)) return parsed;
      if (p[0] > kResetHalt) return unsupportedValue(out, capacity);   // mode 3 or more (oep-if-debug §4.3, core §2.5)
      size_t len = 0;
      bool critical = false;
      if (const uint8_t *method = tail.find(reg::target_riscv_dm::kTlvResetMethod, len, &critical)) {
        if (len != 1) return rejected(kRejectMalformed);
        // Every reset here is ndmreset with haltreq held; a target system reset (PFIC) is the host's to write.
        if (method[0] != reg::target_riscv_dm::kResetMethodProbeDefault &&
            method[0] != reg::target_riscv_dm::kResetMethodNdmreset) {
          const Result r = tail.refuse(reg::target_riscv_dm::kTlvResetMethod, critical, out, capacity);
          if (refused(r)) return r;
        }
      }
      if (capacity < 7) return failed();
      bool ok;
      if (p[0] == kResetHalt) {   // flags bit0 = halted, pc = dpc; haltreq lowered afterwards, the hart stays halted
        uint32_t dpc = 0;
        ok = dm.resetHalt(dpc);
        dm.lowerHaltreq();
        out[1] = ok ? reg::target_riscv_dm::kResetFlagsReached : 0;
        out[2] = 1;
        putU32(out + 3, dpc);
      } else {
        const Ch32Dm::ResetReport r = dm.reset(p[0] == kResetRunConfirm);
        out[1] = r.flags;
        out[2] = r.attempts;
        putU32(out + 3, r.pc);
        ok = p[0] == kResetRunConfirm ? (r.flags & reg::target_riscv_dm::kResetFlagsVerified) != 0
                                      : (r.flags & reg::target_riscv_dm::kResetFlagsReached) != 0;
      }
      dm.phy().loss().excuseReset();   // the reset asserted: not counted towards wire loss (oep-if-debug §2)
      ++port_->resets;   // the console marks it (detail 1 ndmreset); last-reset binds follow it
      port_->reset_detail = reg::common::kMarkDetailResetNdmreset;
      out[0] = ok ? kStatusOk : failure(kStatusTimeout);
      return tail.finish(outcome(out[0], 0, 7), out, capacity);
    }
    case kOpStep: {   // [TLV] -> status(u8) moved(u8) dpc_before(u32) dpc_after(u32); one resume only, prv kept
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 10) return failed();
      uint32_t before = 0, after = 0;
      bool moved = false;
      uint8_t status = kStatusState;   // not halted: nothing to step
      if (dm.checkHalted() && dm.halted()) {
        const bool ok = dm.step(before, after, moved);
        // an unmoved dpc is not a failure: a self jump (j .) truly steps to itself; the host reads the instruction
        status = !ok ? (dm.lastCmderr() ? kStatusFault : failure(kStatusTimeout)) : kStatusOk;
      } else if (!dm.attached()) {
        status = failure(kStatusState);
      }
      out[0] = status;
      out[1] = moved;
      putU32(out + 2, before);
      putU32(out + 6, after);
      return tail.finish(outcome(status, 0, 10), out, capacity);
    }
    case kOpReadBlock: {   // address(u32) count(u16) [TLV]  ->  done(u16) status(u8) words (done of them) [TLV]
      const Result parsed = plainTail(tail, p, n, 6, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      if (address & 3) return rejected(kRejectMalformed);
      // count x 4 over the declared max_length: unsupported, payload 0x00 (oep-if-debug §4.5); the answer's room too
      if (!blockCountFits(count, maxLength()) || 3u + 4u * count > capacity) return unsupportedValue(out, capacity);
      if (capacity < 3) return failed();
      uint8_t status = kStatusState;
      uint16_t done = 0;
      if (count == 0) {
        status = kStatusOk;   // nothing to read: success, done 0 (oep-if-debug §4.5)
      } else if (dm.checkHalted() && dm.halted()) {
        uint8_t cmderr = 0;
        if (dm.readWords(address, words_, count, &cmderr)) {
          status = kStatusOk;
          done = count;
        } else {
          status = cmderr && cmderr != 3 ? kStatusFault : failure(kStatusFault);
        }
      } else if (!dm.attached()) {
        status = failure(kStatusState);
      }
      putU16(out, done);
      out[2] = status;
      for (uint16_t i = 0; i < done; ++i) putU32(out + 3 + 4 * i, words_[i]);
      return tail.finish(outcome(status, done, 3u + 4u * done), out, capacity);
    }
    case kOpWriteBlock: {   // address(u32) count(u16) count x word [TLV]  ->  done(u16) status(u8) [TLV]
      if (n < 6) return rejected(kRejectMalformed);
      const uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      const Result parsed = plainTail(tail, p, n, 6u + 4u * count, out, capacity);
      if (refused(parsed)) return parsed;
      if (address & 3) return rejected(kRejectMalformed);
      if (!blockCountFits(count, maxLength())) return unsupportedValue(out, capacity);   // over max_length (oep-if-debug §4.5)
      if (capacity < 3) return failed();
      uint8_t status = kStatusState;
      uint16_t done = 0;
      if (count == 0) {
        status = kStatusOk;   // nothing to write: success, done 0
      } else if (dm.checkHalted() && dm.halted()) {
        for (size_t i = 0; i < count; ++i) words_[i] = getU32(p + 6 + 4 * i);
        if (dm.writeWordsFast(address, words_, count)) {
          status = kStatusOk;
          done = count;
        } else {
          status = dm.lastCmderr() ? kStatusFault : failure(kStatusFault);
        }
      } else if (!dm.attached()) {
        status = failure(kStatusState);
      }
      putU16(out, done);
      out[2] = status;
      return tail.finish(outcome(status, done, 3), out, capacity);
    }
    case kOpRun: {
      // pc(u32) timeout_ms(u32: 1..max_op_ms) n(u8) n x (regno u16, value u32) n_out(u8) n_out x regno(u16) [TLV]
      //   ->  status(u8) stopped(u8: 0 timed out and halted, 1 stopped by itself, 2 could not be halted) dpc(u32)
      //       elapsed_us(u32) nvals(u8) nvals x value(u32) [TLV]
      if (n < 9) return rejected(kRejectMalformed);
      const uint8_t regs = p[8];
      const size_t outs_at = 9u + 6u * regs;
      if (n < outs_at + 1) return rejected(kRejectMalformed);
      const uint8_t outs = p[outs_at];
      const Result parsed = plainTail(tail, p, n, outs_at + 1 + 2u * outs, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t timeout_ms = getU32(p + 4);
      if (timeout_ms == 0 || 11u + 4u * outs > capacity) return rejected(kRejectMalformed);
      if (timeout_ms > kMaxOpMs || regs > kMaxRegs || outs > kMaxRegs) return unsupportedValue(out, capacity);
      uint16_t regnos[kMaxRegs], out_regnos[kMaxRegs];
      uint32_t values[kMaxRegs], out_values[kMaxRegs];
      for (uint8_t i = 0; i < regs; ++i) {
        regnos[i] = getU16(p + 9 + 6 * i);
        values[i] = getU32(p + 11 + 6 * i);
      }
      for (uint8_t i = 0; i < outs; ++i) out_regnos[i] = getU16(p + outs_at + 1 + 2 * i);
      memset(out, 0, 11u);
      uint8_t status = kStatusState;
      uint8_t nvals = 0;
      if (dm.checkHalted() && dm.halted()) {
        Ch32Dm::RunReport r;
        const bool ok = dm.runUntilHalt(getU32(p), regnos, values, regs, timeout_ms, out_regnos, out_values, outs, r);
        if (!r.halted) {   // the limit passed and the hart could not be stopped: dpc and values invalid
          status = failure(kStatusTimeout);
          out[1] = reg::target_riscv_dm::kRunStoppedNotHalted;
          putU32(out + 6, r.elapsed_us);
        } else {
          status = !ok ? (dm.lastCmderr() ? kStatusFault : failure(kStatusFault)) : r.stopped ? kStatusOk : kStatusTimeout;
          out[1] = r.stopped ? reg::target_riscv_dm::kRunStoppedStopped : reg::target_riscv_dm::kRunStoppedTimeoutHalted;
          putU32(out + 2, r.dpc);
          putU32(out + 6, r.elapsed_us);
          nvals = outs;
          for (uint8_t i = 0; i < outs; ++i) putU32(out + 11 + 4 * i, out_values[i]);
        }
      } else if (!dm.attached()) {
        status = failure(kStatusState);
      }
      out[0] = status;
      out[10] = nvals;
      return tail.finish(outcome(status, 0, 11u + 4u * nvals), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

// n(u16), then n steps, run in order up to the first failure:
//   0x01 write  address(u8) value(u32)
//   0x02 read   address(u8)                                    -> the value
//   0x03 poll   address(u8) mask(u32) value(u32) max(u16)      -> the last value read ((read & mask) == value within
//                                                                 max reads; timeout if never)
//   0x04 delay  us(u32)
//   0x05 poll   address(u8) mask(u32) value(u32) max_us(u32)   -> the last value read (the same, bounded by time:
//                                                                 it means the same on a slow link and a fast one)
// [TLV]  ->  done(u16) status(u8) nvals(u16) values [TLV]. done = steps completed (= the index of the failed step); the
// values are those of the reads and polls among them, plus the failed step's last value when it was a poll that timed
// out (a poll cut off by the line adds nothing). The waits (delays and the polls' time limits) may not add up to more
// than max_op_ms (rejected unsupported).
Result TargetRiscvDm::dmi(const uint8_t *p, size_t length, uint8_t *out, size_t capacity) {
  if (length < 2) return rejected(kRejectMalformed);
  const uint16_t count = getU16(p);
  // Check the whole list before running any of it: a broken list is malformed, not a partial run.
  size_t at = 2, values = 0;
  uint64_t wait_us = 0;
  for (uint16_t i = 0; i < count; ++i) {
    if (at >= length) return rejected(kRejectMalformed);
    size_t size = 0;
    switch (p[at]) {
      case kStepWrite: size = 6; break;
      case kStepRead: size = 2; ++values; break;
      case kStepPoll: size = 12; ++values; break;
      case kStepDelay: size = 5; if (at + size <= length) wait_us += getU32(p + at + 1); break;
      case kStepPollTime: size = 14; ++values; if (at + size <= length) wait_us += getU32(p + at + 10); break;
      default: return rejected(kRejectMalformed);   // an unknown step kind cannot be sized: the list is broken
    }
    if (at + size > length) return rejected(kRejectMalformed);
    at += size;
  }
  Tail tail;
  const Result parsed = tail.parse(p + at, length - at, out, capacity);
  if (refused(parsed)) return parsed;
  if (5 + 4 * values > capacity) return rejected(kRejectMalformed);   // the answer would not fit a frame
  if (wait_us > static_cast<uint64_t>(kMaxOpMs) * 1000u) return unsupportedValue(out, capacity);
  Ch32Dm &dm = port_->dm;
  size_t written = 5;
  uint16_t done = 0, nvals = 0;
  uint8_t status = kStatusOk;
  at = 2;
  for (uint16_t i = 0; i < count && status == kStatusOk; ++i) {
    const uint8_t kind = p[at];
    const uint8_t *s = p + at + 1;
    switch (kind) {
      case kStepWrite:
        if (!dm.writeDmi(s[0], getU32(s + 1))) status = kStatusLine;
        at += 6;
        break;
      case kStepRead: {
        uint32_t v = 0;
        if (!dm.readDmi(s[0], v)) status = kStatusLine;
        else { putU32(out + written, v); written += 4; ++nvals; }
        at += 2;
        break;
      }
      case kStepPoll: {
        const uint32_t mask = getU32(s + 1), want = getU32(s + 5);
        const uint16_t max = getU16(s + 9);
        bool met = false;
        uint32_t last = 0;
        for (uint16_t k = 0; k < (max ? max : 1) && !met && status == kStatusOk; ++k) {   // max 0: one read
          uint32_t v = 0;
          if (!dm.readDmi(s[0], v)) status = kStatusLine;
          else { last = v; met = (v & mask) == want; }
        }
        if (status == kStatusOk && !met) status = kStatusTimeout;
        if (status != kStatusLine) { putU32(out + written, last); written += 4; ++nvals; }   // met or timed out, not cut off
        at += 12;
        break;
      }
      case kStepDelay:
        delayMicroseconds(getU32(s));
        at += 5;
        break;
      case kStepPollTime: {
        const uint32_t mask = getU32(s + 1), want = getU32(s + 5), max_us = getU32(s + 9);
        bool met = false;
        uint32_t last = 0;
        const uint32_t started = micros();
        do {
          uint32_t v = 0;
          if (!dm.readDmi(s[0], v)) status = kStatusLine;
          else { last = v; met = (v & mask) == want; }
        } while (!met && status == kStatusOk && micros() - started < max_us);
        if (status == kStatusOk && !met) status = kStatusTimeout;
        if (status != kStatusLine) { putU32(out + written, last); written += 4; ++nvals; }
        at += 14;
        break;
      }
    }
    if (status == kStatusOk) ++done;
  }
  if (status == kStatusLine) failure(kStatusLine);   // no answer at all: the connection closes after this answer
  putU16(out, done);
  out[2] = status;
  putU16(out + 3, nvals);
  return tail.finish(outcome(status, done, written), out, capacity);
}

}  // namespace oep
