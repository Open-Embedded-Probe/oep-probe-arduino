// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepTarget.h"

#include <Arduino.h>

#include "OepPinTable.h"
#include "OepPlatform.h"

namespace oep {
namespace {

namespace wire = reg::wire_rvswd;

constexpr uint8_t kDmStatus = 0x11, kDmControl = 0x10;

// Whether a module answers on a live link: all zeros / all ones is a line held low or floating up through its pull-up
// (DmiPhy::outcomeOf), no answer. Two reads (Ch32Dm::moduleAnswersSure): one missed read gives the read before it - a
// DATA0 of 0 the console's idle poll read - and alone made attach bring a live link up afresh (a wake: an L103 restarts).
bool moduleAnswers(Ch32Dm &dm, uint32_t &status) { return dm.moduleAnswersSure(status); }

// A cold CH32 ignores the first wake now and then (the CH32L103 answered on the fifth, 2026-09-23), and one that
// sat idle past its link timeout has dropped the link: bring the bus up afresh and try again before saying no - while
// the attach budget lasts (the PHY's deadline). Each try again counts in search_retries.
// after_reset: the reset line was just let go of (attach's reset TLV with method 0): the tries go on until the search's
// deadline, not three - a target that restarts itself on its way out of reset (a CH32V003 booting through its
// bootloader, which then hands over with a system reset) answers nothing for a few hundred ms (as the riscv-dm reset op
// waits it out: Ch32Dm::reset), and three tries took a few ms. (1000 tries, each 1 ms apart at least, stop it should
// a caller have set no deadline.)
bool attachAndRead(Ch32Dm &dm, uint32_t &status, bool after_reset = false) {
  for (int attempt = 0; attempt < (after_reset ? 1000 : 3); ++attempt) {
    if (attempt) {
      if (dm.phy().pastDeadline()) break;
      dm.phy().countSearchRetry();
      if (after_reset) delay(1);
    }
    if (dm.attach() && dm.readDmi(kDmStatus, status) && status != 0 && status != 0xffffffffu) return true;
    dm.detach();
  }
  return false;
}

// The max_speed TLV (0x01, u32 Hz) of attach / scan: 0 when absent. A refusal: another length than 4 bytes or 0
// (malformed, core §2.3).
Result maxSpeed(Tail &tail, uint8_t tag, uint32_t &hz, bool &critical, uint8_t *out, size_t capacity) {
  hz = 0;
  const uint8_t *v = nullptr;
  const Result r = tail.fixed(tag, 4, v, out, capacity, &critical);
  if (refused(r)) return r;
  if (v && getU32(v) == 0) return rejected(kRejectMalformed);
  if (v) hz = getU32(v);
  return completed();
}

// The idle_clock TLV (0x04, u8 0 high / 1 low; rvswd only) of attach / scan. given: it was carried - absent, a new
// connection rests high, and an existing one keeps its own (oep-if-debug §1, §3). A refusal as maxSpeed's.
Result idleClock(Tail &tail, uint8_t tag, bool &low, bool &given, bool &critical, uint8_t *out, size_t capacity) {
  low = given = false;
  const uint8_t *v = nullptr;
  const Result r = tail.fixed(tag, 1, v, out, capacity, &critical);
  if (refused(r)) return r;
  if (!v) return completed();
  if (v[0] > wire::kIdleClockLow) return rejected(kRejectMalformed);
  low = v[0] == wire::kIdleClockLow;
  given = true;
  return completed();
}

// The reset line, open drain: pulled low, then released - never driven high (oep-if-debug §3) - and back to its idle
// state when the settings give it one (oep-core §8; without one it stays released, Hi-Z).
struct ResetLine {
  int channel;
  PinTable *pins;
  WireLossClock *loss;    // the connection's wire-loss clock: the hold and the limits::kWireLostMs after it do not count
};
void holdReset(void *ctx) {
  const ResetLine &line = *static_cast<ResetLine *>(ctx);
  platformGpio(line.channel, kGpioOpenDrainLow);
}
void releaseReset(void *ctx) {
  const ResetLine &line = *static_cast<ResetLine *>(ctx);
  platformGpio(line.channel, kGpioOpenDrainRelease);
  if (line.pins) line.pins->rest(static_cast<uint16_t>(line.channel));
  if (line.loss) line.loss->excuseReset();
}

// The halted hart's dpc for the attach answer's TLV 0x11 (DATA0 is used and put back).
bool readDpc(Ch32Dm &dm, uint32_t &dpc) { return dm.readDpc(dpc); }

}  // namespace

// ---- oep.wire.rvswd / oep.wire.swio ------------------------------------------------------------------

void chooseRest(DebugPort &port) {
  namespace rv = reg::wire_rvswd;
  namespace pc = reg::probe_config;
  uint32_t id = 0;
  port.has_tid = port.dm.readDmiSure(0x7f, id) && id != 0 && id != 0xffffffffu;
  port.tid = port.has_tid ? id : 0;
  const uint8_t mode = port.rest_setting ? port.rest_setting(port.rest_context, port.swclk) : pc::kRvswdRestModeAuto;
  bool low = false;
  if (mode == pc::kRvswdRestModeLow) low = true;
  else if (mode == pc::kRvswdRestModeAuto && port.has_tid)
    low = (port.tid >> rv::kTargetIdFamilyShift) == rv::kIdleLowFamilyCh32l103;   // registry idle_low_family
  port.rest_low = low;
  port.dm.phy().setIdleClockLow(low);
}

// The attach result's target_id (oep-if-debug §1): scheme dmi_7f, the u32 at DMI 0x7F; 0 and all ones = none.
size_t targetId(DebugPort &port, uint8_t *out, size_t room) {
  uint32_t id = 0;
  // read twice (readDmiSure): a missed read gives the value read before it, not the target's
  port.has_tid = port.dm.readDmiSure(0x7f, id) && id != 0 && id != 0xffffffffu;   // kept for connections
  port.tid = port.has_tid ? id : 0;
  if (room < kTlvHeader + 5 || !port.has_tid) return 0;
  putTlvHeader(out, wire::kTlvAttachAnswerTargetId, 5);   // scheme(u8) id(u32)
  out[3] = reg::common::kTargetIdSchemeDmi7f;
  putU32(out + 4, id);
  return kTlvHeader + 5;
}

bool residentAttach(DebugPort &port, uint16_t swdio, uint16_t swclk, uint32_t max_hz) {
  if (port.connected || port.resident) return port.resident;
  if (!usePair(port, swdio, swclk)) return false;
  DmiPhy &phy = port.dm.phy();
  if (!phy.setMaxHz(max_hz)) phy.setMaxHz(0);
  phy.setIdleClockLow(false);
  if (port.dm.attached() && phy.clockHz() > max_hz) port.dm.detach();
  AttachDeadline budget(phy);
  uint32_t status = 0;
  if (!attachAndRead(port.dm, status)) {
    port.dm.detach();
    freeWire(port);
    return false;
  }
  if (phy.canIdleClockLow()) chooseRest(port);   // rvswd: the setting, else from target_id (oep-if-debug §3.1)
  port.dm.ackHaveReset();
  port.resident = true;
  phy.loss().clear();
  phy.holdWakes(true);   // no wake outside attach and reset (debug §2)
  return true;
}

void residentRelease(DebugPort &port) {
  if (!port.resident) return;
  port.resident = false;
  port.dm.detach();   // the target untouched (debug §2: closing does not reset or resume it)
  port.dm.phy().holdWakes(false);
  freeWire(port);
}

bool attachRunning(DebugPort &port, uint8_t user, uint32_t &dmstatus, uint32_t max_hz, bool idle_low) {
  if (port.connected) {
    if (!moduleAnswers(port.dm, dmstatus)) return false;
    // a pending havereset first (oep-if-debug §4.6)
    if (port.dm.ackHaveReset() && !port.dm.readDmiSure(kDmStatus, dmstatus)) return false;
  } else {
    DmiPhy &phy = port.dm.phy();
    if (!phy.setMaxHz(max_hz)) phy.setMaxHz(0);   // the slot's settings were checked when it was set
    phy.setIdleClockLow(idle_low);
    if (max_hz && port.dm.attached() && phy.clockHz() > max_hz) port.dm.detach();
    AttachDeadline budget(phy);   // oep-if-debug §1, as a host's attach
    // a failed try holds nothing: the pins go free until the slot tries again
    auto fail = [&]() {
      port.dm.detach();
      freeWire(port);
      return false;
    };
    if (!attachAndRead(port.dm, dmstatus)) return fail();
    if (!idle_low && phy.canIdleClockLow()) chooseRest(port);   // rvswd without a given rest (oep-if-debug §3.1)
    else port.rest_low = idle_low;
    port.dm.ackHaveReset();
    if (!port.dm.readDmiSure(kDmStatus, dmstatus)) return fail();
    const uint32_t number = ResourceNumbers::take(ResourceNumbers::kConnection);
    if (!number) return fail();
    port.connected = true;
    port.number = number;
    port.lost = false;
    phy.loss().clear();   // a new connection: its own wire-loss clock (oep-if-debug §2)
    phy.holdWakes(true);   // a connection: no wake outside attach and reset (oep-if-debug §2)
    holdPins(port);
    uint8_t tlv[8];
    targetId(port, tlv, sizeof tlv);   // the connections entry's tid
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

void releaseConnection(DebugPort &port, uint8_t user, bool force, bool lost, bool detached) {
  port.users &= static_cast<uint8_t>(~user);
  if (!port.connected || (port.users && !force)) return;
  port.dm.detach();   // haltreq lowered, dmactive kept (the DM is not reset: DATA0 holds the console's frame)
  port.dm.phy().holdWakes(false);
  port.connected = false;
  port.users = 0;
  port.has_tid = false;
  port.lost = lost;
  port.detached = detached;
  ++port.closes;
  ResourceNumbers::close(port.number);
  // oep-if-debug §2: not a lost line - the lines stay driven at the connection's rest (a CH32L103 drops its halt about
  // 5 ms after the drive stops), the pins free for anyone to take
  if (!lost && port.hold.allows(port.pins, port.swdio, port.swclk)) {
    port.dm.phy().hold();
    port.holding = true;
    if (port.pins && port.pin_choice) port.pins->releaseQuiet(port.pin_owner);
    return;
  }
  port.holding = false;
  freeWire(port);
}

void checkHold(DebugPort &port) {
  if (!port.holding) return;
  if (port.connected) { port.holding = false; return; }
  const bool taken = port.pins && (port.pins->owner(port.swdio) != 0 ||
                                   (port.swclk != 0xffff && port.pins->owner(port.swclk) != 0));
  if (taken) { port.holding = false; return; }   // the taker drives them now
  if (!port.hold.allows(port.pins, port.swdio, port.swclk)) {
    port.holding = false;
    freeWire(port);
  }
}

void checkHoldOf(void *port) { checkHold(*static_cast<DebugPort *>(port)); }

// One DMSTATUS read, with the wire retries of one request (oep-if-debug §2). A check that gets nothing back is not wire
// loss by itself: the connection closes only once the wire has failed for limits::kWireLostMs with no good exchange between
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
// a request naming one is refused (cause 5) only when the idle is an output.
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

uint16_t pairHeld(const DebugPort &port, uint16_t swdio, uint16_t swclk) {
  if (!port.pin_choice || !port.pins) return 0xffff;
  auto held = [&](uint16_t c) {   // this wire's own connection is not "something else" (its seat is another matter)
    if (c == 0xffff) return false;
    const uint8_t owner = port.pins->owner(c);
    return owner != 0 && owner != port.pin_owner;
  };
  return held(swdio) ? swdio : held(swclk) ? swclk : 0xffff;
}

Result pairHeldRefusal(const DebugPort &port, uint16_t swdio, uint16_t swclk, uint8_t *out, size_t capacity) {
  const uint16_t held = pairHeld(port, swdio, swclk);
  if (held == 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse);
  return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, held);
}

bool usePair(DebugPort &port, uint16_t swdio, uint16_t swclk) {
  if (!port.connected && pairIdle(port, swdio, swclk, true) != 0xffff) return false;   // an output idle: never driven
  // the pair the link is on: a live connection's own, or - nothing live - only while nothing else holds its pins (a
  // plan may have taken them since the link was last there)
  if (swdio == port.swdio && swclk == port.swclk) return port.connected || pairFree(port, swdio, swclk);
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

// connections (oep-if-debug §2.1): first(u8) -> more(u8) count(u8), count x entry: connection(u32) swdio(u16) swclk(u16)
// speed_hz(u32) tid_scheme(u8) tid_len(u8) tid (no element length, core §2.3), [TLV]. One connection per wire here
// (max_connections 1): at most one entry.
Result connectionsOf(DebugPort &port, uint8_t first, uint32_t speed_hz, uint8_t *out, size_t capacity, bool rest) {
  if (capacity < 2 + 18 + kTlvHeader + 2) return failed();
  out[0] = 0;   // more: never (one entry at most)
  out[1] = 0;
  if (!port.connected || first > 0) return completed(2);
  out[1] = 1;
  uint8_t *e = out + 2;
  putU32(e, port.number);
  putU16(e + 4, port.swdio);
  putU16(e + 6, port.swclk);
  putU32(e + 8, speed_hz);
  e[12] = port.has_tid ? reg::common::kTargetIdSchemeDmi7f : 0;
  e[13] = port.has_tid ? 4 : 0;   // tid_len: scheme dmi_7f's value is a u32
  if (port.has_tid) putU32(e + 14, port.tid);
  size_t n = 2u + (port.has_tid ? 18 : 14);
  if (rest) {   // rvswd: TLV idle_clock (entry_index, rest) for the entry (oep-if-debug §2.1, §3.1)
    putTlvHeader(out + n, reg::wire_rvswd::kTlvConnectionsAnswerIdleClock, 2);
    out[n + kTlvHeader] = 0;
    out[n + kTlvHeader + 1] = port.rest_low ? reg::wire_rvswd::kIdleClockLow : reg::wire_rvswd::kIdleClockHigh;
    n += kTlvHeader + 2;
  }
  return completed(n);
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
  if (halt_) w.u32(kTagFeatures, 1u << 0);   // attach_halt: attach's method 1 (oep-if-debug §3)
  if (phy.minClockHz()) w.u32(kTagMinClockHz, phy.minClockHz());   // a max_speed under it is unsupported
  if (phy.maxClockHz()) w.u32(kTagMaxClockHz, phy.maxClockHz());   // a fixed-speed link: min = max = its speed
  return w.ok() ? w.length() : 0;
}

// The attach pins TLV (oep-if-debug §1): the pair to attach on, the link moved there. Absent: the live connection's pair
// (join it), else the fixed pair, else - pins the host chooses, nothing live - refused (the host names one). Another pair
// than the live connection's takes its seat only when a slot alone uses it (the seat rule); else refused. 0, or a
// reject reason; for unavailable, `why` says what core §4.3's payload carries.
uint8_t WireRvswd::choosePair(const uint8_t *pins, size_t len, PinRefusal &why) {
  why = {reg::core::kUnavailableCauseWrongState, 0xffff};
  if (!pins) return port_.pin_choice && !port_.connected ? kRejectUnavailable : 0;   // the host names a pair
  if (len != 4) return kRejectMalformed;
  const uint16_t d = getU16(pins), c = getU16(pins + 2);
  if (!pairAllowed(port_, d, c)) return kRejectUnsupported;   // not a pair this wire declares (oep-if-debug §1)
  const uint16_t held = pairHeld(port_, d, c);                 // the pair the link is on included (a plan may hold it now)
  if (held != 0xffff) {
    why = {reg::core::kUnavailableCausePinInUse, held};
    return kRejectUnavailable;
  }
  if (d == port_.swdio && c == port_.swclk) return 0;          // the pair the link is on (live or not)
  if (!port_.pins) return kRejectUnavailable;
  if (port_.connected) {
    // no seat: the host's connection is on another pair (max_connections 1)
    why = {reg::core::kUnavailableCauseLimit, 0xffff};
    if (port_.users != DebugPort::kUserSlot) return kRejectUnavailable;
    releaseConnection(port_, DebugPort::kUserSlot, true);     // the seat rule: a slot-only connection makes room
  }
  why = {reg::core::kUnavailableCausePinInUse, 0xffff};
  return usePair(port_, d, c) ? 0 : kRejectUnavailable;
}

// count(u8) pairs [TLV 0x01 max_speed, 0x02 skip (count 0), 0x04 idle_clock (rvswd)]
//   ->  tried(u8) count(u8), count x (kind(u8) swdio(u16) swclk(u16) raw DMSTATUS(u32)) (oep-if-debug §1, §3; 9 bytes
// each, no element length). Only dmactive is written; a pair without a module goes back to Hi-Z. A live connection's
// settings (its speed, its idle_clock) are never changed by a scan (debug §1).
Result WireRvswd::scan(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // count(u8) count x (swdio u16, swclk u16) [TLV 0x01 max_speed, 0x04 idle_clock (rvswd)]: the host names every
  // candidate, count 1 or more (oep-if-debug §1: count 0 is malformed, no implicit list)
  if (length < 1 || payload[0] == 0 || length < 1u + 4u * payload[0]) return rejected(kRejectMalformed);
  const uint8_t count = payload[0];
  const size_t fixed = 1u + 4u * count;
  static const uint8_t kScanTags[] = {wire::kTlvScanMaxSpeed, wire::kTlvScanIdleClock};
  Tail tail;
  Result unknown = completed();   // an unknown critical tag: unsupported once the format is checked
  const Result parsed = tail.parse(payload + fixed, length - fixed, kScanTags, isRvswd() ? 2 : 1, out, capacity, &unknown);
  if (refused(parsed)) return parsed;
  DmiPhy &phy = port_.dm.phy();
  for (uint8_t k = 0; k < count; ++k)   // the same candidate twice is malformed (oep-if-debug §1)
    for (uint8_t q = 0; q < k; ++q)
      if (getU32(payload + 1 + 4 * k) == getU32(payload + 1 + 4 * q)) return rejected(kRejectMalformed);
  uint32_t max_hz = 0;
  bool critical = false, idle_low = false, idle_given = false, idle_critical = false;
  {
    const Result r = maxSpeed(tail, wire::kTlvScanMaxSpeed, max_hz, critical, out, capacity);
    if (refused(r)) return r;
  }
  if (isRvswd()) {
    const Result r = idleClock(tail, wire::kTlvScanIdleClock, idle_low, idle_given, idle_critical, out, capacity);
    if (refused(r)) return r;
  }
  if (refused(unknown)) return unknown;
  if (max_hz && phy.minClockHz() && max_hz < phy.minClockHz()) return unsupportedTag(out, capacity, wire::kTlvScanMaxSpeed | (critical ? kTagCritical : 0));
  // every pair listed is one this wire allows (oep-if-debug §1: unsupported, tag 0x00 and TLV 0x40 its index); every
  // check before anything is looked at on the wire (core §4.3)
  for (uint8_t k = 0; k < count; ++k)
    if (!pairAllowed(port_, getU16(payload + 1 + 4 * k), getU16(payload + 3 + 4 * k))) return unsupportedIndex(out, capacity, k);
  // then whose pins nothing else holds, and - the one seat taken - the live one
  for (uint8_t k = 0; k < count; ++k) {
    const uint16_t d = getU16(payload + 1 + 4 * k), c = getU16(payload + 3 + 4 * k);
    const uint16_t off = pairDisabled(port_, d, c);   // the settings disable it: cause 5 (probe.config §1)
    if (off != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, off);
    const uint16_t idle = pairIdle(port_, d, c, true);   // an output idle: cause 5 (debug §1)
    if (idle != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, idle);
    if (port_.connected) {   // a live connection's pins are never searched (cause 1); its one PHY is busy (cause 2)
      if (d == port_.swdio || d == port_.swclk) return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, d);
      if (c != 0xffff && (c == port_.swdio || c == port_.swclk))
        return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, c);
      return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
    }
    if (!pairFree(port_, d, c)) return pairHeldRefusal(port_, d, c, out, capacity);   // cause 1, the channel
  }
  if (capacity < 2) return failed();
  if (!port_.connected) {   // the scan's line settings (a connection keeps its own); no max_speed: the slowest speed
    if (!phy.setMaxHz(max_hz ? max_hz : phy.minClockHz())) phy.setMaxHz(0);
    phy.setIdleClockLow(idle_low);
    if (max_hz && port_.dm.attached() && phy.clockHz() > max_hz) port_.dm.detach();
  }
  size_t at = 2;
  uint8_t tried = 0, found = 0;
  // No pair starts later than limits::kScanBudgetMs after the request (oep-if-debug §1: a scan answers within
  // max_op_ms; this implementation stops well before): the host goes on with the rest (count 0 with skip, or the
  // pairs after tried). 26 free pins bit-banged pair by pair kept an RP2350 from answering for seconds
  // (0.0.18). At least one pair is tried while any is left (tried 0 means the list is used up); one try is bounded by
  // the attach budget.
  const uint32_t began = millis();
  auto tryPair = [&](uint16_t d, uint16_t c) {   // false: the answer is full, or its time is up
    if (at + 9 > capacity || (tried && millis() - began >= limits::kScanBudgetMs)) return false;
    uint32_t status = 0;
    bool ok = false;
    if (usePair(port_, d, c)) {
      {
        AttachDeadline budget(phy);
        ok = port_.dm.probe(status);                     // wake / configuration and dmactive only (debug §1)
      }
      port_.dm.detach();                                 // nothing held: dmactive stays
      freeWire(port_);                                   // found or not, the pair to its free state (debug §1)
    }
    if (ok) {
      out[at] = wire::kScanKindRiscvDm;
      putU16(out + at + 1, d);
      putU16(out + at + 3, c);
      putU32(out + at + 5, status);
      at += 9;
      ++found;
    }
    ++tried;
    return true;
  };
  for (uint8_t k = 0; k < count && tryPair(getU16(payload + 1 + 4 * k), getU16(payload + 3 + 4 * k)); ) ++k;
  out[0] = tried;
  out[1] = found;
  return completed(at);
}

// method(u8: 0 leave it running, 1 halt) [TLV 0x01 max_speed (required), 0x03 pins, 0x04 idle_clock, 0x05 reset]
//   ->  connection(u16) DMSTATUS(u32, after the attach: halted when it halted) flags(u8: bit0 acknowledged a pending
//       havereset, bit1 existing, bit3 halted) speed_hz(u32) [TLV 0x10 target_id, 0x11 dpc while halted, 0x12
//       search_retries when a speed search ran]
// A failed attach answers status(u8) alone: line (no answer), timeout (the hart did not stop / start).
Result WireRvswd::attach(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // method(u8) swdio(u16) swclk(u16) max_speed(u32) [TLV] (oep-if-debug §1, §3): idle_clock 0x04 is rvswd's (on swio an
  // unknown tag), reset 0x05 on both. A short fixed part is malformed.
  static const uint8_t kAttachTags[] = {wire::kTlvAttachReset, wire::kTlvAttachIdleClock};
  if (length < 9) return rejected(kRejectMalformed);
  const uint8_t method = payload[0];
  const uint16_t swdio = getU16(payload + 1), swclk = getU16(payload + 3);
  const uint32_t max_hz = getU32(payload + 5);
  Tail tail;
  // Every format check of the request first, then what it does not handle (core §4.3: every check before anything
  // changes): an unknown critical tag is held back until then.
  Result unknown = completed();
  const Result parsed = tail.parse(payload + 9, length - 9, kAttachTags, isRvswd() ? 2 : 1, out, capacity, &unknown);
  if (refused(parsed)) return parsed;
  DmiPhy &phy = port_.dm.phy();
  bool idle_low = false, idle_given = false, idle_critical = false;
  if (isRvswd()) {
    const Result r = idleClock(tail, wire::kTlvAttachIdleClock, idle_low, idle_given, idle_critical, out, capacity);
    if (refused(r)) return r;
  }
  if (!max_hz) return rejected(kRejectMalformed);   // max_speed 0 (oep-if-debug §1)
  bool reset_critical = false;
  const uint8_t *reset_value = nullptr;
  {
    const Result q = tail.fixed(wire::kTlvAttachReset, 4, reset_value, out, capacity, &reset_critical);
    if (refused(q)) return q;
  }
  if (refused(unknown)) return unknown;
  if (port_.resident && port_.yield) port_.yield(port_.yield_context);   // a pass-through gives way (config §1.2)
  // method: 0 always, 1 when describe's features bit0 (attach_halt) says so; other values unsupported 0x00 (§3)
  if (method > wire::kAttachMethodHalt || (method == wire::kAttachMethodHalt && !halt_)) return unsupportedValue(out, capacity);
  const bool halt = method == wire::kAttachMethodHalt;
  // reset(channel u16, hold_ms u16): the line to hold first; one this probe allows (unsupported otherwise) and nobody
  // holds (unavailable), held at most max_op_ms (core §7.5)
  bool with_reset = false;
  int reset_channel = -1;
  uint16_t hold_ms = 0;
  if (reset_value) {
    reset_channel = getU16(reset_value);
    hold_ms = getU16(reset_value + 2);
    if (reset_channel > 63 || !((port_.reset_allowed >> reset_channel) & 1) || hold_ms > kMaxOpMs)
      return Tail::refuse(wire::kTlvAttachReset, reset_critical, out, capacity);
    with_reset = true;
  }
  // below min_clock_hz, or a pair the declaration does not allow: unsupported, the fixed part's 0x00 (oep-if-debug §1)
  if (phy.minClockHz() && max_hz < phy.minClockHz()) return unsupportedValue(out, capacity);
  if (!pairAllowed(port_, swdio, swclk)) return unsupportedValue(out, capacity);
  // a channel the settings disable, or whose idle is an output - the pair, the reset line: cause 5 with the channel
  uint16_t off = pairDisabled(port_, swdio, swclk);
  if (off == 0xffff && with_reset && port_.pins && port_.pins->disabled(static_cast<uint16_t>(reset_channel)))
    off = static_cast<uint16_t>(reset_channel);
  if (off != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, off);
  uint16_t idle = pairIdle(port_, swdio, swclk, true);
  if (idle == 0xffff && with_reset && port_.pins) {
    const uint8_t mode = port_.pins->idle(static_cast<uint16_t>(reset_channel));
    if (mode == PinTable::kIdleOutputLow || mode == PinTable::kIdleOutputHigh) idle = static_cast<uint16_t>(reset_channel);
  }
  if (idle != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, idle);
  // attach makes a new connection, never joins or evicts one (oep-if-debug §1): a live connection holds these pins -
  // cause 1 with the channel - or the wire's one connection is taken - cause 2 (max_connections 1)
  if (port_.connected) {
    if (port_.swdio == swdio || port_.swclk == swdio) return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, swdio);
    if (swclk != 0xffff && (port_.swdio == swclk || port_.swclk == swclk))
      return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, swclk);
    return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
  }
  {
    const uint16_t held = pairHeld(port_, swdio, swclk);   // a plan or another wire holds one of them: cause 1
    if (held != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, held);
    if (!usePair(port_, swdio, swclk)) return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse);
  }
  if (with_reset && port_.pins && port_.pins->owner(static_cast<uint16_t>(reset_channel)))
    return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, static_cast<uint16_t>(reset_channel));
  // How the line rests (oep-if-debug §1, §3): an attach that carries idle_clock sets it - an existing connection's too;
  // one that does not leaves a live connection's as it is (a tool joining a slot's connection must not change how the
  // line rests), and a new connection rests high.
  if ((idle_given || !port_.connected) && !phy.setIdleClockLow(idle_low)) {
    if (idle_given) return Tail::refuse(wire::kTlvAttachIdleClock, idle_critical, out, capacity);
    phy.setIdleClockLow(false);
  }
  if (capacity < 13) return failed();
  ResetLine reset_line{reset_channel, port_.pins, &phy.loss()};
  uint32_t status = 0, dpc = 0;
  uint8_t flags = 0;
  uint8_t failure = kStatusOk;
  bool have_dpc = false;
  // The speed search and its retries take at most the attach budget, a reset's hold_ms aside (this implementation's
  // limits::kAttachBudgetMs, well within max_op_ms, oep-if-debug §1). What failed on the way is the answer's
  // search_retries. Inside attach the wire's wake may go out on a live connection too (oep-if-debug §2).
  AttachDeadline budget(phy, with_reset ? hold_ms : 0);
  DmiPhy::WakeScope wakes(phy);
  phy.clearSearchRetries();
  {
    if (!phy.setMaxHz(max_hz))   // a ceiling this link cannot keep (a fixed speed above it): the fixed part's 0x00
      return unsupportedValue(out, capacity);
    // A scan may have left the link up at a speed over the new ceiling: search again under it.
    if (port_.dm.attached() && phy.clockHz() > max_hz) port_.dm.detach();
    if (with_reset && halt) {
      if (!port_.dm.attachUnderReset(holdReset, releaseReset, &reset_line, hold_ms, dpc)) {
        // the module answers but the hart never stopped: timeout; no answer: line
        const bool answers = moduleAnswers(port_.dm, status);
        failure = answers ? kStatusTimeout : kStatusLine;
      } else {
        have_dpc = true;
        settleRest(idle_given, idle_low);
        if (!port_.dm.readDmiSure(kDmStatus, status)) failure = kStatusLine;
      }
    } else {
      if (with_reset) port_.dm.pulseReset(holdReset, releaseReset, &reset_line, hold_ms);
      if (!attachAndRead(port_.dm, status, with_reset)) {
        failure = kStatusLine;
      } else {
        settleRest(idle_given, idle_low);   // first: a CH32L103 resting high loses its debug module in ~1 ms (§3.1)
        // A pending havereset freezes a V00x's DMSTATUS halt / run bits at their reset values (ch32rv 0.8.0):
        // acknowledge it first so the DMSTATUS returned is current, and say that it was there (flags bit0).
        if (port_.dm.ackHaveReset()) flags |= wire::kAttachFlagsHaveresetAcked;
        if (!port_.dm.readDmiSure(kDmStatus, status)) failure = kStatusLine;
        else if (halt && !port_.dm.halt()) failure = kStatusTimeout;
        else if (halt && !port_.dm.readDmiSure(kDmStatus, status)) failure = kStatusLine;   // the answer: DMSTATUS after the halt
      }
    }
    if (failure == kStatusOk) {
      const uint32_t number = ResourceNumbers::take(ResourceNumbers::kConnection);
      if (!number) {
        port_.dm.detach();
        freeWire(port_);
        return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
      }
      port_.connected = true;
      port_.number = number;
      port_.lost = false;
      phy.loss().clear();   // a new connection: its own wire-loss clock (oep-if-debug §2)
      phy.holdWakes(true);  // a connection: no wake outside attach and reset (oep-if-debug §2)
      holdPins(port_);
      if (with_reset) { ++port_.resets; port_.reset_detail = legacy::kMarkDetailResetAttachReset; }
    } else {
      port_.dm.detach();   // a failed attach holds no pins: they go free
      freeWire(port_);
    }
  }
  if (failure != kStatusOk) return failedStatus(failure, out, capacity);
  port_.users |= DebugPort::kUserHost;
  if (port_.dm.halted()) {
    flags |= wire::kAttachFlagsHalted;
    if (!have_dpc) have_dpc = readDpc(port_.dm, dpc);
  }
  putU32(out, port_.number);   // connection(u32) id(u32: DMSTATUS) flags(u8) speed_hz(u32) [TLV] (oep-if-debug §3)
  putU32(out + 4, status);
  out[8] = flags;
  putU32(out + 9, phy.clockHz());
  size_t n = 13 + targetId(port_, out + 13, capacity - 13);
  if (have_dpc && (flags & wire::kAttachFlagsHalted) && n + kTlvHeader + 4 <= capacity) {
    putTlvHeader(out + n, wire::kTlvAttachAnswerDpc, 4);
    putU32(out + n + kTlvHeader, dpc);
    n += kTlvHeader + 4;
  }
  if (isRvswd() && n + kTlvHeader + 1 <= capacity) {   // the rest in use, always (oep-if-debug §3.1)
    putTlvHeader(out + n, reg::wire_rvswd::kTlvAttachAnswerIdleClock, 1);
    out[n + kTlvHeader] = port_.rest_low ? reg::wire_rvswd::kIdleClockLow : reg::wire_rvswd::kIdleClockHigh;
    n += kTlvHeader + 1;
  }
  return completed(n);
}

// The rest of the connection just made (oep-if-debug §3.1): the attach's idle_clock as given, else chooseRest.
void WireRvswd::settleRest(bool given, bool low) {
  if (!isRvswd()) return;
  if (given) port_.rest_low = low;
  else chooseRest(port_);
}

Result WireRvswd::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  port_.dm.phy().beginRequest();   // the request's allowance for wire retries (oep-if-debug §2)
  checkHold(port_);
  if (op == kOpScan || op == kOpAttach) port_.holding = false;   // they take the line (or move it): the hold ends (§2)
  switch (op) {
    case reg::wire_rvswd::kOpHolds: {   // first(u8) [TLV] -> more count entries (swdio swclk rest), lock-free (§2.2)
      const Result parsed = plainTail(tail, payload, length, 1, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 7) return failed();
      out[0] = out[1] = 0;
      if (!port_.holding || payload[0] > 0) return completed(2);
      out[1] = 1;
      putU16(out + 2, port_.swdio);
      putU16(out + 4, port_.swclk);
      out[6] = isRvswd() && port_.rest_low ? reg::wire_rvswd::kIdleClockLow : reg::wire_rvswd::kIdleClockHigh;
      return completed(7);
    }
    case kOpScan: return scan(payload, length, out, capacity);
    case kOpAttach: {
      const Result r = attach(payload, length, out, capacity);
      port_.dm.noteRequest();   // it may have halted the hart or reset the target: DMSTATUS before the console's DATA0
      return r;
    }
    case kOpConnections: {   // first(u8) [TLV] -> more count entries: the live connections from the first-th (no lock)
      const Result parsed = plainTail(tail, payload, length, 1, out, capacity);
      if (refused(parsed)) return parsed;
      return connectionsOf(port_, payload[0], port_.dm.phy().clockHz(), out, capacity, isRvswd());
    }
    case kOpDetach: {   // connection(u32) [TLV]: the connection and the streams on it go (oep-if-debug §2)
      if (length < 4) return rejected(kRejectMalformed);
      const Result parsed = plainTail(tail, payload, length, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (!port_.connected || getU32(payload) != port_.number)
        return ResourceNumbers::refuse(getU32(payload), ResourceNumbers::kConnection, out, capacity);
      releaseConnection(port_, DebugPort::kUserHost, true, false, false);
      return completed();
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

// ---- oep.target.riscv-dm -------------------------------------------------------------------------

size_t TargetRiscvDm::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);   // the optional ops are in the ops tag the endpoint writes (offers); no features
  // block read / write use a0, a1, s0, s1 and put them back before answering (oep-if-debug §4.5)
  // One block operation's data in bytes (oep-if-debug §4.5): what fits the endpoint's frame - write_block's request
  // and read_block's answer both fit max_frame - and the word buffer. The host takes count from this, not from max_frame.
  w.u16(kTagMaxLength, maxLength());
  return w.ok() ? w.length() : 0;
}

// The status of an op that did not go through: `otherwise` when the module still answers, line when it does not
// (oep-if-debug §2). A request that gets nothing back within its limits::kWireRetryMs answers status line and keeps the
// connection; the connection is closed once the answer is out only when the wire has now failed for limits::kWireLostMs of
// real time with no good exchange between (the wire-loss clock, shared with the console's reads).
uint8_t TargetRiscvDm::failure(uint8_t otherwise) {
  uint32_t status = 0;
  WireLossClock &loss = port_->dm.phy().loss();
  if (moduleAnswers(port_->dm, status)) return otherwise;   // two reads: a missed one alone is no "line"
  loss.silent();   // all zeros / ones: no module behind the answer
  if (loss.lost()) line_lost_ = true;
  return kStatusLine;
}

// An answer that ran (completed) turned into status line, its form kept: the status byte is out[0] (halt, resume, reset,
// step, run) or out[2] after done(u16) (dmi, read_block, write_block); done stays what the op progressed.
Result TargetRiscvDm::asLine(uint8_t op, uint8_t *out, const Result &r) {
  const bool with_done = op == kOpDmi || op == kOpReadBlock || op == kOpWriteBlock;
  const size_t at = with_done ? 2 : 0;
  if (r.length <= at) return r;   // no status in it (not expected for a completed answer)
  out[at] = kStatusLine;
  return outcome(kStatusLine, with_done ? getU16(out) : 0, r.length);
}

Result TargetRiscvDm::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 4) return rejected(kRejectMalformed);   // connection(u32) first (common §2)
  // The request's form and values first, nothing run (port_ null), then the connection it names (core §4.3: every
  // check before anything runs).
  port_ = nullptr;
  const Result checked = dispatch(op, payload + 4, length - 4, out, capacity);
  if (refused(checked)) return checked;
  // The wire whose live connection the request names (one riscv-dm serves every wire's connections).
  for (DebugPort *port : ports_)
    if (port && port->connected && getU32(payload) == port->number) port_ = port;
  if (!port_) return ResourceNumbers::refuse(getU32(payload), ResourceNumbers::kConnection, out, capacity);
  line_lost_ = false;
  port_->dm.phy().beginRequest();   // the request's allowance for wire retries (oep-if-debug §2)
  // the ops other than dmi are on hart 0: hartsel 0 before, and so after (oep-if-debug §4); dmi is the host's own
  if (op != kOpDmi) port_->dm.selectHart0();
  Result r = dispatch(op, payload + 4, length - 4, out, capacity);
  port_->dm.noteRequest();   // the console reads DMSTATUS before its next DATA0 read (oep-if-console §3)
  // Wire loss is looked at after every request on the connection, whatever the op answered (oep-if-debug §2): an op
  // that ran on reads which may be a register's own all zeros / all ones (a raw dmi read of DATA0, say) leaves the
  // clock running. Decided here, the answer says line and the connection closes after it.
  if (!line_lost_ && r.resolution == kResolutionCompleted && port_->dm.phy().loss().lost()) {
    line_lost_ = true;
    r = asLine(op, out, r);
  }
  // a block op that could not put back what it used: its answer, then the connection and its streams close (debug §4)
  const bool unclean = (op == kOpReadBlock || op == kOpWriteBlock) && port_->dm.cleanupFailed();
  if (line_lost_ || unclean) releaseConnection(*port_, 0xff, true, true);   // the answer first; the connection goes
  return r;
}

// port_ null: the request is only checked (a refusal, or completed when it would run).
Result TargetRiscvDm::dispatch(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  Ch32Dm *const target = port_ ? &port_->dm : nullptr;
  Tail tail;
  switch (op) {
    case kOpDmi: return dmi(p, n, out, capacity);
    case kOpHalt:     // [TLV] -> status(u8); already halted = ok, nothing done
    case kOpResume: { // [TLV] -> status(u8); ok = the hart left debug mode once (re-issue rules in Ch32Dm::resume)
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!target) return completed();
      Ch32Dm &dm = *target;
      if (capacity < 1) return failed();
      const bool ok = op == kOpHalt ? dm.halt() : dm.resume();
      out[0] = ok ? kStatusOk : failure(op == kOpHalt ? kStatusTimeout : kStatusState);
      return outcome(out[0], 0, 1);
    }
    case kOpReset: {
      // mode(u8: 0 run, 1 run + confirm, 2 stop before the first instruction) [TLV]  ->  status(u8) flags(u8: bit0
      // reached, bit1 verified) pc(u32) (mode 2: dpc). Always ndmreset (oep-if-debug §4.3); how long it waits for the
      // hart and whether it redoes the sequence is this probe's (Ch32Dm::reset).
      const Result parsed = plainTail(tail, p, n, 1, out, capacity);
      if (refused(parsed)) return parsed;
      // modes 0 and 2 only (oep-if-debug §4.3): any other unsupported, the fixed part's 0x00
      if (p[0] != kResetRun && p[0] != kResetHalt) return unsupportedValue(out, capacity);
      if (!target) return completed();
      Ch32Dm &dm = *target;
      if (capacity < 6) return failed();
      DmiPhy::WakeScope wakes(dm.phy());   // inside reset the wire's wake may go out (oep-if-debug §2)
      bool ok;
      if (p[0] == kResetHalt) {   // flags bit0 = halted, pc = dpc; haltreq lowered afterwards, the hart stays halted
        uint32_t dpc = 0;
        ok = dm.resetHalt(dpc);
        dm.lowerHaltreq();
        out[1] = ok ? reg::target_riscv_dm::kResetFlagsReached : 0;
        putU32(out + 2, dpc);
      } else {
        const Ch32Dm::ResetReport r = dm.reset(p[0] == kResetRunConfirm);
        // bit0 and bit1 only: the others are reserved (0)
        out[1] = r.flags & (reg::target_riscv_dm::kResetFlagsReached | legacy::kResetFlagsVerified);
        putU32(out + 2, r.pc);
        ok = p[0] == kResetRunConfirm ? (r.flags & legacy::kResetFlagsVerified) != 0
                                      : (r.flags & reg::target_riscv_dm::kResetFlagsReached) != 0;
      }
      dm.phy().loss().excuseReset();   // the reset asserted: the target may not answer for a while (oep-if-debug §2)
      ++port_->resets;   // the console marks it (detail 1 ndmreset)
      port_->reset_detail = reg::target_console::kMarkDetailResetNdmreset;
      // not reached: the module answers - timeout, or fault for a cmderr; it does not - line (oep-if-debug §4.3)
      out[0] = ok ? kStatusOk : dm.lastCmderr() ? kStatusFault : failure(kStatusTimeout);
      return outcome(out[0], 0, 6);
    }
    case kOpStep: {
      // [TLV] -> status(u8) moved(u8) dpc_before(u32) dpc_after(u32) [TLV 0x01 step_left]; one resume only, prv kept.
      // Not back in debug mode within limits::kDmWaitMs: status state, moved and both dpc 0 - with step_left (length 0)
      // when it still runs (oep-if-debug §4.2).
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!target) return completed();
      Ch32Dm &dm = *target;
      if (capacity < 10 + kTlvHeader) return failed();
      uint32_t before = 0, after = 0;
      bool moved = false, left = false;
      uint8_t status = kStatusState;   // not halted: nothing to step
      if (dm.checkHalted() && dm.halted()) {
        Ch32Dm::StepEnd end = Ch32Dm::kStepped;
        const bool ok = dm.step(before, after, moved, end);
        // an unmoved dpc is not a failure: a self jump (j .) truly steps to itself; the host reads the instruction
        if (!ok) status = dm.lastCmderr() ? kStatusFault : failure(kStatusTimeout);
        else status = end == Ch32Dm::kStepped ? kStatusOk : kStatusState;
        left = ok && end == Ch32Dm::kLeftRunning;
      } else {
        status = failure(kStatusState);   // not halted (the module answers), or no answer at all: line
      }
      // moved, dpc_before and dpc_after mean something with status ok only: 0 otherwise (oep-if-debug §4.2)
      const bool stepped = status == kStatusOk;
      out[0] = status;
      out[1] = stepped && moved;
      putU32(out + 2, stepped ? before : 0);
      putU32(out + 6, stepped ? after : 0);
      size_t length = 10;
      if (left) {
        putTlvHeader(out + 10, reg::target_riscv_dm::kTlvStepAnswerStepLeft, 0);
        length = 10 + kTlvHeader;
      }
      return outcome(status, 0, length);
    }
    case kOpReadBlock: {   // address(u32) count(u16) [TLV]  ->  done(u16) status(u8) words (done of them) [TLV]
      const Result parsed = plainTail(tail, p, n, 6, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      if (address & 3) return rejected(kRejectMalformed);
      // count x 4 over the declared max_length: unsupported, payload 0x00 (oep-if-debug §4.5); the answer's room too
      if (!blockCountFits(count, maxLength()) || 3u + 4u * count > capacity) return unsupportedValue(out, capacity);
      if (!target) return completed();
      Ch32Dm &dm = *target;
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
      } else {
        status = failure(kStatusState);   // not halted (the module answers), or no answer at all: line
      }
      putU16(out, done);
      out[2] = status;
      for (uint16_t i = 0; i < done; ++i) putU32(out + 3 + 4 * i, words_[i]);
      return outcome(status, done, 3u + 4u * done);
    }
    case kOpWriteBlock: {   // address(u32) count(u16) count x word [TLV]  ->  done(u16) status(u8) [TLV]
      if (n < 6) return rejected(kRejectMalformed);
      const uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      const Result parsed = plainTail(tail, p, n, 6u + 4u * count, out, capacity);
      if (refused(parsed)) return parsed;
      if (address & 3) return rejected(kRejectMalformed);
      if (!blockCountFits(count, maxLength())) return unsupportedValue(out, capacity);   // over max_length (oep-if-debug §4.5)
      if (!target) return completed();
      Ch32Dm &dm = *target;
      if (capacity < 3) return failed();
      uint8_t status = kStatusState;
      uint16_t done = 0;
      if (count == 0) {
        status = kStatusOk;   // nothing to write: success, done 0
      } else if (dm.checkHalted() && dm.halted()) {
        for (size_t i = 0; i < count; ++i) words_[i] = getU32(p + 6 + 4 * i);
        // done: the words stored, in order (a failure may come after some of them; none is stored twice)
        size_t written = 0;
        if (dm.writeWordsFast(address, words_, count, &written)) {
          status = kStatusOk;
          done = count;
        } else {
          status = dm.lastCmderr() ? kStatusFault : failure(kStatusFault);
          done = static_cast<uint16_t>(written);
        }
      } else {
        status = failure(kStatusState);   // not halted (the module answers), or no answer at all: line
      }
      putU16(out, done);
      out[2] = status;
      return outcome(status, done, 3);
    }
    case kOpRun: {
      // pc(u32) timeout_ms(u32: up to max_op_ms) n(u8) n x (regno u16, value u32) n_out(u8) n_out x regno(u16) [TLV]
      //   ->  status(u8) stopped(u8: 0 timed out and halted, 1 stopped by itself, 2 could not be halted, 3 not run)
      //       dpc(u32) elapsed_us(u32) nvals(u8) nvals x value(u32) [TLV]
      if (n < 9) return rejected(kRejectMalformed);
      const uint8_t regs = p[8];
      const size_t outs_at = 9u + 6u * regs;
      if (n < outs_at + 1) return rejected(kRejectMalformed);
      const uint8_t outs = p[outs_at];
      const Result parsed = plainTail(tail, p, n, outs_at + 1 + 2u * outs, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t timeout_ms = getU32(p + 4);
      if (11u + 4u * outs > capacity) return rejected(kRejectMalformed);   // the answer would not fit a frame
      if (timeout_ms > kMaxOpMs || regs > kMaxRegs || outs > kMaxRegs) return unsupportedValue(out, capacity);
      if (!target) return completed();
      Ch32Dm &dm = *target;
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
        if (r.not_run) {   // the preparation failed: not run, the hart still halted (oep-if-debug §4.4, stopped 3)
          status = failure(kStatusFault);   // no answer from the wire: line; a cmderr of its commands: fault
          out[1] = reg::target_riscv_dm::kRunStoppedNotRun;
        } else if (!r.halted) {   // the limit passed and the hart could not be stopped: dpc and values invalid
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
      } else {
        status = failure(kStatusState);   // not halted (the module answers), or no answer at all: line
      }
      out[0] = status;
      out[10] = nvals;
      return outcome(status, 0, 11u + 4u * nvals);
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
//
// The link held over the request (the 2026-10-06 debug-link proposal, P4): a look before the steps and one after them
// (dmLinkLook: DMSTATUS a module's with authenticated, DMCONTROL with bit 7 clear), and no revive of the PHY's between
// them (DmiPhy::revives). A link that drops (a CH32L103's, after a change of hart state) loses writes and reads the last
// value read or all ones with nothing in the answer to say so; a drop stays until the link is brought up again, so a
// look that passes after the steps says every step met the link up - unless the PHY brought it up again behind them
// (its revive after a rest: a wait step, or a gap), which only the count tells. A request that is not seen held answers
// status line with no values (none of them is known to be the register's) and done = the steps completed before the
// check that failed: 0 when the look before them fails, all of them (or up to a timeout's step) when the look after
// them or the count does - their writes may have been done. A step that fails on the line answers line with no values
// too. Before this the request's values were returned as read, and a revive between the host's own looks (RiscvDm.held
// in oep-client-python) hid the drop from them.
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
      // an unknown step kind: its arguments' length is not guessed; checking stops there and the whole request is
      // unsupported, the fixed part's 0x00, nothing before it run (oep-if-debug §4.1)
      default: return unsupportedValue(out, capacity);
    }
    if (at + size > length) return rejected(kRejectMalformed);
    at += size;
  }
  Tail tail;
  const Result parsed = tail.parse(p + at, length - at, out, capacity);
  if (refused(parsed)) return parsed;
  if (5 + 4 * values > capacity) return rejected(kRejectMalformed);   // the answer would not fit a frame
  if (wait_us > static_cast<uint64_t>(kMaxOpMs) * 1000u) return unsupportedValue(out, capacity);
  if (!port_) return completed();   // checked only (handle)
  Ch32Dm &dm = port_->dm;
  WireLossClock &loss = dm.phy().loss();
  // One read of a step: a DMSTATUS of all zeros / all ones is no answer (no module behind it, DmiPhy::outcomeOf) and
  // fails the step with line like a read that got nothing - the same reads the wire-loss clock counts as no answer
  // (oep-if-debug §2: the failures it counts are status line). Any other register's value is the register's.
  auto readStep = [&](uint8_t address, uint32_t &v) {
    return dm.readDmi(address, v) && DmiPhy::outcomeOf(address, true, v) != DmiPhy::kNoAnswer;
  };
  size_t written = 5;
  uint16_t done = 0, nvals = 0;
  uint8_t status = kStatusOk;
  // The whole request within max_op_ms of time (oep-if-debug §4.1): a request that reaches it ends at the step running
  // then (or the next to start) with status timeout, done = that step's index - a poll that read adds its last value.
  const uint32_t began_us = micros();
  constexpr uint32_t kMaxOpUs = kMaxOpMs * 1000u;
  auto over = [&]() { return micros() - began_us >= kMaxOpUs; };
  // the look before and after the steps (P4, above): its first read may be where the PHY revives a rested link
  auto look = [&]() {
    uint32_t status_value = 0, control = 0;
    return readStep(kDmStatus, status_value) && dm.readDmi(kDmControl, control) && dmLinkLook(status_value, control);
  };
  if (!look()) status = kStatusLine;
  const uint32_t revives = dm.phy().revives();
  at = 2;
  for (uint16_t i = 0; i < count && status == kStatusOk; ++i) {
    const uint8_t kind = p[at];
    const uint8_t *s = p + at + 1;
    const size_t step_written = written;
    const uint16_t step_nvals = nvals;
    if (over()) { status = kStatusTimeout; break; }
    switch (kind) {
      case kStepWrite:
        if (!dm.writeDmi(s[0], getU32(s + 1))) status = kStatusLine;
        at += 6;
        break;
      case kStepRead: {
        uint32_t v = 0;
        if (!readStep(s[0], v)) status = kStatusLine;
        else { putU32(out + written, v); written += 4; ++nvals; }
        at += 2;
        break;
      }
      case kStepPoll: {
        const uint32_t mask = getU32(s + 1), want = getU32(s + 5);
        const uint16_t max = getU16(s + 9);
        bool met = false;
        uint32_t last = 0;
        for (uint16_t k = 0; k < (max ? max : 1) && !met && status == kStatusOk && !(k && over()); ++k) {   // max 0: one read
          uint32_t v = 0;
          if (!readStep(s[0], v)) status = kStatusLine;
          else { last = v; met = (v & mask) == want; }
        }
        if (status == kStatusOk && !met) status = kStatusTimeout;
        if (status != kStatusLine) { putU32(out + written, last); written += 4; ++nvals; }   // met or timed out, not cut off
        at += 12;
        break;
      }
      case kStepDelay: {
        const uint32_t left = kMaxOpUs - (micros() - began_us);   // over() was false: some is left
        const uint32_t us = getU32(s);
        delayMicroseconds(us < left ? us : left);
        if (us >= left) status = kStatusTimeout;
        at += 5;
        break;
      }
      case kStepPollTime: {
        const uint32_t mask = getU32(s + 1), want = getU32(s + 5), max_us = getU32(s + 9);
        bool met = false;
        uint32_t last = 0;
        const uint32_t started = micros();
        do {
          uint32_t v = 0;
          if (!readStep(s[0], v)) status = kStatusLine;
          else { last = v; met = (v & mask) == want; }
        } while (!met && status == kStatusOk && micros() - started < max_us && !over());
        if (status == kStatusOk && !met) status = kStatusTimeout;
        if (status != kStatusLine) { putU32(out + written, last); written += 4; ++nvals; }
        at += 14;
        break;
      }
    }
    // Wire loss decided at this step (oep-if-debug §2: no good exchange for limits::kWireLostMs, a register's own all zeros /
    // all ones read meanwhile): the step fails with line, its value out, and the connection closes after the answer.
    if (status != kStatusLine && loss.lost()) {
      status = kStatusLine;
      written = step_written;
      nvals = step_nvals;
      line_lost_ = true;
    }
    if (status == kStatusOk) ++done;
  }
  // after the steps: the look, and no revive since the look before them (a timeout's request too: its values stand)
  if (status != kStatusLine && !line_lost_ && !(look() && dm.phy().revives() == revives)) status = kStatusLine;
  if (status == kStatusLine) {   // no values: none is known to be the register's
    written = 5;
    nvals = 0;
  }
  if (status == kStatusLine && !line_lost_) failure(kStatusLine);   // the connection closes after this answer once lost
  putU16(out, done);
  out[2] = status;
  putU16(out + 3, nvals);
  return outcome(status, done, written);
}

}  // namespace oep
