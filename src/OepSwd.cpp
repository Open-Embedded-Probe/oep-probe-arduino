// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepSwd.h"

#if defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_SWD)

#include "OepSwdFrame.h"

namespace oep {
namespace {

namespace sw = reg::wire_swd;
constexpr int kWaitRetries = limits::kSwdWaitRetries;

// oep-if-common §3 status from the last ACK: WAIT to the end = wait, FAULT = fault, no answer or parity = line.
uint8_t statusOf(uint8_t ack) {
  return ack == swd::kOk ? kStatusOk : ack == swd::kFault ? kStatusFault : ack == swd::kWait ? kStatusWait : kStatusLine;
}

// A request's outcome on the wire-loss clock (oep-if-debug §2): any answer (OK, WAIT, FAULT) stops it, status line
// starts it. true: the wire is now lost - no good exchange for limits::kWireLostMs of real time - and the connection closes
// once the answer is out. One request that got nothing back within its retries keeps the connection.
bool lostAfter(SwdPort &port, uint8_t status) {
  if (status != kStatusLine) { port.loss.answered(); return false; }
  port.loss.silent();
  return port.loss.lost();
}

uint32_t hzOf(uint32_t half_ns) { return half_ns ? static_cast<uint32_t>(500000000u / half_ns) : 0; }
constexpr uint32_t kSlowHalfNs = 50000;   // the slowest SWCLK this probe uses (10 kHz): a scan without max_speed, min_clock_hz

// The lines while the wire does not answer (oep-if-debug §2, §5): on a live connection, from an exchange that got no
// answer until one answers, the lines rest free between exchanges - released, SWDIO without the pull-up the setup gave
// it (its rest level is low), SWCLK with no pull - and are driven only during an exchange (a packet, the line reset,
// the JTAG-to-SWD switch, the dormant wake, each with its 8 idle cycles, which every one of them clocks before the
// lines are let go). An answer brings the rest state back at once: SWDIO driven low, SWCLK high (the levels the idle
// cycles leave latched).
void exchangeBegins(SwdPort &port) {
  if (!port.connected || !port.rest_free) return;
  gpio_pull_up(port.swdio);   // during the exchange: a released SWDIO reads high (no ACK) rather than floating
  port.io.driveBoth();
}
void exchangeEnds(SwdPort &port, bool answered, bool sent_only = false) {
  if (!port.connected) return;
  if (answered) { port.rest_free = false; return; }
  if (sent_only && !port.rest_free) return;   // a sequence with no answer of its own, on a healthy link
  port.rest_free = true;
  port.io.releaseBoth();
  gpio_disable_pulls(port.swdio);
  gpio_disable_pulls(port.swclk);
}

// The link back after a transfer that got nothing back (oep-if-debug §5: in the retries of §2 the line reset and the
// wake from dormant are redone): the JTAG-to-SWD switch with its line resets, then the dormant wake when that brings no
// answer, each followed by TARGETSEL on a multidrop connection and the DPIDR read a DP needs after a line reset before
// it takes another request. true: DPIDR answered. The DP's other registers (SELECT, CTRL/STAT) survive a line reset.
bool relink(SwdPort &port) {
  for (int how = 0; how < 2; ++how) {
    exchangeBegins(port);
    if (how == 0) swd::jtagToSwd(port.io); else swd::dormantToSwd(port.io);
    exchangeEnds(port, false, true);
    if (port.active_targetsel) {
      exchangeBegins(port);
      swd::targetSelect(port.io, port.targetsel);
      exchangeEnds(port, false, true);
    }
    uint32_t id = 0;
    exchangeBegins(port);
    const bool up = swd::transfer(port.io, false, true, 0x0, id) == swd::kOk && id != 0 && id != 0xffffffffu;
    exchangeEnds(port, up);
    if (up) return true;
  }
  return false;
}

// About how long one round of relink() and the transfer again takes, before one has been measured: the two wakes
// (136 + 208 cells), TARGETSEL and DPIDR twice and the transfer (46 cells each), two half periods a cell.
uint32_t roundUs(const SwdPort &port) {
  const uint32_t half = port.active_half_ns ? port.active_half_ns : port.half_ns;
  return static_cast<uint32_t>((uint64_t{344 + 5 * 46} * 2u * half + 999) / 1000);
}

// A transfer with the WAIT retries (kWaitRetries) and the request's wire retries (oep-if-debug §2, §5): a transfer that
// got nothing back - no ACK, or a read's data parity - is tried again after relink() while one more round still ends
// inside the request's limits::kWireRetryMs. An AP read with a bad parity is not repeated: the target took it (TAR moved, the
// posted value changed), and a second read would return another word. Returns the last ACK.
uint8_t transferRetried(SwdPort &port, bool ap, bool read, uint8_t a23, uint32_t &data, WireRetry &retry) {
  int waits = 0;
  bool retrying = false;
  uint32_t t0 = 0, cost = roundUs(port);
  for (;;) {
    bool bad_parity = false;
    exchangeBegins(port);
    const uint8_t ack = swd::transfer(port.io, ap, read, a23, data, &bad_parity);
    exchangeEnds(port, ack != swd::kNoReply);   // OK, WAIT, FAULT: an answer
    if (retrying) {   // the round: relink() and this transfer
      cost = micros() - t0;
      retry.spent_us += cost;
      retrying = false;
    }
    if (ack == swd::kWait && waits++ < kWaitRetries) continue;
    if (ack != swd::kNoReply || (bad_parity && ap && read) || !retry.fits(cost)) return ack;
    t0 = micros();
    retrying = true;
    relink(port);   // whatever it got, the transfer again says how it went
  }
}

size_t describePins(const SwdPort &port, uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  if (port.pin_choice) {                // any free pair of these (oep-if-debug §1)
    static const uint8_t kPair[] = {reg::wire_swd::kPinRoleSwdio, reg::wire_swd::kPinRoleSwclk};
    w.roleChannels(kPair, 2, port.pin_choice);
  } else {
    w.pinGroup(port.swdio, port.swclk); // fixed on this probe
  }
  w.u32(kTagMaxClockHz, hzOf(port.half_ns));
  w.u32(kTagMinClockHz, hzOf(kSlowHalfNs));
  return w.ok() ? w.length() : 0;
}

}  // namespace

// ---- oep.wire.swd --------------------------------------------------------------------------------

size_t WireSwd::describe(uint8_t *out, size_t capacity) { return describePins(port_, out, capacity); }

bool WireSwd::allowed(uint16_t swdio, uint16_t swclk) const {
  if (!port_.pin_choice) return swdio == port_.swdio && swclk == port_.swclk;
  return swdio < 64 && swclk < 64 && swdio != swclk && ((port_.pin_choice >> swdio) & 1) && ((port_.pin_choice >> swclk) & 1);
}

uint16_t WireSwd::disabledOf(uint16_t swdio, uint16_t swclk) const {
  if (!port_.pins) return 0xffff;
  return port_.pins->disabled(swdio) ? swdio : port_.pins->disabled(swclk) ? swclk : 0xffff;
}

uint16_t WireSwd::idleOf(uint16_t swdio, uint16_t swclk, bool outputs) const {
  if (!port_.pins) return 0xffff;
  auto has = [&](uint16_t c) {
    const uint8_t mode = port_.pins->idle(c);
    return outputs ? mode == PinTable::kIdleOutputLow || mode == PinTable::kIdleOutputHigh : mode != PinTable::kIdleUnset;
  };
  return has(swdio) ? swdio : has(swclk) ? swclk : 0xffff;
}

bool WireSwd::free(uint16_t swdio, uint16_t swclk) const {
  if (disabledOf(swdio, swclk) != 0xffff) return false;   // the settings disable it (probe.config §1): never used
  if (!port_.pin_choice || !port_.pins) return true;   // a fixed pair is the wire's own (kept out of the pin table)
  const bool mine = port_.connected && swdio == port_.swdio && swclk == port_.swclk;
  auto ok = [&](uint16_t c) { const uint8_t o = port_.pins->owner(c); return o == 0 || (mine && o == port_.pin_owner); };
  return ok(swdio) && ok(swclk);
}

Result WireSwd::heldRefusal(uint16_t swdio, uint16_t swclk, uint8_t *out, size_t capacity) const {
  uint16_t held = 0xffff;
  if (port_.pins) {
    auto taken = [&](uint16_t c) { const uint8_t o = port_.pins->owner(c); return o != 0 && o != port_.pin_owner; };
    held = taken(swdio) ? swdio : taken(swclk) ? swclk : 0xffff;
  }
  if (held == 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse);
  return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, held);
}

bool WireSwd::move(uint16_t swdio, uint16_t swclk) {
  if (swdio == port_.swdio && swclk == port_.swclk) return true;
  if (port_.connected || !port_.pin_choice || !allowed(swdio, swclk) || !free(swdio, swclk)) return false;
  port_.io.releaseBoth();   // the old pair back to plain Hi-Z inputs (setup pulled SWDIO up)
  gpio_disable_pulls(port_.swdio);
  gpio_disable_pulls(port_.swclk);
  port_.swdio = swdio;
  port_.swclk = swclk;
  return true;              // wake() sets the new pair up
}

// Nothing holds the pair (the connection closed, a scan's try, a failed attach): Hi-Z without the pull-up setup gave
// SWDIO, then a channel with an idle set goes to it (oep-core §8). wake() sets the pins up again.
static void freePort(SwdPort &port) {
  // the 8 idle cycles before the lines are let go (oep-if-debug §5): the target completes the last packet. Every
  // packet and sequence here ends with them already; clocked again only while the lines are driven.
  if (port.io_driven && !port.rest_free) {
    port.io.hostDrives(true);
    swd::idle(port.io, 8);
  }
  port.io_driven = false;
  port.rest_free = false;
  port.io.releaseBoth();
  gpio_disable_pulls(port.swdio);
  gpio_disable_pulls(port.swclk);
  if (!port.pins) return;
  if (port.pin_choice && !port.connected) port.pins->releaseQuiet(port.pin_owner);
  port.pins->rest(port.swdio);
  port.pins->rest(port.swclk);
}

// The live connection goes: pins to their free state, its number closed, let go of in the pin table.
void closePort(SwdPort &port) {
  if (!port.connected) return;
  port.connected = false;
  ResourceNumbers::close(port.number);
  freePort(port);
}

void WireSwd::close() { closePort(port_); }

bool WireSwd::xferDpidr(uint32_t &dpidr) {
  uint32_t id = 0;
  WireRetry retry;   // this request's wire retries (oep-if-debug §2)
  if (transferRetried(port_, false, true, 0x0, id, retry) != swd::kOk || id == 0 || id == 0xffffffffu) return false;
  dpidr = id;
  return true;
}

// Wake the port and read DPIDR: the legacy JTAG-to-SWD select first, then the dormant wake (an SWD v2 port with
// dormant support - the RP2350's - only answers the latter). A multidrop port needs TARGETSEL after each line reset.
bool WireSwd::wake(const uint32_t *targetsel, uint32_t half_ns, uint32_t &dpidr, bool &dormant) {
  port_.io.setup(port_.swdio, port_.swclk);
  port_.io.setHalfNs(half_ns);
  port_.io.driveBoth();
  port_.io_driven = true;
  port_.rest_free = false;
  for (int how = 0; how < 2; ++how) {
    if (how == 0) swd::jtagToSwd(port_.io); else swd::dormantToSwd(port_.io);
    if (targetsel) swd::targetSelect(port_.io, *targetsel);
    uint32_t id = 0;
    if (swd::transfer(port_.io, false, true, 0x0, id) == swd::kOk && id != 0 && id != 0xffffffffu) {
      dpidr = id;
      dormant = how == 1;
      return true;
    }
  }
  port_.io.releaseBoth();   // after the last packet's idle cycles
  port_.io_driven = false;
  return false;
}

Result WireSwd::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // the reset TLV (0x05) is not in the known list: critical, it is rejected unsupported with its tag (oep-if-debug §5)
  static const uint8_t kAttachTags[] = {sw::kTlvAttachMaxSpeed, sw::kTlvAttachTargetsel, sw::kTlvAttachPins};
  Tail tail;
  switch (op) {
    case kOpScan: {
      // count(u8) pairs [TLV 0x01 max_speed, 0x02 skip, 0x06 targetsel]  ->  tried(u8) count(u8), count x (kind(u8)
      // swdio(u16) swclk(u16) DPIDR(u32)) (oep-if-debug §1, §5; 9 bytes each, no element length)
      if (length < 1 || length < 1u + 4u * payload[0]) return rejected(kRejectMalformed);
      const uint8_t count = payload[0];
      const size_t fixed = 1u + 4u * count;
      static const uint8_t kScanTags[] = {sw::kTlvScanMaxSpeed, sw::kTlvScanSkip, sw::kTlvScanTargetsel};
      Result unknown = completed();   // an unknown critical tag: unsupported once the format is checked
      const Result parsed = tail.parse(payload + fixed, length - fixed, kScanTags, out, capacity, &unknown);
      if (refused(parsed)) return parsed;
      uint16_t skip = 0;
      uint32_t max_hz = 0, scan_targetsel = 0;
      bool scan_select = false;
      {
        // each TLV's one form (core §2.3: another length is malformed, critical or not)
        const uint8_t *v = nullptr;
        Result r = tail.fixed(sw::kTlvScanSkip, 2, v, out, capacity);
        if (refused(r)) return r;
        if (v && count == 0) skip = getU16(v);   // a count > 0 scan does not look at skip (oep-if-debug §1)
        r = tail.fixed(sw::kTlvScanMaxSpeed, 4, v, out, capacity);
        if (refused(r)) return r;
        if (v && getU32(v) == 0) return rejected(kRejectMalformed);
        if (v) max_hz = getU32(v);
        r = tail.fixed(sw::kTlvScanTargetsel, 4, v, out, capacity);
        if (refused(r)) return r;
        if (v) { scan_targetsel = getU32(v); scan_select = true; }
      }
      if (refused(unknown)) return unknown;
      // every pair listed allowed (oep-if-debug §1: unsupported, tag 0x00 and TLV 0x40 its index); every check before
      // anything is done on the wire (core §4.3)
      for (uint8_t k = 0; k < count; ++k)
        if (!allowed(getU16(payload + 1 + 4 * k), getU16(payload + 3 + 4 * k))) return unsupportedIndex(out, capacity, k);
      for (uint8_t k = 0; k < count; ++k) {   // free, and - the one seat taken - the live pair
        const uint16_t d = getU16(payload + 1 + 4 * k), c = getU16(payload + 3 + 4 * k);
        const uint16_t off = disabledOf(d, c);   // cause 5 with the channel (probe.config §1)
        if (off != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, off);
        const uint16_t idle = idleOf(d, c, true);   // an output idle: cause 5 (oep-if-debug §1)
        if (idle != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, idle);
        if (port_.connected && (d != port_.swdio || c != port_.swclk))   // the one seat taken (debug §1): a count limit
          return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
        if (!free(d, c)) return heldRefusal(d, c, out, capacity);   // cause 1, the channel (core §4.3)
      }
      if (capacity < 2) return failed();
      // the scan's clock: the slowest this probe goes when no ceiling is given (oep-if-debug §3), else under max_speed
      uint32_t half = kSlowHalfNs;
      if (max_hz && hzOf(half) > max_hz) half = static_cast<uint32_t>((500000000ull + max_hz - 1) / max_hz);
      if (half < port_.half_ns) half = port_.half_ns;
      size_t at = 2;
      uint8_t tried = 0, found = 0;
      // No pair starts later than limits::kScanBudgetMs after the request (oep-if-debug §1): the host goes on with the rest
      // (count 0 with skip, or the pairs after tried). 26 free pins bit-banged pair by pair kept an RP2350 from answering
      // for seconds (0.0.18). One try is a line reset and a dormant wake at most: well inside the attach budget.
      const uint32_t began = millis();
      auto tryPair = [&](uint16_t d, uint16_t c) {   // false: the answer is full, or its time is up
        if (at + 9 > capacity || (tried && millis() - began >= limits::kScanBudgetMs)) return false;
        uint32_t dpidr = 0;
        bool ok = false;
        if (port_.connected) {   // look through the live connection: waking the port again would reset its DP state
          ok = xferDpidr(dpidr);
          // a scan is a request on the connection: it runs the wire-loss clock, and closes it once lost (§2)
          if (lostAfter(port_, ok ? kStatusOk : kStatusLine)) close();
        } else if (move(d, c)) {
          bool dormant = false;
          ok = wake(scan_select ? &scan_targetsel : nullptr, half, dpidr, dormant);
          freePort(port_);   // found or not, the pair to its free state (oep-if-debug §1)
        }
        if (ok) {
          out[at] = sw::kScanKindArmAdi;
          putU16(out + at + 1, d);
          putU16(out + at + 3, c);
          putU32(out + at + 5, dpidr);
          at += 9;
          ++found;
        }
        ++tried;
        return true;
      };
      if (count) {
        for (uint8_t k = 0; k < count && tryPair(getU16(payload + 1 + 4 * k), getU16(payload + 3 + 4 * k)); ) ++k;
      } else if (port_.connected || !port_.pin_choice) {   // the count-0 list: the live pair, or the fixed one
        // a fixed pair with an idle item on a channel is not in the list (oep-if-debug §1); the live pair is
        if (skip == 0 && disabledOf(port_.swdio, port_.swclk) == 0xffff &&
            (port_.connected || idleOf(port_.swdio, port_.swclk, false) == 0xffff))
          tryPair(port_.swdio, port_.swclk);
      } else {                                            // swdio ascending, then swclk; held, disabled, idle items left out
        uint32_t index = 0;
        bool more = true;
        for (uint16_t d = 0; d < 64; ++d)
          for (uint16_t c = 0; c < 64; ++c)
            if (more && tried < 255 && allowed(d, c) && free(d, c) && idleOf(d, c, false) == 0xffff && index++ >= skip)
              more = tryPair(d, c);
      }
      out[0] = tried;
      out[1] = found;
      return completed(at);
    }
    case kOpAttach: {
      // method(u8: 0 only) [TLV 0x01 max_speed u32 Hz (required), 0x02 targetsel u32, 0x03 pins]
      //   ->  connection(u16) DPIDR(u32) flags(u8: bit1 existing connection, bit2 woke from dormant) speed_hz(u32) [TLV]
      if (length < 1) return rejected(kRejectMalformed);
      // every format check before anything this probe does not handle: an unknown critical tag (the reset TLV among
      // them) is held back until then
      Result unknown = completed();
      const Result parsed = tail.parse(payload + 1, length - 1, kAttachTags, out, capacity, &unknown);
      if (refused(parsed)) return parsed;
      bool critical = false;
      uint32_t max_hz = 0, targetsel = 0;
      bool have_targetsel = false;
      bool pins_critical = false;
      const uint8_t *pins = nullptr;
      {
        // each TLV's one form (core §2.3: another length is malformed, critical or not)
        const uint8_t *v = nullptr;
        Result r = tail.fixed(sw::kTlvAttachMaxSpeed, 4, v, out, capacity, &critical);
        if (refused(r)) return r;
        if (v && getU32(v) == 0) return rejected(kRejectMalformed);
        if (v) max_hz = getU32(v);
        if (!max_hz) return rejected(kRejectMalformed);   // required (oep-if-debug §1)
        r = tail.fixed(sw::kTlvAttachTargetsel, 4, v, out, capacity);
        if (refused(r)) return r;
        if (v) { targetsel = getU32(v); have_targetsel = true; }
        r = tail.fixed(sw::kTlvAttachPins, 4, pins, out, capacity, &pins_critical);
        if (refused(r)) return r;
      }
      if (refused(unknown)) return unknown;
      if (payload[0] != 0) return unsupportedValue(out, capacity);   // 0 only: arm-adi has no halt (oep-if-debug §5)
      if (max_hz < hzOf(kSlowHalfNs)) return unsupportedTag(out, capacity, sw::kTlvAttachMaxSpeed | (critical ? kTagCritical : 0));
      // a pair this wire does not declare: unsupported with the tag as received, critical or not (core §2.3)
      if (pins && !allowed(getU16(pins), getU16(pins + 2))) return Tail::refuse(sw::kTlvAttachPins, pins_critical, out, capacity);
      {
        if (!pins) {   // the live connection's pair (join it), the fixed pair, else the host names one
          if (port_.pin_choice && !port_.connected) return unavailable(out, capacity, reg::core::kUnavailableCauseWrongState);
          const uint16_t off = port_.connected ? 0xffff : disabledOf(port_.swdio, port_.swclk);
          if (off != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, off);
          // the fixed pair with an idle item is not a candidate (oep-if-debug §1): none left
          const uint16_t idle = port_.connected ? 0xffff : idleOf(port_.swdio, port_.swclk, false);
          if (idle != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, idle);
        } else {
          const uint16_t d = getU16(pins), c = getU16(pins + 2);
          const uint16_t off = disabledOf(d, c);   // cause 5 with the channel (probe.config §1)
          if (off != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, off);
          const uint16_t idle = idleOf(d, c, true);   // an output idle: cause 5
          if (idle != 0xffff) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, idle);
          if (port_.connected && (d != port_.swdio || c != port_.swclk))   // no seat (max_connections 1): a count limit
            return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
          if (!free(d, c)) return heldRefusal(d, c, out, capacity);   // cause 1, the channel (core §4.3)
          if (!move(d, c)) return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse);
        }
      }
      if (capacity < 11) return failed();
      // The slowest half period that keeps SWCLK at or under the ceiling (nominal: the loop overhead only slows it).
      uint32_t half = port_.half_ns;
      if (hzOf(half) > max_hz) half = static_cast<uint32_t>((500000000ull + max_hz - 1) / max_hz);
      uint32_t dpidr = 0, search_retries = 0;
      uint8_t flags = 0;
      bool searched = false;
      bool ok;   // a failed attach answers its status alone: line (no answer from the port)
      if (port_.connected &&
          (have_targetsel != port_.active_targetsel || (have_targetsel && targetsel != port_.targetsel)))
        return wrongState(out, capacity);   // another target on these pins: the host detaches first
      if (port_.connected) {
        // Already attached: the same connection, nothing redone - a live link faster than this max_speed is slowed to
        // it and returned (oep-if-debug §1; going slower is safe). It refused the ceiling.
        if (hzOf(port_.active_half_ns) > max_hz) {
          port_.active_half_ns = half;
          port_.io.setHalfNs(half);
          searched = true;   // its speed lowered: search_retries in the answer (§1), the DPIDR read below the check
        }
        flags |= sw::kAttachFlagsExisting;
        ok = xferDpidr(dpidr);
        // nothing back: status line; the connection closes after the answer only once the wire is lost (§2)
        if (lostAfter(port_, ok ? kStatusOk : kStatusLine)) close();
      } else {
        // The wake, tried again while the wire retries of oep-if-debug §2 last (each try the line reset, the dormant
        // wake, TARGETSEL: §5), a try started only when it still ends inside limits::kWireRetryMs - well inside the attach
        // budget. Each failed try is one of the answer's search_retries.
        bool dormant = false;
        WireRetry retry;
        uint32_t t0 = micros();
        ok = wake(have_targetsel ? &targetsel : nullptr, half, dpidr, dormant);
        uint32_t cost = micros() - t0;
        while (!ok && retry.fits(cost)) {
          ++search_retries;
          t0 = micros();
          ok = wake(have_targetsel ? &targetsel : nullptr, half, dpidr, dormant);
          cost = micros() - t0;
          retry.spent_us += cost;
        }
        searched = true;
        if (!ok) freePort(port_);   // a failed attach holds nothing: the pair goes free
        if (ok) {
          const uint16_t number = ResourceNumbers::take(ResourceNumbers::kConnection);
          if (!number) { freePort(port_); return unavailable(out, capacity, reg::core::kUnavailableCauseLimit); }
          port_.connected = true;
          port_.number = number;
          port_.loss.clear();   // a new connection: its own wire-loss clock (oep-if-debug §2)
          if (port_.pin_choice && port_.pins) {   // the live connection holds its pins (core §8.1)
            port_.pins->claim(port_.swdio, port_.pin_owner);
            port_.pins->claim(port_.swclk, port_.pin_owner);
          }
          port_.active_half_ns = half;
          port_.active_targetsel = have_targetsel;
          port_.targetsel = targetsel;
          if (dormant) flags |= sw::kAttachFlagsDormantWoken;
        }
      }
      if (!ok) return failedStatus(kStatusLine, out, capacity);
      putU16(out, port_.number);
      putU32(out + 2, dpidr);
      out[6] = flags;
      putU32(out + 7, hzOf(port_.active_half_ns));
      size_t n = 11;
      if (searched && capacity >= n + kTlvHeader + 2) {   // search_retries (oep-if-debug §1): the wakes that failed, 0xFFFF = more
        putTlvHeader(out + n, sw::kTlvAttachAnswerSearchRetries, 2);
        putU16(out + n + kTlvHeader, static_cast<uint16_t>(search_retries < 0xffff ? search_retries : 0xffff));
        n += kTlvHeader + 2;
      }
      return completed(n);
    }
    case kOpConnections: {   // first(u8) [TLV] -> more(u8) count(u8), the live connection (users: the host only; tid scheme 2 = TARGETSEL)
      const Result parsed = plainTail(tail, payload, length, 1, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 20) return failed();
      out[0] = 0;   // more: never (one entry at most)
      out[1] = port_.connected && payload[0] == 0 ? 1 : 0;
      if (!out[1]) return completed(2);
      putU16(out + 2, port_.number);   // count x entry, no element length (core §2.3)
      putU16(out + 4, port_.swdio);
      putU16(out + 6, port_.swclk);
      putU32(out + 8, port_.active_half_ns ? 500000000u / port_.active_half_ns : 0);
      out[12] = sw::kConnectionUsersHostSession;
      out[13] = 0xff;   // no slot
      out[14] = reg::common::kTargetIdSchemeTargetsel;
      out[15] = 4;   // tid_len: the TARGETSEL value is a u32
      putU32(out + 16, port_.active_targetsel ? port_.targetsel : 0);
      return completed(20);
    }
    case kOpDetach: {   // connection(u16) [TLV 0x01 force]
      static const uint8_t kDetachTags[] = {sw::kTlvDetachForce};
      if (length < 2) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 2, length - 2, kDetachTags, out, capacity);
      if (refused(parsed)) return parsed;
      const uint8_t *force = nullptr;   // length 0 (oep-if-debug §3); longer: a longer request TLV (core §2.3)
      const Result r = tail.fixed(sw::kTlvDetachForce, 0, force, out, capacity);
      if (refused(r)) return r;
      if (!port_.connected || getU16(payload) != port_.number)
        return ResourceNumbers::refuse(getU16(payload), ResourceNumbers::kConnection, out, capacity);
      close();   // the host is this connection's only user: its detach closes it (oep-if-debug §5)
      return completed();
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

// ---- oep.target.arm-adi --------------------------------------------------------------------------

size_t TargetArmAdi::describe(uint8_t *out, size_t capacity) {
  const size_t n = describePins(port_, out, capacity);
  if (!n) return 0;
  // One block op's bytes (oep-if-debug §6): read_block's answer and write_block's request both fit the frame
  TlvWriter w(out + n, capacity - n);
  w.u16(kTagMaxLength, maxLength());
  return w.ok() ? n + w.length() : 0;
}

uint8_t TargetArmAdi::xfer(bool ap, bool read, uint8_t a23, uint32_t &data) {
  return transferRetried(port_, ap, read, a23, data, retry_);
}

Result TargetArmAdi::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 2) return rejected(kRejectMalformed);
  // The request's form and values first, nothing run, then the connection it names (core §4.3: every check before
  // anything runs).
  checking_ = true;
  const Result checked = run(op, payload + 2, length - 2, out, capacity);
  checking_ = false;
  if (refused(checked)) return checked;
  if (!port_.connected || getU16(payload) != port_.number)
    return ResourceNumbers::refuse(getU16(payload), ResourceNumbers::kConnection, out, capacity);
  retry_ = WireRetry();   // the request's allowance for wire retries (oep-if-debug §2)
  return run(op, payload + 2, length - 2, out, capacity);
}

// One request after its connection (checking_: only checked - a refusal, or completed when it would run).
Result TargetArmAdi::run(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case kOpTransfer: {
      // n(u16), then n transfers: req(u8: bit0 APnDP, bit1 RnW, bits 2-3 A[3:2], bits 4-7 0) [+ value(u32) for a write]
      // [TLV]  ->  done(u16) status(u8) ack(u8: the last raw ACK, line order) nvals(u16) nvals x value(u32) [TLV]
      // AP reads are posted: the host reads RDBUFF or the next AP read. WAIT is retried here; FAULT stops the list.
      if (n < 2) return rejected(kRejectMalformed);
      const uint16_t count = getU16(p);
      size_t at = 2, reads = 0;
      for (uint16_t i = 0; i < count; ++i) {   // the whole list is checked before any of it runs
        if (at >= n || (p[at] & 0xf0)) return rejected(kRejectMalformed);
        const bool read = p[at] & 2;
        at += read ? 1 : 5;
        if (at > n) return rejected(kRejectMalformed);
        if (read) ++reads;
      }
      const Result parsed = tail.parse(p + at, n - at, out, capacity);
      if (refused(parsed)) return parsed;
      if (6 + 4 * reads > capacity) return rejected(kRejectMalformed);   // the answer would not fit a frame
      if (checking_) return completed();
      size_t o = 6;
      uint16_t done = 0, nvals = 0;
      uint8_t status = kStatusOk, ack = swd::kOk;
      at = 2;
      for (uint16_t i = 0; i < count; ++i) {
        const uint8_t req = p[at++];
        const bool ap = req & 1, read = req & 2;
        const uint8_t a23 = (req >> 2) & 3;
        uint32_t value = 0;
        if (!read) { value = getU32(p + at); at += 4; }
        ack = xfer(ap, read, a23, value);
        if (ack != swd::kOk) { status = statusOf(ack); break; }
        if (read) { putU32(out + o, value); o += 4; ++nvals; }
        ++done;
      }
      putU16(out, done);
      out[2] = status;
      out[3] = ack;
      putU16(out + 4, nvals);
      if (lostAfter(port_, status)) closePort(port_);   // only once the wire is lost (oep-if-debug §2)
      return outcome(status, done, o);
    }
    case kOpReadBlock: {
      // address(u32) count(u16) [TLV] words through the current MEM-AP (the host has set SELECT to the bank holding
      // TAR / DRW and CSW to 32-bit, single increment). TAR is written again at each limits::kTarRewriteBytes boundary,
      // where its auto-increment may stop (ADI).  ->  done(u16) status(u8) words (done of them)
      const Result parsed = plainTail(tail, p, n, 6, out, capacity);
      if (refused(parsed)) return parsed;
      uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      if (address & 3) return rejected(kRejectMalformed);
      // count x 4 over the declared max_length: unsupported, payload 0x00 (oep-if-debug §6); the answer's room too
      if (!blockCountFits(count, maxLength()) || 3 + size_t(count) * 4 > capacity) return unsupportedValue(out, capacity);
      if (checking_) return completed();
      if (capacity < 3) return failed();
      size_t o = 3;
      uint16_t left = count;
      uint8_t ack = swd::kOk;
      while (left && ack == swd::kOk) {
        const uint16_t chunk =
            uint16_t(min<uint32_t>(left, (limits::kTarRewriteBytes - (address & (limits::kTarRewriteBytes - 1))) / 4));
        uint32_t v = address;
        if ((ack = xfer(true, false, 0x1, v)) != swd::kOk) break;             // TAR
        if ((ack = xfer(true, true, 0x3, v)) != swd::kOk) break;              // DRW: posted, first value is stale
        for (uint16_t i = 1; i < chunk && ack == swd::kOk; ++i) {
          if ((ack = xfer(true, true, 0x3, v)) != swd::kOk) break;
          putU32(out + o, v); o += 4;
        }
        if (ack != swd::kOk) break;
        if ((ack = xfer(false, true, 0x3, v)) != swd::kOk) break;             // DP RDBUFF: the last one
        putU32(out + o, v); o += 4;
        address += uint32_t(chunk) * 4;
        left -= chunk;
      }
      const uint16_t done = static_cast<uint16_t>((o - 3) / 4);
      const uint8_t status = statusOf(ack);
      putU16(out, done);
      out[2] = status;
      if (lostAfter(port_, status)) closePort(port_);   // only once the wire is lost (oep-if-debug §2)
      return outcome(status, done, o);
    }
    case kOpWriteBlock: {
      // address(u32) count(u16) count x word [TLV]  ->  done(u16) status(u8). A write's bus error shows as FAULT on
      // the transfer after it, so a FAULT on word i means word i - 1 did not land; WAIT to the end means word i
      // was not taken.
      if (n < 6) return rejected(kRejectMalformed);
      uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      const Result parsed = plainTail(tail, p, n, 6u + 4u * count, out, capacity);
      if (refused(parsed)) return parsed;
      if (address & 3) return rejected(kRejectMalformed);
      if (!blockCountFits(count, maxLength())) return unsupportedValue(out, capacity);   // over max_length (oep-if-debug §6)
      if (checking_) return completed();
      if (capacity < 3) return failed();
      size_t index = 0;
      uint16_t done = 0;
      uint8_t ack = swd::kOk;
      bool faulted = false;
      while (index < count && ack == swd::kOk) {
        const size_t chunk =
            min<size_t>(count - index, (limits::kTarRewriteBytes - (address & (limits::kTarRewriteBytes - 1))) / 4);
        uint32_t v = address;
        if ((ack = xfer(true, false, 0x1, v)) != swd::kOk) { faulted = ack == swd::kFault; break; }   // TAR
        for (size_t i = 0; i < chunk; ++i) {
          v = getU32(p + 6 + 4 * index);
          if ((ack = xfer(true, false, 0x3, v)) != swd::kOk) { faulted = ack == swd::kFault; break; }
          ++index;
        }
        address += uint32_t(chunk) * 4;
      }
      if (ack == swd::kOk) {
        uint32_t v = 0;
        ack = xfer(false, true, 0x3, v);   // RDBUFF: the last write has landed
        faulted = ack == swd::kFault;
      }
      done = static_cast<uint16_t>(ack == swd::kOk ? index : faulted ? (index ? index - 1 : 0) : index);
      const uint8_t status = statusOf(ack);
      putU16(out, done);
      out[2] = status;
      if (lostAfter(port_, status)) closePort(port_);   // only once the wire is lost (oep-if-debug §2)
      return outcome(status, done, 3);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
