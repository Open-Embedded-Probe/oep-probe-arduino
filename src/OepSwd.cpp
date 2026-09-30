// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepSwd.h"

#if defined(ARDUINO_ARCH_RP2040)

#include "OepSwdFrame.h"

namespace oep {
namespace {

constexpr uint8_t kKindArmAdi = 0x02;
constexpr int kWaitRetries = 100;
inline bool refused(const Result &r) { return r.resolution != kResolutionCompleted; }

// oep-if-common §3 status from the last ACK: WAIT to the end = wait, FAULT = fault, no answer or parity = line.
uint8_t statusOf(uint8_t ack) {
  return ack == swd::kOk ? kStatusOk : ack == swd::kFault ? kStatusFault : ack == swd::kWait ? kStatusWait : kStatusLine;
}

uint32_t hzOf(uint32_t half_ns) { return half_ns ? static_cast<uint32_t>(500000000u / half_ns) : 0; }

size_t describePins(const SwdPort &port, uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  if (port.pin_choice) {                // any free pair of these (oep-if-debug §1)
    static const uint8_t kPair[] = {reg::wire_swd::kPinRoleSwdio, reg::wire_swd::kPinRoleSwclk};
    w.roleChannels(kPair, 2, port.pin_choice);
  } else {
    w.pinGroup(port.swdio, port.swclk); // fixed on this probe
  }
  w.u8(kTagImplementation, 1);          // bit-bang
  return w.ok() ? w.length() : 0;
}

}  // namespace

// ---- oep.wire.swd --------------------------------------------------------------------------------

size_t WireSwd::describe(uint8_t *out, size_t capacity) { return describePins(port_, out, capacity); }

bool WireSwd::allowed(uint16_t swdio, uint16_t swclk) const {
  if (!port_.pin_choice) return swdio == port_.swdio && swclk == port_.swclk;
  return swdio < 64 && swclk < 64 && swdio != swclk && ((port_.pin_choice >> swdio) & 1) && ((port_.pin_choice >> swclk) & 1);
}

bool WireSwd::free(uint16_t swdio, uint16_t swclk) const {
  if (!port_.pin_choice || !port_.pins) return true;   // a fixed pair is the wire's own (kept out of the pin table)
  const bool mine = port_.connected && swdio == port_.swdio && swclk == port_.swclk;
  auto ok = [&](uint16_t c) { const uint8_t o = port_.pins->owner(c); return o == 0 || (mine && o == port_.pin_owner); };
  return ok(swdio) && ok(swclk);
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

void WireSwd::close() {
  port_.io.releaseBoth();
  port_.connected = false;
  if (port_.pin_choice && port_.pins) port_.pins->releaseQuiet(port_.pin_owner);
}

bool WireSwd::xferDpidr(uint32_t &dpidr) {
  uint32_t id = 0;
  if (swd::transfer(port_.io, false, true, 0x0, id) != swd::kOk || id == 0 || id == 0xffffffffu) return false;
  dpidr = id;
  return true;
}

// Wake the port and read DPIDR: the legacy JTAG-to-SWD select first, then the dormant wake (an SWD v2 port with
// dormant support - the RP2350's - only answers the latter). A multidrop port needs TARGETSEL after each line reset.
bool WireSwd::wake(const uint32_t *targetsel, uint32_t half_ns, uint32_t &dpidr, bool &dormant) {
  port_.io.setup(port_.swdio, port_.swclk);
  port_.io.setHalfNs(half_ns);
  port_.io.driveBoth();
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
  port_.io.releaseBoth();
  return false;
}

Result WireSwd::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  static const uint8_t kAttachTags[] = {reg::wire_swd::kTlvAttachMaxSpeed, reg::wire_swd::kTlvAttachTargetsel,
                                         reg::wire_swd::kTlvAttachPins};
  Tail tail;
  switch (op) {
    case kOpScan: {
      // count(u8) pairs [TLV 0x01 skip]  ->  tried(u8) count(u8), then kind(u8) swdio(u16) swclk(u16) DPIDR(u32) per
      // answer (oep-if-debug §1)
      if (length < 1 || length < 1u + 4u * payload[0]) return rejected(kRejectMalformed);
      const uint8_t count = payload[0];
      const size_t fixed = 1u + 4u * count;
      static const uint8_t kScanTags[] = {reg::wire_swd::kTlvScanSkip};
      const Result parsed = tail.parse(payload + fixed, length - fixed, kScanTags, out, capacity);
      if (refused(parsed)) return parsed;
      uint16_t skip = 0;
      {
        uint8_t len = 0;
        if (const uint8_t *v = tail.find(reg::wire_swd::kTlvScanSkip, len)) {
          if (count || len != 2) return rejected(kRejectMalformed);   // skip goes with count 0 only
          skip = getU16(v);
        }
      }
      for (uint8_t k = 0; k < count; ++k) {   // allowed, free, and - the one seat taken - the live pair
        const uint16_t d = getU16(payload + 1 + 4 * k), c = getU16(payload + 3 + 4 * k);
        if (!allowed(d, c) || !free(d, c) || (port_.connected && (d != port_.swdio || c != port_.swclk)))
          return rejected(kRejectUnavailable);
      }
      if (capacity < 2) return failed();
      size_t at = 2;
      uint8_t tried = 0, found = 0;
      auto tryPair = [&](uint16_t d, uint16_t c) {   // false: the answer is full
        if (at + 9 > capacity) return false;
        uint32_t dpidr = 0;
        bool ok = false;
        if (port_.connected) {   // look through the live connection: waking the port again would reset its DP state
          ok = xferDpidr(dpidr);
        } else if (move(d, c)) {
          bool dormant = false;
          ok = wake(nullptr, port_.half_ns, dpidr, dormant);
          port_.io.releaseBoth();
        }
        if (ok) {
          out[at] = kKindArmAdi;
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
        if (skip == 0) tryPair(port_.swdio, port_.swclk);
      } else {                                            // swdio ascending, then swclk; held pairs left out
        uint32_t index = 0;
        bool more = true;
        for (uint16_t d = 0; d < 64; ++d)
          for (uint16_t c = 0; c < 64; ++c)
            if (more && tried < 255 && allowed(d, c) && free(d, c) && index++ >= skip) more = tryPair(d, c);
      }
      out[0] = tried;
      out[1] = found;
      return tail.finish(completed(at), out, capacity);
    }
    case kOpAttach: {
      // [TLV 0x01 max_speed u32 Hz, 0x02 targetsel u32]
      //   ->  connection(u16) DPIDR(u32) flags(u8: bit0 woke from dormant, bit1 existing connection) speed_hz(u32)
      const Result parsed = tail.parse(payload, length, kAttachTags, out, capacity);
      if (refused(parsed)) return parsed;
      {
        uint8_t plen = 0;
        const uint8_t *pins = tail.find(reg::wire_swd::kTlvAttachPins, plen);
        if (!pins) {
          if (port_.pin_choice) return rejected(kRejectUnavailable);   // the host names the pair
        } else {
          if (plen != 4) return rejected(kRejectMalformed);
          const uint16_t d = getU16(pins), c = getU16(pins + 2);
          if (!allowed(d, c) || !free(d, c)) return rejected(kRejectUnavailable);
          if (port_.connected && (d != port_.swdio || c != port_.swclk)) return rejected(kRejectUnavailable);   // no seat
          if (!move(d, c)) return rejected(kRejectUnavailable);
        }
      }
      if (port_.exhausted()) return rejected(kRejectUnavailable);
      if (capacity < 11) return failed();
      uint8_t len = 0;
      bool critical = false;
      uint32_t max_hz = 0, targetsel = 0;
      bool have_targetsel = false;
      if (const uint8_t *v = tail.find(reg::wire_swd::kTlvAttachMaxSpeed, len, &critical)) {
        if (len != 4 || getU32(v) == 0) return rejected(kRejectMalformed);
        max_hz = getU32(v);
      }
      if (const uint8_t *v = tail.find(reg::wire_swd::kTlvAttachTargetsel, len)) {
        if (len != 4) return rejected(kRejectMalformed);
        targetsel = getU32(v);
        have_targetsel = true;
      }
      // The slowest half period that keeps SWCLK at or under the ceiling (nominal: the loop overhead only slows it).
      uint32_t half = port_.half_ns;
      if (max_hz && hzOf(half) > max_hz) half = static_cast<uint32_t>((500000000ull + max_hz - 1) / max_hz);
      uint32_t dpidr = 0;
      uint8_t flags = 0;
      bool ok;   // a failed attach answers its status alone: line (no answer from the port)
      if (port_.connected &&
          (have_targetsel != port_.active_targetsel || (have_targetsel && targetsel != port_.targetsel)))
        return rejected(kRejectUnavailable);   // another target on these pins: the host detaches first
      if (port_.connected) {
        // Already attached: the same connection, nothing redone. A ceiling the live link is over cannot be met
        // without attaching again.
        if (max_hz && hzOf(port_.active_half_ns) > max_hz) {
          const Result r = tail.refuse(reg::wire_swd::kTlvAttachMaxSpeed, critical, out, capacity);
          if (refused(r)) return r;
        }
        flags |= 2;
        ok = xferDpidr(dpidr);
      } else {
        bool dormant = false;
        ok = wake(have_targetsel ? &targetsel : nullptr, half, dpidr, dormant);
        if (ok) {
          port_.connected = true;
          port_.numberNew();
          if (port_.pin_choice && port_.pins) {   // the live connection holds its pins (core §8.1)
            port_.pins->claim(port_.swdio, port_.pin_owner);
            port_.pins->claim(port_.swclk, port_.pin_owner);
          }
          port_.active_half_ns = half;
          port_.active_targetsel = have_targetsel;
          port_.targetsel = targetsel;
          if (dormant) flags |= 1;
        }
      }
      if (!ok) return tail.finish(failedStatus(kStatusLine, out, capacity), out, capacity);
      putU16(out, port_.number);
      putU32(out + 2, dpidr);
      out[6] = flags;
      putU32(out + 7, hzOf(port_.active_half_ns));
      return tail.finish(completed(11), out, capacity);
    }
    case kOpConnections: {   // [TLV] -> count(u8), the live connection (users: the host only; no target_id scheme)
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 15) return failed();
      out[0] = port_.connected ? 1 : 0;
      if (!port_.connected) return tail.finish(completed(1), out, capacity);
      putU16(out + 1, port_.number);
      putU16(out + 3, port_.swdio);
      putU16(out + 5, port_.swclk);
      putU32(out + 7, port_.active_half_ns ? 500000000u / port_.active_half_ns : 0);
      out[11] = reg::wire_swd::kConnectionUsersHostSession;
      out[12] = 0xff;   // no slot
      out[13] = out[14] = 0;
      return tail.finish(completed(15), out, capacity);
    }
    case kOpDetach: {   // connection(u16) [TLV]
      if (length < 2) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 2, length - 2, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU16(payload) != port_.number || !port_.connected) return rejected(kRejectNoConnection);
      close();
      return tail.finish(completed(), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

// ---- oep.target.arm-adi --------------------------------------------------------------------------

size_t TargetArmAdi::describe(uint8_t *out, size_t capacity) { return describePins(port_, out, capacity); }

uint8_t TargetArmAdi::xfer(bool ap, bool read, uint8_t a23, uint32_t &data) {
  uint8_t ack = swd::kNoReply;
  for (int i = 0; i <= kWaitRetries; ++i) {
    ack = swd::transfer(port_.io, ap, read, a23, data);
    if (ack != swd::kWait) break;
  }
  return ack;
}

Result TargetArmAdi::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 2) return rejected(kRejectMalformed);
  if (getU16(payload) != port_.number || !port_.connected) return rejected(kRejectNoConnection);
  const uint8_t *p = payload + 2;
  const size_t n = length - 2;
  Tail tail;
  switch (op) {
    case kOpTransfer: {
      // n(u16), then n transfers: req(u8: bit0 APnDP, bit1 RnW, bits 2-3 A[3:2]) [+ value(u32) for a write] [TLV]
      //   ->  done(u16) status(u8) ack(u8: the last raw ACK), then the value of every completed read, in order.
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
      if (4 + 4 * reads > capacity) return rejected(kRejectMalformed);   // the answer would not fit a frame
      size_t o = 4;
      uint16_t done = 0;
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
        if (read) { putU32(out + o, value); o += 4; }
        ++done;
      }
      putU16(out, done);
      out[2] = status;
      out[3] = ack;
      return tail.finish(outcome(status, done, o), out, capacity);
    }
    case kOpReadBlock: {
      // address(u32) count(u16) [TLV] words through the current MEM-AP (the host has set SELECT to the bank holding
      // TAR / DRW and CSW to 32-bit, single increment). TAR is written again at each 1 KiB boundary, where its
      // auto-increment may stop.  ->  done(u16) status(u8) words (done of them)
      const Result parsed = plainTail(tail, p, n, 6, out, capacity);
      if (refused(parsed)) return parsed;
      uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      if (address & 3 || 3 + size_t(count) * 4 > capacity) return rejected(kRejectMalformed);
      size_t o = 3;
      uint16_t left = count;
      uint8_t ack = swd::kOk;
      while (left && ack == swd::kOk) {
        const uint16_t chunk = uint16_t(min<uint32_t>(left, (0x400 - (address & 0x3ff)) / 4));
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
      return tail.finish(outcome(status, done, o), out, capacity);
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
      if (capacity < 3) return failed();
      size_t index = 0;
      uint16_t done = 0;
      uint8_t ack = swd::kOk;
      bool faulted = false;
      while (index < count && ack == swd::kOk) {
        const size_t chunk = min<size_t>(count - index, (0x400 - (address & 0x3ff)) / 4);
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
      return tail.finish(outcome(status, done, 3), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
