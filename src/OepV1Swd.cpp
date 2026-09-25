#include "OepV1Swd.h"

#if defined(ARDUINO_ARCH_RP2040)

#include "OepSwdFrame.h"

namespace oep {
namespace v1 {
namespace {

constexpr uint8_t kKindArmAdi = 0x02;
constexpr int kWaitRetries = 100;
inline bool refused(const Result &r) { return r.resolution != kResolutionCompleted; }

// §5.4 status from the last ACK: WAIT to the end = wait, FAULT = fault, no answer or parity = line.
uint8_t statusOf(uint8_t ack) {
  return ack == swd::kOk ? kStatusOk : ack == swd::kFault ? kStatusFault : ack == swd::kWait ? kStatusWait : kStatusLine;
}

uint32_t hzOf(uint32_t half_ns) { return half_ns ? static_cast<uint32_t>(500000000u / half_ns) : 0; }

size_t describePins(const SwdPort &port, uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.pinGroup(port.swdio, port.swclk);   // fixed on this probe
  w.u8(kTagImplementation, 1);          // bit-bang
  return w.ok() ? w.length() : 0;
}

}  // namespace

// ---- oep.wire.swd --------------------------------------------------------------------------------

size_t WireSwd::describe(uint8_t *out, size_t capacity) { return describePins(port_, out, capacity); }

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
  static const uint8_t kAttachTags[] = {reg::wire_swd::kTlvAttachMaxSpeed, reg::wire_swd::kTlvAttachTargetsel};
  Tail tail;
  switch (op) {
    case kOpScan: {   // [TLV]  ->  count(u8), then kind(u8) swdio(u16) swclk(u16) DPIDR(u32) per answer
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 10) return failed();
      uint32_t dpidr = 0;
      bool dormant = false;
      out[0] = 0;
      bool found;
      if (port_.connected) {   // look through the live connection: waking the port again would reset its DP state
        found = xferDpidr(dpidr);
      } else {
        found = wake(nullptr, port_.half_ns, dpidr, dormant);
        port_.io.releaseBoth();
      }
      if (!found) return tail.finish(completed(1), out, capacity);
      out[0] = 1;
      out[1] = kKindArmAdi;
      putU16(out + 2, port_.swdio);
      putU16(out + 4, port_.swclk);
      putU32(out + 6, dpidr);
      return tail.finish(completed(10), out, capacity);
    }
    case kOpAttach: {
      // [TLV 0x01 max_speed u32 Hz, 0x02 targetsel u32]
      //   ->  connection(u8) DPIDR(u32) flags(u8: bit0 woke from dormant, bit1 existing connection) speed_hz(u32)
      const Result parsed = tail.parse(payload, length, kAttachTags, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 10) return failed();
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
          port_.active_half_ns = half;
          if (dormant) flags |= 1;
        }
      }
      if (!ok) return tail.finish(failedStatus(kStatusLine, out, capacity), out, capacity);
      out[0] = 1;
      putU32(out + 1, dpidr);
      out[5] = flags;
      putU32(out + 6, hzOf(port_.active_half_ns));
      return tail.finish(completed(10), out, capacity);
    }
    case kOpDetach: {   // connection(u8) [TLV]
      if (length < 1) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 1, length - 1, out, capacity);
      if (refused(parsed)) return parsed;
      if (payload[0] != 1 || !port_.connected) return rejected(kRejectNoConnection);
      port_.io.releaseBoth();
      port_.connected = false;
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
  if (length < 1) return rejected(kRejectMalformed);
  if (payload[0] != 1 || !port_.connected) return rejected(kRejectNoConnection);
  const uint8_t *p = payload + 1;
  const size_t n = length - 1;
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

}  // namespace v1
}  // namespace oep

#endif
