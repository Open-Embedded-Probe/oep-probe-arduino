#include "OepV1Swd.h"

#if defined(ARDUINO_ARCH_RP2040)

#include "OepSwdFrame.h"

namespace oep {
namespace v1 {
namespace {

constexpr uint8_t kKindArmAdi = 0x02;
constexpr int kWaitRetries = 100;
enum : uint8_t { kStatusOk = 0, kStatusMalformed = 1, kStatusFault = 2, kStatusNoReply = 3, kStatusWait = 4 };

uint8_t statusOf(uint8_t ack) {
  return ack == swd::kOk ? kStatusOk : ack == swd::kFault ? kStatusFault : ack == swd::kWait ? kStatusWait : kStatusNoReply;
}

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
bool WireSwd::wake(const uint32_t *targetsel, uint32_t &dpidr, bool &dormant) {
  port_.io.setup(port_.swdio, port_.swclk);
  port_.io.setHalfNs(port_.half_ns);
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
  switch (op) {
    case kOpScan: {   // -> count(u8), then kind(u8) swdio(u16) swclk(u16) DPIDR(u32) per answer
      if (length != 0 || capacity < 10) return rejected(kRejectMalformed);
      uint32_t dpidr = 0;
      bool dormant = false;
      out[0] = 0;
      bool found;
      if (port_.connected) {   // look through the live connection: waking the port again would reset its DP state
        found = xferDpidr(dpidr);
      } else {
        found = wake(nullptr, dpidr, dormant);
        port_.io.releaseBoth();
      }
      if (!found) return completed(1);
      out[0] = 1;
      out[1] = kKindArmAdi;
      putU16(out + 2, port_.swdio);
      putU16(out + 4, port_.swclk);
      putU32(out + 6, dpidr);
      return completed(10);
    }
    case kOpAttach: {   // [TARGETSEL(u32)]  ->  connection(u8), DPIDR(u32), flags(u8: bit0 woke from dormant)
      if ((length != 0 && length != 4) || capacity < 6) return rejected(kRejectMalformed);
      uint32_t targetsel = length == 4 ? getU32(payload) : 0;
      uint32_t dpidr = 0;
      bool dormant = false;
      if (!wake(length == 4 ? &targetsel : nullptr, dpidr, dormant)) return failed();
      port_.connected = true;
      out[0] = 1;
      putU32(out + 1, dpidr);
      out[5] = dormant ? 1 : 0;
      return completed(6);
    }
    case kOpDetach: {
      if (length != 1 || payload[0] != 1) return rejected(kRejectMalformed);
      port_.io.releaseBoth();
      port_.connected = false;
      return completed();
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
  if (length < 1 || payload[0] != 1) return rejected(kRejectMalformed);
  if (!port_.connected) return failed();
  const uint8_t *p = payload + 1;
  size_t n = length - 1;
  switch (op) {
    case kOpTransfer: {
      // steps: req(u8: bit0 APnDP, bit1 RnW, bits 2-3 A[3:2]) [+ value(u32) for a write]
      // -> done(u16), status(u8: 0 ok / 1 malformed / 2 FAULT / 3 no reply or parity / 4 WAIT gave up), ack(u8),
      //    then the value of every read, in order. AP reads are posted: the host reads RDBUFF or the next AP read.
      if (capacity < 4) return rejected(kRejectMalformed);
      size_t at = 0, o = 4;
      uint16_t done = 0;
      uint8_t status = kStatusOk, ack = swd::kOk;
      while (at < n) {
        const uint8_t req = p[at++];
        const bool ap = req & 1, read = req & 2;
        const uint8_t a23 = (req >> 2) & 3;
        if (req & 0xf0) { status = kStatusMalformed; break; }
        uint32_t value = 0;
        if (!read) {
          if (at + 4 > n) { status = kStatusMalformed; break; }
          value = getU32(p + at);
          at += 4;
        } else if (o + 4 > capacity) { status = kStatusMalformed; break; }
        ack = xfer(ap, read, a23, value);
        if (ack != swd::kOk) { status = statusOf(ack); break; }
        if (read) { putU32(out + o, value); o += 4; }
        ++done;
      }
      putU16(out, done);
      out[2] = status;
      out[3] = ack;
      return completed(o);
    }
    case kOpReadBlock: {
      // address(u32), count(u16) words through the current MEM-AP (the host has set SELECT to the bank holding
      // TAR / DRW and CSW to 32-bit, single increment). TAR is written again at each 1 KiB boundary, where its
      // auto-increment may stop. -> the words
      if (n != 6) return rejected(kRejectMalformed);
      uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      if (address & 3 || size_t(count) * 4 > capacity) return rejected(kRejectMalformed);
      size_t o = 0;
      uint16_t left = count;
      while (left) {
        const uint16_t chunk = uint16_t(min<uint32_t>(left, (0x400 - (address & 0x3ff)) / 4));
        uint32_t v = address;
        if (xfer(true, false, 0x1, v) != swd::kOk) return failed(o);             // TAR
        if (xfer(true, true, 0x3, v) != swd::kOk) return failed(o);              // DRW: posted, first value is stale
        for (uint16_t i = 1; i < chunk; ++i) {
          if (xfer(true, true, 0x3, v) != swd::kOk) return failed(o);
          putU32(out + o, v); o += 4;
        }
        if (xfer(false, true, 0x3, v) != swd::kOk) return failed(o);             // DP RDBUFF: the last one
        putU32(out + o, v); o += 4;
        address += uint32_t(chunk) * 4;
        left -= chunk;
      }
      return completed(o);
    }
    case kOpWriteBlock: {   // address(u32), then words
      if (n < 4 || (n - 4) % 4) return rejected(kRejectMalformed);
      uint32_t address = getU32(p);
      if (address & 3) return rejected(kRejectMalformed);
      size_t at = 4;
      while (at < n) {
        const size_t chunk = min<size_t>((n - at) / 4, (0x400 - (address & 0x3ff)) / 4);
        uint32_t v = address;
        if (xfer(true, false, 0x1, v) != swd::kOk) return failed();
        for (size_t i = 0; i < chunk; ++i) {
          v = getU32(p + at);
          at += 4;
          if (xfer(true, false, 0x3, v) != swd::kOk) return failed();
        }
        address += uint32_t(chunk) * 4;
      }
      uint32_t v = 0;
      if (xfer(false, true, 0x3, v) != swd::kOk) return failed();   // RDBUFF: the last write has landed
      return completed();
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace v1
}  // namespace oep

#endif
