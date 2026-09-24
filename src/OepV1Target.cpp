#include "OepV1Target.h"

#include <Arduino.h>

#include "OepPlatform.h"

namespace oep {
namespace v1 {
namespace {

constexpr uint8_t kDmStatus = 0x11;

// A cold CH32 ignores the first wake now and then (the CH32L103 answered on the fifth, 2026-09-23), and one that
// sat idle past its link timeout has dropped the link: bring the bus up afresh and try again before saying no.
bool attachAndRead(Ch32Dm &dm, uint32_t &status) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (dm.attach() && dm.readDmi(kDmStatus, status) && status != 0 && status != 0xffffffffu) return true;
    dm.detach();
  }
  return false;
}
constexpr uint8_t kKindRiscvDm = 0x01;
enum : uint8_t { kStepOk = 0, kStepMalformed = 1, kStepAccess = 2, kStepPollGaveUp = 3 };

}  // namespace

// ---- oep.wire.rvswd ------------------------------------------------------------------------------

size_t WireRvswd::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.pinGroup(port_.swdio, port_.swclk);       // fixed on this probe
  w.u8(kTagImplementation, 1);                // bit-bang
  return w.ok() ? w.length() : 0;
}

Result WireRvswd::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (op) {
    case kOpScan: {   // -> count(u8), then kind(u8) swdio(u16) swclk(u16) raw DMSTATUS(u32) per answer
      if (length != 0 || capacity < 10) return rejected(kRejectMalformed);
      uint32_t status = 0;
      out[0] = 0;
      // On a live connection, look through it: re-attaching (and detaching on a miss) would pull the link out from
      // under the host that holds it.
      const bool found = port_.connected ? port_.dm.readDmi(kDmStatus, status) && status != 0 && status != 0xffffffffu
                                         : attachAndRead(port_.dm, status);
      if (found) {
        out[0] = 1;
        out[1] = kKindRiscvDm;
        putU16(out + 2, port_.swdio);
        putU16(out + 4, port_.swclk);
        putU32(out + 6, status);
        return completed(10);
      }
      return completed(1);
    }
    case kOpAttach: {   // method(u8): 0 leave it running, 1 halt  ->  connection(u8), DMSTATUS(u32), flags(u8)
      if (length != 1 || payload[0] > 1 || capacity < 6) return rejected(kRejectMalformed);
      uint32_t status = 0;
      if (!attachAndRead(port_.dm, status)) return failed();
      // A pending havereset freezes a V00x's DMSTATUS halt / run bits at their reset values (ch32rv 0.8.0):
      // acknowledge it first so the DMSTATUS returned is current, and say that it was there (flags bit0).
      const bool had_reset = port_.dm.ackHaveReset();
      if (!port_.dm.readDmi(kDmStatus, status)) return failed();
      if (payload[0] == 1 && !port_.dm.halt()) return failed();
      port_.connected = true;
      out[0] = 1;
      putU32(out + 1, status);
      out[5] = had_reset ? 1 : 0;
      return completed(6);
    }
    case kOpAttachUnderReset: {   // channel(u16, 0xffff = the probe's default) hold_ms(u16)  ->  connection(u8), dpc(u32)
      if (length != 4 || capacity < 5) return rejected(kRejectMalformed);
      int channel = getU16(payload);
      if (channel == 0xffff) channel = port_.reset_default;
      if (channel < 0 || channel > 63 || !((port_.reset_allowed >> channel) & 1)) return rejected(kRejectUnavailable);
      uint32_t dpc = 0;
      // Open drain: pull low, then release to Hi-Z - never drive a reset line high.
      auto hold = [](void *ctx) { platformGpio(*static_cast<int *>(ctx), kGpioOpenDrainLow); };
      auto release = [](void *ctx) { platformGpio(*static_cast<int *>(ctx), kGpioOpenDrainRelease); };
      const bool ok = port_.dm.attachUnderReset(hold, release, &channel, getU16(payload + 2), dpc);
      if (!ok) return failed();
      port_.connected = true;
      ++port_.resets;
      out[0] = 1;
      putU32(out + 1, dpc);
      return completed(5);
    }
    case kOpDetach:
      if (length != 1 || payload[0] != 1) return rejected(kRejectMalformed);
      port_.dm.detach();
      port_.connected = false;
      return completed();
    default:
      return rejected(kRejectUnknownOperation);
  }
}

// ---- oep.target.riscv-dm -------------------------------------------------------------------------

size_t TargetRiscvDm::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  // bit0 block read/write (progbuf + autoexec), bit1 run until halt, bit2 ndmreset
  w.u32(kTagFeatures, 0b0111);
  w.u8(kTagImplementation, 1);
  // One block operation's data: the word buffer, and what fits a frame (request header, session, connection, address).
  const size_t fits = max_frame_ > 15 ? (max_frame_ - 15) / 4 * 4 : 0;
  w.u16(kTagMaxLength, static_cast<uint16_t>(fits && fits < sizeof words_ ? fits : sizeof words_));
  return w.ok() ? w.length() : 0;
}

Result TargetRiscvDm::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 1) return rejected(kRejectMalformed);
  if (payload[0] != 1 || !port_.connected) return rejected(kRejectUnavailable);   // connection 1 only
  const uint8_t *p = payload + 1;
  const size_t n = length - 1;
  Ch32Dm &dm = port_.dm;
  switch (op) {
    case kOpDmi: return dmi(p, n, out, capacity);
    case kOpHalt: return dm.halt() ? completed() : failed();
    case kOpResume: return dm.resume() ? completed() : failed();
    case kOpReset: {   // mode(u8): 0 run, 1 run + confirm, 2 halt  ->  flags(u8) attempts(u8) pc(u32)
      if (n != 1 || p[0] > kResetHalt || capacity < 6) return rejected(kRejectMalformed);
      if (p[0] == kResetHalt) {   // stopped before the first instruction; flags bit0 = halted, pc = dpc
        uint32_t dpc = 0;
        const bool ok = dm.resetHalt(dpc);
        ++port_.resets;
        out[0] = ok ? 1 : 0;
        out[1] = 1;
        putU32(out + 2, dpc);
        return ok ? completed(6) : failed(6);
      }
      const Ch32Dm::ResetReport r = dm.reset(p[0] == kResetRunConfirm);
      ++port_.resets;
      out[0] = r.flags;
      out[1] = r.attempts;
      putU32(out + 2, r.pc);
      const bool ok = p[0] == kResetRunConfirm ? (r.flags & 2) != 0 : (r.flags & 1) != 0;
      return ok ? completed(6) : failed(6);
    }
    case kOpStep: {   // -> moved(u8) dpc_before(u32) dpc_after(u32); one resume only, the privilege level kept
      if (n != 0 || capacity < 9) return rejected(kRejectMalformed);
      if (!dm.halted()) return rejected(kRejectUnavailable);
      uint32_t before = 0, after = 0;
      bool moved = false;
      const bool ok = dm.step(before, after, moved);
      out[0] = moved;
      putU32(out + 1, before);
      putU32(out + 5, after);
      return ok ? completed(9) : failed(9);
    }
    case kOpReadBlock: {   // address(u32) count(u16)  ->  words
      if (n != 6) return rejected(kRejectMalformed);
      const uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      if ((address & 3) || !count || count > sizeof words_ / 4 || 4u * count > capacity)
        return rejected(kRejectMalformed);
      if (!dm.halted()) return rejected(kRejectUnavailable);
      if (!dm.readWords(address, words_, count)) return failed();
      for (uint16_t i = 0; i < count; ++i) putU32(out + 4 * i, words_[i]);
      return completed(4u * count);
    }
    case kOpWriteBlock: {   // address(u32) words
      if (n < 8 || (n - 4) % 4) return rejected(kRejectMalformed);
      const uint32_t address = getU32(p);
      const size_t count = (n - 4) / 4;
      if ((address & 3) || count > sizeof words_ / 4) return rejected(kRejectMalformed);
      if (!dm.halted()) return rejected(kRejectUnavailable);
      for (size_t i = 0; i < count; ++i) words_[i] = getU32(p + 4 + 4 * i);
      return dm.writeWordsFast(address, words_, count) ? completed() : failed();
    }
    case kOpRun: {   // pc(u32) timeout_ms(u16) n(u8) n x (regno u16, value u32)  ->  stopped dpc a0 elapsed_us
      if (n < 7 || n != 7u + 6u * p[6] || p[6] > 16 || capacity < 13) return rejected(kRejectMalformed);
      if (!dm.halted()) return rejected(kRejectUnavailable);
      uint16_t regnos[16];
      uint32_t values[16];
      for (uint8_t i = 0; i < p[6]; ++i) {
        regnos[i] = getU16(p + 7 + 6 * i);
        values[i] = getU32(p + 9 + 6 * i);
      }
      Ch32Dm::RunReport r;
      const bool ok = dm.runUntilHalt(getU32(p), regnos, values, p[6], uint32_t(getU16(p + 4)) * 1000u, r);
      out[0] = r.stopped;
      putU32(out + 1, r.dpc);
      putU32(out + 5, r.a0);
      putU32(out + 9, r.elapsed_us);
      return ok ? completed(13) : failed(13);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

// Steps run in order and stop at the first failure:
//   0x01 write  address(u8) value(u32)
//   0x02 read   address(u8)                                 (the value is appended to the result)
//   0x03 poll   address(u8) mask(u32) value(u32) max(u16)   ((read & mask) == value within max reads)
//   0x04 delay  us(u32)
//   0x05 poll   address(u8) mask(u32) value(u32) max_us(u32)   (the same, bounded by time: means the same on a
//                                                               slow bit-banged link and a fast one)
// result: done(u16) status(u8: 0 ok, 1 malformed, 2 access, 3 poll gave up) reads
Result TargetRiscvDm::dmi(const uint8_t *p, size_t length, uint8_t *out, size_t capacity) {
  if (capacity < 3) return rejected(kRejectMalformed);
  size_t at = 0, written = 3;
  uint16_t done = 0;
  uint8_t status = kStepOk;
  Ch32Dm &dm = port_.dm;
  while (at < length && status == kStepOk) {
    const uint8_t kind = p[at];
    const size_t left = length - at - 1;
    const uint8_t *s = p + at + 1;
    if (kind == kStepWrite && left >= 5) {
      if (!dm.writeDmi(s[0], getU32(s + 1))) status = kStepAccess;
      at += 6;
    } else if (kind == kStepRead && left >= 1) {
      uint32_t v = 0;
      if (written + 4 > capacity) status = kStepMalformed;
      else if (!dm.readDmi(s[0], v)) status = kStepAccess;
      else { putU32(out + written, v); written += 4; }
      at += 2;
    } else if (kind == kStepPoll && left >= 11) {
      const uint32_t mask = getU32(s + 1), want = getU32(s + 5);
      const uint16_t max = getU16(s + 9);
      bool met = false;
      for (uint16_t i = 0; i < max && !met && status == kStepOk; ++i) {
        uint32_t v = 0;
        if (!dm.readDmi(s[0], v)) status = kStepAccess;
        else met = (v & mask) == want;
      }
      if (status == kStepOk && !met) status = kStepPollGaveUp;
      at += 12;
    } else if (kind == kStepDelay && left >= 4) {
      delayMicroseconds(getU32(s));
      at += 5;
    } else if (kind == kStepPollTime && left >= 13) {
      const uint32_t mask = getU32(s + 1), want = getU32(s + 5), max_us = getU32(s + 9);
      bool met = false;
      const uint32_t started = micros();
      do {
        uint32_t v = 0;
        if (!dm.readDmi(s[0], v)) status = kStepAccess;
        else met = (v & mask) == want;
      } while (!met && status == kStepOk && micros() - started < max_us);
      if (status == kStepOk && !met) status = kStepPollGaveUp;
      at += 14;
    } else {
      status = kStepMalformed;
    }
    if (status == kStepOk) ++done;
  }
  putU16(out, done);
  out[2] = status;
  return status == kStepOk ? completed(written) : failed(written);
}

}  // namespace v1
}  // namespace oep
