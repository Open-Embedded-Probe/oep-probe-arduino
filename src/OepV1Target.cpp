#include "OepV1Target.h"

namespace oep {
namespace v1 {
namespace {

constexpr uint8_t kDmStatus = 0x11;
constexpr uint8_t kKindRiscvDm = 0x01;
enum : uint8_t { kStepOk = 0, kStepMalformed = 1, kStepAccess = 2, kStepPollGaveUp = 3 };

}  // namespace

// ---- oep.wire.rvswd ------------------------------------------------------------------------------

size_t WireRvswd::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  uint8_t group[7] = {1, 1, 0, 0, 2, 0, 0};   // pin set 1: role 1 SWDIO, role 2 SWCLK (fixed on this probe)
  putU16(group + 2, port_.swdio);
  putU16(group + 5, port_.swclk);
  w.put(kTagChannelGroup, group, sizeof group);
  w.u8(kTagImplementation, 1);                // bit-bang
  return w.ok() ? w.length() : 0;
}

Result WireRvswd::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (op) {
    case kOpScan: {   // -> count(u8), then kind(u8) swdio(u16) swclk(u16) raw DMSTATUS(u32) per answer
      if (length != 0 || capacity < 10) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      uint32_t status = 0;
      out[0] = 0;
      if (port_.dm.attach() && port_.dm.readDmi(kDmStatus, status) && status != 0 && status != 0xffffffffu) {
        out[0] = 1;
        out[1] = kKindRiscvDm;
        putU16(out + 2, port_.swdio);
        putU16(out + 4, port_.swclk);
        putU32(out + 6, status);
        return completed(10);
      }
      return completed(1);
    }
    case kOpAttach: {   // method(u8): 0 leave it running, 1 halt  ->  connection(u8), DMSTATUS(u32)
      if (length != 1 || payload[0] > 1 || capacity < 5) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      uint32_t status = 0;
      if (!port_.dm.attach() || !port_.dm.readDmi(kDmStatus, status)) return failed();
      if (payload[0] == 1 && !port_.dm.halt()) return failed();
      port_.connected = true;
      out[0] = 1;
      putU32(out + 1, status);
      return completed(5);
    }
    case kOpDetach:
      if (length != 1 || payload[0] != 1) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      port_.dm.detach();
      port_.connected = false;
      return completed();
    default:
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

// ---- oep.target.riscv-dm -------------------------------------------------------------------------

size_t TargetRiscvDm::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  // bit0 block read/write (progbuf + autoexec), bit1 run until halt, bit2 ndmreset
  w.u32(kTagFeatures, 0b0111);
  w.u8(kTagImplementation, 1);
  w.u16(kTagMaxLength, sizeof words_);
  return w.ok() ? w.length() : 0;
}

Result TargetRiscvDm::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 1) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
  if (payload[0] != 1 || !port_.connected) return rejected(OEP_V0_REJECT_UNAVAILABLE);   // connection 1 only
  const uint8_t *p = payload + 1;
  const size_t n = length - 1;
  Ch32Dm &dm = port_.dm;
  switch (op) {
    case kOpDmi: return dmi(p, n, out, capacity);
    case kOpHalt: return dm.halt() ? completed() : failed();
    case kOpResume: return dm.resume() ? completed() : failed();
    case kOpReset: {   // confirm(u8)  ->  flags(u8) attempts(u8) pc(u32), as the v0 reset report
      if (n != 1 || capacity < 6) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      const Ch32Dm::ResetReport r = dm.reset(p[0] == 1);
      ++port_.resets;
      out[0] = r.flags;
      out[1] = r.attempts;
      putU32(out + 2, r.pc);
      const bool ok = p[0] == 1 ? (r.flags & 2) != 0 : (r.flags & 1) != 0;
      return ok ? completed(6) : failed(6);
    }
    case kOpReadBlock: {   // address(u32) count(u16)  ->  words
      if (n != 6) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      const uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      if ((address & 3) || !count || count > sizeof words_ / 4 || 4u * count > capacity)
        return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      if (!dm.readWords(address, words_, count)) return failed();
      for (uint16_t i = 0; i < count; ++i) putU32(out + 4 * i, words_[i]);
      return completed(4u * count);
    }
    case kOpWriteBlock: {   // address(u32) words
      if (n < 8 || (n - 4) % 4) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      const uint32_t address = getU32(p);
      const size_t count = (n - 4) / 4;
      if ((address & 3) || count > sizeof words_ / 4) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      for (size_t i = 0; i < count; ++i) words_[i] = getU32(p + 4 + 4 * i);
      return dm.writeWordsFast(address, words_, count) ? completed() : failed();
    }
    case kOpRun: {   // pc(u32) timeout_ms(u16) n(u8) n x (regno u16, value u32)  ->  stopped dpc a0 elapsed_us
      if (n < 7 || n != 7u + 6u * p[6] || p[6] > 16 || capacity < 13) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
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
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

// Steps run in order and stop at the first failure:
//   0x01 write  address(u8) value(u32)
//   0x02 read   address(u8)                                 (the value is appended to the result)
//   0x03 poll   address(u8) mask(u32) value(u32) max(u16)   ((read & mask) == value within max reads)
// result: done(u16) status(u8: 0 ok, 1 malformed, 2 access, 3 poll gave up) reads
Result TargetRiscvDm::dmi(const uint8_t *p, size_t length, uint8_t *out, size_t capacity) {
  if (capacity < 3) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
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
