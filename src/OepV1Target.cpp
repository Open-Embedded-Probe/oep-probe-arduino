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
inline bool refused(const Result &r) { return r.resolution != kResolutionCompleted; }

// The max_speed TLV (0x01, u32 Hz) of attach / attach_under_reset: 0 when absent. false = malformed.
bool maxSpeed(const Tail &tail, uint32_t &hz, bool &critical) {
  hz = 0;
  critical = false;
  uint8_t len = 0;
  const uint8_t *v = tail.find(reg::wire_rvswd::kTlvAttachMaxSpeed, len, &critical);
  if (!v) return true;
  if (len != 4 || getU32(v) == 0) return false;
  hz = getU32(v);
  return true;
}

}  // namespace

// ---- oep.wire.rvswd / oep.wire.swio ------------------------------------------------------------------

size_t WireRvswd::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.pinGroup(port_.swdio, port_.swclk);       // fixed on this probe
  w.u8(kTagImplementation, 1);                // bit-bang
  // Diagnostics (interface-specific tags): the link as it is now - the SWCLK rate the last speed search settled
  // on, DMI retries (parity / no answer) and transactions since boot.
  DmiPhy &phy = port_.dm.phy();
  w.u32(0x40, phy.clockHz());
  w.u32(0x41, phy.retries());
  w.u32(0x42, phy.transactions());
  return w.ok() ? w.length() : 0;
}

Result WireRvswd::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  static const uint8_t kAttachTags[] = {reg::wire_rvswd::kTlvAttachMaxSpeed};
  DmiPhy &phy = port_.dm.phy();
  Tail tail;
  switch (op) {
    case kOpScan: {   // [TLV]  ->  count(u8), then kind(u8) swdio(u16) swclk(u16) raw DMSTATUS(u32) per answer
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 10) return failed();
      uint32_t status = 0;
      out[0] = 0;
      // On a live connection, look through it: re-attaching (and detaching on a miss) would pull the link out from
      // under the host that holds it.
      const bool found = port_.connected ? port_.dm.readDmi(kDmStatus, status) && status != 0 && status != 0xffffffffu
                                         : attachAndRead(port_.dm, status);
      if (!found) return tail.finish(completed(1), out, capacity);
      out[0] = 1;
      out[1] = kKindRiscvDm;
      putU16(out + 2, port_.swdio);
      putU16(out + 4, port_.swclk);
      putU32(out + 6, status);
      return tail.finish(completed(10), out, capacity);
    }
    case kOpAttach: {
      // method(u8: 0 leave it running, 1 halt) [TLV 0x01 max_speed]
      //   ->  connection(u8) DMSTATUS(u32) flags(u8: bit0 acknowledged a pending havereset, bit1 existing) speed_hz(u32)
      if (length < 1) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 1, length - 1, kAttachTags, out, capacity);
      if (refused(parsed)) return parsed;
      uint32_t max_hz = 0;
      bool critical = false;
      if (!maxSpeed(tail, max_hz, critical)) return rejected(kRejectMalformed);
      if (payload[0] > reg::wire_rvswd::kAttachMethodHalt) return rejected(kRejectUnsupported);
      if (capacity < 10) return failed();
      const bool halt = payload[0] == reg::wire_rvswd::kAttachMethodHalt;
      uint32_t status = 0;
      uint8_t flags = 0;
      uint8_t failure = kStatusOk;   // a failed attach answers its status alone: line (no answer) or timeout (no halt)
      if (port_.connected) {
        // Already attached: the same connection, nothing redone (a one-command-per-process host gets its link back).
        // A ceiling the running link is over cannot be met without attaching again.
        if (max_hz && phy.clockHz() > max_hz) {
          const Result r = tail.refuse(reg::wire_rvswd::kTlvAttachMaxSpeed, critical, out, capacity);
          if (refused(r)) return r;
        }
        flags |= 2;
        if (!port_.dm.readDmi(kDmStatus, status)) failure = kStatusLine;
        else if (halt && !(status & (1u << 9)) && !(port_.dm.halt() && port_.dm.readDmi(kDmStatus, status)))
          failure = kStatusTimeout;
      } else {
        if (!phy.setMaxHz(max_hz)) {   // a ceiling this link cannot keep (a fixed speed above it)
          const Result r = tail.refuse(reg::wire_rvswd::kTlvAttachMaxSpeed, critical, out, capacity);
          if (refused(r)) return r;
          phy.setMaxHz(0);
        }
        // A scan may have left the link up at a speed over the new ceiling: search again under it.
        if (max_hz && port_.dm.attached() && phy.clockHz() > max_hz) port_.dm.detach();
        if (!attachAndRead(port_.dm, status)) {
          failure = kStatusLine;
        } else {
          // A pending havereset freezes a V00x's DMSTATUS halt / run bits at their reset values (ch32rv 0.8.0):
          // acknowledge it first so the DMSTATUS returned is current, and say that it was there (flags bit0).
          if (port_.dm.ackHaveReset()) flags |= 1;
          if (!port_.dm.readDmi(kDmStatus, status)) failure = kStatusLine;
          else if (halt && !port_.dm.halt()) failure = kStatusTimeout;
        }
        if (failure == kStatusOk) port_.connected = true;
      }
      if (failure != kStatusOk) return tail.finish(failedStatus(failure, out, capacity), out, capacity);
      out[0] = 1;
      putU32(out + 1, status);
      out[5] = flags;
      putU32(out + 6, phy.clockHz());
      return tail.finish(completed(10), out, capacity);
    }
    case kOpAttachUnderReset: {
      // channel(u16, 0xffff = the probe's default) hold_ms(u16) [TLV 0x01 max_speed]  ->  connection(u8) dpc(u32) speed_hz(u32)
      if (length < 4) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 4, length - 4, kAttachTags, out, capacity);
      if (refused(parsed)) return parsed;
      uint32_t max_hz = 0;
      bool critical = false;
      if (!maxSpeed(tail, max_hz, critical)) return rejected(kRejectMalformed);
      if (capacity < 9) return failed();
      int channel = getU16(payload);
      if (channel == 0xffff) channel = port_.reset_default;
      if (channel < 0 || channel > 63 || !((port_.reset_allowed >> channel) & 1)) return rejected(kRejectUnavailable);
      if (!phy.setMaxHz(max_hz)) {
        const Result r = tail.refuse(reg::wire_rvswd::kTlvAttachUnderResetMaxSpeed, critical, out, capacity);
        if (refused(r)) return r;
        phy.setMaxHz(0);
      }
      uint32_t dpc = 0;
      // Open drain: pull low, then release to Hi-Z - never drive a reset line high.
      auto hold = [](void *ctx) { platformGpio(*static_cast<int *>(ctx), kGpioOpenDrainLow); };
      auto release = [](void *ctx) { platformGpio(*static_cast<int *>(ctx), kGpioOpenDrainRelease); };
      if (!port_.dm.attachUnderReset(hold, release, &channel, getU16(payload + 2), dpc)) {
        uint32_t status = 0;   // the module answers but the hart never stopped: timeout; no answer: line
        const bool answers = port_.dm.readDmi(kDmStatus, status) && status != 0 && status != 0xffffffffu;
        return tail.finish(failedStatus(answers ? kStatusTimeout : kStatusLine, out, capacity), out, capacity);
      }
      port_.connected = true;
      ++port_.resets;
      out[0] = 1;
      putU32(out + 1, dpc);
      putU32(out + 5, phy.clockHz());
      return tail.finish(completed(9), out, capacity);
    }
    case kOpDetach: {   // connection(u8) [TLV]
      if (length < 1) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 1, length - 1, out, capacity);
      if (refused(parsed)) return parsed;
      if (payload[0] != 1 || !port_.connected) return rejected(kRejectNoConnection);
      port_.dm.detach();
      port_.connected = false;
      return tail.finish(completed(), out, capacity);
    }
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
  // One block operation's data in bytes: the word buffer, and what fits a frame - write_block's request (header 6,
  // session 4, connection, address, count = 17) and read_block's result (header 5, done, status = 8).
  const size_t fits = max_frame_ > 17 ? (max_frame_ - 17) / 4 * 4 : 0;
  w.u16(kTagMaxLength, static_cast<uint16_t>(fits && fits < sizeof words_ ? fits : sizeof words_));
  return w.ok() ? w.length() : 0;
}

uint8_t TargetRiscvDm::failure(uint8_t otherwise) {
  uint32_t status = 0;
  return port_.dm.readDmi(kDmStatus, status) && status != 0xffffffffu ? otherwise : kStatusLine;
}

Result TargetRiscvDm::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 1) return rejected(kRejectMalformed);
  if (payload[0] != 1 || !port_.connected) return rejected(kRejectNoConnection);   // connection 1 only
  const uint8_t *p = payload + 1;
  const size_t n = length - 1;
  Ch32Dm &dm = port_.dm;
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
      if (p[0] > kResetHalt) return rejected(kRejectUnsupported);
      uint8_t len = 0;
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
      if (p[0] == kResetHalt) {   // flags bit0 = halted, pc = dpc
        uint32_t dpc = 0;
        ok = dm.resetHalt(dpc);
        ++port_.resets;
        out[1] = ok ? 1 : 0;
        out[2] = 1;
        putU32(out + 3, dpc);
      } else {
        const Ch32Dm::ResetReport r = dm.reset(p[0] == kResetRunConfirm);
        ++port_.resets;
        out[1] = r.flags;
        out[2] = r.attempts;
        putU32(out + 3, r.pc);
        ok = p[0] == kResetRunConfirm ? (r.flags & 2) != 0 : (r.flags & 1) != 0;
      }
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
      if (dm.halted()) {
        const bool ok = dm.step(before, after, moved);
        status = !ok ? (dm.lastCmderr() ? kStatusFault : failure(kStatusTimeout)) : moved ? kStatusOk : kStatusState;
      }
      out[0] = status;
      out[1] = moved;
      putU32(out + 2, before);
      putU32(out + 6, after);
      return tail.finish(outcome(status, 0, 10), out, capacity);
    }
    case kOpReadBlock: {   // address(u32) count(u16) [TLV]  ->  done(u16) status(u8) words (done of them)
      const Result parsed = plainTail(tail, p, n, 6, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      if ((address & 3) || !count || count > sizeof words_ / 4 || 3u + 4u * count > capacity)
        return rejected(kRejectMalformed);
      uint8_t status = kStatusState;
      uint16_t done = 0;
      if (dm.halted()) {
        uint8_t cmderr = 0;
        if (dm.readWords(address, words_, count, &cmderr)) {
          status = kStatusOk;
          done = count;
        } else {
          status = cmderr && cmderr != 3 ? kStatusFault : failure(kStatusFault);
        }
      }
      putU16(out, done);
      out[2] = status;
      for (uint16_t i = 0; i < done; ++i) putU32(out + 3 + 4 * i, words_[i]);
      return tail.finish(outcome(status, done, 3u + 4u * done), out, capacity);
    }
    case kOpWriteBlock: {   // address(u32) count(u16) count x word [TLV]  ->  done(u16) status(u8)
      if (n < 6) return rejected(kRejectMalformed);
      const uint32_t address = getU32(p);
      const uint16_t count = getU16(p + 4);
      const Result parsed = plainTail(tail, p, n, 6u + 4u * count, out, capacity);
      if (refused(parsed)) return parsed;
      if ((address & 3) || !count || count > sizeof words_ / 4) return rejected(kRejectMalformed);
      if (capacity < 3) return failed();
      uint8_t status = kStatusState;
      uint16_t done = 0;
      if (dm.halted()) {
        for (size_t i = 0; i < count; ++i) words_[i] = getU32(p + 6 + 4 * i);
        if (dm.writeWordsFast(address, words_, count)) {
          status = kStatusOk;
          done = count;
        } else {
          status = dm.lastCmderr() ? kStatusFault : failure(kStatusFault);
        }
      }
      putU16(out, done);
      out[2] = status;
      return tail.finish(outcome(status, done, 3), out, capacity);
    }
    case kOpRun: {
      // pc(u32) timeout_ms(u32, 0xFFFFFFFF = no limit) n(u8) n x (regno u16, value u32) n_out(u8) n_out x regno(u16)
      // [TLV]  ->  status(u8) stopped(u8) dpc(u32) elapsed_us(u32) n_out x value(u32)
      if (n < 9) return rejected(kRejectMalformed);
      const uint8_t regs = p[8];
      const size_t outs_at = 9u + 6u * regs;
      if (n < outs_at + 1) return rejected(kRejectMalformed);
      const uint8_t outs = p[outs_at];
      const Result parsed = plainTail(tail, p, n, outs_at + 1 + 2u * outs, out, capacity);
      if (refused(parsed)) return parsed;
      if (regs > kMaxRegs || outs > kMaxRegs || 10u + 4u * outs > capacity) return rejected(kRejectMalformed);
      uint16_t regnos[kMaxRegs];
      uint32_t values[kMaxRegs];
      for (uint8_t i = 0; i < regs; ++i) {
        regnos[i] = getU16(p + 9 + 6 * i);
        values[i] = getU32(p + 11 + 6 * i);
      }
      memset(out, 0, 10u + 4u * outs);
      uint8_t status = kStatusState;
      if (dm.halted()) {
        Ch32Dm::RunReport r;
        const bool ok = dm.runUntilHalt(getU32(p), regnos, values, regs, getU32(p + 4), r);
        status = !ok ? (dm.lastCmderr() ? kStatusFault : failure(kStatusFault)) : r.stopped ? kStatusOk : kStatusTimeout;
        out[1] = r.stopped;
        putU32(out + 2, r.dpc);
        putU32(out + 6, r.elapsed_us);
        for (uint8_t i = 0; i < outs && dm.halted(); ++i) {
          uint32_t v = 0;
          if (!dm.readRegister(getU16(p + outs_at + 1 + 2 * i), v) && status == kStatusOk)
            status = dm.lastCmderr() ? kStatusFault : failure(kStatusFault);
          putU32(out + 10 + 4 * i, v);
        }
      }
      out[0] = status;
      return tail.finish(outcome(status, 0, 10u + 4u * outs), out, capacity);
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
// [TLV]  ->  done(u16) status(u8) values. done = steps completed (= the index of the failed step); the values are
// those of the reads and polls among them, plus the failed step's last value when it was a poll that timed out (a
// poll cut off by the line adds nothing).
Result TargetRiscvDm::dmi(const uint8_t *p, size_t length, uint8_t *out, size_t capacity) {
  if (length < 2) return rejected(kRejectMalformed);
  const uint16_t count = getU16(p);
  // Check the whole list before running any of it: a broken list is malformed, not a partial run.
  size_t at = 2, values = 0;
  for (uint16_t i = 0; i < count; ++i) {
    if (at >= length) return rejected(kRejectMalformed);
    size_t size = 0;
    switch (p[at]) {
      case kStepWrite: size = 6; break;
      case kStepRead: size = 2; ++values; break;
      case kStepPoll: size = 12; ++values; break;
      case kStepDelay: size = 5; break;
      case kStepPollTime: size = 14; ++values; break;
      default: return rejected(kRejectMalformed);   // an unknown step kind cannot be sized: the list is broken
    }
    if (at + size > length) return rejected(kRejectMalformed);
    at += size;
  }
  Tail tail;
  const Result parsed = tail.parse(p + at, length - at, out, capacity);
  if (refused(parsed)) return parsed;
  if (3 + 4 * values > capacity) return rejected(kRejectMalformed);   // the answer would not fit a frame
  Ch32Dm &dm = port_.dm;
  size_t written = 3;
  uint16_t done = 0;
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
        else { putU32(out + written, v); written += 4; }
        at += 2;
        break;
      }
      case kStepPoll: {
        const uint32_t mask = getU32(s + 1), want = getU32(s + 5);
        const uint16_t max = getU16(s + 9);
        bool met = false;
        uint32_t last = 0;
        for (uint16_t k = 0; k < max && !met && status == kStatusOk; ++k) {
          uint32_t v = 0;
          if (!dm.readDmi(s[0], v)) status = kStatusLine;
          else { last = v; met = (v & mask) == want; }
        }
        if (status == kStatusOk && !met) status = kStatusTimeout;
        if (status != kStatusLine) { putU32(out + written, last); written += 4; }   // met or timed out, not cut off
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
        if (status != kStatusLine) { putU32(out + written, last); written += 4; }
        at += 14;
        break;
      }
    }
    if (status == kStatusOk) ++done;
  }
  putU16(out, done);
  out[2] = status;
  return tail.finish(outcome(status, done, written), out, capacity);
}

}  // namespace v1
}  // namespace oep
