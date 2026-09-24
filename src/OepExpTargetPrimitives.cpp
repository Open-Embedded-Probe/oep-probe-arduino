#include "OepExpTargetPrimitives.h"

namespace oep {
namespace {

uint16_t getU16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }
uint32_t getU32(const uint8_t *p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }
void putU16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
void putU32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

enum : uint8_t { kOk = 0, kMalformed = 1, kAccess = 2, kPollGaveUp = 3 };

}  // namespace

Result ExpTargetPrimitives::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (!dm_.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
  switch (operation) {
    case 0x01: return steps(payload, length, out, capacity);
    case 0x02: return run(payload, length, out, capacity);
    default: return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

Result ExpTargetPrimitives::steps(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (capacity < 3) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
  size_t at = 0, written = 3;
  uint16_t done = 0;
  uint8_t status = kOk;
  while (at < length && status == kOk) {
    const uint8_t kind = payload[at];
    const uint8_t *p = payload + at + 1;
    const size_t left = length - at - 1;
    switch (kind) {
      case 0x01:
        if (left < 8) { status = kMalformed; break; }
        if (!dm_.writeWord(getU32(p), getU32(p + 4))) { status = kAccess; break; }
        at += 9;
        break;
      case 0x02: {
        if (left < 4 || written + 4 > capacity) { status = kMalformed; break; }
        uint32_t value = 0;
        if (!dm_.readWordScalar(getU32(p), value)) { status = kAccess; break; }
        putU32(out + written, value);
        written += 4;
        at += 5;
        break;
      }
      case 0x03: {
        if (left < 14) { status = kMalformed; break; }
        const uint32_t address = getU32(p), mask = getU32(p + 4), want = getU32(p + 8);
        const uint16_t max_reads = getU16(p + 12);
        bool met = false;
        for (uint16_t i = 0; i < max_reads && !met; ++i) {
          uint32_t value = 0;
          if (!dm_.readWordScalar(address, value)) { status = kAccess; break; }
          met = (value & mask) == want;
        }
        if (status == kOk && !met) status = kPollGaveUp;
        if (status == kOk) at += 15;
        break;
      }
      case 0x04: {
        if (left < 6) { status = kMalformed; break; }
        const uint32_t address = getU32(p);
        const uint16_t count = getU16(p + 4);
        if (!count || count > sizeof block_ / sizeof block_[0] || left < 6u + 4u * count) { status = kMalformed; break; }
        for (uint16_t i = 0; i < count; ++i) block_[i] = getU32(p + 6 + 4 * i);
        if (!dm_.writeWordsFast(address, block_, count)) { status = kAccess; break; }
        at += 7 + 4u * count;
        break;
      }
      default:
        status = kMalformed;
        break;
    }
    if (status == kOk) ++done;
  }
  putU16(out, done);
  out[2] = status;
  return status == kOk ? completed(written) : failed(written);
}

Result ExpTargetPrimitives::run(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 7 || capacity < 13) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
  const uint32_t pc = getU32(payload);
  const uint16_t timeout_ms = getU16(payload + 4);
  const uint8_t n = payload[6];
  if (n > 16 || length != 7u + 6u * n) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
  uint16_t regnos[16];
  uint32_t values[16];
  for (uint8_t i = 0; i < n; ++i) {
    regnos[i] = getU16(payload + 7 + 6 * i);
    values[i] = getU32(payload + 9 + 6 * i);
  }
  Ch32Dm::RunReport report;
  const bool ok = dm_.runUntilHalt(pc, regnos, values, n, uint32_t(timeout_ms) * 1000u, report);
  out[0] = report.stopped;
  putU32(out + 1, report.dpc);
  putU32(out + 5, report.a0);
  putU32(out + 9, report.elapsed_us);
  return ok ? completed(13) : failed(13);
}

}  // namespace oep
