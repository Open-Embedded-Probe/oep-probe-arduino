// The shared parts of the standard wire / target interfaces (oep-spec docs/oep-if-common.ja.md §3 status,
// docs/oep-if-debug.ja.md §1 pin pairs). Not the core: only oep.wire.* / oep.target.* use them.
#pragma once

#include "OepV1.h"

namespace oep {
namespace v1 {

// The status byte of wire and target results (oep-if-common §3). A failure is completed failed / partial with this
// in it.
constexpr uint8_t kStatusOk = reg::kStatusOk, kStatusWait = reg::kStatusWait, kStatusLine = reg::kStatusLine,
                  kStatusFault = reg::kStatusFault, kStatusTimeout = reg::kStatusTimeout, kStatusState = reg::kStatusState;

// A wire / target result that did not go all the way (oep-if-common §3): nothing done = failed, some done = partial.
// The payload has the success shape; its status byte says why.
inline Result outcome(uint8_t status, size_t done, size_t length) {
  if (status == kStatusOk) return completed(length);
  return done ? partial(length) : failed(length);
}

// A wire op that failed before it had anything of its success shape to report (scan, attach, attach_under_reset,
// detach): completed failed with the status byte alone.
inline Result failedStatus(uint8_t status, uint8_t *out, size_t capacity) {
  if (capacity < 1) return failed();
  out[0] = status;
  return failed(1);
}

// Pin pairs (oep-if-debug §1), for a probe whose wire has one fixed pair (swclk 0xffff on one wire): scan's
// count(u8) + count x (swdio u16, swclk u16) - every pair must be that one (count 0 = the probe's pairs); the attach
// pins TLV (swdio u16, swclk u16), absent = that pair. Return 0 when allowed, else a reject reason.
inline uint8_t fixedPairScan(const uint8_t *p, size_t n, uint16_t swdio, uint16_t swclk, size_t &fixed) {
  if (n < 1 || n < 1u + 4u * p[0]) return kRejectMalformed;
  fixed = 1u + 4u * p[0];
  for (uint8_t i = 0; i < p[0]; ++i)
    if (getU16(p + 1 + 4 * i) != swdio || getU16(p + 3 + 4 * i) != swclk) return kRejectUnavailable;   // not allowed here
  return 0;
}
inline uint8_t fixedPairPins(const uint8_t *v, uint8_t len, uint16_t swdio, uint16_t swclk) {
  if (!v) return 0;                                 // absent: the one pair
  if (len != 4) return kRejectMalformed;
  return getU16(v) == swdio && getU16(v + 2) == swclk ? 0 : kRejectUnavailable;
}

}  // namespace v1
}  // namespace oep
