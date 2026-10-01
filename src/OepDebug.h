// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The shared parts of the standard wire / target interfaces (oep-spec docs/oep-if-common.ja.md §3 status,
// docs/oep-if-debug.ja.md §1 pin pairs). Not the core: only oep.wire.* / oep.target.* use them.
#pragma once

#include "Oep.h"

namespace oep {

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

// A wire op whose success shape has no done / status (scan, attach, detach) and failed: completed failed with the
// payload status(u8) [TLV] (oep-if-common §3).
inline Result failedStatus(uint8_t status, uint8_t *out, size_t capacity) {
  if (capacity < 1) return failed();
  out[0] = status;
  return failed(1);
}

}  // namespace oep
