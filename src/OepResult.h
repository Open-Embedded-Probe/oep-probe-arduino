// What a handler returns (resolution, reject reason or outcome, payload length) and the pin-plan entry. Shared by
// the v1 interfaces and the v0 services; the numbers are the same on both wires (oep-spec v1-core-wire-delta §1),
// so this header needs nothing from oep_v0.h.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace oep {

enum : uint8_t { kResolutionRejected = 0x00, kResolutionCompleted = 0x01, kResolutionAccepted = 0x02 };
enum : uint8_t { kOutcomeSuccess = 0x00, kOutcomeFailed = 0x01, kOutcomePartial = 0x02 };
enum : uint8_t {
  kRejectUnknownFunction = 0x01, kRejectUnknownOperation = 0x02, kRejectMalformed = 0x03, kRejectUnavailable = 0x04,
  kRejectBusy = 0x05, kRejectWindowExceeded = 0x06,
};

struct Result {
  uint8_t resolution;
  uint8_t detail;   // reject reason or outcome
  size_t length;    // payload bytes written into the result buffer
};

inline Result completed(size_t length = 0) { return {kResolutionCompleted, kOutcomeSuccess, length}; }
inline Result failed(size_t length = 0) { return {kResolutionCompleted, kOutcomeFailed, length}; }
inline Result partial(size_t length = 0) { return {kResolutionCompleted, kOutcomePartial, length}; }
inline Result rejected(uint8_t reason) { return {kResolutionRejected, reason, 0}; }

// One entry of a plan: this connection asks `function` to use probe `channel` in `role`.
struct RoleAssignment {
  uint16_t function;
  uint8_t role;
  uint16_t channel;
};

}  // namespace oep
