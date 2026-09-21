// OEP v0 service interface. A service is one offered function: the endpoint
// assigns its connection-local reference and routes requests by (fn, op).
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "oep_v0.h"

namespace oep {

struct Result {
  uint8_t resolution;
  uint8_t detail;   // reject reason or outcome
  size_t length;    // payload bytes written into the result buffer
};

inline Result completed(size_t length = 0) { return {OEP_V0_RESOLUTION_COMPLETED, OEP_V0_OUTCOME_SUCCESS, length}; }
inline Result failed(size_t length = 0) { return {OEP_V0_RESOLUTION_COMPLETED, OEP_V0_OUTCOME_FAILED, length}; }
inline Result partial(size_t length = 0) { return {OEP_V0_RESOLUTION_COMPLETED, OEP_V0_OUTCOME_PARTIAL, length}; }
inline Result rejected(uint8_t reason) { return {OEP_V0_RESOLUTION_REJECTED, reason, 0}; }

class Service {
 public:
  virtual ~Service() = default;
  virtual uint16_t owner() const = 0;
  virtual uint16_t id() const = 0;
  virtual uint8_t revision() const = 0;
  virtual uint8_t flags() const { return 0; }
  // Write the result payload into out (capacity bytes) and return the resolution.
  virtual Result handle(uint8_t operation, const uint8_t *payload, size_t length,
                        uint8_t *out, size_t capacity) = 0;
  // Instance constraints as TLV (tag, len, value...), starting at tag index `first`.
  virtual size_t describe(uint8_t first, uint8_t *out, size_t capacity) {
    (void)first; (void)out; (void)capacity;
    return 0;
  }
  // The host vanished: release every lease and return pins to their idle state.
  virtual void abandon() {}

  uint16_t function = 0;  // assigned by Endpoint::addService
};

}  // namespace oep
