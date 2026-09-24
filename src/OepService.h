// OEP v0 service interface. A service is one offered function: the endpoint
// assigns its connection-local reference and routes requests by (fn, op).
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "OepResult.h"
#include "oep_v0.h"

namespace oep {

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
  // Plan (lease) lifecycle. planCheck validates without side effects and returns a
  // reject reason (0 = acceptable); planApply configures; planRelease undoes it.
  virtual uint8_t planCheck(const RoleAssignment *roles, size_t count) {
    (void)roles;
    return count ? OEP_V0_REJECT_UNAVAILABLE : 0;  // by default a service takes no roles
  }
  virtual bool planApply(const RoleAssignment *roles, size_t count) { (void)roles; (void)count; return true; }
  virtual void planRelease() {}
  // The host vanished: release every lease and return pins to their idle state.
  virtual void abandon() { planRelease(); }

  uint16_t function = 0;  // assigned by Endpoint::addService
};

}  // namespace oep
