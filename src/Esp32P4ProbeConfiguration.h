#pragma once

#include "Esp32FixtureUart.h"
#include "OepPrototype.h"

namespace oep::prototype {

// P4 implementation of the generic configuration contract.  It has no DUT
// knowledge: group/channel values are validated solely against this firmware's
// capability declaration.  I2C is intentionally absent until its IDF backend
// gains safe stop/reconfigure support.
class Esp32P4ProbeConfiguration final : public ProbeConfigurationBackend {
 public:
  explicit Esp32P4ProbeConfiguration(Esp32FixtureUart& uart) : uart_(uart) {}
  BackendResult apply(const ProbeConfigurationRole* roles, uint8_t count,
                      uint32_t& lease_id) override;
  BackendResult release(uint32_t lease_id) override;

 private:
  Esp32FixtureUart& uart_;
  uint32_t active_lease_ = 0;
  uint32_t next_lease_ = 1;
};

}  // namespace oep::prototype
