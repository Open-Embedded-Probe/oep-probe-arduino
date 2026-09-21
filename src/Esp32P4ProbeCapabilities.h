#pragma once

#include "OepPrototype.h"

namespace oep::prototype {

// Describes the running P4 firmware only. It deliberately contains no DUT,
// target-board, or wiring-manifest names.
class Esp32P4ProbeCapabilities final : public ProbeCapabilitiesBackend {
 public:
  BackendResult getSummary(ProbeCapabilitiesSummary& summary) override;
  BackendResult getChannel(uint8_t ordinal,
                           ProbeChannelCapability& channel) override;
  BackendResult getGroup(uint8_t ordinal,
                         ProbeGroupCapability& group) override;
  BackendResult getVoltageDomain(
      uint8_t ordinal, ProbeVoltageDomainCapability& domain) override;
  BackendResult getGroupRole(uint8_t group_ordinal, uint8_t role_ordinal,
                             ProbeGroupRoleCapability& role) override;
};

}  // namespace oep::prototype
