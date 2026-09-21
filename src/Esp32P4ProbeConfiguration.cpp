#include "Esp32P4ProbeConfiguration.h"

namespace oep::prototype {
namespace {

bool hasRole(const ProbeConfigurationRole* roles, uint8_t count,
             uint8_t function, uint16_t channel) {
  for (uint8_t index = 0; index < count; ++index)
    if (roles[index].group_id == 1 && roles[index].function == function &&
        roles[index].channel_id == channel) return true;
  return false;
}

}  // namespace

BackendResult Esp32P4ProbeConfiguration::apply(
    const ProbeConfigurationRole* roles, uint8_t count, uint32_t& lease_id) {
  // Validate the complete plan before changing UART ownership.  Revision-1
  // P4 declares precisely one configurable peer: Serial1 RX GPIO12 / TX GPIO6.
  if (active_lease_ || count != 2 ||
      !hasRole(roles, count, ProbeUartRx, 12) ||
      !hasRole(roles, count, ProbeUartTx, 6)) return BackendResult::Unavailable;
  if (!uart_.setPins(12, 6)) return BackendResult::Failed;
  lease_id = next_lease_++;
  if (!next_lease_) ++next_lease_;
  active_lease_ = lease_id;
  return BackendResult::Success;
}

BackendResult Esp32P4ProbeConfiguration::release(uint32_t lease_id) {
  if (!lease_id || lease_id != active_lease_) return BackendResult::Unavailable;
  // Keep all released probe pins benign.  FixtureUart operations are rejected
  // until a new configuration and baud request, preventing stale use.
  if (!uart_.setPins(12, 6)) return BackendResult::Failed;
  active_lease_ = 0;
  return BackendResult::Success;
}

void Esp32P4ProbeConfiguration::abandon() {
  if (!active_lease_) return;
  // `setPins` ends a configured UART before making subsequent operations
  // unavailable.  Ignore an impossible GPIO failure: either way the lease is
  // no longer valid and a host cannot continue using it.
  (void)uart_.setPins(12, 6);
  active_lease_ = 0;
}

}  // namespace oep::prototype
