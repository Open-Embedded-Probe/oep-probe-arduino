#include "Esp32P4ProbeConfiguration.h"

namespace oep::prototype {
namespace {

bool hasRole(const ProbeConfigurationRole* roles, uint8_t count,
             uint16_t group, uint8_t function, uint16_t channel) {
  for (uint8_t index = 0; index < count; ++index)
    if (roles[index].group_id == group && roles[index].function == function &&
        roles[index].channel_id == channel) return true;
  return false;
}

}  // namespace

BackendResult Esp32P4ProbeConfiguration::apply(
    const ProbeConfigurationRole* roles, uint8_t count, uint32_t& lease_id) {
  // Validate every selected group before changing either peripheral.  The P4
  // revision-1 candidates are Serial1 RX GPIO12/TX GPIO6 and I2C1 SDA
  // GPIO50/SCL GPIO52.  Both may be one atomic plan, but partial groups and
  // duplicate/unknown entries fail closed.
  if (active_lease_ || count < 2 || count > 4) return BackendResult::Unavailable;
  const bool wants_uart = hasRole(roles, count, 1, ProbeUartRx, 12) ||
      hasRole(roles, count, 1, ProbeUartTx, 6);
  const bool wants_i2c = hasRole(roles, count, 2, ProbeI2cSda, 50) ||
      hasRole(roles, count, 2, ProbeI2cScl, 52);
  const uint8_t expected = (wants_uart ? 2 : 0) + (wants_i2c ? 2 : 0);
  if (count != expected ||
      (wants_uart && (!hasRole(roles, count, 1, ProbeUartRx, 12) ||
                      !hasRole(roles, count, 1, ProbeUartTx, 6))) ||
      (wants_i2c && (!hasRole(roles, count, 2, ProbeI2cSda, 50) ||
                     !hasRole(roles, count, 2, ProbeI2cScl, 52))) ||
      (!wants_uart && !wants_i2c)) return BackendResult::Unavailable;

  // An I2C startup failure occurs before UART state changes.  `begin` cleans
  // its IDF device on failure, so this preserves the all-or-nothing contract.
  if (wants_i2c && (!i2c_.setPins(50, 52) || !i2c_.begin()))
    return BackendResult::Failed;
  if (wants_uart && !uart_.setPins(12, 6)) {
    if (wants_i2c) i2c_.end();
    return BackendResult::Failed;
  }
  lease_id = next_lease_++;
  if (!next_lease_) ++next_lease_;
  active_lease_ = lease_id;
  active_uart_ = wants_uart;
  active_i2c_ = wants_i2c;
  return BackendResult::Success;
}

BackendResult Esp32P4ProbeConfiguration::release(uint32_t lease_id) {
  if (!lease_id || lease_id != active_lease_) return BackendResult::Unavailable;
  // Keep all released probe pins benign.  FixtureUart operations are rejected
  // until a new configuration and baud request, preventing stale use.
  if (active_i2c_) i2c_.end();
  if (active_uart_ && !uart_.setPins(12, 6)) return BackendResult::Failed;
  active_lease_ = 0;
  active_uart_ = false;
  active_i2c_ = false;
  return BackendResult::Success;
}

void Esp32P4ProbeConfiguration::abandon() {
  if (!active_lease_) return;
  // `setPins` ends a configured UART before making subsequent operations
  // unavailable.  Ignore an impossible GPIO failure: either way the lease is
  // no longer valid and a host cannot continue using it.
  if (active_i2c_) i2c_.end();
  if (active_uart_) (void)uart_.setPins(12, 6);
  active_lease_ = 0;
  active_uart_ = false;
  active_i2c_ = false;
}

}  // namespace oep::prototype
