#include "Esp32P4ProbeConfiguration.h"

namespace oep::prototype {
namespace {

bool hasRole(const ProbeConfigurationRole* roles, uint8_t count,
             uint16_t group, uint8_t role_id, uint8_t function,
             uint16_t channel) {
  for (uint8_t index = 0; index < count; ++index)
    if (roles[index].group_id == group && roles[index].function == function &&
        (!roles[index].role_id || roles[index].role_id == role_id) &&
        roles[index].channel_id == channel) return true;
  return false;
}

const ProbeConfigurationRole* findRole(const ProbeConfigurationRole* roles,
                                       uint8_t count, uint16_t group,
                                       uint8_t role_id, uint8_t function) {
  for (uint8_t index = 0; index < count; ++index)
    if (roles[index].group_id == group && roles[index].function == function &&
        (!roles[index].role_id || roles[index].role_id == role_id))
      return &roles[index];
  return nullptr;
}

bool capturePinAllowed(uint16_t pin) {
  return pin <= 54 && pin != 2 && pin != 54 &&
      !(pin >= 24 && pin <= 27);
}

}  // namespace

BackendResult Esp32P4ProbeConfiguration::apply(
    const ProbeConfigurationRole* roles, uint8_t count, uint32_t& lease_id) {
  // Validate every selected group before changing either peripheral.  The P4
  // revision-1 candidates are Serial1 RX GPIO12/TX GPIO6 and I2C1 SDA
  // GPIO50/SCL GPIO52.  Both may be one atomic plan, but partial groups and
  // duplicate/unknown entries fail closed.
  if (active_lease_ || count < 2 || count > 6) return BackendResult::Unavailable;
  const bool wants_uart = hasRole(roles, count, 1, 1, ProbeUartRx, 12) ||
      hasRole(roles, count, 1, 2, ProbeUartTx, 6);
  const bool wants_i2c = hasRole(roles, count, 2, 1, ProbeI2cSda, 50) ||
      hasRole(roles, count, 2, 2, ProbeI2cScl, 52);
  const auto* capture_clock = findRole(roles, count, 3, 1, ProbeCapture);
  const auto* capture_data = findRole(roles, count, 3, 2, ProbeCapture);
  const bool wants_capture = capture_clock || capture_data;
  const uint8_t expected = (wants_uart ? 2 : 0) + (wants_i2c ? 2 : 0) +
      (wants_capture ? 2 : 0);
  if (count != expected ||
      (wants_uart && (!hasRole(roles, count, 1, 1, ProbeUartRx, 12) ||
                      !hasRole(roles, count, 1, 2, ProbeUartTx, 6))) ||
      (wants_i2c && (!hasRole(roles, count, 2, 1, ProbeI2cSda, 50) ||
                     !hasRole(roles, count, 2, 2, ProbeI2cScl, 52))) ||
      (wants_capture && (!capture_clock || !capture_data ||
                         !capturePinAllowed(capture_clock->channel_id) ||
                         !capturePinAllowed(capture_data->channel_id) ||
                         capture_clock->channel_id == capture_data->channel_id)) ||
      (wants_capture && wants_uart &&
       (capture_clock->channel_id == 6 || capture_clock->channel_id == 12 ||
        capture_data->channel_id == 6 || capture_data->channel_id == 12)) ||
      (wants_capture && wants_i2c &&
       (capture_clock->channel_id == 50 || capture_clock->channel_id == 52 ||
        capture_data->channel_id == 50 || capture_data->channel_id == 52)) ||
      (!wants_uart && !wants_i2c && !wants_capture)) return BackendResult::Unavailable;

  // An I2C startup failure occurs before UART state changes.  `begin` cleans
  // its IDF device on failure, so this preserves the all-or-nothing contract.
  if (wants_capture && !capture_.configure(capture_clock->channel_id,
                                            capture_data->channel_id))
    return BackendResult::Failed;
  if (wants_i2c && (!i2c_.setPins(50, 52) || !i2c_.begin())) {
    if (wants_capture) capture_.end();
    return BackendResult::Failed;
  }
  if (wants_uart && (!uart_.setPins(12, 6) || !uart_.enable())) {
    if (wants_i2c) i2c_.end();
    if (wants_capture) capture_.end();
    return BackendResult::Failed;
  }
  lease_id = next_lease_++;
  if (!next_lease_) ++next_lease_;
  active_lease_ = lease_id;
  active_uart_ = wants_uart;
  active_i2c_ = wants_i2c;
  active_capture_ = wants_capture;
  if (wants_capture) gpio_.lockCapturePins(capture_clock->channel_id,
                                           capture_data->channel_id);
  return BackendResult::Success;
}

BackendResult Esp32P4ProbeConfiguration::release(uint32_t lease_id) {
  if (!lease_id || lease_id != active_lease_) return BackendResult::Unavailable;
  // Keep all released probe pins benign.  FixtureUart operations are rejected
  // until a new configuration and baud request, preventing stale use.
  if (active_i2c_) i2c_.end();
  if (active_capture_) capture_.end();
  if (active_capture_) gpio_.unlockCapturePins();
  if (active_uart_) uart_.disable();
  active_lease_ = 0;
  active_uart_ = false;
  active_i2c_ = false;
  active_capture_ = false;
  return BackendResult::Success;
}

void Esp32P4ProbeConfiguration::abandon() {
  if (!active_lease_) return;
  // `setPins` ends a configured UART before making subsequent operations
  // unavailable.  Ignore an impossible GPIO failure: either way the lease is
  // no longer valid and a host cannot continue using it.
  if (active_i2c_) i2c_.end();
  if (active_capture_) capture_.end();
  if (active_capture_) gpio_.unlockCapturePins();
  if (active_uart_) uart_.disable();
  active_lease_ = 0;
  active_uart_ = false;
  active_i2c_ = false;
  active_capture_ = false;
}

}  // namespace oep::prototype
