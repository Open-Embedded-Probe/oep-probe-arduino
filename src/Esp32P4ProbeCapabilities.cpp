#include "Esp32P4ProbeCapabilities.h"

namespace oep::prototype {
namespace {

constexpr uint64_t function(uint8_t bit) { return uint64_t{1} << bit; }

constexpr uint64_t kDigitalInput =
    function(ProbeGpioIn) | function(ProbePullUp) | function(ProbePullDown);
constexpr uint64_t kDigitalOutput =
    function(ProbeGpioOut) | function(ProbeOpenDrain);

bool reserved(uint8_t pin) {
  return pin == 2 || pin == 54 || (pin >= 24 && pin <= 27);
}

}  // namespace

BackendResult Esp32P4ProbeCapabilities::getSummary(
    ProbeCapabilitiesSummary& summary) {
  summary.revision = 2;
  summary.channel_count = 55;
  summary.group_count = 3;
  summary.voltage_domain_count = 1;
  return BackendResult::Success;
}

BackendResult Esp32P4ProbeCapabilities::getChannel(
    uint8_t ordinal, ProbeChannelCapability& channel) {
  if (ordinal >= 55) return BackendResult::Unavailable;
  channel.id = ordinal;
  channel.flags = (ordinal == 46 ? 0x01 : 0) |
      (reserved(ordinal) ? 0x02 : 0);
  channel.voltage_domain_mask = 0x01;
  channel.function_mask = kDigitalInput;
  if (ordinal != 46) channel.function_mask |= kDigitalOutput;
  if (!reserved(ordinal)) channel.function_mask |= function(ProbeCapture);

  // These are capabilities of the currently compiled backends, not a DUT
  // mapping. Dynamic peripheral muxing may widen the candidates later.
  if (ordinal == 12) channel.function_mask |= function(ProbeUartRx);
  if (ordinal == 6) channel.function_mask |= function(ProbeUartTx);
  if (ordinal == 50) channel.function_mask |= function(ProbeI2cSda);
  if (ordinal == 52) channel.function_mask |= function(ProbeI2cScl);
  return BackendResult::Success;
}

BackendResult Esp32P4ProbeCapabilities::getGroup(
    uint8_t ordinal, ProbeGroupCapability& group) {
  group.exclusive_group_mask = 0;
  if (ordinal == 0) {
    group.id = 1;
    group.kind = ProbeGroupUart;
    group.instance = 1;
    group.role_mask = function(ProbeUartRx) | function(ProbeUartTx);
    return BackendResult::Success;
  }
  if (ordinal == 1) {
    group.id = 2;
    group.kind = ProbeGroupI2cTarget;
    group.instance = 1;
    group.role_mask = function(ProbeI2cSda) | function(ProbeI2cScl);
    return BackendResult::Success;
  }
  if (ordinal == 2) {
    group.id = 3;
    group.kind = ProbeGroupCapture;
    group.instance = 1;
    group.role_mask = function(ProbeCapture);
    return BackendResult::Success;
  }
  return BackendResult::Unavailable;
}

BackendResult Esp32P4ProbeCapabilities::getVoltageDomain(
    uint8_t ordinal, ProbeVoltageDomainCapability& domain) {
  if (ordinal != 0) return BackendResult::Unavailable;
  domain.id = 1;
  domain.flags = 0x01;
  domain.nominal_mv = 3300;
  domain.input_max_mv = 3600;
  return BackendResult::Success;
}

BackendResult Esp32P4ProbeCapabilities::getGroupRole(
    uint8_t group_ordinal, uint8_t role_ordinal,
    ProbeGroupRoleCapability& role) {
  // Role IDs are group-local and stable across probe MCUs: 1/2 identify the
  // two terminals, while `function` states their electrical purpose.
  if (group_ordinal == 0 && role_ordinal < 2) {
    role.group_id = 1;
    role.role_id = role_ordinal + 1;  // rx, tx
    role.function = role_ordinal ? ProbeUartTx : ProbeUartRx;
    return BackendResult::Success;
  }
  if (group_ordinal == 1 && role_ordinal < 2) {
    role.group_id = 2;
    role.role_id = role_ordinal + 1;  // sda, scl
    role.function = role_ordinal ? ProbeI2cScl : ProbeI2cSda;
    return BackendResult::Success;
  }
  if (group_ordinal == 2 && role_ordinal < 2) {
    role.group_id = 3;
    role.role_id = role_ordinal + 1;  // clock, data
    role.function = ProbeCapture;
    return BackendResult::Success;
  }
  return BackendResult::Unavailable;
}

}  // namespace oep::prototype
