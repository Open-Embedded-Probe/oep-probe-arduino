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
  summary.revision = 1;
  summary.channel_count = 55;
  summary.group_count = 1;
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

  // These are capabilities of the currently compiled backends, not a DUT
  // mapping. Dynamic peripheral muxing may widen the candidates later.
  if (ordinal == 12) channel.function_mask |= function(ProbeUartRx);
  if (ordinal == 6) channel.function_mask |= function(ProbeUartTx);
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

}  // namespace oep::prototype
