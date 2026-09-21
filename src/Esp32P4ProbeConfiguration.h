#pragma once

#include "Esp32FixtureUart.h"
#include "Esp32FixtureIdfI2c.h"
#include "Esp32FixtureGpio.h"
#include "Esp32FixtureRmtCapture.h"
#include "OepPrototype.h"

namespace oep::prototype {

// P4 implementation of the generic configuration contract.  It has no DUT
// knowledge: group/channel values are validated solely against this firmware's
// capability declaration.  It owns lifecycle only; bus protocol behavior is
// still supplied by the independent FixtureUart/FixtureI2c backends.
class Esp32P4ProbeConfiguration final : public ProbeConfigurationBackend {
 public:
  Esp32P4ProbeConfiguration(Esp32FixtureUart& uart, Esp32FixtureIdfI2c& i2c,
                             Esp32FixtureRmtCapture& capture,
                             Esp32FixtureGpio& gpio)
      : uart_(uart), i2c_(i2c), capture_(capture), gpio_(gpio) {}
  BackendResult apply(const ProbeConfigurationRole* roles, uint8_t count,
                      uint32_t& lease_id) override;
  BackendResult release(uint32_t lease_id) override;
  void abandon() override;

 private:
  Esp32FixtureUart& uart_;
  Esp32FixtureIdfI2c& i2c_;
  Esp32FixtureRmtCapture& capture_;
  Esp32FixtureGpio& gpio_;
  uint32_t active_lease_ = 0;
  uint32_t next_lease_ = 1;
  bool active_uart_ = false;
  bool active_i2c_ = false;
  bool active_capture_ = false;
};

}  // namespace oep::prototype
