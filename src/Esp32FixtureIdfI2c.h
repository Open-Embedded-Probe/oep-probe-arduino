#pragma once

#include <driver/i2c_slave.h>

#include "OepPrototype.h"

namespace oep::prototype {

// Direct ESP-IDF slave-driver diagnostic. This bypasses Arduino Wire and its
// legacy slave HAL; it is not yet a portable fixture implementation.
class Esp32FixtureIdfI2c : public FixtureI2cBackend {
 public:
  Esp32FixtureIdfI2c(i2c_port_num_t port, uint8_t address, int sda_pin,
                     int scl_pin, uint32_t frequency_hz)
      : port_(port), address_(address), sda_pin_(sda_pin), scl_pin_(scl_pin),
        frequency_hz_(frequency_hz) {}
  bool begin();
  // Safe lifecycle used by ProbeConfiguration.  `end` releases the IDF
  // device and returns both bus pins to neutral GPIO inputs.
  bool setPins(int sda_pin, int scl_pin);
  void end();
  BackendResult getStatus(FixtureI2cStatus& status) override;

 private:
  static bool received(i2c_slave_dev_handle_t,
                       const i2c_slave_rx_done_event_data_t*, void* arg);
  static bool stretched(i2c_slave_dev_handle_t,
                        const i2c_slave_stretch_event_data_t*, void* arg);
  i2c_port_num_t port_;
  uint8_t address_;
  int sda_pin_;
  int scl_pin_;
  uint32_t frequency_hz_;
  i2c_slave_dev_handle_t device_ = nullptr;
  uint8_t rx_buffer_[32]{};
  volatile uint16_t rx_transactions_ = 0;
  volatile uint8_t stretch_cause_mask_ = 0;
};

}  // namespace oep::prototype
