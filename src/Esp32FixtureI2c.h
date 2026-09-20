#pragma once

#include <Wire.h>

#include "OepPrototype.h"

namespace oep::prototype {

// Single hardware-target instance for the fixture. The callbacks run in the
// Arduino Wire context and only retain bounded diagnostic state.
class Esp32FixtureI2c : public FixtureI2cBackend {
 public:
  Esp32FixtureI2c(TwoWire& wire, uint8_t address, int sda_pin, int scl_pin,
                  uint32_t frequency_hz)
      : wire_(wire), address_(address), sda_pin_(sda_pin), scl_pin_(scl_pin),
        frequency_hz_(frequency_hz) {}

  bool begin();
  BackendResult getStatus(FixtureI2cStatus& status) override;

 private:
  static void receive(int length);
  static void request();

  static Esp32FixtureI2c* active_;
  TwoWire& wire_;
  uint8_t address_;
  int sda_pin_;
  int scl_pin_;
  uint32_t frequency_hz_;
  bool started_ = false;
  volatile uint8_t last_rx_length_ = 0;
  volatile uint16_t rx_transactions_ = 0;
  volatile uint16_t request_transactions_ = 0;
};

}  // namespace oep::prototype
