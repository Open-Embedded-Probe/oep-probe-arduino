#pragma once

#include "OepPrototype.h"

namespace oep::prototype {

class Esp32FixtureGpio : public FixtureGpioBackend {
 public:
  Esp32FixtureGpio(const uint8_t* allowed_pins, size_t count)
      : allowed_pins_(allowed_pins), count_(count) {}
  BackendResult readDigital(uint8_t pin, uint8_t& value) override;
  BackendResult configureDigital(uint8_t pin, uint8_t mode) override;
  void lockCapturePins(uint8_t first, uint8_t second);
  void unlockCapturePins();

 private:
  bool allowed(uint8_t pin) const;
  bool locked(uint8_t pin) const;
  const uint8_t* allowed_pins_;
  size_t count_;
  uint8_t capture_first_ = 0xff;
  uint8_t capture_second_ = 0xff;
};

}  // namespace oep::prototype
