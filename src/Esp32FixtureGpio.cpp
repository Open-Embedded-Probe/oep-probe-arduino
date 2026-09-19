#include "Esp32FixtureGpio.h"

namespace oep::prototype {

BackendResult Esp32FixtureGpio::readDigital(uint8_t pin, uint8_t& value) {
  bool allowed = false;
  for (size_t index = 0; index < count_; ++index)
    if (allowed_pins_[index] == pin) allowed = true;
  if (!allowed) return BackendResult::Unavailable;
  pinMode(pin, INPUT);
  value = digitalRead(pin) ? 1 : 0;
  return BackendResult::Success;
}

}  // namespace oep::prototype
