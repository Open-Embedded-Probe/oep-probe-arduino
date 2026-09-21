#include "Esp32FixtureGpio.h"

#include <driver/gpio.h>

namespace oep::prototype {

bool Esp32FixtureGpio::allowed(uint8_t pin) const {
  for (size_t index = 0; index < count_; ++index)
    if (allowed_pins_[index] == pin) return true;
  return false;
}

bool Esp32FixtureGpio::locked(uint8_t pin) const {
  return pin == capture_first_ || pin == capture_second_;
}

void Esp32FixtureGpio::lockCapturePins(uint8_t first, uint8_t second) {
  capture_first_ = first;
  capture_second_ = second;
}

void Esp32FixtureGpio::unlockCapturePins() {
  capture_first_ = 0xff;
  capture_second_ = 0xff;
}

BackendResult Esp32FixtureGpio::readDigital(uint8_t pin, uint8_t& value) {
  if (!allowed(pin) || locked(pin)) return BackendResult::Unavailable;
  pinMode(pin, INPUT);
  value = digitalRead(pin) ? 1 : 0;
  return BackendResult::Success;
}

BackendResult Esp32FixtureGpio::configureDigital(uint8_t pin, uint8_t mode) {
  if (!allowed(pin) || locked(pin)) return BackendResult::Unavailable;

  gpio_config_t config = {};
  config.pin_bit_mask = uint64_t{1} << pin;
  config.intr_type = GPIO_INTR_DISABLE;
  switch (mode) {
    case FixtureInputFloating:
      config.mode = GPIO_MODE_INPUT;
      break;
    case FixtureInputPullUp:
    case FixtureInputPullDown:
    case FixtureInputPullUpDown:
      config.mode = GPIO_MODE_INPUT;
      break;
    case FixtureOutputLow:
    case FixtureOutputHigh:
      config.mode = GPIO_MODE_OUTPUT;
      break;
    case FixtureOpenDrainLow:
    case FixtureOpenDrainRelease:
      config.mode = GPIO_MODE_INPUT_OUTPUT_OD;
      break;
    default:
      return BackendResult::Unavailable;
  }
  if (gpio_config(&config) != ESP_OK) return BackendResult::Failed;

  gpio_pull_mode_t pull = GPIO_FLOATING;
  if (mode == FixtureInputPullUp) pull = GPIO_PULLUP_ONLY;
  if (mode == FixtureInputPullDown) pull = GPIO_PULLDOWN_ONLY;
  if (mode == FixtureInputPullUpDown) pull = GPIO_PULLUP_PULLDOWN;
  if (gpio_set_pull_mode(static_cast<gpio_num_t>(pin), pull) != ESP_OK)
    return BackendResult::Failed;

  if (mode == FixtureOutputLow || mode == FixtureOpenDrainLow)
    gpio_set_level(static_cast<gpio_num_t>(pin), 0);
  if (mode == FixtureOutputHigh || mode == FixtureOpenDrainRelease)
    gpio_set_level(static_cast<gpio_num_t>(pin), 1);
  return BackendResult::Success;
}

}  // namespace oep::prototype
