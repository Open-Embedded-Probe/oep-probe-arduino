#include "Esp32FixtureUart.h"

namespace oep::prototype {

bool Esp32FixtureUart::setPins(int8_t rx_pin, int8_t tx_pin) {
  if (rx_pin < 0 || tx_pin < 0 || rx_pin == tx_pin) return false;
  if (configured_) serial_.end();
  rx_pin_ = rx_pin;
  tx_pin_ = tx_pin;
  configured_ = false;
  return true;
}

BackendResult Esp32FixtureUart::configure(uint32_t requested_baud,
                                          uint32_t& actual_baud) {
  if (requested_baud < 1200 || requested_baud > 2000000)
    return BackendResult::Unavailable;
  if (rx_pin_ < 0 || tx_pin_ < 0) return BackendResult::Unavailable;
  serial_.begin(requested_baud, SERIAL_8N1, rx_pin_, tx_pin_);
  configured_ = true;
  actual_baud = requested_baud;
  return BackendResult::Success;
}

BackendResult Esp32FixtureUart::writeBytes(
    const uint8_t* data, size_t length, size_t& written) {
  if (!configured_) return BackendResult::Unavailable;
  written = serial_.write(data, length);
  serial_.flush();
  return written == length ? BackendResult::Success : BackendResult::Failed;
}

BackendResult Esp32FixtureUart::readAvailable(
    uint8_t* output, size_t capacity, size_t& length) {
  if (!configured_) return BackendResult::Unavailable;
  length = 0;
  while (length < capacity && serial_.available())
    output[length++] = static_cast<uint8_t>(serial_.read());
  return BackendResult::Success;
}

}  // namespace oep::prototype
