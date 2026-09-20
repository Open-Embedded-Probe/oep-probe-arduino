#include "Esp32FixtureI2c.h"

#include <Wire.h>

namespace oep::prototype {

Esp32FixtureI2c* Esp32FixtureI2c::active_ = nullptr;

bool Esp32FixtureI2c::begin() {
  // Wire callbacks are process-global in Arduino-ESP32, so deliberately make
  // the one fixture peer explicit rather than pretending several can coexist.
  if (active_ && active_ != this) return false;
  started_ = wire_.begin(address_, sda_pin_, scl_pin_, frequency_hz_);
  if (!started_) return false;
  active_ = this;
  wire_.onReceive(receive);
  wire_.onRequest(request);
  return true;
}

BackendResult Esp32FixtureI2c::getStatus(FixtureI2cStatus& status) {
  if (!started_) return BackendResult::Unavailable;
  noInterrupts();
  const uint8_t last_rx_length = last_rx_length_;
  const uint16_t rx_transactions = rx_transactions_;
  const uint16_t request_transactions = request_transactions_;
  interrupts();

  status.flags = 0x01;
  if (digitalRead(scl_pin_)) status.flags |= 0x02;
  if (digitalRead(sda_pin_)) status.flags |= 0x04;
  status.last_rx_length = last_rx_length;
  status.rx_transactions = rx_transactions;
  status.request_transactions = request_transactions;
  status.frequency_hz = frequency_hz_;
  return BackendResult::Success;
}

void Esp32FixtureI2c::receive(int length) {
  if (!active_) return;
  uint8_t consumed = 0;
  while (active_->wire_.available()) {
    (void)active_->wire_.read();
    if (consumed != 0xff) ++consumed;
  }
  active_->last_rx_length_ = length < 0 ? 0 :
      (length > 0xff ? 0xff : static_cast<uint8_t>(length));
  ++active_->rx_transactions_;
}

void Esp32FixtureI2c::request() {
  if (!active_) return;
  // A fixed response makes the temporary peer useful without adding a
  // timing-sensitive host round trip to the I2C transaction.
  active_->wire_.write(static_cast<uint8_t>(0xA5));
  active_->wire_.write(static_cast<uint8_t>(0x5A));
  ++active_->request_transactions_;
}

}  // namespace oep::prototype
