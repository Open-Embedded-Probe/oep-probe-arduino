#pragma once

#include "OepPrototype.h"

namespace oep::prototype {

class Esp32FixtureUart : public FixtureUartBackend {
 public:
  Esp32FixtureUart(HardwareSerial& serial, int8_t rx_pin, int8_t tx_pin)
      : serial_(serial), rx_pin_(rx_pin), tx_pin_(tx_pin) {}
  BackendResult configure(uint32_t requested_baud,
                          uint32_t& actual_baud) override;
  BackendResult writeBytes(const uint8_t* data, size_t length,
                           size_t& written) override;
  BackendResult readAvailable(uint8_t* output, size_t capacity,
                              size_t& length) override;

 private:
  HardwareSerial& serial_;
  int8_t rx_pin_;
  int8_t tx_pin_;
  bool configured_ = false;
};

}  // namespace oep::prototype
