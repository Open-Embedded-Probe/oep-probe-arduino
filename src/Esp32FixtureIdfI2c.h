#pragma once

#if CONFIG_IDF_TARGET_ESP32P4

#include <driver/i2c_slave.h>

#include "OepPrototype.h"

namespace oep::prototype {

// Direct ESP-IDF slave-driver diagnostic. This bypasses Arduino Wire and its
// legacy slave HAL; it is not yet a portable fixture implementation.
class Esp32FixtureIdfI2c : public FixtureI2cLifecycle {
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
  // IDF I2C slave v1 FIFO reception is not a variable-length stream: the
  // receive job must match one whole write transaction.  This diagnostic
  // contract is deliberately four bytes, matching the first X035 probe
  // frame; do not widen it to make an "up to N bytes" API.
  static constexpr size_t kFixedWriteBytes = 4;
  uint8_t rx_buffer_[kFixedWriteBytes]{};
  volatile uint8_t last_rx_length_ = 0;
  volatile uint16_t rx_transactions_ = 0;
  volatile uint8_t stretch_cause_mask_ = 0;
};

}  // namespace oep::prototype

#endif  // CONFIG_IDF_TARGET_ESP32P4
