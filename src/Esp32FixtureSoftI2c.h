#pragma once

#include "OepPrototype.h"

namespace oep::prototype {

// Deliberately narrow diagnostic target. It ACKs a matching 7-bit write
// address and following write bytes. It is for 10 kHz fault isolation, not a
// replacement for a hardware I2C target or a general Wire-compatible slave.
class Esp32FixtureSoftI2c : public FixtureI2cLifecycle {
 public:
  Esp32FixtureSoftI2c(uint8_t address, int sda_pin, int scl_pin,
                      uint32_t frequency_hz)
      : address_(address), sda_pin_(sda_pin), scl_pin_(scl_pin),
        frequency_hz_(frequency_hz) {}

  bool begin();
  bool setPins(int sda_pin, int scl_pin) override;
  void end() override;
  void service();
  BackendResult getStatus(FixtureI2cStatus& status) override;

 private:
  enum class Phase : uint8_t { Idle, Receive, AckPrepare, AckHigh };

  void releaseSda();
  void pullSdaLow();
  void start();
  void stop();
  void receiveBit(uint8_t bit);

  uint8_t address_;
  int sda_pin_;
  int scl_pin_;
  uint32_t frequency_hz_;
  bool started_ = false;
  bool last_sda_ = true;
  bool last_scl_ = true;
  Phase phase_ = Phase::Idle;
  uint8_t bit_count_ = 0;
  uint8_t byte_ = 0;
  bool selected_ = false;
  bool acknowledge_ = false;
  uint8_t last_rx_length_ = 0;
  uint16_t rx_transactions_ = 0;
};

}  // namespace oep::prototype
