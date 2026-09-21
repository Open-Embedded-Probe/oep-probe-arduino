#include "Esp32FixtureSoftI2c.h"

#include <driver/gpio.h>

namespace oep::prototype {

bool Esp32FixtureSoftI2c::begin() {
  // A software target has no useful timing margin above its declared rate.
  if (!frequency_hz_ || frequency_hz_ > 10000) return false;
  gpio_config_t scl = {};
  scl.pin_bit_mask = uint64_t{1} << scl_pin_;
  scl.mode = GPIO_MODE_INPUT;
  if (gpio_config(&scl) != ESP_OK ||
      gpio_set_pull_mode(static_cast<gpio_num_t>(scl_pin_), GPIO_PULLUP_ONLY) != ESP_OK)
    return false;

  gpio_config_t sda = {};
  sda.pin_bit_mask = uint64_t{1} << sda_pin_;
  sda.mode = GPIO_MODE_INPUT_OUTPUT_OD;
  if (gpio_config(&sda) != ESP_OK ||
      gpio_set_pull_mode(static_cast<gpio_num_t>(sda_pin_), GPIO_PULLUP_ONLY) != ESP_OK)
    return false;
  releaseSda();
  last_sda_ = digitalRead(sda_pin_);
  last_scl_ = digitalRead(scl_pin_);
  started_ = true;
  return true;
}

bool Esp32FixtureSoftI2c::setPins(int sda_pin, int scl_pin) {
  if (sda_pin < 0 || scl_pin < 0 || sda_pin == scl_pin) return false;
  end();
  sda_pin_ = sda_pin;
  scl_pin_ = scl_pin;
  return true;
}

void Esp32FixtureSoftI2c::end() {
  if (sda_pin_ >= 0) pinMode(sda_pin_, INPUT);
  if (scl_pin_ >= 0) pinMode(scl_pin_, INPUT);
  started_ = false;
  phase_ = Phase::Idle;
}

void Esp32FixtureSoftI2c::releaseSda() {
  gpio_set_level(static_cast<gpio_num_t>(sda_pin_), 1);
}

void Esp32FixtureSoftI2c::pullSdaLow() {
  gpio_set_level(static_cast<gpio_num_t>(sda_pin_), 0);
}

void Esp32FixtureSoftI2c::start() {
  phase_ = Phase::Receive;
  bit_count_ = 0;
  byte_ = 0;
  selected_ = false;
  acknowledge_ = false;
  releaseSda();
}

void Esp32FixtureSoftI2c::stop() {
  phase_ = Phase::Idle;
  selected_ = false;
  releaseSda();
}

void Esp32FixtureSoftI2c::receiveBit(uint8_t bit) {
  byte_ = static_cast<uint8_t>((byte_ << 1) | bit);
  if (++bit_count_ != 8) return;

  if (!selected_) {
    // Match only write addressing. A read response is intentionally omitted
    // from this first diagnostic implementation.
    selected_ = (byte_ >> 1) == address_ && !(byte_ & 1);
    acknowledge_ = selected_;
    if (selected_) ++rx_transactions_;
  } else {
    acknowledge_ = true;
    if (last_rx_length_ != 0xff) ++last_rx_length_;
  }
  phase_ = Phase::AckPrepare;
}

void Esp32FixtureSoftI2c::service() {
  if (!started_) return;
  const bool sda = digitalRead(sda_pin_);
  const bool scl = digitalRead(scl_pin_);

  if (last_sda_ && !sda && scl) start();
  if (!last_sda_ && sda && scl) stop();

  if (!last_scl_ && scl && phase_ == Phase::Receive) receiveBit(sda);
  if (last_scl_ && !scl && phase_ == Phase::AckPrepare) {
    if (acknowledge_) pullSdaLow(); else releaseSda();
    phase_ = Phase::AckHigh;
  } else if (last_scl_ && !scl && phase_ == Phase::AckHigh) {
    releaseSda();
    phase_ = Phase::Receive;
    bit_count_ = 0;
    byte_ = 0;
  }
  last_sda_ = sda;
  last_scl_ = scl;
}

BackendResult Esp32FixtureSoftI2c::getStatus(FixtureI2cStatus& status) {
  if (!started_) return BackendResult::Unavailable;
  status.flags = 0x09;
  if (digitalRead(scl_pin_)) status.flags |= 0x02;
  if (digitalRead(sda_pin_)) status.flags |= 0x04;
  status.last_rx_length = last_rx_length_;
  status.rx_transactions = rx_transactions_;
  status.request_transactions = 0;
  status.frequency_hz = frequency_hz_;
  return BackendResult::Success;
}

}  // namespace oep::prototype
