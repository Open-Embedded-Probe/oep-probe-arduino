#include "Esp32FixtureIdfI2c.h"

namespace oep::prototype {

bool Esp32FixtureIdfI2c::begin() {
  i2c_slave_config_t config = {};
  config.i2c_port = port_;
  config.sda_io_num = static_cast<gpio_num_t>(sda_pin_);
  config.scl_io_num = static_cast<gpio_num_t>(scl_pin_);
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  config.send_buf_depth = 32;
  config.slave_addr = address_;
  config.addr_bit_len = I2C_ADDR_BIT_LEN_7;
  config.flags.stretch_en = 1;
  if (i2c_new_slave_device(&config, &device_) != ESP_OK) return false;
  const i2c_slave_event_callbacks_t callbacks = { .on_recv_done = received };
  if (i2c_slave_register_event_callbacks(device_, &callbacks, this) != ESP_OK ||
      i2c_slave_receive(device_, rx_buffer_, sizeof(rx_buffer_)) != ESP_OK) {
    i2c_del_slave_device(device_);
    device_ = nullptr;
    return false;
  }
  return true;
}

bool Esp32FixtureIdfI2c::received(
    i2c_slave_dev_handle_t device, const i2c_slave_rx_done_event_data_t*,
    void* arg) {
  auto* self = static_cast<Esp32FixtureIdfI2c*>(arg);
  ++self->rx_transactions_;
  // Do not re-arm from the ISR: the IDF driver takes a mutex while installing
  // a receive job and this causes an interrupt watchdog reset. One bounded
  // receive is enough for the address-ACK diagnostic; a later task-backed
  // implementation can re-arm outside interrupt context.
  (void)device;
  return false;
}

BackendResult Esp32FixtureIdfI2c::getStatus(FixtureI2cStatus& status) {
  if (!device_) return BackendResult::Unavailable;
  noInterrupts();
  const uint16_t received = rx_transactions_;
  interrupts();
  status.flags = 0x01;
  if (digitalRead(scl_pin_)) status.flags |= 0x02;
  if (digitalRead(sda_pin_)) status.flags |= 0x04;
  status.rx_transactions = received;
  status.frequency_hz = frequency_hz_;
  return BackendResult::Success;
}

}  // namespace oep::prototype
