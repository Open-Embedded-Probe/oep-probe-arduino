#include "Esp32FixtureRmtCapture.h"

namespace oep::prototype {
namespace {

constexpr rmt_receive_config_t kReceiveConfig = {
    .signal_range_min_ns = 50,
    // I2C 10 kHz has a 100 us period; terminate after a quiet half-period.
    .signal_range_max_ns = 50'000,
    .flags = {},
};

}  // namespace

bool Esp32FixtureRmtCapture::configure(uint8_t clock_pin, uint8_t data_pin) {
  if (active_ || clock_pin == data_pin) return false;
  clock_.pin = clock_pin;
  data_.pin = data_pin;
  rmt_rx_channel_config_t config = {
      .gpio_num = static_cast<gpio_num_t>(clock_pin),
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = kResolutionHz,
      .mem_block_symbols = kMaximumSymbols,
      .intr_priority = 0,
      .flags = {},
  };
  if (rmt_new_rx_channel(&config, &clock_.channel) != ESP_OK) {
    end();
    return false;
  }
  config.gpio_num = static_cast<gpio_num_t>(data_pin);
  if (rmt_new_rx_channel(&config, &data_.channel) != ESP_OK) {
    end();
    return false;
  }
  const rmt_rx_event_callbacks_t callbacks = {.on_recv_done = received};
  if (rmt_rx_register_event_callbacks(clock_.channel, &callbacks, this) != ESP_OK ||
      rmt_rx_register_event_callbacks(data_.channel, &callbacks, this) != ESP_OK ||
      rmt_enable(clock_.channel) != ESP_OK || rmt_enable(data_.channel) != ESP_OK ||
      !start(clock_) || !start(data_)) {
    end();
    return false;
  }
  active_ = true;
  return true;
}

bool Esp32FixtureRmtCapture::start(Slot& slot) {
  slot.symbols = 0;
  slot.complete = false;
  slot.partial = false;
  return rmt_receive(slot.channel, slot.buffer, sizeof(slot.buffer),
                     &kReceiveConfig) == ESP_OK;
}

bool Esp32FixtureRmtCapture::received(
    rmt_channel_handle_t channel, const rmt_rx_done_event_data_t* event,
    void* context) {
  auto* self = static_cast<Esp32FixtureRmtCapture*>(context);
  Slot* slot = channel == self->clock_.channel ? &self->clock_ :
      channel == self->data_.channel ? &self->data_ : nullptr;
  if (!slot) return false;
  // The driver owns event memory, but `slot.buffer` was supplied by us. Do
  // not call OEP/Arduino APIs from this ISR callback.
  slot->symbols = event->num_symbols > kMaximumSymbols ? kMaximumSymbols :
      static_cast<uint8_t>(event->num_symbols);
  slot->partial = !event->flags.is_last || event->num_symbols > kMaximumSymbols;
  slot->complete = true;
  return false;
}

void Esp32FixtureRmtCapture::stop(Slot& slot) {
  if (slot.channel) {
    rmt_disable(slot.channel);
    rmt_del_channel(slot.channel);
    slot.channel = nullptr;
  }
  if (slot.pin <= 54) pinMode(slot.pin, INPUT);
  slot.symbols = 0;
  slot.complete = false;
  slot.partial = false;
}

void Esp32FixtureRmtCapture::end() {
  stop(clock_);
  stop(data_);
  active_ = false;
}

BackendResult Esp32FixtureRmtCapture::getStatus(FixtureCaptureStatus& status) {
  if (!active_) return BackendResult::Unavailable;
  status.flags = 0x01;
  if (digitalRead(clock_.pin)) status.flags |= 0x02;
  if (digitalRead(data_.pin)) status.flags |= 0x04;
  if (clock_.complete) status.flags |= 0x08;
  if (data_.complete) status.flags |= 0x10;
  if (clock_.partial) status.flags |= 0x20;
  if (data_.partial) status.flags |= 0x40;
  status.clock_symbols = clock_.symbols;
  status.data_symbols = data_.symbols;
  status.resolution_hz = kResolutionHz;
  return BackendResult::Success;
}

BackendResult Esp32FixtureRmtCapture::readSymbols(
    uint8_t role_id, uint8_t offset, uint8_t maximum, uint32_t* output,
    size_t& count) {
  if (!active_ || !maximum || role_id < 1 || role_id > 2)
    return BackendResult::Unavailable;
  Slot& slot = role_id == 1 ? clock_ : data_;
  if (offset >= slot.symbols) {
    count = 0;
    return BackendResult::Success;
  }
  count = min<size_t>(maximum, slot.symbols - offset);
  for (size_t index = 0; index < count; ++index)
    output[index] = slot.buffer[offset + index].val;
  return BackendResult::Success;
}

}  // namespace oep::prototype
