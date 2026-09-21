#pragma once

#include <driver/rmt_rx.h>

#include "OepPrototype.h"

namespace oep::prototype {

// Two independent, input-only RMT traces. This intentionally does not decode
// a bus protocol: the raw records retain enough information for a host to
// distinguish an electrical NACK from a peripheral callback artifact.
class Esp32FixtureRmtCapture final : public FixtureCaptureBackend {
 public:
  static constexpr uint8_t kMaximumSymbols = 16;
  static constexpr uint32_t kResolutionHz = 10'000'000;

  bool configure(uint8_t clock_pin, uint8_t data_pin);
  BackendResult startCapture() override;
  void end();
  BackendResult getStatus(FixtureCaptureStatus& status) override;
  BackendResult readSymbols(uint8_t role_id, uint8_t offset,
                            uint8_t maximum, uint32_t* output,
                            size_t& count) override;

 private:
  struct Slot {
    rmt_channel_handle_t channel = nullptr;
    rmt_symbol_word_t buffer[kMaximumSymbols]{};
    volatile uint8_t symbols = 0;
    volatile bool complete = false;
    volatile bool partial = false;
    uint8_t pin = 0;
  };

  static bool received(rmt_channel_handle_t channel,
                       const rmt_rx_done_event_data_t* event, void* context);
  bool start(Slot& slot);
  void stop(Slot& slot);
  Slot clock_;
  Slot data_;
  bool active_ = false;
};

}  // namespace oep::prototype
