// OEP v0 endpoint: reads frames, routes requests to services, answers with
// results. Requests are processed in order; the host keeps outstanding bytes
// within the declared window.
#pragma once

#include <Arduino.h>

#include "OepFrame.h"
#include "OepService.h"

namespace oep {

struct Limits {
  uint16_t max_frame;
  uint16_t window_bytes;
  uint8_t max_inflight;
};

class Endpoint {
 public:
  static constexpr size_t kMaxServices = 16;
  static constexpr size_t kRequestHeader = 6;  // role, corr16, fn16, op
  static constexpr size_t kResultHeader = 5;   // role, corr16, resolution, detail

  Endpoint(Stream &stream, uint8_t *rx_buffer, size_t rx_capacity,
           uint8_t *tx_buffer, size_t tx_capacity, Limits limits)
      : stream_(stream), reader_(rx_buffer, rx_capacity, limits.max_frame),
        tx_(tx_buffer), tx_capacity_(tx_capacity), limits_(limits) {}

  // Services receive fn = 1, 2, ... in registration order. fn 0 is the core.
  bool addService(Service &service);
  void poll();
  bool idleFor(uint32_t milliseconds) const;
  void abandonAll();
  uint32_t droppedFrames() const { return reader_.dropped(); }
  uint32_t requests() const { return requests_; }

 private:
  Stream &stream_;
  FrameReader reader_;
  uint8_t *tx_;
  size_t tx_capacity_;
  Limits limits_;
  Service *services_[kMaxServices] = {};
  size_t service_count_ = 0;
  uint16_t active_lease_ = 0;   // one lease per connection in v0
  uint16_t next_lease_ = 1;
  bool leased_[kMaxServices] = {};
  uint32_t last_request_millis_ = 0;
  uint32_t requests_ = 0;

  void handleMessage(const uint8_t *message, size_t length);
  Result handleCore(uint8_t operation, const uint8_t *payload, size_t length,
                    uint8_t *out, size_t capacity);
  Result planApply(const uint8_t *tlv, size_t length, uint8_t *out, size_t capacity);
  void releaseLease();
  void sendResult(uint16_t correlation, Result result);
};

}  // namespace oep
