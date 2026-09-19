#pragma once

#include <Arduino.h>

namespace oep::prototype {

constexpr size_t kMaximumMessage = 64;
constexpr size_t kMaximumWire = kMaximumMessage + 5;

enum FunctionReference : uint16_t {
  TargetControl = 0x0101,
  TargetMemory = 0x0102,
  TargetFlash = 0x0103,
  FixtureGpio = 0x0201,
  FixtureUart = 0x0202,
  FixtureI2c = 0x0203,
  FixtureSpi = 0x0204,
};

class Endpoint {
 public:
  explicit Endpoint(Stream& stream) : stream_(stream) {}
  void poll();

 private:
  Stream& stream_;
  uint8_t encoded_[kMaximumWire]{};
  size_t encoded_length_ = 0;
  bool discard_ = false;

  void consumeFrame();
  void handleMessage(uint8_t* message, size_t length);
  void sendMessage(const uint8_t* message, size_t length);
};

}  // namespace oep::prototype
