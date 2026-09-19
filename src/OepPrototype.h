#pragma once

#include <Arduino.h>

namespace oep::prototype {

constexpr size_t kMaximumMessage = 96;
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

enum TargetControlOperation : uint8_t {
  TargetGetStatus = 0x01,
  TargetNormalizeUser = 0x02,
  TargetEnterProductBootloader = 0x03,
};

enum TargetMemoryOperation : uint8_t {
  TargetReadMemory = 0x01,
};

enum TargetFlashOperation : uint8_t {
  TargetProgramPage64 = 0x01,
};

enum FixtureGpioOperation : uint8_t {
  FixtureReadDigital = 0x01,
};

enum class BackendResult : uint8_t {
  Success,
  Failed,
  Unavailable,
};

struct TargetStatus {
  uint8_t flags = 0;
  uint8_t start_mode = 0;
  uint8_t boot_status = 0;
};

class TargetControlBackend {
 public:
  virtual ~TargetControlBackend() = default;
  virtual BackendResult getStatus(TargetStatus& status) = 0;
  virtual BackendResult normalizeUser() = 0;
  virtual BackendResult enterProductBootloader() = 0;
};

class TargetMemoryBackend {
 public:
  virtual ~TargetMemoryBackend() = default;
  virtual BackendResult readMemory(uint32_t address, uint8_t* output,
                                   size_t length) = 0;
};

class TargetFlashBackend {
 public:
  virtual ~TargetFlashBackend() = default;
  virtual BackendResult programPage64(uint32_t address,
                                      const uint8_t* data) = 0;
};

class FixtureGpioBackend {
 public:
  virtual ~FixtureGpioBackend() = default;
  virtual BackendResult readDigital(uint8_t pin, uint8_t& value) = 0;
};

class Endpoint {
 public:
  explicit Endpoint(Stream& stream, TargetControlBackend* target = nullptr,
                    TargetMemoryBackend* memory = nullptr,
                    TargetFlashBackend* flash = nullptr,
                    FixtureGpioBackend* gpio = nullptr)
      : stream_(stream), target_(target), memory_(memory), flash_(flash),
        gpio_(gpio) {}
  void poll();

 private:
  Stream& stream_;
  TargetControlBackend* target_;
  TargetMemoryBackend* memory_;
  TargetFlashBackend* flash_;
  FixtureGpioBackend* gpio_;
  uint8_t encoded_[kMaximumWire]{};
  size_t encoded_length_ = 0;
  bool discard_ = false;

  void consumeFrame();
  void handleMessage(uint8_t* message, size_t length);
  void handleCoreRequest(uint8_t* message, size_t length);
  void handleFunctionRequest(uint8_t* message, size_t length);
  void sendMessage(const uint8_t* message, size_t length);
};

}  // namespace oep::prototype
