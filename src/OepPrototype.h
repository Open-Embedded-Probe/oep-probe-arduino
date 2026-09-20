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
  FixtureReadDigitalBank = 0x02,
  FixtureConfigureDigital = 0x03,
};

enum FixtureGpioMode : uint8_t {
  FixtureInputFloating = 0x00,
  FixtureInputPullUp = 0x01,
  FixtureInputPullDown = 0x02,
  FixtureInputPullUpDown = 0x03,
  FixtureOutputLow = 0x04,
  FixtureOutputHigh = 0x05,
  FixtureOpenDrainLow = 0x06,
  FixtureOpenDrainRelease = 0x07,
};

enum FixtureUartOperation : uint8_t {
  FixtureUartConfigure = 0x01,
  FixtureUartWrite = 0x02,
  FixtureUartReadAvailable = 0x03,
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
  virtual BackendResult programPage64(uint32_t address, const uint8_t* data,
                                      uint8_t& diagnostic) = 0;
};

class FixtureGpioBackend {
 public:
  virtual ~FixtureGpioBackend() = default;
  virtual BackendResult readDigital(uint8_t pin, uint8_t& value) = 0;
  virtual BackendResult readDigitalBank(uint64_t& available, uint64_t& values);
  virtual BackendResult configureDigital(uint8_t pin, uint8_t mode) = 0;
};

class FixtureUartBackend {
 public:
  virtual ~FixtureUartBackend() = default;
  virtual BackendResult configure(uint32_t requested_baud,
                                  uint32_t& actual_baud) = 0;
  virtual BackendResult writeBytes(const uint8_t* data, size_t length,
                                   size_t& written) = 0;
  virtual BackendResult readAvailable(uint8_t* output, size_t capacity,
                                      size_t& length) = 0;
};

class Endpoint {
 public:
  explicit Endpoint(Stream& stream, TargetControlBackend* target = nullptr,
                    TargetMemoryBackend* memory = nullptr,
                    TargetFlashBackend* flash = nullptr,
                    FixtureGpioBackend* gpio = nullptr,
                    FixtureUartBackend* uart = nullptr)
      : stream_(stream), target_(target), memory_(memory), flash_(flash),
        gpio_(gpio), uart_(uart) {}
  void poll();

 private:
  Stream& stream_;
  TargetControlBackend* target_;
  TargetMemoryBackend* memory_;
  TargetFlashBackend* flash_;
  FixtureGpioBackend* gpio_;
  FixtureUartBackend* uart_;
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
