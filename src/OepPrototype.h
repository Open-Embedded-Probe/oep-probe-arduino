#pragma once

#include <Arduino.h>

namespace oep::prototype {

constexpr size_t kMaximumMessage = 96;
constexpr size_t kMaximumWire = kMaximumMessage + 5;

enum FunctionReference : uint16_t {
  ProbeInfo = 0x0001,
  PinMatrix = 0x0002,
  TargetControl = 0x0101,
  TargetMemory = 0x0102,
  TargetFlash = 0x0103,
  FixtureGpio = 0x0201,
  FixtureUart = 0x0202,
  FixtureI2c = 0x0203,
  FixtureSpi = 0x0204,
};

enum ProbeInfoOperation : uint8_t { ProbeInfoGet = 0x01 };
enum PinMatrixOperation : uint8_t { PinMatrixGet = 0x01 };

enum class BackendResult : uint8_t {
  Success,
  Failed,
  Unavailable,
};

// Deliberately compact: fixed width makes this safe inside the prototype's
// 96-byte envelope. Pin-level details remain a separate, paged capability.
struct ProbeInfoStatus {
  uint32_t profile_id = 0;
  uint32_t firmware_revision = 0;
  uint64_t reserved_pin_mask = 0;
  uint64_t fixture_pin_mask = 0;
};

class ProbeInfoBackend {
 public:
  virtual ~ProbeInfoBackend() = default;
  virtual BackendResult getInfo(ProbeInfoStatus& status) = 0;
};

struct PinMatrixEntry { uint8_t signal, dut_port, dut_pin, probe_pin, flags, resource; };
class PinMatrixBackend {
 public:
  virtual ~PinMatrixBackend() = default;
  virtual BackendResult getPin(uint8_t signal, PinMatrixEntry& entry) = 0;
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
  // Revision 2 streaming path: four aligned fragments are buffered in probe
  // RAM, then one physical 256-byte erase/program transaction commits them.
  TargetStagePage64 = 0x02,
  TargetCommitPage256 = 0x03,
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

enum FixtureI2cOperation : uint8_t {
  FixtureI2cGetStatus = 0x01,
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
  virtual BackendResult stagePage64(uint32_t, const uint8_t*, uint8_t&) {
    return BackendResult::Unavailable;
  }
  virtual BackendResult commitPage256(uint32_t, uint8_t&) {
    return BackendResult::Unavailable;
  }
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

// This intentionally reports peer state only. I2C configuration and capture
// operations are added once their timing and trace format are fixed.
struct FixtureI2cStatus {
  // bit 0: peer started, bit 1: SCL high, bit 2: SDA high,
  // bit 3: software peer (rather than a hardware I2C peripheral).
  uint8_t flags = 0;
  uint8_t last_rx_length = 0;
  uint16_t rx_transactions = 0;
  uint16_t request_transactions = 0;
  uint32_t frequency_hz = 0;
};

class FixtureI2cBackend {
 public:
  virtual ~FixtureI2cBackend() = default;
  virtual BackendResult getStatus(FixtureI2cStatus& status) = 0;
};

class Endpoint {
 public:
  explicit Endpoint(Stream& stream, TargetControlBackend* target = nullptr,
                    TargetMemoryBackend* memory = nullptr,
                    TargetFlashBackend* flash = nullptr,
                    FixtureGpioBackend* gpio = nullptr,
                    FixtureUartBackend* uart = nullptr,
                    FixtureI2cBackend* i2c = nullptr,
                    ProbeInfoBackend* info = nullptr, PinMatrixBackend* pins = nullptr)
      : stream_(stream), target_(target), memory_(memory), flash_(flash),
        gpio_(gpio), uart_(uart), i2c_(i2c), info_(info), pins_(pins) {}
  void poll();
  bool idleFor(uint32_t milliseconds) const;

 private:
  Stream& stream_;
  TargetControlBackend* target_;
  TargetMemoryBackend* memory_;
  TargetFlashBackend* flash_;
  FixtureGpioBackend* gpio_;
  FixtureUartBackend* uart_;
  FixtureI2cBackend* i2c_;
  ProbeInfoBackend* info_;
  PinMatrixBackend* pins_;
  uint8_t encoded_[kMaximumWire]{};
  size_t encoded_length_ = 0;
  bool discard_ = false;
  uint32_t last_request_millis_ = 0;

  void consumeFrame();
  void handleMessage(uint8_t* message, size_t length);
  void handleCoreRequest(uint8_t* message, size_t length);
  void handleFunctionRequest(uint8_t* message, size_t length);
  void sendMessage(const uint8_t* message, size_t length);
};

}  // namespace oep::prototype
