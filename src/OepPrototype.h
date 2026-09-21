#pragma once

#include <Arduino.h>

namespace oep::prototype {

constexpr size_t kMaximumMessage = 96;
constexpr size_t kMaximumWire = kMaximumMessage + 5;

enum FunctionReference : uint16_t {
  ProbeInfo = 0x0001,
  ProbeCapabilities = 0x0002,
  ProbeConfiguration = 0x0003,
  TargetControl = 0x0101,
  TargetMemory = 0x0102,
  TargetFlash = 0x0103,
  FixtureGpio = 0x0201,
  FixtureUart = 0x0202,
  FixtureI2c = 0x0203,
  FixtureSpi = 0x0204,
  FixtureCapture = 0x0205,
};

enum ProbeInfoOperation : uint8_t { ProbeInfoGet = 0x01 };
enum ProbeCapabilitiesOperation : uint8_t {
  ProbeCapabilitiesGetSummary = 0x01,
  ProbeCapabilitiesGetChannel = 0x02,
  ProbeCapabilitiesGetGroup = 0x03,
  ProbeCapabilitiesGetVoltageDomain = 0x04,
  ProbeCapabilitiesGetGroupRole = 0x05,
};

// Configuration is deliberately separate from capability discovery.  A host
// first resolves its own connection manifest against ProbeCapabilities, then
// atomically supplies the resulting (group, function, probe-channel) roles.
// No target-board name or pin number appears on this wire interface.
enum ProbeConfigurationOperation : uint8_t {
  ProbeConfigurationApply = 0x01,
  ProbeConfigurationRelease = 0x02,
};

enum ProbeChannelFunction : uint8_t {
  ProbeGpioIn = 0,
  ProbeGpioOut = 1,
  ProbeOpenDrain = 2,
  ProbePullUp = 3,
  ProbePullDown = 4,
  ProbeCapture = 5,
  ProbeUartRx = 6,
  ProbeUartTx = 7,
  ProbeI2cSda = 8,
  ProbeI2cScl = 9,
  ProbeSpiRx = 10,
  ProbeSpiTx = 11,
  ProbeSpiSck = 12,
  ProbeSpiCs = 13,
  ProbePwmOut = 14,
  ProbeAnalogIn = 15,
  ProbeAnalogOut = 16,
  ProbeEdgeOut = 17,
};

enum ProbeGroupKind : uint8_t {
  ProbeGroupUart = 1,
  ProbeGroupI2cController = 2,
  ProbeGroupI2cTarget = 3,
  ProbeGroupSpiController = 4,
  ProbeGroupSpiTarget = 5,
  ProbeGroupCapture = 6,
  ProbeGroupPwm = 7,
  ProbeGroupAnalog = 8,
};

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

struct ProbeCapabilitiesSummary {
  uint8_t revision = 1;
  uint8_t channel_count = 0;
  uint8_t group_count = 0;
  uint8_t voltage_domain_count = 0;
};

struct ProbeChannelCapability {
  uint16_t id = 0;
  // bit 0: input-only, bit 1: reserved by the running probe firmware.
  uint8_t flags = 0;
  // Bit N references voltage-domain ordinal N.
  uint8_t voltage_domain_mask = 0;
  // Revision-1 function vocabulary; see docs/development-probe-functional-spec.ja.md.
  uint64_t function_mask = 0;
};

struct ProbeGroupCapability {
  uint16_t id = 0;
  uint8_t kind = 0;
  uint8_t instance = 0;
  uint64_t role_mask = 0;
  // Bit N excludes group ordinal N.
  uint64_t exclusive_group_mask = 0;
};

struct ProbeGroupRoleCapability {
  uint16_t group_id = 0;
  uint8_t role_id = 0;
  uint8_t function = 0;
};

struct ProbeVoltageDomainCapability {
  uint8_t id = 0;
  // bit 0: probe may actively drive this domain.
  uint8_t flags = 0;
  uint16_t nominal_mv = 0;
  uint16_t input_max_mv = 0;
};

class ProbeCapabilitiesBackend {
 public:
  virtual ~ProbeCapabilitiesBackend() = default;
  virtual BackendResult getSummary(ProbeCapabilitiesSummary& summary) = 0;
  virtual BackendResult getChannel(uint8_t ordinal,
                                   ProbeChannelCapability& channel) = 0;
  virtual BackendResult getGroup(uint8_t ordinal,
                                 ProbeGroupCapability& group) = 0;
  virtual BackendResult getVoltageDomain(
      uint8_t ordinal, ProbeVoltageDomainCapability& domain) = 0;
  virtual BackendResult getGroupRole(uint8_t group_ordinal,
                                     uint8_t role_ordinal,
                                     ProbeGroupRoleCapability& role) = 0;
};

struct ProbeConfigurationRole {
  uint16_t group_id = 0;
  // Zero is the revision-1 compatibility form, where function identifies a
  // role.  Revision 2 carries the stable capability role identifier.
  uint8_t role_id = 0;
  uint8_t function = 0;
  uint16_t channel_id = 0;
};

class ProbeConfigurationBackend {
 public:
  virtual ~ProbeConfigurationBackend() = default;
  // The endpoint has already bounded the plan to kMaximumMessage.  The
  // backend must validate every role before changing any peripheral state.
  virtual BackendResult apply(const ProbeConfigurationRole* roles,
                              uint8_t count, uint32_t& lease_id) = 0;
  virtual BackendResult release(uint32_t lease_id) = 0;
  // Called by the transport watchdog when the host disappears.  It must be
  // idempotent and leave every configured resource in its safe idle state.
  virtual void abandon() = 0;
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

enum FixtureCaptureOperation : uint8_t {
  FixtureCaptureGetStatus = 0x01,
  // role id, symbol offset, maximum symbol count. Raw RMT words are returned
  // little endian; semantic I2C decoding stays on the host.
  FixtureCaptureReadSymbols = 0x02,
  FixtureCaptureStart = 0x03,
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
  uint8_t stretch_cause_mask = 0;
};

class FixtureI2cBackend {
 public:
  virtual ~FixtureI2cBackend() = default;
  virtual BackendResult getStatus(FixtureI2cStatus& status) = 0;
};

class FixtureI2cLifecycle : public FixtureI2cBackend {
 public:
  virtual bool setPins(int sda_pin, int scl_pin) = 0;
  virtual bool begin() = 0;
  virtual void end() = 0;
};

struct FixtureCaptureStatus {
  // bit 0 active, 1/2: clock/data idle level, 3/4: clock/data completed,
  // 5/6: clock/data partial/overflow.
  uint8_t flags = 0;
  uint8_t clock_symbols = 0;
  uint8_t data_symbols = 0;
  uint32_t resolution_hz = 0;
};

class FixtureCaptureBackend {
 public:
  virtual ~FixtureCaptureBackend() = default;
  virtual BackendResult getStatus(FixtureCaptureStatus& status) = 0;
  virtual BackendResult readSymbols(uint8_t role_id, uint8_t offset,
                                    uint8_t maximum, uint32_t* output,
                                    size_t& count) = 0;
  virtual BackendResult startCapture() = 0;
};

class Endpoint {
 public:
  explicit Endpoint(Stream& stream, TargetControlBackend* target = nullptr,
                    TargetMemoryBackend* memory = nullptr,
                    TargetFlashBackend* flash = nullptr,
                    FixtureGpioBackend* gpio = nullptr,
                    FixtureUartBackend* uart = nullptr,
                    FixtureI2cBackend* i2c = nullptr,
                    FixtureCaptureBackend* capture = nullptr,
                    ProbeInfoBackend* info = nullptr,
                    ProbeCapabilitiesBackend* capabilities = nullptr,
                    ProbeConfigurationBackend* configuration = nullptr)
      : stream_(stream), target_(target), memory_(memory), flash_(flash),
        gpio_(gpio), uart_(uart), i2c_(i2c), capture_(capture), info_(info),
        capabilities_(capabilities), configuration_(configuration) {}
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
  FixtureCaptureBackend* capture_;
  ProbeInfoBackend* info_;
  ProbeCapabilitiesBackend* capabilities_;
  ProbeConfigurationBackend* configuration_;
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
