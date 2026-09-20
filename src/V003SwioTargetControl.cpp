#include "V003SwioTargetControl.h"

#if defined(ARDUINO_ARCH_ESP32)

#include "driver/gpio.h"
#include "soc/gpio_struct.h"

namespace oep::prototype {
namespace {

constexpr int kCoefficient = 10;
constexpr uint8_t kSwioPin = 16;
constexpr uint32_t kSwioMask = 1u << kSwioPin;
constexpr uint8_t kData0 = 0x04;
constexpr uint8_t kData1 = 0x05;
constexpr uint8_t kDmControl = 0x10;
constexpr uint8_t kDmStatus = 0x11;
constexpr uint8_t kDmHartInfo = 0x12;
constexpr uint8_t kDmAbstractCs = 0x16;
constexpr uint8_t kDmCommand = 0x17;
constexpr uint8_t kDmAbstractAuto = 0x18;
constexpr uint8_t kProgBuf0 = 0x20;
constexpr uint8_t kDmCfgr = 0x7d;
constexpr uint8_t kDmShadowCfgr = 0x7e;
constexpr uint32_t kCfgr = 0x5aa50400;

portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

inline void IRAM_ATTR waitCycles(int count) {
  asm volatile("1: addi %[n], %[n], -1\n   bbci %[n], 31, 1b\n"
               : [n] "+r"(count));
}
inline void IRAM_ATTR low() { GPIO.out_w1tc = kSwioMask; }
inline void IRAM_ATTR high() { GPIO.out_w1ts = kSwioMask; }
inline void IRAM_ATTR outputOn() { GPIO.enable_w1ts = kSwioMask; }
inline void IRAM_ATTR outputOff() { GPIO.enable_w1tc = kSwioMask; }

inline void IRAM_ATTR sendOne(int coefficient) {
  low(); waitCycles(coefficient); high(); waitCycles(coefficient);
}

inline void IRAM_ATTR sendZero(int coefficient) {
  low(); waitCycles(coefficient * 4); high(); waitCycles(coefficient);
}

inline int IRAM_ATTR readBit(int coefficient) {
  low();
  waitCycles(coefficient);
  outputOff();
  high();
  waitCycles(coefficient / 2);
  outputOn();
  outputOff();
  waitCycles(coefficient / 2);
  const int sampled = (GPIO.in & kSwioMask) != 0;
  if (!sampled) {
    waitCycles(coefficient * 2);
    outputOn();
    outputOff();
  }
  for (int timeout = 0; timeout < 1000; ++timeout) {
    if (GPIO.in & kSwioMask) {
      outputOn();
      waitCycles(coefficient / 2);
      return sampled;
    }
  }
  outputOn();
  return 2;
}

void IRAM_ATTR writeDmi(uint8_t address, uint32_t value) {
  high();
  outputOn();
  portENTER_CRITICAL(&gMux);
  sendOne(kCoefficient);
  for (uint8_t mask = 0x40; mask; mask >>= 1)
    (address & mask) ? sendOne(kCoefficient) : sendZero(kCoefficient);
  sendOne(kCoefficient);
  for (uint32_t mask = 0x80000000u; mask; mask >>= 1)
    (value & mask) ? sendOne(kCoefficient) : sendZero(kCoefficient);
  portEXIT_CRITICAL(&gMux);
  delayMicroseconds(8);
}

int IRAM_ATTR readDmi(uint8_t address, uint32_t* value) {
  high();
  outputOn();
  uint32_t result = 0;
  portENTER_CRITICAL(&gMux);
  sendOne(kCoefficient);
  for (uint8_t mask = 0x40; mask; mask >>= 1)
    (address & mask) ? sendOne(kCoefficient) : sendZero(kCoefficient);
  sendZero(kCoefficient);
  for (int bit = 0; bit < 32; ++bit) {
    result <<= 1;
    const int decoded = readBit(kCoefficient);
    if (decoded == 2) {
      portEXIT_CRITICAL(&gMux);
      return -21;
    }
    result |= decoded;
  }
  portEXIT_CRITICAL(&gMux);
  delayMicroseconds(8);
  *value = result;
  return 0;
}

void configureIo() {
  gpio_config_t config = {};
  config.pin_bit_mask = uint64_t{1} << kSwioPin;
  config.mode = GPIO_MODE_INPUT_OUTPUT;
  config.pull_up_en = GPIO_PULLUP_ENABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&config);
  high();
  outputOn();
}

int waitAbstract() {
  for (int timeout = 0; timeout < 1000; ++timeout) {
    uint32_t value = 0;
    const int result = readDmi(kDmAbstractCs, &value);
    if (result) return result;
    if (!(value & (1u << 12))) {
      const int error = (value >> 8) & 7;
      if (error) {
        writeDmi(kDmAbstractCs, 0x00000700);
        return -30 - error;
      }
      return 0;
    }
  }
  return -8;
}

bool attachAndHalt() {
  pinMode(kSwioPin, INPUT_PULLUP);
  delay(2);
  if (!digitalRead(kSwioPin)) return false;
  configureIo();
  writeDmi(kDmShadowCfgr, kCfgr);
  writeDmi(kDmCfgr, kCfgr);
  writeDmi(kDmShadowCfgr, kCfgr);
  writeDmi(kDmCfgr, kCfgr);
  writeDmi(kDmControl, 1);
  writeDmi(kDmControl, 1);
  uint32_t configuration = 0;
  if (readDmi(kDmCfgr, &configuration) ||
      (configuration & 0xffff0000u) != 0x5aa50000u) return false;

  writeDmi(kDmControl, 0x80000001);
  writeDmi(kDmControl, 0x80000001);
  for (int timeout = 0; timeout < 100; ++timeout) {
    uint32_t status = 0;
    if (readDmi(kDmStatus, &status)) return false;
    if ((status & 0x00000300u) == 0x00000300u) return true;
    delay(1);
  }
  return false;
}

int prepareWordWriter() {
  uint32_t hart_info = 0;
  int result = readDmi(kDmHartInfo, &hart_info);
  if (result) return result;
  const uint32_t data0_address = 0xe0000000u | (hart_info & 0x7ffu);
  writeDmi(kDmAbstractAuto, 0);
  writeDmi(kData0, data0_address);
  writeDmi(kDmCommand, 0x0023100a);
  if ((result = waitAbstract())) return result;
  writeDmi(kData0, data0_address + 4);
  writeDmi(kDmCommand, 0x0023100b);
  if ((result = waitAbstract())) return result;
  writeDmi(kProgBuf0 + 0, 0x41844100);
  writeDmi(kProgBuf0 + 1, 0x0491c080);
  writeDmi(kProgBuf0 + 2, 0x9002c184);
  return 0;
}

int writeMemoryWord(uint32_t address, uint32_t value) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    writeDmi(kData1, address);
    writeDmi(kData0, value);
    writeDmi(kDmCommand, 0x00240000);
    const int result = waitAbstract();
    if (!result) return 0;
    if (prepareWordWriter()) return result;
  }
  return -1;
}

int readMemoryWord(uint32_t address, uint32_t* value) {
  writeDmi(kDmAbstractAuto, 0);
  writeDmi(kProgBuf0 + 0, 0x40044180);
  writeDmi(kProgBuf0 + 1, 0xc1040001);
  writeDmi(kProgBuf0 + 2, 0x9002c180);
  writeDmi(kData1, address);
  writeDmi(kDmCommand, 0x00240000);
  int result = waitAbstract();
  if (!result) result = readDmi(kData0, value);
  if (!result) result = prepareWordWriter();
  return result;
}

bool waitFlashIdle(uint32_t* last_status = nullptr) {
  uint32_t status = 0;
  for (int attempt = 0; attempt < 400; ++attempt) {
    if (readMemoryWord(0x4002200c, &status)) continue;
    if (!(status & 1u)) {
      if (last_status) *last_status = status;
      for (int restore = 0; restore < 5; ++restore)
        if (!prepareWordWriter()) return true;
      return false;
    }
  }
  if (last_status) *last_status = status;
  return false;
}

bool flashWriteWord(uint32_t address, uint32_t value) {
  for (int attempt = 0; attempt < 5; ++attempt) {
    writeDmi(kData1, address);
    writeDmi(kData0, value);
    uint32_t address_read = 0;
    uint32_t value_read = 0;
    if (readDmi(kData1, &address_read) || readDmi(kData0, &value_read) ||
        address_read != address || value_read != value) continue;
    writeDmi(kDmCommand, 0x00240000);
    if (!waitAbstract()) {
      for (int poll = 0; poll < 5; ++poll) {
        uint32_t next_address = 0;
        if (!readDmi(kData1, &next_address) &&
            next_address == address + 4u) return true;
      }
    }
    prepareWordWriter();
  }
  return false;
}

int injectWords(const uint32_t* words, size_t count) {
  for (size_t index = 0; index < count; ++index) {
    const uint32_t address = 0x20000000u + index * 4u;
    bool verified = false;
    for (int attempt = 0; attempt < 5; ++attempt) {
      uint32_t actual = 0;
      if (!writeMemoryWord(address, words[index]) &&
          !readMemoryWord(address, &actual) && actual == words[index]) {
        verified = true;
        break;
      }
    }
    if (!verified) return -1;
  }
  return 0;
}

const uint32_t kNormalizeUserReset[] = {
  0x400222b7, 0x00428293, 0x45670337, 0x12330313, 0x0062a023,
  0xcdef9337, 0x9ab30313, 0x0062a023, 0x400222b7, 0x02428293,
  0x45670337, 0x12330313, 0x0062a023, 0xcdef9337, 0x9ab30313,
  0x0062a023, 0x400222b7, 0x02828293, 0x45670337, 0x12330313,
  0x0062a023, 0xcdef9337, 0x9ab30313, 0x0062a023, 0x400222b7,
  0x00c28293, 0x0002a303, 0xffffc3b7, 0xfff38393, 0x00737333,
  0x0062a023, 0xe000e2b7, 0x04828293, 0xbeef0337, 0x08030313,
  0x0062a023, 0x0000006f,
};

const uint32_t kPrepareBootAndReset[] = {
  0x400222b7, 0x00428293, 0x45670337, 0x12330313, 0x0062a023,
  0xcdef9337, 0x9ab30313, 0x0062a023, 0x400222b7, 0x02428293,
  0x45670337, 0x12330313, 0x0062a023, 0xcdef9337, 0x9ab30313,
  0x0062a023, 0x400222b7, 0x02828293, 0x45670337, 0x12330313,
  0x0062a023, 0xcdef9337, 0x9ab30313, 0x0062a023, 0x400222b7,
  0x00c28293, 0x0002a303, 0xffffc3b7, 0xfff38393, 0x00737333,
  0x000043b7, 0x00736333, 0x0062a023, 0x400212b7, 0x01828293,
  0x0002a303, 0x02036313, 0x0062a023, 0x400112b7, 0x40028293,
  0x0002a303, 0xfff103b7, 0xfff38393, 0x00737333, 0x000303b7,
  0x00736333, 0x0062a023, 0x400112b7, 0x41428293, 0x01000313,
  0x0062a023, 0x004c52b7, 0xb4028293, 0xfff28293, 0xfe029ee3,
  0xe000e2b7, 0x04828293, 0xbeef0337, 0x08030313, 0x0062a023,
  0x0000006f,
};

}  // namespace

void V003SwioTargetControl::begin() {
  // The proven classic-ESP32 PHY depends on GPIO16 being constant-folded in
  // the cycle-sensitive hot path. Other pins need separately measured timing.
  pinMode(kSwioPin, INPUT_PULLUP);
}

BackendResult V003SwioTargetControl::getStatus(TargetStatus& status) {
  if (!attachAndHalt() || prepareWordWriter()) return BackendResult::Failed;
  uint32_t flash_status = 0;
  if (readMemoryWord(0x4002200c, &flash_status)) return BackendResult::Failed;
  status.flags = 0x03;  // attached and halted by this operation
  status.start_mode = (flash_status >> 14) & 1u;
  status.boot_status = (flash_status >> 13) & 1u;
  return BackendResult::Success;
}

bool V003SwioTargetControl::runPayload(const uint32_t* words, size_t count) {
  if (!attachAndHalt() || prepareWordWriter() || injectWords(words, count))
    return false;
  writeDmi(kDmAbstractAuto, 0);
  writeDmi(kData0, 0x20000000);
  writeDmi(kDmCommand, 0x002307b1);
  if (waitAbstract()) return false;
  writeDmi(kDmControl, 0x40000001);
  writeDmi(kDmControl, 0x40000001);
  writeDmi(kDmControl, 1);
  return true;
}

BackendResult V003SwioTargetControl::normalizeUser() {
  return runPayload(kNormalizeUserReset,
                    sizeof(kNormalizeUserReset) / sizeof(kNormalizeUserReset[0]))
      ? BackendResult::Success : BackendResult::Failed;
}

BackendResult V003SwioTargetControl::enterProductBootloader() {
  return runPayload(kPrepareBootAndReset,
                    sizeof(kPrepareBootAndReset) / sizeof(kPrepareBootAndReset[0]))
      ? BackendResult::Success : BackendResult::Failed;
}

BackendResult V003SwioTargetControl::readMemory(
    uint32_t address, uint8_t* output, size_t length) {
  if (!attachAndHalt() || prepareWordWriter()) return BackendResult::Failed;
  for (size_t offset = 0; offset < length; offset += 4) {
    uint32_t value = 0;
    if (readMemoryWord(address + offset, &value))
      return BackendResult::Failed;
    output[offset] = value;
    output[offset + 1] = value >> 8;
    output[offset + 2] = value >> 16;
    output[offset + 3] = value >> 24;
  }
  return BackendResult::Success;
}

BackendResult V003SwioTargetControl::programPage64(
    uint32_t address, const uint8_t* data, uint8_t& diagnostic) {
  diagnostic = 0;
  if ((address & 63u) || address < 0x08000000u ||
      address >= 0x08004000u) return BackendResult::Unavailable;
  if (!attachAndHalt() || prepareWordWriter()) {
    diagnostic = 1;
    return BackendResult::Failed;
  }

  bool already_matches = true;
  for (uint32_t offset = 0; offset < 64; offset += 4) {
    const uint32_t expected = data[offset] |
        static_cast<uint32_t>(data[offset + 1]) << 8 |
        static_cast<uint32_t>(data[offset + 2]) << 16 |
        static_cast<uint32_t>(data[offset + 3]) << 24;
    uint32_t actual = 0;
    if (readMemoryWord(address + offset, &actual) || actual != expected) {
      already_matches = false;
      break;
    }
  }
  if (already_matches) return BackendResult::Success;

  const uint32_t unlock[][2] = {
      {0x40022004u, 0x45670123u}, {0x40022004u, 0xcdef89abu},
      {0x40022024u, 0x45670123u}, {0x40022024u, 0xcdef89abu},
  };
  for (const auto& item : unlock)
    if (!flashWriteWord(item[0], item[1])) {
      diagnostic = 2;
      return BackendResult::Failed;
    }

  uint32_t status = 0;
  if (!waitFlashIdle(&status) ||
      !flashWriteWord(0x40022010u, 0x00020000u) ||
      !flashWriteWord(0x40022014u, address) ||
      !flashWriteWord(0x40022010u, 0x00020040u) ||
      !waitFlashIdle(&status) ||
      !flashWriteWord(0x40022010u, 0x00010000u) ||
      !flashWriteWord(0x40022010u, 0x00090000u) ||
      !waitFlashIdle(&status)) {
    diagnostic = 3;
    return BackendResult::Failed;
  }

  for (uint32_t offset = 0; offset < 64; offset += 4) {
    const uint32_t word = data[offset] |
        static_cast<uint32_t>(data[offset + 1]) << 8 |
        static_cast<uint32_t>(data[offset + 2]) << 16 |
        static_cast<uint32_t>(data[offset + 3]) << 24;
    if (!flashWriteWord(address + offset, word) ||
        !flashWriteWord(0x40022010u, 0x00050000u) ||
        !waitFlashIdle(&status)) {
      diagnostic = 5 + offset / 4;
      return BackendResult::Failed;
    }
  }
  if (!flashWriteWord(0x40022014u, address) ||
      !flashWriteWord(0x40022010u, 0x00010040u) ||
      !waitFlashIdle(&status) ||
      !flashWriteWord(0x40022010u, 0)) {
    diagnostic = 21;
    return BackendResult::Failed;
  }

  for (uint32_t offset = 0; offset < 64; offset += 4) {
    const uint32_t expected = data[offset] |
        static_cast<uint32_t>(data[offset + 1]) << 8 |
        static_cast<uint32_t>(data[offset + 2]) << 16 |
        static_cast<uint32_t>(data[offset + 3]) << 24;
    uint32_t actual = 0;
    bool verified = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (!readMemoryWord(address + offset, &actual) && actual == expected) {
        verified = true;
        break;
      }
    }
    if (!verified) {
      diagnostic = 22 + offset / 4;
      return BackendResult::Failed;
    }
  }
  return BackendResult::Success;
}

}  // namespace oep::prototype

#else

namespace oep::prototype {
void V003SwioTargetControl::begin() {}
BackendResult V003SwioTargetControl::getStatus(TargetStatus&) {
  return BackendResult::Unavailable;
}
BackendResult V003SwioTargetControl::normalizeUser() {
  return BackendResult::Unavailable;
}
BackendResult V003SwioTargetControl::enterProductBootloader() {
  return BackendResult::Unavailable;
}
BackendResult V003SwioTargetControl::readMemory(
    uint32_t, uint8_t*, size_t) {
  return BackendResult::Unavailable;
}
BackendResult V003SwioTargetControl::programPage64(
    uint32_t, const uint8_t*, uint8_t&) {
  return BackendResult::Unavailable;
}
bool V003SwioTargetControl::runPayload(const uint32_t*, size_t) { return false; }
}  // namespace oep::prototype

#endif
