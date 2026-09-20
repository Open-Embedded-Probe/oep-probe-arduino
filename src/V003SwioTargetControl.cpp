#include "V003SwioTargetControl.h"

#if defined(ARDUINO_ARCH_ESP32)

#include "driver/gpio.h"
#include "soc/gpio_struct.h"

#ifndef OEP_V003_FORCE_SEQUENTIAL_FLASH
#define OEP_V003_FORCE_SEQUENTIAL_FLASH 0
#endif
#ifndef OEP_V003_INJECT_LOADER_FAILURE
#define OEP_V003_INJECT_LOADER_FAILURE 0
#endif
#ifndef OEP_V003_DISABLE_SEQUENTIAL_FALLBACK
#define OEP_V003_DISABLE_SEQUENTIAL_FALLBACK 0
#endif

namespace oep::prototype {
namespace {

// E133 measured 262.5/862.5 ns at coefficient 8, closest to the real LinkE
// capture's 240/860 ns.  It also gave 100/100 DATA1 reads; the former
// host-driven flash sequence was the operation that failed at this setting.
constexpr int kCoefficient = 8;
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

// RV32EC CH32V003 flash loader from ch32-rs/wlink (MIT/Apache-2.0), built
// from the WCH EVT flash routine.  It executes on the target, takes
// a0=operation flags, a1=flash address, a2=length, and reads input at
// 0x20000200.  The real LinkE capture uses the same load/run arrangement.
const uint32_t kV003FlashLoader[] = {
  0xcc221111u, 0xc802ca26u, 0x00157793u, 0x06b7cf99u, 0x27b74567u, 0x86934002u,
  0x97371236u, 0xc3d4cdefu, 0x9ab70713u, 0xd3d4c3d8u, 0x7793d3d8u, 0xc79d0025u,
  0x400227b7u, 0x66ad4b98u, 0x40003337u, 0x00476713u, 0x4b98cb98u, 0xaaa68693u,
  0x04076713u, 0x47d8cb98u, 0x16638b05u, 0x4b981007u, 0xcb989b6du, 0x00457793u,
  0x0793cba9u, 0x839903f6u, 0x632dc02eu, 0xc43e7681u, 0x400032b7u, 0x400227b7u,
  0xaaa30313u, 0x4b9816fdu, 0x000203b7u, 0x00776733u, 0x4702cb98u, 0x4b98cbd8u,
  0x04076713u, 0x47d8cb98u, 0xe7698b05u, 0x8f754b98u, 0x4702cb98u, 0x04070713u,
  0x4722c03au, 0xc43a177du, 0x7793f779u, 0xcff10085u, 0x03f60793u, 0x8399c02eu,
  0x40022737u, 0x4b1cc43eu, 0x632d66c1u, 0xcb1c8fd5u, 0x20000737u, 0x20070713u,
  0x400227b7u, 0x000803b7u, 0x400032b7u, 0xaaa30313u, 0xe6b34b94u, 0xcb940076u,
  0x8a8547d4u, 0x4682fef5u, 0x043784bau, 0xc2360004u, 0xc63646c1u, 0x40844692u,
  0xc2840711u, 0x8ec14b94u, 0x47d4cb94u, 0xeab18a85u, 0x84ba4692u, 0xc2360691u,
  0x16fd46b2u, 0xfef9c636u, 0xcbd44682u, 0xe6934b94u, 0xcb940406u, 0x8a8547d4u,
  0x47d4ee85u, 0xce858ac1u, 0x06b747d8u, 0x16fdfff3u, 0x01076713u, 0x4b98c7d8u,
  0x8f754521u, 0x4462cb98u, 0x017144d2u, 0x20239002u, 0xb5f500d3u, 0x0062a023u,
  0xa023b73du, 0xb7550062u, 0x0062a023u, 0x4682b7c1u, 0x04068693u, 0x46a2c036u,
  0xc43616fdu, 0x4b98f2b5u, 0xfff306b7u, 0x8f7516fdu, 0x8941cb98u, 0x4501e119u,
  0xc02ebf7du, 0xc402060du, 0x07b78209u, 0xc6322000u, 0x20078793u, 0x87134394u,
  0x47a20047u, 0x078a4602u, 0x439c97b2u, 0x02f69963u, 0x468247a2u, 0x97b6078au,
  0x47c24394u, 0xc83e97b6u, 0x078547a2u, 0x4622c43eu, 0x87ba46b2u, 0xfcd668e3u,
  0x200007b7u, 0x6107a703u, 0x06e347c2u, 0x4541faf7u, 0xffffb79du,
};

int writeRegister(uint16_t regno, uint32_t value) {
  writeDmi(kData0, value);
  writeDmi(kDmCommand, 0x00230000u | regno);
  return waitAbstract();
}

int readRegister(uint16_t regno, uint32_t* value) {
  writeDmi(kDmCommand, 0x00220000u | regno);
  int result = waitAbstract();
  if (!result) result = readDmi(kData0, value);
  return result;
}

bool sequentialWriteWord(uint32_t address, uint32_t value) {
  for (int attempt = 0; attempt < 5; ++attempt) {
    writeDmi(kData1, address);
    writeDmi(kData0, value);
    uint32_t address_read = 0;
    uint32_t value_read = 0;
    if (readDmi(kData1, &address_read) || readDmi(kData0, &value_read) ||
        address_read != address || value_read != value) continue;
    writeDmi(kDmCommand, 0x00240000u);
    if (!waitAbstract()) {
      for (int poll = 0; poll < 5; ++poll) {
        uint32_t next_address = 0;
        if (!readDmi(kData1, &next_address) &&
            next_address == address + 4u) return true;
      }
    }
    if (prepareWordWriter()) return false;
  }
  return false;
}

bool sequentialWaitFlashIdle(uint32_t* final_status = nullptr) {
  uint32_t status = 0;
  // E136 measured erase <=3.3 ms and program <=2.9 ms. One abstract status
  // read takes about 0.46 ms; 400 polls retain a very large diagnostic margin.
  for (int poll = 0; poll < 400; ++poll) {
    if (readMemoryWord(0x4002200cu, &status)) continue;
    if (!(status & 1u)) {
      if (final_status) *final_status = status;
      return !prepareWordWriter();
    }
  }
  if (final_status) *final_status = status;
  return false;
}

bool sequentialProgramPage64(uint32_t address, const uint8_t* data,
                             uint8_t& diagnostic) {
  const uint32_t unlock[][2] = {
    {0x40022004u, 0x45670123u}, {0x40022004u, 0xcdef89abu},
    {0x40022024u, 0x45670123u}, {0x40022024u, 0xcdef89abu},
  };
  for (const auto& item : unlock) {
    if (!sequentialWriteWord(item[0], item[1])) {
      diagnostic = 0x80;
      return false;
    }
  }

  uint32_t status = 0;
  if (!sequentialWriteWord(0x40022010u, 0x00020000u) ||
      !sequentialWriteWord(0x40022014u, address) ||
      !sequentialWriteWord(0x40022010u, 0x00020040u) ||
      !sequentialWaitFlashIdle(&status)) {
    diagnostic = 0x81;
    return false;
  }
  if (!sequentialWriteWord(0x40022010u, 0x00010000u) ||
      !sequentialWriteWord(0x40022010u, 0x00090000u) ||
      !sequentialWaitFlashIdle(&status)) {
    diagnostic = 0x82;
    return false;
  }
  for (uint32_t offset = 0; offset < 64; offset += 4) {
    const uint32_t word = data[offset] |
        static_cast<uint32_t>(data[offset + 1]) << 8 |
        static_cast<uint32_t>(data[offset + 2]) << 16 |
        static_cast<uint32_t>(data[offset + 3]) << 24;
    if (!sequentialWriteWord(address + offset, word) ||
        !sequentialWriteWord(0x40022010u, 0x00050000u) ||
        !sequentialWaitFlashIdle(&status)) {
      diagnostic = 0x90 + offset / 4;
      return false;
    }
  }
  if (!sequentialWriteWord(0x40022014u, address) ||
      !sequentialWriteWord(0x40022010u, 0x00010040u) ||
      !sequentialWaitFlashIdle(&status) ||
      !sequentialWriteWord(0x40022010u, 0)) {
    diagnostic = 0x83;
    return false;
  }
  return true;
}

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
  // Every control payload overwrites the loader at 0x20000000, and reset may
  // let the application reuse RAM. Never carry loader residency across it.
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

BackendResult V003SwioTargetControl::programPage64Attempt(
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

#if OEP_V003_FORCE_SEQUENTIAL_FLASH
  if (!sequentialProgramPage64(address, data, diagnostic))
    return BackendResult::Failed;
#else
#if OEP_V003_INJECT_LOADER_FAILURE == 1
  diagnostic = 0x7f;
  return BackendResult::Failed;
#endif
  if (injectWords(kV003FlashLoader,
                  sizeof(kV003FlashLoader) / sizeof(kV003FlashLoader[0]))) {
    diagnostic = 2;
    return BackendResult::Failed;
  }
  for (uint32_t offset = 0; offset < 64; offset += 4) {
    const uint32_t word = data[offset] |
        static_cast<uint32_t>(data[offset + 1]) << 8 |
        static_cast<uint32_t>(data[offset + 2]) << 16 |
        static_cast<uint32_t>(data[offset + 3]) << 24;
    if (writeMemoryWord(0x20000200u + offset, word)) {
      diagnostic = 3 + offset / 4;
      return BackendResult::Failed;
    }
  }
#if OEP_V003_INJECT_LOADER_FAILURE == 2
  diagnostic = 0x7e;
  return BackendResult::Failed;
#endif

  writeDmi(kDmAbstractAuto, 0);
  if (writeRegister(0x100a, 0x1du) ||       // unlock, erase, program, verify
      writeRegister(0x100b, address) ||
      writeRegister(0x100c, 64) ||
      writeRegister(0x0300, 0) ||           // dcsr
      writeRegister(0x1002, 0x20000800u) || // sp: top of 2 KiB RAM
      writeRegister(0x07b1, 0x20000000u)) { // dpc: loader entry
    diagnostic = 19;
    return BackendResult::Failed;
  }
  writeDmi(kDmControl, 0x40000001u);
  bool halted = false;
  const uint32_t poll_started = micros();
  for (int poll = 0; poll < 128 && micros() - poll_started < 20000u; ++poll) {
    uint32_t status = 0;
    if (!readDmi(kDmStatus, &status) &&
        (status & 0x00000300u) == 0x00000300u) {
      halted = true;
      break;
    }
  }
  // A complete 64-byte write has been observed even when every completion
  // status read was lost by software SWIO.  Re-attach/halt is therefore also
  // the completion fence, matching the independent-session verification
  // required below rather than treating a missed poll as a failed program.
  if (!halted) halted = attachAndHalt();
  if (!halted) {
    diagnostic = 20;
    return BackendResult::Failed;
  }
#if OEP_V003_INJECT_LOADER_FAILURE == 3
  diagnostic = 0x7d;
  return BackendResult::Failed;
#endif
#endif

  // programPage64() owns the single authoritative fresh-attach read-back.
  // Do not duplicate all 16 abstract reads here; the loader already verifies
  // internally and the outer check also covers the sequential path.
  return BackendResult::Success;
}

BackendResult V003SwioTargetControl::programPage64(
    uint32_t address, const uint8_t* data, uint8_t& diagnostic) {
  if ((address & 63u) || address < 0x08000000u ||
      address >= 0x08004000u) {
    diagnostic = 0;
    return BackendResult::Unavailable;
  }

  // A DMI error may leave an otherwise valid page only partly programmed.
  // Retry the complete erase/program sequence, as the proven E131 fixture did,
  // and accept it only after a fresh attach can read the whole page back.
  uint8_t last_diagnostic = 0;
  for (int page_attempt = 0; page_attempt < 3; ++page_attempt) {
    BackendResult result =
        programPage64Attempt(address, data, last_diagnostic);
#if !OEP_V003_FORCE_SEQUENTIAL_FLASH && !OEP_V003_DISABLE_SEQUENTIAL_FALLBACK
    // Keep loader as the normal fast path, but retain the independently
    // verified host-sequenced path. A complete sequential erase/program is
    // safe after a partial loader attempt because it starts by erasing the
    // whole 64-byte page again.
    if (result == BackendResult::Failed && attachAndHalt() &&
        !prepareWordWriter() &&
        sequentialProgramPage64(address, data, last_diagnostic)) {
      result = BackendResult::Success;
    }
#endif
    if (result == BackendResult::Unavailable) {
      diagnostic = last_diagnostic;
      return result;
    }
    if (result != BackendResult::Success || !attachAndHalt() ||
        prepareWordWriter()) {
      continue;
    }

    bool verified = true;
    for (uint32_t offset = 0; offset < 64; offset += 4) {
      const uint32_t expected = data[offset] |
          static_cast<uint32_t>(data[offset + 1]) << 8 |
          static_cast<uint32_t>(data[offset + 2]) << 16 |
          static_cast<uint32_t>(data[offset + 3]) << 24;
      uint32_t actual = 0;
      if (readMemoryWord(address + offset, &actual) || actual != expected) {
        verified = false;
        last_diagnostic = 0x40 + page_attempt;
        break;
      }
    }
    if (verified) {
      diagnostic = 0;
      return BackendResult::Success;
    }
  }
  diagnostic = last_diagnostic ? last_diagnostic : 0x43;
  return BackendResult::Failed;
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
