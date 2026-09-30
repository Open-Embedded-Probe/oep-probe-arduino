// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepSwioPhy.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32)
#include <driver/gpio.h>
#include <soc/gpio_struct.h>

namespace oep {
namespace {
constexpr int kCoefficient = 8;
// The pin (GPIO0-31: one out / enable register) as a mask. The hot paths read it once into a local, so it sits in a
// register like the compile-time constant it replaced and the cycle-counted timing is the same (not yet measured on
// hardware, 2026-09-30).
int gPin = -1;
uint32_t gMask = 0;
constexpr uint8_t kDmControl = 0x10, kDmCfgr = 0x7d, kDmShadowCfgr = 0x7e;
constexpr uint32_t kCfgr = 0x5aa50400;   // key + outen (E123)
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

inline void IRAM_ATTR waitCycles(int count) {
  asm volatile("1: addi %[n], %[n], -1\n   bbci %[n], 31, 1b\n" : [n] "+r"(count));
}
inline void IRAM_ATTR low(uint32_t m) { GPIO.out_w1tc = m; }
inline void IRAM_ATTR high(uint32_t m) { GPIO.out_w1ts = m; }
inline void IRAM_ATTR outputOn(uint32_t m) { GPIO.enable_w1ts = m; }
inline void IRAM_ATTR outputOff(uint32_t m) { GPIO.enable_w1tc = m; }
inline void IRAM_ATTR sendOne(uint32_t m) { low(m); waitCycles(kCoefficient); high(m); waitCycles(kCoefficient); }
inline void IRAM_ATTR sendZero(uint32_t m) { low(m); waitCycles(kCoefficient * 4); high(m); waitCycles(kCoefficient); }

// Read one bit: drive the low start, release, recharge the line high, sample. A slow
// rise (the target holding a zero) gets a second recharge; 2 = the line never came back.
inline int IRAM_ATTR readBit(uint32_t m) {
  low(m);
  waitCycles(kCoefficient);
  outputOff(m);
  high(m);
  waitCycles(kCoefficient / 2);
  outputOn(m);
  outputOff(m);
  waitCycles(kCoefficient / 2);
  const int sampled = (GPIO.in & m) != 0;
  if (!sampled) {
    waitCycles(kCoefficient * 2);
    outputOn(m);
    outputOff(m);
  }
  for (int timeout = 0; timeout < 1000; ++timeout) {
    if (GPIO.in & m) {
      outputOn(m);
      waitCycles(kCoefficient / 2);
      return sampled;
    }
  }
  outputOn(m);
  return 2;
}

void IRAM_ATTR writeRaw(uint8_t address, uint32_t value) {
  const uint32_t m = gMask;
  high(m);
  outputOn(m);
  portENTER_CRITICAL(&gMux);
  sendOne(m);
  for (uint8_t mask = 0x40; mask; mask >>= 1) (address & mask) ? sendOne(m) : sendZero(m);
  sendOne(m);
  for (uint32_t mask = 0x80000000u; mask; mask >>= 1) (value & mask) ? sendOne(m) : sendZero(m);
  portEXIT_CRITICAL(&gMux);
  delayMicroseconds(8);   // E135: LinkE frame gap median 6.7 us
}

void configureIo() {
  gpio_config_t config = {};
  config.pin_bit_mask = uint64_t{1} << gPin;
  config.mode = GPIO_MODE_INPUT_OUTPUT;
  config.pull_up_en = GPIO_PULLUP_ENABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&config);
  high(gMask);
  outputOn(gMask);
}
}  // namespace

bool SwioPhy::begin(int swio) {
  if (swio < 0 || swio > 31) return false;   // GPIO0-31 (one register; 34-39 are inputs only anyway)
  gPin = swio;
  gMask = 1u << swio;
  pinMode(gPin, INPUT_PULLUP);
  ready_ = true;
  return true;
}

bool SwioPhy::usePins(int swdio, int swclk) {
  if (swdio == gPin && swclk < 0) return true;
  if (attached_ || swclk >= 0 || swdio < 0 || swdio > 31) return false;
  if (gPin >= 0) pinMode(gPin, INPUT);   // the old pin Hi-Z, without the pull-up it had
  return begin(swdio);
}

// IRAM like the E123-E137 originals: the coefficient-8 bit timing cannot afford flash-cache misses.
bool IRAM_ATTR SwioPhy::readRaw(uint8_t address, uint32_t &value) {
  const uint32_t m = gMask;
  high(m);
  outputOn(m);
  uint32_t result = 0;
  portENTER_CRITICAL(&gMux);
  sendOne(m);
  for (uint8_t mask = 0x40; mask; mask >>= 1) (address & mask) ? sendOne(m) : sendZero(m);
  sendZero(m);
  for (int bit = 0; bit < 32; ++bit) {
    result <<= 1;
    const int decoded = readBit(m);
    if (decoded == 2) { portEXIT_CRITICAL(&gMux); delayMicroseconds(8); return false; }
    result |= decoded;
  }
  portEXIT_CRITICAL(&gMux);
  delayMicroseconds(8);
  value = result;
  return true;
}

bool SwioPhy::read(uint8_t address, uint32_t &value) {
  if (!ready_) return false;
  ++transactions_;
  for (int attempt = 0; attempt < 4; ++attempt) {
    if (readRaw(address, value)) return true;
    ++retries_;
  }
  return false;
}

void SwioPhy::write(uint8_t address, uint32_t value) {
  if (!ready_) return;
  ++transactions_;
  writeRaw(address, value);
}

bool SwioPhy::attach() {
  if (!ready_) return false;
  if (attached_) return true;
  pinMode(gPin, INPUT_PULLUP);
  delay(2);
  if (!digitalRead(gPin)) return false;   // line held low: no pull-up, or the target is wedged
  configureIo();
  // E123: the configuration register must be written twice, shadow first, before the DM answers.
  writeRaw(kDmShadowCfgr, kCfgr); writeRaw(kDmCfgr, kCfgr);
  writeRaw(kDmShadowCfgr, kCfgr); writeRaw(kDmCfgr, kCfgr);
  writeRaw(kDmControl, 1); writeRaw(kDmControl, 1);
  uint32_t configuration = 0;
  const uint32_t started = micros();
  const bool ok = readRaw(kDmCfgr, configuration);
  dmi_ns_ = (micros() - started) * 1000u;
  if (!ok || (configuration & 0xffff0000u) != 0x5aa50000u) { release(); return false; }
  attached_ = true;
  return true;
}

void SwioPhy::release() {
  attached_ = false;
  if (gPin >= 0) pinMode(gPin, INPUT_PULLUP);
}

}  // namespace oep

#else

namespace oep {
bool SwioPhy::begin(int) { return false; }
bool SwioPhy::usePins(int, int) { return false; }
bool SwioPhy::readRaw(uint8_t, uint32_t &) { return false; }
bool SwioPhy::read(uint8_t, uint32_t &) { return false; }
void SwioPhy::write(uint8_t, uint32_t) {}
bool SwioPhy::attach() { return false; }
void SwioPhy::release() { attached_ = false; }
}  // namespace oep

#endif
