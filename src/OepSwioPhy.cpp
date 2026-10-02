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
  // The weakest drive (about 5 mA): the line's sharp edges at the default 20 mA coupled into the fixture lines next to
  // it - on the V003 jig a 1 MHz SPI target lost or shifted bits while the console was read over this wire (23 of 36
  // frames good; 72 of 72 at the weakest drive, the wire's own timing unchanged, 2026-10-02). OEP_SWIO_DRIVE_CAP
  // chooses another gpio_drive_cap_t.
#ifndef OEP_SWIO_DRIVE_CAP
#define OEP_SWIO_DRIVE_CAP GPIO_DRIVE_CAP_0
#endif
  gpio_set_drive_capability(static_cast<gpio_num_t>(gPin), OEP_SWIO_DRIVE_CAP);
  high(gMask);
  outputOn(gMask);
}
}  // namespace

bool SwioPhy::begin(int swio) {
  if (swio < 0 || swio > 31) return false;   // GPIO0-31 (one register; 34-39 are inputs only anyway)
  gPin = swio;
  gMask = 1u << swio;
  pinMode(gPin, INPUT);   // free until a connection takes it (oep-core §8), the same as after a release; attach pulls it up
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
  return readRetried(address, value);
}

void SwioPhy::write(uint8_t address, uint32_t value) {
  if (!ready_) return;
  ++transactions_;
  writeRaw(address, value);
}

// The line up and driven: left to its pull-up for 2 ms first, no target if it then reads low (held low: no pull-up, or
// the target is wedged).
bool SwioPhy::lineUp() {
  pinMode(gPin, INPUT_PULLUP);
  delay(2);
  if (!digitalRead(gPin)) return false;
  configureIo();
  return true;
}

void SwioPhy::release() {
  attached_ = false;
  if (gPin >= 0) pinMode(gPin, INPUT_PULLUP);
}

// The free state (oep-core §8): Hi-Z, no pull. attach() puts the pull-up back before it looks at the line.
void SwioPhy::free() {
  attached_ = false;
  if (gPin >= 0) pinMode(gPin, INPUT);
}

}  // namespace oep

#elif defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)
#include <driver/dedic_gpio.h>
#ifndef OEP_SWIO_DRIVE_CAP
#define OEP_SWIO_DRIVE_CAP GPIO_DRIVE_CAP_0   // see the classic block: sharp edges couple into fixture lines
#endif
#include <driver/gpio.h>
#include <esp_cpu.h>
#include <esp_rom_gpio.h>
#include <hal/dedic_gpio_cpu_ll.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_sig_map.h>
#include <soc/gpio_struct.h>

// ESP32-P4 (RISC-V, 360 MHz): the classic's frames and read recharge, on the CPU's dedicated GPIO (one channel out,
// its output enable and one channel in, all CSR accesses of a cycle; a GPIO register access takes about 260 ns here,
// 2026-10-02, as long as a whole short pulse) and timed in nanoseconds against the cycle counter, every edge at a
// deadline from the one before - so the bit timing does not depend on the clock. Any GPIO0-54. The pin is in the
// dedicated GPIO bundle only while attached; release gives it back as a plain GPIO input with its pull-up.
namespace oep {
namespace {
// The classic's coefficient-8 timing (262.5 / 862.5 ns low, the high as long as a short low: the WCH-LinkE's 240 /
// 860 ns), and its read phases (half a short pulse to the recharge, half again to the sample; a zero's wait twice
// a short pulse).
constexpr uint32_t kShortLowNs = 262, kLongLowNs = 862, kHighNs = 262;
constexpr uint32_t kHalfNs = 131, kRechargeNs = 30, kZeroNs = 524;
constexpr uint32_t kRiseTimeoutNs = 100000;   // the line never came back (the classic: 1000 polls of GPIO.in)
struct Times { uint32_t shortLow, longLow, high, half, recharge, zero, riseTimeout; };
Times gT = {};
int gPin = -1;
uint32_t gOut = 0, gIn = 0;   // the bundles' channel masks (CSR bits)
dedic_gpio_bundle_handle_t gOutBundle = nullptr, gInBundle = nullptr;
constexpr uint8_t kDmControl = 0x10, kDmCfgr = 0x7d, kDmShadowCfgr = 0x7e;
constexpr uint32_t kCfgr = 0x5aa50400;   // key + outen (E123)
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

uint32_t cyclesFor(uint32_t ns, uint32_t mhz) { return (ns * mhz + 999) / 1000; }

inline uint32_t IRAM_ATTR now() { return esp_cpu_get_cycle_count(); }
inline void IRAM_ATTR until(uint32_t deadline) { while (static_cast<int32_t>(now() - deadline) < 0) {} }
inline void IRAM_ATTR low(uint32_t m) { RV_CLEAR_CSR(CSR_GPIO_OUT_USER, m); }
inline void IRAM_ATTR high(uint32_t m) { RV_SET_CSR(CSR_GPIO_OUT_USER, m); }
inline void IRAM_ATTR outputOn(uint32_t m) { RV_CLEAR_CSR(CSR_GPIO_OEN_USER, m); }   // OEN: active low
inline void IRAM_ATTR outputOff(uint32_t m) { RV_SET_CSR(CSR_GPIO_OEN_USER, m); }
inline bool IRAM_ATTR sample(uint32_t in) { return (RV_READ_CSR(CSR_GPIO_IN_USER) & in) != 0; }

// `count` bits of `bits`, most significant first, from deadline t (the previous high's end): each a low (short for a
// 1, long for a 0) then a high. The next bit's length is worked out before the wait, so each edge comes right at its
// deadline (deciding it after the wait shortened every low by about 80 ns, 2026-10-02). Returns the last high's end,
// not yet waited for.
inline uint32_t IRAM_ATTR sendBits(uint32_t m, const Times &tm, uint64_t bits, int count, uint32_t t) {
  for (uint64_t mask = uint64_t{1} << (count - 1); mask; mask >>= 1) {
    const uint32_t lowFor = (bits & mask) ? tm.shortLow : tm.longLow;
    until(t);
    low(m);
    until(t += lowFor);
    high(m);
    t += tm.high;
  }
  return t;
}

// Read one bit: drive the low start, release, recharge the line high, sample. A slow
// rise (the target holding a zero) gets a second recharge; 2 = the line never came back.
inline int IRAM_ATTR readBit(uint32_t m, uint32_t in, const Times &tm) {
  uint32_t t = now();
  low(m);
  until(t += tm.shortLow);
  outputOff(m);
  high(m);
  until(t += tm.half);
  outputOn(m);
  until(t += tm.recharge);
  outputOff(m);
  until(t += tm.half);
  const int sampled = sample(in);
  if (!sampled) {
    until(t += tm.zero);
    outputOn(m);
    until(t += tm.recharge);
    outputOff(m);
  }
  t = now();
  do {
    if (sample(in)) {
      outputOn(m);
      until(now() + tm.half);
      return sampled;
    }
  } while (now() - t < tm.riseTimeout);
  outputOn(m);
  return 2;
}

void IRAM_ATTR writeRaw(uint8_t address, uint32_t value) {
  const uint32_t m = gOut;
  const Times tm = gT;
  high(m);
  outputOn(m);
  // start 1, address, 1 (write), data
  const uint64_t bits = (uint64_t{1} << 40) | (uint64_t{address & 0x7fu} << 33) | (uint64_t{1} << 32) | value;
  portENTER_CRITICAL(&gMux);
  until(sendBits(m, tm, bits, 41, now()));
  portEXIT_CRITICAL(&gMux);
  delayMicroseconds(8);   // E135: LinkE frame gap median 6.7 us
}

void dropBundles() {
  if (gOutBundle) dedic_gpio_del_bundle(gOutBundle);
  if (gInBundle) dedic_gpio_del_bundle(gInBundle);
  gOutBundle = gInBundle = nullptr;
  gOut = gIn = 0;
}

// The pin into a one-channel dedicated GPIO bundle (out, and in), its output enable taken from the bundle's OEN bit;
// left driven high. Made on the calling task's core, where every frame runs (loop()).
bool configureIo() {
  dropBundles();
  gpio_set_level(gpio_num_t(gPin), 1);
  gpio_config_t config = {};
  config.pin_bit_mask = uint64_t{1} << gPin;
  config.mode = GPIO_MODE_INPUT_OUTPUT;
  config.pull_up_en = GPIO_PULLUP_ENABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&config);
  gpio_set_drive_capability(gpio_num_t(gPin), OEP_SWIO_DRIVE_CAP);   // the weakest, as the classic and the RVSWD PHY
  const int pins[] = {gPin};
  dedic_gpio_bundle_config_t out = {};
  out.gpio_array = pins;
  out.array_size = 1;
  out.flags.out_en = 1;
  dedic_gpio_bundle_config_t in = {};
  in.gpio_array = pins;
  in.array_size = 1;
  in.flags.in_en = 1;
  if (dedic_gpio_new_bundle(&out, &gOutBundle) != ESP_OK || dedic_gpio_new_bundle(&in, &gInBundle) != ESP_OK ||
      dedic_gpio_get_out_mask(gOutBundle, &gOut) != ESP_OK || dedic_gpio_get_in_mask(gInBundle, &gIn) != ESP_OK) {
    dropBundles();
    return false;
  }
  high(gOut);
  outputOn(gOut);
  gpio_ll_set_output_enable_ctrl(&GPIO, gPin, true, false);   // the pad's output enable: the bundle's OEN bit
  gpio_ll_input_enable(&GPIO, gPin);
  gpio_ll_pullup_en(&GPIO, gPin);
  return true;
}

// Out of the bundle: a plain GPIO input with the pull-up (the classic's release).
void unconfigureIo() {
  if (gPin < 0) return;
  const bool had = gOutBundle != nullptr;
  if (had) outputOff(gOut);
  dropBundles();
  if (had) {
    gpio_ll_set_output_enable_ctrl(&GPIO, gPin, false, false);
    esp_rom_gpio_connect_out_signal(gPin, SIG_GPIO_OUT_IDX, false, false);
  }
  pinMode(gPin, INPUT_PULLUP);
}
}  // namespace

bool SwioPhy::begin(int swio) {
  if (swio < 0 || swio > 54) return false;   // GPIO0-54
  gPin = swio;
  const uint32_t mhz = getCpuFrequencyMhz();
  gT = {cyclesFor(kShortLowNs, mhz), cyclesFor(kLongLowNs, mhz), cyclesFor(kHighNs, mhz), cyclesFor(kHalfNs, mhz),
        cyclesFor(kRechargeNs, mhz), cyclesFor(kZeroNs, mhz), cyclesFor(kRiseTimeoutNs, mhz)};
  pinMode(gPin, INPUT);   // free until a connection takes it (oep-core §8), the same as after a release; attach pulls it up
  ready_ = true;
  return true;
}

bool SwioPhy::usePins(int swdio, int swclk) {
  if (swdio == gPin && swclk < 0) return true;
  if (attached_ || swclk >= 0 || swdio < 0 || swdio > 54) return false;
  if (gPin >= 0) { unconfigureIo(); pinMode(gPin, INPUT); }   // the old pin Hi-Z, without the pull-up it had
  return begin(swdio);
}

bool IRAM_ATTR SwioPhy::readRaw(uint8_t address, uint32_t &value) {
  const uint32_t m = gOut, in = gIn;
  const Times tm = gT;
  high(m);
  outputOn(m);
  uint32_t result = 0;
  const uint64_t bits = (1u << 8) | ((address & 0x7fu) << 1);   // start 1, address, 0 (read)
  portENTER_CRITICAL(&gMux);
  until(sendBits(m, tm, bits, 9, now()));
  for (int bit = 0; bit < 32; ++bit) {
    result <<= 1;
    const int decoded = readBit(m, in, tm);
    if (decoded == 2) { portEXIT_CRITICAL(&gMux); delayMicroseconds(8); return false; }
    result |= decoded;
  }
  portEXIT_CRITICAL(&gMux);
  delayMicroseconds(8);
  value = result;
  return true;
}

bool SwioPhy::read(uint8_t address, uint32_t &value) {
  if (!ready_ || !gOutBundle) return false;
  ++transactions_;
  return readRetried(address, value);
}

void SwioPhy::write(uint8_t address, uint32_t value) {
  if (!ready_ || !gOutBundle) return;
  ++transactions_;
  writeRaw(address, value);
}

// The line up and in the bundle: left to its pull-up for 2 ms first (out of any bundle), no target if it then reads
// low (held low: no pull-up, or the target is wedged).
bool SwioPhy::lineUp() {
  unconfigureIo();   // pinMode(INPUT_PULLUP), out of any bundle
  delay(2);
  if (!digitalRead(gPin)) return false;
  if (!configureIo()) { unconfigureIo(); return false; }
  return true;
}

void SwioPhy::release() {
  attached_ = false;
  unconfigureIo();
}

// The free state (oep-core §8): out of the bundle, Hi-Z, no pull. attach() puts the pull-up back before it looks.
void SwioPhy::free() {
  release();
  if (gPin >= 0) pinMode(gPin, INPUT);
}

}  // namespace oep

#endif

#if defined(ARDUINO_ARCH_ESP32) && (defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32P4))
// ---- shared by the classic ESP32 and the ESP32-P4 -------------------------------------------------------------
namespace oep {
namespace {
constexpr uint8_t kSwDmStatus = 0x11, kSwAbstractAuto = 0x18, kSwProgBuf0 = 0x20;
// The wire has one speed, so nothing slower to fall back to (as the RVSWD's slowest period): the checks take a few
// retries. SWIO has no parity (oep-if-debug §3.2): only a read that never comes back high is seen as failed; a wrong
// bit shows up only in a comparison.
constexpr uint8_t kCheckRetries = 3;
constexpr int kCheckRounds = 2;   // x 8 patterns: about 2 ms of frames
}  // namespace

bool SwioPhy::readRetried(uint8_t address, uint32_t &value) {
  if (readRaw(address, value)) return true;
  for (int attempt = 1; attempt < 4 && retryLeft(); ++attempt) {   // within the request's wire_retry_ms (oep-if-debug §2)
    ++retries_;
    const uint32_t t0 = micros();
    const bool ok = readRaw(address, value);
    spentRetrying(micros() - t0);
    if (ok) return true;
  }
  return false;
}

// The wake / configuration sequence and dmactive (oep-if-debug §1 items 1 and 2, §3): the configuration pair twice -
// E123: shadow first, before the DM answers - then dmactive only when DMCONTROL does not already read it set (that write
// clears haltreq: a target halted earlier would run again). The configuration read back says a module is there.
bool SwioPhy::configureModule() {
  writeRaw(kDmShadowCfgr, kCfgr); writeRaw(kDmCfgr, kCfgr);
  writeRaw(kDmShadowCfgr, kCfgr); writeRaw(kDmCfgr, kCfgr);
  uint32_t control = 0;
  if (!(readRaw(kDmControl, control) && control != 0xffffffffu && (control & 1))) {
    writeRaw(kDmControl, 1); writeRaw(kDmControl, 1);   // the two writes the E123 bring-up made
  }
  uint32_t configuration = 0;
  const uint32_t started = micros();
  const bool ok = readRaw(kDmCfgr, configuration);
  dmi_ns_ = (micros() - started) * 1000u;
  return ok && (configuration & 0xffff0000u) == 0x5aa50000u;
}

// The write path at the wire's one speed (oep-if-debug §1, §3): only the scratch, PROGBUF0, and ABSTRACTAUTO = 0 that
// makes it free (not restored). PROGBUF0 is read first and written back after.
bool SwioPhy::writesLand() {
  static const uint32_t kPatterns[] = {0xa5a5a5a5u, 0x5a5a5a5au, 0xffffffffu, 0x00000001u,
                                       0x0f0f0f0fu, 0xf0f0f0f0u, 0x80000000u, 0x7fffffffu};
  uint8_t missed = 0;
  uint32_t saved = 0;
  while (!readRaw(kSwProgBuf0, saved)) {
    if (missed++ >= kCheckRetries) return false;
    ++search_retries_;
  }
  writeRaw(kSwAbstractAuto, 0);
  bool ok = true;
  for (int round = 0; round < kCheckRounds && ok; ++round) {
    for (uint32_t pattern : kPatterns) {
      for (;;) {
        writeRaw(kSwProgBuf0, pattern);
        uint32_t read_back = 0;
        if (readRaw(kSwProgBuf0, read_back) && read_back == pattern) break;
        if (missed++ >= kCheckRetries) { ok = false; break; }
        ++search_retries_;
      }
      if (!ok) break;
    }
  }
  for (int attempt = 0; attempt <= kCheckRetries; ++attempt) {   // the value back, read to confirm
    writeRaw(kSwProgBuf0, saved);
    uint32_t read_back = 0;
    if (readRaw(kSwProgBuf0, read_back) && read_back == saved) break;
  }
  return ok;
}

bool SwioPhy::attach() {
  if (!ready_) return false;
  if (attached_) return true;
  if (!lineUp()) return false;
  if (!configureModule() || !writesLand()) { release(); return false; }
  attached_ = true;
  return true;
}

// A scan's look (oep-if-debug §1): the wake / configuration and dmactive, DMSTATUS read; no write check, released after.
bool SwioPhy::bringUp(uint32_t &dmstatus) {
  if (!ready_) return false;
  if (attached_) return read(kSwDmStatus, dmstatus);
  if (!lineUp()) return false;
  const bool ok = configureModule() && readRaw(kSwDmStatus, dmstatus);
  release();
  return ok;
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
void SwioPhy::free() { attached_ = false; }
bool SwioPhy::bringUp(uint32_t &) { return false; }
}  // namespace oep

#endif
