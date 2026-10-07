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
constexpr int kRisePolls = 1000;   // a read frame's polls of GPIO.in for the line to come back high, all bits together
// Before a frame: the wire taken (held until loop() comes round; waited for while a sampler window is exclusive), then
// the frame announced - the sampler stops reading GPIO.in until frameEnd(), right after its last GPIO access (before the
// gap after it) (OepWireGate.h).
inline void IRAM_ATTR waitWire() {
  gWireGate.hold();
  gWireGate.frameBegin();
}

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
// rise_polls: the frame's polls left for the line to come back high (kRisePolls a frame, not a bit: interrupts are off
// for the whole frame, and a line rising slowly at every bit kept them off 32 x 1000 polls).
inline int IRAM_ATTR readBit(uint32_t m, int &rise_polls) {
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
  do {   // one poll always (a frame's polls spent leave a bit whose line is already high to be read)
    if (GPIO.in & m) {
      outputOn(m);
      waitCycles(kCoefficient / 2);
      return sampled;
    }
  } while (--rise_polls > 0);
  return 2;   // the line never came back: left released to its pull-up, not driven high against it (§3.2)
}

// free_after: the wire does not answer (oep-if-debug §2, §3.2) - the line released to its pull-up once the frame is out.
void IRAM_ATTR writeRaw(uint8_t address, uint32_t value, bool free_after = false) {
  waitWire();
  const uint32_t m = gMask;
  high(m);
  outputOn(m);
  portENTER_CRITICAL(&gMux);
  sendOne(m);
  for (uint8_t mask = 0x40; mask; mask >>= 1) (address & mask) ? sendOne(m) : sendZero(m);
  sendOne(m);
  for (uint32_t mask = 0x80000000u; mask; mask >>= 1) (value & mask) ? sendOne(m) : sendZero(m);
  if (free_after) outputOff(m);
  portEXIT_CRITICAL(&gMux);
  gWireGate.frameEnd();
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
  waitWire();
  const uint32_t m = gMask;
  high(m);
  outputOn(m);
  uint32_t result = 0;
  int rise_polls = kRisePolls;
  portENTER_CRITICAL(&gMux);
  sendOne(m);
  for (uint8_t mask = 0x40; mask; mask >>= 1) (address & mask) ? sendOne(m) : sendZero(m);
  sendZero(m);
  for (int bit = 0; bit < 32; ++bit) {
    result <<= 1;
    const int decoded = readBit(m, rise_polls);
    if (decoded == 2) {   // no answer: released to the pull-up until a read answers (oep-if-debug §2, §3.2)
      outputOff(m);
      portEXIT_CRITICAL(&gMux);
      gWireGate.frameEnd();
      rest_free_ = true;
      delayMicroseconds(8);
      return false;
    }
    result |= decoded;
  }
  const DmiPhy::Outcome outcome = DmiPhy::outcomeOf(address, true, result);
  if (outcome == DmiPhy::kAnswered) rest_free_ = false;
  else if (outcome == DmiPhy::kNoAnswer) rest_free_ = true;   // a DMSTATUS of all ones: the line, no module
  if (rest_free_) outputOff(m);
  portEXIT_CRITICAL(&gMux);
  gWireGate.frameEnd();
  delayMicroseconds(8);
  value = result;
  return true;
}

bool SwioPhy::readWire(uint8_t address, uint32_t &value) {
  if (!ready_) return false;
  ++transactions_;
  return readRetried(address, value);
}

void SwioPhy::write(uint8_t address, uint32_t value) {
  if (!ready_) return;
  ++transactions_;
  writeRaw(address, value, rest_free_);
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
  attached_ = rest_free_ = false;
  if (gPin >= 0) pinMode(gPin, INPUT_PULLUP);
}

// The free state (oep-core §8): Hi-Z, no pull. attach() puts the pull-up back before it looks at the line.
void SwioPhy::free() {
  attached_ = rest_free_ = false;
  if (gPin >= 0) pinMode(gPin, INPUT);
}

// The console's turn (DmiPhy::backgroundTurn): the wire taken unless a sampler window is exclusive; then refused - the
// console reads nothing this poll, and a trigger search is told a turn was wanted. Once the console has sent the target
// what it had, a search's turn may end (backgroundSent).
bool SwioPhy::backgroundTurn() { return gWireGate.tryHold(); }
void SwioPhy::backgroundDone() {}
void SwioPhy::backgroundSent() { gWireGate.endTurn(); }

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
// the line never came back: a read frame's whole wait for the line to rise, all bits together (the classic: 1000 polls
// of GPIO.in) - interrupts are off for the frame, and a line rising slowly at every bit kept them off 32 x 100 us
constexpr uint32_t kRiseTimeoutNs = 100000;
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
// rise_left: the frame's cycles left for the line to come back high (tm.riseTimeout a frame).
inline int IRAM_ATTR readBit(uint32_t m, uint32_t in, const Times &tm, uint32_t &rise_left) {
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
      const uint32_t waited = now() - t;
      rise_left = waited < rise_left ? rise_left - waited : 0;
      until(now() + tm.half);
      return sampled;
    }
  } while (now() - t < rise_left);
  return 2;   // the line never came back: left released to its pull-up, not driven high against it (§3.2)
}

// free_after: the wire does not answer (oep-if-debug §2, §3.2) - the line released to its pull-up once the frame is out.
void IRAM_ATTR writeRaw(uint8_t address, uint32_t value, bool free_after = false) {
  const uint32_t m = gOut;
  const Times tm = gT;
  high(m);
  outputOn(m);
  // start 1, address, 1 (write), data
  const uint64_t bits = (uint64_t{1} << 40) | (uint64_t{address & 0x7fu} << 33) | (uint64_t{1} << 32) | value;
  portENTER_CRITICAL(&gMux);
  until(sendBits(m, tm, bits, 41, now()));
  if (free_after) outputOff(m);
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
  uint32_t rise_left = tm.riseTimeout;
  portENTER_CRITICAL(&gMux);
  until(sendBits(m, tm, bits, 9, now()));
  for (int bit = 0; bit < 32; ++bit) {
    result <<= 1;
    const int decoded = readBit(m, in, tm, rise_left);
    if (decoded == 2) {   // no answer: released to the pull-up until a read answers (oep-if-debug §2, §3.2)
      outputOff(m);
      portEXIT_CRITICAL(&gMux);
      rest_free_ = true;
      delayMicroseconds(8);
      return false;
    }
    result |= decoded;
  }
  const DmiPhy::Outcome outcome = DmiPhy::outcomeOf(address, true, result);
  if (outcome == DmiPhy::kAnswered) rest_free_ = false;
  else if (outcome == DmiPhy::kNoAnswer) rest_free_ = true;   // a DMSTATUS of all ones: the line, no module
  if (rest_free_) outputOff(m);
  portEXIT_CRITICAL(&gMux);
  delayMicroseconds(8);
  value = result;
  return true;
}

bool SwioPhy::readWire(uint8_t address, uint32_t &value) {
  if (!ready_ || !gOutBundle) return false;
  ++transactions_;
  return readRetried(address, value);
}

void SwioPhy::write(uint8_t address, uint32_t value) {
  if (!ready_ || !gOutBundle) return;
  ++transactions_;
  writeRaw(address, value, rest_free_);
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
  attached_ = rest_free_ = false;
  unconfigureIo();
}

// The free state (oep-core §8): out of the bundle, Hi-Z, no pull. attach() puts the pull-up back before it looks.
void SwioPhy::free() {
  release();
  if (gPin >= 0) pinMode(gPin, INPUT);
}

// The P4's capture is PARLIO's DMA (no sampler, no window): the wire never pauses.
bool SwioPhy::backgroundTurn() { return true; }
void SwioPhy::backgroundDone() {}
void SwioPhy::backgroundSent() {}

}  // namespace oep

#elif defined(OEP_HOST_FAKE_SWIO)
// Host tests (tests/host): whole frames against a simulated target (fake_swio_io.h). A read that the target leaves
// unanswered reads all ones (the line stays at its pull-up); one that never comes back high fails.
#include <fake_swio_io.h>

namespace oep {
namespace {
int gPin = -1;
constexpr uint8_t kDmControl = 0x10, kDmCfgr = 0x7d, kDmShadowCfgr = 0x7e;
constexpr uint32_t kCfgr = 0x5aa50400;
// as the classic's: the frames go through the sampler's gate (OepWireGate.h)
void waitWire() {
  gWireGate.hold();
  gWireGate.frameBegin();
}
void writeRaw(uint8_t address, uint32_t value, bool free_after = false) {
  waitWire();
  fakeSwioWrite(address, value, free_after);
  gWireGate.frameEnd();
}
}  // namespace

bool SwioPhy::backgroundTurn() { return gWireGate.tryHold(); }
void SwioPhy::backgroundDone() {}
void SwioPhy::backgroundSent() { gWireGate.endTurn(); }

bool SwioPhy::begin(int swio) {
  gPin = swio;
  ready_ = swio >= 0;
  return ready_;
}
bool SwioPhy::usePins(int swdio, int) { return begin(swdio); }
bool SwioPhy::readRaw(uint8_t address, uint32_t &value) {
  waitWire();
  uint32_t result = 0;
  const bool back = fakeSwioRead(address, result);
  gWireGate.frameEnd();
  if (!back) { rest_free_ = true; return false; }
  const DmiPhy::Outcome outcome = DmiPhy::outcomeOf(address, true, result);
  if (outcome == DmiPhy::kAnswered) rest_free_ = false;
  else if (outcome == DmiPhy::kNoAnswer) rest_free_ = true;
  value = result;
  return true;
}
bool SwioPhy::readWire(uint8_t address, uint32_t &value) {
  if (!ready_) return false;
  ++transactions_;
  return readRetried(address, value);
}
void SwioPhy::write(uint8_t address, uint32_t value) {
  if (!ready_) return;
  ++transactions_;
  writeRaw(address, value, rest_free_);
}
bool SwioPhy::lineUp() { return fakeSwioLineHigh(); }
void SwioPhy::release() { attached_ = rest_free_ = false; }
void SwioPhy::free() { release(); }

}  // namespace oep

#endif

#if (defined(ARDUINO_ARCH_ESP32) && (defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32P4))) || \
    defined(OEP_HOST_FAKE_SWIO)
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

// A read that got nothing back - it failed, or DMSTATUS read all ones / all zeros (no module) - is retried with the
// link brought back in step first (resync): a target that reset itself through a system reset - a CH32V003 whose
// bootloader hands over to the application - drops its SWIO configuration and its debug module with it, and nothing
// answers until the configuration pair is written again (a fresh attach worked; the connection was lost: oep-if-debug
// §2 says a target reset does not close it). A retry starts only when it still ends within limits::kWireRetryMs (§2).
bool SwioPhy::readRetried(uint8_t address, uint32_t &value) {
  auto answered = [&](bool ok, uint32_t v) { return ok && DmiPhy::outcomeOf(address, true, v) != DmiPhy::kNoAnswer; };
  uint32_t t0 = micros();
  bool ok = readRaw(address, value);
  if (answered(ok, value)) return true;
  uint32_t cost = micros() - t0;
  for (int attempt = 1; attempt < 4 && retryFits(cost); ++attempt) {
    ++retries_;
    t0 = micros();
    resync();
    uint32_t v = 0;
    const bool got = readRaw(address, v);
    cost = micros() - t0;
    spentRetrying(cost);
    if (got) { value = v; ok = true; }
    if (answered(got, v)) return true;
  }
  return ok;   // DMSTATUS of no module: read, its value says so
}

// Back in step (oep-if-debug §3: the wake / configuration sequence of swio - the configuration pair twice - and
// dmactive when DMCONTROL reads it clear, §1 item 2): what a target that reset itself through a system reset
// dropped. Only while the wire does not answer (rest_free_: from a read with no answer until one answers) - a link in
// step pays nothing for Ch32Dm's relink after a change of hart state.
void SwioPhy::resync() {
  for (int i = 0; i < 2; ++i) {
    writeRaw(kDmShadowCfgr, kCfgr, rest_free_);
    writeRaw(kDmCfgr, kCfgr, rest_free_);
  }
  // dmactive only when DMCONTROL plainly reads it clear (a module reset with the target): a read that did not come
  // through is no reason for a write that would clear the haltreq a reset holds (Ch32Dm::resetHalt)
  uint32_t control = 0;
  if (readRaw(kDmControl, control) && control != 0xffffffffu && !(control & 1)) writeRaw(kDmControl, 1, rest_free_);
}

void SwioPhy::reinit() {
  if (attached_ && rest_free_) resync();
}

// The wake / configuration sequence and dmactive (oep-if-debug §1 items 1 and 2, §3): the configuration pair twice -
// E123: shadow first, before the DM answers - then dmactive only when DMCONTROL does not already read it set (that write
// clears haltreq: a target halted earlier would run again). The configuration read back says a module is there.
bool SwioPhy::configureModule() {
  writeRaw(kDmShadowCfgr, kCfgr, rest_free_); writeRaw(kDmCfgr, kCfgr, rest_free_);
  writeRaw(kDmShadowCfgr, kCfgr, rest_free_); writeRaw(kDmCfgr, kCfgr, rest_free_);
  uint32_t control = 0;
  if (!(readRaw(kDmControl, control) && control != 0xffffffffu && (control & 1))) {
    writeRaw(kDmControl, 1, rest_free_); writeRaw(kDmControl, 1, rest_free_);   // the two writes the E123 bring-up made
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
  writeRaw(kSwAbstractAuto, 0, rest_free_);
  bool ok = true;
  for (int round = 0; round < kCheckRounds && ok; ++round) {
    for (uint32_t pattern : kPatterns) {
      for (;;) {
        writeRaw(kSwProgBuf0, pattern, rest_free_);
        uint32_t read_back = 0;
        if (readRaw(kSwProgBuf0, read_back) && read_back == pattern) break;
        if (missed++ >= kCheckRetries) { ok = false; break; }
        ++search_retries_;
      }
      if (!ok) break;
    }
  }
  for (int attempt = 0; attempt <= kCheckRetries; ++attempt) {   // the value back, read to confirm
    writeRaw(kSwProgBuf0, saved, rest_free_);
    uint32_t read_back = 0;
    if (readRaw(kSwProgBuf0, read_back) && read_back == saved) break;
  }
  return ok;
}

bool SwioPhy::attach() {
  if (!ready_) return false;
  if (attached_) return true;
  rest_free_ = false;
  if (!lineUp()) return false;
  if (!configureModule() || !writesLand()) { release(); return false; }
  attached_ = true;
  return true;
}

// A scan's look (oep-if-debug §1): the wake / configuration and dmactive, DMSTATUS read; no write check, released after.
bool SwioPhy::bringUp(uint32_t &dmstatus) {
  if (!ready_) return false;
  if (attached_) return read(kSwDmStatus, dmstatus);
  rest_free_ = false;
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
bool SwioPhy::readWire(uint8_t, uint32_t &) { return false; }
void SwioPhy::write(uint8_t, uint32_t) {}
bool SwioPhy::attach() { return false; }
void SwioPhy::release() { attached_ = false; }
void SwioPhy::reinit() {}
void SwioPhy::free() { attached_ = false; }
bool SwioPhy::bringUp(uint32_t &) { return false; }
bool SwioPhy::backgroundTurn() { return true; }
void SwioPhy::backgroundDone() {}
void SwioPhy::backgroundSent() {}
}  // namespace oep

#endif
