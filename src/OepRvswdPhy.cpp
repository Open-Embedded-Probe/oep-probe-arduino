#include "OepRvswdPhy.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(SOC_DEDICATED_GPIO_SUPPORTED)
#include <driver/dedic_gpio.h>
#include <driver/gpio.h>
#include <esp_cpu.h>
#include <hal/dedic_gpio_cpu_ll.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>

namespace oep {
namespace {
// Dedicated GPIO bundle: bit0 = SWDIO, bit1 = SWCLK. One bundle per process.
dedic_gpio_bundle_handle_t gOut = nullptr;
dedic_gpio_bundle_handle_t gIn = nullptr;
int gDio = -1;
uint32_t gHalfCycles = 0;

inline void spin() {
  if (!gHalfCycles) return;
  const uint32_t start = esp_cpu_get_cycle_count();
  while (esp_cpu_get_cycle_count() - start < gHalfCycles) {}
}
inline void clkLowDio(bool v) { dedic_gpio_cpu_ll_write_mask(0x3, v ? 0x1 : 0x0); }
inline void clkHigh() { dedic_gpio_cpu_ll_write_mask(0x2, 0x2); }
inline void clk(bool v) { dedic_gpio_cpu_ll_write_mask(0x2, v ? 0x2 : 0); }
inline void dio(bool v) { dedic_gpio_cpu_ll_write_mask(0x1, v ? 0x1 : 0); }
inline bool dioRead() { return dedic_gpio_cpu_ll_read_in() & 0x1; }
inline void hostDrives(bool yes) {
  if (yes) gpio_ll_output_enable(&GPIO, gDio); else gpio_ll_output_disable(&GPIO, gDio);
}
inline void clockBit(bool v) { clkLowDio(v); spin(); clkHigh(); spin(); }
inline bool readBit() { clk(false); spin(); const bool v = dioRead(); clkHigh(); spin(); return v; }
inline void startFrame() { dedic_gpio_cpu_ll_write_mask(0x3, 0x3); spin(); dio(false); spin(); }
inline void stopFrame() { clockBit(false); dedic_gpio_cpu_ll_write_mask(0x3, 0x3); spin(); }
inline void header(uint8_t address, bool write) {
  bool parity = write;
  for (int bit = 6; bit >= 0; --bit) { const bool v = (address >> bit) & 1; parity ^= v; clockBit(v); }
  clockBit(write); clockBit(parity);
}
inline void aux(uint8_t pattern) { for (int b = 4; b >= 0; --b) clockBit((pattern >> b) & 1); }
}  // namespace

bool RvswdPhy::begin(int swdio, int swclk) {
  swdio_ = swdio; swclk_ = swclk; gDio = swdio;
  pinMode(swdio, INPUT); pinMode(swclk, INPUT);
  if (!gOut) {
    pinMode(swdio, OUTPUT | PULLUP);
    pinMode(swclk, OUTPUT);
    const int outPins[] = {swdio, swclk};
    dedic_gpio_bundle_config_t outCfg = {};
    outCfg.gpio_array = outPins; outCfg.array_size = 2; outCfg.flags.out_en = 1;
    if (dedic_gpio_new_bundle(&outCfg, &gOut) != ESP_OK) return false;
    const int inPins[] = {swdio};
    dedic_gpio_bundle_config_t inCfg = {};
    inCfg.gpio_array = inPins; inCfg.array_size = 1; inCfg.flags.in_en = 1;
    if (dedic_gpio_new_bundle(&inCfg, &gIn) != ESP_OK) return false;
    gpio_ll_od_disable(&GPIO, swdio);
    gpio_ll_pullup_en(&GPIO, swdio);
    gpio_ll_input_enable(&GPIO, swdio);
    gpio_set_drive_capability(gpio_num_t(swdio), GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(gpio_num_t(swclk), GPIO_DRIVE_CAP_0);
    dedic_gpio_cpu_ll_write_mask(0x3, 0x3);
  }
  release();
  ready_ = true;
  return true;
}

void RvswdPhy::setHalf(uint32_t half_ns) {
  half_ns_ = half_ns;
  gHalfCycles = half_cycles_ = (uint32_t)((uint64_t)half_ns * getCpuFrequencyMhz() / 1000);
}

void RvswdPhy::release() {
  if (swdio_ < 0) return;
  gpio_ll_output_disable(&GPIO, swdio_);
  gpio_ll_output_disable(&GPIO, swclk_);
  attached_ = false;
}

void RvswdPhy::configureBus() {
  dedic_gpio_cpu_ll_write_mask(0x3, 0x3);
  gpio_ll_output_enable(&GPIO, swclk_);
  gpio_ll_output_enable(&GPIO, swdio_);
  delayMicroseconds(20);
  for (int i = 0; i < 100; ++i) { clkLowDio(true); spin(); clkHigh(); spin(); }
  clkLowDio(false); spin(); clkHigh(); spin(); dio(true);
  delayMicroseconds(20);
}

bool RvswdPhy::readRaw(uint8_t address, uint32_t &value) {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);
  startFrame(); header(address, false); aux(0x15);
  hostDrives(false);
  uint32_t data = 0; bool parity = false;
  for (int bit = 31; bit >= 0; --bit) { const bool v = readBit(); data |= uint32_t(v) << bit; parity ^= v; }
  const bool ok = readBit() == parity;
  hostDrives(true);
  aux(0x17); stopFrame();
  portEXIT_CRITICAL(&mux);
  ++transactions_;
  value = data;
  return ok;
}

bool RvswdPhy::read(uint8_t address, uint32_t &value) {
  for (int attempt = 0; attempt < 200; ++attempt) {
    if (readRaw(address, value)) return true;
    ++retries_;
  }
  return false;
}

void RvswdPhy::write(uint8_t address, uint32_t data) {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);
  startFrame(); header(address, true); aux(0x15);
  bool parity = false;
  for (int bit = 31; bit >= 0; --bit) { const bool v = (data >> bit) & 1; parity ^= v; clockBit(v); }
  clockBit(parity); aux(0x17); stopFrame();
  portEXIT_CRITICAL(&mux);
  ++transactions_;
}

bool RvswdPhy::attach() {
  if (!ready_) return false;
  if (attached_) return true;
  setHalf(500);
  configureBus();
  write(0x10, 1);  // DMCONTROL.dmactive
  // Margin check (E156/E157): half 0 ns sometimes fails for a whole run.
  static const uint32_t kHalfNs[] = {0, 25, 50, 100, 200, 500};
  for (uint32_t half : kHalfNs) {
    setHalf(half);
    uint32_t first = 0, value = 0;
    bool clean = true;
    for (int i = 0; i < 1000 && clean; ++i) {
      if (!readRaw(0x11, value)) { clean = false; break; }
      if (i == 0) first = value; else if (value != first) clean = false;
    }
    if (clean && ((first >> 8) & 0xf) != 0) { attached_ = true; return true; }  // DMSTATUS.version nonzero
  }
  release();
  return false;
}

}  // namespace oep

#else  // stub for other architectures

namespace oep {
bool RvswdPhy::begin(int, int) { return false; }
bool RvswdPhy::attach() { return false; }
void RvswdPhy::release() { attached_ = false; }
bool RvswdPhy::read(uint8_t, uint32_t &) { return false; }
void RvswdPhy::write(uint8_t, uint32_t) {}
void RvswdPhy::setHalf(uint32_t) {}
void RvswdPhy::configureBus() {}
bool RvswdPhy::readRaw(uint8_t, uint32_t &) { return false; }
}  // namespace oep

#endif
