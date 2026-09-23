#include "OepRvswdPhy.h"

#include "OepRvswdFrame.h"

// One frame implementation (OepRvswdFrame.h) over a per-core pin backend. Each
// backend supplies the Io primitives, a critical section, and pad setup; the
// public methods below are shared so the two cannot drift apart.

#if defined(ARDUINO_ARCH_ESP32) && defined(SOC_DEDICATED_GPIO_SUPPORTED)
#define OEP_RVSWD_BACKEND 1
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

struct Io {
  inline void spin() const {
    if (!gHalfCycles) return;
    const uint32_t start = esp_cpu_get_cycle_count();
    while (esp_cpu_get_cycle_count() - start < gHalfCycles) {}
  }
  inline void bothHigh() const { dedic_gpio_cpu_ll_write_mask(0x3, 0x3); }
  inline void clkLowDio(bool v) const { dedic_gpio_cpu_ll_write_mask(0x3, v ? 0x1 : 0x0); }
  inline void clkHigh() const { dedic_gpio_cpu_ll_write_mask(0x2, 0x2); }
  inline void clk(bool v) const { dedic_gpio_cpu_ll_write_mask(0x2, v ? 0x2 : 0); }
  inline void dio(bool v) const { dedic_gpio_cpu_ll_write_mask(0x1, v ? 0x1 : 0); }
  inline bool dioRead() const { return dedic_gpio_cpu_ll_read_in() & 0x1; }
  inline void hostDrives(bool yes) const {
    if (yes) gpio_ll_output_enable(&GPIO, gDio); else gpio_ll_output_disable(&GPIO, gDio);
  }
};
Io gIo;

struct Critical {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  Critical() { portENTER_CRITICAL(&mux); }
  ~Critical() { portEXIT_CRITICAL(&mux); }
};

bool ioBegin(int dio, int clk) {
  gDio = dio;
  pinMode(dio, INPUT);
  pinMode(clk, INPUT);
  if (!gOut) {
    pinMode(dio, OUTPUT | PULLUP);
    pinMode(clk, OUTPUT);
    const int outPins[] = {dio, clk};
    dedic_gpio_bundle_config_t outCfg = {};
    outCfg.gpio_array = outPins; outCfg.array_size = 2; outCfg.flags.out_en = 1;
    if (dedic_gpio_new_bundle(&outCfg, &gOut) != ESP_OK) return false;
    const int inPins[] = {dio};
    dedic_gpio_bundle_config_t inCfg = {};
    inCfg.gpio_array = inPins; inCfg.array_size = 1; inCfg.flags.in_en = 1;
    if (dedic_gpio_new_bundle(&inCfg, &gIn) != ESP_OK) return false;
    gpio_ll_od_disable(&GPIO, dio);
    gpio_ll_pullup_en(&GPIO, dio);
    gpio_ll_input_enable(&GPIO, dio);
    gpio_set_drive_capability(gpio_num_t(dio), GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(gpio_num_t(clk), GPIO_DRIVE_CAP_0);
    gIo.bothHigh();
  }
  return true;
}

void ioDrive(int dio, int clk) { gpio_ll_output_enable(&GPIO, clk); gpio_ll_output_enable(&GPIO, dio); }
void ioRelease(int dio, int clk) { gpio_ll_output_disable(&GPIO, dio); gpio_ll_output_disable(&GPIO, clk); }
uint32_t ioSetHalf(uint32_t half_ns) {
  gHalfCycles = (uint32_t)((uint64_t)half_ns * getCpuFrequencyMhz() / 1000);
  return gHalfCycles;
}

}  // namespace
}  // namespace oep

#elif defined(ARDUINO_ARCH_RP2040)
#define OEP_RVSWD_BACKEND 1
#include "OepRp2BitBang.h"

namespace oep {
namespace {

using Io = rp2::BitBang;
Io gIo;

struct Critical {
  Critical() { noInterrupts(); }
  ~Critical() { interrupts(); }
};

bool ioBegin(int dio, int clk) { return gIo.setup(dio, clk); }
void ioDrive(int, int) { gIo.driveBoth(); }
void ioRelease(int, int) { gIo.releaseBoth(); }
uint32_t ioSetHalf(uint32_t half_ns) { return gIo.setHalfNs(half_ns); }

}  // namespace
}  // namespace oep

#endif  // backend selection

#if OEP_RVSWD_BACKEND

namespace oep {

bool RvswdPhy::begin(int swdio, int swclk) {
  swdio_ = swdio;
  swclk_ = swclk;
  if (!ioBegin(swdio, swclk)) return false;
  release();
  ready_ = true;
  return true;
}

void RvswdPhy::setHalf(uint32_t half_ns) {
  half_ns_ = half_ns;
  half_cycles_ = ioSetHalf(half_ns);
}

void RvswdPhy::release() {
  if (swdio_ < 0) return;
  ioRelease(swdio_, swclk_);
  attached_ = false;
}

void RvswdPhy::configureBus() {
  gIo.bothHigh();
  ioDrive(swdio_, swclk_);
  delayMicroseconds(20);
  rvswd::wake(gIo);
  delayMicroseconds(20);
}

bool RvswdPhy::readRaw(uint8_t address, uint32_t &value) {
  bool ok;
  {
    Critical lock;
    ok = rvswd::readWord(gIo, address, value);
  }
  ++transactions_;
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
  {
    Critical lock;
    rvswd::writeWord(gIo, address, data);
  }
  ++transactions_;
}

bool RvswdPhy::probeOnce(uint32_t half_ns, uint32_t &dmstatus) {
  if (!ready_) return false;
  setHalf(half_ns);
  configureBus();
  write(0x10, 1);  // DMCONTROL.dmactive
  dmstatus = 0;
  const bool ok = readRaw(0x11, dmstatus);
  ioRelease(swdio_, swclk_);
  attached_ = false;
  // A debug module reports a nonzero DMSTATUS.version; an idle bus reads all ones or zeros.
  return ok && ((dmstatus >> 8) & 0xf) != 0 && dmstatus != 0xffffffffu;
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
    const uint32_t t0 = micros();
    for (int i = 0; i < 1000 && clean; ++i) {
      if (!readRaw(0x11, value)) { clean = false; break; }
      if (i == 0) first = value; else if (value != first) clean = false;
    }
    if (clean && ((first >> 8) & 0xf) != 0) {  // DMSTATUS.version nonzero
      dmi_ns_ = micros() - t0;                 // 1000 reads -> ns per read
      attached_ = true;
      return true;
    }
  }
  release();
  return false;
}

}  // namespace oep

#else  // stub for cores without a backend

namespace oep {
bool RvswdPhy::begin(int, int) { return false; }
bool RvswdPhy::probeOnce(uint32_t half_ns, uint32_t &dmstatus) {
  if (!ready_) return false;
  setHalf(half_ns);
  configureBus();
  write(0x10, 1);  // DMCONTROL.dmactive
  dmstatus = 0;
  const bool ok = readRaw(0x11, dmstatus);
  ioRelease(swdio_, swclk_);
  attached_ = false;
  // A debug module reports a nonzero DMSTATUS.version; an idle bus reads all ones or zeros.
  return ok && ((dmstatus >> 8) & 0xf) != 0 && dmstatus != 0xffffffffu;
}

bool RvswdPhy::attach() { return false; }
void RvswdPhy::release() { attached_ = false; }
bool RvswdPhy::read(uint8_t, uint32_t &) { return false; }
void RvswdPhy::write(uint8_t, uint32_t) {}
void RvswdPhy::setHalf(uint32_t) {}
void RvswdPhy::configureBus() {}
bool RvswdPhy::readRaw(uint8_t, uint32_t &) { return false; }
bool RvswdPhy::probeOnce(uint32_t, uint32_t &) { return false; }
}  // namespace oep

#endif
