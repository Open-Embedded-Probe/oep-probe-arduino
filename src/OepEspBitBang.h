// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// GPIO bit-bang pins for an ESP32's ARM SWD (OepSwd.h): the same primitives as rp2::BitBang (host drives while SWCLK is
// low, both sides sample on the rising edge), on the GPIO registers - any output GPIO, both lines plain GPIOs (the
// matrix's simple output). Not the dedicated GPIO bundle the RVSWD PHY takes (OepRvswdPhy.cpp: one bundle, and the
// classic ESP32 has none). The half period is CPU cycles counted (SWD waits on its clock, so an interrupt between edges
// only stretches it).
#pragma once

#if defined(ARDUINO_ARCH_ESP32)

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_cpu.h>
// ESP-IDF's GPIO matrix initializer leaves fields zero; silence its aggregate warning only in the SDK header.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include <hal/gpio_ll.h>
#pragma GCC diagnostic pop
#include <soc/gpio_struct.h>

namespace oep {
namespace esp {

struct BitBang {
  int dio_pin = -1, clk_pin = -1;
  uint32_t cycles = 0;

  inline void spin() const {
    if (!cycles) return;
    const uint32_t start = esp_cpu_get_cycle_count();
    while (esp_cpu_get_cycle_count() - start < cycles) {}
  }
  inline void set(int pin, bool v) const { gpio_ll_set_level(&GPIO, static_cast<uint32_t>(pin), v ? 1 : 0); }
  inline void bothHigh() const { set(dio_pin, true); set(clk_pin, true); }
  // SWCLK low and SWDIO to the bit: the clock first, so its edge never rides on a stale data level
  inline void clkLowDio(bool v) const { set(clk_pin, false); set(dio_pin, v); }
  inline void clkHigh() const { set(clk_pin, true); }
  inline void clk(bool v) const { set(clk_pin, v); }
  inline void dio(bool v) const { set(dio_pin, v); }
  inline bool dioRead() const { return gpio_ll_get_level(&GPIO, static_cast<uint32_t>(dio_pin)) != 0; }
  inline void hostDrives(bool yes) const {
    if (yes) gpio_ll_output_enable(&GPIO, static_cast<uint32_t>(dio_pin));
    else gpio_ll_output_disable(&GPIO, static_cast<uint32_t>(dio_pin));
  }

  // `dio`, `clk`: GPIO numbers, both able to drive (not the classic ESP32's input-only 34-39). Both left Hi-Z inputs,
  // their output latches high, SWDIO pulled up (the data line idles high while the target drives it).
  bool setup(int dio, int clk) {
    if (dio < 0 || clk < 0 || dio == clk || !GPIO_IS_VALID_OUTPUT_GPIO(dio) || !GPIO_IS_VALID_OUTPUT_GPIO(clk))
      return false;
    dio_pin = dio;
    clk_pin = clk;
    for (const int p : {dio, clk}) {
      pinMode(p, INPUT);   // the pad a GPIO, its input on
      set(p, true);
      gpio_ll_output_disable(&GPIO, static_cast<uint32_t>(p));   // Hi-Z, and the matrix's simple GPIO output when on
      gpio_set_drive_capability(static_cast<gpio_num_t>(p), GPIO_DRIVE_CAP_0);   // the weakest, as the RVSWD PHY's
    }
    gpio_ll_pulldown_dis(&GPIO, static_cast<uint32_t>(dio));
    gpio_ll_pullup_en(&GPIO, static_cast<uint32_t>(dio));
    return true;
  }
  void driveBoth() const {
    bothHigh();   // the latches high before the outputs are on
    gpio_ll_output_enable(&GPIO, static_cast<uint32_t>(dio_pin));
    gpio_ll_output_enable(&GPIO, static_cast<uint32_t>(clk_pin));
  }
  void releaseBoth() const {
    if (dio_pin < 0) return;
    gpio_ll_output_disable(&GPIO, static_cast<uint32_t>(dio_pin));
    gpio_ll_output_disable(&GPIO, static_cast<uint32_t>(clk_pin));
  }
  uint32_t setHalfNs(uint32_t half_ns) {
    cycles = static_cast<uint32_t>(static_cast<uint64_t>(half_ns) * getCpuFrequencyMhz() / 1000);
    return cycles;
  }
};

}  // namespace esp
}  // namespace oep

#endif
