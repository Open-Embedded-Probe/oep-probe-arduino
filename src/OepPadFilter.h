// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The pad's glitch filter on the ESP32-P4 (the GPIO pin filter: pulses under 2 cycles of 80 MHz, 25 ns, dropped), held
// by count: whoever wants it on a pin takes it and gives it back, the filter is made at the first take and removed at
// the last give. Two functions can want it on one pin at once - the spi-target's CS, and a logic capture with its
// glitch filter listening on that same pin (capture: a logic capture shares any pin) - and the one that ends first
// leaves it on for the other.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <driver/gpio_filter.h>

namespace oep {

class PadFilter {
 public:
  static constexpr size_t kPins = 64;
  // false: no filter on the pin for this caller (the driver refused it; nothing to give back)
  static bool take(int pin) {
    if (pin < 0 || static_cast<size_t>(pin) >= kPins) return false;
    Pad &p = pads_[pin];
    if (!p.count) {
      gpio_pin_glitch_filter_config_t c = {};
      c.clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT;   // PLL_F80M: 2 cycles, 25 ns
      c.gpio_num = static_cast<gpio_num_t>(pin);
      if (gpio_new_pin_glitch_filter(&c, &p.handle) != ESP_OK) { p.handle = nullptr; return false; }
      if (gpio_glitch_filter_enable(p.handle) != ESP_OK) {
        gpio_del_glitch_filter(p.handle);
        p.handle = nullptr;
        return false;
      }
    }
    ++p.count;
    return true;
  }
  // A take that returned true, given back
  static void give(int pin) {
    if (pin < 0 || static_cast<size_t>(pin) >= kPins) return;
    Pad &p = pads_[pin];
    if (!p.count || --p.count) return;
    gpio_glitch_filter_disable(p.handle);
    gpio_del_glitch_filter(p.handle);
    p.handle = nullptr;
  }
  static uint8_t holders(int pin) { return pin < 0 || static_cast<size_t>(pin) >= kPins ? 0 : pads_[pin].count; }

 private:
  struct Pad { gpio_glitch_filter_handle_t handle; uint8_t count; };   // zero: static storage
  static inline Pad pads_[kPins];
};

}  // namespace oep
