// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of the ESP-IDF GPIO LL calls the SPI target's MISO gate uses, on driver/gpio.h's fake pins.
#pragma once
#include <driver/gpio.h>

inline void gpio_ll_set_output_enable_ctrl(gpio_dev_t *, uint8_t pin, bool ctrl_by_periph, bool) {
  g_fake_gpio.oe_by_gpio[pin] = !ctrl_by_periph;
}
inline void gpio_ll_output_enable(gpio_dev_t *, uint32_t pin) { g_fake_gpio.enable[pin] = true; }
inline void gpio_ll_output_disable(gpio_dev_t *, uint32_t pin) { g_fake_gpio.enable[pin] = false; }
inline int gpio_ll_get_level(gpio_dev_t *, uint32_t pin) { return g_fake_gpio.level[pin]; }
