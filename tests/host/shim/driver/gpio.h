// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of the little of the ESP-IDF GPIO driver and GPIO LL the SPI target's MISO gate uses (OEP_HOST_FAKE_SPI_SLAVE):
// a pin's level, its output enable and who controls it (the peripheral routed to it, or the GPIO enable bit), the ISR
// service with one handler per pin, called at once on an edge of the type set.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include <esp_ipc.h>
#include <fake_core.h>

typedef int esp_err_t;
#ifndef ESP_OK
#define ESP_OK 0
#define ESP_FAIL -1
#endif
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_INTR_FLAG_LEVEL3 (1 << 3)
#define ESP_INTR_FLAG_IRAM (1 << 10)
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
typedef int gpio_num_t;
enum gpio_int_type_t { GPIO_INTR_DISABLE = 0, GPIO_INTR_POSEDGE = 1, GPIO_INTR_NEGEDGE = 2, GPIO_INTR_ANYEDGE = 3 };
typedef void (*gpio_isr_t)(void *arg);
struct gpio_dev_t {};
inline gpio_dev_t GPIO;

struct FakeGpio {
  bool service = false;
  int service_core = -1;      // where the ISR service was installed: its handlers run there
  int level[64] = {};
  bool oe_by_gpio[64] = {};   // false: the peripheral routed to the pin enables its output
  bool enable[64] = {};       // the GPIO enable bit
  gpio_int_type_t intr[64] = {};
  gpio_isr_t isr[64] = {};
  void *arg[64] = {};
};
inline FakeGpio g_fake_gpio;

// As ESP-IDF's: a second install is refused (with an error log there, counted here); the first takes the core it is
// called on and allocates the interrupt there through esp_ipc_call_blocking (gpio_isr_register), so an install made
// from inside an IPC call to that core never returns on the chip.
inline int g_fake_gpio_error_logs = 0;
inline void fakeGpioRegister(void *) { g_fake_gpio.service = true; }
inline esp_err_t gpio_install_isr_service(int) {
  if (g_fake_gpio.service) { ++g_fake_gpio_error_logs; return ESP_ERR_INVALID_STATE; }
  g_fake_gpio.service_core = g_fake_core;
  if (esp_ipc_call_blocking(static_cast<uint32_t>(g_fake_core), fakeGpioRegister, nullptr) != ESP_OK) {
    ++g_fake_gpio_error_logs;
    g_fake_gpio.service_core = -1;
    return 0x105;   // ESP_ERR_NOT_FOUND
  }
  return ESP_OK;
}
inline esp_err_t gpio_set_intr_type(gpio_num_t pin, gpio_int_type_t type) { g_fake_gpio.intr[pin] = type; return ESP_OK; }
inline esp_err_t gpio_isr_handler_add(gpio_num_t pin, gpio_isr_t isr, void *arg) {
  if (!g_fake_gpio.service || g_fake_gpio.isr[pin]) return ESP_FAIL;
  g_fake_gpio.isr[pin] = isr; g_fake_gpio.arg[pin] = arg;
  return ESP_OK;
}
inline esp_err_t gpio_isr_handler_remove(gpio_num_t pin) { g_fake_gpio.isr[pin] = nullptr; return ESP_OK; }

// The outside world moves a pin: its handler runs at once on an edge of the type set.
inline void fakeGpioLevel(int pin, int level) {
  if (pin < 0 || pin >= 64 || g_fake_gpio.level[pin] == level) return;
  g_fake_gpio.level[pin] = level;
  const gpio_int_type_t t = g_fake_gpio.intr[pin];
  const bool edge = t == GPIO_INTR_ANYEDGE || (t == GPIO_INTR_POSEDGE && level) || (t == GPIO_INTR_NEGEDGE && !level);
  if (edge && g_fake_gpio.isr[pin]) g_fake_gpio.isr[pin](g_fake_gpio.arg[pin]);
}
