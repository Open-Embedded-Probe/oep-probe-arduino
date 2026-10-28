// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's GPIO glitch filter driver for host tests: the flex filters made (at most 8), their pin and
// window, enabled or not.
#pragma once
#include <stdint.h>

#include <driver/gpio.h>

enum glitch_filter_clock_source_t { GLITCH_FILTER_CLK_SRC_XTAL = 1, GLITCH_FILTER_CLK_SRC_PLL_F80M = 2 };
struct FakeGlitchFilter { int gpio; uint32_t width_ns, thres_ns; bool enabled = false; };
typedef FakeGlitchFilter *gpio_glitch_filter_handle_t;
typedef struct {
  glitch_filter_clock_source_t clk_src;
  gpio_num_t gpio_num;
  uint32_t window_width_ns;
  uint32_t window_thres_ns;
} gpio_flex_glitch_filter_config_t;

inline int g_fake_glitch_filters = 0;   // made and not deleted
inline FakeGlitchFilter *g_fake_glitch_last = nullptr;
inline esp_err_t gpio_new_flex_glitch_filter(const gpio_flex_glitch_filter_config_t *c, gpio_glitch_filter_handle_t *f) {
  if (g_fake_glitch_filters >= 8) return ESP_FAIL;
  *f = new FakeGlitchFilter{c->gpio_num, c->window_width_ns, c->window_thres_ns};
  g_fake_glitch_last = *f;
  ++g_fake_glitch_filters;
  return ESP_OK;
}
inline esp_err_t gpio_glitch_filter_enable(gpio_glitch_filter_handle_t f) { f->enabled = true; return ESP_OK; }
inline esp_err_t gpio_glitch_filter_disable(gpio_glitch_filter_handle_t f) { f->enabled = false; return ESP_OK; }
inline esp_err_t gpio_del_glitch_filter(gpio_glitch_filter_handle_t f) {
  if (g_fake_glitch_last == f) g_fake_glitch_last = nullptr;
  delete f;
  --g_fake_glitch_filters;
  return ESP_OK;
}
