// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's PARLIO RX driver for host tests (OEP_HOST_FAKE_PARLIO): units and delimiters are made and
// deleted (counted), the divider the unit would set is written to HP_SYS_CLKRST, nothing is ever received. Enough for
// the logic capture's configure / query paths and its memory use.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include <driver/gpio.h>
#include <soc/hp_sys_clkrst_struct.h>

#define PARLIO_RX_UNIT_MAX_DATA_WIDTH 16
#ifndef GPIO_NUM_NC
#define GPIO_NUM_NC (-1)
#endif
enum parlio_clock_source_t { PARLIO_CLK_SRC_PLL_F160M = 1 };
enum parlio_sample_edge_t { PARLIO_SAMPLE_EDGE_NEG = 0, PARLIO_SAMPLE_EDGE_POS = 1 };
enum parlio_bit_pack_order_t { PARLIO_BIT_PACK_ORDER_LSB = 0, PARLIO_BIT_PACK_ORDER_MSB = 1 };
struct FakeParlioUnit { size_t max_recv_size; uint32_t rate; };
struct FakeParlioDelimiter { size_t eof_data_len; };
typedef FakeParlioUnit *parlio_rx_unit_handle_t;
typedef FakeParlioDelimiter *parlio_rx_delimiter_handle_t;
typedef struct {
  size_t trans_queue_depth, max_recv_size, data_width;
  parlio_clock_source_t clk_src;
  uint32_t exp_clk_freq_hz;
  int clk_in_gpio_num, clk_out_gpio_num, valid_gpio_num;
  int data_gpio_nums[PARLIO_RX_UNIT_MAX_DATA_WIDTH];
} parlio_rx_unit_config_t;
typedef struct { void *data; size_t recv_bytes; } parlio_rx_event_data_t;
typedef bool (*parlio_rx_callback_t)(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *);
typedef struct { parlio_rx_callback_t on_partial_receive, on_receive_done, on_timeout; } parlio_rx_event_callbacks_t;
typedef struct {
  parlio_sample_edge_t sample_edge;
  parlio_bit_pack_order_t bit_pack_order;
  size_t eof_data_len;
  uint32_t timeout_ticks;
} parlio_rx_soft_delimiter_config_t;
typedef struct { parlio_rx_delimiter_handle_t delimiter; struct { uint32_t partial_rx_en : 1, indirect_mount : 1; } flags; } parlio_receive_config_t;

inline int g_fake_parlio_units = 0, g_fake_parlio_delimiters = 0;
inline esp_err_t parlio_new_rx_unit(const parlio_rx_unit_config_t *c, parlio_rx_unit_handle_t *unit) {
  if (g_fake_parlio_units) return ESP_FAIL;   // one RX unit
  uint32_t n = (160000000u + c->exp_clk_freq_hz / 2) / c->exp_clk_freq_hz;
  if (n > 255) n = 255;
  HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_div_num = n - 1;
  HP_SYS_CLKRST.peri_clk_ctrl118.reg_parlio_rx_clk_div_numerator = 0;
  HP_SYS_CLKRST.peri_clk_ctrl118.reg_parlio_rx_clk_div_denominator = 0;
  *unit = new FakeParlioUnit{c->max_recv_size, c->exp_clk_freq_hz};
  ++g_fake_parlio_units;
  return ESP_OK;
}
inline esp_err_t parlio_del_rx_unit(parlio_rx_unit_handle_t unit) { delete unit; --g_fake_parlio_units; return ESP_OK; }
inline esp_err_t parlio_new_rx_soft_delimiter(const parlio_rx_soft_delimiter_config_t *c, parlio_rx_delimiter_handle_t *d) {
  *d = new FakeParlioDelimiter{c->eof_data_len};
  ++g_fake_parlio_delimiters;
  return ESP_OK;
}
inline esp_err_t parlio_del_rx_delimiter(parlio_rx_delimiter_handle_t d) { delete d; --g_fake_parlio_delimiters; return ESP_OK; }
inline esp_err_t parlio_rx_unit_register_event_callbacks(parlio_rx_unit_handle_t, const parlio_rx_event_callbacks_t *, void *) { return ESP_OK; }
inline esp_err_t parlio_rx_unit_enable(parlio_rx_unit_handle_t, bool) { return ESP_OK; }
inline esp_err_t parlio_rx_unit_disable(parlio_rx_unit_handle_t) { return ESP_OK; }
inline esp_err_t parlio_rx_unit_receive(parlio_rx_unit_handle_t, void *, size_t, const parlio_receive_config_t *) { return ESP_OK; }
inline esp_err_t parlio_rx_soft_delimiter_start_stop(parlio_rx_unit_handle_t, parlio_rx_delimiter_handle_t, bool) { return ESP_OK; }
