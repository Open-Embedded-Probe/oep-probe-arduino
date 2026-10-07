// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's ADC continuous driver for host tests (OEP_HOST_FAKE_ESP_ADC), as the ESP32-P4 has it (type 2
// records of 4 bytes, ADC1 on GPIO16..23 as channels 0..7): a test puts conversions in the driver's pool
// (g_fake_adc.records) and the capture's adc_continuous_read takes them out, a conversion frame's worth at most a read.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include <deque>

#ifndef ESP_OK
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#endif
#ifndef ESP_ERR_TIMEOUT
#define ESP_ERR_TIMEOUT 0x107
#endif
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#define SOC_ADC_DIGI_MAX_BITWIDTH 12
enum adc_unit_t { ADC_UNIT_1 = 0, ADC_UNIT_2 = 1 };
typedef int adc_channel_t;
typedef int adc_atten_t;
enum adc_digi_convert_mode_t { ADC_CONV_SINGLE_UNIT_1 = 1 };
enum adc_digi_output_format_t { ADC_DIGI_OUTPUT_FORMAT_TYPE1 = 0, ADC_DIGI_OUTPUT_FORMAT_TYPE2 = 1 };
typedef struct {
  union {
    struct { uint16_t data : 12; uint16_t channel : 4; } type1;
    struct { uint32_t data : 12; uint32_t reserved12 : 1; uint32_t channel : 4; uint32_t unit : 1; uint32_t reserved : 14; } type2;
    uint32_t val;
  };
} adc_digi_output_data_t;
typedef struct { uint8_t atten, channel, unit, bit_width; } adc_digi_pattern_config_t;
typedef struct { uint32_t max_store_buf_size, conv_frame_size; struct { uint32_t flush_pool : 1; } flags; } adc_continuous_handle_cfg_t;
typedef struct {
  uint32_t pattern_num;
  adc_digi_pattern_config_t *adc_pattern;
  uint32_t sample_freq_hz;
  adc_digi_convert_mode_t conv_mode;
  adc_digi_output_format_t format;
} adc_continuous_config_t;
struct FakeAdcHandle { int unused; };
typedef FakeAdcHandle *adc_continuous_handle_t;
typedef struct { uint8_t *conv_frame_buffer; uint32_t size; } adc_continuous_evt_data_t;
typedef bool (*adc_continuous_callback_t)(adc_continuous_handle_t, const adc_continuous_evt_data_t *, void *);
typedef struct { adc_continuous_callback_t on_conv_done, on_pool_ovf; } adc_continuous_evt_cbs_t;

struct FakeAdc {
  std::deque<uint32_t> records;   // the pool: type 2 records, oldest first
  bool running = false;
  int handles = 0;
  uint32_t freq = 0, patterns = 0;
  adc_continuous_evt_cbs_t cbs = {};
  void *context = nullptr;
  adc_continuous_handle_t handle = nullptr;
  // one conversion of `channel` (ADC1) with `value`
  void put(uint8_t channel, uint16_t value) {
    adc_digi_output_data_t r;
    r.val = 0;
    r.type2.data = value;
    r.type2.channel = channel;
    r.type2.unit = 0;
    records.push_back(r.val);
  }
  void overflow() { if (cbs.on_pool_ovf) cbs.on_pool_ovf(handle, nullptr, context); }
};
inline FakeAdc g_fake_adc;

inline esp_err_t adc_continuous_new_handle(const adc_continuous_handle_cfg_t *, adc_continuous_handle_t *h) {
  *h = new FakeAdcHandle{};
  g_fake_adc.handle = *h;
  ++g_fake_adc.handles;
  return ESP_OK;
}
inline esp_err_t adc_continuous_deinit(adc_continuous_handle_t h) { delete h; --g_fake_adc.handles; g_fake_adc.running = false; return ESP_OK; }
inline esp_err_t adc_continuous_config(adc_continuous_handle_t, const adc_continuous_config_t *c) {
  g_fake_adc.freq = c->sample_freq_hz;
  g_fake_adc.patterns = c->pattern_num;
  return ESP_OK;
}
inline esp_err_t adc_continuous_register_event_callbacks(adc_continuous_handle_t, const adc_continuous_evt_cbs_t *cbs, void *ctx) {
  g_fake_adc.cbs = *cbs;
  g_fake_adc.context = ctx;
  return ESP_OK;
}
inline esp_err_t adc_continuous_start(adc_continuous_handle_t) { g_fake_adc.running = true; g_fake_adc.records.clear(); return ESP_OK; }
inline esp_err_t adc_continuous_stop(adc_continuous_handle_t) { g_fake_adc.running = false; return ESP_OK; }
inline esp_err_t adc_continuous_io_to_channel(int io, adc_unit_t *unit, adc_channel_t *channel) {
  if (io < 16 || io > 23) return ESP_FAIL;
  *unit = ADC_UNIT_1;
  *channel = io - 16;
  return ESP_OK;
}
inline esp_err_t adc_continuous_read(adc_continuous_handle_t, uint8_t *buf, uint32_t length, uint32_t *out, uint32_t) {
  uint32_t n = 0;
  while (n + 4 <= length && n < 256 && !g_fake_adc.records.empty()) {
    const uint32_t v = g_fake_adc.records.front();
    g_fake_adc.records.pop_front();
    buf[n] = static_cast<uint8_t>(v);
    buf[n + 1] = static_cast<uint8_t>(v >> 8);
    buf[n + 2] = static_cast<uint8_t>(v >> 16);
    buf[n + 3] = static_cast<uint8_t>(v >> 24);
    n += 4;
  }
  *out = n;
  return n ? ESP_OK : ESP_ERR_TIMEOUT;
}
