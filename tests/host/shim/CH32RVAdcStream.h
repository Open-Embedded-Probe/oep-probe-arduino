// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host test fake of ArduinoCore-CH32RV's CH32RVAdcStream (0.4.0), for OEP_HOST_FAKE_CH32_ADC: the sample-time steps
// (g_ch32_steps, conversions a second, fastest first; begin() takes the fastest whose rounds are at or below the rate,
// else the slowest), the two halves the DMA fills (hostConvert puts values in; a half filled again before it was taken
// is an overrun) and readBuffer().
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifndef CH32RV_ADC_BITS
#define CH32RV_ADC_BITS 12
#endif
typedef uint32_t pin_size_t;

inline uint32_t g_ch32_steps[8] = {250000, 160000, 100000, 60000, 0, 0, 0, 0};
inline int g_ch32_no_adc_pin = -1;   // a pin begin() refuses (no ADC1 input)

class CH32RVAdcStream {
 public:
  static constexpr uint8_t MAX_PINS = 6;
  pin_size_t pins[MAX_PINS] = {};
  uint8_t count = 0;
  bool begun = false, running = false;

  bool begin(pin_size_t pin, uint32_t rate) { return begin(&pin, 1, rate); }
  bool begin(const pin_size_t *p, uint8_t n, uint32_t rate) {
    if (!p || !n || n > MAX_PINS || !rate) return false;
    for (uint8_t i = 0; i < n; ++i) if (static_cast<int>(p[i]) == g_ch32_no_adc_pin) return false;
    for (uint8_t i = 0; i < n; ++i) pins[i] = p[i];
    count = n;
    rate_ = 0;
    for (const uint32_t s : g_ch32_steps) {
      if (!s) break;
      rate_ = s / n;
      if (rate_ <= rate) break;
    }
    begun = true;
    return true;
  }
  void end() { begun = running = false; rate_ = 0; }
  bool startContinuous(uint16_t *buffer, uint16_t samples) {
    if (!begun || !buffer || !samples) return false;
    buf_ = buffer;
    samples_ = samples;
    at_ = 0;
    ready_ = 0;
    running = true;
    return true;
  }
  void stop() { running = false; }
  const uint16_t *readBuffer() {
    for (uint8_t h = 0; h < 2; ++h) {
      const uint8_t k = (taken_ + h) % 2;
      if (ready_ >> k & 1) { ready_ &= ~(1u << k); taken_ = (k + 1) % 2; return buf_ + k * samples_; }
    }
    return nullptr;
  }
  uint32_t overruns() const { return overruns_; }
  uint32_t sampleRate() const { return rate_; }

  // The DMA: n values of `value` (or value + i with ramp), half after half
  void hostConvert(uint32_t n, uint16_t value, bool ramp = false) {
    for (uint32_t i = 0; i < n && running; ++i) {
      buf_[at_] = static_cast<uint16_t>(ramp ? value + i : value);
      if (++at_ % samples_ == 0) {
        const uint8_t half = static_cast<uint8_t>((at_ / samples_ - 1) % 2);
        if (ready_ >> half & 1) ++overruns_;
        ready_ |= 1u << half;
        if (at_ == 2u * samples_) at_ = 0;
      }
    }
  }

 private:
  uint32_t rate_ = 0;
  uint16_t *buf_ = nullptr;
  uint16_t samples_ = 0;
  uint32_t at_ = 0;
  uint8_t ready_ = 0, taken_ = 0;
  uint32_t overruns_ = 0;
};
