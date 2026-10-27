// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host test fake of ArduinoCore-CH32RV's CH32RVLogicCapture (0.4.0), for OEP_HOST_FAKE_CH32_LOGIC: begin() rounds the rate
// to F_CPU / a whole divider, the DMA's two halves (hostSample puts port samples in; a half filled again before it was
// taken is an overrun) and readBuffer().
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 144000000UL
#endif

enum CH32RVLogicPort : uint8_t { PORT_A = 0, PORT_B, PORT_C, PORT_D, PORT_E, PORT_F };

class CH32RVLogicCapture {
 public:
  CH32RVLogicPort port = PORT_A;
  bool begun = false, running = false;
  bool begin(CH32RVLogicPort p, uint32_t sampleRate) {
    if (!sampleRate) return false;
    port = p;
    rate_ = F_CPU / ((F_CPU + sampleRate / 2) / sampleRate);
    begun = true;
    return true;
  }
  void end() { begun = running = false; }
  uint32_t sampleRate() const { return rate_; }
  bool startContinuous(uint16_t *buffer, uint32_t samples) {
    if (!begun || !buffer || samples < 2) return false;
    buf_ = buffer;
    half_ = samples / 2;
    at_ = 0;
    ready_ = 0;
    taken_ = 0;
    running = true;
    return true;
  }
  void stop() { running = false; }
  const uint16_t *readBuffer() {
    const uint8_t k = taken_;
    if (!(ready_ >> k & 1)) return nullptr;
    ready_ &= ~(1u << k);
    taken_ = (k + 1) % 2;
    return buf_ + k * half_;
  }
  uint32_t overruns() const { return overruns_; }
  // n samples of the port, from `next(i)` (i counted from this call's first)
  template <typename F> void hostSample(uint32_t n, F next) {
    for (uint32_t i = 0; i < n && running; ++i) {
      buf_[at_] = next(i);
      if (++at_ % half_ == 0) {
        const uint8_t half = static_cast<uint8_t>(at_ / half_ - 1);
        if (ready_ >> half & 1) ++overruns_;
        ready_ |= 1u << half;
        if (at_ == 2 * half_) at_ = 0;
      }
    }
  }

 private:
  uint32_t rate_ = 0;
  uint16_t *buf_ = nullptr;
  uint32_t half_ = 0, at_ = 0;
  uint8_t ready_ = 0, taken_ = 0;
  uint32_t overruns_ = 0;
};
