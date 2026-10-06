// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of the RP2's ADC (pico-sdk hardware/adc.h) for host tests (OEP_HOST_FAKE_RP2_ADC): calls are recorded, nothing
// converts. Enough for the analog capture's configure, start and stop paths.
#pragma once
#include <stdint.h>

#define ADC_BASE_PIN 26
struct FakeAdcHw { uint32_t fifo; };
inline FakeAdcHw g_fake_adc_hw = {};
#define adc_hw (&g_fake_adc_hw)
struct FakeAdc { bool running; uint32_t round_robin; unsigned input; float clkdiv; int inits; };
inline FakeAdc g_fake_adc = {};
inline void adc_init() { ++g_fake_adc.inits; }
inline void adc_gpio_init(unsigned) {}
inline void adc_run(bool on) { g_fake_adc.running = on; }
inline void adc_fifo_drain() {}
inline void adc_select_input(unsigned input) { g_fake_adc.input = input; }
inline void adc_set_round_robin(uint32_t mask) { g_fake_adc.round_robin = mask; }
inline void adc_fifo_setup(bool, bool, unsigned, bool, bool) {}
inline void adc_set_clkdiv(float div) { g_fake_adc.clkdiv = div; }
