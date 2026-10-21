// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// GPIO bit-bang pins for the debug PHYs on ArduinoCore-CH32RV (WCH CH32 as the probe), the same primitives as
// OepRp2BitBang.h: the host drives while SWCLK is low, both sides sample on the rising edge. A CH32 pin's level goes
// through BSHR (BSXR for bits 16-23) and BCR, its direction through its 4-bit CNF+MODE field in CFGLR / CFGHR / CFGXR
// (no output-enable bit as the RP2's SIO has): SWDIO's turnaround rewrites that field. The two pins may be on different
// ports. Released, SWDIO is an input with the pull-up (OUTDR 1), SWCLK a floating input.
#pragma once

#if defined(ARDUINO_ARCH_CH32RV)

#include <Arduino.h>
#include <ch32rv_gpio.h>

namespace oep {
namespace ch32 {

__attribute__((noinline)) inline void nopLoop(uint32_t n) { for (uint32_t i = n; i; --i) __asm__ volatile("nop"); }

// Picoseconds per iteration of the spin loop, measured once (as OepRp2BitBang.h).
inline uint32_t loopPicoseconds() {
  static uint32_t ps = 0;
  if (!ps) {
    constexpr uint32_t kIterations = 100000;
    const uint32_t t0 = micros();
    nopLoop(kIterations);
    const uint32_t dt = micros() - t0;
    ps = dt ? dt * (1000000u / kIterations) : 40000u;   // us x 1e6 / 1e5: in 32 bits
    if (!ps) ps = 1;
  }
  return ps;
}

// One pin's registers, resolved once.
struct Pin {
  volatile uint32_t *set = nullptr, *clr = nullptr, *in = nullptr, *cfg = nullptr;
  uint32_t set_mask = 0, clr_mask = 0, in_mask = 0, cfg_shift = 0;
  bool resolve(int pad) {
    if (pad < 0 || pad == NOT_A_PIN) return false;
    const uint8_t port = CH32RV_PIN_PORT(pad), bit = CH32RV_PIN_BIT(pad);
    if (port >= CH32RV_PORT_COUNT || bit >= CH32RV_GPIO_PORT_WIDTH) return false;
    ch32rv_gpio_clock_enable(port);
#if CH32RV_GPIO_PORT_WIDTH > 16
    if (bit >= 16) { set = &CH32RV_GPIO_BSXR(port); set_mask = 1u << (bit - 16u); }
    else
#endif
    { set = &CH32RV_GPIO_BSHR(port); set_mask = 1u << bit; }
    clr = &CH32RV_GPIO_BCR(port);
    clr_mask = 1u << bit;
    in = &CH32RV_GPIO_INDR(port);
    in_mask = 1u << bit;
    if (bit < 8) { cfg = &CH32RV_GPIO_CFGLR(port); cfg_shift = bit * 4u; }
#if CH32RV_GPIO_PORT_WIDTH > 8
    else if (bit < 16) { cfg = &CH32RV_GPIO_CFGHR(port); cfg_shift = (bit - 8u) * 4u; }
#endif
#if CH32RV_GPIO_PORT_WIDTH > 16
    else { cfg = &CH32RV_GPIO_CFGXR(port); cfg_shift = (bit - 16u) * 4u; }
#endif
    return true;
  }
  inline void high() const { *set = set_mask; }
  inline void low() const { *clr = clr_mask; }
  inline bool read() const { return (*in & in_mask) != 0; }
  inline void config(uint32_t nibble) const { *cfg = (*cfg & ~(0xFu << cfg_shift)) | (nibble << cfg_shift); }
};

struct BitBang {
  Pin dio_pin, clk_pin;
  uint32_t loops = 0;

  inline void spin() const { if (loops) nopLoop(loops); }
  inline void bothHigh() const { dio_pin.high(); clk_pin.high(); }
  inline void clkLowDio(bool v) const { clk_pin.low(); if (v) dio_pin.high(); else dio_pin.low(); }
  inline void clkHigh() const { clk_pin.high(); }
  inline void clk(bool v) const { if (v) clk_pin.high(); else clk_pin.low(); }
  inline void dio(bool v) const { if (v) dio_pin.high(); else dio_pin.low(); }
  inline bool dioRead() const { return dio_pin.read(); }
  // SWDIO's turnaround: a push-pull output, or an input with the pull-up (OUTDR 1 picks the pull's direction)
  inline void hostDrives(bool yes) const {
    if (yes) dio_pin.config(CH32RV_GPIO_CFG_OUT_PP_10M);
    else { dio_pin.high(); dio_pin.config(CH32RV_GPIO_CFG_IN_PULL); }
  }

  // `dio`, `clk`: the core's pads (platformPad of the channels)
  bool setup(int dio, int clk) {
    if (dio == clk || !dio_pin.resolve(dio) || !clk_pin.resolve(clk)) return false;
    releaseBoth();
    loopPicoseconds();
    return true;
  }
  void driveBoth() const {
    bothHigh();   // the latches high before the outputs are on
    dio_pin.config(CH32RV_GPIO_CFG_OUT_PP_10M);
    clk_pin.config(CH32RV_GPIO_CFG_OUT_PP_10M);
  }
  void releaseBoth() const {
    dio_pin.high();
    dio_pin.config(CH32RV_GPIO_CFG_IN_PULL);   // the data line idles high while the target drives it
    clk_pin.config(CH32RV_GPIO_CFG_IN_FLOAT);
  }
  // SWCLK released with a pull toward `dir` (> 0 up, < 0 down, 0 none)
  void clkPull(int dir) const {
    if (!dir) { clk_pin.config(CH32RV_GPIO_CFG_IN_FLOAT); return; }
    if (dir > 0) clk_pin.high(); else clk_pin.low();
    clk_pin.config(CH32RV_GPIO_CFG_IN_PULL);
  }
  // both floating inputs, no pull (the free state, oep-core §8)
  void freeBoth() const {
    dio_pin.config(CH32RV_GPIO_CFG_IN_FLOAT);
    clk_pin.config(CH32RV_GPIO_CFG_IN_FLOAT);
  }
  uint32_t setHalfNs(uint32_t half_ns) {
    const uint32_t ps = loopPicoseconds();
    // in 32 bits: a half period of up to 4.29 ms (the slowest link's is 10 us)
    const uint32_t ns = half_ns < 4294967u ? half_ns : 4294966u;
    loops = ns ? (ns * 1000u + ps / 2) / ps : 0;
    return loops;
  }
};

}  // namespace ch32
}  // namespace oep

#endif
