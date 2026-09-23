// SIO bit-bang pins for the RP2040/RP2350 debug PHYs. RVSWD and ARM SWD clock their
// wires the same way (host drives while SWCLK is low, both sides sample on the rising
// edge), so they share these primitives and differ only in their frame.
//
// Deliberately not PIO: the bench wiring is provisional and PIO wants a fixed, ordered
// pin group (2026-09-23). One SIO write is a single cycle, which is fast enough for the
// half periods the CH32 debug module accepts.
#pragma once

#if defined(ARDUINO_ARCH_RP2040)

#include <Arduino.h>
#include <hardware/gpio.h>
#include <hardware/structs/sio.h>

namespace oep {
namespace rp2 {

// Picoseconds per iteration of the spin loop, measured once. The half periods are tens
// of nanoseconds, shorter than any timer the core exposes; noinline keeps calibration
// and use on the identical loop.
__attribute__((noinline)) inline void nopLoop(uint32_t n) { for (uint32_t i = n; i; --i) __asm__ volatile("nop"); }

inline uint32_t loopPicoseconds() {
  static uint32_t ps = 0;
  if (!ps) {
    constexpr uint32_t kIterations = 200000;
    const uint32_t t0 = micros();
    nopLoop(kIterations);
    const uint32_t dt = micros() - t0;
    ps = dt ? (uint32_t)((uint64_t)dt * 1000000ull / kIterations) : 7000u;
    if (!ps) ps = 1;
  }
  return ps;
}

struct BitBang {
  uint32_t dio_mask = 0, clk_mask = 0, both_mask = 0, loops = 0;

  inline void spin() const { for (uint32_t i = loops; i; --i) __asm__ volatile("nop"); }
  inline void bothHigh() const { sio_hw->gpio_set = both_mask; }
  // SWCLK low and SWDIO to the bit: the clear lands first so the clock edge never rides
  // on a stale data level.
  inline void clkLowDio(bool v) const {
    sio_hw->gpio_clr = clk_mask | (v ? 0u : dio_mask);
    if (v) sio_hw->gpio_set = dio_mask;
  }
  inline void clkHigh() const { sio_hw->gpio_set = clk_mask; }
  inline void clk(bool v) const { if (v) sio_hw->gpio_set = clk_mask; else sio_hw->gpio_clr = clk_mask; }
  inline void dio(bool v) const { if (v) sio_hw->gpio_set = dio_mask; else sio_hw->gpio_clr = dio_mask; }
  inline bool dioRead() const { return (sio_hw->gpio_in & dio_mask) != 0; }
  inline void hostDrives(bool yes) const {
    if (yes) sio_hw->gpio_oe_set = dio_mask; else sio_hw->gpio_oe_clr = dio_mask;
  }

  // Both lines must sit below GPIO 32 so one register pair covers them.
  bool setup(int dio, int clk) {
    if (dio < 0 || clk < 0 || dio > 31 || clk > 31 || dio == clk) return false;
    dio_mask = 1u << dio;
    clk_mask = 1u << clk;
    both_mask = dio_mask | clk_mask;
    gpio_init(dio);
    gpio_init(clk);
    gpio_set_dir(dio, GPIO_IN);
    gpio_set_dir(clk, GPIO_IN);
    gpio_pull_up(dio);          // the data line idles high while the target drives it
    gpio_disable_pulls(clk);
    gpio_set_drive_strength(dio, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(clk, GPIO_DRIVE_STRENGTH_2MA);
    bothHigh();                 // latch high before any output enable
    loopPicoseconds();
    return true;
  }

  void driveBoth() const { sio_hw->gpio_oe_set = both_mask; }
  void releaseBoth() const { sio_hw->gpio_oe_clr = both_mask; }
  uint32_t setHalfNs(uint32_t half_ns) {
    const uint32_t ps = loopPicoseconds();
    loops = half_ns ? (uint32_t)(((uint64_t)half_ns * 1000ull + ps / 2) / ps) : 0;
    return loops;
  }
};

}  // namespace rp2
}  // namespace oep

#endif
