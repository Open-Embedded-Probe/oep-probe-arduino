// The RVSWD wire frame, written once for every backend.
//
// Sequence as measured in E151-E157 on the CH32X035: START (both lines high,
// then SWDIO low while SWCLK is high), a 7-bit register address MSB first, the
// direction bit, its parity, a 5-bit aux pattern 0x15, then 32 data bits MSB
// first plus parity (the host releases SWDIO for a read), aux 0x17, and STOP.
//
// `Io` supplies only the pin primitives, so the timing-critical parts stay
// per-platform while the frame itself cannot drift between backends:
//   spin()               wait one half period
//   bothHigh()           drive SWDIO and SWCLK high together
//   clkLowDio(bool)      SWCLK low and SWDIO to the bit, together
//   clkHigh() / clk(b)   SWCLK
//   dio(bool)            SWDIO
//   dioRead()            sample SWDIO
//   hostDrives(bool)     SWDIO output enable
#pragma once

#include <stdint.h>

namespace oep {
namespace rvswd {

template <typename Io> inline void clockBit(Io &io, bool v) { io.clkLowDio(v); io.spin(); io.clkHigh(); io.spin(); }
template <typename Io> inline bool readBit(Io &io) { io.clk(false); io.spin(); const bool v = io.dioRead(); io.clkHigh(); io.spin(); return v; }
template <typename Io> inline void startFrame(Io &io) { io.bothHigh(); io.spin(); io.dio(false); io.spin(); }
// The stop condition is SWDIO rising while SWCLK is high, and the frame ends there with
// both lines high. What the bus should do while it then sits idle depends on the target,
// so that is RvswdPhy's decision (see its idle policy), not the frame's.
template <typename Io> inline void stopFrame(Io &io) { clockBit(io, false); io.bothHigh(); io.spin(); }

template <typename Io>
inline void header(Io &io, uint8_t address, bool write) {
  bool parity = write;
  for (int bit = 6; bit >= 0; --bit) { const bool v = (address >> bit) & 1; parity ^= v; clockBit(io, v); }
  clockBit(io, write);
  clockBit(io, parity);
}

template <typename Io>
inline void aux(Io &io, uint8_t pattern) { for (int b = 4; b >= 0; --b) clockBit(io, (pattern >> b) & 1); }

// One read transaction. false = the data parity did not match (the caller retries).
template <typename Io>
inline bool readWord(Io &io, uint8_t address, uint32_t &value) {
  startFrame(io);
  header(io, address, false);
  aux(io, 0x15);
  io.hostDrives(false);
  uint32_t data = 0;
  bool parity = false;
  for (int bit = 31; bit >= 0; --bit) { const bool v = readBit(io); data |= uint32_t(v) << bit; parity ^= v; }
  const bool ok = readBit(io) == parity;
  io.hostDrives(true);
  aux(io, 0x17);
  stopFrame(io);
  value = data;
  return ok;
}

template <typename Io>
inline void writeWord(Io &io, uint8_t address, uint32_t data) {
  startFrame(io);
  header(io, address, true);
  aux(io, 0x15);
  bool parity = false;
  for (int bit = 31; bit >= 0; --bit) { const bool v = (data >> bit) & 1; parity ^= v; clockBit(io, v); }
  clockBit(io, parity);
  aux(io, 0x17);
  stopFrame(io);
}

// Bring the bus up: both lines driven high, 100 idle clocks, one low bit, idle.
template <typename Io>
inline void wake(Io &io) {
  for (int i = 0; i < 100; ++i) { io.clkLowDio(true); io.spin(); io.clkHigh(); io.spin(); }
  io.clkLowDio(false); io.spin(); io.clkHigh(); io.spin();
  io.dio(true);
}

}  // namespace rvswd
}  // namespace oep
