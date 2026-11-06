// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

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

// The 84-bit long frame, the RISC-V DTM's dmi register as it is - the only one a CH32V103 answers (wch-protocols
// link-to-target#long-frame, E165 / E173): START, the host's address (7 bits), data (32) and op (2: 1 read, 2 write)
// and one 0 (bit 41; no parity on a V103), then - SWDIO released - the target's echo of the address (7), its data (32:
// the value read, or the data written echoed) and status (2: 0 success), then the host's bit 83 (1 after a read, 0
// after a write; a V103 takes a write that ends with 1 as a read), the terminal clock 0 and STOP: 85 clocks. false: the echo is not the address or the status not success (the caller retries).
template <typename Io>
inline bool longFrame(Io &io, uint8_t address, uint32_t data, uint8_t op, uint32_t &answer) {
  startFrame(io);
  for (int bit = 6; bit >= 0; --bit) clockBit(io, (address >> bit) & 1);
  for (int bit = 31; bit >= 0; --bit) clockBit(io, (data >> bit) & 1);
  clockBit(io, op & 2);
  clockBit(io, op & 1);
  clockBit(io, false);
  io.hostDrives(false);
  uint8_t echo = 0;
  for (int bit = 6; bit >= 0; --bit) echo = static_cast<uint8_t>(echo << 1 | readBit(io));
  uint32_t value = 0;
  for (int bit = 31; bit >= 0; --bit) value |= uint32_t(readBit(io)) << bit;
  uint8_t status = static_cast<uint8_t>(readBit(io) << 1);
  status |= static_cast<uint8_t>(readBit(io));
  io.hostDrives(true);
  clockBit(io, op == 1);   // bit 83 is the host's: 1 after a read, 0 after a write, as a WCH-LinkE sends it (E173)
  stopFrame(io);
  answer = value;
  return echo == address && status == 0;
}
template <typename Io> inline bool readLong(Io &io, uint8_t address, uint32_t &value) {
  return longFrame(io, address, 0, 1, value);
}
template <typename Io> inline void writeLong(Io &io, uint8_t address, uint32_t data) {
  uint32_t echo = 0;
  (void)longFrame(io, address, data, 2, echo);
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
