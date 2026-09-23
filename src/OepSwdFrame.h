// The ordinary ARM SWD (ADIv5) wire frame, over the same bit-bang Io as the CH32 RVSWD
// frame: the host drives while SWCLK is low and both sides sample on the rising edge.
//
// Request: start(1), APnDP, RnW, A2, A3, parity, stop(0), park(1) - LSB first on the
// wire. Then a turnaround clock, a 3-bit ACK (OK = 0b001), and either 32 read bits plus
// parity followed by a turnaround, or a turnaround followed by 32 write bits plus parity.
// A line reset is 52 clocks with the data line high and then idle clocks low; on a
// multidrop port (RP2040, RP2350) every target is deselected by that reset until the
// host writes DP TARGETSEL, which no target acknowledges.
#pragma once

#include <stdint.h>

namespace oep {
namespace swd {

enum Ack : uint8_t { kOk = 1, kWait = 2, kFault = 4, kNoReply = 7 };

template <typename Io> inline void clockBit(Io &io, bool v) { io.clkLowDio(v); io.spin(); io.clkHigh(); io.spin(); }
template <typename Io> inline bool sampleBit(Io &io) { io.clk(false); io.spin(); const bool v = io.dioRead(); io.clkHigh(); io.spin(); return v; }
template <typename Io> inline void writeBits(Io &io, uint32_t value, int count) { for (int i = 0; i < count; ++i) clockBit(io, (value >> i) & 1); }
template <typename Io> inline uint32_t readBits(Io &io, int count) {
  uint32_t v = 0;
  for (int i = 0; i < count; ++i) if (sampleBit(io)) v |= 1u << i;
  return v;
}
inline bool parity32(uint32_t v) { v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1; return v & 1; }

template <typename Io> inline void idle(Io &io, int clocks) { for (int i = 0; i < clocks; ++i) clockBit(io, false); }

template <typename Io>
inline void lineReset(Io &io) {
  io.hostDrives(true);
  for (int i = 0; i < 52; ++i) clockBit(io, true);
  idle(io, 8);
}

// Legacy JTAG-to-SWD select, harmless on an SWD-only port.
template <typename Io>
inline void jtagToSwd(Io &io) {
  lineReset(io);
  writeBits(io, 0xE79E, 16);
  lineReset(io);
}

inline uint8_t request(bool ap, bool read_op, uint8_t a2_3) {
  const bool a2 = a2_3 & 1, a3 = (a2_3 >> 1) & 1;
  const bool par = ap ^ read_op ^ a2 ^ a3;
  return uint8_t(1 | (ap << 1) | (read_op << 2) | (a2 << 3) | (a3 << 4) | (par << 5) | (0 << 6) | (1 << 7));
}

// Returns the ACK. On kOk the data is valid (reads) or was sent (writes).
template <typename Io>
inline uint8_t transfer(Io &io, bool ap, bool read_op, uint8_t a2_3, uint32_t &data) {
  io.hostDrives(true);
  writeBits(io, request(ap, read_op, a2_3), 8);
  io.hostDrives(false);
  sampleBit(io);                   // turnaround
  const uint8_t ack = uint8_t(readBits(io, 3));
  if (read_op) {
    const uint32_t value = readBits(io, 32);
    const bool par = sampleBit(io);
    sampleBit(io);                 // turnaround
    io.hostDrives(true);
    if (ack == kOk && par != parity32(value)) return kNoReply;
    data = value;
  } else {
    sampleBit(io);                 // turnaround
    io.hostDrives(true);
    writeBits(io, data, 32);
    clockBit(io, parity32(data));
  }
  idle(io, 8);
  return ack;
}

// DP TARGETSEL (address 0b11, write): a multidrop port needs it after every line reset,
// and no target drives the ACK, so the reply is clocked and discarded.
template <typename Io>
inline void targetSelect(Io &io, uint32_t target_id) {
  io.hostDrives(true);
  writeBits(io, request(false, false, 0x3), 8);
  io.hostDrives(false);
  sampleBit(io);
  readBits(io, 3);                 // no target acknowledges this one
  sampleBit(io);
  io.hostDrives(true);
  writeBits(io, target_id, 32);
  clockBit(io, parity32(target_id));
  idle(io, 8);
}

}  // namespace swd
}  // namespace oep
