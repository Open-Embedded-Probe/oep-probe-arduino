// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Data format 2, the analog blocks (oep-if-capture §1.4): a block of r <= L original frames becomes, for each channel in
// the layout's order, its part ending on a byte:
//   packed          r values of b bits
//   minmax / sum    per interval of D frames (the last one partial) min and max (b bits each) and / or the sum
//                   (b + ceil(log2 D) bits), in that order
// bits by §1's bit order (bit j of the part: byte j / 8, bit j mod 8). A block's size depends only on r and the
// carriage, so a segment's bytes and any position in it are known before it is read: the capture keeps its frames as
// they came and encodes the blocks a read asks for.
//
// No chip in here (the host tests run it against the shared vectors, tests/vectors/capture_formats.json).
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace oep {
namespace analog_blocks {

constexpr uint8_t kMaxChannels = 4;
constexpr uint32_t kMaxBlock = 256;          // L at most: a block of 4 packed 12-bit channels is 1536 bytes
constexpr uint8_t kPacked = 1, kMinMax = 2, kSum = 4;   // registry analog_carriage (a bit set)

struct Plan {
  uint8_t channels = 0;                      // C, in the layout's order
  uint8_t bits = 12;                         // b
  uint8_t kind[kMaxChannels] = {kPacked, kPacked, kPacked, kPacked};   // by frame slot
  uint32_t d[kMaxChannels] = {1, 1, 1, 1};
  uint32_t block = 0;                        // L: a multiple of 8 and of every D of minmax / sum, at most kMaxBlock
};

inline uint8_t ceilLog2(uint32_t d) {
  uint8_t n = 0;
  while ((1ull << n) < d) ++n;
  return n;
}

inline size_t partBits(const Plan &p, uint8_t m, uint32_t r) {
  if (p.kind[m] == kPacked) return static_cast<size_t>(r) * p.bits;
  const size_t per = (p.kind[m] & kMinMax ? 2u * p.bits : 0u) + (p.kind[m] & kSum ? p.bits + ceilLog2(p.d[m]) : 0u);
  return static_cast<size_t>((r + p.d[m] - 1) / p.d[m]) * per;
}

// The bytes of one block of r frames.
inline size_t blockBytes(const Plan &p, uint32_t r) {
  size_t n = 0;
  for (uint8_t m = 0; m < p.channels; ++m) n += (partBits(p, m, r) + 7) / 8;
  return n;
}

// The bytes of a segment of `frames` frames: whole blocks of L, the last of what is left.
inline uint64_t segmentBytes(const Plan &p, uint64_t frames) {
  if (!p.block) return 0;
  const uint64_t whole = frames / p.block, rest = frames % p.block;
  return whole * blockBytes(p, p.block) + (rest ? blockBytes(p, static_cast<uint32_t>(rest)) : 0);
}

class BitWriter {
 public:
  explicit BitWriter(uint8_t *out) : out_(out) {}
  void put(uint64_t v, uint8_t bits) {
    for (uint8_t i = 0; i < bits; ++i, ++at_) {
      if ((at_ & 7) == 0) out_[at_ >> 3] = 0;
      if ((v >> i) & 1) out_[at_ >> 3] |= static_cast<uint8_t>(1u << (at_ & 7));
    }
  }
  size_t bytes() const { return (at_ + 7) / 8; }
 private:
  uint8_t *out_;
  size_t at_ = 0;
};

// Frames 0 .. r-1 of `frames` (C values a frame, slot m at frames[f * C + m]; the value in its low b bits) -> one block
// at `out` (blockBytes(p, r) of room). -> the bytes written.
inline size_t encode(const Plan &p, const uint16_t *frames, uint32_t r, uint8_t *out) {
  size_t used = 0;
  const uint32_t mask = (1u << p.bits) - 1u;
  for (uint8_t m = 0; m < p.channels; ++m) {
    BitWriter w(out + used);
    if (p.kind[m] == kPacked) {
      for (uint32_t f = 0; f < r; ++f) w.put(frames[f * p.channels + m] & mask, p.bits);
    } else {
      const uint8_t sum_bits = static_cast<uint8_t>(p.bits + ceilLog2(p.d[m]));
      for (uint32_t lo = 0; lo < r; lo += p.d[m]) {
        const uint32_t hi = lo + p.d[m] < r ? lo + p.d[m] : r;
        uint32_t mn = mask, mx = 0;
        uint64_t sum = 0;
        for (uint32_t f = lo; f < hi; ++f) {
          const uint32_t v = frames[f * p.channels + m] & mask;
          if (v < mn) mn = v;
          if (v > mx) mx = v;
          sum += v;
        }
        if (p.kind[m] & kMinMax) { w.put(mn, p.bits); w.put(mx, p.bits); }
        if (p.kind[m] & kSum) w.put(sum, sum_bits);
      }
    }
    used += (partBits(p, m, r) + 7) / 8;
  }
  return used;
}

}  // namespace analog_blocks
}  // namespace oep
