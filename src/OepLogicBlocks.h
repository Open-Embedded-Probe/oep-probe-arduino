// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Data format 1, the logic blocks (oep-if-capture §1.3): a block of r <= L original samples, taken from the capture's
// own w-bit samples, becomes
//   raw part: the raw channels packed R bits a sample (sample i at bit i * R, role order), padded to a byte
//   reduced part: one start bit per change_latch channel (the level its first sample is compared with), then every
//     reduced channel's interval values in role order and interval order (b bits each, bit 0 first), padded at its end
// A reduced channel's interval k is samples [k * D, min((k + 1) * D, r)): sample (the first sample), any_high (OR),
// any_low (AND) - 1 bit; change_latch - bit 0 the last level, bit 1 a change from the sample before - 2 bits. The size
// is fixed by the plan, whatever the signal (bytesFor).
//
// No chip in here (the host tests run it against the shared vectors, tests/vectors/capture_formats.json).
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "OepRegistry.h"

#pragma GCC push_options
#pragma GCC optimize("O2")

namespace oep {
namespace blocks {

constexpr size_t kMaxChannels = 16;
constexpr uint32_t kMaxBlock = 4096;   // L at most
// the carriages (registry logic_carriage)
namespace lc = v1::reg::fixture_logic;
constexpr uint8_t kRaw = lc::kLogicCarriageRaw, kSample = lc::kLogicCarriageSample,
                  kAnyHigh = lc::kLogicCarriageAnyHigh, kAnyLow = lc::kLogicCarriageAnyLow,
                  kChangeLatch = lc::kLogicCarriageChangeLatch;

inline uint8_t valueBits(uint8_t kind) { return kind == kChangeLatch ? 2 : 1; }

struct Plan {
  uint8_t w = 8;                          // the capture's sample width: 1, 2, 4, 8 or 16 bits
  uint8_t raw = 0, reduced = 0;           // R, and the reduced channels
  uint8_t raw_bit[kMaxChannels] = {};     // raw channel j (role order): its bit in a sample
  uint8_t red_bit[kMaxChannels] = {};     // reduced channel c (role order): its bit
  uint8_t kind[kMaxChannels] = {};        // reduced channel c: its carriage (kSample .. kChangeLatch)
  uint32_t d[kMaxChannels] = {};          // reduced channel c: D (2 or more)
  uint32_t block = 0;                     // L: a multiple of 8 and of every D, at most kMaxBlock
  uint8_t latches() const {
    uint8_t s = 0;
    for (uint8_t c = 0; c < reduced; ++c) s += kind[c] == kChangeLatch;
    return s;
  }
};

// The bytes a block of r samples takes (§1.3): fixed by the plan.
inline size_t reducedBits(const Plan &p, uint32_t r) {
  size_t bits = p.latches();
  for (uint8_t c = 0; c < p.reduced; ++c) bits += static_cast<size_t>((r + p.d[c] - 1) / p.d[c]) * valueBits(p.kind[c]);
  return bits;
}
inline size_t bytesFor(const Plan &p, uint32_t r) {
  return (static_cast<size_t>(r) * p.raw + 7) / 8 + (reducedBits(p, r) + 7) / 8;
}

class Encoder {
 public:
  void reset() { have_before_ = false; }

  // Samples 0 .. r-1 of `src` (the capture's packed w-bit samples, sample 0 at bit 0) -> one block at `out`. `contiguous`:
  // the sample before sample 0 is the last one of the previous call (a block after the first of a segment, or a segment
  // that follows on with no gap); else sample 0 is compared with nothing. -> the bytes written, 0 when `capacity` cannot
  // hold them.
  size_t encode(const Plan &p, const uint8_t *src, uint32_t r, uint8_t *out, size_t capacity, bool contiguous) {
    const size_t need = bytesFor(p, r);
    if (!r || need > capacity) return 0;
    memset(out, 0, need);
    // raw part: R bits a sample
    size_t j = 0;
    for (uint32_t i = 0; i < r; ++i) {
      const uint32_t s = sample(p, src, i);
      for (uint8_t k = 0; k < p.raw; ++k, ++j)
        if ((s >> p.raw_bit[k]) & 1) out[j >> 3] |= static_cast<uint8_t>(1u << (j & 7));
    }
    uint8_t *red = out + (static_cast<size_t>(r) * p.raw + 7) / 8;
    const uint32_t before = contiguous && have_before_ ? before_ : sample(p, src, 0);
    size_t at = 0;
    for (uint8_t c = 0; c < p.reduced; ++c)   // start: change_latch's level before the block
      if (p.kind[c] == kChangeLatch) put(red, at, (before >> p.red_bit[c]) & 1, 1);
    for (uint8_t c = 0; c < p.reduced; ++c) {
      const uint8_t b = p.red_bit[c], kind = p.kind[c];
      const uint32_t D = p.d[c];
      uint32_t last = (before >> b) & 1;
      for (uint32_t lo = 0; lo < r; lo += D) {
        const uint32_t hi = lo + D < r ? lo + D : r;
        uint32_t v = 0;
        if (kind == kSample) {
          v = (sample(p, src, lo) >> b) & 1;
        } else if (kind == kAnyHigh || kind == kAnyLow) {
          const uint32_t want = kind == kAnyHigh ? 1 : 0;
          v = 1 - want;
          for (uint32_t i = lo; i < hi; ++i)
            if (((sample(p, src, i) >> b) & 1) == want) { v = want; break; }
        } else {   // change_latch
          uint32_t changed = 0;
          for (uint32_t i = lo; i < hi; ++i) {
            const uint32_t x = (sample(p, src, i) >> b) & 1;
            changed |= x ^ last;
            last = x;
          }
          v = last | (changed << 1);
        }
        put(red, at, v, valueBits(kind));
      }
    }
    before_ = sample(p, src, r - 1);
    have_before_ = true;
    return need;
  }

 private:
  static uint32_t sample(const Plan &p, const uint8_t *src, uint32_t i) {
    switch (p.w) {
      case 16: return static_cast<uint32_t>(src[2 * i] | (src[2 * i + 1] << 8));
      case 8: return src[i];
      default: {
        const uint32_t bit = i * p.w;
        return (src[bit >> 3] >> (bit & 7)) & ((1u << p.w) - 1);
      }
    }
  }
  static void put(uint8_t *out, size_t &at, uint32_t v, uint8_t n) {
    for (uint8_t k = 0; k < n; ++k, ++at)
      if ((v >> k) & 1) out[at >> 3] |= static_cast<uint8_t>(1u << (at & 7));
  }
  uint32_t before_ = 0;
  bool have_before_ = false;
};

}  // namespace blocks
}  // namespace oep

#pragma GCC pop_options
