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
  uint8_t wout = 0;                       // see below (0: R)
  uint8_t raw = 0, reduced = 0;           // R, and the reduced channels
  // the raw part's bits a sample (the answer's layout w): R, or w itself when the raw lanes go as taken (w 16 with more
  // than 8 raw channels: a copy, the other lanes' bits there undefined, §1.1); raw channel j at bit j of it either way
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
inline uint8_t rawBits(const Plan &p) { return p.wout ? p.wout : p.raw; }
inline size_t bytesFor(const Plan &p, uint32_t r) {
  return (static_cast<size_t>(r) * rawBits(p) + 7) / 8 + (reducedBits(p, r) + 7) / 8;
}

// The fast path's view of a plan: the raw channels on bits 0 .. R-1 in role order, the reduced channels in groups of
// lanes side by side with one (carriage, D) each. lanes(): whether the plan is laid out so.
struct Group {
  uint8_t kind = 0, lane = 0, n = 0;   // carriage, first lane, lanes
  uint32_t d = 0;
  uint8_t chan[kMaxChannels] = {};     // lane `lane + i` is reduced channel chan[i] (role order)
};

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
    const uint32_t before = contiguous && have_before_ ? before_ : sample(p, src, 0);
    const size_t raw_bytes = (static_cast<size_t>(r) * rawBits(p) + 7) / 8;
    uint8_t *red = out + raw_bytes;
    size_t at = 0;
    for (uint8_t c = 0; c < p.reduced; ++c)   // start: change_latch's level before the block
      if (p.kind[c] == kChangeLatch) put(red, at, (before >> p.red_bit[c]) & 1, 1);
    if (fastOk(p, src, r)) {
      packRaw(p, src, r, out);
      fastReduced(p, src, r, red, at, before);
    } else {
      referenceRaw(p, src, r, out);
      referenceReduced(p, src, r, red, at, before);
    }
    before_ = sample(p, src, r - 1);
    have_before_ = true;
    return need;
  }

  // The plan's groups (fast path), -> their count; 0 with reduced channels when the lanes are not laid out for it.
  static uint8_t groups(const Plan &p, Group *g) {
    uint8_t n = 0;
    for (uint8_t lane = p.raw; lane < p.raw + p.reduced; ++lane) {
      uint8_t c = 0;
      while (c < p.reduced && p.red_bit[c] != lane) ++c;
      if (c == p.reduced) return 0;
      if (n && g[n - 1].kind == p.kind[c] && g[n - 1].d == p.d[c] && g[n - 1].lane + g[n - 1].n == lane) {
        g[n - 1].chan[g[n - 1].n++] = c;
      } else {
        g[n].kind = p.kind[c];
        g[n].d = p.d[c];
        g[n].lane = lane;
        g[n].n = 1;
        g[n].chan[0] = c;
        ++n;
      }
    }
    return n;
  }
  // How a plan's roles go on the capture's lanes for the fast path: raw roles first (role order), then the reduced
  // ones grouped by (carriage, D). kinds[role] (kRaw or a reduction), ds[role]; -> lane[role].
  static void lanesFor(uint8_t roles, const uint8_t *kinds, const uint32_t *ds, uint8_t *lane) {
    uint8_t next = 0;
    for (uint8_t k = 0; k < roles; ++k)
      if (kinds[k] == kRaw) lane[k] = next++;
    bool done[kMaxChannels] = {};
    for (uint8_t k = 0; k < roles; ++k) {
      if (kinds[k] == kRaw || done[k]) continue;
      for (uint8_t m = k; m < roles; ++m)
        if (!done[m] && kinds[m] == kinds[k] && ds[m] == ds[k]) { lane[m] = next++; done[m] = true; }
    }
  }

 private:
  // ---- the fast path: w 4 / 8 / 16, raw on bits 0 .. R-1, groups of lanes, a whole block of whole words a D ----
  static bool fastOk(const Plan &p, const uint8_t *src, uint32_t r) {
    if ((p.w != 4 && p.w != 8 && p.w != 16) || (reinterpret_cast<uintptr_t>(src) & 3) || r % 8) return false;
    for (uint8_t j = 0; j < p.raw; ++j)
      if (p.raw_bit[j] != j) return false;
    const uint32_t per = 32u / p.w;   // samples a word
    for (uint8_t c = 0; c < p.reduced; ++c)
      if (p.d[c] % per || r % p.d[c]) return false;
    Group g[kMaxChannels];
    return !p.reduced || groups(p, g);
  }
  // R raw bits a sample, 8 samples -> R bytes (R fixed at compile time: the shifts and the store's size constant)
  template <uint32_t W, uint32_t R>
  static void packT(const uint32_t *wd, uint32_t r, uint8_t *o) {
    constexpr uint32_t m = (1u << R) - 1;
    if (W == 8) {   // 4 samples a word: their low R bits side by side (4R bits), two words -> R bytes
      constexpr uint32_t m4 = m * 0x01010101u, mlo = m * 0x00010001u, mhi = (m << R) * 0x00010001u,
                         m2 = (1u << (2 * R)) - 1;
      for (uint32_t i = 0; i < r / 4; i += 2) {
        uint32_t h[2];
        for (int k = 0; k < 2; ++k) {
          uint32_t t = wd[i + k] & m4;
          t = (t & mlo) | ((t >> (8 - R)) & mhi);
          h[k] = (t & m2) | ((t >> (16 - 2 * R)) & (m2 << (2 * R)));
        }
        if (R <= 4) {
          const uint32_t bits = h[0] | (h[1] << (4 * R));
          memcpy(o, &bits, R);
        } else {
          const uint64_t bits = h[0] | (static_cast<uint64_t>(h[1]) << (4 * R));
          memcpy(o, &bits, R);
        }
        o += R;
      }
    } else if (W == 16) {   // 2 samples a word: 2R bits; four words -> R bytes (R <= 8)
      for (uint32_t i = 0; i < r / 2; i += 4) {
        uint32_t t[4];
        for (int k = 0; k < 4; ++k) t[k] = (wd[i + k] & m) | ((wd[i + k] >> (16 - R)) & (m << R));
        if (R <= 4) {
          const uint32_t bits = t[0] | (t[1] << (2 * R)) | (t[2] << (4 * R)) | (t[3] << (6 * R));
          memcpy(o, &bits, R);
        } else {
          const uint64_t bits = t[0] | (static_cast<uint64_t>(t[1]) << (2 * R)) |
                                (static_cast<uint64_t>(t[2]) << (4 * R)) | (static_cast<uint64_t>(t[3]) << (6 * R));
          memcpy(o, &bits, R);
        }
        o += R;
      }
    } else {   // w 4: 8 samples a word -> R bytes
      for (uint32_t i = 0; i < r / 8; ++i) {
        const uint32_t x = wd[i];
        uint32_t bits = 0;
        for (int k = 0; k < 8; ++k) bits |= ((x >> (4 * k)) & m) << (R * k);
        memcpy(o, &bits, R);
        o += R;
      }
    }
  }
  static void packRaw(const Plan &p, const uint8_t *src, uint32_t r, uint8_t *out) {
    const uint32_t R = p.raw;
    if (!R) return;
    if (rawBits(p) == p.w) { memcpy(out, src, static_cast<size_t>(r) * p.w / 8); return; }
    const uint32_t *wd = reinterpret_cast<const uint32_t *>(src);
    switch (p.w * 16 + R) {
      case 8 * 16 + 1: return packT<8, 1>(wd, r, out);
      case 8 * 16 + 2: return packT<8, 2>(wd, r, out);
      case 8 * 16 + 3: return packT<8, 3>(wd, r, out);
      case 8 * 16 + 4: return packT<8, 4>(wd, r, out);
      case 8 * 16 + 5: return packT<8, 5>(wd, r, out);
      case 8 * 16 + 6: return packT<8, 6>(wd, r, out);
      case 8 * 16 + 7: return packT<8, 7>(wd, r, out);
      case 16 * 16 + 1: return packT<16, 1>(wd, r, out);
      case 16 * 16 + 2: return packT<16, 2>(wd, r, out);
      case 16 * 16 + 3: return packT<16, 3>(wd, r, out);
      case 16 * 16 + 4: return packT<16, 4>(wd, r, out);
      case 16 * 16 + 5: return packT<16, 5>(wd, r, out);
      case 16 * 16 + 6: return packT<16, 6>(wd, r, out);
      case 16 * 16 + 7: return packT<16, 7>(wd, r, out);
      case 16 * 16 + 8: return packT<16, 8>(wd, r, out);
      case 4 * 16 + 1: return packT<4, 1>(wd, r, out);
      case 4 * 16 + 2: return packT<4, 2>(wd, r, out);
      case 4 * 16 + 3: return packT<4, 3>(wd, r, out);
      default: referenceRaw(p, src, r, out); return;
    }
  }
  // Every group: per interval the whole-word aggregates (first, last, OR, AND, a change), then each lane's bit to its
  // channel's place (channel c's interval k at bit `off_c + k * b` of the reduced part, after the starts).
  void fastReduced(const Plan &p, const uint8_t *src, uint32_t r, uint8_t *red, size_t at, uint32_t before) {
    if (!p.reduced) return;
    Group g[kMaxChannels];
    const uint8_t ng = groups(p, g);
    size_t off[kMaxChannels];   // channel c's first value bit
    for (uint8_t c = 0; c < p.reduced; ++c) {
      off[c] = at;
      at += static_cast<size_t>(r / p.d[c]) * valueBits(p.kind[c]);
    }
    const uint32_t *wd = reinterpret_cast<const uint32_t *>(src);
    const uint32_t W = p.w, per = 32u / W, smask = W == 32 ? 0xFFFFFFFFu : (1u << W) - 1;
    for (uint8_t gi = 0; gi < ng; ++gi) {
      const Group &gr = g[gi];
      const uint32_t words = gr.d / per, lmask = (1u << gr.n) - 1;
      uint32_t prev = before;   // the sample before the interval (change_latch)
      for (uint32_t k = 0; k < r / gr.d; ++k) {
        const uint32_t *x = wd + k * words;
        uint32_t v;
        if (gr.kind == kSample) {
          v = x[0] & smask;
        } else if (gr.kind == kAnyHigh) {
          uint32_t o = 0;
          for (uint32_t i = 0; i < words; ++i) o |= x[i];
          v = fold(o, W, false);
        } else if (gr.kind == kAnyLow) {
          uint32_t a = 0xFFFFFFFFu;
          for (uint32_t i = 0; i < words; ++i) a &= x[i];
          v = fold(a, W, true);
        } else {   // change_latch: XOR of each sample with the one before, ORed
          uint32_t ch = 0, last = prev;
          for (uint32_t i = 0; i < words; ++i) {
            const uint32_t y = x[i];
            ch |= y ^ ((y << W) | (last & smask));
            last = y >> (32 - W);
          }
          prev = last;
          v = (last & smask) | (fold(ch, W, false) << 16);   // level in bits 0-15, change in 16-31
        }
        lvs[k & 31] = (v >> gr.lane) & lmask;
        if (gr.kind == kChangeLatch) cvs[k & 31] = (v >> (16 + gr.lane)) & lmask;
        if ((k & 31) == 31 || k + 1 == r / gr.d) flush(gr, k & ~31u, (k & 31) + 1, off, red);
      }
    }
  }
  // 32 intervals' lane values at a time -> each channel's word of them, written at its place (channel c's interval k at
  // bit off[c] + k * b): a lane's bits gathered, then whole bytes stored, not one bit at a time
  uint32_t lvs[32] = {}, cvs[32] = {};
  void flush(const Group &gr, uint32_t k0, uint32_t cnt, const size_t *off, uint8_t *red) {
    for (uint8_t i = 0; i < gr.n; ++i) {
      uint32_t lw = 0, cw = 0;
      for (uint32_t j = 0; j < cnt; ++j) lw |= ((lvs[j] >> i) & 1u) << j;
      const uint8_t c = gr.chan[i];
      if (gr.kind == kChangeLatch) {
        for (uint32_t j = 0; j < cnt; ++j) cw |= ((cvs[j] >> i) & 1u) << j;
        const uint64_t both = static_cast<uint64_t>(spread(lw & 0xFFFF) | (spread(cw & 0xFFFF) << 1)) |
                              (static_cast<uint64_t>(spread(lw >> 16) | (spread(cw >> 16) << 1)) << 32);
        writeBits(red, off[c] + 2 * k0, both, 2 * cnt);
      } else {
        writeBits(red, off[c] + k0, lw, cnt);
      }
    }
  }
  static uint32_t spread(uint32_t x) {   // 16 bits -> bit i at bit 2i
    x = (x | (x << 8)) & 0x00FF00FFu;
    x = (x | (x << 4)) & 0x0F0F0F0Fu;
    x = (x | (x << 2)) & 0x33333333u;
    return (x | (x << 1)) & 0x55555555u;
  }
  static void writeBits(uint8_t *buf, size_t bit, uint64_t v, uint32_t n) {   // n bits of v ORed in from `bit`
    while (n) {
      const uint32_t sh = bit & 7, take = 8 - sh < n ? 8 - sh : n;
      buf[bit >> 3] |= static_cast<uint8_t>((v & ((1u << take) - 1)) << sh);
      v >>= take;
      bit += take;
      n -= take;
    }
  }
  // the samples of a word ORed (or ANDed) into one sample's bits
  static uint32_t fold(uint32_t x, uint32_t W, bool all) {
    for (uint32_t s = 16; s >= W; s >>= 1) x = all ? x & (x >> s) : x | (x >> s);
    return x & ((1u << W) - 1);
  }

  // ---- the reference: any plan, bit by bit ----
  static void referenceRaw(const Plan &p, const uint8_t *src, uint32_t r, uint8_t *out) {
    if (rawBits(p) == p.w && p.raw) {   // as taken (raw channel j on lane j)
      memcpy(out, src, (static_cast<size_t>(r) * p.w + 7) / 8);
      return;
    }
    size_t j = 0;
    for (uint32_t i = 0; i < r; ++i) {
      const uint32_t s = sample(p, src, i);
      for (uint8_t k = 0; k < p.raw; ++k, ++j)
        if ((s >> p.raw_bit[k]) & 1) out[j >> 3] |= static_cast<uint8_t>(1u << (j & 7));
    }
  }
  static void referenceReduced(const Plan &p, const uint8_t *src, uint32_t r, uint8_t *red, size_t at, uint32_t before) {
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
  }
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
