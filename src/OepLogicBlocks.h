// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Data format 1, the logic blocks (oep-if-capture §1.3): a block of r <= L original samples, taken from the capture's
// own w-bit samples, becomes
//   fallback(ceil(E/8)) start_level(ceil(E/8)) n(u32) raw part(ceil(r * w_b / 8)) n x event(tick u16, ch u8, state u8)
// with the raw channels (and the events channels carried raw in this block: fallback) packed w_b bits a sample, and
// for every other events channel one event a tick of D samples where its level changed (the level after the tick, bit
// 7; the changes in it, saturated at 127), in channel order and then tick order. A channel whose events would pass
// L / 8 bytes (L / 32 events) goes raw: it is required then (§1.3), and it bounds the work - counting stops there.
//
// No chip in here (the host tests run it against the shared vectors, tests/vectors/capture_formats.json). The cost is
// a look at every sample for a change on the events channels; w 8 looks four samples at a time and skips a word in which
// no events channel changed - an idle line costs about a cycle a byte.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

// The encoder's loops at -O2 whatever the sketch is built with (-Os made a word's load four byte loads and the shifts:
// 10 cycles a sample on the P4 where 2 or 3 do)
#pragma GCC push_options
#pragma GCC optimize("O2")

namespace oep {
namespace blocks {

constexpr size_t kMaxChannels = 16;
constexpr uint32_t kMaxBlock = 4096;            // L at most (the event scratch is sized for it)
constexpr uint32_t kMaxEvents = kMaxBlock / 32; // a channel's events before it must go raw (L / 8 bytes)
constexpr uint8_t kCountMax = 127;

struct Plan {
  uint8_t w = 8;                         // the capture's sample width: 1, 2, 4, 8 or 16 bits
  uint8_t raw = 0, events = 0;           // R, E
  uint8_t raw_bit[kMaxChannels] = {};    // raw channel j (role order): its bit in a sample
  uint8_t ev_bit[kMaxChannels] = {};     // events channel e (role order): its bit
  uint32_t d[kMaxChannels] = {};         // events channel e: D (1 or more)
  uint32_t block = 0;                    // L: a multiple of 8 and of every D, at most kMaxBlock
};

// The most bytes a block of `r` samples takes (§1.3's worst: every events channel at min(L / 8, (L / D) x 4)).
inline size_t worstBytes(const Plan &p, uint32_t r) {
  const size_t nb = (p.events + 7u) / 8u;
  size_t n = 2 * nb + 4 + (static_cast<size_t>(r) * p.raw + 7) / 8;
  for (uint8_t e = 0; e < p.events; ++e) {
    const size_t raw = (r + 7u) / 8u, ev = static_cast<size_t>((r + p.d[e] - 1) / p.d[e]) * 4u;
    n += raw < ev ? raw : ev;
  }
  return n;
}

class Encoder {
 public:
  void reset() { have_before_ = false; }
  uint32_t fallbacks() const { return fallbacks_; }   // channels carried raw in the last block

  // Samples 0 .. r-1 of `src` (the capture's packed w-bit samples, sample 0 at bit 0) -> one block at `out`. `contiguous`:
  // the sample before sample 0 is the last one of the previous call (start_level, §1.3); else start_level is sample
  // 0's level. -> the bytes written, 0 when `capacity` cannot hold them.
  size_t encode(const Plan &p, const uint8_t *src, uint32_t r, uint8_t *out, size_t capacity, bool contiguous) {
    const uint8_t E = p.events;
    limit_ = p.block / 32 < kMaxEvents ? p.block / 32 : kMaxEvents;   // L / 8 bytes of events (§1.3)
    const uint32_t first = sample(p, src, 0);
    uint32_t prev = contiguous && have_before_ ? before_ : first;
    uint32_t evmask = 0;
    for (uint8_t e = 0; e < E; ++e) {
      evmask |= 1u << p.ev_bit[e];
      start_[e] = (prev >> p.ev_bit[e]) & 1;
      level_[e] = start_[e];
      tick_[e] = 0xFFFFFFFFu;
      count_[e] = 0;
      n_[e] = 0;
      raw_[e] = false;
    }
    if (E) {
      uint32_t i = 0;
      if (p.w == 8 && !(reinterpret_cast<uintptr_t>(src) & 3)) {   // four samples a word (src word-aligned): a word in
        // which no events channel changed is skipped whole
        const uint32_t *words = reinterpret_cast<const uint32_t *>(src);
        const uint32_t mask4 = (evmask & 0xFF) * 0x01010101u;
        while (i < r) {
          if (i + 4 <= r) {
            const uint32_t word = words[i >> 2];
            const uint32_t shifted = (word << 8) | (prev & 0xFF);
            if (((word ^ shifted) & mask4) == 0) {
              prev = word >> 24;
              i += 4;
              continue;
            }
          }
          const uint32_t end = i + 4 < r ? i + 4 : r;
          for (; i < end; ++i) prev = look(p, i, src[i], prev, evmask);
        }
      } else {
        for (; i < r; ++i) prev = look(p, i, sample(p, src, i), prev, evmask);
      }
      for (uint8_t e = 0; e < E; ++e) flush(e);
    }
    // the block: fallback, start_level, n, raw part, events
    const size_t nb = (E + 7u) / 8u;
    uint8_t cols_bit[2 * kMaxChannels];
    uint8_t cols = 0;
    for (uint8_t j = 0; j < p.raw; ++j) cols_bit[cols++] = p.raw_bit[j];
    uint32_t n = 0;
    fallbacks_ = 0;
    for (uint8_t e = 0; e < E; ++e) {
      if (raw_[e]) { cols_bit[cols++] = p.ev_bit[e]; ++fallbacks_; }
      else n += n_[e];
    }
    const size_t raw_bytes = (static_cast<size_t>(r) * cols + 7) / 8;
    const size_t total = 2 * nb + 4 + raw_bytes + 4 * static_cast<size_t>(n);
    if (total > capacity) return 0;
    memset(out, 0, 2 * nb + 4 + raw_bytes);
    for (uint8_t e = 0; e < E; ++e) {
      if (raw_[e]) out[e / 8] |= static_cast<uint8_t>(1u << (e % 8));
      if (start_[e]) out[nb + e / 8] |= static_cast<uint8_t>(1u << (e % 8));
    }
    out[2 * nb] = static_cast<uint8_t>(n);
    out[2 * nb + 1] = static_cast<uint8_t>(n >> 8);
    out[2 * nb + 2] = static_cast<uint8_t>(n >> 16);
    out[2 * nb + 3] = static_cast<uint8_t>(n >> 24);
    uint8_t *raw = out + 2 * nb + 4;
    bool same = cols == p.w;   // every bit of the sample, in order: the samples as they came
    for (uint8_t k = 0; k < cols && same; ++k) same = cols_bit[k] == k;
    if (same) {
      memcpy(raw, src, raw_bytes);
      if ((static_cast<size_t>(r) * cols) % 8) raw[raw_bytes - 1] &= static_cast<uint8_t>((1u << ((r * cols) % 8)) - 1);
    } else if (p.w == 8 && (cols == 1 || cols == 2 || cols == 4) && lowBits(cols_bit, cols) &&
               !(reinterpret_cast<uintptr_t>(src) & 3)) {
      packLow(src, r, cols, raw);   // the raw channels on the low bits: a word of four samples at a time
    } else if (cols) {
      size_t bit = 0;
      for (uint32_t i = 0; i < r; ++i) {
        const uint32_t s = sample(p, src, i);
        for (uint8_t k = 0; k < cols; ++k, ++bit)
          if ((s >> cols_bit[k]) & 1) raw[bit >> 3] |= static_cast<uint8_t>(1u << (bit & 7));
      }
    }
    uint8_t *ev = raw + raw_bytes;
    for (uint8_t e = 0; e < E; ++e) {
      if (raw_[e]) continue;
      for (uint32_t k = 0; k < n_[e]; ++k, ev += 4) {
        const uint32_t v = events_[e][k];
        ev[0] = static_cast<uint8_t>(v);
        ev[1] = static_cast<uint8_t>(v >> 8);
        ev[2] = e;
        ev[3] = static_cast<uint8_t>(v >> 16);
      }
    }
    before_ = r ? sample(p, src, r - 1) : prev;
    have_before_ = true;
    return total;
  }

  static bool lowBits(const uint8_t *bits, uint8_t cols) {
    for (uint8_t k = 0; k < cols; ++k)
      if (bits[k] != k) return false;
    return true;
  }
  // w 8 samples whose low `cols` bits (1, 2 or 4) are the columns, packed cols bits a sample: four samples a 32-bit
  // word, gathered by shifts and masks
  static void packLow(const uint8_t *src, uint32_t r, uint8_t cols, uint8_t *raw) {
    uint32_t i = 0;
    size_t bit = 0;
    const uint32_t *words = reinterpret_cast<const uint32_t *>(src);   // word-aligned (the caller checked)
    for (; i + 4 <= r; i += 4, bit += 4u * cols) {
      uint32_t x = words[i >> 2];
      uint32_t v;
      if (cols == 4) {
        x &= 0x0F0F0F0Fu;
        x = (x | (x >> 4)) & 0x00FF00FFu;
        v = (x | (x >> 8)) & 0xFFFFu;
      } else if (cols == 2) {
        x &= 0x03030303u;
        x = (x | (x >> 6)) & 0x000F000Fu;
        v = (x | (x >> 12)) & 0xFFu;
      } else {
        x &= 0x01010101u;
        x = (x | (x >> 7)) & 0x00030003u;
        v = (x | (x >> 14)) & 0xFu;
      }
      const size_t at = bit >> 3, shift = bit & 7;   // 4 x cols bits: whole bytes for 2 and 4, a nibble for 1
      raw[at] |= static_cast<uint8_t>(v << shift);
      if (cols == 4) raw[at + 1] = static_cast<uint8_t>(v >> 8);
    }
    for (; i < r; ++i, bit += cols)
      for (uint8_t k = 0; k < cols; ++k)
        if ((src[i] >> k) & 1) raw[(bit + k) >> 3] |= static_cast<uint8_t>(1u << ((bit + k) & 7));
  }

  static uint32_t sample(const Plan &p, const uint8_t *src, uint32_t i) {
    if (p.w == 16) return static_cast<uint32_t>(src[2 * i] | src[2 * i + 1] << 8);
    if (p.w == 8) return src[i];
    const uint32_t bit = i * p.w;
    return (src[bit >> 3] >> (bit & 7)) & ((1u << p.w) - 1);
  }

 private:
  // sample i (value s) after `prev`: each events channel that changed counts in its tick
  uint32_t look(const Plan &p, uint32_t i, uint32_t s, uint32_t prev, uint32_t evmask) {
    uint32_t x = (s ^ prev) & evmask;
    if (!x) return s;
    for (uint8_t e = 0; e < p.events && x; ++e) {
      const uint32_t bit = 1u << p.ev_bit[e];
      if (!(x & bit)) continue;
      x &= ~bit;
      if (raw_[e]) continue;
      const uint32_t t = i / p.d[e];
      if (t != tick_[e]) {
        flush(e);
        if (raw_[e]) continue;
        tick_[e] = t;
      }
      if (count_[e] < kCountMax) ++count_[e];
      level_[e] = (s & bit) ? 1 : 0;
    }
    return s;
  }
  // the tick being counted becomes an event; past L / 32 events the channel goes raw
  void flush(uint8_t e) {
    if (!count_[e]) return;
    if (n_[e] >= limit_) { raw_[e] = true; count_[e] = 0; return; }
    events_[e][n_[e]++] = (tick_[e] & 0xFFFF) | (static_cast<uint32_t>(level_[e] << 7 | count_[e]) << 16);
    count_[e] = 0;
  }

  uint32_t limit_ = kMaxEvents;   // a channel's events in this block before it goes raw
  uint32_t before_ = 0;
  bool have_before_ = false;
  uint32_t fallbacks_ = 0;
  uint8_t start_[kMaxChannels] = {}, level_[kMaxChannels] = {}, count_[kMaxChannels] = {};
  bool raw_[kMaxChannels] = {};
  uint32_t tick_[kMaxChannels] = {}, n_[kMaxChannels] = {};
  uint32_t events_[kMaxChannels][kMaxEvents] = {};
};

}  // namespace blocks
}  // namespace oep

#pragma GCC pop_options
