// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The probe's channels as a set: kChannelCount channels (the core's channel is a u16, oep-core §8), a uint64_t where 64
// are enough (every platform but the CH32 with pads past channel 63) and WideMask<words> past that. A CH32 channel is
// port x 24 + pin (OepPlatform.h platformPad): the H41x's PF15 is channel 135, a V307's PE15 111. The X035 (24-pin
// ports, PC16-PC23 past 63) stays at 64: its pads past them are its own (USB, SWD) and its flash is short.
// OEP_CHANNEL_WORDS overrides the count (64-bit words).
#pragma once

#include <Arduino.h>   // the core's pin map (NUM_DIGITAL_PINS, CH32RV_GPIO_PORT_WIDTH)
#include <stddef.h>
#include <stdint.h>

#if !defined(OEP_CHANNEL_WORDS)
#if defined(ARDUINO_ARCH_CH32RV) && !defined(ARDUINO_CH32X035) && defined(NUM_DIGITAL_PINS) && defined(CH32RV_GPIO_PORT_WIDTH)
// pin = ((port + 2) << 5) | bit (the core's ADR-0010): the last port's channels end at port x 24 + the port's width
#define OEP_CHANNEL_WORDS ((((NUM_DIGITAL_PINS - 1) / 32 - 2) * 24 + CH32RV_GPIO_PORT_WIDTH + 63) / 64)
#else
#define OEP_CHANNEL_WORDS 1
#endif
#endif

namespace oep {

constexpr uint16_t kChannelCount = 64 * OEP_CHANNEL_WORDS;

// A set of channels past 64: what code does with a uint64_t mask (& | ~ ^, ==, a channel's bit) on W words.
template <size_t W>
struct WideMask {
  uint64_t w[W] = {};
  constexpr WideMask() = default;
  constexpr WideMask(uint64_t low) : w{} { w[0] = low; }   // channels 0-63 (implicit: a uint64_t mask reads the same)
  // friends, so a uint64_t mask on either side converts
  friend constexpr WideMask operator&(const WideMask &a, const WideMask &b) { WideMask r; for (size_t i = 0; i < W; ++i) r.w[i] = a.w[i] & b.w[i]; return r; }
  friend constexpr WideMask operator|(const WideMask &a, const WideMask &b) { WideMask r; for (size_t i = 0; i < W; ++i) r.w[i] = a.w[i] | b.w[i]; return r; }
  friend constexpr WideMask operator^(const WideMask &a, const WideMask &b) { WideMask r; for (size_t i = 0; i < W; ++i) r.w[i] = a.w[i] ^ b.w[i]; return r; }
  constexpr WideMask operator~() const { WideMask r; for (size_t i = 0; i < W; ++i) r.w[i] = ~w[i]; return r; }
  WideMask &operator&=(const WideMask &o) { return *this = *this & o; }
  WideMask &operator|=(const WideMask &o) { return *this = *this | o; }
  friend constexpr bool operator==(const WideMask &a, const WideMask &b) { for (size_t i = 0; i < W; ++i) if (a.w[i] != b.w[i]) return false; return true; }
  friend constexpr bool operator!=(const WideMask &a, const WideMask &b) { return !(a == b); }
  constexpr explicit operator bool() const { for (size_t i = 0; i < W; ++i) if (w[i]) return true; return false; }
  constexpr bool operator!() const { return !static_cast<bool>(*this); }
};

#if OEP_CHANNEL_WORDS == 1
using ChannelMask = uint64_t;
using ChannelIndex = uint8_t;    // a channel number inside the pin table's loops
#else
using ChannelMask = WideMask<OEP_CHANNEL_WORDS>;
using ChannelIndex = uint16_t;
#endif

// Channel c's bit, whether a set holds c, and c added - for either kind
constexpr bool hasChannel(uint64_t m, uint16_t c) { return c < 64 && ((m >> c) & 1); }
template <size_t W>
constexpr bool hasChannel(const WideMask<W> &m, uint16_t c) { return c < 64 * W && ((m.w[c / 64] >> (c % 64)) & 1); }
// a channel's bit (c below kChannelCount: the callers' channels are the table's)
constexpr ChannelMask channelBit(uint16_t c) {
#if OEP_CHANNEL_WORDS == 1
  return uint64_t{1} << c;
#else
  ChannelMask m;
  if (c < kChannelCount) m.w[c / 64] = uint64_t{1} << (c % 64);
  return m;
#endif
}
// The highest channel in a set, or -1
inline int topChannel(const ChannelMask &m) {
  for (int c = kChannelCount - 1; c >= 0; --c)
    if (hasChannel(m, static_cast<uint16_t>(c))) return c;
  return -1;
}
// Bits [8i, 8i + 8) of a set: the bitmap bytes of a role_channels (core §7.4)
inline uint8_t channelByte(const ChannelMask &m, size_t i) {
#if OEP_CHANNEL_WORDS == 1
  return i < 8 ? static_cast<uint8_t>(m >> (8 * i)) : 0;
#else
  return i < 8 * OEP_CHANNEL_WORDS ? static_cast<uint8_t>(m.w[i / 8] >> (8 * (i % 8))) : 0;
#endif
}

}  // namespace oep
