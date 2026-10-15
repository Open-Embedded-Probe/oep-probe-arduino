// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepBind.h"

#include <string.h>

namespace oep {

void Binds::put(uint8_t port, const uint8_t *data, size_t length) {
  if (port >= kMaxPorts) return;
  Ring &r = rings_[port];
  for (size_t i = 0; i < length; ++i) {
    if (r.count == kRing) { r.head = (r.head + 1) % kRing; --r.count; }   // full: the oldest goes
    r.data[(r.head + r.count) % kRing] = data[i];
    ++r.count;
  }
}

void Binds::drop(uint8_t port) {
  if (port < kMaxPorts) rings_[port] = Ring{};
}

size_t Binds::rawOut(uint8_t port, uint8_t *out, size_t room) {
  if (port >= kMaxPorts) return 0;
  const Ring &r = rings_[port];
  size_t n = r.count < room ? r.count : room;
  const size_t first = kRing - r.head;   // in one piece up to the ring's end
  if (n > first) n = first;
  memcpy(out, r.data + r.head, n);
  return n;
}

void Binds::rawSent(uint8_t port, size_t length) {
  if (port >= kMaxPorts) return;
  Ring &r = rings_[port];
  if (length > r.count) length = r.count;
  r.head = (r.head + length) % kRing;
  r.count -= length;
}

}  // namespace oep
