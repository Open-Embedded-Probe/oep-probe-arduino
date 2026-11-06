// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepBind.h"

#include <stddef.h>
#include <string.h>

namespace oep {

void Binds::put(uint8_t port, const uint8_t *data, size_t length) {
  Ring *r = find(port);
  if (!r) {   // this port's first pass-through bytes: its ring, for good
    r = static_cast<Ring *>(takeForGood(offsetof(Ring, data) + ring_));
    if (!r) return;
    r->port = port;
    r->next = rings_;
    rings_ = r;
  }
  for (size_t i = 0; i < length; ++i) {
    if (r->count == ring_) { r->head = (r->head + 1) % ring_; --r->count; }   // full: the oldest goes
    r->data[(r->head + r->count) % ring_] = data[i];
    ++r->count;
  }
}

void Binds::drop(uint8_t port) {
  if (Ring *r = find(port)) r->head = r->count = 0;
}

size_t Binds::rawOut(uint8_t port, uint8_t *out, size_t room) {
  const Ring *r = find(port);
  if (!r) return 0;
  size_t n = r->count < room ? r->count : room;
  const size_t first = ring_ - r->head;   // in one piece up to the ring's end
  if (n > first) n = first;
  memcpy(out, r->data + r->head, n);
  return n;
}

void Binds::rawSent(uint8_t port, size_t length) {
  Ring *r = find(port);
  if (!r) return;
  if (length > r->count) length = r->count;
  r->head = (r->head + length) % ring_;
  r->count -= length;
}

}  // namespace oep
