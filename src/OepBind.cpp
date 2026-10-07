// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepBind.h"

#include <string.h>

namespace oep {

void Binds::set(uint8_t port, const Spec &spec) {
  if (port >= kMaxPorts) return;
  specs_[port] = spec;
  flows_[port] = Flow{};
  const BindSource *s = spec.set ? spec.source.stream : nullptr;
  if (const PositionStream *ps = s ? s->bindStream() : nullptr) {   // from now
    Flow &f = flows_[port];
    f.known = true;
    f.number = s->bindStreamNumber();
    f.pos = ps->end();
  }
}

const PositionStream *Binds::follow(uint8_t port) {
  BindSource *src = specs_[port].set ? specs_[port].source.stream : nullptr;
  const PositionStream *s = src ? src->bindStream() : nullptr;
  if (!s) return nullptr;
  Flow &f = flows_[port];
  if (!f.known || f.number != src->bindStreamNumber()) {   // a stream first seen: from now; a new one: its start
    f.pos = f.known ? s->oldest() : s->end();
    f.known = true;
    f.number = src->bindStreamNumber();
  }
  if (f.pos < s->oldest()) f.pos = s->oldest();   // a whole buffer went past the port
  if (f.pos > s->end()) f.pos = s->end();
  return s;
}

bool Binds::streaming(uint8_t port) const {
  if (port >= kMaxPorts || !specs_[port].set) return false;
  const BindSource *src = specs_[port].source.stream;
  return src && src->bindStream();
}

void Binds::rawIn(uint8_t port, const uint8_t *data, size_t length) {
  if (port >= kMaxPorts || !specs_[port].set) return;
  BindSource *src = specs_[port].source.stream;
  if (src) src->bindInput(data, length);   // what the other end cannot take now is dropped
}

size_t Binds::rawOut(uint8_t port, uint8_t *out, size_t room) {
  if (port >= kMaxPorts || !room) return 0;
  const PositionStream *s = follow(port);
  if (!s) return 0;
  const Flow &f = flows_[port];
  const uint8_t *data = nullptr;
  size_t n = f.pos < s->end() ? s->contiguous(f.pos, data) : 0;
  if (n > room) n = room;
  if (n) memcpy(out, data, n);
  return n;
}

void Binds::rawSent(uint8_t port, size_t length) {
  if (port >= kMaxPorts || !specs_[port].set) return;
  flows_[port].pos += length;
}

}  // namespace oep
