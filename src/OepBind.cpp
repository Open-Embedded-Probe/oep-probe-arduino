// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepBind.h"

#include <stdio.h>
#include <string.h>

namespace oep {

void Binds::set(uint8_t port, const Spec &spec) {
  if (port >= kMaxPorts) return;
  specs_[port] = spec;
  selected_[port] = spec.mode == kManual ? spec.selected : 0;   // last-reset starts at the first
  pending_length_[port] = pending_at_[port] = 0;
  for (size_t i = 0; i < kMaxStreams; ++i) {
    flows_[port][i] = Flow{};
    resets_[port][i] = Reset{};
    const BindSource *s = i < spec.count ? spec.sources[i].stream : nullptr;
    seen_resets_[port][i] = s ? s->hostResets() : 0;
    if (const PositionStream *ps = s ? s->bindStream() : nullptr) {   // from now
      Flow &f = flows_[port][i];
      f.known = true;
      f.number = s->bindStreamNumber();
      f.pos = ps->end();
    }
  }
}

const PositionStream *Binds::follow(uint8_t port, uint8_t index) {
  const Spec &b = specs_[port];
  BindSource *src = index < b.count ? b.sources[index].stream : nullptr;
  const PositionStream *s = src ? src->bindStream() : nullptr;
  if (!s) return nullptr;
  Flow &f = flows_[port][index];
  if (!f.known || f.number != src->bindStreamNumber()) {   // a stream first seen: from now; a new one: its start
    f.pos = f.known ? s->oldest() : s->end();
    f.known = true;
    f.number = src->bindStreamNumber();
    f.line_length = 0;
  }
  if (f.pos < s->oldest()) f.pos = s->oldest();   // a whole buffer went past the port
  if (f.pos > s->end()) f.pos = s->end();
  return s;
}

bool Binds::streaming(uint8_t port) const {
  if (port >= kMaxPorts || !specs_[port].set) return false;
  const Spec &b = specs_[port];
  for (uint8_t i = 0; i < b.count; ++i) {
    if (b.mode != kMixed && i != selected_[port]) continue;
    if (b.sources[i].stream && b.sources[i].stream->bindStream()) return true;
  }
  return false;
}

void Binds::rawIn(uint8_t port, const uint8_t *data, size_t length) {
  if (port >= kMaxPorts || !specs_[port].set || specs_[port].mode == kMixed) return;   // mixed takes no input
  BindSource *src = specs_[port].sources[selected_[port]].stream;
  if (src) src->bindInput(data, length);   // what the other end cannot take now is dropped
}

size_t Binds::rawOut(uint8_t port, uint8_t *out, size_t room) {
  if (port >= kMaxPorts || !specs_[port].set || !room) return 0;
  if (specs_[port].mode != kMixed) {
    const uint8_t i = selected_[port];
    const PositionStream *s = follow(port, i);
    if (!s) return 0;
    Flow &f = flows_[port][i];
    const uint8_t *data = nullptr;
    size_t n = f.pos < s->end() ? s->contiguous(f.pos, data) : 0;
    if (n > room) n = room;
    if (n) memcpy(out, data, n);
    return n;
  }
  if (pending_at_[port] >= pending_length_[port] && !mixedLine(port)) return 0;
  size_t n = pending_length_[port] - pending_at_[port];
  if (n > room) n = room;
  memcpy(out, pending_[port] + pending_at_[port], n);
  return n;
}

void Binds::rawSent(uint8_t port, size_t length) {
  if (port >= kMaxPorts || !specs_[port].set) return;
  if (specs_[port].mode != kMixed) flows_[port][selected_[port]].pos += length;
  else pending_at_[port] = static_cast<uint16_t>(pending_at_[port] + length);
}

// mixed: take new bytes of every stream into its line; the first line closed (LF, kLineMax, or kQuietMs of quiet)
// goes into pending_ with its mark.
bool Binds::mixedLine(uint8_t port) {
  const Spec &b = specs_[port];
  const uint32_t now = millis();
  for (uint8_t i = 0; i < b.count; ++i) {
    const PositionStream *s = follow(port, i);
    Flow &f = flows_[port][i];
    bool closed = false;
    while (s && f.pos < s->end() && !closed) {
      const uint8_t *data = nullptr;
      const size_t n = s->contiguous(f.pos, data);
      size_t k = 0;
      while (k < n && f.line_length < kLineMax) {
        const uint8_t c = data[k++];
        f.line[f.line_length++] = c;
        if (c == '\n') { closed = true; break; }
      }
      f.pos += k;
      f.last_ms = now;
      if (f.line_length >= kLineMax) closed = true;
      if (!k) break;
    }
    if (!closed && f.line_length && static_cast<uint32_t>(now - f.last_ms) >= kQuietMs) closed = true;
    if (!closed) continue;
    char name[40] = "?";
    size_t name_length = namer_ ? namer_(namer_context_, b.sources[i].kind, b.sources[i].id, name, sizeof name) : 1;
    if (name_length > sizeof name) name_length = sizeof name;
    uint8_t *p = pending_[port];
    size_t at = 0;
    p[at++] = '[';
    memcpy(p + at, name, name_length);
    at += name_length;
    p[at++] = ']';
    p[at++] = ' ';
    memcpy(p + at, f.line, f.line_length);
    at += f.line_length;
    if (f.line[f.line_length - 1] != '\n') p[at++] = '\n';
    f.line_length = 0;
    pending_length_[port] = static_cast<uint16_t>(at);
    pending_at_[port] = 0;
    return true;
  }
  return false;
}

// Host resets (probe.config §1.2): last-reset follows the reset target; during a session each stream's position at
// the reset is kept for the port to start from when the session ends (a UART's at any host reset).
void Binds::poll(bool session) {
  for (uint8_t port = 0; port < kMaxPorts; ++port) {
    const Spec &b = specs_[port];
    if (!b.set) continue;
    bool any = false;
    for (uint8_t i = 0; i < b.count; ++i) {
      const BindSource *src = b.sources[i].stream;
      if (!src || b.sources[i].kind != kSlotConsole) continue;
      const uint32_t r = src->hostResets();
      if (r == seen_resets_[port][i]) continue;
      seen_resets_[port][i] = r;
      any = true;
      if (b.mode == kLastReset) selected_[port] = i;
      if (session) {
        if (const PositionStream *s = src->bindStream())
          resets_[port][i] = {true, src->bindStreamNumber(), s->end()};
      }
    }
    if (!any || !session) continue;
    for (uint8_t i = 0; i < b.count; ++i) {
      const BindSource *src = b.sources[i].stream;
      if (!src || b.sources[i].kind != kFixtureUart) continue;
      if (const PositionStream *s = src->bindStream()) resets_[port][i] = {true, src->bindStreamNumber(), s->end()};
    }
  }
}

void Binds::sessionOver(uint32_t held) {
  for (uint8_t port = 0; port < kMaxPorts; ++port) {
    const Spec &b = specs_[port];
    for (uint8_t i = 0; b.set && ((held >> port) & 1) && i < b.count; ++i) {
      BindSource *src = b.sources[i].stream;
      const PositionStream *s = src ? src->bindStream() : nullptr;
      if (!s) continue;
      Flow &f = flows_[port][i];
      const Reset &r = resets_[port][i];
      f.known = true;
      f.number = src->bindStreamNumber();
      f.pos = r.have && r.number == f.number ? r.pos : s->end();
      f.line_length = 0;
    }
    for (uint8_t i = 0; i < kMaxStreams; ++i) resets_[port][i] = Reset{};
    if ((held >> port) & 1) pending_length_[port] = pending_at_[port] = 0;
  }
}

}  // namespace oep
