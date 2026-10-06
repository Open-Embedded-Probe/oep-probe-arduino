// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests of fn 0 (oep.core) against core §1.2, §7.1, §7.5 and §12: confirm's transport TLV on every transport kind
// (the index of the describe entry it came on), the confirm vectors of oep-spec tests/vectors/confirm.json, describe's
// discoverable always sent, list's reserved flags and prefix text, open's force and owner, a repeated non-repeating TLV,
// the header refusals (§4.3 order 1) neither kept nor restarting the lease, and the length-prefixed reader's over-long
// length and TCP pause rules (§3.1, §3.2).
#include <stdio.h>
#include <string.h>
#include <vector>

#include "OepFrame.h"
#include "OepEndpoint.h"

uint32_t g_millis = 1000;

using Bytes = std::vector<uint8_t>;
using namespace oep;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

class MemStream final : public Stream {
 public:
  Bytes rx, tx;
  size_t at = 0;
  int available() override { return static_cast<int>(rx.size() - at); }
  int read() override { return at < rx.size() ? rx[at++] : -1; }
  int peek() override { return at < rx.size() ? rx[at] : -1; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override { tx.insert(tx.end(), b, b + n); return n; }
  int availableForWrite() override { return 4096; }
  void send(const Bytes &b) { rx.insert(rx.end(), b.begin(), b.end()); }
};

static Bytes u32(uint32_t v) { return {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)}; }
static Bytes request(uint16_t corr, uint16_t fn, uint8_t op, const Bytes &payload, bool session = false, uint32_t id = 0) {
  Bytes m = {uint8_t(session ? 0x81 : 0x01), uint8_t(corr), uint8_t(corr >> 8), uint8_t(fn), uint8_t(fn >> 8), op};
  if (session) { const Bytes s = u32(id); m.insert(m.end(), s.begin(), s.end()); }
  m.insert(m.end(), payload.begin(), payload.end());
  return m;
}
static Bytes confirmReq(uint8_t lo = 1, uint8_t hi = 1) { return {'O', 'E', 'P', '?', lo, hi}; }
static Bytes openReq(uint32_t id, uint32_t lease, uint8_t force = 0) {
  Bytes p = u32(id);
  const Bytes l = u32(lease);
  p.insert(p.end(), l.begin(), l.end());
  p.push_back(force);
  return p;
}
static Bytes withTlv(Bytes p, uint8_t tag, const Bytes &value) {
  p.push_back(tag);
  p.push_back(static_cast<uint8_t>(value.size()));
  p.insert(p.end(), value.begin(), value.end());
  return p;
}
static Bytes hex(const char *h) {
  Bytes b;
  for (; h[0] && h[1]; h += 2) { unsigned v; sscanf(h, "%2x", &v); b.push_back(static_cast<uint8_t>(v)); }
  return b;
}

// What one transport sends and gets back, in its own framing (COBS on the serial ports, length-prefixed elsewhere).
static bool unframe(const Bytes &enc, Bytes &out) {
  out.clear();
  size_t in = 0;
  while (in < enc.size()) {
    const uint8_t code = enc[in++];
    if (code == 0 || in + code - 1 > enc.size()) return false;
    out.insert(out.end(), enc.begin() + in, enc.begin() + in + code - 1);
    in += code - 1;
    if (code != 0xff && in < enc.size()) out.push_back(0);
  }
  if (out.size() < 3) return false;
  const uint16_t got = static_cast<uint16_t>(out[out.size() - 2] | out[out.size() - 1] << 8);
  out.resize(out.size() - 2);
  return crc16Ccitt(out.data(), out.size()) == got;
}
// -> the whole answer message (header included), empty when nothing came back
static Bytes exchange(Endpoint &ep, MemStream &s, bool serial, const Bytes &m) {
  s.tx.clear();
  if (serial) {
    MemStream f;
    writeCobsFrame(f, m.data(), m.size());
    s.send(f.tx);
  } else {
    s.send({uint8_t(m.size()), uint8_t(m.size() >> 8)});
    s.send(m);
  }
  ep.poll();
  if (serial) {
    if (s.tx.size() < 3 || s.tx.front() != 0 || s.tx.back() != 0) return {};
    Bytes body(s.tx.begin() + 1, s.tx.end() - 1), msg;
    return unframe(body, msg) ? msg : Bytes{};
  }
  if (s.tx.size() < 2) return {};
  return Bytes(s.tx.begin() + 2, s.tx.end());
}

// The transport entries of fn 0's describe (index -> kind), from the answer of describe fn 0 first 0.
static bool describedAs(const Bytes &answer, uint8_t index, uint8_t kind) {
  for (size_t i = 6; i + 5 <= answer.size(); ++i)
    if (answer[i] == reg::core::kTlvDescribeTransport && answer[i + 1] == 3 && answer[i + 2] == index && answer[i + 3] == kind)
      return true;
  return false;
}
static int discoverableIn(const Bytes &answer) {   // -1: absent
  for (size_t i = 6; i + 3 <= answer.size();) {
    if (answer[i] == reg::core::kTlvDescribeDiscoverable && answer[i + 1] == 1) return answer[i + 2];
    i += 2 + answer[i + 1];
  }
  return -1;
}

// core §7.1: the probe always attaches TLV 0x01 transport (u8), the index of the transport the confirm came on, the
// entry of describe's transport list. Every kind: UART bridge, USB CDC, built-in USB serial, vendor bulk, HID, TCP.
static void testConfirmTransportEveryKind() {
  struct Port { MemStream s; uint8_t kind; bool serial; };
  static uint8_t rx[6][1200], tx[2][1100];
  Port a[4] = {{{}, Endpoint::kUartBridge, true}, {{}, Endpoint::kUsbCdc, true}, {{}, Endpoint::kVendorBulk, false},
               {{}, Endpoint::kHid, false}};
  Endpoint ea(a[0].s, rx[0], sizeof rx[0], tx[0], sizeof tx[0], {1024, 4096, 4}, Endpoint::kUartBridge);
  CHECK(ea.addTransport(a[1].s, rx[1], sizeof rx[1], Endpoint::kUsbCdc, 0));
  CHECK(ea.addTransport(a[2].s, rx[2], sizeof rx[2], Endpoint::kVendorBulk, 2));
  CHECK(ea.addTransport(a[3].s, rx[3], sizeof rx[3], Endpoint::kHid, 3));
  Port b[2] = {{{}, Endpoint::kUsbSerialJtag, true}, {{}, Endpoint::kTcp, false}};
  Endpoint eb(b[0].s, rx[4], sizeof rx[4], tx[1], sizeof tx[1], {1024, 4096, 4}, Endpoint::kUsbSerialJtag);
  CHECK(eb.addTransport(b[1].s, rx[5], sizeof rx[5], Endpoint::kTcp));
  struct Case { Endpoint *ep; Port *port; uint8_t index; };
  const Case cases[] = {{&ea, &a[0], 0}, {&ea, &a[1], 1}, {&ea, &a[2], 2}, {&ea, &a[3], 3}, {&eb, &b[0], 0}, {&eb, &b[1], 1}};
  uint16_t corr = 1;
  for (const Case &c : cases) {
    const Bytes r = exchange(*c.ep, c.port->s, c.port->serial, request(corr++, 0, 0x01, confirmReq()));
    CHECK(r.size() == 5 + 20 && r[3] == kResolutionCompleted);
    if (r.size() == 5 + 20) CHECK(r[5 + 17] == reg::core::kTlvConfirmAnswerTransport && r[5 + 18] == 1 && r[5 + 19] == c.index);
    // that index names this transport's entry in the describe returned on the same connection
    const Bytes d = exchange(*c.ep, c.port->s, c.port->serial, request(corr++, 0, 0x03, {0, 0, 0, 0}));
    CHECK(d.size() > 6 && d[3] == kResolutionCompleted && describedAs(d, c.index, c.port->kind));
  }
}

// oep-spec tests/vectors/confirm.json, byte for byte (max_frame 1024, window 4096, max_inflight 4, boot_id 0x12345678,
// transport index 0), and a range with min_rev > max_rev (refusals.json): malformed.
static void testConfirmVectors() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kUartBridge);
  ep.setBootId(0x12345678);
  CHECK(exchange(ep, s, true, hex("0101000000014f45503f0101")) == hex("02010001004f45502101000004001000000478563412010100"));
  CHECK(exchange(ep, s, true, hex("0102000000014f45503f0203")) == hex("020200000b0001020101"));
  CHECK(exchange(ep, s, true, hex("011a000000014f45503f0201")) == hex("021a000003"));
}

// core §7.5: discoverable is always sent - 0 by a probe that does not enumerate with the project's VID:PID.
static void testDiscoverableAlways() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  CHECK(discoverableIn(exchange(ep, s, false, request(1, 0, 0x03, {0, 0, 0, 0}))) == 0);
  ep.setDiscoverable(true);
  CHECK(discoverableIn(exchange(ep, s, false, request(2, 0, 0x03, {0, 0, 0, 0}))) == 1);
}

static uint8_t reason(const Bytes &r) { return r.size() >= 5 && r[3] == kResolutionRejected ? r[4] : 0xff; }

// core §7.2: list's flags bits 1 to 7 are reserved (unsupported, tag 0x00); the prefix is text (core §2.1, malformed).
// core §6.4 / §2.1: open's force is a boolean; owner is text of 1 to 32 bytes; core §2.3: a repeated non-repeating TLV.
static void testRequestValues() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  Bytes r = exchange(ep, s, false, request(1, 0, 0x02, {0x02, 0, 0, 0}));
  CHECK(reason(r) == kRejectUnsupported && r.size() == 6 && r[5] == 0);
  CHECK(reason(exchange(ep, s, false, request(2, 0, 0x02, {0, 0, 0, 3, 'o', 0x01, 'p'}))) == kRejectMalformed);
  CHECK(reason(exchange(ep, s, false, request(3, 0, 0x02, {0, 0, 0, 2, 0xC3, 0x28}))) == kRejectMalformed);   // bad UTF-8
  r = exchange(ep, s, false, request(4, 0, 0x02, {1, 0, 0, 3, 'o', 'e', 'p'}));   // exact "oep": none, fine
  CHECK(r.size() >= 8 && r[3] == kResolutionCompleted && r[5 + 2] == 0);

  CHECK(reason(exchange(ep, s, false, request(5, 0, 0x10, openReq(9, 3000, 2)))) == kRejectMalformed);
  CHECK(reason(exchange(ep, s, false, request(6, 0, 0x10, withTlv(openReq(9, 3000), 0x01, {})))) == kRejectMalformed);
  CHECK(reason(exchange(ep, s, false, request(7, 0, 0x10, withTlv(openReq(9, 3000), 0x01, Bytes(33, 'a'))))) == kRejectMalformed);
  CHECK(reason(exchange(ep, s, false, request(8, 0, 0x10, withTlv(openReq(9, 3000), 0x81, {'a', 0x0a})))) == kRejectMalformed);
  CHECK(reason(exchange(ep, s, false, request(9, 0, 0x10, withTlv(openReq(9, 3000), 0x01, {0xED, 0xA0, 0x80})))) == kRejectMalformed);
  const Bytes twice = withTlv(withTlv(openReq(9, 3000), 0x01, {'a'}), 0x01, {'b'});
  CHECK(reason(exchange(ep, s, false, request(10, 0, 0x10, twice))) == kRejectMalformed);
  CHECK(!ep.locked());
  r = exchange(ep, s, false, request(11, 0, 0x10, withTlv(openReq(9, 3000, 1), 0x01, {'t', 0xC3, 0xA9, 's', 't'})));
  CHECK(r.size() >= 5 + 9 && r[3] == kResolutionCompleted && ep.locked());
  r = exchange(ep, s, false, request(12, 0, 0x13, {}));   // lock_state shows the owner as sent
  const Bytes owner = {0x01, 5, 't', 0xC3, 0xA9, 's', 't'};
  CHECK(r.size() == 5 + 5 + owner.size() && Bytes(r.begin() + 10, r.end()) == owner);
}

// core §4.3 order 1 comes before the resend table (order 2): a header refusal of the last session's request is not
// kept (its corr is free for the next request) and restarts no lease (core §6.1: only what passed order 3).
static void testHeaderRefusalsNotKept() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  CHECK(exchange(ep, s, false, request(1, 0, 0x10, openReq(7, 1000)))[3] == kResolutionCompleted);
  g_millis += 900;
  CHECK(reason(exchange(ep, s, false, request(2, 5, 0x01, {}, true, 7))) == kRejectUnknownFunction);
  CHECK(reason(exchange(ep, s, false, request(3, 0, 0x50, {}, true, 7))) == kRejectUnknownOperation);
  CHECK(reason(exchange(ep, s, false, request(4, 0, 0x04, {}, true, 7))) == kRejectUnknownOperation);   // no plan roles
  CHECK(reason(exchange(ep, s, false, request(5, 0, 0x14, {}, true, 7))) == kRejectUnknownOperation);   // port_speed off
  g_millis += 200;   // 1100 ms after the open: the refusals did not extend it
  ep.poll();
  CHECK(!ep.locked());
  // the expired session's next request: expired, and corr 2 was not taken by the refusal (no corr_reused)
  CHECK(reason(exchange(ep, s, false, request(2, 0, 0x12, {}, true, 7))) == kRejectExpired);
}

// core §3.1: a length over max_frame - that frame and the input up to the next pause of probe_frame_gap_ms are
// discarded, unanswered; the next frame after the pause is read. core §3.2: a pause inside a frame restarts the read
// on vendor bulk, not on TCP.
static void testLengthPrefixedReader() {
  for (const uint8_t kind : {Endpoint::kVendorBulk, Endpoint::kTcp}) {
    MemStream s;
    static uint8_t rx[1200], tx[1100];
    Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {256, 1024, 4}, kind, kind == Endpoint::kTcp ? 0xff : 0);
    const Bytes confirm = request(1, 0, 0x01, confirmReq());
    s.send({0x00, 0x10});   // 4096 > max_frame
    s.send({uint8_t(confirm.size()), 0});
    s.send(confirm);        // inside the discarded input: no answer
    ep.poll();
    CHECK(s.tx.empty());
    g_millis += 250;        // the pause
    CHECK(exchange(ep, s, false, confirm).size() == 5 + 20);
    // half a frame, a pause of 300 ms, the rest
    s.tx.clear();
    s.send({uint8_t(confirm.size()), 0});
    s.send(Bytes(confirm.begin(), confirm.begin() + 5));
    ep.poll();
    g_millis += 300;
    s.send(Bytes(confirm.begin() + 5, confirm.end()));
    ep.poll();
    if (kind == Endpoint::kTcp) CHECK(s.tx.size() == 2 + 5 + 20);   // TCP keeps reading that frame
    else CHECK(s.tx.empty());                                        // vendor bulk restarted: the rest is not a frame
    g_millis += 250;
  }
}

static void testRequestText() {
  const uint8_t ok[] = {'a', 0xC3, 0xA9, 0xE2, 0x82, 0xAC, 0xF0, 0x9F, 0x98, 0x80};
  CHECK(requestText(ok, sizeof ok));
  const uint8_t overlong[] = {0xC0, 0xAF}, surrogate[] = {0xED, 0xA0, 0x80}, cut[] = {'a', 0xE2, 0x82},
                high[] = {0xF4, 0x90, 0x80, 0x80}, del[] = {0x7F}, tab[] = {0x09};
  CHECK(!requestText(overlong, 2) && !requestText(surrogate, 3) && !requestText(cut, 3) && !requestText(high, 4));
  CHECK(!requestText(del, 1) && !requestText(tab, 1) && requestText(nullptr, 0));
}

int main() {
  testConfirmTransportEveryKind();
  testConfirmVectors();
  testDiscoverableAlways();
  testRequestValues();
  testHeaderRefusalsNotKept();
  testLengthPrefixedReader();
  testRequestText();
  printf("TEST done %d/%d\n", checks - failures, checks);
  return failures ? 1 : 0;
}
