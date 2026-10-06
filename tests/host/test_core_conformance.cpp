// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests of fn 0 (oep.core) against core §7.1 and §7.5: confirm's transport TLV on every transport kind (the index
// of the describe entry it came on), the confirm vectors of oep-spec tests/vectors/confirm.json, describe's
// discoverable always sent.
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

int main() {
  testConfirmTransportEveryKind();
  testConfirmVectors();
  testDiscoverableAlways();
  printf("TEST done %d/%d\n", checks - failures, checks);
  return failures ? 1 : 0;
}
