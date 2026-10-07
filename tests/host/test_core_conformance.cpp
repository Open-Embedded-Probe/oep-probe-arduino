// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests of fn 0 (the core) against core §1.2, §7.1, §7.5, §7.7 and §12: confirm's transport TLV on every transport kind
// (the index of the describe entry it came on), the ops tag first in every fn's describe and fn 0's describe without the
// removed declarations, list's first only, open's force and owner, a repeated non-repeating TLV (the first used), the
// header refusals (§4.3 order 1) neither kept nor restarting the lease, unknown TLVs (§2.3: no ignored list), the
// resend table keyed on corr alone (§5.2), resource numbers (§9), no resume (§6.2, §9),
// the length-prefixed reader's over-long length and TCP pause rules (transports §1, §2), the ops encoding (core §7.4:
// oep-spec's ops_encoding.json, checked in test_vectors.cpp, and every fn's ops here), and the optional oep.probe.restart
// (oep-if-restart: listed after the sketch's interfaces with restart_max_ms, its refusals, the answer first, nothing
// served or sent after it). The byte vectors of
// oep-spec tests/vectors are test_vectors.cpp's.
#include <stdio.h>
#include <string.h>
#include <string>
#include <utility>
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
// One request: role 0x01 corr fn op session_id (0 = none) payload, the 10-byte header (core §4.1). An open written
// without a session carries its id at the front of its payload here (openPayload): it goes into the header.
static Bytes request(uint16_t corr, uint16_t fn, uint8_t op, const Bytes &payload, bool session = false, uint32_t id = 0) {
  Bytes p = payload;
  if (fn == 0 && op == 0x10 && !session && p.size() >= 4) {
    id = uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
    p.erase(p.begin(), p.begin() + 4);
  } else if (!session) {
    id = 0;
  }
  Bytes m = {0x01, uint8_t(corr), uint8_t(corr >> 8), uint8_t(fn), uint8_t(fn >> 8), op};
  const Bytes s = u32(id);
  m.insert(m.end(), s.begin(), s.end());
  m.insert(m.end(), p.begin(), p.end());
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
static Bytes withTlv(Bytes p, uint8_t tag, const Bytes &value) {   // tag len(u16) value (core §2.2)
  p.push_back(tag);
  p.push_back(static_cast<uint8_t>(value.size()));
  p.push_back(static_cast<uint8_t>(value.size() >> 8));
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
// A describe answer's TLV of `tag` (its value), empty when absent; found: whether it was there.
static Bytes describeTlv(const Bytes &answer, uint8_t tag, bool *found = nullptr) {
  if (found) *found = false;
  for (size_t i = 6; i + 3 <= answer.size();) {
    const size_t len = answer[i + 1] | answer[i + 2] << 8;
    if (answer[i] == tag) {
      if (found) *found = true;
      return Bytes(answer.begin() + i + 3, answer.begin() + i + 3 + len);
    }
    i += 3 + len;
  }
  return {};
}
static bool describedAs(const Bytes &answer, uint8_t index, uint8_t kind) {
  for (size_t i = 6; i + 3 <= answer.size();) {
    const size_t len = answer[i + 1] | answer[i + 2] << 8;
    if (answer[i] == reg::core::kTlvDescribeTransport && len == 3 && answer[i + 3] == index && answer[i + 4] == kind)
      return true;
    i += 3 + len;
  }
  return false;
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
    CHECK(r.size() == 5 + 21 && r[3] == kResolutionCompleted);
    if (r.size() == 5 + 21)
      CHECK(r[5 + 17] == reg::core::kTlvConfirmAnswerTransport && r[5 + 18] == 1 && r[5 + 19] == 0 && r[5 + 20] == c.index);
    // that index names this transport's entry in the describe returned on the same connection
    const Bytes d = exchange(*c.ep, c.port->s, c.port->serial, request(corr++, 0, 0x03, {0, 0, 0, 0}));
    CHECK(d.size() > 6 && d[3] == kResolutionCompleted && describedAs(d, c.index, c.port->kind));
  }
}

static uint8_t reason(const Bytes &r) { return r.size() >= 5 && r[3] == kResolutionRejected ? r[4] : 0xff; }

// core §1.2, §7.4: every fn's describe starts with ops (base + bitmap, base the lowest op, the last byte non-zero) -
// fn 0's the required ops (confirm, list, describe, clock, open, end, keepalive, lock_state: no plan, restart or
// subscription), an interface's what offers() says; an op not set is unknown_operation. An interface with no op is not
// added (its ops would have no encoding).
class Toy final : public Interface {
 public:
  const char *name() const override { return "io.github.test.toy"; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }
  bool offers(uint8_t op) const override { return op == 0x02 || op == 0x05 || op == 0x11; }
  bool lockFree(uint8_t) const override { return true; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return completed(); }
};
class Silent final : public Interface {   // offers no op
 public:
  const char *name() const override { return "io.github.test.silent"; }
  uint16_t instance() const override { return 0; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return completed(); }
};
static void testOps() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  Toy toy;
  Link link(ep);
  ep.add(toy);
  ep.add(link);
  Bytes r = exchange(ep, s, false, request(1, 0, 0x03, {0, 0, 0, 0}));
  CHECK(r.size() > 6 && r[6] == kTagOps && Bytes(r.begin() + 6, r.begin() + 6 + 7) == hex("090400010f8007"));
  r = exchange(ep, s, false, request(2, 0, 0x03, {1, 0, 0, 0}));   // base 2: ops 2, 5, 0x11 (bits 0, 3, 15)
  CHECK(r.size() == 6 + 3 + 3 && Bytes(r.begin() + 6, r.end()) == hex("090300020980"));
  r = exchange(ep, s, false, request(3, 0, 0x03, {2, 0, 0, 0}));   // oep.probe.link without port_speed: source and sink
  CHECK(r.size() == 6 + 3 + 2 && Bytes(r.begin() + 6, r.end()) == hex("0902000103"));
  CHECK(reason(exchange(ep, s, false, request(4, 1, 0x03, {}))) == kRejectUnknownOperation);
  CHECK(exchange(ep, s, false, request(5, 1, 0x05, {}))[3] == kResolutionCompleted);
  CHECK(reason(exchange(ep, s, false, request(6, 2, 0x03, {}))) == kRejectUnknownOperation);   // port_speed: not in ops
  ep.setPortSpeed([](uint8_t, uint32_t b, bool) { return b; }, 115200);
  r = exchange(ep, s, false, request(7, 0, 0x03, {2, 0, 0, 0}));
  CHECK(r.size() == 6 + 3 + 2 && Bytes(r.begin() + 6, r.end()) == hex("0902000107"));
  // oep-if-link §2: source answers at most max_frame - 7 bytes; the largest sink that fits is max_frame - 12 (the
  // request's header 10 and count 2): a request of max_frame bytes, completed
  r = exchange(ep, s, false, request(20, 2, reg::probe_link::kOpSource, u32(5000)));
  CHECK(r.size() == 1024 && r[3] == kResolutionCompleted && r[5] == uint8_t(1024 - 7) && r[6] == uint8_t((1024 - 7) >> 8));
  Bytes sink = {uint8_t(1024 - 12), uint8_t((1024 - 12) >> 8)};
  sink.resize(2 + 1024 - 12, 0x5a);
  CHECK(request(21, 2, reg::probe_link::kOpSink, sink).size() == 1024);
  r = exchange(ep, s, false, request(21, 2, reg::probe_link::kOpSink, sink));
  CHECK(r.size() == 5 && r[3] == kResolutionCompleted);
  CHECK(reason(exchange(ep, s, false, request(8, 3, 0x03, {0, 0, 0, 0}))) == kRejectUnknownFunction);   // no plan, no restart
  for (uint8_t op : {0x05, 0x14, 0x30, 0x32})   // fn 0: the old plan_release / restart, subscribe / unsubscribe
    CHECK(reason(exchange(ep, s, false, request(9, 0, op, {}, true, 7))) == kRejectUnknownOperation);
  Silent none;
  CHECK(!ep.add(none));
}

// core §2.3: an unknown non-critical TLV is ignored and nothing says so (no ignored list, tags 0x00 and 0x7F included);
// an unknown critical one is refused unsupported with its tag as received; an implemented TLV with another length than
// its definition is malformed, critical or not.
static void testUnknownTlvs() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  Bytes p;
  for (uint8_t t : {0x20, 0x21, 0x7f, 0x00}) p = withTlv(p, t, {1, 2});
  Bytes r = exchange(ep, s, false, request(1, 0, 0x13, p));   // lock_state with unknown non-critical TLVs
  CHECK(r.size() == 5 + 5 && r[3] == kResolutionCompleted);   // locked(u8) remaining(u32), nothing after
  r = exchange(ep, s, false, request(2, 0, 0x13, withTlv(p, 0xa1, {})));
  CHECK(reason(r) == kRejectUnsupported && r.size() == 6 && r[5] == 0xa1);
  r = exchange(ep, s, false, request(3, 0, 0x13, withTlv({}, 0xff, {})));   // 0xFF: tag 0x7F critical, unknown
  CHECK(reason(r) == kRejectUnsupported && r.size() == 6 && r[5] == 0xff);
  // open's owner (implemented): another length than 1..32 is malformed with bit 7 or without
  CHECK(reason(exchange(ep, s, false, request(4, 0, 0x10, withTlv(openReq(9, 3000), 0x01, {})))) == kRejectMalformed);
  CHECK(reason(exchange(ep, s, false, request(5, 0, 0x10, withTlv(openReq(9, 3000), 0x81, Bytes(33, 'a'))))) == kRejectMalformed);
  CHECK(!ep.locked());
}

// core §5.2: the resend table keeps (corr, answer) - the same corr is a resend whatever it carries (no corr_reused).
static void testResendByCorr() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  CHECK(exchange(ep, s, false, request(1, 0, 0x10, openReq(7, 5000)))[3] == kResolutionCompleted);
  const Bytes first = exchange(ep, s, false, request(2, 0, 0x12, {}, true, 7));   // keepalive
  CHECK(first.size() == 5 && first[3] == kResolutionCompleted);
  // corr 2 again with another op (end): the kept answer, nothing run - the lock stays
  CHECK(exchange(ep, s, false, request(2, 0, 0x11, {}, true, 7)) == first && ep.locked());
  // an older corr not in the table: result_lost
  CHECK(reason(exchange(ep, s, false, request(1, 0, 0x12, {}, true, 7))) == kRejectResultLost);
  CHECK(exchange(ep, s, false, request(3, 0, 0x11, {}, true, 7))[3] == kResolutionCompleted && !ep.locked());
}

// core §6.2, §6.4, §9: no resume - after end the id is no_session; a lapse and force release alike; a resent open of
// the holder restarts the lease; the owner comes from the open that takes the lock.
static void testNoResume() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  CHECK(exchange(ep, s, false, request(1, 0, 0x10, withTlv(openReq(7, 1000), 0x01, {'a'})))[3] == kResolutionCompleted);
  CHECK(exchange(ep, s, false, request(2, 0, 0x11, {}, true, 7))[3] == kResolutionCompleted);   // end
  CHECK(!ep.locked());
  CHECK(reason(exchange(ep, s, false, request(3, 0, 0x12, {}, true, 7))) == kRejectNoSession);
  CHECK(reason(exchange(ep, s, false, request(4, 0, 0x12, {}, true, 9))) == kRejectNoSession);   // any id
  CHECK(reason(exchange(ep, s, false, request(5, 0, 0x13, {}, true, 9))) == kRejectNoSession);   // lock-free with an id
  CHECK(exchange(ep, s, false, request(6, 0, 0x13, {}))[3] == kResolutionCompleted);              // with 0: no check
  // a lapse: the same
  CHECK(exchange(ep, s, false, request(7, 0, 0x10, openReq(7, 1000)))[3] == kResolutionCompleted);
  g_millis += 1001;
  CHECK(reason(exchange(ep, s, false, request(8, 0, 0x12, {}, true, 7))) == kRejectNoSession);
  // an open of the holder again: the lease restarts, the owner stays the first open's
  CHECK(exchange(ep, s, false, request(9, 0, 0x10, withTlv(openReq(7, 1000), 0x01, {'x'})))[3] == kResolutionCompleted);
  g_millis += 800;
  CHECK(exchange(ep, s, false, request(10, 0, 0x10, withTlv(openReq(7, 1000), 0x01, {'y'})))[3] == kResolutionCompleted);
  g_millis += 800;
  Bytes r = exchange(ep, s, false, request(11, 0, 0x13, {}));
  CHECK(r.size() == 5 + 5 + 4 && r[5] == 1 && Bytes(r.begin() + 10, r.end()) == hex("01010078"));   // owner "x"
  // force: the old holder's next request is locked, and the new owner shows
  CHECK(exchange(ep, s, false, request(12, 0, 0x10, withTlv(openReq(8, 1000, 1), 0x01, {'z'})))[3] == kResolutionCompleted);
  r = exchange(ep, s, false, request(13, 0, 0x12, {}, true, 7));
  CHECK(reason(r) == kRejectLocked && r.size() == 5 + 4 + 4 && Bytes(r.begin() + 9, r.end()) == hex("0101007a"));
}

// core §7.5: fn 0's describe carries no discoverable, reserved, profile, resets_on_open or implementation (removed
// from v1); the transports and max_op_ms are the endpoint's. core §7.2: list is first(u16) only; total is the
// interfaces' count, a first past it answers count 0.
static void testDescribeAndList() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  Toy toy;
  ep.add(toy);
  const Bytes d = exchange(ep, s, false, request(1, 0, 0x03, {0, 0, 0, 0}));
  bool found = false;
  for (uint8_t gone : {0x44, 0x45, 0x47, 0x4A, 0x07}) {
    describeTlv(d, gone, &found);
    CHECK(!found);
  }
  describeTlv(d, reg::core::kTlvDescribeMaxOpMs, &found);
  CHECK(found);
  Bytes r = exchange(ep, s, false, request(2, 0, 0x02, {0, 0}));
  CHECK(r.size() == 5 + 3 + 7 + strlen(toy.name()) && getU16(&r[5]) == 1 && r[7] == 1);
  r = exchange(ep, s, false, request(3, 0, 0x02, {1, 0}));
  CHECK(r.size() == 5 + 3 && getU16(&r[5]) == 1 && r[7] == 0);
  CHECK(reason(exchange(ep, s, false, request(4, 0, 0x02, {0}))) == kRejectMalformed);
  r = exchange(ep, s, false, request(5, 0, 0x02, withTlv({0, 0}, 0x90, {})));
  CHECK(reason(r) == kRejectUnsupported && r.size() == 6 && r[5] == 0x90);
}

// core §6.4 / §2.1: open's force reads any non-zero as true; owner is text of 1 to 32 bytes (other text is taken as
// sent); core §2.3: a repeated non-repeating TLV - the first one is used.
static void testRequestValues() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  CHECK(reason(exchange(ep, s, false, request(1, 0, 0x10, withTlv(openReq(9, 3000), 0x01, Bytes(33, 'a'))))) == kRejectMalformed);
  const Bytes twice = withTlv(withTlv(openReq(9, 3000), 0x01, {'a', 0x0a}), 0x01, {'b'});
  Bytes r = exchange(ep, s, false, request(2, 0, 0x10, twice));
  CHECK(r.size() == 5 + 8 && r[3] == kResolutionCompleted && ep.locked());   // lease_ms boot_id (core §6.4)
  r = exchange(ep, s, false, request(3, 0, 0x13, {}));   // lock_state shows the first owner, as sent
  const Bytes owner = {0x01, 2, 0, 'a', 0x0a};
  CHECK(r.size() == 5 + 5 + owner.size() && Bytes(r.begin() + 10, r.end()) == owner);
  // force 2 is true: another session takes the lock
  r = exchange(ep, s, false, request(4, 0, 0x10, openReq(8, 3000, 2)));
  CHECK(r[3] == kResolutionCompleted && ep.locked());
  CHECK(reason(exchange(ep, s, false, request(5, 0, 0x12, {}, true, 9))) == kRejectLocked);
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
  CHECK(reason(exchange(ep, s, false, request(4, 0, 0x05, {}, true, 7))) == kRejectUnknownOperation);   // plan_release: not fn 0's
  CHECK(reason(exchange(ep, s, false, request(5, 0, 0x14, {}, true, 7))) == kRejectUnknownOperation);   // restart: not fn 0's
  CHECK(reason(exchange(ep, s, false, request(6, 0, 0x12, {}))) == kRejectSessionRequired);              // session_id 0
  g_millis += 200;   // 1100 ms after the open: the refusals did not extend it
  ep.poll();
  CHECK(!ep.locked());
  // the lapsed session's next request: no_session (no resume), and corr 2 was not taken by the refusal
  CHECK(reason(exchange(ep, s, false, request(2, 0, 0x12, {}, true, 7))) == kRejectNoSession);
}

// transports §1: a length over max_frame - that frame and the input up to the next pause of probe_frame_gap_ms are
// discarded, unanswered; the next frame after the pause is read. transports §2: a pause inside a frame restarts the read
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
    CHECK(exchange(ep, s, false, confirm).size() == 5 + 21);
    // half a frame, a pause of 300 ms, the rest
    s.tx.clear();
    s.send({uint8_t(confirm.size()), 0});
    s.send(Bytes(confirm.begin(), confirm.begin() + 5));
    ep.poll();
    g_millis += 300;
    s.send(Bytes(confirm.begin() + 5, confirm.end()));
    ep.poll();
    if (kind == Endpoint::kTcp) CHECK(s.tx.size() == 2 + 5 + 21);   // TCP keeps reading that frame
    else CHECK(s.tx.empty());                                        // vendor bulk restarted: the rest is not a frame
    g_millis += 250;
  }
}

// core §5.2: the resend table remembers at least max_inflight requests - confirm never offers more than it has entries.
static void testInflightWithinTable() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 64}, Endpoint::kVendorBulk, 0);
  const Bytes r = exchange(ep, s, false, request(1, 0, 0x01, confirmReq()));
  CHECK(r.size() == 5 + 21 && r[5 + 12] >= 1 && r[5 + 12] <= 8);
}

// core §9: each new resource number is the previous plus 1 (65535 then 1), a number in use skipped.
static void testResourceNumbers() {
  using RN = ResourceNumbers;
  RN::reset();
  const uint16_t a = RN::take(RN::kConnection), b = RN::take(RN::kStream);
  CHECK(a == 1 && b == 2);
  RN::close(a);
  const uint16_t c = RN::take(RN::kStream);
  CHECK(c == 3);   // not a, though it is free
  for (uint32_t i = 0; i < 0xFFFF - 3; ++i) RN::close(RN::take(RN::kStream));   // round to 65535
  CHECK(RN::take(RN::kStream) == 1);   // 1 free again: taken; 2 and 3 live: skipped
  CHECK(RN::take(RN::kStream) == 4);
  RN::reset();
}

// core §7.2: interfaces with the same (name, revision) are numbered from 0 in ascending fn, whatever instance() the
// interface was built with.
class Named final : public Interface {
 public:
  explicit Named(const char *name, uint8_t revision = 1) : name_(name), revision_(revision) {}
  const char *name() const override { return name_; }
  uint16_t instance() const override { return 7; }   // wrong on purpose: the endpoint counts
  uint8_t revision() const override { return revision_; }
  bool offers(uint8_t op) const override { return op == 1; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return completed(); }
 private:
  const char *name_;
  uint8_t revision_;
};
static void testInstanceNumbering() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  Named a("io.github.test.x"), b("io.github.test.y"), c("io.github.test.x"), d("io.github.test.x", 2);
  ep.add(a);
  ep.add(b);
  ep.add(c);
  ep.add(d);
  const Bytes r = exchange(ep, s, false, request(1, 0, 0x02, {0, 0}));
  // total(u16) count(u8), then fn(u16) instance(u16) revision flags name_len name (no element length): fn 1 to 4 (fn 0,
  // the core, is never an entry)
  std::vector<uint16_t> fns, instances;
  for (size_t at = 5 + 3; at + 7 <= r.size() && instances.size() < 5; at += 7 + r[at + 6]) {
    fns.push_back(getU16(&r[at]));
    instances.push_back(getU16(&r[at + 2]));
  }
  CHECK(r.size() >= 8 && getU16(&r[5]) == 4 && fns == std::vector<uint16_t>({1, 2, 3, 4}));
  CHECK(instances == std::vector<uint16_t>({0, 0, 1, 0}));
  CHECK(ep.instanceOf(3) == 1 && ep.instanceOf(4) == 0);
}

// core §13 rule 1 / §7.5: an interface name outside the form is not added; describeCore refuses a unit_id outside
// a-z 0-9 - (model is free text).
static void testNamesAndTokens() {
  MemStream s;
  static uint8_t rx[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  Named bad1("single"), bad2("io.Github.x"), bad3("io..x"), bad4("io.-x"), bad5("io.x-"), good("io.github.a-b.c9");
  CHECK(!ep.add(bad1) && !ep.add(bad2) && !ep.add(bad3) && !ep.add(bad4) && !ep.add(bad5));
  CHECK(ep.add(good));
  // core §7.2 / §13 rule 1: 1 to 48 bytes (a list answer with one entry fits the smallest max_frame)
  static const std::string n48 = "io.github." + std::string(38, 'a'), n49 = n48 + "b";
  Named long48(n48.c_str()), long49(n49.c_str());
  CHECK(n48.size() == 48 && ep.add(long48));
  CHECK(!ep.add(long49));
  uint8_t buf[200];
  const uint8_t id[] = {'a', '1'}, bad_id[] = {'A', '1'};
  TlvWriter w1(buf, sizeof buf);
  CHECK(describeCore(w1, "my-probe", id, sizeof id, 0));
  TlvWriter w2(buf, sizeof buf);
  CHECK(describeCore(w2, "My Probe 2", id, sizeof id, 0));
  TlvWriter w3(buf, sizeof buf);
  CHECK(!describeCore(w3, "my-probe", bad_id, sizeof bad_id, 0));
}

// oep.probe.restart (oep-if-restart): listed only with a handler set before the first poll, after the sketch's
// interfaces and oep.probe.plan, with restart (0x01) in its ops and restart_max_ms (0x40) in its describe; a handler
// set after the first poll changes nothing (the list is fixed for the boot). restart needs the lock - session_required,
// no_session, locked, the handler not called; a critical TLV is unsupported (no restart), another is ignored. The
// answer goes first: the session's notifications end (an event queued is not sent), it is written, then everything is
// let go of - the session (sessionOver), what the settings keep (probeRestart), every plan, the settings' too - and the
// handler is called kRestartSettleMs after it; the handler's own detach (kRestartDetachMs) and reset (kRestartResetMs)
// start the reset within about 100 ms of the answer. Nothing after the answer is served or sent, on any transport: the
// request behind it in the same read, a request on another transport.
class PlanToy final : public Interface {
 public:
  explicit PlanToy(const char *name, bool talks = false) : name_(name), talks_(talks) {}
  const char *name() const override { return name_; }
  bool notifies() const override { return talks_; }
  bool subscribe(bool on) override { subscribed = on; return true; }
  uint16_t instance() const override { return 0; }
  bool offers(uint8_t op) const override { return op == 1; }   // one op: an interface offers at least one (core §7.4)
  uint8_t revision() const override { return 1; }
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *, size_t) override { return 0; }
  bool planApply(const RoleAssignment *, size_t) override { planned = true; return true; }
  void planRelease() override { planned = false; ++releases; }
  void sessionOver() override { ++session_overs; }
  void probeRestart() override { ++probe_restarts; restart_saw_planned = planned; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return completed(); }
  bool planned = false, restart_saw_planned = false, subscribed = false;
  int releases = 0, session_overs = 0, probe_restarts = 0;

 private:
  const char *name_;
  bool talks_;
};
struct RestartSeen {
  int calls = 0;
  size_t tx = 0;             // what the transport had been given when the handler ran
  uint32_t at_ms = 0;
  bool locked = true, planned = true;
  MemStream *stream = nullptr;
  Endpoint *ep = nullptr;
  PlanToy *a = nullptr, *b = nullptr;
};
static RestartSeen g_seen;
static void testRestart() {
  auto describe = [](Endpoint &ep, MemStream &s, uint16_t corr, uint16_t fn) {
    return exchange(ep, s, false, request(corr, 0, 0x03, {uint8_t(fn), uint8_t(fn >> 8), 0, 0}));
  };
  auto restartMaxMs = [](const Bytes &d) {
    bool found = false;
    const Bytes v = describeTlv(d, reg::probe_restart::kTlvDescribeRestartMaxMs, &found);
    return found && v.size() == 4 ? getU32(v.data()) : 0u;
  };
  auto hook = []() {
    ++g_seen.calls;
    g_seen.tx = g_seen.stream->tx.size();
    g_seen.at_ms = g_millis;
    g_seen.locked = g_seen.ep->locked();
    RoleAssignment roles[4];
    g_seen.planned = g_seen.ep->plan(roles, 4) != 0 || g_seen.a->planned || g_seen.b->planned;
  };
  {   // no handler: not listed - fn 3 is oep.probe.plan, fn 4 unknown; a handler after the first poll changes nothing
    MemStream s;
    static uint8_t rx[1200], tx[1100];
    Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
    PlanToy a("io.github.test.a"), b("io.github.test.b");
    ep.add(a);
    ep.add(b);
    CHECK(ep.planFn() == 3 && ep.restartFn() == 0);
    const Bytes l = exchange(ep, s, false, request(1, 0, 0x02, {0, 0}));
    CHECK(l.size() >= 8 && getU16(&l[5]) == 3);
    CHECK(reason(describe(ep, s, 2, 4)) == kRejectUnknownFunction);
    CHECK(reason(exchange(ep, s, false, request(3, 4, kOpRestart, {}, true, 7))) == kRejectUnknownFunction);
    ep.setRestart(hook, 1500);
    CHECK(ep.restartFn() == 0 && reason(describe(ep, s, 4, 4)) == kRejectUnknownFunction);
  }
  {   // a handler with 50 ms: restart_max_ms 50 as given; its ops restart only; no plan roles
    MemStream s;
    static uint8_t rx[1200], tx[1100];
    Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
    Named x("io.github.test.x");
    ep.add(x);
    ep.setRestart(hook, 50);
    CHECK(ep.planFn() == 0 && ep.restartFn() == 2);
    const Bytes d = describe(ep, s, 1, 2);
    CHECK(restartMaxMs(d) == 50);
    CHECK(Bytes(d.begin() + 6, d.end()) == hex("090200010140040032000000"));   // ops {restart}, restart_max_ms 50
    const Bytes l = exchange(ep, s, false, request(2, 0, 0x02, {1, 0}));   // from the second: oep.probe.restart
    CHECK(l.size() == 5 + 3 + 7 + 17 && getU16(&l[5]) == 2 && getU16(&l[8]) == 2 && getU16(&l[10]) == 0);   // fn 2, instance 0
  }
  MemStream s, u;
  static uint8_t rx[1200], rx2[1200], tx[1100];
  Endpoint ep(s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0);
  CHECK(ep.addTransport(u, rx2, sizeof rx2, Endpoint::kUartBridge));
  PlanToy a("io.github.test.a"), b("io.github.test.b", true);
  ep.add(a);
  ep.add(b);
  g_seen = RestartSeen{};
  g_seen.stream = &s;
  g_seen.ep = &ep;
  g_seen.a = &a;
  g_seen.b = &b;
  ep.setRestart(hook, 1500);
  const uint16_t pf = ep.planFn(), rf = ep.restartFn();
  CHECK(pf == 3 && rf == 4);
  CHECK(restartMaxMs(describe(ep, s, 4, rf)) == 1500);
  CHECK(pf == ep.planFn() && rf == ep.restartFn());   // the same after the first poll
  // the lock (core §6.3, §4.3): session_required, no_session, locked - nothing restarts
  CHECK(reason(exchange(ep, s, false, request(5, rf, kOpRestart, {}))) == kRejectSessionRequired);
  CHECK(reason(exchange(ep, s, false, request(6, rf, kOpRestart, {}, true, 7))) == kRejectNoSession);
  CHECK(exchange(ep, s, false, request(7, 0, 0x10, openReq(7, 5000)))[3] == kResolutionCompleted);
  CHECK(reason(exchange(ep, s, false, request(8, rf, kOpRestart, {}, true, 9))) == kRejectLocked);
  // a critical TLV restart does not know: unsupported with its tag (core §2.3)
  Bytes r = exchange(ep, s, false, request(9, rf, kOpRestart, withTlv({}, 0xa0, {1}), true, 7));
  CHECK(reason(r) == kRejectUnsupported && r.size() == 6 && r[5] == 0xa0);
  CHECK(g_seen.calls == 0 && ep.locked() && !ep.restarting());
  // a session's plan (a), the settings' plan (b), and b's notifications subscribed
  CHECK(exchange(ep, s, false, request(10, pf, kOpPlanApply, withTlv({}, 0x90, {1, 0, 0, 3, 0}), true, 7))[3] == kResolutionCompleted);
  const RoleAssignment settings[] = {{2, 0, 4}};
  const uint16_t fn_b = 2;
  CHECK(ep.replacePlan(settings, 1, &fn_b, 1) == 0 && a.planned && b.planned);
  CHECK(exchange(ep, s, false, request(11, 2, kOpSubscribe, {0, 0, 10, 0, 0, 0}, true, 7))[3] == kResolutionCompleted);
  CHECK(b.subscribed);
  // restart with an unknown TLV (ignored), and a lock_state right behind it in the same read; an event of b queued
  s.tx.clear();
  const Bytes m1 = request(12, rf, kOpRestart, withTlv({}, 0x20, {}), true, 7), m2 = request(13, 0, kOpLockState, {});
  for (const Bytes *m : {&m1, &m2}) { s.send({uint8_t(m->size()), uint8_t(m->size() >> 8)}); s.send(*m); }
  const uint8_t ev[2] = {1, 2};
  CHECK(ep.event(b, 0x01, ev, sizeof ev));
  const uint32_t before = g_millis;
  ep.poll();
  // one answer: completed success, no payload (core §2.3: nothing says the TLV was ignored); no event
  CHECK(s.tx == hex("0500020c000100"));
  CHECK(g_seen.calls == 1 && g_seen.tx == s.tx.size() && ep.restarting());
  CHECK(g_seen.at_ms - before >= Endpoint::kRestartSettleMs && g_seen.at_ms - before < 100);
  // the handler's own part - a USB device off the bus kRestartDetachMs, then the reset (kRestartResetMs to start) - still
  // starts the reset within about 100 ms of the answer (a reset with the device on the bus left the host failing its
  // device descriptor request after the restart)
  CHECK(g_seen.at_ms - before + kRestartDetachMs + kRestartResetMs < 100);
  CHECK(kRestartDetachMs >= 50);   // long enough for the host to see the device gone
  CHECK(!g_seen.locked && !g_seen.planned && !b.subscribed);   // let go of before the handler
  CHECK(a.session_overs == 1 && b.session_overs == 1 && a.probe_restarts == 1 && b.probe_restarts == 1);
  CHECK(b.restart_saw_planned && !a.restart_saw_planned);   // the session's plan went with the session, the settings' after probeRestart
  CHECK(a.releases == 1 && b.releases == 1);
  // nothing more: a request on either transport
  s.tx.clear();
  g_millis += 1000;
  CHECK(exchange(ep, s, false, request(14, 0, 0x01, confirmReq())).empty() && s.tx.empty());
  CHECK(exchange(ep, u, true, request(15, 0, 0x01, confirmReq())).empty() && u.tx.empty());
  CHECK(g_seen.calls == 1);
}

int main() {
  testConfirmTransportEveryKind();
  testOps();
  testUnknownTlvs();
  testResendByCorr();
  testNoResume();
  testDescribeAndList();
  testRequestValues();
  testHeaderRefusalsNotKept();
  testLengthPrefixedReader();
  testInflightWithinTable();
  testResourceNumbers();
  testInstanceNumbering();
  testNamesAndTokens();
  testRestart();
  printf("TEST done %d/%d\n", checks - failures, checks);
  return failures ? 1 : 0;
}
