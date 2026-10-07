// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the serial-port reader (oep-transports §1, §4), the endpoint's serial-port rules (raw bytes, the ports a
// session holds, the resume from its last host reset), owner and the transport list, the binds' modes, the TLV long
// form, the session table (expired, resumed 2), subscriptions, the resource numbers, the stream shapes.
#include <algorithm>
#include <stdio.h>
#include <string>
#include <vector>

#include "OepFrame.h"
#include "OepBind.h"
#include "OepEndpoint.h"
#include "OepCaptureGroup.h"
#include "OepStream.h"

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
  Bytes rx, tx;   // host -> probe, probe -> host
  size_t at = 0;
  int room = 4096;
  int available() override { return static_cast<int>(rx.size() - at); }
  int read() override { return at < rx.size() ? rx[at++] : -1; }
  int peek() override { return at < rx.size() ? rx[at] : -1; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    const size_t k = room < 0 ? 0 : (n < static_cast<size_t>(room) ? n : static_cast<size_t>(room));
    tx.insert(tx.end(), b, b + k);
    return k;
  }
  int availableForWrite() override { return room; }
  void send(const Bytes &b) { rx.insert(rx.end(), b.begin(), b.end()); }
};

// The host side of the framing: 0x00 <COBS(message + CRC)> 0x00, and the frames found in what came back.
static Bytes frame(const Bytes &message) {
  MemStream s;
  writeCobsFrame(s, message.data(), message.size());
  return s.tx;
}
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
// Splits what the probe sent into frames (results) and the rest (raw bytes).
static void split(const Bytes &tx, std::vector<Bytes> &frames, Bytes &raw) {
  frames.clear();
  raw.clear();
  size_t i = 0;
  while (i < tx.size()) {
    if (tx[i] != 0) { raw.push_back(tx[i++]); continue; }
    size_t j = i + 1;
    while (j < tx.size() && tx[j] != 0) ++j;
    Bytes body(tx.begin() + i + 1, tx.begin() + j), msg;
    if (j < tx.size() && !body.empty() && unframe(body, msg)) { frames.push_back(msg); i = j + 1; continue; }
    if (body.empty()) { i = j; continue; }   // 0x00 0x00
    raw.push_back(0);
    raw.insert(raw.end(), body.begin(), body.end());
    i = j;
  }
}

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
static Bytes openPayload(uint32_t id, uint32_t lease, const char *owner = nullptr) {
  Bytes p = u32(id);
  const Bytes l = u32(lease);
  p.insert(p.end(), l.begin(), l.end());
  p.push_back(0);
  if (owner) { p.push_back(0x01); p.push_back(static_cast<uint8_t>(strlen(owner))); p.push_back(0); p.insert(p.end(), owner, owner + strlen(owner)); }
  return p;
}

// A bindable stream (a console): bytes the target says, what the port sent it.
class FakeSource final : public BindSource {
 public:
  uint8_t buffer[256];
  PositionStream::Mark marks[4];
  PositionStream stream{buffer, sizeof buffer, marks, 4};
  Bytes input;
  void say(const char *text) { for (const char *p = text; *p; ++p) stream.put(static_cast<uint8_t>(*p)); }
  const PositionStream *bindStream() const override { return &stream; }
  uint16_t bindStreamNumber() const override { return 1; }
  size_t bindInput(const uint8_t *d, size_t n) override { input.insert(input.end(), d, d + n); return n; }
};

static std::string text(const Bytes &b) { return std::string(b.begin(), b.end()); }

// ---- SerialReader -------------------------------------------------------------------------------------------------------

static Bytes g_raw;
static void sink(void *, const uint8_t *d, size_t n) { g_raw.insert(g_raw.end(), d, d + n); }

static void testReader() {
  uint8_t enc[64], dec[64];
  SerialReader r;
  r.reset(enc, sizeof enc, dec, sizeof dec);
  g_raw.clear();
  Bytes in = {'h', 'i'};
  const Bytes f = frame({1, 2, 3});
  in.insert(in.end(), f.begin(), f.end());
  in.insert(in.end(), {0, 'z', 'z', 0});   // a broken candidate: raw with its leading 0x00
  const uint8_t *p = in.data();
  size_t n = in.size();
  CHECK(r.feed(p, n, sink, nullptr));
  CHECK(r.length() == 3 && r.message()[0] == 1 && r.message()[2] == 3);
  CHECK(text(g_raw) == "hi");
  CHECK(!r.feed(p, n, sink, nullptr));
  // "0x00 z z": the frame's closing 0x00 started a candidate, then 0x00 0x00 (empty) and "zz" closed by a 0x00
  CHECK(g_raw.size() == 2 + 3 && g_raw[2] == 0 && g_raw[3] == 'z' && g_raw[4] == 'z');
  g_raw.clear();
  const Bytes open = {0, 'a', 'b'};
  p = open.data();
  n = open.size();
  r.feed(p, n, sink, nullptr);
  CHECK(g_raw.empty());
  g_millis += 250;   // stopped for 200 ms: raw
  r.idle(sink, nullptr);
  CHECK(g_raw.size() == 3 && g_raw[0] == 0 && g_raw[1] == 'a');
  // a frame's closing 0x00 opens a candidate; the 200 ms gap after it gives nothing raw (it was a delimiter)
  g_raw.clear();
  const Bytes lone = frame({9, 9, 9});
  p = lone.data();
  n = lone.size();
  CHECK(r.feed(p, n, sink, nullptr));
  CHECK(!r.feed(p, n, sink, nullptr));
  g_millis += 250;
  r.idle(sink, nullptr);
  CHECK(g_raw.empty());
  // a full block at the end: no empty block after it, both forms taken
  Bytes big(254, 7);
  const Bytes fb = frame(big);
  CHECK(fb.size() == 1 + 1 + 254 + 1 + 2 + 1 || fb.size() == big.size() + 2 + 2 + 2);   // 0 FF 254 03 crc crc 0
  uint8_t enc2[400], dec2[400];
  SerialReader r2;
  r2.reset(enc2, sizeof enc2, dec2, sizeof dec2);
  p = fb.data();
  n = fb.size();
  CHECK(r2.feed(p, n, sink, nullptr) && r2.length() == 254);
}

// ---- the endpoint on a serial port, with binds ---------------------------------------------------------------------------

static void testEndpointSerialPort() {
  MemStream usj, bulk;
  static uint8_t rx[1100], rx2[1100], tx[1100];
  Endpoint ep(bulk, rx2, sizeof rx2, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kVendorBulk, 1);
  ep.addTransport(usj, rx, sizeof rx, Endpoint::kUsbSerialJtag);
  Binds binds;
  ep.setRawPorts(&binds);
  FakeSource console;
  Binds::Spec spec;
  spec.set = true;
  spec.source = {Binds::kSlotConsole, 0, &console};
  binds.set(1, spec);

  console.say("boot\n");
  ep.poll();
  std::vector<Bytes> frames;
  Bytes raw;
  split(usj.tx, frames, raw);
  CHECK(text(raw) == "boot\n" && frames.empty());
  usj.tx.clear();

  usj.send({'t', 'y', 'p', 'e', 'd'});   // raw from the host: to the console's input
  ep.poll();
  CHECK(text(console.input) == "typed");

  usj.send(frame(request(1, 0, 0x10, openPayload(0x51, 3000, "test owner"))));   // open over the serial port
  ep.poll();
  split(usj.tx, frames, raw);
  CHECK(frames.size() == 1 && frames[0][3] == 1 && frames[0][4] == 0);
  CHECK(ep.held(1));
  usj.tx.clear();
  console.say("held\n");
  usj.send({'x'});
  ep.poll();
  CHECK(usj.tx.empty() && text(console.input) == "typed");   // held: nothing out, the raw byte dropped

  bulk.send({6, 0});   // lock_state over vendor bulk (length framing): the owner shows
  const Bytes ls = request(2, 0, 0x13, {});
  bulk.send(ls);
  bulk.rx[bulk.rx.size() - ls.size() - 2] = static_cast<uint8_t>(ls.size());
  ep.poll();
  const std::string lsr = text(bulk.tx);
  CHECK(lsr.find("test owner") != std::string::npos);

  console.say("later\n");
  usj.send(frame(request(3, 0, 0x11, {}, true, 0x51)));   // end
  ep.poll();
  split(usj.tx, frames, raw);
  CHECK(frames.size() == 1 && !ep.held(1));
  CHECK(text(raw) == "held\nlater\n");   // where it stopped (probe.config §1.2)

  // the transport list in oep.core's describe
  usj.tx.clear();
  usj.send(frame(request(4, 0, 0x03, {0, 0, 0, 0})));
  ep.poll();
  split(usj.tx, frames, raw);
  CHECK(frames.size() == 1);
  const Bytes &d = frames[0];
  bool vendor = false, usjSeen = false;
  for (size_t i = 6; i + 6 <= d.size(); ++i)
    if (d[i] == 0x49 && d[i + 1] == 3 && d[i + 2] == 0) { vendor |= d[i + 3] == 0 && d[i + 4] == 4 && d[i + 5] == 1; usjSeen |= d[i + 3] == 1 && d[i + 4] == 3; }
  CHECK(vendor && usjSeen);

  // the stream overflowing during a session: the port goes on from the oldest byte left (probe.config §1.2)
  usj.tx.clear();
  usj.send(frame(request(5, 0, 0x10, openPayload(0x52, 3000))));
  ep.poll();
  CHECK(ep.held(1));
  for (int i = 0; i < 40; ++i) console.say("0123456789");   // 400 bytes into a 256-byte stream
  usj.tx.clear();
  usj.send(frame(request(6, 0, 0x11, {}, true, 0x52)));   // end
  ep.poll();
  split(usj.tx, frames, raw);
  CHECK(!ep.held(1) && raw.size() == 256 && text(raw).substr(0, 4) == "4567");
}

// An interface that takes any plan (for the plan rules).
class PlanSink final : public Interface {
 public:
  const char *name() const override { return "io.github.test.plan"; }
  uint16_t instance() const override { return 0; }
  bool offers(uint8_t op) const override { return op == 1; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return rejected(kRejectUnknownOperation); }
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *, size_t) override { return 0; }
  bool planApply(const RoleAssignment *, size_t) override { return true; }
};

static void testLastMarkMissing() {
  FakeSource src;
  src.say("old output");
  const uint8_t req[11] = {3, 1, 0, 0, 0, 0, 0, 0, 0, 64, 0};   // from last mark, kind reset, max 64
  uint8_t out[80];
  const Result r = src.stream.read(req, out, sizeof out, 64);
  CHECK(r.length == 11 && getU64(out) == 10 && getU16(out + 9) == 0);   // no reset mark: from now (common §1.2), len 0
  const uint8_t oldest[11] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0};        // from the oldest, at most 4
  const Result r2 = src.stream.read(oldest, out, sizeof out, 64);
  CHECK(r2.length == 11 + 4 && getU64(out) == 0 && out[8] == reg::common::kReadFlagsMore && getU16(out + 9) == 4 && out[11] == 'o');
  // marks: 22 bytes each with time_ns (u64), no element length (core §2.3)
  src.stream.mark(reg::common::kMarkKindHost, 7);
  const size_t n = src.stream.marks(0, out, sizeof out);
  CHECK(n == 2 + 22 && out[1] == 1 && out[2 + 12] == reg::common::kMarkKindHost && out[2 + 21] == 7);
  CHECK(getU64(out + 2 + 13) == static_cast<uint64_t>(g_millis) * 1000000u);
}

// The TLV long form (core §2.2): 255 bytes and more as tag 0xFF len(u16); the short form for 254 and fewer; nothing else.
// core §2.2: one TLV form, tag(u8) len(u16) value, whatever the value's length; a value running past the end, tag
// 0x00 / 0x80 / 0x7F / 0xFF: malformed; an unknown critical tag: unsupported with the tag as received. core §2.3: a
// known request TLV longer than its one form - critical: unsupported, else ignored (Tail::fixed).
static void testTlvForm() {
  uint8_t buf[600];
  TlvWriter w(buf, sizeof buf);
  uint8_t big[300];
  for (size_t i = 0; i < sizeof big; ++i) big[i] = static_cast<uint8_t>(i);
  CHECK(w.u8(0x01, 9) && w.put(0x02, big, 300) && w.put(0x03, big, 254));
  CHECK(buf[0] == 0x01 && getU16(buf + 1) == 1 && buf[4] == 0x02 && getU16(buf + 5) == 300);
  CHECK(buf[4 + 3 + 300] == 0x03 && getU16(buf + 4 + 3 + 300 + 1) == 254);
  Tail tail;
  uint8_t out[8];
  static const uint8_t kKnown[] = {0x01, 0x02, 0x03};
  CHECK(tail.parse(buf, w.length(), kKnown, out, sizeof out).resolution == kResolutionCompleted);
  size_t len = 0;
  const uint8_t *v = tail.find(0x02, len);
  CHECK(v && len == 300 && v[299] == 299 % 256);
  v = tail.find(0x03, len);
  CHECK(v && len == 254);
  const uint8_t past[] = {0x02, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};   // 11 bytes said, 10 there: malformed
  CHECK(tail.parse(past, sizeof past, kKnown, out, sizeof out).detail == kRejectMalformed);
  for (uint8_t t : {0x00, 0x7F}) {                                      // never a tag: unknown, ignored
    const uint8_t never[] = {t, 1, 0, 1};
    CHECK(tail.parse(never, sizeof never, kKnown, out, sizeof out).resolution == kResolutionCompleted);
  }
  for (uint8_t t : {0x80, 0xFF}) {                                      // the same, critical: unsupported
    const uint8_t never[] = {t, 1, 0, 1};
    const Result u = tail.parse(never, sizeof never, kKnown, out, sizeof out);
    CHECK(u.detail == kRejectUnsupported && out[0] == t);
  }
  const uint8_t twice[] = {0x01, 1, 0, 5, 0x81, 1, 0, 6};                // a repeated tag: the first is used
  CHECK(tail.parse(twice, sizeof twice, kKnown, out, sizeof out).resolution == kResolutionCompleted);
  v = tail.find(0x01, len);
  CHECK(v && len == 1 && v[0] == 5);
  const uint8_t unknown[] = {0x85, 1, 0, 1};                              // an unknown critical tag
  const Result r = tail.parse(unknown, sizeof unknown, kKnown, out, sizeof out);
  CHECK(r.detail == kRejectUnsupported && r.length == 1 && out[0] == 0x85);
  // fixed(): 0x01 of 1 byte - another length is malformed, critical or not (core §2.3)
  const uint8_t shorter[] = {0x01, 0, 0};
  CHECK(tail.parse(shorter, sizeof shorter, kKnown, out, sizeof out).resolution == kResolutionCompleted);
  CHECK(tail.fixed(0x01, 1, v, out, sizeof out).detail == kRejectMalformed && !v);
  const uint8_t longer[] = {0x81, 2, 0, 7, 8};
  CHECK(tail.parse(longer, sizeof longer, kKnown, out, sizeof out).resolution == kResolutionCompleted);
  const Result l = tail.fixed(0x01, 1, v, out, sizeof out);
  CHECK(l.detail == kRejectMalformed && !v);
  const uint8_t plain[] = {0x01, 2, 0, 7, 8};
  CHECK(tail.parse(plain, sizeof plain, kKnown, out, sizeof out).resolution == kResolutionCompleted);
  CHECK(tail.fixed(0x01, 1, v, out, sizeof out).detail == kRejectMalformed && !v);
  const Result u = Tail::refuse(0x01, false, out, sizeof out);   // a value not handled: unsupported, critical or not
  CHECK(u.detail == kRejectUnsupported && u.length == 1 && out[0] == 0x01);
  const uint8_t exact[] = {0x01, 1, 0, 7};
  CHECK(tail.parse(exact, sizeof exact, kKnown, out, sizeof out).resolution == kResolutionCompleted);
  CHECK(tail.fixed(0x01, 1, v, out, sizeof out).resolution == kResolutionCompleted && v && v[0] == 7);
}

// A vendor-bulk endpoint and the whole result payload of one request.
struct Bulk {
  MemStream stream;
  uint8_t rx[1100], tx[1100];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kVendorBulk, 0};
  // -> the result after its header: resolution, detail, payload
  Bytes send(const Bytes &m) {
    const Bytes f = {uint8_t(m.size()), uint8_t(m.size() >> 8)};
    stream.send(f);
    stream.send(m);
    stream.tx.clear();
    ep.poll();
    if (stream.tx.size() < 7) return {};
    return Bytes(stream.tx.begin() + 5, stream.tx.end());
  }
};

// core §2.3: no ignored list - a completed answer carries only its payload, whatever the request's unknown TLVs; a
// describe request's TLVs follow the same rule (unknown non-critical: ignored).
class FailsEarly final : public Interface {
 public:
  const char *name() const override { return "io.github.test.fails"; }
  uint16_t instance() const override { return 0; }
  bool lockFree(uint8_t) const override { return true; }
  bool offers(uint8_t op) const override { return op == 1; }
  Result handle(uint8_t, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override {
    Tail tail;
    const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
    if (refused(parsed)) return parsed;
    out[0] = 0x05;   // a status
    return failed(1);
  }
};

static void testNoIgnoredList() {
  Bulk b;
  FailsEarly fails;
  b.ep.add(fails);   // fn 1
  Bytes r = b.send(request(1, 1, 1, {0x30, 1, 0, 9, 0x31, 0, 0, 0x30, 0, 0}));
  CHECK(r.size() == 3 && r[0] == kResolutionCompleted && r[1] == kOutcomeFailed && r[2] == 0x05);
  r = b.send(request(3, 0, reg::core::kOpDescribe, {1, 0, 0, 0}));   // describe fn 1: fine
  CHECK(!r.empty() && r[0] == kResolutionCompleted);
  const Bytes plain = r;
  r = b.send(request(4, 0, reg::core::kOpDescribe, {1, 0, 0, 0, 0x30, 0, 0}));   // with an unknown TLV: ignored
  CHECK(r == plain);
  r = b.send(request(5, 0, reg::core::kOpDescribe, {1, 0, 0, 0, 0xb0, 0, 0}));   // critical: unsupported
  CHECK(r.size() == 3 && r[0] == kResolutionRejected && r[1] == kRejectUnsupported && r[2] == 0xb0);
}

// core §6.2 / §6.4 / §9: no resume - end releases the session (its id is then no_session, a resent end is answered
// from the resend table), a lapse the same; open answers lease_ms boot_id; an open with session_id 0 is malformed
// (before locked / force); a resent open of the holder restarts the lease; force takes the lock; confirm carries the
// boot id.
static void testSessionTable() {
  Bulk b;
  b.ep.setBootId(0x11223344);
  Bytes r = b.send(request(1, 0, 0x01, {'O', 'E', 'P', '?', 1, 1}));
  CHECK(r.size() == 2 + 21 && r[0] == 1 && getU32(&r[2 + 13]) == 0x11223344);   // confirm: ... boot_id(u32)
  r = b.send(request(2, 0, 0x10, openPayload(7, 1000)));
  CHECK(r.size() == 2 + 8 && r[0] == 1 && getU32(&r[2]) == 1000 && getU32(&r[2 + 4]) == 0x11223344);
  r = b.send(request(21, 0, 0x10, openPayload(0, 1000)));                    // session_id 0: malformed
  CHECK(r.size() == 2 && r[0] == kResolutionRejected && r[1] == kRejectMalformed);
  Bytes force0 = openPayload(0, 1000);
  force0[8] = 1;
  r = b.send(request(22, 0, 0x10, force0));                                  // malformed before locked / force
  CHECK(r.size() == 2 && r[0] == kResolutionRejected && r[1] == kRejectMalformed && b.ep.locked());
  r = b.send(request(3, 0, 0x11, {}, true, 7));                              // end: everything released
  CHECK(r[0] == 1 && !b.ep.locked());
  r = b.send(request(3, 0, 0x11, {}, true, 7));                              // the same end resent: from the table
  CHECK(r.size() == 2 && r[0] == 1 && !b.ep.locked());
  r = b.send(request(4, 0, 0x12, {}, true, 7));                              // the ended id: no_session (no resume)
  CHECK(r.size() == 2 && r[0] == 0 && r[1] == kRejectNoSession);
  r = b.send(request(5, 0, 0x10, openPayload(7, 1000)));                     // a new open (the same id is allowed)
  CHECK(r[0] == 1 && b.ep.locked());
  g_millis += 1500;                                                          // the lease lapses
  b.ep.poll();
  CHECK(!b.ep.locked());
  r = b.send(request(6, 0, 0x12, {}, true, 7));                              // as after an end: no_session
  CHECK(r.size() == 2 && r[0] == 0 && r[1] == kRejectNoSession);
  r = b.send(request(7, 0, 0x10, openPayload(7, 1000)));
  CHECK(r[0] == 1);
  r = b.send(request(8, 0, 0x10, openPayload(7, 1000)));                     // held and opened again: the lease anew
  CHECK(r[0] == 1);
  r = b.send(request(9, 0, 0x10, openPayload(8, 1000, "other")));            // another id while held: locked
  CHECK(r[0] == 0 && r[1] == kRejectLocked);
  Bytes force = openPayload(8, 1000);
  force[8] = 1;
  r = b.send(request(10, 0, 0x10, force));                                   // taken by force
  CHECK(r[0] == 1);
  r = b.send(request(12, 0, 0x12, {}, true, 7));                             // the forced-out id: locked while 8 holds
  CHECK(r[0] == 0 && r[1] == kRejectLocked);
  r = b.send(request(11, 0, 0x11, {}, true, 8));
  CHECK(r[0] == 1);
  r = b.send(request(13, 0, 0x12, {}, true, 7));                             // then no_session
  CHECK(r[0] == 0 && r[1] == kRejectNoSession);
}

// core §11.3: subscribe (0x30) min_bytes(u16) max_delay_ms(u32) [TLV] and unsubscribe (0x32) [TLV] are ops of the
// interface that sends the notifications, in its ops only when it does (notifies): a fn that sends none, and fn 0, answer
// them unknown_operation; an unsubscribe of nothing is fine; an event goes out at the next poll whatever the batching
// (min_bytes / max_delay_ms batch the data only). core §7.7: clock answers boot_id(u32) uptime_ns(u64), without a
// session too. fn 0's describe has no plan_roles: oep.probe.plan's describe has it (oep-if-plan §1).
class Talker final : public Interface {
 public:
  const char *name() const override { return "io.github.test.talker"; }
  uint16_t instance() const override { return 0; }
  bool offers(uint8_t op) const override { return op == 1; }
  bool notifies() const override { return true; }
  bool subscribe(bool on) override { subscribed = on; ++calls; return true; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return completed(); }
  bool subscribed = false;
  int calls = 0;
};
static void testSubscriptions() {
  Bulk b;
  b.ep.setBootId(5);
  PlanSink quiet;
  Talker talker;
  CHECK(b.ep.add(quiet));    // fn 1: emits nothing
  CHECK(b.ep.add(talker));   // fn 2: notifies
  CHECK(b.ep.planFn() == 3 && b.ep.restartFn() == 0);
  CHECK(b.send(request(1, 0, 0x10, openPayload(7, 3000)))[0] == 1);
  Bytes r = b.send(request(2, 1, 0x30, {0, 0, 0, 0, 0, 0}, true, 7));
  CHECK(r.size() == 2 && r[0] == 0 && r[1] == kRejectUnknownOperation);
  CHECK(b.send(request(3, 1, 0x32, {}, true, 7))[1] == kRejectUnknownOperation);
  CHECK(b.send(request(4, 0, 0x30, {0, 0, 0, 0, 0, 0}, true, 7))[1] == kRejectUnknownOperation);   // fn 0: none
  CHECK(b.send(request(5, 2, 0x32, {}, true, 7))[0] == 1 && talker.calls == 0);   // nothing subscribed: fine
  CHECK(b.send(request(6, 2, 0x30, {0, 0, 0, 0, 0, 0}))[1] == kRejectSessionRequired);
  r = b.send(request(7, 2, 0x30, {0, 0, 0, 0, 100}, true, 7));   // too short: malformed
  CHECK(r[0] == 0 && r[1] == kRejectMalformed && !talker.subscribed);
  // min_bytes 1000, max_delay_ms 1000: the event goes at the next poll all the same
  CHECK(b.send(request(8, 2, 0x30, {0xe8, 0x03, 0xe8, 0x03, 0, 0}, true, 7))[0] == 1 && talker.subscribed);
  const uint8_t ev[3] = {9, 8, 7};
  CHECK(b.ep.event(talker, 0x01, ev, sizeof ev));
  b.stream.tx.clear();
  b.ep.poll();
  const Bytes &t = b.stream.tx;   // length(u16) role fn(u16) seq(u16) kind payload
  CHECK(t.size() == 2 + 6 + 3 && t[2] == kRoleEvent && getU16(&t[3]) == 2 && getU16(&t[5]) == 0 && t[7] == 0x01 && t[8] == 9);
  CHECK(b.send(request(9, 2, 0x32, {}, true, 7))[0] == 1 && !talker.subscribed);
  CHECK(!b.ep.event(talker, 0x01, ev, sizeof ev));   // not subscribed: not queued
  // ops: fn 2 sets 0x30 / 0x32 (base 1: bits 0, 47, 49), fn 1 neither
  Bytes d = b.send(request(10, 0, 0x03, {2, 0, 0, 0}));
  CHECK(d.size() == 3 + 3 + 1 + 7 && d[3] == 0x09 && d[6] == 1 && d[7] == 1 && d[12] == 0x80 && d[13] == 0x02);
  d = b.send(request(11, 0, 0x03, {1, 0, 0, 0}));
  CHECK(d.size() == 3 + 3 + 2 && d[3] == 0x09 && d[6] == 1 && d[7] == 1);
  // clock (core §7.7): boot_id, then the clock read for the answer; with session_id 0 the lock is untouched
  g_millis += 150;
  r = b.send(request(12, 0, 0x04, {}));
  CHECK(r.size() == 2 + 12 && r[0] == 1 && getU32(&r[2]) == 5 && getU64(&r[6]) == static_cast<uint64_t>(g_millis) * 1000000u);
  CHECK(b.send(request(13, 0, 0x04, {1}))[1] == kRejectMalformed);   // not a TLV
  // the describe of fn 0: max_op_ms(u32), no plan_roles; oep.probe.plan's (fn 3): plan_roles 0x40
  d = b.send(request(14, 0, 0x03, {0, 0, 0, 0}));
  const Bytes roles = {0x4B, 4, 0}, op = {0x4D, 4, 0, 0x10, 0x27, 0, 0};
  CHECK(std::search(d.begin(), d.end(), roles.begin(), roles.end()) == d.end());
  CHECK(std::search(d.begin(), d.end(), op.begin(), op.end()) != d.end());
  d = b.send(request(15, 0, 0x03, {3, 0, 0, 0}));
  const Bytes plan = {0x09, 2, 0, 1, 0x03, 0x40, 4, 0, uint8_t(Endpoint::kMaxRoles), 0, 0, 0};
  CHECK(Bytes(d.begin() + 3, d.end()) == plan);
}

// core §9: one space of numbers, each the previous plus 1, a live number of another kind refused
// unavailable cause 6, an unknown one no_connection.
static void testResourceNumbers() {
  const uint16_t a = ResourceNumbers::take(ResourceNumbers::kConnection);
  const uint16_t s = ResourceNumbers::take(ResourceNumbers::kStream);
  CHECK(a != 0 && s != 0 && a != s);
  CHECK(ResourceNumbers::kindOf(a) == ResourceNumbers::kConnection && ResourceNumbers::kindOf(s) == ResourceNumbers::kStream);
  uint8_t out[8];
  Result r = ResourceNumbers::refuse(s, ResourceNumbers::kConnection, out, sizeof out);
  CHECK(r.detail == kRejectUnavailable && r.length == 4 && out[3] == reg::core::kUnavailableCauseWrongState);
  ResourceNumbers::close(a);
  r = ResourceNumbers::refuse(a, ResourceNumbers::kConnection, out, sizeof out);
  CHECK(r.detail == kRejectNoConnection);
  bool reused = false;
  for (int i = 0; i < 10; ++i) reused |= ResourceNumbers::take(ResourceNumbers::kConnection) == a;
  CHECK(!reused);   // the count goes on past it (+1 each time)
  CHECK(ResourceNumbers::reopen(a, ResourceNumbers::kConnection) && ResourceNumbers::kindOf(a) == ResourceNumbers::kConnection);
  ResourceNumbers::close(a);
  ResourceNumbers::close(s);
}

static void testSettingsPlanStays() {
  MemStream bulk;
  static uint8_t rx[1100], tx[1100];
  Endpoint ep(bulk, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kVendorBulk, 0);
  PlanSink a, b;
  ep.add(a);                                   // fn 1: its plan from the settings
  ep.add(b);                                   // fn 2: a session's plan
  const RoleAssignment saved[] = {{1, 1, 12}};
  const uint16_t fn1[] = {1};
  CHECK(ep.replacePlan(saved, 1, fn1, 1) == 0);
  auto send = [&](const Bytes &m) {
    const Bytes f = {uint8_t(m.size()), uint8_t(m.size() >> 8)};
    bulk.send(f);
    bulk.send(m);
    bulk.tx.clear();
    ep.poll();
    return bulk.tx.size() >= 7 ? bulk.tx[5] : 0xff;   // the resolution (after length(2) role corr(2))
  };
  CHECK(send(request(1, 0, 0x10, openPayload(7, 3000))) == 1);
  const Bytes plan2 = {0x90, 5, 0, 2, 0, 1, 20, 0};
  CHECK(send(request(2, ep.planFn(), kOpPlanApply, plan2, true, 7)) == 1);            // fn 2 planned by the session
  const Bytes plan1 = {0x90, 5, 0, 1, 0, 1, 13, 0};
  CHECK(send(request(3, ep.planFn(), kOpPlanApply, plan1, true, 7)) == 0);            // fn 1 is the settings': refused
  CHECK(send(request(4, ep.planFn(), kOpPlanRelease, {0}, true, 7)) == 1);              // release every fn
  RoleAssignment now[4];
  const size_t n = ep.plan(now, 4);
  CHECK(n == 1 && now[0].function == 1 && now[0].channel == 12);    // the settings' plan stays, the session's went
}

// An interface that declares no channel the host asks for: plan_apply is rejected unsupported with tag 0x90 (core §8's
// table, §4.3: tag(u8) first in the payload).
class PlanUndeclared final : public Interface {
 public:
  const char *name() const override { return "io.github.test.undeclared"; }
  uint16_t instance() const override { return 0; }
  bool offers(uint8_t op) const override { return op == 1; }   // one op: an interface offers at least one (core §7.4)
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return rejected(kRejectUnknownOperation); }
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *, size_t) override { return kRejectUnsupported; }
};

static void testPlanUnsupportedTag() {
  MemStream bulk;
  static uint8_t rx[1100], tx[1100];
  Endpoint ep(bulk, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kVendorBulk, 0);
  PlanUndeclared u;
  ep.add(u);   // fn 1
  auto send = [&](const Bytes &m) {
    const Bytes f = {uint8_t(m.size()), uint8_t(m.size() >> 8)};
    bulk.send(f);
    bulk.send(m);
    bulk.tx.clear();
    ep.poll();
  };
  send(request(1, 0, 0x10, openPayload(7, 3000)));
  send(request(2, ep.planFn(), kOpPlanApply, {0x90, 5, 0, 1, 0, 1, 20, 0}, true, 7));
  // length(2) role corr(2) resolution detail payload
  CHECK(bulk.tx.size() == 8 && bulk.tx[5] == kResolutionRejected && bulk.tx[6] == kRejectUnsupported && bulk.tx[7] == 0x90);
  RoleAssignment now[2];
  CHECK(ep.plan(now, 2) == 0);
}

// An interface whose channels are shared with no other plan (an analog input, oep-if-capture §1.2).
class PlanAlone final : public Interface {
 public:
  const char *name() const override { return "io.github.test.alone"; }
  uint16_t instance() const override { return 0; }
  bool offers(uint8_t op) const override { return op == 1; }   // one op: an interface offers at least one (core §7.4)
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return rejected(kRejectUnknownOperation); }
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *, size_t) override { return 0; }
  bool planApply(const RoleAssignment *, size_t) override { return true; }
  bool planShares() const override { return false; }
};

static void testPlanNotShared() {
  MemStream bulk;
  static uint8_t rx[1100], tx[1100];
  Endpoint ep(bulk, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kVendorBulk, 0);
  PlanSink a, b;
  PlanAlone alone;
  ep.add(a);       // fn 1
  ep.add(b);       // fn 2
  ep.add(alone);   // fn 3
  const uint16_t f1[] = {1}, f2[] = {2}, f3[] = {3}, f13[] = {1, 3};
  const RoleAssignment a12[] = {{1, 0, 12}}, b12[] = {{2, 0, 12}}, alone12[] = {{3, 0, 12}}, alone13[] = {{3, 0, 13}};
  CHECK(ep.replacePlan(a12, 1, f1, 1) == 0);
  CHECK(ep.replacePlan(b12, 1, f2, 1) == 0);               // two that share: fine
  CHECK(ep.replacePlan(alone12, 1, f3, 1) == kRejectUnavailable);   // onto a planned channel: refused
  CHECK(ep.replacePlan(alone13, 1, f3, 1) == 0);
  const RoleAssignment a13[] = {{1, 0, 13}};
  CHECK(ep.replacePlan(a13, 1, f1, 1) == kRejectUnavailable);       // onto its channel: refused too
  RoleAssignment now[4];
  CHECK(ep.plan(now, 4) == 3);                                      // nothing changed (a still on 12)
  const RoleAssignment both[] = {{1, 0, 14}, {3, 0, 14}};
  CHECK(ep.replacePlan(both, 2, f13, 2) == kRejectUnavailable);     // in one request
}

// A capture-group track for the group's own rules: it starts, follows, and is told or tells the trigger's time.
class FakeTrack final : public Interface, public GroupTrack {
 public:
  explicit FakeTrack(bool trigger) : trigger_(trigger) {}
  const char *name() const override { return "io.github.test.track"; }
  uint16_t instance() const override { return 0; }
  bool offers(uint8_t op) const override { return op == 1; }   // one op: an interface offers at least one (core §7.4)
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return rejected(kRejectUnknownOperation); }
  bool trackReady() const override { return state != 2 && state != 3; }
  uint8_t trackMode() const override { return 1; }
  bool trackTriggered() const override { return trigger_; }
  uint32_t trackLoad() const override { return 0; }
  bool trackStart() override { state = 2; fired = false; ++starts; ++generation; return true; }   // waiting; the last trigger gone
  uint32_t trackGeneration() const override { return generation; }
  uint32_t generation = 0;
  void trackStop() override { state = 1; }
  uint8_t trackState() const override { return state; }
  bool trackCanFollow() const override { return !trigger_; }
  bool trackStartFollowing() override { state = 2; told = ~uint64_t{0}; return true; }
  void trackTriggerAt(uint64_t ns) override { told = ns; state = 4; }
  bool trackTriggerNs(uint64_t &ns) const override { if (fired) ns = fired_ns; return fired; }
  bool trackArmed() const override { return armed; }
  void fire(uint64_t ns) { fired = true; fired_ns = ns; state = 4; }
  uint8_t state = 1;   // configured
  bool fired = false, armed = true;
  uint64_t fired_ns = 0, told = ~uint64_t{0};
  int starts = 0;

 private:
  bool trigger_;
};

// Two triggered runs in one boot: the second follows its own trigger, not the first's (0.0.17: the group asked the
// trigger track before starting it, while the followers' pretriggers filled, and took the last run's time).
static void testGroupSecondRun() {
  MemStream bulk;
  static uint8_t rx[1100], tx[1100];
  Endpoint ep(bulk, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kVendorBulk, 0);
  FakeTrack logic(true), analog(false);
  CaptureGroup group(ep);
  ep.add(logic);    // fn 1
  ep.add(analog);   // fn 2
  ep.add(group);    // fn 3
  group.addTrack(logic, logic);
  group.addTrack(analog, analog);
  uint8_t out[64];
  const uint8_t bind[] = {2, 1, 0, 2, 0, 0x01, 2, 0, 1, 0};   // fns 1, 2; trigger_track fn 1
  const uint8_t none[] = {0};
  auto triggerNs = [&]() {
    group.handle(0x05, nullptr, 0, out, sizeof out);
    return getU64(out + 9);
  };
  for (uint64_t run = 1; run <= 2; ++run) {
    CHECK(group.handle(0x01, bind, sizeof bind, out, sizeof out).resolution == kResolutionCompleted);
    analog.armed = false;                                   // its pretrigger is filling
    const Result started = group.handle(0x02, nullptr, 0, out, sizeof out);
    // blocking_ms start_ns, the group's generation (one up at every start, kept across binds), n = 2, (fn, generation)
    // in bind order: fn 1 = run (its start comes later, the one it will make), fn 2 as the fake keeps it (0)
    CHECK(started.resolution == kResolutionCompleted && started.length == 12 + 4 + 1 + 12);
    CHECK(getU32(out + 12) == run && out[16] == 2 && getU16(out + 17) == 1 && getU32(out + 19) == run &&
          getU16(out + 23) == 2 && getU32(out + 25) == 0);
    CHECK(group.handle(0x05, nullptr, 0, out, sizeof out).length == 23 && getU32(out + 19) == run);   // status: the group's
    group.poll();
    CHECK(logic.starts == static_cast<int>(run) - 1);        // not yet: the follower is not armed
    CHECK(triggerNs() == ~uint64_t{0} && analog.told == ~uint64_t{0});   // nothing from the last run
    analog.armed = true;
    group.poll();
    CHECK(logic.starts == static_cast<int>(run));
    CHECK(triggerNs() == ~uint64_t{0});
    logic.fire(1000 * run);
    group.poll();
    CHECK(triggerNs() == 1000 * run && analog.told == 1000 * run);
    CHECK(group.handle(0x01, none, sizeof none, out, sizeof out).resolution == kResolutionCompleted);   // unbind
  }
  // the trigger track's generation after 0xFFFFFFFF is 1, never 0 (capture §3.2: nextGeneration)
  CHECK(group.handle(0x01, bind, sizeof bind, out, sizeof out).resolution == kResolutionCompleted);
  logic.generation = 0xFFFFFFFFu;
  CHECK(group.handle(0x02, nullptr, 0, out, sizeof out).resolution == kResolutionCompleted);
  CHECK(getU32(out + 12) == 3 && getU16(out + 17) == 1 && getU32(out + 19) == 1);
  CHECK(nextGeneration(0) == 1 && nextGeneration(1) == 2 && nextGeneration(0xFFFFFFFFu) == 1);
  // capture §4.1's P_k = ceil(P x num_k x den_t / (den_k x num_t)): the spec's example (20 MHz, P 1000 -> 48 kHz: 3),
  // exact multiples, products past 64 bits, and more than a u32
  uint32_t pk = 0;
  CHECK(groupPretrigger(1000, 20000000, 1, 48000, 1, pk) && pk == 3);
  CHECK(groupPretrigger(1000, 20000000, 1, 10000000, 1, pk) && pk == 500);
  CHECK(groupPretrigger(0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, pk) && pk == 0xFFFFFFFFu);
  CHECK(groupPretrigger(3, 0xFFFFFFFFu, 7, 0xFFFFFFFEu, 5, pk) && pk == 5);   // 3 x 7 x (2^32 - 2) / (5 x (2^32 - 1)) = 4.199..
  CHECK(!groupPretrigger(0xFFFFFFFFu, 1, 1, 2, 1, pk));                        // 2^33 - 2: no u32
  CHECK(!groupPretrigger(10, 0, 1, 1, 1, pk));
}

static void testPlanCapacity() {
  MemStream bulk;
  static uint8_t rx[1100], tx[1100];
  Endpoint ep(bulk, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kVendorBulk, 0);
  PlanSink a;
  ep.add(a);
  auto send = [&](const Bytes &m) {
    const Bytes f = {uint8_t(m.size()), uint8_t(m.size() >> 8)};
    bulk.send(f);
    bulk.send(m);
    bulk.tx.clear();
    ep.poll();
    return bulk.tx;
  };
  CHECK(send(request(1, 0, 0x10, openPayload(7, 3000))).size() >= 7);
  auto plan = [](size_t n) {
    Bytes p;
    for (size_t k = 0; k < n; ++k) p.insert(p.end(), {0x90, 5, 0, 1, 0, 1, uint8_t(k), 0});
    return p;
  };
  const Bytes over = send(request(2, ep.planFn(), kOpPlanApply, plan(Endpoint::kMaxRoles + 1), true, 7));
  CHECK(over.size() >= 7 && over[5] == 0 && over[6] == kRejectUnavailable);   // more than the plan holds: unavailable
  CHECK(send(request(3, ep.planFn(), kOpPlanApply, plan(Endpoint::kMaxRoles), true, 7))[5] == 1);
  const Bytes d = send(request(4, 0, 0x03, {uint8_t(ep.planFn()), 0, 0, 0}));  // describe oep.probe.plan
  const Bytes want = {0x40, 4, 0, uint8_t(Endpoint::kMaxRoles), 0, 0, 0};
  CHECK(std::search(d.begin(), d.end(), want.begin(), want.end()) != d.end());   // plan_roles (u32) declared
}

// probe.config disable (§1): a plan_apply naming a disabled channel is unavailable cause 5 with the channel.
static void testDisabledChannel() {
  MemStream bulk;
  static uint8_t rx[1100], tx[1100];
  Endpoint ep(bulk, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kVendorBulk, 0);
  PlanSink a;
  ep.add(a);
  auto send = [&](const Bytes &m) {
    const Bytes f = {uint8_t(m.size()), uint8_t(m.size() >> 8)};
    bulk.send(f);
    bulk.send(m);
    bulk.tx.clear();
    ep.poll();
    return bulk.tx;
  };
  send(request(1, 0, 0x10, openPayload(7, 3000)));
  ep.setDisabled(uint64_t{1} << 12);
  const Bytes r = send(request(2, ep.planFn(), kOpPlanApply, {0x90, 5, 0, 1, 0, 1, 12, 0}, true, 7));
  const Bytes cause5 = {0x01, 1, 0, 5, 0x02, 2, 0, 12, 0};   // cause 5, channel 12 (no holder_kind, core §4.3)
  CHECK(r.size() == 2 + 5 + cause5.size() && r[5] == 0 && r[6] == kRejectUnavailable);
  CHECK(Bytes(r.begin() + 7, r.end()) == cause5);
  CHECK(send(request(3, ep.planFn(), kOpPlanApply, {0x90, 5, 0, 1, 0, 1, 13, 0}, true, 7))[5] == 1);   // another channel: as before
  ep.setDisabled(0);
  CHECK(send(request(4, ep.planFn(), kOpPlanApply, {0x90, 5, 0, 1, 0, 1, 12, 0}, true, 7))[5] == 1);   // enabled again
}

// ---- oep.probe.link port_speed (oep-if-link §3) ---------------------------------------------------------------------------------------------

static uint32_t g_baud = 115200;
static int g_switches = 0;
static MemStream *g_speed_stream = nullptr;
static size_t g_tx_at_switch = 0;   // what the port had sent when it switched
static uint32_t speedHook(uint8_t, uint32_t baud, bool apply) {
  if (baud < 1200 || baud > 5000000) return 0;   // this "UART" cannot make it
  if (apply) { g_baud = baud; ++g_switches; g_tx_at_switch = g_speed_stream->tx.size(); }
  return baud;
}

// A UART bridge (transport 0) and a vendor bulk (transport 1), oep.probe.link as fn 1; the result's resolution, detail and
// payload.
struct Uart {
  MemStream stream, bulk;
  uint8_t rx[1100], rx2[1100], tx[1100];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kUartBridge};
  Link link{ep};
  Uart(bool on = true) {
    ep.addTransport(bulk, rx2, sizeof rx2, Endpoint::kVendorBulk, 0);
    ep.add(link);
    if (on) ep.setPortSpeed(speedHook, 115200);
    g_baud = 115200;
    g_switches = 0;
    g_speed_stream = &stream;
  }
  Bytes send(const Bytes &m) {
    stream.send(frame(m));
    stream.tx.clear();
    ep.poll();
    std::vector<Bytes> frames;
    Bytes raw;
    split(stream.tx, frames, raw);
    if (frames.empty() || frames.back().size() < 5) return {};
    return Bytes(frames.back().begin() + 3, frames.back().end());
  }
  Bytes sendBulk(const Bytes &m) {
    const Bytes f = {uint8_t(m.size()), uint8_t(m.size() >> 8)};
    bulk.send(f);
    bulk.send(m);
    bulk.tx.clear();
    ep.poll();
    if (bulk.tx.size() < 7) return {};
    return Bytes(bulk.tx.begin() + 5, bulk.tx.end());
  }
  void noise() { stream.send({0, 0x31, 0x32, 0x33, 0}); ep.poll(); }   // a candidate whose CRC does not match
};
// An interface whose one op takes `takes_ms` of the clock to answer (a long verify, a block op on a slow line).
class SlowSink final : public Interface {
 public:
  uint32_t takes_ms = 0;
  const char *name() const override { return "io.github.test.slow"; }
  uint16_t instance() const override { return 0; }
  bool offers(uint8_t op) const override { return op == 1; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { g_millis += takes_ms; return completed(); }
};
// port_speed's request (oep-if-link §3): baud(u32) step(u8) verify_ms(u16) - the port is the one it comes on
static Bytes speedReq(uint32_t baud, uint8_t step, uint16_t verify_ms) {
  Bytes p = u32(baud);
  p.push_back(step);
  p.push_back(uint8_t(verify_ms));
  p.push_back(uint8_t(verify_ms >> 8));
  return p;
}
// oep.probe.link (fn 1) declares port_speed (op 0x03) in its ops tag (core §1.2, §7.4)
static bool describesPortSpeed(Uart &u) {
  const Bytes r = u.send(request(90, 0, 0x03, {1, 0, 0, 0}));
  if (r.size() < 3 + 5 || r[0] != 1 || r[3] != kTagOps) return false;   // ops first: 09 len(u16) base bitmap
  const uint8_t base = r[6];
  return base <= 3 && 3u - base < 8u * (getU16(&r[4]) - 1u) && ((r[7 + (3 - base) / 8] >> ((3 - base) % 8)) & 1);
}

static void testPortSpeed() {
  {   // off: not in oep.probe.link's ops, the op unknown
    Uart u(false);
    CHECK(!describesPortSpeed(u));
    u.send(request(1, 0, 0x10, openPayload(5, 5000)));
    const Bytes r = u.send(request(2, 1, 0x03, speedReq(1500000, 0, 2000), true, 5));
    CHECK(r.size() == 2 && r[0] == 0 && r[1] == kRejectUnknownOperation);
  }
  Uart u;
  SlowSink slow;
  CHECK(u.ep.add(slow));   // fn 2, before the first poll (core §7.2: the list is fixed for a boot)
  CHECK(describesPortSpeed(u));
  Bytes r = u.send(request(1, 1, 0x03, speedReq(1500000, 0, 2000)));   // the lock is needed
  CHECK(r.size() >= 2 && r[0] == 0 && r[1] == kRejectSessionRequired);
  u.send(request(2, 0, 0x10, openPayload(5, 60000)));
  r = u.sendBulk(request(4, 1, 0x03, speedReq(1500000, 0, 2000), true, 5));   // the bulk is no UART bridge: cause 6
  CHECK(r.size() >= 6 && r[0] == 0 && r[1] == kRejectUnavailable && r[5] == 6);
  r = u.send(request(5, 1, 0x03, speedReq(9000000, 0, 2000), true, 5));   // a baud the UART cannot make
  CHECK(r.size() == 3 && r[0] == 0 && r[1] == kRejectUnsupported && r[2] == 0);
  r = u.send(request(6, 1, 0x03, speedReq(1500000, 1, 2000), true, 5));   // commit at the boot speed: cause 6
  CHECK(r.size() >= 5 && r[1] == kRejectUnavailable && r[2] == 0x01 && r[5] == 6);
  r = u.send(request(7, 1, 0x03, speedReq(0, 2, 0), true, 5));   // revert at the boot speed: cause 6
  CHECK(r.size() >= 5 && r[1] == kRejectUnavailable && r[2] == 0x01 && r[5] == 6);
  // step 3 and up: a value a later revision may define - unsupported, tag 0x00 (core §2.5, oep-if-link §3)
  r = u.send(request(8, 1, 0x03, speedReq(1500000, 3, 2000), true, 5));
  CHECK(r.size() == 3 && r[0] == 0 && r[1] == kRejectUnsupported && r[2] == 0);
  r = u.send(request(9, 1, 0x03, speedReq(1500000, 0, 0), true, 5));   // try with verify_ms 0: malformed
  CHECK(r.size() == 2 && r[0] == 0 && r[1] == kRejectMalformed);
  r = u.send(request(10, 1, 0x03, Bytes{0x60, 0xe3, 0x16, 0, 0, 0xd0, 0x07, 0, 0, 0, 0}, true, 5));   // the old form
  CHECK(r.size() == 2 && r[0] == 0 && r[1] == kRejectMalformed);   // its tail is no TLV
  CHECK(g_switches == 0);
  {   // port_speed_tolerance_pct: a UART whose nearest rate is more than 2 % off is unsupported
    Uart v;
    v.ep.setPortSpeed([](uint8_t, uint32_t baud, bool) { return baud + baud / 40; }, 115200);   // 2.5 % fast
    v.send(request(1, 0, 0x10, openPayload(5, 5000)));
    r = v.send(request(2, 1, 0x03, speedReq(1000000, 0, 2000), true, 5));
    CHECK(r.size() == 3 && r[0] == 0 && r[1] == kRejectUnsupported && r[2] == 0);
    v.ep.setPortSpeed([](uint8_t, uint32_t baud, bool) { return baud + baud / 50; }, 115200);   // 2 %: taken
    r = v.send(request(3, 1, 0x03, speedReq(1000000, 0, 2000), true, 5));
    CHECK(r.size() == 6 && r[0] == 1 && getU32(&r[2]) == 1020000);
  }
  u.stream.tx.clear();
  g_speed_stream = &u.stream;
  g_baud = 115200;
  g_switches = 0;

  // try -> commit: answered at the old speed (the hook not yet called when the answer was written), then switched
  r = u.send(request(11, 1, 0x03, speedReq(1500000, 0, 2000), true, 5));
  CHECK(r.size() == 6 && r[0] == 1 && getU32(&r[2]) == 1500000);
  CHECK(g_tx_at_switch == u.stream.tx.size());   // the whole answer was out before the switch
  CHECK(g_baud == 1500000 && !u.ep.portSpeedCommitted() && u.ep.portSpeedNow() == 1500000);
  r = u.send(request(12, 1, 0x03, speedReq(1000000, 1, 0), true, 5));   // commit of another baud: cause 6
  CHECK(r.size() >= 6 && r[1] == kRejectUnavailable && r[5] == 6);
  r = u.send(request(13, 1, 0x03, speedReq(2000000, 0, 2000), true, 5));   // try while trying: cause 6, no switch
  CHECK(r.size() >= 5 && r[1] == kRejectUnavailable && r[2] == 0x01 && r[5] == 6);
  r = u.sendBulk(request(14, 1, 0x03, speedReq(2000000, 0, 2000), true, 5));   // another port meanwhile: cause 6
  CHECK(r.size() >= 6 && r[1] == kRejectUnavailable && r[5] == 6);
  CHECK(g_baud == 1500000 && g_switches == 1 && !u.ep.portSpeedCommitted());
  // broken candidates change nothing (no revert on them, oep-if-link §3)
  u.noise();
  u.noise();
  u.noise();
  CHECK(g_baud == 1500000);
  g_millis += 500;
  r = u.send(request(15, 1, 0x03, speedReq(1500000, 1, 0), true, 5));
  CHECK(r.size() == 6 && r[0] == 1 && getU32(&r[2]) == 1500000 && u.ep.portSpeedCommitted());
  r = u.send(request(16, 1, 0x03, speedReq(1500000, 1, 0), true, 5));   // commit while committed: cause 6
  CHECK(r.size() >= 5 && r[1] == kRejectUnavailable && r[2] == 0x01 && r[5] == 6);
  r = u.send(request(17, 1, 0x03, speedReq(2000000, 0, 2000), true, 5));   // try while committed: cause 6
  CHECK(r.size() >= 5 && r[1] == kRejectUnavailable && r[2] == 0x01 && r[5] == 6);
  CHECK(g_baud == 1500000 && g_switches == 1 && u.ep.portSpeedCommitted());
  for (int i = 0; i < 5; ++i) u.noise();
  CHECK(g_baud == 1500000);
  // committed: port_speed_idle_ms with no good frame reverts; a good frame restarts the count
  g_millis += reg::kPortSpeedIdleMs - 1;
  u.ep.poll();
  CHECK(g_baud == 1500000);
  u.send(request(18, 0, 0x13, {}));   // any good frame (lock_state, no session)
  g_millis += reg::kPortSpeedIdleMs - 1;
  u.ep.poll();
  CHECK(g_baud == 1500000);
  g_millis += 1;
  u.ep.poll();
  CHECK(g_baud == 115200 && u.ep.portSpeedNow() == 0);

  // try timeout: no commit within verify_ms
  r = u.send(request(19, 1, 0x03, speedReq(750000, 0, 1000), true, 5));
  CHECK(r[0] == 1 && g_baud == 750000);
  g_millis += 999;
  u.ep.poll();
  CHECK(g_baud == 750000);
  g_millis += 1;
  u.ep.poll();
  CHECK(g_baud == 115200);
  r = u.send(request(20, 1, 0x03, speedReq(750000, 1, 0), true, 5));   // too late to commit
  CHECK(r.size() >= 6 && r[5] == 6);

  // step 2: answered (the boot speed) at the speed now, then back
  u.send(request(23, 1, 0x03, speedReq(500000, 0, 2000), true, 5));
  u.send(request(24, 1, 0x03, speedReq(500000, 1, 0), true, 5));
  const int before = g_switches;
  u.stream.tx.clear();
  r = u.send(request(25, 1, 0x03, speedReq(0, 2, 0), true, 5));
  CHECK(r.size() == 6 && r[0] == 1 && getU32(&r[2]) == 115200 && g_baud == 115200 && g_switches == before + 1);
  CHECK(g_tx_at_switch == u.stream.tx.size());

  // the session's end: its answer at the fast speed, then back; a lapse too
  u.send(request(26, 1, 0x03, speedReq(500000, 0, 2000), true, 5));
  u.send(request(27, 1, 0x03, speedReq(500000, 1, 0), true, 5));
  r = u.send(request(28, 0, 0x11, {}, true, 5));
  CHECK(r.size() >= 1 && r[0] == 1 && g_baud == 115200 && g_tx_at_switch == u.stream.tx.size());
  u.send(request(29, 0, 0x10, openPayload(5, 1000)));
  u.send(request(30, 1, 0x03, speedReq(500000, 0, 2000), true, 5));
  u.send(request(31, 1, 0x03, speedReq(500000, 1, 0), true, 5));
  CHECK(g_baud == 500000);
  g_millis += 1100;   // the lease lapses
  u.ep.poll();
  CHECK(!u.ep.locked() && g_baud == 115200);
  // taken by force from the other transport
  u.send(request(32, 0, 0x10, openPayload(6, 5000)));
  u.send(request(33, 1, 0x03, speedReq(500000, 0, 2000), true, 6));
  CHECK(g_baud == 500000);
  Bytes force = openPayload(7, 5000);
  force[8] = 1;
  r = u.sendBulk(request(34, 0, 0x10, force));
  CHECK(r.size() >= 1 && r[0] == 1 && g_baud == 115200);
  // taken back by force on the UART
  force = openPayload(8, 60000);
  force[8] = 1;
  r = u.send(request(40, 0, 0x10, force));
  CHECK(r.size() >= 1 && r[0] == 1);
  // condition 2: port_speed_idle_ms is not counted while a request runs (like the lease, it runs from the answer). A
  // request that takes longer than that leaves the port at the raised speed, and the count starts over after it.
  u.send(request(45, 1, 0x03, speedReq(500000, 0, 2000), true, 8));
  u.send(request(46, 1, 0x03, speedReq(500000, 1, 0), true, 8));
  CHECK(u.ep.portSpeedCommitted());
  g_millis += 2900;
  slow.takes_ms = 3500;   // longer than port_speed_idle_ms
  r = u.send(request(47, 2, 0x01, {}, true, 8));
  CHECK(r.size() >= 1 && r[0] == 1 && g_baud == 500000 && u.ep.portSpeedCommitted());
  g_millis += reg::kPortSpeedIdleMs - 1;   // the count runs from the answer
  u.ep.poll();
  CHECK(g_baud == 500000);
  g_millis += 1;
  u.ep.poll();
  CHECK(g_baud == 115200);
}

// The block ops' length rule (core §7.4 max_length, oep-if-debug §4.5 / §6), shared by riscv-dm and arm-adi through
// blockMaxLength / blockCountFits: what each profile declares, that both frames fit it, and the refusal over it.
static void testBlockLength() {
  CHECK(blockMaxLength(512, 1024) == 488);     // Esp32.h: max_frame 512, riscv-dm's 1 KiB word buffer (was 492)
  CHECK(blockMaxLength(1024, 1024) == 1000);   // Esp32P4.h / Rp2.h: max_frame 1024 (was 1004)
  CHECK(blockMaxLength(1024, 0) == 1000);      // Rp2.h arm-adi: no buffer of its own, the frame alone bounds it
  CHECK(blockMaxLength(4096, 1024) == 1024);   // a wider frame: the buffer bounds it
  CHECK(blockMaxLength(0, 1024) == 1024);      // the frame limit not told yet
  CHECK(blockMaxLength(27, 0) == 0 && blockMaxLength(30, 0) == 4 && blockMaxLength(0x20000, 0) == 0xFFFC);
  for (size_t max_frame : {size_t(512), size_t(1024), size_t(2048)}) {   // the spec's two frames at the declared length
    const uint16_t length = blockMaxLength(max_frame, 1024);
    CHECK(length % 4 == 0 && length <= max_frame - 24);
    CHECK(5u + 2u + 1u + length <= max_frame);               // read_block's answer
    CHECK(10u + 2u + 4u + 2u + length <= max_frame);         // write_block's request
  }
  CHECK(blockCountFits(0, 488) && blockCountFits(122, 488) && !blockCountFits(123, 488));
  CHECK(blockCountFits(250, 1000) && !blockCountFits(251, 1000) && blockCountFits(0, 0) && !blockCountFits(1, 0));
  CHECK(!blockCountFits(0xFFFF, 0xFFFC));
  uint8_t out[8];
  const Result r = unsupportedValue(out, sizeof out);   // the refusal the targets send for count x 4 > max_length
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && r.length == 1 && out[0] == 0x00);
}

// core §6.5 (C-19): a sketch that sets no boot_id gets one picked when the first message arrives - on a platform with
// no hardware random source (the host build's), from the timer's count at that moment, so two boots whose first
// message comes at different times differ; it stays the same for the boot. core §2.6a (C-31): the clock does not
// decrease or wrap while the boot_id is the same - a 32-bit micros() is extended past its wrap at 71.6 minutes.
static void testBootIdAndClock() {
  const Bytes confirm = request(1, 0, 0x01, {'O', 'E', 'P', '?', 1, 1});
  uint32_t ids[2];
  for (int boot = 0; boot < 2; ++boot) {
    Bulk b;
    g_millis += 1234 + 777 * boot;                  // the first message comes at another time
    Bytes r = b.send(confirm);
    CHECK(r.size() == 2 + 21 && r[0] == 1);
    ids[boot] = getU32(&r[2 + 13]);
    g_millis += 5000;
    r = b.send(request(2, 0, 0x01, {'O', 'E', 'P', '?', 1, 1}));
    CHECK(r.size() == 2 + 21 && getU32(&r[2 + 13]) == ids[boot]);   // fixed for the boot
  }
  CHECK(ids[0] != ids[1]);
  Bulk set;
  set.ep.setBootId(0);                              // a sketch's own value, 0 included
  Bytes r = set.send(confirm);
  CHECK(r.size() == 2 + 21 && getU32(&r[2 + 13]) == 0);
  // the clock across the 32-bit micros() wrap
  const uint32_t saved = g_millis;
  g_millis = 4294967;                               // micros() = 4294967000, 295 ms before it wraps
  set.ep.poll();
  const uint64_t before = nowNs();
  g_millis += 200;
  set.ep.poll();
  g_millis += 200;                                  // micros() wrapped
  set.ep.poll();
  const uint64_t after = nowNs();
  CHECK(after > before && after - before == 400000000ull);
  g_millis += 1000;
  CHECK(nowNs() > after);                           // further on: still never back
  g_millis = saved;                                 // a new boot for the tests after this one
  g_micros_extender = MicrosExtender{};
}

// core §7.1 (C-20): confirm's values stay within the bounds whatever the sketch gave (max_frame >= 64, window >=
// max_frame, max_inflight >= 1). core §7.2 (C-39): the interface list is fixed for a boot - an add() after the first
// poll() is refused. core §7.5 (C-47): max_op_ms is 1 to max_op_ms_max (a static_assert in Oep.h; checked here too).
static void testConfirmBoundsAndFixedList() {
  MemStream stream;
  static uint8_t rx[1100], tx[1100];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {32, 16, 0}, Endpoint::kVendorBulk, 0};
  const Bytes m = request(1, 0, 0x01, {'O', 'E', 'P', '?', 1, 1});
  stream.send({uint8_t(m.size()), 0});
  stream.send(m);
  ep.poll();
  CHECK(stream.tx.size() == 2 + 5 + 21);
  if (stream.tx.size() == 2 + 5 + 21) {
    const uint8_t *c = stream.tx.data() + 7;
    CHECK(getU16(c + 6) == 64 && getU32(c + 8) == 64 && c[12] == 1);   // max_frame window max_inflight
  }
  FailsEarly late;
  CHECK(!ep.add(late));   // after poll(): the list a host has seen stays
  CHECK(Endpoint::kMaxOpMs >= 1 && Endpoint::kMaxOpMs <= reg::kLimitMaxOpMsMax);
}

// oep-if-plan §2.5 / core §4.3 (C-21): plan_apply's form over the whole request comes first (order 5: a role_assignment
// of another length, fn 0: malformed), then at the end of order 5 the fns named inside it (unknown_function), then an
// unknown critical tag and a fn with no plan role - oep.probe.plan itself - (unsupported, order 6). oep-if-plan: a probe
// none of whose interfaces has plan roles does not list oep.probe.plan (its fn is unknown); one with them lists it after
// the sketch's interfaces, with plan_roles in its describe and none in fn 0's.
static void testPlanApplyOrder() {
  Bulk b;
  PlanSink sink;
  b.ep.add(sink);   // fn 1; oep.probe.plan fn 2
  const uint16_t pf = b.ep.planFn();
  CHECK(pf == 2);
  b.send(request(1, 0, 0x10, openPayload(9, 3000)));
  CHECK(b.ep.planFn() == 2 && b.ep.interfaceAt(2) && strcmp(b.ep.interfaceAt(2)->name(), "oep.probe.plan") == 0);
  auto reject = [](const Bytes &r) { return r.size() >= 2 && r[0] == kResolutionRejected ? r[1] : 0xff; };
  CHECK(reject(b.send(request(2, pf, 0x01, {0x90, 5, 0, 0, 0, 0, 3, 0}, true, 9))) == kRejectMalformed);   // fn 0
  CHECK(reject(b.send(request(3, pf, 0x01, {0x90, 5, 0, 7, 0, 0, 3, 0, 0x90, 4, 0, 1, 0, 0, 3}, true, 9))) == kRejectMalformed);
  CHECK(reject(b.send(request(4, pf, 0x01, {0xB0, 0, 0, 0x90, 5, 0, 7, 0, 0, 3, 0}, true, 9))) == kRejectUnknownFunction);
  Bytes r = b.send(request(5, pf, 0x01, {0xB0, 0, 0, 0x90, 5, 0, 1, 0, 0, 3, 0}, true, 9));
  CHECK(r.size() == 3 && reject(r) == kRejectUnsupported && r[2] == 0xB0);
  r = b.send(request(6, pf, 0x01, {0x90, 5, 0, 2, 0, 0, 3, 0}, true, 9));   // oep.probe.plan has no plan role
  CHECK(r.size() == 3 && reject(r) == kRejectUnsupported && r[2] == 0x90);
  CHECK(reject(b.send(request(7, pf, 0x01, {0x90, 5, 0, 1, 0, 0, 3, 0}, true, 9))) == 0xff);   // fine
  CHECK(reject(b.send(request(8, pf, 0x02, {1, 1, 0}, true, 9))) == 0xff);
  CHECK(reject(b.send(request(9, pf, 0x02, {2, 1, 0}, true, 9))) == kRejectMalformed);   // n = 2 with one fn
  CHECK(reject(b.send(request(10, pf, 0x01, {0x90, 5, 0, 1, 0, 0, 3, 0}))) == kRejectSessionRequired);
  Bytes d = b.send(request(11, 0, reg::core::kOpDescribe, {0, 0, 0, 0}));
  const Bytes old_roles = {0x4B, 4, 0};
  CHECK(!d.empty() && d[0] == kResolutionCompleted && std::search(d.begin(), d.end(), old_roles.begin(), old_roles.end()) == d.end());
  d = b.send(request(12, 0, reg::core::kOpDescribe, {uint8_t(pf), 0, 0, 0}));
  const Bytes roles = {reg::probe_plan::kTlvDescribePlanRoles, 4, 0};
  CHECK(std::search(d.begin(), d.end(), roles.begin(), roles.end()) != d.end());
  Bulk none;   // no interface with plan roles: no oep.probe.plan
  FailsEarly other;
  none.ep.add(other);
  CHECK(none.ep.planFn() == 0);
  CHECK(reject(none.send(request(1, 2, 0x01, {0x90, 5, 0, 1, 0, 0, 3, 0}))) == kRejectUnknownFunction);
  r = none.send(request(2, 0, reg::core::kOpList, {0, 0}));
  CHECK(r.size() >= 5 && r[0] == kResolutionCompleted && getU16(&r[2]) == 1);   // fn 1 only
}

// core §1.2 / §4.3 order 1: an op the interface does not offer (one it does not define, or an optional one it does not
// declare) is unknown_operation before the session is looked at - not session_required.
class OffersTwo final : public Interface {
 public:
  const char *name() const override { return "io.github.test.offers"; }
  uint16_t instance() const override { return 0; }
  bool offers(uint8_t op) const override { return op == 1 || op == 2; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return completed(); }
};
static void testUnknownOperationFirst() {
  Bulk b;
  OffersTwo two;
  b.ep.add(two);   // fn 1
  Bytes r = b.send(request(1, 1, 3, {}));
  CHECK(r.size() == 2 && r[0] == kResolutionRejected && r[1] == kRejectUnknownOperation);
  r = b.send(request(2, 1, 1, {}));   // an op it offers, no session: session_required
  CHECK(r.size() == 2 && r[0] == kResolutionRejected && r[1] == kRejectSessionRequired);
  b.send(request(3, 0, 0x10, openPayload(4, 3000)));
  r = b.send(request(4, 1, 0x7F, {}, true, 4));
  CHECK(r.size() == 2 && r[0] == kResolutionRejected && r[1] == kRejectUnknownOperation);
  r = b.send(request(5, 1, 2, {}, true, 4));
  CHECK(r.size() == 2 && r[0] == kResolutionCompleted);
}

// core §2.4 (C-36): the probe discards, unanswered, a message whose role is not a request role and a request shorter
// than its header (6 bytes, 10 with session_id); the next good request is answered as usual.
static void testShortAndWrongRoleDiscarded() {
  Bulk b;
  const Bytes confirm = request(1, 0, 0x01, {'O', 'E', 'P', '?', 1, 1});
  for (uint8_t role : {0x02, 0x05, 0x06, 0x00, 0x7f}) {
    Bytes m = confirm;
    m[0] = role;
    CHECK(b.send(m).empty() && b.stream.tx.empty());
  }
  CHECK(b.send({0x01, 1, 0, 0, 0}).empty() && b.stream.tx.empty());               // 5 bytes
  CHECK(b.send({0x81, 1, 0, 0, 0, 0x12, 7, 0, 0}).empty() && b.stream.tx.empty());   // 9 bytes with session
  const Bytes r = b.send(confirm);
  CHECK(r.size() == 2 + 21 && r[0] == kResolutionCompleted);
}

int main() {
  testShortAndWrongRoleDiscarded();
  testUnknownOperationFirst();
  testPlanApplyOrder();
  testConfirmBoundsAndFixedList();
  testBootIdAndClock();
  testBlockLength();
  testPortSpeed();
  testDisabledChannel();
  testTlvForm();
  testSessionTable();
  testNoIgnoredList();
  testSubscriptions();
  testResourceNumbers();
  testGroupSecondRun();
  testPlanCapacity();
  testPlanNotShared();
  testPlanUnsupportedTag();
  testSettingsPlanStays();
  testLastMarkMissing();
  testReader();
  testEndpointSerialPort();
  printf("TEST done %d/%d\n", checks - failures, checks);
  return failures ? 1 : 0;
}
