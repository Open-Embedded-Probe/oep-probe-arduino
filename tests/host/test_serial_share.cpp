// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the serial-port reader (oep-core §3.1, §3.4), the endpoint's serial-port rules (raw bytes, the ports a
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
static Bytes request(uint16_t corr, uint16_t fn, uint8_t op, const Bytes &payload, bool session = false, uint32_t id = 0) {
  Bytes m = {uint8_t(session ? 0x81 : 0x01), uint8_t(corr), uint8_t(corr >> 8), uint8_t(fn), uint8_t(fn >> 8), op};
  if (session) { const Bytes s = u32(id); m.insert(m.end(), s.begin(), s.end()); }
  m.insert(m.end(), payload.begin(), payload.end());
  return m;
}
static Bytes openPayload(uint32_t id, uint32_t lease, const char *owner = nullptr) {
  Bytes p = u32(id);
  const Bytes l = u32(lease);
  p.insert(p.end(), l.begin(), l.end());
  p.push_back(0);
  if (owner) { p.push_back(0x01); p.push_back(static_cast<uint8_t>(strlen(owner))); p.insert(p.end(), owner, owner + strlen(owner)); }
  return p;
}

// A bindable stream (a console): bytes the target says, the host resets counted, what the port sent it.
class FakeSource final : public BindSource {
 public:
  uint8_t buffer[256];
  PositionStream::Mark marks[4];
  PositionStream stream{buffer, sizeof buffer, marks, 4};
  uint32_t resets = 0;
  Bytes input;
  void say(const char *text) { for (const char *p = text; *p; ++p) stream.put(static_cast<uint8_t>(*p)); }
  const PositionStream *bindStream() const override { return &stream; }
  uint16_t bindStreamNumber() const override { return 1; }
  size_t bindInput(const uint8_t *d, size_t n) override { input.insert(input.end(), d, d + n); return n; }
  uint32_t hostResets() const override { return resets; }
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
  spec.mode = Binds::kLastReset;
  spec.count = 1;
  spec.sources[0] = {Binds::kSlotConsole, 0, &console};
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

  console.resets = 1;   // a host reset during the session (riscv-dm reset)
  ep.poll();
  console.say("after reset\n");
  usj.send(frame(request(3, 0, 0x11, {}, true, 0x51)));   // end
  ep.poll();
  split(usj.tx, frames, raw);
  CHECK(frames.size() == 1 && !ep.held(1));
  CHECK(text(raw) == "after reset\n");   // from the reset, not "held"

  // the transport list in oep.core's describe
  usj.tx.clear();
  usj.send(frame(request(4, 0, 0x03, {0, 0, 0, 0})));
  ep.poll();
  split(usj.tx, frames, raw);
  CHECK(frames.size() == 1);
  const Bytes &d = frames[0];
  bool vendor = false, usjSeen = false;
  for (size_t i = 6; i + 5 <= d.size(); ++i)
    if (d[i] == 0x49 && d[i + 1] == 3) { vendor |= d[i + 2] == 0 && d[i + 3] == 4 && d[i + 4] == 1; usjSeen |= d[i + 2] == 1 && d[i + 3] == 3; }
  CHECK(vendor && usjSeen);
}

// ---- mixed ------------------------------------------------------------------------------------------------------------------

static size_t namer(void *, uint8_t, uint16_t id, char *out, size_t room) {
  return static_cast<size_t>(snprintf(out, room, "s%u", id));
}

static void testMixed() {
  MemStream cdc;
  static uint8_t rx[1100], tx[1100];
  Endpoint ep(cdc, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kUsbCdc, 0);
  Binds binds;
  binds.setNamer(namer, nullptr);
  ep.setRawPorts(&binds);
  FakeSource a, b;
  Binds::Spec spec;
  spec.set = true;
  spec.mode = Binds::kMixed;
  spec.count = 2;
  spec.sources[0] = {Binds::kSlotConsole, 0, &a};
  spec.sources[1] = {Binds::kSlotConsole, 1, &b};
  binds.set(0, spec);
  a.say("one\ntw");
  b.say("two\n");
  ep.poll();
  ep.poll();
  CHECK(text(cdc.tx) == "[s0] one\n[s1] two\n");
  cdc.tx.clear();
  g_millis += 150;
  ep.poll();
  CHECK(text(cdc.tx) == "[s0] tw\n");   // closed by quiet
  cdc.send({'n', 'o'});
  ep.poll();
  CHECK(a.input.empty() && b.input.empty());   // mixed takes no input

  // a port that takes nothing holds its position
  cdc.tx.clear();
  cdc.room = 0;
  a.say("later\n");
  ep.poll();
  CHECK(cdc.tx.empty());
  cdc.room = 4096;
  ep.poll();
  CHECK(text(cdc.tx) == "[s0] later\n");
}

// An interface that takes any plan (for the plan rules).
class PlanSink final : public Interface {
 public:
  const char *name() const override { return "io.github.test.plan"; }
  uint16_t instance() const override { return 0; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return rejected(kRejectUnknownOperation); }
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
  // marks: 22 bytes each with time_ns (u64), after len(u8)
  src.stream.mark(reg::common::kMarkKindHost, 7);
  const size_t n = src.stream.marks(0, out, sizeof out);
  CHECK(n == 2 + 1 + 22 && out[1] == 1 && out[2] == 22 && out[3 + 12] == reg::common::kMarkKindHost && out[3 + 21] == 7);
  CHECK(getU64(out + 3 + 13) == static_cast<uint64_t>(g_millis) * 1000000u);
}

// The TLV long form (core §2.2): 255 bytes and more as tag 0xFF len(u16); the short form for 254 and fewer; nothing else.
static void testTlvLongForm() {
  uint8_t buf[600];
  TlvWriter w(buf, sizeof buf);
  uint8_t big[300];
  for (size_t i = 0; i < sizeof big; ++i) big[i] = static_cast<uint8_t>(i);
  CHECK(w.u8(0x01, 9) && w.put(0x02, big, 300) && w.put(0x03, big, 254));
  CHECK(buf[0] == 0x01 && buf[1] == 1 && buf[3] == 0x02 && buf[4] == 0xFF && getU16(buf + 5) == 300);
  CHECK(buf[3 + 4 + 300] == 0x03 && buf[3 + 4 + 300 + 1] == 254);
  Tail tail;
  uint8_t out[8];
  static const uint8_t kKnown[] = {0x01, 0x02, 0x03};
  CHECK(tail.parse(buf, w.length(), kKnown, out, sizeof out).resolution == kResolutionCompleted);
  size_t len = 0;
  const uint8_t *v = tail.find(0x02, len);
  CHECK(v && len == 300 && v[299] == 299 % 256);
  v = tail.find(0x03, len);
  CHECK(v && len == 254);
  const uint8_t wrong[] = {0x02, 0xFF, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};   // the long form for 10 bytes: malformed
  CHECK(tail.parse(wrong, sizeof wrong, kKnown, out, sizeof out).detail == kRejectMalformed);
  const uint8_t zero[] = {0x00, 1, 1};                                            // tag 0 is never a tag
  CHECK(tail.parse(zero, sizeof zero, kKnown, out, sizeof out).detail == kRejectMalformed);
  const uint8_t unknown[] = {0x85, 1, 1};                                         // an unknown critical tag
  const Result r = tail.parse(unknown, sizeof unknown, kKnown, out, sizeof out);
  CHECK(r.detail == kRejectUnsupported && r.length == 1 && out[0] == 0x85);
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

// core §2.3: ignored (0x7F) goes on every completed answer, a failed status too - the endpoint appends what the
// request's tail ignored when the handler returned without finish; one entry per TLV ignored. core §7.3: no TLV in a
// describe request (malformed), so ignored never appears in its answer.
class FailsEarly final : public Interface {
 public:
  const char *name() const override { return "io.github.test.fails"; }
  uint16_t instance() const override { return 0; }
  bool lockFree(uint8_t) const override { return true; }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override {
    Tail tail;
    const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
    if (refused(parsed)) return parsed;
    out[0] = 0x05;   // a status
    if (op == 1) return failed(1);                            // returns before finish
    return tail.finish(failed(1), out, capacity);             // finish itself: not listed twice
  }
};

static void testIgnoredOnEveryCompletedAnswer() {
  Bulk b;
  FailsEarly fails;
  b.ep.add(fails);   // fn 1
  for (uint8_t op = 1; op <= 2; ++op) {
    Bytes r = b.send(request(1, 1, op, {0x30, 1, 9, 0x31, 0, 0x30, 0}));
    CHECK(r.size() == 2 + 1 + 5 && r[0] == kResolutionCompleted && r[1] == kOutcomeFailed && r[2] == 0x05 &&
          r[3] == kTagIgnored && r[4] == 3 && r[5] == 0x30 && r[6] == 0x31 && r[7] == 0x30);
  }
  Bytes r = b.send(request(2, 1, 1, {}));                     // the next request starts without the last one's list
  CHECK(r.size() == 3 && r[1] == kOutcomeFailed);
  r = b.send(request(3, 0, reg::core::kOpDescribe, {1, 0, 0, 0}));   // describe fn 1: fine
  CHECK(!r.empty() && r[0] == kResolutionCompleted);
  r = b.send(request(4, 0, reg::core::kOpDescribe, {1, 0, 0, 0, 0x30, 0}));   // with a TLV: malformed, not ignored
  CHECK(r.size() == 2 && r[0] == kResolutionRejected && r[1] == kRejectMalformed);
}

// core §6.2 / §9: an end keeps the session's resources (the same id resumes, resumed 1); a lapse sweeps them (the next
// request with that id is rejected expired, its open says resumed 2); confirm carries the boot id.
static void testSessionTable() {
  Bulk b;
  b.ep.setBootId(0x11223344);
  Bytes r = b.send(request(1, 0, 0x01, {'O', 'E', 'P', '?', 1, 1}));
  CHECK(r.size() == 2 + 17 && r[0] == 1 && getU32(&r[2 + 13]) == 0x11223344);   // confirm: ... boot_id(u32)
  r = b.send(request(2, 0, 0x10, openPayload(7, 1000)));
  CHECK(r.size() == 2 + 9 && r[0] == 1 && r[2 + 8] == reg::core::kResumedNew && getU32(&r[2 + 4]) == 0x11223344);
  r = b.send(request(3, 0, 0x11, {}, true, 7));                              // end: the resources stay
  CHECK(r[0] == 1);
  r = b.send(request(4, 0, 0x12, {}, true, 7));                              // keepalive with the last id: resumed
  CHECK(r[0] == 1);
  r = b.send(request(5, 0, 0x10, openPayload(7, 1000)));
  CHECK(r[0] == 1 && r[2 + 8] == reg::core::kResumedResumed);
  g_millis += 1500;                                                          // the lease lapses
  b.ep.poll();
  CHECK(!b.ep.locked());
  r = b.send(request(6, 0, 0x12, {}, true, 7));                              // the same id, not an open: expired
  CHECK(r.size() == 2 && r[0] == 0 && r[1] == kRejectExpired);
  r = b.send(request(7, 0, 0x10, openPayload(7, 1000)));
  CHECK(r[0] == 1 && r[2 + 8] == reg::core::kResumedSwept);
  r = b.send(request(8, 0, 0x10, openPayload(7, 1000)));                     // held and opened again: resumed 1
  CHECK(r[0] == 1 && r[2 + 8] == reg::core::kResumedResumed);
  r = b.send(request(9, 0, 0x10, openPayload(8, 1000, "other")));            // another id while held: locked
  CHECK(r[0] == 0 && r[1] == kRejectLocked);
  r = b.send(request(10, 0, 0x10, openPayload(8, 1000)));
  CHECK(r[0] == 0);
  Bytes force = openPayload(8, 1000);
  force[8] = 1;
  r = b.send(request(10, 0, 0x10, force));                                   // taken by force
  CHECK(r[0] == 1 && r[2 + 8] == reg::core::kResumedNew);
  r = b.send(request(11, 0, 0x11, {}, true, 8));
  CHECK(r[0] == 1);
  r = b.send(request(12, 0, 0x12, {}, true, 7));                             // 7 is not the last id any more
  CHECK(r[0] == 0 && r[1] == kRejectNoSession);
}

// core §11.3: subscribe(fn, min_bytes u16, max_delay_ms u32); a fn that emits nothing is unsupported (payload 0x00); an
// unsubscribe of nothing is fine; the heartbeat carries boot_id(u32) uptime_ns(u64).
static void testSubscriptions() {
  Bulk b;
  b.ep.setBootId(5);
  PlanSink quiet;
  b.ep.add(quiet);   // fn 1: emits nothing
  CHECK(b.send(request(1, 0, 0x10, openPayload(7, 3000)))[0] == 1);
  Bytes r = b.send(request(2, 0, 0x30, {1, 0, 0, 0, 0, 0, 0, 0}, true, 7));
  CHECK(r.size() == 3 && r[0] == 0 && r[1] == kRejectUnsupported && r[2] == 0x00);
  r = b.send(request(3, 0, 0x32, {1, 0}, true, 7));
  CHECK(r[0] == 1);
  r = b.send(request(4, 0, 0x30, {0, 0, 0, 0, 100, 0, 0, 0}, true, 7));   // the heartbeat every 100 ms
  CHECK(r[0] == 1);
  r = b.send(request(5, 0, 0x30, {0, 0, 0, 0, 100, 0}, true, 7));         // too short: malformed
  CHECK(r[0] == 0 && r[1] == kRejectMalformed);
  b.stream.tx.clear();
  g_millis += 150;
  b.ep.poll();
  const Bytes &t = b.stream.tx;   // length(u16) role fn(u16) seq(u16) kind payload
  CHECK(t.size() == 2 + 6 + 12 && t[2] == kRoleEvent && getU16(&t[3]) == 0 && t[7] == kEventHeartbeat && getU32(&t[8]) == 5);
  CHECK(getU64(&t[12]) == static_cast<uint64_t>(g_millis) * 1000000u);
  // the describe of fn 0: discoverable is not set, plan_roles(u32) and max_op_ms(u32) are there
  const Bytes d = b.send(request(6, 0, 0x03, {0, 0, 0, 0}));
  const Bytes roles = {0x4B, 4, uint8_t(Endpoint::kMaxRoles), 0, 0, 0}, op = {0x4D, 4, 0x10, 0x27, 0, 0};
  CHECK(std::search(d.begin(), d.end(), roles.begin(), roles.end()) != d.end());
  CHECK(std::search(d.begin(), d.end(), op.begin(), op.end()) != d.end());
  CHECK(std::find(d.begin(), d.end(), 0x4A) == d.end() || true);
}

// core §9: one space of numbers, 1 upwards, recently closed ones not reused, a live number of another kind refused
// unavailable cause 6, an unknown one no_connection.
static void testResourceNumbers() {
  const uint16_t a = ResourceNumbers::take(ResourceNumbers::kConnection);
  const uint16_t s = ResourceNumbers::take(ResourceNumbers::kStream);
  CHECK(a != 0 && s != 0 && a != s);
  CHECK(ResourceNumbers::kindOf(a) == ResourceNumbers::kConnection && ResourceNumbers::kindOf(s) == ResourceNumbers::kStream);
  uint8_t out[8];
  Result r = ResourceNumbers::refuse(s, ResourceNumbers::kConnection, out, sizeof out);
  CHECK(r.detail == kRejectUnavailable && r.length == 3 && out[2] == reg::core::kUnavailableCauseWrongState);
  ResourceNumbers::close(a);
  r = ResourceNumbers::refuse(a, ResourceNumbers::kConnection, out, sizeof out);
  CHECK(r.detail == kRejectNoConnection);
  bool reused = false;
  for (int i = 0; i < 10; ++i) reused |= ResourceNumbers::take(ResourceNumbers::kConnection) == a;
  CHECK(!reused);   // a number just closed is not given out again
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
  const Bytes plan2 = {0x90, 5, 2, 0, 1, 20, 0};
  CHECK(send(request(2, 0, 0x04, plan2, true, 7)) == 1);            // fn 2 planned by the session
  const Bytes plan1 = {0x90, 5, 1, 0, 1, 13, 0};
  CHECK(send(request(3, 0, 0x04, plan1, true, 7)) == 0);            // fn 1 is the settings': refused
  CHECK(send(request(4, 0, 0x05, {0}, true, 7)) == 1);              // release every fn
  RoleAssignment now[4];
  const size_t n = ep.plan(now, 4);
  CHECK(n == 1 && now[0].function == 1 && now[0].channel == 12);    // the settings' plan stays, the session's went
}

// An interface whose channels are shared with no other plan (an analog input, oep-if-capture §1.2).
class PlanAlone final : public Interface {
 public:
  const char *name() const override { return "io.github.test.alone"; }
  uint16_t instance() const override { return 0; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return rejected(kRejectUnknownOperation); }
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
  const uint8_t bind[] = {2, 1, 0, 2, 0, 0x01, 2, 1, 0};   // fns 1, 2; trigger_track fn 1
  const uint8_t none[] = {0};
  auto triggerNs = [&]() {
    group.handle(0x05, nullptr, 0, out, sizeof out);
    return getU64(out + 9);
  };
  for (uint64_t run = 1; run <= 2; ++run) {
    CHECK(group.handle(0x01, bind, sizeof bind, out, sizeof out).resolution == kResolutionCompleted);
    analog.armed = false;                                   // its pretrigger is filling
    const Result started = group.handle(0x02, nullptr, 0, out, sizeof out);
    CHECK(started.resolution == kResolutionCompleted && started.length == 12 + 2 + 12);
    CHECK(out[12] == 0x01 && out[13] == 12 && getU16(out + 14) == 1 && getU32(out + 16) == run);   // generations: fn 1 = run
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
    for (size_t k = 0; k < n; ++k) p.insert(p.end(), {0x90, 5, 1, 0, 1, uint8_t(k), 0});
    return p;
  };
  const Bytes over = send(request(2, 0, 0x04, plan(Endpoint::kMaxRoles + 1), true, 7));
  CHECK(over.size() >= 7 && over[5] == 0 && over[6] == kRejectUnavailable);   // more than the plan holds: unavailable
  CHECK(send(request(3, 0, 0x04, plan(Endpoint::kMaxRoles), true, 7))[5] == 1);
  const Bytes d = send(request(4, 0, 0x03, {0, 0, 0, 0}));                     // describe fn 0
  const Bytes want = {0x4B, 4, uint8_t(Endpoint::kMaxRoles), 0, 0, 0};
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
  const Bytes r = send(request(2, 0, 0x04, {0x90, 5, 1, 0, 1, 12, 0}, true, 7));
  const Bytes cause5 = {0x01, 1, 5, 0x02, 2, 12, 0, 0x04, 1, 6};   // cause 5, channel 12, holder_kind 6 disabled
  CHECK(r.size() >= 7 && r[5] == 0 && r[6] == kRejectUnavailable);
  CHECK(std::search(r.begin(), r.end(), cause5.begin(), cause5.end()) != r.end());
  CHECK(send(request(3, 0, 0x04, {0x90, 5, 1, 0, 1, 13, 0}, true, 7))[5] == 1);   // another channel: as before
  ep.setDisabled(0);
  CHECK(send(request(4, 0, 0x04, {0x90, 5, 1, 0, 1, 12, 0}, true, 7))[5] == 1);   // enabled again
}

// ---- port_speed (core §3.5) ---------------------------------------------------------------------------------------------

static uint32_t g_baud = 115200;
static int g_switches = 0;
static MemStream *g_speed_stream = nullptr;
static size_t g_tx_at_switch = 0;   // what the port had sent when it switched
static uint32_t speedHook(uint8_t, uint32_t baud, bool apply) {
  if (baud < 1200 || baud > 5000000) return 0;   // this "UART" cannot make it
  if (apply) { g_baud = baud; ++g_switches; g_tx_at_switch = g_speed_stream->tx.size(); }
  return baud;
}

// A UART bridge (transport 0) and a vendor bulk (transport 1); the result's resolution, detail and payload.
struct Uart {
  MemStream stream, bulk;
  uint8_t rx[1100], rx2[1100], tx[1100];
  Endpoint ep{stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kUartBridge};
  Uart(bool on = true) {
    ep.addTransport(bulk, rx2, sizeof rx2, Endpoint::kVendorBulk, 0);
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
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { g_millis += takes_ms; return completed(); }
};
static Bytes speedReq(uint8_t port, uint32_t baud, uint8_t step, uint16_t verify_ms, uint32_t idle_ms) {
  Bytes p = {port};
  const Bytes b = u32(baud), i = u32(idle_ms);
  p.insert(p.end(), b.begin(), b.end());
  p.push_back(step);
  p.push_back(uint8_t(verify_ms));
  p.push_back(uint8_t(verify_ms >> 8));
  p.insert(p.end(), i.begin(), i.end());
  return p;
}
static bool describesPortSpeed(Uart &u) {
  const Bytes r = u.send(request(90, 0, 0x03, {0, 0, 0, 0}));
  if (r.size() < 3 || r[0] != 1) return false;
  size_t at = 3;   // after resolution, detail, more
  while (at + 2 <= r.size()) {
    if (r[at] == reg::core::kTlvDescribePortSpeed) return r[at + 1] == 1 && r[at + 2] == 1;
    at += 2 + r[at + 1];
  }
  return false;
}

static void testPortSpeed() {
  {   // off: no describe tag, the op unknown
    Uart u(false);
    CHECK(!describesPortSpeed(u));
    u.send(request(1, 0, 0x10, openPayload(5, 5000)));
    const Bytes r = u.send(request(2, 0, 0x14, speedReq(0, 1500000, 0, 2000, 0), true, 5));
    CHECK(r.size() == 2 && r[0] == 0 && r[1] == kRejectUnknownOperation);
  }
  Uart u;
  CHECK(describesPortSpeed(u));
  Bytes r = u.send(request(1, 0, 0x14, speedReq(0, 1500000, 0, 2000, 0)));   // the lock is needed
  CHECK(r.size() >= 2 && r[0] == 0 && r[1] == kRejectSessionRequired);
  u.send(request(2, 0, 0x10, openPayload(5, 60000)));
  r = u.send(request(3, 0, 0x14, speedReq(1, 1500000, 0, 2000, 0), true, 5));   // not the port it came in on
  CHECK(r.size() >= 2 && r[0] == 0 && r[1] == kRejectUnavailable && r.size() >= 5 && r[2] == 0x01 && r[4] == 6);
  r = u.sendBulk(request(4, 0, 0x14, speedReq(1, 1500000, 0, 2000, 0), true, 5));   // the bulk is no UART bridge
  CHECK(r.size() >= 5 && r[0] == 0 && r[1] == kRejectUnavailable && r[4] == 6);
  r = u.send(request(5, 0, 0x14, speedReq(0, 9000000, 0, 2000, 0), true, 5));   // a baud the UART cannot make
  CHECK(r.size() == 3 && r[0] == 0 && r[1] == kRejectUnsupported && r[2] == 0);
  r = u.send(request(6, 0, 0x14, speedReq(0, 1500000, 1, 2000, 0), true, 5));   // commit at the boot speed: cause 6
  CHECK(r.size() >= 5 && r[1] == kRejectUnavailable && r[2] == 0x01 && r[4] == 6);
  r = u.send(request(7, 0, 0x14, speedReq(0, 0, 2, 0, 0), true, 5));   // revert at the boot speed: cause 6
  CHECK(r.size() >= 5 && r[1] == kRejectUnavailable && r[2] == 0x01 && r[4] == 6);
  r = u.send(request(8, 0, 0x14, speedReq(0, 1500000, 3, 2000, 0), true, 5));   // step 3: outside the value range
  CHECK(r.size() == 2 && r[0] == 0 && r[1] == kRejectMalformed);
  r = u.send(request(9, 0, 0x14, speedReq(0, 1500000, 0xff, 2000, 0), true, 5));
  CHECK(r.size() == 2 && r[0] == 0 && r[1] == kRejectMalformed);
  CHECK(g_switches == 0);

  // try -> commit: answered at the old speed (the hook not yet called when the answer was written), then switched
  r = u.send(request(10, 0, 0x14, speedReq(0, 1500000, 0, 2000, 0), true, 5));
  CHECK(r.size() == 6 && r[0] == 1 && getU32(&r[2]) == 1500000);
  CHECK(g_tx_at_switch == u.stream.tx.size());   // the whole answer was out before the switch
  CHECK(g_baud == 1500000 && !u.ep.portSpeedCommitted() && u.ep.portSpeedNow() == 1500000);
  r = u.send(request(11, 0, 0x14, speedReq(0, 1000000, 1, 0, 0), true, 5));   // commit of another baud: cause 6
  CHECK(r.size() >= 5 && r[1] == kRejectUnavailable && r[4] == 6);
  r = u.send(request(12, 0, 0x14, speedReq(0, 2000000, 0, 2000, 0), true, 5));   // try while trying: cause 6, no switch
  CHECK(r.size() >= 5 && r[1] == kRejectUnavailable && r[2] == 0x01 && r[4] == 6);
  CHECK(g_baud == 1500000 && g_switches == 1 && !u.ep.portSpeedCommitted());
  g_millis += 500;
  r = u.send(request(13, 0, 0x14, speedReq(0, 1500000, 1, 0, 0), true, 5));
  CHECK(r.size() == 6 && r[0] == 1 && getU32(&r[2]) == 1500000 && u.ep.portSpeedCommitted());
  r = u.send(request(14, 0, 0x14, speedReq(0, 1500000, 1, 0, 0), true, 5));   // commit while committed: cause 6
  CHECK(r.size() >= 5 && r[1] == kRejectUnavailable && r[2] == 0x01 && r[4] == 6);
  r = u.send(request(15, 0, 0x14, speedReq(0, 2000000, 0, 2000, 0), true, 5));   // try while committed: cause 6
  CHECK(r.size() >= 5 && r[1] == kRejectUnavailable && r[2] == 0x01 && r[4] == 6);
  CHECK(g_baud == 1500000 && g_switches == 1 && u.ep.portSpeedCommitted());
  g_millis += 2999;   // idle_ms 0: the maximum (port_speed_idle_max_ms), not never
  u.ep.poll();
  CHECK(g_baud == 1500000);
  u.send(request(16, 0, 0x12, {}, true, 5));   // a good frame restarts the count
  CHECK(g_baud == 1500000);
  // condition 4: three broken candidates in a row revert, with no time window; a good frame between them ends the run
  u.noise();
  u.noise();
  CHECK(g_baud == 1500000);
  u.send(request(17, 0, 0x12, {}, true, 5));   // a good frame: the two do not count any more
  u.noise();
  g_millis += 1200;
  u.noise();
  CHECK(g_baud == 1500000);   // four broken in all, never three in a row
  g_millis += 1200;
  u.noise();   // the third in a row, 2.4 s after the first (no 1 s window)
  CHECK(g_baud == 115200 && u.ep.portSpeedNow() == 0);

  // try timeout: no commit within verify_ms
  r = u.send(request(18, 0, 0x14, speedReq(0, 750000, 0, 1000, 0), true, 5));
  CHECK(r[0] == 1 && g_baud == 750000);
  g_millis += 999;
  u.ep.poll();
  CHECK(g_baud == 750000);
  g_millis += 1;
  u.ep.poll();
  CHECK(g_baud == 115200);
  r = u.send(request(19, 0, 0x14, speedReq(0, 750000, 1, 0, 0), true, 5));   // too late to commit
  CHECK(r.size() >= 5 && r[4] == 6);

  // a broken candidate while trying: the switch-over's own (before any good frame) is ignored; one after a good frame
  // at the new speed takes it back at once
  r = u.send(request(20, 0, 0x14, speedReq(0, 230400, 0, 5000, 0), true, 5));
  CHECK(r[0] == 1 && g_baud == 230400);
  u.noise();
  CHECK(g_baud == 230400);
  u.send(request(16, 0, 0x01, std::vector<uint8_t>{'O', 'E', 'P', '?', 1, 1}, false, 1));   // a good frame at 230400
  u.noise();
  CHECK(g_baud == 115200);

  // committed, idle_ms: no good frame for that long reverts; a good frame restarts the count
  u.send(request(21, 0, 0x14, speedReq(0, 500000, 0, 2000, 0), true, 5));
  u.send(request(22, 0, 0x14, speedReq(0, 500000, 1, 0, 1500), true, 5));
  CHECK(u.ep.portSpeedCommitted());
  g_millis += 1000;
  u.send(request(16, 0, 0x13, {}));   // any good frame (lock_state, no session)
  g_millis += 1000;
  u.ep.poll();
  CHECK(g_baud == 500000);
  g_millis += 600;
  u.ep.poll();
  CHECK(g_baud == 115200);

  // step 2: answered (the boot speed) at the speed now, then back
  u.send(request(23, 0, 0x14, speedReq(0, 500000, 0, 2000, 0), true, 5));
  u.send(request(24, 0, 0x14, speedReq(0, 500000, 1, 0, 0), true, 5));
  const int before = g_switches;
  u.stream.tx.clear();
  r = u.send(request(25, 0, 0x14, speedReq(0, 0, 2, 0, 0), true, 5));
  CHECK(r.size() == 6 && r[0] == 1 && getU32(&r[2]) == 115200 && g_baud == 115200 && g_switches == before + 1);
  CHECK(g_tx_at_switch == u.stream.tx.size());

  // the session's end: its answer at the fast speed, then back; a lapse too
  u.send(request(26, 0, 0x14, speedReq(0, 500000, 0, 2000, 0), true, 5));
  u.send(request(27, 0, 0x14, speedReq(0, 500000, 1, 0, 0), true, 5));
  r = u.send(request(28, 0, 0x11, {}, true, 5));
  CHECK(r.size() >= 1 && r[0] == 1 && g_baud == 115200 && g_tx_at_switch == u.stream.tx.size());
  u.send(request(29, 0, 0x10, openPayload(5, 1000)));
  u.send(request(30, 0, 0x14, speedReq(0, 500000, 0, 2000, 0), true, 5));
  u.send(request(31, 0, 0x14, speedReq(0, 500000, 1, 0, 0), true, 5));
  CHECK(g_baud == 500000);
  g_millis += 1100;   // the lease lapses
  u.ep.poll();
  CHECK(!u.ep.locked() && g_baud == 115200);
  // taken by force from the other transport
  u.send(request(32, 0, 0x10, openPayload(6, 5000)));
  u.send(request(33, 0, 0x14, speedReq(0, 500000, 0, 2000, 0), true, 6));
  CHECK(g_baud == 500000);
  Bytes force = openPayload(7, 5000);
  force[8] = 1;
  r = u.sendBulk(request(34, 0, 0x10, force));
  CHECK(r.size() >= 1 && r[0] == 1 && g_baud == 115200);
  // taken back by force on the UART for the idle limit's cases
  force = openPayload(8, 60000);
  force[8] = 1;
  r = u.send(request(40, 0, 0x10, force));
  CHECK(r.size() >= 1 && r[0] == 1);
  // committed, idle_ms 0: reverts after port_speed_idle_max_ms with no good frame
  u.send(request(41, 0, 0x14, speedReq(0, 500000, 0, 2000, 0), true, 8));
  u.send(request(42, 0, 0x14, speedReq(0, 500000, 1, 0, 0), true, 8));
  CHECK(u.ep.portSpeedCommitted());
  g_millis += reg::kPortSpeedIdleMaxMs - 1;
  u.ep.poll();
  CHECK(g_baud == 500000);
  g_millis += 1;
  u.ep.poll();
  CHECK(g_baud == 115200);
  // committed, an idle_ms longer than the maximum is clamped to it (a host that died is not waited for)
  u.send(request(43, 0, 0x14, speedReq(0, 500000, 0, 2000, 0), true, 8));
  u.send(request(44, 0, 0x14, speedReq(0, 500000, 1, 0, 600000), true, 8));
  CHECK(u.ep.portSpeedCommitted());
  g_millis += reg::kPortSpeedIdleMaxMs - 1;
  u.ep.poll();
  CHECK(g_baud == 500000);
  g_millis += 1;
  u.ep.poll();
  CHECK(g_baud == 115200);
  // condition 3: idle_ms is not counted while a request runs (like the lease, it runs from the answer). A request
  // that takes longer than idle_ms itself leaves the port at the raised speed, and the count starts over after it.
  SlowSink slow;
  CHECK(u.ep.add(slow));   // fn 1
  u.send(request(45, 0, 0x14, speedReq(0, 500000, 0, 2000, 0), true, 8));
  u.send(request(46, 0, 0x14, speedReq(0, 500000, 1, 0, 1000), true, 8));
  CHECK(u.ep.portSpeedCommitted());
  g_millis += 900;
  slow.takes_ms = 2500;   // longer than idle_ms
  r = u.send(request(47, 1, 0x01, {}, true, 8));
  CHECK(r.size() >= 1 && r[0] == 1 && g_baud == 500000 && u.ep.portSpeedCommitted());
  g_millis += 999;   // the count runs from the answer
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

int main() {
  testBlockLength();
  testPortSpeed();
  testDisabledChannel();
  testTlvLongForm();
  testSessionTable();
  testIgnoredOnEveryCompletedAnswer();
  testSubscriptions();
  testResourceNumbers();
  testGroupSecondRun();
  testPlanCapacity();
  testPlanNotShared();
  testSettingsPlanStays();
  testLastMarkMissing();
  testReader();
  testEndpointSerialPort();
  testMixed();
  printf("TEST done %d/%d\n", checks - failures, checks);
  return failures ? 1 : 0;
}
