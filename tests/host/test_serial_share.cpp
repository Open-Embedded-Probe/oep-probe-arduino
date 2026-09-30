// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the serial-port reader (oep-core §3.1, §3.4), the endpoint's serial-port rules (raw bytes, the ports a
#include <algorithm>
// session holds, the resume from its last host reset), owner and the transport list, and the binds' modes.
#include <stdio.h>
#include <string>
#include <vector>

#include "OepFrame.h"
#include "OepBind.h"
#include "OepEndpoint.h"
#include "OepCaptureGroup.h"

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

static void testLastMarkMissing() {
  FakeSource src;
  src.say("old output");
  const uint8_t req[11] = {3, 1, 0, 0, 0, 0, 0, 0, 0, 64, 0};   // from last mark, kind reset, max 64
  uint8_t out[80];
  const Result r = src.stream.read(req, out, sizeof out, 64);
  CHECK(r.length == 9 && getU64(out) == 10);                      // no reset mark: from now (common §1.2)
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
  bool trackStart() override { state = 2; fired = false; ++starts; return true; }   // waiting; the last trigger gone
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
    CHECK(group.handle(0x02, nullptr, 0, out, sizeof out).resolution == kResolutionCompleted);
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
  const Bytes want = {0x4B, 2, uint8_t(Endpoint::kMaxRoles), 0};
  CHECK(std::search(d.begin(), d.end(), want.begin(), want.end()) != d.end());   // plan_roles declared
}

int main() {
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
