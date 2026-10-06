// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.target.console (OepConsole.cpp) over DmConsole and Ch32Dm on a fake DMI PHY whose debug module keeps
// DATA0 / DATA1 and DMSTATUS's halted / havereset bits; the test plays the target's side of the mailbox.
// - core §4.3's order: a stream op's form and values are checked before its stream number (no_connection last); read
//   from 3 with arg over 0xFF is malformed (common §1.2), the fixture UART's read too.
// - write takes only the send slot (console §2); the console reads while DMSTATUS says the hart runs (console §3).
#include <stdio.h>

#include <vector>

#include "OepConsole.h"
#include "OepDmConsole.h"
#include "OepFixture.h"
#include "OepPinTable.h"
#include "OepTarget.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace wire = reg::wire_rvswd;
namespace con = reg::target_console;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// A debug module behind a link that always answers: DATA0 / DATA1 as written, DMCONTROL's haltreq / resumereq /
// ackhavereset acted on, every DMI read counted.
class FakePhy final : public DmiPhy {
 public:
  bool attached_flag = false, halted = false, havereset = false;
  uint32_t data0 = 0, data1 = 0;
  int reads = 0, status_reads = 0, data0_reads = 0;
  std::vector<uint32_t> data0_writes;
  bool attach() override { attached_flag = true; return true; }
  void release() override { attached_flag = false; }
  bool attached() const override { return attached_flag; }
  void write(uint8_t address, uint32_t value) override {
    if (address == 0x04) { data0 = value; data0_writes.push_back(value); }
    if (address == 0x05) data1 = value;
    if (address == 0x10) {
      if (value & (1u << 31)) halted = true;
      if (value & (1u << 30)) halted = false;
      if (value & (1u << 28)) havereset = false;
    }
  }
  bool setIdleClockLow(bool) override { return true; }
  bool canIdleClockLow() const override { return true; }
  bool setMaxHz(uint32_t) override { return true; }
  bool keepsMaxHz(uint32_t) const override { return true; }
  uint32_t dmiNs() const override { return 1000; }
  uint32_t clockHz() const override { return 1000000; }
  uint32_t retries() const override { return 0; }
  uint32_t transactions() const override { return 0; }

 protected:
  bool readWire(uint8_t address, uint32_t &value) override {
    advanceMicros(10);   // a read takes time (the waits for DM state are bounded by time)
    if (!attached_flag) return false;
    ++reads;
    switch (address) {
      case 0x04: value = data0; ++data0_reads; break;
      case 0x05: value = data1; break;
      case 0x10: value = 1; break;
      case 0x11:
        ++status_reads;
        value = 2 | (1u << 7) | (halted ? (3u << 8) : (3u << 10)) | (havereset ? (3u << 18) : 0);
        break;
      case 0x12: value = 0x0002'1000u | 0x380; break;
      case 0x16: value = 2; break;
      default: value = 0; break;
    }
    return true;
  }
};

static Result call(Interface &i, uint8_t op, const Bytes &payload, Bytes &out) {
  out.assign(512, 0);
  const Result r = i.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static bool rejectedWith(const Result &r, uint8_t reason) { return r.resolution == kResolutionRejected && r.detail == reason; }
static Bytes le16(uint16_t v) { return {uint8_t(v), uint8_t(v >> 8)}; }
static Bytes cat(Bytes a, const Bytes &b) { a.insert(a.end(), b.begin(), b.end()); return a; }
struct MarkSeen { uint32_t serial; uint64_t position; uint8_t kind, detail; };
// Every mark the stream keeps (marks from serial 0; the ring holds 16).
static std::vector<MarkSeen> marksOf(Interface &console, uint16_t stream) {
  Bytes out;
  const Bytes req = {uint8_t(stream), uint8_t(stream >> 8), 0, 0, 0, 0};
  std::vector<MarkSeen> all;
  if (call(console, TargetConsoleStream::kOpMarks, req, out).resolution != kResolutionCompleted || out.size() < 2) return all;
  size_t at = 2;
  for (uint8_t i = 0; i < out[1] && at + 23 <= out.size(); ++i, at += 23) {
    const uint8_t *m = out.data() + at + 1;
    uint64_t position = 0;
    for (int b = 7; b >= 0; --b) position = position << 8 | m[4 + b];
    all.push_back({uint32_t(m[0] | m[1] << 8 | m[2] << 16 | uint32_t(m[3]) << 24), position, m[12], m[21]});
  }
  return all;
}
static uint8_t crc8(const uint8_t *p, size_t n) {   // dmseq's CRC-8: poly 0x07, init 0xFF
  uint8_t crc = 0xff;
  for (size_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (int b = 0; b < 8; ++b) crc = uint8_t((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
  }
  return crc;
}
static Bytes attachRequest() { return {0, uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 4, 0x40, 0x42, 0x0f, 0x00}; }

// A dmseq target as the reference one (oep-spec experiments/dm-console-seq DmSeq.h): its sketch calls available() every
// `period` ms, which takes an answer and, idle, posts an empty frame; input rides on the answers.
struct SeqTarget {
  uint32_t &d0;
  uint8_t s = 0, last_h = 1;
  bool posted = false, syn = true;
  uint32_t w0 = 0;
  std::vector<uint8_t> rx;
  explicit SeqTarget(uint32_t &data0) : d0(data0) {}
  void post() {
    uint8_t b[2] = {uint8_t(0x80 | (s << 5) | (last_h << 4) | (syn ? 0x08 : 0)), 0};
    b[1] = crc8(b, 1);
    w0 = d0 = b[0] | uint32_t(b[1]) << 8;
    posted = true;
  }
  void service() {
    if (!posted) return;
    const uint32_t w = d0;
    if (w & 0x80u) { if (w != w0) d0 = w0; return; }   // target rule 0
    if (w == 0) return;
    const uint8_t a[4] = {uint8_t(w), uint8_t(w >> 8), uint8_t(w >> 16), uint8_t(w >> 24)};
    const uint8_t k = (a[0] >> 5) & 1, h = (a[0] >> 4) & 1, m = a[0] & 7;
    if (k != s || m > 2 || crc8(a, 1 + m) != a[1 + m]) { d0 = w0; return; }
    posted = syn = false;
    s ^= 1;
    if (m && h != last_h) { rx.insert(rx.end(), a + 1, a + 1 + m); last_h = h; }
  }
  void available() { service(); if (!posted) post(); }
};

// The time from a host's first write of "PING\n" until the target has all of it (oep-if-console §2: write takes the
// send slot; the host writes the rest as the slot frees, retrying every 5 ms when it is not free; each request takes
// `rtt_ms` there and back). The probe polls the console every 250 us between requests.
static uint32_t pingDeliveryMs(Interface &wire, DebugPort &port, TargetConsoleStream &console, FakePhy &phy,
                               uint32_t period_ms, uint32_t rtt_ms) {
  Bytes out;
  if (!port.connected) call(wire, WireRvswd::kOpAttach, attachRequest(), out);
  call(console, TargetConsoleStream::kOpOpen, cat(le16(port.number), {con::kMechanismDmseq}), out);
  const uint16_t stream = uint16_t(out[0] | out[1] << 8);
  SeqTarget target(phy.data0);
  uint32_t next = millis();
  auto run = [&](uint32_t us) {
    for (uint32_t t = 0; t < us; t += 250) {
      advanceMicros(250);
      console.poll();
      if (int32_t(millis() - next) >= 0) { target.available(); next += period_ms; }
    }
  };
  run(200 * 1000);   // synced
  const uint8_t msg[] = {'P', 'I', 'N', 'G', '\n'};
  size_t sent = 0;
  const uint32_t start = millis();
  while (target.rx.size() < sizeof msg && millis() - start < 5000) {
    if (sent == sizeof msg) { run(250); continue; }
    run(rtt_ms * 500);
    Bytes req = cat(le16(stream), {uint8_t(sizeof msg - sent), 0});
    req.insert(req.end(), msg + sent, msg + sizeof msg);
    call(console, TargetConsoleStream::kOpWrite, req, out);
    const size_t took = out.size() >= 2 ? size_t(out[0] | out[1] << 8) : 0;
    run(rtt_ms * 500);
    sent += took;
    if (!took) run(5000);
  }
  const uint32_t ms = millis() - start;
  call(console, TargetConsoleStream::kOpClose, le16(stream), out);
  return target.rx == std::vector<uint8_t>(msg, msg + sizeof msg) ? ms : 99999;
}

int main() {
  static FakePhy phy;
  static Ch32Dm dm(phy);
  static DebugPort port{dm, 0, 1};
  static WireRvswd wire_fn(port, 0);
  static DmConsole driver(dm, phy);
  static TargetConsoleStream console(port, driver, 0);
  Bytes out;

  Result r = call(wire_fn, WireRvswd::kOpAttach, attachRequest(), out);
  CHECK(ok(r) && port.connected);
  r = call(console, TargetConsoleStream::kOpOpen, cat(le16(port.number), {con::kMechanismDmseq}), out);
  CHECK(ok(r) && out.size() == 3);
  const uint16_t stream = uint16_t(out[0] | out[1] << 8);
  {
    // mark attach carries no detail (common §1.3: "—"; it carried the mechanism, 2)
    const std::vector<MarkSeen> marks = marksOf(console, stream);
    CHECK(!marks.empty() && marks.back().kind == reg::common::kMarkKindAttach && marks.back().detail == 0);
  }

  // ---- core §4.3: the form first, an unknown stream number last (it answered no_connection first) ----
  {
    const Bytes unknown = le16(uint16_t(stream + 100));
    r = call(console, TargetConsoleStream::kOpRead, cat(unknown, {0, 0, 0}), out);   // cut short
    CHECK(rejectedWith(r, kRejectMalformed));
    r = call(console, TargetConsoleStream::kOpRead, cat(unknown, {4, 0, 0, 0, 0, 0, 0, 0, 0, 16, 0}), out);   // from 4
    CHECK(rejectedWith(r, kRejectUnsupported));
    r = call(console, TargetConsoleStream::kOpWrite, cat(unknown, {0, 0}), out);   // count 0
    CHECK(rejectedWith(r, kRejectMalformed));
    r = call(console, TargetConsoleStream::kOpMark, unknown, out);                 // no value
    CHECK(rejectedWith(r, kRejectMalformed));
    r = call(console, TargetConsoleStream::kOpMark, cat(unknown, {1}), out);       // well formed: no_connection
    CHECK(rejectedWith(r, kRejectNoConnection));
  }

  // ---- read from 3 (the last mark of kind arg): arg over 0xFF is malformed (common §1.2; it read kind arg & 0xFF) ----
  {
    r = call(console, TargetConsoleStream::kOpRead, cat(le16(stream), {3, 0x01, 0x01, 0, 0, 0, 0, 0, 0, 16, 0}), out);
    CHECK(rejectedWith(r, kRejectMalformed));
    r = call(console, TargetConsoleStream::kOpRead, cat(le16(stream), {3, 0xff, 0, 0, 0, 0, 0, 0, 0, 16, 0}), out);
    CHECK(ok(r));
    // the fixture UART's read alike
    static PinTable uart_pins(0);
    static HardwareSerial serial;
    static FixtureUart uart(uart_pins, serial, 0);
    r = call(uart, FixtureUart::kOpRead, {3, 0x00, 0x01, 0, 0, 0, 0, 0, 0, 16, 0}, out);
    CHECK(rejectedWith(r, kRejectMalformed));
    r = call(uart, FixtureUart::kOpRead, {4, 0x00, 0x01, 0, 0, 0, 0, 0, 0, 16, 0}, out);   // from 4: unsupported
    CHECK(rejectedWith(r, kRejectUnsupported));
    r = call(uart, FixtureUart::kOpRead, {3, 0x07, 0, 0, 0, 0, 0, 0, 0, 16, 0}, out);
    CHECK(ok(r));
  }

  // ---- write takes only what the send slot carries in one go (oep-if-console §2: 2 bytes for dmseq, 3 for DMDATA),
  // nothing while it is not free; a bind's input still queues what the driver's queue takes (write took up to 255) ----
  {
    const Bytes five = cat(le16(stream), {5, 0, 'a', 'b', 'c', 'd', 'e'});
    r = call(console, TargetConsoleStream::kOpWrite, five, out);
    CHECK(r.detail == kOutcomePartial && out == Bytes({2, 0}));
    r = call(console, TargetConsoleStream::kOpWrite, five, out);   // the slot not free yet: accepted 0, failed
    CHECK(r.detail == kOutcomeFailed && out == Bytes({0, 0}));
    r = call(console, TargetConsoleStream::kOpClose, le16(stream), out);
    CHECK(ok(r) && !console.isOpen());
    r = call(console, TargetConsoleStream::kOpOpen, cat(le16(port.number), {con::kMechanismDmdata}), out);
    CHECK(ok(r) && console.isOpen());
    const uint16_t dmdata = uint16_t(out[0] | out[1] << 8);
    r = call(console, TargetConsoleStream::kOpWrite, cat(le16(dmdata), {5, 0, 'a', 'b', 'c', 'd', 'e'}), out);
    CHECK(r.detail == kOutcomePartial && out == Bytes({3, 0}));
    const uint8_t typed[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(console.bindInput(typed, sizeof typed) == sizeof typed);
    r = call(console, TargetConsoleStream::kOpClose, le16(dmdata), out);
    CHECK(ok(r));
  }

  // ---- reading goes on while the hart runs, judged from DMSTATUS at least every 20 ms (oep-if-console §3): a hart the
  // probe halted and the host resumed through raw DMI is read again (it waited for the next high-level op) ----
  {
    r = call(console, TargetConsoleStream::kOpOpen, cat(le16(port.number), {con::kMechanismDmseq}), out);
    CHECK(ok(r) && console.isOpen());
    CHECK(dm.halt() && dm.halted() && phy.halted);
    g_millis += 25;
    console.poll();
    int data0_reads = phy.data0_reads;
    g_millis += 5;
    console.poll();                                        // halted: DATA0 left alone
    CHECK(phy.data0_reads == data0_reads);
    CHECK(dm.writeDmi(0x10, 0x40000001) && !phy.halted);   // the host's raw resume
    phy.data0 = 0x00003298u;                               // the target posts an empty SYN frame (S 0, A 1)
    for (int i = 0; i < 3; ++i) { g_millis += 10; console.poll(); }
    CHECK(!dm.halted() && phy.data0_reads > data0_reads);
    CHECK(!phy.data0_writes.empty() && !(phy.data0_writes.back() & 0x80u));   // answered
    // the host's raw halt: DATA0 is no longer read once DMSTATUS has been looked at
    CHECK(dm.writeDmi(0x10, 0x80000001) && phy.halted);
    g_millis += 20;
    console.poll();
    data0_reads = phy.data0_reads;
    for (int i = 0; i < 3; ++i) { g_millis += 5; console.poll(); }
    CHECK(phy.data0_reads == data0_reads);
    CHECK(dm.writeDmi(0x10, 0x40000001) && !phy.halted);
  }

  // ---- a dmseq frame accepted with TO set: mark lost 4 (the target's TO) right after its payload (common §1.3; none
  // was attached) ----
  {
    // the stream is synced on the SYN frame above (S 0): the next frame, S 1, A 0, TO, one byte 'x'
    uint8_t f[3] = {uint8_t(0x80 | 0x40 | 0x20 | 1), 'x', 0};
    f[2] = crc8(f, 2);
    phy.data0 = f[0] | f[1] << 8 | uint32_t(f[2]) << 16;
    for (int i = 0; i < 3; ++i) { g_millis += 10; console.poll(); }
    const std::vector<MarkSeen> marks = marksOf(console, console.bindStreamNumber());
    CHECK(!marks.empty() && marks.back().kind == reg::common::kMarkKindLost &&
          marks.back().detail == reg::common::kMarkDetailLostTargetTimeout);
    const PositionStream *ps = console.bindStream();
    CHECK(ps && !marks.empty() && marks.back().position == ps->end());   // after the 'x'
    // the same frame read again (a duplicate) adds nothing
    const size_t count = marks.size();
    phy.data0 = f[0] | f[1] << 8 | uint32_t(f[2]) << 16;
    for (int i = 0; i < 3; ++i) { g_millis += 10; console.poll(); }
    CHECK(marksOf(console, console.bindStreamNumber()).size() == count);
  }

  // ---- the ring pushing bytes out: mark lost 1 (overflow) once per episode - at the first byte pushed out, again only
  // after a clear has emptied the ring (common §1.1, §1.3; nothing was marked) ----
  {
    uint8_t buffer[16];
    PositionStream::Mark ring[8];
    PositionStream ps(buffer, sizeof buffer, ring, 8);
    auto lostMarks = [&]() {
      Bytes m(2 + 8 * 23);
      const size_t n = ps.marks(0, m.data(), m.size());
      int lost = 0;
      for (size_t at = 2; at + 23 <= n; at += 23)
        if (m[at + 13] == reg::common::kMarkKindLost && m[at + 22] == reg::common::kMarkDetailLostOverflow) ++lost;
      return lost;
    };
    for (int i = 0; i < 16; ++i) ps.put(uint8_t(i));
    CHECK(lostMarks() == 0);   // full, nothing pushed out yet
    ps.put(16);
    CHECK(lostMarks() == 1);
    for (int i = 0; i < 40; ++i) ps.put(uint8_t(i));
    CHECK(lostMarks() == 1);   // the same episode
    ps.clear();
    for (int i = 0; i < 16; ++i) ps.put(uint8_t(i));
    CHECK(lostMarks() == 1);
    ps.put(0);
    CHECK(lostMarks() == 2);   // a new episode after the clear
  }

  // ---- an open with another mechanism makes a new stream and erases the old one (oep-if-console §2; the new stream
  // showed the old one's marks): its only mark is attach, its positions go on; the same mechanism again reopens it with
  // its marks; a fixture UART's stream disappears with its plan (oep-if-fixture §2) ----
  {
    r = call(console, TargetConsoleStream::kOpOpen, cat(le16(port.number), {con::kMechanismDmseq}), out);
    CHECK(ok(r));
    const uint16_t old_stream = uint16_t(out[0] | out[1] << 8);
    r = call(console, TargetConsoleStream::kOpMark, cat(le16(old_stream), {0x5a}), out);
    CHECK(ok(r));
    const std::vector<MarkSeen> before = marksOf(console, old_stream);
    CHECK(before.size() >= 2);
    r = call(console, TargetConsoleStream::kOpClose, le16(old_stream), out);
    CHECK(ok(r));
    r = call(console, TargetConsoleStream::kOpOpen, cat(le16(port.number), {con::kMechanismDmdata}), out);
    CHECK(ok(r) && out[2] == 0);
    const uint16_t fresh = uint16_t(out[0] | out[1] << 8);
    CHECK(fresh != old_stream);
    std::vector<MarkSeen> marks = marksOf(console, fresh);
    CHECK(marks.size() == 1 && marks[0].kind == reg::common::kMarkKindAttach);
    if (!before.empty() && marks.size() == 1) CHECK(marks[0].serial > before.back().serial);   // serials go on
    r = call(console, TargetConsoleStream::kOpRead, cat(le16(fresh), {1, 0, 0, 0, 0, 0, 0, 0, 0, 16, 0}), out);
    CHECK(ok(r) && out.size() == 11 && out[8] == 0 && out[9] == 0);   // nothing of the old stream, no gap
    r = call(console, TargetConsoleStream::kOpRead, cat(le16(old_stream), {1, 0, 0, 0, 0, 0, 0, 0, 0, 16, 0}), out);
    CHECK(rejectedWith(r, kRejectNoConnection));   // the old number is gone (no_connection, core §4.3)
    r = call(console, TargetConsoleStream::kOpClose, le16(fresh), out);
    CHECK(ok(r));
    r = call(console, TargetConsoleStream::kOpOpen, cat(le16(port.number), {con::kMechanismDmdata}), out);
    CHECK(ok(r) && out[2] == con::kOpenFlagsExisting && uint16_t(out[0] | out[1] << 8) == fresh);
    marks = marksOf(console, fresh);
    CHECK(marks.size() == 3);   // attach, closed, attach: reopened at the same place, the marks continue
    r = call(console, TargetConsoleStream::kOpClose, le16(fresh), out);
    CHECK(ok(r));

    static PinTable uart_pins(0);
    static HardwareSerial serial;
    static FixtureUart uart(uart_pins, serial, 1);
    r = call(uart, FixtureUart::kOpMark, {0x33}, out);
    CHECK(ok(r));
    r = call(uart, FixtureUart::kOpMarks, {0, 0, 0, 0}, out);
    CHECK(ok(r) && out.size() >= 2 && out[1] == 1);
    uart.planRelease();
    r = call(uart, FixtureUart::kOpMarks, {0, 0, 0, 0}, out);
    CHECK(ok(r) && out.size() == 2 && out[1] == 0);
  }

  // ---- detach with force closes the connection: its stream marks detach, then closed 4 (oep-if-debug §2's table; it
  // marked closed 4 alone); a plain detach that closes it marks closed 4 alone ----
  for (bool force : {true, false}) {
    if (!port.connected) {
      r = call(wire_fn, WireRvswd::kOpAttach, attachRequest(), out);
      CHECK(ok(r) && port.connected);
    }
    r = call(console, TargetConsoleStream::kOpOpen, cat(le16(port.number), {con::kMechanismDmseq}), out);
    CHECK(ok(r) && console.isOpen());
    const uint16_t s = uint16_t(out[0] | out[1] << 8);
    Bytes detach = le16(port.number);
    if (force) detach.insert(detach.end(), {uint8_t(wire::kTlvDetachForce | kTagCritical), 0});
    r = call(wire_fn, WireRvswd::kOpDetach, detach, out);
    CHECK(ok(r) && !port.connected);
    console.poll();
    CHECK(!console.isOpen());
    const std::vector<MarkSeen> marks = marksOf(console, s);
    CHECK(marks.size() >= 2 && marks.back().kind == reg::common::kMarkKindClosed &&
          marks.back().detail == reg::common::kMarkDetailClosedConnectionClosed);
    if (marks.size() >= 2) CHECK((marks[marks.size() - 2].kind == reg::common::kMarkKindDetach) == force);
  }
  r = call(wire_fn, WireRvswd::kOpAttach, attachRequest(), out);
  CHECK(ok(r) && port.connected);

  // ---- open on a live connection this console does not ride on (an arm-adi one): unavailable cause 6
  // (oep-if-console §1; it answered no_connection) ----
  {
    const uint16_t swd = ResourceNumbers::take(ResourceNumbers::kConnection);
    r = call(console, TargetConsoleStream::kOpOpen, cat(le16(swd), {con::kMechanismDmseq}), out);
    CHECK(rejectedWith(r, kRejectUnavailable) && out == Bytes({reg::core::kTlvUnavailablePayloadCause, 1,
                                                                reg::core::kUnavailableCauseWrongState}));
    ResourceNumbers::close(swd);
    r = call(console, TargetConsoleStream::kOpOpen, cat(le16(swd), {con::kMechanismDmseq}), out);   // closed: unknown
    CHECK(rejectedWith(r, kRejectNoConnection));
  }

  // ---- the console's write latency (oep-if-console §2): a host's line reaches the target one chunk per target poll,
  // as when write queued it whole (0.0.28: 13-26 ms for a PING / PONG); a slot that freed only at the target's ack missed
  // that same exchange and took two polls a chunk (50 ms and more) ----
  {
    static FakePhy phy2;
    static Ch32Dm dm2(phy2);
    static DebugPort port2{dm2, 2, 3};
    static WireRvswd wire2(port2, 1);
    static DmConsole driver2(dm2, phy2);
    static TargetConsoleStream console2(port2, driver2, 1);
    for (const uint32_t period : {1u, 5u, 10u}) {
      for (const uint32_t rtt : {1u, 4u}) {
        const uint32_t ms = pingDeliveryMs(wire2, port2, console2, phy2, period, rtt);
        printf("  PING delivered: target poll %u ms, request %u ms: %u ms\n", period, rtt, ms);
        CHECK(ms <= 3 * period + 3 * rtt + 2);
      }
    }
  }

  printf("console: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
