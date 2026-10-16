// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.target.console (OepConsole.cpp) over DmConsole and Ch32Dm on a fake DMI PHY whose debug module keeps
// DATA0 / DATA1 and DMSTATUS's halted / havereset bits; the test plays the target's side of the mailbox.
// - core §4.3: a stream op's form and values are checked before its stream number; read from 3 with arg over 0xFF
//   reads from now (common §1.2), the fixture UART's read too; write count 0 is success.
// - write fills the stream's send queue, not declared (console §1, §2); DMDATA by §3.2 (no slot, empty slot, a
//   bit-7-clear word left alone, open writes nothing); the console reads while DMSTATUS says the hart runs, and after
//   a riscv-dm request of the connection it reads DMSTATUS before DATA0 (console §3).
// - a session's end (end or lapse) takes its share of the stream before the connection's: closed 2 when it was the last
//   user, readable after, and the same place and mechanism opened again gives the same number and marks (console §2).
#include <stdio.h>

#include <vector>

#include "OepConsole.h"
#include "OepDmConsole.h"
#include "OepEndpoint.h"
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
  std::vector<uint8_t> order;   // the DMSTATUS (0x11) and DATA0 (0x04) reads, in order
  std::vector<uint32_t> data0_writes;
  // faults: the next `bad_havereset` DMSTATUS reads come back with havereset set though none is pending (a bad read);
  // the next `lose_data0` writes of DATA0 do not land; `dropped`: the link reads all ones until reinit() (a CH32L103
  // after a change of hart state)
  int bad_havereset = 0, lose_data0 = 0, reinits = 0;
  bool dropped = false;
  void reinit() override { dropped = false; ++reinits; }
  bool attach() override { attached_flag = true; return true; }
  void release() override { attached_flag = false; }
  bool attached() const override { return attached_flag; }
  void write(uint8_t address, uint32_t value) override {
    if (dropped) return;
    if (address == 0x04 && lose_data0 > 0) { --lose_data0; return; }
    if (address == 0x04) { data0 = value; data0_writes.push_back(value); }
    if (address == 0x05) data1 = value;
    if (address == 0x10) {
      if (value & (1u << 31)) halted = true;
      if (value & (1u << 30)) halted = false;
      if (value & (1u << 28)) havereset = false;
    }
  }
  // the wire's turn for the console (DmiPhy::backgroundTurn): refused while `paused`; turns given and ended counted
  bool paused = false;
  int turns = 0, turns_done = 0;
  bool backgroundTurn() override { if (paused) return false; ++turns; return true; }
  void backgroundDone() override { ++turns_done; }
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
    if (dropped) { value = 0xffffffffu; return true; }
    switch (address) {
      case 0x04: value = data0; ++data0_reads; order.push_back(0x04); break;
      case 0x05: value = data1; break;
      case 0x10: value = 1; break;
      case 0x11:
        ++status_reads;
        order.push_back(0x11);
        value = 2 | (1u << 7) | (halted ? (3u << 8) : (3u << 10)) | (havereset || bad_havereset > 0 ? (3u << 18) : 0);
        if (bad_havereset > 0) --bad_havereset;
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
static Bytes le32(uint32_t v) { return {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)}; }   // a resource number
static Bytes cat(Bytes a, const Bytes &b) { a.insert(a.end(), b.begin(), b.end()); return a; }
struct MarkSeen { uint32_t serial; uint64_t position; uint8_t kind, detail; };
// A marks answer: more(u8) count(u8) count x (serial u32, position u64, kind u8, time_ns u64, detail u8) - 22 bytes
// each, no element length (core §2.3).
static std::vector<MarkSeen> parseMarks(const Bytes &out) {
  std::vector<MarkSeen> all;
  if (out.size() < 2) return all;
  size_t at = 2;
  for (uint8_t i = 0; i < out[1] && at + PositionStream::kMarkBytes <= out.size(); ++i, at += PositionStream::kMarkBytes) {
    const uint8_t *m = out.data() + at;
    uint64_t position = 0;
    for (int b = 7; b >= 0; --b) position = position << 8 | m[4 + b];
    all.push_back({uint32_t(m[0] | m[1] << 8 | m[2] << 16 | uint32_t(m[3]) << 24), position, m[12], m[21]});
  }
  return all;
}
// Every mark the stream keeps (marks from serial 0; the ring holds 16).
static std::vector<MarkSeen> marksOf(Interface &console, uint32_t stream) {
  Bytes out;
  const Bytes req = cat(le32(stream), {0, 0, 0, 0});   // stream(u32) from_serial(u32)
  if (call(console, TargetConsoleStream::kOpMarks, req, out).resolution != kResolutionCompleted) return {};
  return parseMarks(out);
}
static uint8_t crc8(const uint8_t *p, size_t n) {   // dmseq's CRC-8: poly 0x07, init 0xFF
  uint8_t crc = 0xff;
  for (size_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (int b = 0; b < 8; ++b) crc = uint8_t((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
  }
  return crc;
}
// method 0, max_speed 1 MHz: tag len(u16) value (core §2.2)
// attach (debug §1): method 0, swdio swclk (the port's pair), max_speed 1 MHz
static Bytes attachRequest(uint16_t swdio = 0, uint16_t swclk = 1) {
  return {0, uint8_t(swdio), uint8_t(swdio >> 8), uint8_t(swclk), uint8_t(swclk >> 8), 0x40, 0x42, 0x0f, 0x00};
}

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

// The time from a host's first write of "PING\n" until the target has all of it (oep-if-console §2: write fills the
// send queue; the host writes any rest as it frees, retrying every 5 ms when it is full; each request takes
// `rtt_ms` there and back). The probe polls the console every 250 us between requests.
static uint32_t pingDeliveryMs(Interface &wire, DebugPort &port, TargetConsoleStream &console, FakePhy &phy,
                               uint32_t period_ms, uint32_t rtt_ms) {
  Bytes out;
  if (!port.connected) call(wire, WireRvswd::kOpAttach, attachRequest(port.swdio, port.swclk), out);
  call(console, TargetConsoleStream::kOpOpen, cat(le32(port.number), {console.numberOf(TargetConsoleStream::kDriverDmseq)}), out);
  const uint32_t stream = getU32(out.data());
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
    Bytes req = cat(le32(stream), {uint8_t(sizeof msg - sent), 0});
    req.insert(req.end(), msg + sent, msg + sizeof msg);
    call(console, TargetConsoleStream::kOpWrite, req, out);
    const size_t took = out.size() >= 2 ? size_t(out[0] | out[1] << 8) : 0;
    run(rtt_ms * 500);
    sent += took;
    if (!took) run(5000);
  }
  const uint32_t ms = millis() - start;
  call(console, TargetConsoleStream::kOpClose, le32(stream), out);
  return target.rx == std::vector<uint8_t>(msg, msg + sizeof msg) ? ms : 99999;
}

// A length-prefixed transport (transports §1) in memory: what the host sent, what the probe answered.
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
};
static Bytes u32(uint32_t v) { return {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)}; }
// One request (core §4.1): role 0x01, corr(u16), fn(u16), op(u8), session_id(u32; 0 = no session), payload; sent as
// length(u16) message, the answer message (role, corr, resolution, detail, payload) taken back (empty: none).
[[maybe_unused]] static Bytes exchange(Endpoint &ep, MemStream &link, uint16_t corr, uint16_t fn, uint8_t op, const Bytes &payload,
                      uint32_t session) {
  Bytes m = {0x01, uint8_t(corr), uint8_t(corr >> 8), uint8_t(fn), uint8_t(fn >> 8), op};
  m = cat(cat(m, u32(session)), payload);
  link.tx.clear();
  link.rx.insert(link.rx.end(), {uint8_t(m.size()), uint8_t(m.size() >> 8)});
  link.rx.insert(link.rx.end(), m.begin(), m.end());
  ep.poll();
  if (link.tx.size() < 2 + kResultHeader) return {};
  return Bytes(link.tx.begin() + 2, link.tx.end());
}
static bool answered(const Bytes &a, uint8_t resolution, uint8_t detail) {
  return a.size() >= kResultHeader && a[3] == resolution && a[4] == detail;
}
[[maybe_unused]] static bool answeredOk(const Bytes &a) { return answered(a, kResolutionCompleted, kOutcomeSuccess); }
[[maybe_unused]] static Bytes answerPayload(const Bytes &a) { return a.size() > kResultHeader ? Bytes(a.begin() + kResultHeader, a.end()) : Bytes{}; }

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
  r = call(console, TargetConsoleStream::kOpOpen, cat(le32(port.number), {console.numberOf(TargetConsoleStream::kDriverDmseq)}), out);
  CHECK(ok(r) && out.size() == 4);   // stream(u32) (console §1)
  const uint32_t stream = getU32(out.data());
  CHECK(marksOf(console, stream).empty());   // no attach mark (console §2)

  // ---- core §4.3: the form first, then the stream number ----
  {
    const Bytes unknown = le32(stream + 100);
    r = call(console, TargetConsoleStream::kOpRead, cat(unknown, {0, 0, 0}), out);   // cut short
    CHECK(rejectedWith(r, kRejectMalformed));
    r = call(console, TargetConsoleStream::kOpRead, cat(unknown, {4, 0, 0, 0, 0, 0, 0, 0, 0, 16, 0}), out);   // from 4
    CHECK(rejectedWith(r, kRejectUnsupported));
    r = call(console, TargetConsoleStream::kOpWrite, cat(unknown, {0, 0}), out);   // count 0: well formed
    CHECK(rejectedWith(r, kRejectNoResource));
    r = call(console, TargetConsoleStream::kOpWrite, cat(le32(stream), {0, 0}), out);   // accepted 0 = count: success
    CHECK(ok(r) && out == le16(0));
    r = call(console, TargetConsoleStream::kOpMark, unknown, out);                 // no value
    CHECK(rejectedWith(r, kRejectMalformed));
    r = call(console, TargetConsoleStream::kOpMark, cat(unknown, {1}), out);       // well formed: no_connection
    CHECK(rejectedWith(r, kRejectNoResource));
  }

  // ---- read from 3 (the last mark of kind arg): arg over 0xFF matches no mark - from now (common §1.2; it read kind
  // arg & 0xFF, then was malformed) ----
  {
    r = call(console, TargetConsoleStream::kOpRead, cat(le32(stream), {3, 0x01, 0x01, 0, 0, 0, 0, 0, 0, 16, 0}), out);
    CHECK(ok(r) && out.size() == 11 && out[8] == 0 && out[9] == 0);   // start = now, no flags, len 0
    r = call(console, TargetConsoleStream::kOpRead, cat(le32(stream), {3, 0xff, 0, 0, 0, 0, 0, 0, 0, 16, 0}), out);
    CHECK(ok(r));
    // the fixture UART's read alike
    static PinTable uart_pins(0);
    static HardwareSerial serial;
    static FixtureUart uart(uart_pins, serial, 0);
    r = call(uart, FixtureUart::kOpRead, {3, 0x00, 0x01, 0, 0, 0, 0, 0, 0, 16, 0}, out);
    CHECK(!rejectedWith(r, kRejectMalformed));
    r = call(uart, FixtureUart::kOpRead, {4, 0x00, 0x01, 0, 0, 0, 0, 0, 0, 16, 0}, out);   // from 4: unsupported
    CHECK(rejectedWith(r, kRejectUnsupported));
    r = call(uart, FixtureUart::kOpRead, {3, 0x07, 0, 0, 0, 0, 0, 0, 0, 16, 0}, out);
    CHECK(ok(r));
  }

  // ---- write fills the stream's send queue (oep-if-console §2): accepted = min(count, free space), 0 only when it is
  // full; describe does not declare its size (console §1: the probe's); SDI takes nothing; the queue
  // goes with the stream when it closes; a bind's input fills the same queue. (It took only the mechanism's send slot,
  // 2 / 3 bytes, and 0 while that held a chunk.) ----
  {
    Bytes d(64);
    const size_t dn = console.describe(d.data(), d.size());
    bool declared = false;
    for (size_t at = 0; at + kTlvHeader <= dn; at += kTlvHeader + (d[at + 1] | d[at + 2] << 8))
      declared |= d[at] == 0x41;   // the old send_queue tag
    // the mechanisms only, one name each (console §1, core §7.7): sdi, dmdata, dmseq
    CHECK(!declared && dn == 3 * kTlvHeader + strlen("sdi") + strlen("dmdata") + strlen("dmseq"));
    phy.halted = true;   // nothing taken from the queue meanwhile (the console does not read a halted hart's mailbox)
    g_millis += 25;
    console.poll();
    const Bytes five = cat(le32(stream), {5, 0, 'a', 'b', 'c', 'd', 'e'});
    r = call(console, TargetConsoleStream::kOpWrite, five, out);
    CHECK(ok(r) && out == Bytes({5, 0}));
    Bytes big = cat(le32(stream), le16(300));
    big.resize(big.size() + 300, 'x');
    r = call(console, TargetConsoleStream::kOpWrite, big, out);   // what fits: the rest of the queue
    CHECK(r.detail == kOutcomePartial && out == le16(uint16_t(DmConsole::kSendQueue - 5)));
    r = call(console, TargetConsoleStream::kOpWrite, five, out);   // full: accepted 0, failed
    CHECK(r.detail == kOutcomeFailed && out == Bytes({0, 0}));
    const uint8_t typed[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(console.bindInput(typed, sizeof typed) == 0);            // a bind's input: the same queue
    r = call(console, TargetConsoleStream::kOpClose, le32(stream), out);
    CHECK(ok(r) && !console.isOpen());
    CHECK(driver.room() == 0);   // closed: nothing taken
    phy.halted = false;
    r = call(console, TargetConsoleStream::kOpOpen, cat(le32(port.number), {console.numberOf(TargetConsoleStream::kDriverDmdata)}), out);
    CHECK(ok(r) && console.isOpen());
    CHECK(driver.room() == DmConsole::kSendQueue);   // the queue went with the closed stream
    const uint32_t dmdata = getU32(out.data());
    r = call(console, TargetConsoleStream::kOpWrite, cat(le32(dmdata), {5, 0, 'a', 'b', 'c', 'd', 'e'}), out);
    CHECK(ok(r) && out == Bytes({5, 0}));
    CHECK(console.bindInput(typed, sizeof typed) == sizeof typed);
    r = call(console, TargetConsoleStream::kOpClose, le32(dmdata), out);
    CHECK(ok(r));
    r = call(console, TargetConsoleStream::kOpOpen, cat(le32(port.number), {console.numberOf(TargetConsoleStream::kDriverSdi)}), out);
    CHECK(ok(r));
    const uint32_t sdi = getU32(out.data());
    r = call(console, TargetConsoleStream::kOpWrite, cat(le32(sdi), {1, 0, 'a'}), out);   // one way (console §1)
    CHECK(rejectedWith(r, kRejectUnsupported) && out == Bytes({0}));
    r = call(console, TargetConsoleStream::kOpClose, le32(sdi), out);
    CHECK(ok(r));
  }

  // ---- DMDATA (oep-if-console §3.2) against a target that plays by it: every target slot answered once, input three
  // bytes per answer from the queue's head, a bit-7-clear word never written over; a bit-7 word with L 0-3 or 12-63 is
  // no slot - it carries no bytes and gets 0 with no input on it (it took 7 bytes of 0xff from L 12-63 and answered L 0-3
  // with input); open writes nothing to the mailbox and throws away none of the target's output (it zeroed DATA0 over a
  // bit-7-clear word and dropped four polls of output) ----
  {
    // a target slot already waiting when the stream opens: "hi!" (L 7)
    phy.halted = false;
    phy.data0 = 0x80u | 7u | uint32_t('h') << 8 | uint32_t('i') << 16 | uint32_t('!') << 24;
    const size_t writes_before = phy.data0_writes.size();
    r = call(console, TargetConsoleStream::kOpOpen, cat(le32(port.number), {console.numberOf(TargetConsoleStream::kDriverDmdata)}), out);
    CHECK(ok(r));
    const uint32_t s1 = getU32(out.data());
    CHECK(phy.data0_writes.size() == writes_before);   // open wrote nothing
    const PositionStream *ps = console.bindStream();
    const uint64_t at_open = ps ? ps->end() : 0;
    auto polls = [&](int n) { for (int i = 0; i < n; ++i) { g_millis += 1; console.poll(); } };
    polls(1);
    CHECK(ps && ps->end() == at_open + 3);              // "hi!" kept
    CHECK(phy.data0 == 0);                              // answered with 0: nothing queued
    // a bit-7-clear word (our answer, or 0) is never written over
    phy.data0 = 0x00000005u | uint32_t('z') << 8;       // an answer of ours the target has not taken yet
    const size_t writes_now = phy.data0_writes.size();
    polls(3);
    CHECK(phy.data0_writes.size() == writes_now);
    // input rides on the answers, three bytes each, from the queue's head
    r = call(console, TargetConsoleStream::kOpWrite, cat(le32(s1), {5, 0, 'P', 'I', 'N', 'G', '\n'}), out);
    CHECK(ok(r) && out == Bytes({5, 0}));
    phy.data0 = 0x80u | 5u | uint32_t('a') << 8;        // a slot with one byte: answered with "PIN"
    polls(1);
    CHECK(phy.data0 == (7u | uint32_t('P') << 8 | uint32_t('I') << 16 | uint32_t('N') << 24));
    // not slots: L 0-3 and 12-63 with bit 7 (0xffffffff among them) - 0, nothing pushed, the queue kept
    for (uint32_t word : {0xffffffffu, 0x80u | 12u, 0x80u | 3u, 0x80u | 0u, 0x80u | 0x3fu | 0x41424300u}) {
      const uint64_t end = ps ? ps->end() : 0;
      phy.data0 = word;
      polls(1);
      CHECK(phy.data0 == 0 && ps && ps->end() == end);
      CHECK(driver.room() == DmConsole::kSendQueue - 2);   // "G\n" still queued
    }
    // a slot of 7 bytes (3 in DATA0, 4 in DATA1): all of them, then the rest of the input
    phy.data1 = uint32_t('d') | uint32_t('e') << 8 | uint32_t('f') << 16 | uint32_t('g') << 24;
    phy.data0 = 0x80u | 11u | uint32_t('a') << 8 | uint32_t('b') << 16 | uint32_t('c') << 24;
    const uint64_t end7 = ps ? ps->end() : 0;
    polls(1);
    CHECK(ps && ps->end() == end7 + 7);
    CHECK(phy.data0 == (6u | uint32_t('G') << 8 | uint32_t('\n') << 16));
    // the empty slot (L 4) is answered: with nothing queued, 0
    phy.data0 = 0x84u;
    polls(3);
    CHECK(phy.data0 == 0);
    r = call(console, TargetConsoleStream::kOpClose, le32(s1), out);
    CHECK(ok(r));
    phy.data0 = phy.data1 = 0;
  }

  // ---- reading goes on while the hart runs, judged from DMSTATUS at least every 20 ms (oep-if-console §3): a hart the
  // probe halted and the host resumed through raw DMI is read again (it waited for the next high-level op) ----
  {
    r = call(console, TargetConsoleStream::kOpOpen, cat(le32(port.number), {console.numberOf(TargetConsoleStream::kDriverDmseq)}), out);
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
    // the order (console §3): after a riscv-dm request of the connection is answered (noteRequest), DMSTATUS is read
    // before the next DATA0 read - well inside the status interval, which alone would not read it
    g_millis += 25;
    console.poll();
    g_millis += 1;
    console.poll();
    phy.order.clear();
    console.poll();
    CHECK(!phy.order.empty() && phy.order.front() == 0x04);   // no request: DATA0 straight away
    dm.noteRequest();
    phy.order.clear();
    console.poll();
    CHECK(phy.order.size() >= 2 && phy.order.front() == 0x11);   // DMSTATUS first
    // a request that halted the hart (a raw dmi halt): after it, DATA0 is not read at all
    CHECK(dm.writeDmi(0x10, 0x80000001) && phy.halted);
    dm.noteRequest();
    phy.order.clear();
    console.poll();
    bool data0_read = false;
    for (uint8_t a : phy.order) data0_read |= a == 0x04;
    CHECK(!phy.order.empty() && phy.order.front() == 0x11 && !data0_read);
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
          marks.back().detail == reg::target_console::kMarkDetailLostTargetTimeout);
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
      Bytes m(2 + 8 * PositionStream::kMarkBytes);
      m.resize(ps.marks(0, m.data(), m.size()));
      int lost = 0;
      for (const MarkSeen &mk : parseMarks(m))
        if (mk.kind == reg::common::kMarkKindLost && mk.detail == reg::common::kMarkDetailLostOverflow) ++lost;
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

  // ---- one live stream on a connection (oep-if-console §2): another open while one is open, with the same mechanism or
  // another, is cause 6; a fixture UART's stream disappears with its plan (oep-if-fixture §2) ----
  {
    for (uint8_t d : {uint8_t(TargetConsoleStream::kDriverDmseq), uint8_t(TargetConsoleStream::kDriverDmdata)}) {
      r = call(console, TargetConsoleStream::kOpOpen, cat(le32(port.number), {console.numberOf(d)}), out);
      CHECK(rejectedWith(r, kRejectUnavailable));
    }

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

  // ---- open on a live connection this console does not ride on (an arm-adi one): unavailable cause 6
  // (oep-if-console §1; it answered no_connection) ----
  {
    const uint16_t swd = ResourceNumbers::take(ResourceNumbers::kConnection);
    r = call(console, TargetConsoleStream::kOpOpen, cat(le32(swd), {console.numberOf(TargetConsoleStream::kDriverDmseq)}), out);
    CHECK(rejectedWith(r, kRejectUnavailable) && out == Bytes({reg::core::kTlvUnavailablePayloadCause, 1, 0,
                                                                reg::core::kUnavailableCauseWrongState}));
    ResourceNumbers::close(swd);
    r = call(console, TargetConsoleStream::kOpOpen, cat(le32(swd), {console.numberOf(TargetConsoleStream::kDriverDmseq)}), out);   // closed: unknown
    CHECK(rejectedWith(r, kRejectNoResource));
  }

  // ---- one bad DMSTATUS read with havereset set is no restart (ackHaveReset's own read decides): it unsynced dmseq,
  // which dropped the input chunk on its way - here the one whose answer was lost - and the target never got "RE" of
  // "READ 13" (a command left unanswered, X035 on the P4 under gpio activity); and a DMSTATUS of all ones brings the bus
  // back in step (a link that dropped: it read all ones until wire_lost_ms closed the stream) ----
  {
    static FakePhy phy3;
    static Ch32Dm dm3(phy3);
    static DebugPort port3{dm3, 4, 5};
    static WireRvswd wire3(port3, 2);
    static DmConsole driver3(dm3, phy3);
    static TargetConsoleStream console3(port3, driver3, 2);
    r = call(wire3, WireRvswd::kOpAttach, attachRequest(4, 5), out);
    CHECK(ok(r) && port3.connected);
    r = call(console3, TargetConsoleStream::kOpOpen, cat(le32(port3.number), {console3.numberOf(TargetConsoleStream::kDriverDmseq)}), out);
    CHECK(ok(r));
    const uint32_t s3 = getU32(out.data());
    SeqTarget target(phy3.data0);
    uint32_t next = millis();
    auto run = [&](uint32_t us) {
      for (uint32_t t = 0; t < us; t += 250) {
        advanceMicros(250);
        console3.poll();
        if (int32_t(millis() - next) >= 0) { target.available(); next += 5; }
      }
    };
    run(100 * 1000);   // synced
    const size_t marks_before = marksOf(console3, s3).size();
    const uint8_t line[] = {'R', 'E', 'A', 'D', ' ', '1', '3', '\n'};
    r = call(console3, TargetConsoleStream::kOpWrite, cat(cat(le32(s3), {uint8_t(sizeof line), 0}), Bytes(line, line + sizeof line)), out);
    CHECK(ok(r) && out == Bytes({uint8_t(sizeof line), 0}));
    phy3.lose_data0 = 1;       // the answer that carries "RE" (to the target's next frame) does not land
    for (int i = 0; i < 100 && phy3.lose_data0; ++i) run(250);
    CHECK(phy3.lose_data0 == 0 && target.rx.empty());
    phy3.bad_havereset = 1;    // the next DMSTATUS look reads havereset once
    g_millis += 25;
    run(200 * 1000);
    CHECK(target.rx == std::vector<uint8_t>(line, line + sizeof line));
    CHECK(marksOf(console3, s3).size() == marks_before);   // no restart marked
    CHECK(driver3.resyncs() == 1);
    // all ones: re-synced (reinit), and the console goes on
    const int reinits = phy3.reinits;
    phy3.dropped = true;
    g_millis += 25;
    run(1000);
    CHECK(phy3.reinits > reinits && !phy3.dropped && console3.isOpen());
    target.rx.clear();
    r = call(console3, TargetConsoleStream::kOpWrite, cat(le32(s3), {2, 0, 'o', 'k'}), out);
    CHECK(ok(r));
    run(100 * 1000);
    CHECK(target.rx == std::vector<uint8_t>({'o', 'k'}));
  }

  // ---- the console's write latency (oep-if-console §2): a host's line goes in one write and reaches the target one
  // chunk per target poll (0.0.28: 13-26 ms for a PING / PONG); with the send slot a line took a request per chunk ----
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

  // ---- a paused wire (DmiPhy::backgroundTurn false: the classic ESP32's sampler window): the console's poll reads
  // nothing and writes nothing, the write op still fills the queue, and once the wire is back the line reaches the target whole; every turn given is ended ----
  {
    static FakePhy phy5;
    static Ch32Dm dm5(phy5);
    static DebugPort port5{dm5, 4, 5};
    static WireRvswd wire5(port5, 2);
    static DmConsole driver5(dm5, phy5);
    static TargetConsoleStream console5(port5, driver5, 2);
    CHECK(ok(call(wire5, WireRvswd::kOpAttach, attachRequest(4, 5), out)));
    CHECK(ok(call(console5, TargetConsoleStream::kOpOpen, cat(le32(port5.number), {console5.numberOf(TargetConsoleStream::kDriverDmseq)}), out)));
    const uint32_t s5 = getU32(out.data());
    SeqTarget target(phy5.data0);
    for (int i = 0; i < 200; ++i) { advanceMicros(500); console5.poll(); target.available(); }   // synced
    phy5.paused = true;
    const int reads = phy5.reads;
    const size_t answers = phy5.data0_writes.size();
    const uint8_t line[] = {'B', 'U', 'R', 'S', 'T', ' ', '2', '0', '6', '\n'};
    Bytes req = cat(le32(s5), {uint8_t(sizeof line), 0});
    req.insert(req.end(), line, line + sizeof line);
    r = call(console5, TargetConsoleStream::kOpWrite, req, out);
    CHECK(ok(r) && out == le16(sizeof line));                     // queued whole while the wire is paused
    for (int i = 0; i < 400; ++i) { advanceMicros(500); console5.poll(); target.available(); }   // 200 ms paused
    CHECK(phy5.reads == reads && phy5.data0_writes.size() == answers && target.rx.empty());
    phy5.paused = false;
    for (int i = 0; i < 200 && target.rx.size() < sizeof line; ++i) { advanceMicros(500); console5.poll(); target.available(); }
    CHECK(target.rx == Bytes(line, line + sizeof line));
    CHECK(phy5.turns > 0 && phy5.turns == phy5.turns_done);
  }

  printf("console: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
