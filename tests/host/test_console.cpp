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
static Bytes attachRequest() { return {0, uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 4, 0x40, 0x42, 0x0f, 0x00}; }

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

  printf("console: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
