// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.target.console (OepConsole.cpp) over DmConsole and Ch32Dm on a fake DMI PHY whose debug module keeps
// DATA0 / DATA1 and DMSTATUS's halted / havereset bits; the test plays the target's side of the mailbox.
// - core §4.3's order: a stream op's form and values are checked before its stream number (no_connection last).
#include <stdio.h>

#include <vector>

#include "OepConsole.h"
#include "OepDmConsole.h"
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

  printf("console: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
