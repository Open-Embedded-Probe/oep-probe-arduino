// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: attach / detach on oep.wire.rvswd through the whole stack below the endpoint - WireRvswd -> Ch32Dm ->
// RvswdPhy (OEP_HOST_FAKE_RVSWD) -> a simulated target that decodes the RVSWD frames and keeps the time (as
// test_rvswd_phy.cpp's, plus haltreq / resumereq).
// - Every attach without a live connection is a fresh bring-up: flags bit1 clear, search_retries (TLV 0x12) present; a
//   connection a slot shares outlives the host's detach, and the next attach joins it (flags bit1, no TLV 0x12).
// - The attach budget (oep-if-debug §1, limits.attach_budget_ms) is a hard bound on one attach answer, a reset's hold_ms
//   aside, in the worst cases the simulation can make (writes that never land, reads that fail at the faster periods,
//   a slow max_speed, the reset TLV with halt): 0.0.28 took up to 4.4 s.
// - A max_speed under describe's min_clock_hz (50 kHz) is refused unsupported with the tag as received: under it one
//   attach's minimum checks cannot fit the budget.
#include <stdio.h>

#include <set>
#include <vector>

#include "OepCh32Dm.h"
#include "OepPinTable.h"
#include "OepRvswdPhy.h"
#include "OepTarget.h"
#include "fake_rvswd_io.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace wire = reg::wire_rvswd;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                                          \
  do {                                                                                       \
    ++checks;                                                                                \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); }      \
  } while (0)

struct Target {
  bool begun = false, driven = false, pulled = false;
  uint32_t half_ns = 0;
  uint32_t cell_overhead_ps = 220000;
  uint64_t ps = 0;
  bool clk = true, dio = true, host_drives = true;
  bool awake = false;
  int wake_cells = 0, wakes = 0;
  bool in_frame = false;
  int pos = 0;
  uint8_t address = 0;
  bool write = false, header_parity = false, header_ok = false;
  uint32_t data = 0, out = 0;
  bool data_parity = false, out_parity = false;
  int frames = 0, reads = 0;
  uint32_t dmcontrol = 0, progbuf0 = 0x12345678u, abstractauto = 1, hart_bits = 3u << 10;
  uint32_t min_read_half = 0, min_write_half = 0;
  void tick() {
    ps += uint64_t(half_ns) * 1000u + cell_overhead_ps;
    if (ps >= 1000000) { advanceMicros(static_cast<uint32_t>(ps / 1000000)); ps %= 1000000; }
  }
  uint32_t dmstatus() const { return 0x00400082u | hart_bits; }
  uint32_t readReg(uint8_t a) {
    switch (a) {
      case 0x10: return dmcontrol;
      case 0x11: return dmstatus();
      case 0x12: return 0x0002'1000u | 0x380;
      case 0x16: return 2;
      case 0x18: return abstractauto;
      case 0x20: return progbuf0;
      case 0x7d: case 0x7e: return 0x5aa50400u;
      case 0x7f: return 0x10350000u;
      default: return 0;
    }
  }
  void writeReg(uint8_t a, uint32_t v) {
    if (half_ns < min_write_half) v ^= 0x00010000u;
    if (a == 0x10) {
      dmcontrol = v;
      if (v & (1u << 31)) hart_bits = 3u << 8;                    // haltreq: halted
      else if (v & (1u << 30)) hart_bits = (3u << 10) | (3u << 16);   // resumereq
    }
    if (a == 0x18) abstractauto = v;
    if (a == 0x20) progbuf0 = v;
  }
  bool parityOf(uint32_t v) { bool p = false; while (v) { p ^= v & 1; v >>= 1; } return p; }
  void rise() {
    if (!driven) return;
    if (!in_frame) {
      wake_cells = dio ? wake_cells + 1 : (wake_cells >= 100 ? (awake = true, ++wakes, 0) : 0);
      return;
    }
    const bool bit = host_drives ? dio : true;
    if (pos <= 6) { address = static_cast<uint8_t>((address << 1) | dio); header_parity ^= dio; }
    else if (pos == 7) { write = dio; header_parity ^= dio; }
    else if (pos == 8) {
      header_ok = header_parity == dio;
      if (!write) {
        ++reads;
        out = readReg(address);
        out_parity = parityOf(out);
        if (half_ns < min_read_half) out_parity = !out_parity;
      }
    } else if (pos >= 14 && pos <= 45 && write) { data = (data << 1) | bit; data_parity ^= bit; }
    else if (pos == 46 && write && header_ok && awake && data_parity == dio) writeReg(address, data);
    ++pos;
  }
  void dioChanged(bool from, bool to) {
    if (!driven || !clk || from == to) return;
    if (!to) { in_frame = true; ++frames; pos = 0; address = 0; header_parity = false; data = 0; data_parity = false; wake_cells = 0; }
    else in_frame = false;
  }
  bool targetBit() const {
    if (!awake || !header_ok || write || !in_frame) return pulled;
    if (pos >= 14 && pos <= 45) return (out >> (45 - pos)) & 1;
    if (pos == 46) return out_parity;
    return pulled;
  }
};
static Target t;

void FakeRvswdIo::spin() const { t.tick(); }
void FakeRvswdIo::bothHigh() const {
  if (!t.clk) { t.clk = true; t.rise(); }
  const bool was = t.dio; t.dio = true; t.dioChanged(was, true);
}
void FakeRvswdIo::clkLowDio(bool v) const { t.clk = false; t.dio = v; }
void FakeRvswdIo::clkHigh() const { if (!t.clk) { t.clk = true; t.rise(); } }
void FakeRvswdIo::clk(bool v) const { if (v) clkHigh(); else t.clk = false; }
void FakeRvswdIo::dio(bool v) const { const bool was = t.dio; t.dio = v; t.dioChanged(was, v); }
bool FakeRvswdIo::dioRead() const { return t.host_drives ? t.dio : t.targetBit(); }
void FakeRvswdIo::hostDrives(bool yes) const { t.host_drives = yes; }
bool FakeRvswdIo::begin(int, int) const { t.begun = true; t.pulled = true; t.driven = false; return true; }
void FakeRvswdIo::pullUp(bool on) const { t.pulled = on; }
void FakeRvswdIo::clkPull(int) const {}
void FakeRvswdIo::driveBoth(bool on) const { t.driven = on; }
uint32_t FakeRvswdIo::setHalfNs(uint32_t half_ns) const { t.half_ns = half_ns; return half_ns; }

static Result call(Interface &i, uint8_t op, const Bytes &payload, Bytes &out, size_t capacity = 1024 - 5) {
  out.assign(capacity, 0);
  const Result r = i.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
// what riscv.Wire.attach_body sends: method, max_speed (critical), pins (critical), idle_clock low (critical); every
// TLV tag(u8) len(u16) value (core §2.2)
static Bytes attachRequest(uint8_t method, uint32_t hz, int swdio, int swclk, bool idle_low) {
  Bytes p = {method, uint8_t(wire::kTlvAttachMaxSpeed | kTagCritical), 4, 0, uint8_t(hz), uint8_t(hz >> 8),
             uint8_t(hz >> 16), uint8_t(hz >> 24)};
  p.insert(p.end(), {uint8_t(wire::kTlvAttachPins | kTagCritical), 4, 0, uint8_t(swdio), 0, uint8_t(swclk), 0});
  if (idle_low) p.insert(p.end(), {uint8_t(wire::kTlvAttachIdleClock | kTagCritical), 1, 0, 1});
  return p;
}
static Bytes detachRequest(uint16_t number) { return {uint8_t(number), uint8_t(number >> 8)}; }
static const uint8_t *answerTlv(const Bytes &out, size_t from, uint8_t tag, size_t &len) {
  for (size_t at = from; at + 3 <= out.size(); at += 3u + (out[at + 1] | out[at + 2] << 8)) {
    const size_t n = out[at + 1] | out[at + 2] << 8;
    if (out[at] == tag && at + 3u + n <= out.size()) { len = n; return out.data() + at + 3; }
  }
  return nullptr;
}

int main() {
  static RvswdPhy phy;
  static Ch32Dm dm(phy);
  static PinTable pins((1ull << 30) - 1);
  static DebugPort port{dm, 0xfffe, 0xfffe};
  static WireRvswd w(port, 0);
  port.pin_choice = (1ull << 30) - 1;
  port.pins = &pins;
  port.reset_allowed = port.pin_choice;
  Bytes out;
  const uint32_t kHz = 1000000;   // max_speed: what the bench's L103 attach asks (any; only the slowest periods qualify)

  // ---- A: no slot, attach / detach x N with a short and a long gap ----
  for (int halt = 0; halt < 2; ++halt) {
    for (uint32_t gap_ms : {50u, 200u, 1000u, 3000u}) {
      uint32_t took_min = ~0u, took_max = 0;
      for (int k = 0; k < 5; ++k) {
        g_millis += gap_ms;
        const uint32_t t0 = micros();
        phy.beginRequest();
        Result r = call(w, WireRvswd::kOpAttach, attachRequest(uint8_t(halt), kHz, 0, 1, true), out);
        const uint32_t took = micros() - t0;
        took_min = took < took_min ? took : took_min;
        took_max = took > took_max ? took : took_max;
        CHECK(ok(r) && out.size() >= 11);
        CHECK(!(out[6] & wire::kAttachFlagsExisting));
        size_t len = 0;
        const uint8_t *v = answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len);
        CHECK(v && len == 2);
        r = call(w, WireRvswd::kOpDetach, detachRequest(port.number), out);
        CHECK(ok(r) && !port.connected);
      }
      printf("  A %s gap %4u ms: attach %u.%03u-%u.%03u ms (sim), search_retries present\n", halt ? "halt" : "run ",
             gap_ms, took_min / 1000, took_min % 1000, took_max / 1000, took_max % 1000);
    }
  }

  // ---- B: a slot shares the connection: the host's detach leaves it up ----
  {
    Result r = call(w, WireRvswd::kOpAttach, attachRequest(0, kHz, 0, 1, true), out);
    CHECK(ok(r) && !(out[6] & wire::kAttachFlagsExisting));
    size_t len = 0;
    CHECK(answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len) != nullptr);
    port.users |= DebugPort::kUserSlot;   // as a slot's automatic attach / a bind's console would have it
    r = call(w, WireRvswd::kOpDetach, detachRequest(port.number), out);
    CHECK(ok(r) && port.connected);       // the host's use goes; the link stays
    g_millis += 50;
    const uint32_t t0 = micros();
    r = call(w, WireRvswd::kOpAttach, attachRequest(0, kHz, 0, 1, true), out);
    const uint32_t took = micros() - t0;
    CHECK(ok(r) && (out[6] & wire::kAttachFlagsExisting));
    CHECK(answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len) == nullptr);
    printf("  B slot-shared: second attach flags 0x%02x (existing), no TLV 0x12, %u.%03u ms (sim)\n", out[6],
           took / 1000, took % 1000);
    r = call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);   // force
    CHECK(ok(r) && !port.connected);
  }

  // ---- C: the answer at a small capacity: where TLV 0x12 would be dropped silently (the fixed part 11 bytes, then
  // target_id 8, dpc 7 and search_retries 5 with their 3-byte headers) ----
  {
    for (size_t cap : {11u, 19u, 26u, 30u, 31u}) {
      Result r = call(w, WireRvswd::kOpAttach, attachRequest(1, kHz, 0, 1, true), out, cap);
      size_t len = 0;
      printf("  C capacity %2zu: %s, %zu bytes, TLV 0x12 %s\n", cap, ok(r) ? "ok" : "not ok", out.size(),
             answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len) ? "present" : "absent");
      if (port.connected) call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);
    }
  }

  // ---- D: the attach budget - how long one attach request takes on the probe, worst cases the sim can make ----
  {
    struct Case { const char *name; uint32_t hz; uint32_t min_read, min_write; uint8_t method; };
    const Case cases[] = {
        {"writes never land, 1 MHz", 1000000, 0, 100000, 0},
        {"writes never land, 20 kHz", 20000, 0, 100000, 0},
        {"clean, 20 kHz", 20000, 0, 0, 1},
        {"clean, 5 kHz", 5000, 0, 0, 1},
        {"fast reads fail, 10 MHz", 10000000, 400, 0, 1},
        {"writes never land, 5 kHz", 5000, 0, 100000, 0},
        {"clean, 10 MHz", 10000000, 0, 0, 0},
        {"clean, 2.5 MHz (200 ns ok)", 2500000, 0, 0, 0},
        {"10 MHz, reads ok >=100ns, writes ok >=500", 10000000, 100, 500, 0},
        {"10 MHz, reads ok >=200ns", 10000000, 200, 0, 0},
        {"clean, 2 kHz", 2000, 0, 0, 1},
        {"clean, 1 kHz", 1000, 0, 0, 1},
        {"writes never land, 10 kHz", 10000, 0, 100000, 1},
        {"clean, 50 kHz", 50000, 0, 0, 1},
        {"writes never land, 50 kHz", 50000, 0, 100000, 1},
    };
    for (const Case &c : cases) {
      t.min_read_half = c.min_read;
      t.min_write_half = c.min_write;
      phy.beginRequest();
      const uint32_t t0 = micros();
      Result r = call(w, WireRvswd::kOpAttach, attachRequest(c.method, c.hz, 0, 1, true), out);
      const uint32_t took = micros() - t0;
      size_t len = 0;
      const uint8_t *v = ok(r) && out.size() > 11 ? answerTlv(out, 11, wire::kTlvAttachAnswerSearchRetries, len) : nullptr;
      printf("  D %-28s %s  %4u.%03u ms (sim)%s%u\n", c.name, ok(r) && out.size() >= 11 && out[0] != 0xff ? "answer" : "status",
             took / 1000, took % 1000, v ? "  search_retries " : "", v ? unsigned(v[0] | v[1] << 8) : 0u);
      if (c.hz < RvswdPhy::kMinClockHz)   // under min_clock_hz: unsupported, the tag as received, nothing run
        CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out.size() == 1 &&
              out[0] == (wire::kTlvAttachMaxSpeed | kTagCritical) && took == 0);
      else
        CHECK(r.resolution == kResolutionCompleted && took <= reg::kLimitAttachBudgetMs * 1000u);
      if (port.connected) call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);
    }
    t.min_read_half = t.min_write_half = 0;
  }

  // ---- E: attach with the reset TLV (channel 5, hold 20 ms), halt and run ----
  for (uint32_t hz : {1000000u, 10000000u, 50000u, 20000u}) {
    for (uint8_t method : {uint8_t(1), uint8_t(0)}) {
      Bytes req = attachRequest(method, hz, 0, 1, true);
      req.insert(req.end(), {uint8_t(wire::kTlvAttachReset | kTagCritical), 4, 0, 5, 0, 20, 0});
      phy.beginRequest();
      const uint32_t t0 = micros();
      Result r = call(w, WireRvswd::kOpAttach, req, out);
      const uint32_t took = micros() - t0;
      printf("  E reset 20 ms, %s, %8u Hz: %s %u.%03u ms (sim)\n", method ? "halt" : "run ", hz,
             ok(r) && out.size() >= 11 ? "answer" : "status/reject", took / 1000, took % 1000);
      if (hz < RvswdPhy::kMinClockHz) CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported);
      else CHECK(ok(r) && took <= (reg::kLimitAttachBudgetMs + 20) * 1000u);   // the budget, the hold aside
      if (port.connected) call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);
    }
  }

  // ---- F: the target's power gone and back under a live connection: attach answers the same connection, its link
  // woken again. On RVSWD a read of the sleeping link fails its parity and the PHY's re-sync wakes it inside the read;
  // on SWIO the line reads all ones with no parity to fail, and 0.0.28+68d9694 answered timeout until a forced detach
  // (P4 + CH32V003 bench) - WireRvswd::attach now brings the link up afresh (test_wire's stale PHY). ----
  for (uint32_t off_ms : {300u, 2000u}) {
    phy.beginRequest();
    Result r = call(w, WireRvswd::kOpAttach, attachRequest(0, kHz, 0, 1, true), out);
    CHECK(ok(r) && port.connected);
    const uint16_t number = port.number;
    t.awake = false;                          // power gone: the line rests on its pull-up
    g_millis += off_ms;
    t.dmcontrol = 0;                          // power back: a fresh module, the hart running, the link asleep
    t.hart_bits = 3u << 10;
    for (uint8_t method : {uint8_t(0), uint8_t(1)}) {
      phy.beginRequest();
      const uint32_t t0 = micros();
      r = call(w, WireRvswd::kOpAttach, attachRequest(method, kHz, 0, 1, true), out);
      const uint32_t took = micros() - t0;
      CHECK(ok(r) && out.size() >= 11 && port.connected && port.number == number && (out[6] & wire::kAttachFlagsExisting));
      CHECK(took <= (reg::kLimitAttachBudgetMs + 20) * 1000u);
      if (method == 1) CHECK(out[6] & wire::kAttachFlagsHalted);
      printf("  F power off %4u ms, attach %s: %s %u.%03u ms (sim)\n", off_ms, method ? "halt" : "run ",
             ok(r) ? "answer" : "status", took / 1000, took % 1000);
    }
    call(w, WireRvswd::kOpDetach, {uint8_t(port.number), uint8_t(port.number >> 8), 0x01, 0, 0}, out);
    CHECK(!port.connected);
  }

  printf("attach-cycle: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
