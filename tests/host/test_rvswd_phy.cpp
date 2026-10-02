// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: RvswdPhy's attach and scan bring-up over the RVSWD frames (OepRvswdFrame.h) against a simulated target that
// decodes them from the edges (oep-if-debug §3.1) and keeps the time - each half period spun, plus a per-cell overhead
// like the RP2350's loop (1008 reads at the 500 ns half period took 79 ms on the bench).
// - Before the speed is verified only the wake / configuration pair and dmactive (only when not already set) are
//   written, at the slowest period; the write check writes PROGBUF0 and ABSTRACTAUTO = 0 only, PROGBUF0 put back
//   (oep-if-debug §1, §3). scan's bring-up writes no scratch.
// - The read check ignores the harts' state bits (8-19). At the slowest period an isolated parity error is retried (3 at
//   most); a faster period with one is not used. The slowest period's reads are checked once.
// - The attach budget stops the search (setDeadline); search_retries counts what failed.
// - begin() leaves the pins free: not driven, no pull (oep-core §8).
#include <stdio.h>

#include <set>
#include <vector>

#include "OepRvswdPhy.h"
#include "fake_rvswd_io.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// ---- the target --------------------------------------------------------------------------------------------------
struct Write { uint8_t address; uint32_t value; uint32_t half_ns; };

struct Target {
  // pads
  bool begun = false, driven = false, pulled = false;
  uint32_t half_ns = 0;
  uint32_t cell_overhead_ps = 220000;   // per half period, on top of it (RP2350: about 24 us a frame at 500 ns)
  uint64_t ps = 0;                      // time not yet handed to advanceMicros
  // lines as the probe drives them
  bool clk = true, dio = true, host_drives = true;
  // link
  bool awake = false;
  int wake_cells = 0;
  bool in_frame = false;
  int pos = 0;
  uint8_t address = 0;
  bool write = false, header_parity = false, header_ok = false;
  uint32_t data = 0, out = 0;
  bool data_parity = false, out_parity = false;
  int frames = 0, reads = 0;
  // the debug module
  uint32_t dmcontrol = 0, progbuf0 = 0x12345678u, abstractauto = 1, hart_bits = 3u << 10;   // running
  int cfgr_writes = 0;
  std::vector<Write> writes;
  // faults: reads at a half period under min_read_half fail their parity, writes under min_write_half land garbled;
  // reads whose number is in bad_reads fail their parity; flip_every: the harts' state bits change every n reads
  uint32_t min_read_half = 0, min_write_half = 0;
  std::set<int> bad_reads;
  int flip_every = 0;

  void reset() { *this = Target(); }
  void tick() {
    ps += uint64_t(half_ns) * 1000u + cell_overhead_ps;
    if (ps >= 1000000) { advanceMicros(static_cast<uint32_t>(ps / 1000000)); ps %= 1000000; }
  }
  uint32_t dmstatus() const { return 0x00400082u | hart_bits; }   // version 2, authenticated, impebreak
  uint32_t readReg(uint8_t a) {
    switch (a) {
      case 0x10: return dmcontrol;
      case 0x11: return dmstatus();
      case 0x18: return abstractauto;
      case 0x20: return progbuf0;
      case 0x7d: case 0x7e: return 0x5aa50400u;
      case 0x7f: return 0x10350000u;
      default: return 0;
    }
  }
  void writeReg(uint8_t a, uint32_t v) {
    if (half_ns < min_write_half) v ^= 0x00010000u;   // garbled
    writes.push_back({a, v, half_ns});
    if (a == 0x10) dmcontrol = v;
    if (a == 0x18) abstractauto = v;
    if (a == 0x20) progbuf0 = v;
    if (a == 0x7d || a == 0x7e) ++cfgr_writes;
  }
  bool parityOf(uint32_t v) { bool p = false; while (v) { p ^= v & 1; v >>= 1; } return p; }
  // a rising edge of SWCLK: the cell at pos
  void rise() {
    if (!driven) return;
    if (!in_frame) {
      wake_cells = dio ? wake_cells + 1 : (wake_cells >= 100 ? (awake = true, 0) : 0);
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
        if (flip_every && reads % flip_every == 0) hart_bits ^= (3u << 8) | (3u << 10);
        out_parity = parityOf(out);
        if (bad_reads.count(reads) || half_ns < min_read_half) out_parity = !out_parity;
      }
    } else if (pos >= 14 && pos <= 45 && write) { data = (data << 1) | bit; data_parity ^= bit; }
    else if (pos == 46 && write && header_ok && awake && data_parity == dio) writeReg(address, data);
    (void)bit;
    ++pos;
  }
  void dioChanged(bool from, bool to) {
    if (!driven || !clk || from == to) return;
    if (!to) {   // START
      in_frame = true; ++frames; pos = 0; address = 0; header_parity = false; data = 0; data_parity = false;
      wake_cells = 0;
    } else {
      in_frame = false;   // STOP
    }
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
bool FakeRvswdIo::begin(int, int) const { t.begun = true; t.pulled = true; t.driven = false; return true; }   // the pull-up a setup gives
void FakeRvswdIo::pullUp(bool on) const { t.pulled = on; }
void FakeRvswdIo::driveBoth(bool on) const { t.driven = on; }
uint32_t FakeRvswdIo::setHalfNs(uint32_t half_ns) const { t.half_ns = half_ns; return half_ns; }

// ---- checks ------------------------------------------------------------------------------------------------------
// Writes before the first PROGBUF0 write (the write check): the configuration pair and DMCONTROL = 1, all at the slowest
// half period; ABSTRACTAUTO = 0 only right before the check; after the attach PROGBUF0 holds its old value.
static bool onlyAllowedBeforeCheck(uint32_t slowest, bool dmactive_allowed) {
  for (size_t i = 0; i < t.writes.size(); ++i) {
    const Write &w = t.writes[i];
    if (w.address == 0x20) return true;
    if (w.address == 0x18 && w.value == 0 && i + 1 < t.writes.size() && t.writes[i + 1].address == 0x20) continue;
    const bool cfgr = (w.address == 0x7d || w.address == 0x7e) && w.value == 0x5aa50400u;
    const bool active = w.address == 0x10 && w.value == 1 && dmactive_allowed;
    if (!(cfgr || active)) { printf("  write 0x%02x = 0x%08x before the check\n", w.address, w.value); return false; }
    if (w.half_ns != slowest) { printf("  write 0x%02x at %u ns before the check\n", w.address, w.half_ns); return false; }
  }
  return true;
}
static int dmstatusReads() { return t.reads; }

int main() {
  RvswdPhy phy;

  // ---- boot: free, like a release (oep-core §8) ----
  CHECK(phy.begin(2, 3));
  CHECK(t.begun && !t.driven && !t.pulled);

  // ---- a clean attach at max_speed 1 MHz: only the slowest period is a candidate ----
  {
    phy.setMaxHz(1000000);
    const uint32_t before_us = micros();
    phy.clearSearchRetries();
    CHECK(phy.attach());
    const uint32_t took_us = micros() - before_us;
    printf("  attach at 1 MHz: %u.%03u ms, %d reads, %zu writes, %d frames\n", took_us / 1000, took_us % 1000,
           dmstatusReads(), t.writes.size(), t.frames);
    CHECK(onlyAllowedBeforeCheck(500, true));
    CHECK(t.progbuf0 == 0x12345678u);   // put back
    CHECK(t.abstractauto == 0);         // made free, not restored
    CHECK(phy.searchRetries() == 0);
    CHECK(t.reads < 1300);              // one read check of the slowest period (0.0.28 ran it twice: 2016)
    CHECK(took_us < 150000);            // 0.0.28: about 200 ms of checks at this period
    CHECK(t.pulled && t.driven);
    phy.free();
    CHECK(!t.pulled && !t.driven);
  }

  // ---- dmactive already set (a halted target): DMCONTROL is not written (the write would clear haltreq) ----
  {
    t.reset(); t.begun = true;
    t.dmcontrol = 0x80000001u;
    CHECK(phy.attach());
    bool wrote_control = false;
    for (const Write &w : t.writes) wrote_control |= w.address == 0x10;
    CHECK(!wrote_control && t.dmcontrol == 0x80000001u);
    CHECK(onlyAllowedBeforeCheck(500, false));
    phy.free();
  }

  // ---- the harts' state changes during the read check: still one attach (F2) ----
  {
    t.reset(); t.begun = true;
    t.flip_every = 7;
    phy.clearSearchRetries();
    CHECK(phy.attach());
    CHECK(phy.searchRetries() == 0);
    phy.free();
  }

  // ---- isolated parity errors at the slowest period: up to 3 retried, the attach goes on (F4) ----
  {
    t.reset(); t.begun = true;
    t.bad_reads = {50, 400, 900};
    phy.clearSearchRetries();
    const uint32_t before = micros();
    CHECK(phy.attach());
    CHECK(phy.searchRetries() == 3);
    CHECK(micros() - before < 150000);   // no second pass
    phy.free();
    // four in one check: that pass fails, the second one attaches
    t.reset(); t.begun = true;
    t.bad_reads = {50, 51, 52, 53};
    phy.clearSearchRetries();
    CHECK(phy.attach());
    CHECK(phy.searchRetries() >= 4);
    CHECK(t.progbuf0 == 0x12345678u);
    phy.free();
  }

  // ---- no ceiling: faster periods by reads only; one with a parity error is not used, writes there checked ----
  {
    t.reset(); t.begun = true;
    t.min_read_half = 50;   // 25 ns and 0 ns fail
    phy.setMaxHz(0);
    phy.clearSearchRetries();
    CHECK(phy.attach());
    CHECK(phy.halfNs() == 50);
    CHECK(onlyAllowedBeforeCheck(500, true));
    CHECK(t.progbuf0 == 0x12345678u);
    phy.free();
    // the writes at 50 ns do not land: 100 ns is proved next, the scratch put back at the slowest period
    t.reset(); t.begun = true;
    t.min_read_half = 50;
    t.min_write_half = 100;
    phy.clearSearchRetries();
    CHECK(phy.attach());
    CHECK(phy.halfNs() == 100);
    CHECK(phy.searchRetries() == 1);
    CHECK(t.progbuf0 == 0x12345678u);
    phy.free();
    // an isolated error at a faster period ends the search there (no retries above the slowest)
    t.reset(); t.begun = true;
    t.bad_reads = {1100};   // in the 200 ns check
    CHECK(phy.attach());
    CHECK(phy.halfNs() == 500);
    phy.free();
  }

  // ---- a slow ceiling: the checks shorten so the attach stays inside the budget ----
  {
    t.reset(); t.begun = true;
    phy.setMaxHz(50000);   // 10 us half period
    const uint32_t before = micros();
    CHECK(phy.attach());
    printf("  attach at 50 kHz: %u ms\n", (micros() - before) / 1000);
    CHECK(micros() - before < 750000);
    phy.free();
    phy.setMaxHz(1000000);
  }

  // ---- the attach budget: no target, the search stops at the deadline ----
  {
    t.reset(); t.begun = true;
    t.min_read_half = 100000;   // every read fails
    phy.clearSearchRetries();
    const uint32_t before = millis();
    phy.setDeadline(millis() + 5);
    CHECK(!phy.attach());
    phy.clearDeadline();
    CHECK(millis() - before < 50);
    phy.free();
  }

  // ---- scan's bring-up: the wake / configuration pair and dmactive only, released after ----
  {
    t.reset(); t.begun = true;
    uint32_t status = 0;
    CHECK(phy.bringUp(status));
    CHECK((status & 0xf) == 2);
    bool other = false;
    for (const Write &w : t.writes) other |= !(w.address == 0x7d || w.address == 0x7e || (w.address == 0x10 && w.value == 1));
    CHECK(!other);
    CHECK(t.abstractauto == 1 && t.progbuf0 == 0x12345678u);
    CHECK(!phy.attached() && !t.driven);
    // dmactive set: not written
    t.reset(); t.begun = true;
    t.dmcontrol = 0x80000001u;
    CHECK(phy.bringUp(status));
    for (const Write &w : t.writes) CHECK(w.address != 0x10);
  }

  printf("rvswd-phy: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
