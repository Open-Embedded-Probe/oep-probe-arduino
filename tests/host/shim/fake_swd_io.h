// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests (OEP_HOST_FAKE_SWD): rp2::BitBang over a simulated SWD target, in place of OepRp2BitBang.h, so OepSwd.cpp
// runs on the host. The target sees each clock cell at SWCLK's rising edge: a bit the host drives, or one it samples
// (the target's output is decided before the host reads it). It knows the line reset (50 ones or more; then only a
// DPIDR read is taken until one comes), requests with their parity, the ACK, read data with parity, write data, and
// TARGETSEL (no ACK). DP registers DPIDR / CTRL/STAT / SELECT / RDBUFF; AP reads are posted (an AP read answers the
// previous AP read's value) from four AP registers. Faults to test with: `absent` (nothing answers), `drop_requests`
// (that many requests get no reply, as a line that lost step), `parity_reads` (that many read data phases go out with
// a wrong parity). Each half period moves the host clock on (advanceMicros), so time-bounded loops end.
#pragma once
#include <Arduino.h>
#include <stdint.h>

#include <deque>

inline void gpio_disable_pulls(int) {}

struct FakeSwdTarget {
  bool absent = false;
  int drop_requests = 0, parity_reads = 0;
  bool multidrop = false;          // only answers after TARGETSEL == targetsel following a line reset
  uint32_t targetsel = 0;
  uint32_t dpidr = 0x0bc12477u;
  uint32_t ctrl = 0, select = 0, rdbuff = 0;
  uint32_t ap[4] = {0x11, 0x22, 0x33, 0x44};
  uint32_t ap_reads = 0, ap_writes = 0, requests = 0, line_resets = 0;
  // state
  bool locked = true, selected = true;
  int ones = 0;
  enum Phase { kIdle, kRequest, kHostData } phase = kIdle;
  uint8_t req = 0;
  int req_bits = 0, data_bits = 0;
  uint64_t data_in = 0;
  bool data_is_targetsel = false;
  std::deque<int> out;             // the target's bits for the cells the host samples (-1: not driven)

  int output() const { return out.empty() ? -1 : out.front(); }
  void sampled() { if (!out.empty()) out.pop_front(); }
  static bool parity(uint32_t v) { v ^= v >> 16; v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1; return v & 1; }
  void pushBits(uint32_t v, int n) { for (int i = 0; i < n; ++i) out.push_back((v >> i) & 1); }

  void hostBit(bool v) {
    if (v) { if (++ones >= 50) { if (ones == 50) { ++line_resets; locked = true; if (multidrop) selected = false; } phase = kIdle; out.clear(); } }
    else ones = 0;
    switch (phase) {
      case kIdle:
        if (v && ones < 50) { phase = kRequest; req = 1; req_bits = 1; }
        break;
      case kRequest:
        req |= static_cast<uint8_t>(v << req_bits);
        if (++req_bits == 8) { phase = kIdle; request(); }
        break;
      case kHostData:
        data_in |= static_cast<uint64_t>(v) << data_bits;
        if (++data_bits == 33) { phase = kIdle; written(); }
        break;
    }
  }
  void request() {
    const bool ap_ = req & 2, read = req & 4;
    const uint8_t a = (req >> 3) & 3;
    const bool par = ((req >> 1) ^ (req >> 2) ^ (req >> 3) ^ (req >> 4)) & 1;
    if (!(req & 1) || (req & 0x40) || !(req & 0x80) || par != ((req >> 5) & 1)) return;   // not a request
    ++requests;
    if (!ap_ && !read && a == 3) {   // TARGETSEL: nobody answers, the data is taken
      data_is_targetsel = true;
      phase = kHostData; data_bits = 0; data_in = 0;
      out.push_back(-1); out.push_back(-1); out.push_back(-1); out.push_back(-1); out.push_back(-1);   // turn + ack + turn
      return;
    }
    if (absent || !selected) return;   // nothing driven: the host reads the pull-up (all ones)
    if (drop_requests > 0) { --drop_requests; return; }
    if (locked && !(read && !ap_ && a == 0)) return;   // after a line reset: DPIDR first
    out.push_back(-1);                                    // turnaround
    pushBits(1, 3);                                       // OK
    if (read) {
      uint32_t v = 0;
      if (ap_) { v = rdbuff; rdbuff = ap[a]; ++ap_reads; }
      else if (a == 0) { v = dpidr; locked = false; }
      else if (a == 1) v = ctrl;
      else if (a == 3) v = rdbuff;
      bool p = parity(v);
      if (parity_reads > 0) { --parity_reads; p = !p; }
      pushBits(v, 32);
      out.push_back(p);
      out.push_back(-1);                                  // turnaround
    } else {
      out.push_back(-1);                                  // turnaround, then the host's 33 bits
      data_is_targetsel = false;
      phase = kHostData; data_bits = 0; data_in = 0;
      write_ap = ap_; write_a = a;
    }
  }
  bool write_ap = false;
  uint8_t write_a = 0;
  void written() {
    const uint32_t v = static_cast<uint32_t>(data_in);
    if (data_is_targetsel) { selected = !multidrop || v == targetsel; data_is_targetsel = false; return; }
    if (write_ap) { ap[write_a] = v; ++ap_writes; }
    else if (write_a == 1) ctrl = v;
    else if (write_a == 2) select = v;
  }
};
inline FakeSwdTarget g_swd;

namespace oep {
namespace rp2 {
struct BitBang {
  uint32_t loops = 0, half_ns = 0;
  mutable bool host_drives = true, clk_low = false, dio_out = true;
  mutable bool sampling = false;   // the host read the line in this cell
  void spin() const { advanceMicros(half_ns >= 1000 ? half_ns / 1000 : 0); tick_ns += half_ns % 1000; if (tick_ns >= 1000) { advanceMicros(1); tick_ns -= 1000; } }
  void bothHigh() const { dio_out = true; }
  void clkLowDio(bool v) const { clk_low = true; dio_out = v; }
  void clkHigh() const { edge(); }
  void clk(bool v) const { if (v) edge(); else clk_low = true; }
  void dio(bool v) const { dio_out = v; }
  bool dioRead() const {
    sampling = true;
    if (host_drives) return dio_out;
    const int b = g_swd.output();
    return b < 0 ? true : b != 0;   // undriven: the pull-up
  }
  void hostDrives(bool yes) const { host_drives = yes; }
  bool setup(int, int) { return true; }
  void driveBoth() const {}
  void releaseBoth() const {}
  uint32_t setHalfNs(uint32_t ns) { half_ns = ns; loops = ns; return ns; }

 private:
  mutable uint32_t tick_ns = 0;
  void edge() const {
    if (!clk_low) return;
    clk_low = false;
    if (host_drives) g_swd.hostBit(dio_out);
    else g_swd.sampled();
    sampling = false;
  }
};
}  // namespace rp2
}  // namespace oep
