// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The HID transport's reports (transports §1, the HID rules): a report carries count(u16), first(u16) and count bytes
// of the length-prefixed stream, the rest zero. first is where the first frame starting in the report begins
// (kHidNoStart when none). An OS HID driver drops reports read too late, and a sender may drop one it cannot send: a
// receiver whose own reading puts the next frame's start elsewhere than first has lost one, and reads on from first.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "OepRegistry.h"

namespace oep {

constexpr size_t kHidHead = v1::reg::kUsbHidReportHeaderBytes;                         // count, first
constexpr uint16_t kHidNoStart = static_cast<uint16_t>(v1::reg::kUsbHidNoFrameStart);

// Where the frames of a length-prefixed stream start, counted as the bytes go by (nothing kept).
class HidFrameStarts {
 public:
  // The position in d[0, n) where the next frame starts, or kHidNoStart when the frame begun runs past it.
  uint16_t next(const uint8_t *d, size_t n) const;
  void advance(const uint8_t *d, size_t n);
  uint32_t partial() const { return got_; }   // bytes of the frame begun
  void reset() { have_ = 0, left_ = 0, got_ = 0; }

 private:
  uint8_t have_ = 0;   // 0 at a frame's start, 1 its length's low byte read (lo_), 2 in its body (left_ to go)
  uint8_t lo_ = 0;
  uint32_t left_ = 0, got_ = 0;
};

// One input report into out[0, kHidHead + room): count bytes of `data` (count <= room), first by `starts` (which then
// counts them), zero padding. Returns the report's length (kHidHead + room).
size_t hidPackReport(uint8_t *out, size_t room, const uint8_t *data, size_t count, HidFrameStarts &starts);

// The output reports' bytes for the endpoint: one producer (take(), the USB side) and one consumer (the endpoint's
// available() / read()). After a report found missing or broken (rules 6, 7), or one whose bytes do not fit, the bytes
// read on from the next first come after a mark: the consumer reads up to it, then takeRestart() is true once and its
// frame reader starts over (the frame begun is dropped). Marks wait in a small queue; while it is full the reports
// are dropped (the stream is broken anyway) and the reading goes on once the consumer has passed one.
class HidIn {
 public:
  HidIn(uint8_t *ring, size_t size) : ring_(ring), size_(size) {}

  // One output report after its report ID: its count bytes in, by the HID rules 6 and 7.
  void take(const uint8_t *d, size_t len);
  size_t available() const;
  size_t read(uint8_t *out, size_t n);
  int read() {
    uint8_t c;
    return read(&c, 1) ? c : -1;
  }
  int peek() const;
  // True once when the consumer has reached a mark: its frame reader starts over before reading on (inline: the
  // endpoint calls it).
  bool takeRestart() {
    const uint32_t t = __atomic_load_n(&mark_tail_, __ATOMIC_RELAXED);
    if (t == __atomic_load_n(&mark_head_, __ATOMIC_ACQUIRE)) return false;
    if (marks_[t % kMarks] != __atomic_load_n(&consumed_, __ATOMIC_RELAXED)) return false;
    __atomic_store_n(&mark_tail_, t + 1, __ATOMIC_RELEASE);
    return true;
  }
  // The consumer's frame reader found the stream broken (a length over max_frame): from the next report with a first.
  void askBreak() { __atomic_store_n(&break_asked_, true, __ATOMIC_RELEASE); }
  size_t room() const;
  uint32_t losses() const { return losses_; }   // reports found missing or broken

 private:
  static constexpr uint32_t kMarks = 4;
  bool marksFull() const;
  void broken();
  uint32_t limit() const;   // what the consumer may read up to: the bytes in, or the next mark

  uint8_t *ring_;
  size_t size_;
  uint32_t produced_ = 0, consumed_ = 0;   // totals (u32, wrapping: differences only)
  uint32_t marks_[kMarks] = {};
  uint32_t mark_head_ = 0, mark_tail_ = 0;
  HidFrameStarts starts_;
  bool synced_ = false;   // just opened: read from the first report with a first
  bool owe_mark_ = false;   // broken while read: a mark before the bytes read on from the next first
  bool break_asked_ = false;
  uint32_t losses_ = 0;
};

}  // namespace oep
