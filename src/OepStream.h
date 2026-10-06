// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The position-addressed byte stream behind oep.target.console and oep.fixture.uart (oep-spec
// docs/oep-if-common.ja.md §1): bytes kept in a ring that reads do not consume,
// addressed by a u64 position (it does not wrap in practice), and marks with their own u32 serial numbers. One of the
// standard interfaces' shared forms (the capture uses the same u64 positions).
//
//   read(from u8, arg u64, max u16)  ->  start(u64) flags(u8: bit0 more, bit1 gap) len(u16) data [TLV]
//        from: 0 position = arg, 1 oldest, 2 now, 3 the last mark of kind arg (0 = any)
//   marks(from_serial u32)           ->  more(u8) count(u8) count x (len u8, serial u32, position u64, kind u8,
//                                        time_ns u64, detail u8) [TLV]
// Positions and mark serials never go back within a boot: a stream closed and made again at the same place goes on
// from where it was (common §1.1), so the ring and its counters are the interface's for good.
#pragma once

#include <Arduino.h>

#include "Oep.h"

namespace oep {

class PositionStream {
 public:
  struct Mark { uint32_t serial; uint64_t position; uint8_t kind; uint64_t time_ns; uint8_t detail; };
  static constexpr size_t kMarkBytes = 22;
  static constexpr size_t kReadRequest = 11;   // from(u8) arg(u64) max(u16)
  static constexpr size_t kReadHeader = 11;    // start(u64) flags(u8) len(u16)

  // capacity: a power of two (the ring index is position & (capacity - 1)).
  PositionStream(uint8_t *buffer, size_t capacity, Mark *marks, size_t mark_capacity)
      : buffer_(buffer), capacity_(capacity), marks_(marks), mark_capacity_(mark_capacity) {}

  // The oldest byte goes when the ring is full (a read then reports a gap).
  void put(uint8_t byte) { buffer_[total_ & (capacity_ - 1)] = byte; ++total_; }
  uint64_t end() const { return total_; }   // the position of the next byte
  // The bytes kept from `from` on that lie in one piece of the ring (from must be within oldest()..end()).
  size_t contiguous(uint64_t from, const uint8_t *&data) const {
    const size_t at = static_cast<size_t>(from & (capacity_ - 1)), left = static_cast<size_t>(total_ - from);
    data = buffer_ + at;
    return left < capacity_ - at ? left : capacity_ - at;
  }
  uint64_t oldest() const { return total_ - base_ > capacity_ ? total_ - capacity_ : base_; }
  void mark(uint8_t kind, uint8_t detail = 0) {
    marks_[slot_] = {serial_, total_, kind, nowNs(), detail};
    slot_ = (slot_ + 1) % mark_capacity_;
    if (kept_ < mark_capacity_) ++kept_;
    ++serial_;   // wraps at 2^32 (common §1.3); the slots and the count kept do not depend on it (core §2.6)
  }
  void clear() {   // nothing before now is kept
    base_ = total_;
    mark(reg::common::kMarkKindClear);
  }

  // A read request's values (common §1.2), p: from(u8) arg(u64) max(u16): from 3 with arg over 0xFF is malformed (a mark
  // kind is u8), from 4 or more unsupported, payload 0x00 (a later revision may define it). Completed: it may be read.
  static Result checkRead(const uint8_t *p, uint8_t *out, size_t capacity) {
    if (p[0] == reg::common::kReadFromLastMark && getU64(p + 1) > 0xff) return rejected(kRejectMalformed);
    if (p[0] > reg::common::kReadFromLastMark) return unsupportedValue(out, capacity);
    return completed();
  }
  // p: from(u8) arg(u64) max(u16), as checkRead passes it. The answer leaves `reserve` bytes of the capacity for what the
  // caller appends (an ignored TLV).
  Result read(const uint8_t *p, uint8_t *out, size_t capacity, uint16_t max_read, size_t reserve = 0) const {
    if (capacity < kReadHeader + reserve) return failed();
    const uint8_t from = p[0];
    const uint64_t arg = getU64(p + 1);
    uint64_t start = total_;
    if (from == reg::common::kReadFromPosition) {
      start = arg;
    } else if (from == reg::common::kReadFromOldest) {
      start = oldest();
    } else if (from == reg::common::kReadFromLastMark) {
      start = total_;   // no such mark kept (never, or pushed out of the ring): from now (common §1.2)
      for (uint32_t k = 0; k < kept_; ++k) {
        const Mark &mk = marks_[slotBack(k + 1)];
        if (arg == 0 || mk.kind == arg) { start = mk.position; break; }
      }
    }
    uint8_t flags = 0;
    if (start < oldest()) { start = oldest(); flags |= reg::common::kReadFlagsGap; }   // gap: pushed out or cleared
    if (start > total_) start = total_;
    uint32_t count = static_cast<uint32_t>(total_ - start);
    uint32_t room = static_cast<uint32_t>(capacity - kReadHeader - reserve);
    uint16_t max = getU16(p + 9);
    if (max > max_read) max = max_read;
    if (room > max) room = max;
    if (count > room) { count = room; flags |= reg::common::kReadFlagsMore; }
    putU64(out, start);
    out[8] = flags;
    putU16(out + 9, static_cast<uint16_t>(count));
    for (uint32_t i = 0; i < count; ++i) out[kReadHeader + i] = buffer_[(start + i) & (capacity_ - 1)];
    return completed(kReadHeader + count);
  }

  // Marks from serial `from` on (serial arithmetic; an older one starts at the oldest kept). -> bytes written.
  size_t marks(uint32_t from, uint8_t *out, size_t capacity) const {
    const uint32_t first = serial_ - kept_;   // serial number arithmetic (core §2.6)
    if (static_cast<int32_t>(from - first) < 0) from = first;
    uint8_t count = 0;
    size_t used = 2;
    bool more = false;
    for (uint32_t s = from; static_cast<int32_t>(s - serial_) < 0; ++s) {
      if (used + 1 + kMarkBytes > capacity || count == 255) { more = true; break; }
      const Mark &mk = marks_[slotBack(serial_ - s)];
      out[used++] = kMarkBytes;   // the element's length (core §2.3)
      putU32(out + used, mk.serial);
      putU64(out + used + 4, mk.position);
      out[used + 12] = mk.kind;
      putU64(out + used + 13, mk.time_ns);
      out[used + 21] = mk.detail;
      used += kMarkBytes;
      ++count;
    }
    out[0] = more;
    out[1] = count;
    return used;
  }

 private:
  uint8_t *buffer_;
  size_t capacity_;
  Mark *marks_;
  size_t mark_capacity_;
  uint64_t total_ = 0;    // bytes ever collected = the position of the next byte
  uint64_t base_ = 0;     // nothing before this position is kept (clear)
  uint32_t serial_ = 0;   // the serial of the next mark (u32, wraps)
  size_t slot_ = 0;       // where the next mark goes
  uint32_t kept_ = 0;     // marks kept (at most mark_capacity_)
  size_t slotBack(uint32_t back) const {   // the slot of the mark `back` before the next one (1: the newest)
    return (slot_ + mark_capacity_ - back % mark_capacity_) % mark_capacity_;
  }
};

// A stream a serial port's bind can carry (oep.probe.config §1.2): the target's console of a slot, a fixture UART's
// receive side. The binds read it from positions of their own (it is never consumed) and hand the port's raw bytes to
// its other end.
class BindSource {
 public:
  virtual const PositionStream *bindStream() const = 0;      // nullptr: nothing to carry now
  virtual uint16_t bindStreamNumber() const = 0;             // the stream's number (a console's; a UART's is fixed)
  virtual size_t bindInput(const uint8_t *data, size_t length) = 0;   // the port's raw bytes: how many were taken
  // Resets of this stream's target that last-reset counts (riscv-dm reset, attach's reset TLV); 0 for a UART.
  virtual uint32_t hostResets() const { return 0; }
  // The position of the reset mark (common §1.3) the stream placed when hostResets() reached `resets`; false: none
  // placed for it (yet, or the stream was not open). A port resumes from it after a session (probe.config §1.2).
  virtual bool hostResetMark(uint32_t resets, uint64_t &position) const { (void)resets; (void)position; return false; }

 protected:
  ~BindSource() = default;
};

}  // namespace oep
