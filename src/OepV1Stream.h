// The position-addressed byte stream behind oep.target.console and oep.fixture.uart (oep-spec
// v1-core-wire-delta.ja.md §5.7 / §5.8, console-stream.ja.md): bytes kept in a ring that reads do not consume,
// addressed by a u32 position that wraps, and marks with their own u32 serial numbers.
//
//   read(from u8, arg u32, max u16)  ->  start(u32) flags(u8: bit0 more, bit1 gap) data        (closed tail)
//        from: 0 position = arg, 1 oldest, 2 now, 3 the last mark of kind arg (0 = any)
//   marks(from_serial u32)           ->  more(u8) count(u8) count x (serial u32, position u32, kind u8, time_ms u32,
//                                        detail u8)
#pragma once

#include <Arduino.h>

#include "OepV1.h"

namespace oep {
namespace v1 {

class PositionStream {
 public:
  struct Mark { uint32_t serial, position; uint8_t kind; uint32_t time_ms; uint8_t detail; };
  static constexpr size_t kMarkBytes = 14;

  // capacity: a power of two (positions wrap at 2^32, the ring index is position & (capacity - 1)).
  PositionStream(uint8_t *buffer, size_t capacity, Mark *marks, size_t mark_capacity)
      : buffer_(buffer), capacity_(capacity), marks_(marks), mark_capacity_(mark_capacity) {}

  // The oldest byte goes when the ring is full (a read then reports a gap).
  void put(uint8_t byte) { buffer_[total_ & (capacity_ - 1)] = byte; ++total_; }
  uint32_t end() const { return total_; }   // the position of the next byte
  // The bytes kept from `from` on that lie in one piece of the ring (from must be within oldest()..end()).
  size_t contiguous(uint32_t from, const uint8_t *&data) const {
    const size_t at = from & (capacity_ - 1), left = total_ - from;
    data = buffer_ + at;
    return left < capacity_ - at ? left : capacity_ - at;
  }
  uint32_t oldest() const { return total_ - base_ > capacity_ ? total_ - static_cast<uint32_t>(capacity_) : base_; }
  void mark(uint8_t kind, uint8_t detail = 0) {
    marks_[serial_ % mark_capacity_] = {serial_, total_, kind, static_cast<uint32_t>(millis()), detail};
    ++serial_;
  }
  void clear() {   // nothing before now is kept
    base_ = total_;
    mark(reg::target_console::kMarkKindClear);
  }

  // p: from(u8) arg(u32) max(u16). from > 3 is the caller's to refuse (unknown value).
  Result read(const uint8_t *p, uint8_t *out, size_t capacity, uint16_t max_read) const {
    if (capacity < 5) return failed();
    const uint8_t from = p[0];
    const uint32_t arg = getU32(p + 1);
    uint32_t start = total_;
    if (from == reg::target_console::kReadFromPosition) {
      start = arg;
    } else if (from == reg::target_console::kReadFromOldest) {
      start = oldest();
    } else if (from == reg::target_console::kReadFromLastMark) {
      start = oldest();
      const uint32_t kept = serial_ < mark_capacity_ ? serial_ : static_cast<uint32_t>(mark_capacity_);
      for (uint32_t k = 0; k < kept; ++k) {
        const Mark &mk = marks_[(serial_ - 1 - k) % mark_capacity_];
        if ((arg & 0xff) == 0 || mk.kind == (arg & 0xff)) { start = mk.position; break; }
      }
    }
    uint8_t flags = 0;
    if (static_cast<int32_t>(start - oldest()) < 0) { start = oldest(); flags |= 2; }   // gap: pushed out or cleared
    if (static_cast<int32_t>(start - total_) > 0) start = total_;
    uint32_t count = total_ - start;
    uint32_t room = static_cast<uint32_t>(capacity - 5);
    uint16_t max = getU16(p + 5);
    if (max > max_read) max = max_read;
    if (room > max) room = max;
    if (count > room) { count = room; flags |= 1; }
    putU32(out, start);
    out[4] = flags;
    for (uint32_t i = 0; i < count; ++i) out[5 + i] = buffer_[(start + i) & (capacity_ - 1)];
    return completed(5 + count);
  }

  // Marks from serial `from` on (serial arithmetic; an older one starts at the oldest kept). -> bytes written.
  size_t marks(uint32_t from, uint8_t *out, size_t capacity) const {
    const uint32_t kept = serial_ < mark_capacity_ ? serial_ : static_cast<uint32_t>(mark_capacity_);
    const uint32_t first = serial_ - kept;
    if (static_cast<int32_t>(from - first) < 0) from = first;
    uint8_t count = 0;
    size_t used = 2;
    bool more = false;
    for (uint32_t s = from; static_cast<int32_t>(s - serial_) < 0; ++s) {
      if (used + kMarkBytes > capacity || count == 255) { more = true; break; }
      const Mark &mk = marks_[s % mark_capacity_];
      putU32(out + used, mk.serial);
      putU32(out + used + 4, mk.position);
      out[used + 8] = mk.kind;
      putU32(out + used + 9, mk.time_ms);
      out[used + 13] = mk.detail;
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
  uint32_t total_ = 0;    // bytes ever collected = the position of the next byte
  uint32_t base_ = 0;     // nothing before this position is kept (clear)
  uint32_t serial_ = 0;   // marks ever made = the serial of the next one
};

}  // namespace v1
}  // namespace oep
