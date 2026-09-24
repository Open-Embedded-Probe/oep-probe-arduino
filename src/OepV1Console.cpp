#include "OepV1Console.h"

#include <Arduino.h>

namespace oep {
namespace v1 {

size_t TargetConsoleStream::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.u32(kTagFeatures, 0b0111);            // mechanisms SDI, DMDATA, dmseq (on the debug connection)
  w.u32(0x40, kCapacity);                 // buffer_bytes
  w.u8(0x41, kMarks);                     // mark_capacity
  w.u16(0x43, max_read_);                 // max_read
  return w.ok() ? w.length() : 0;
}

uint32_t TargetConsoleStream::oldest() const {
  const uint32_t kept = total_ - base_;
  return kept > kCapacity ? total_ - kCapacity : base_;
}

void TargetConsoleStream::mark(uint8_t kind, uint8_t detail) {
  marks_[mark_count_ % kMarks] = {total_, kind, millis(), detail};
  ++mark_count_;
}

void TargetConsoleStream::poll() {
  if (!open_) return;
  if (!port_.connected) {                 // detached: the stream ends with the connection
    driver_.stop();
    mark(kMarkDetach);
    open_ = false;
    return;
  }
  if (port_.resets != seen_resets_) {     // a reset issued through riscv-dm
    seen_resets_ = port_.resets;
    mark(kMarkReset, 0);                  // detail 0: ndmreset
  }
  driver_.poll();
  if (driver_.resyncs() > seen_resyncs_) {   // the target's console started over: after the first, a restart
    if (seen_resyncs_ > 0) mark(kMarkRestart, 1);   // detail 1: console resync
    seen_resyncs_ = driver_.resyncs();
  }
}

Result TargetConsoleStream::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (op == kOpOpen) {
    if (length != 2 || payload[1] > 2 || capacity < 1) return rejected(kRejectMalformed);
    if (payload[0] != 1 || !port_.connected) return rejected(kRejectUnavailable);
    if (!driver_.start(payload[1])) return failed();
    open_ = true;
    seen_resets_ = port_.resets;
    seen_resyncs_ = driver_.resyncs();
    mark(kMarkAttach, payload[1]);
    out[0] = 1;
    return completed(1);
  }
  if (length < 1 || payload[0] != 1) return rejected(kRejectUnavailable);   // stream 1 only
  const uint8_t *p = payload + 1;
  const size_t n = length - 1;
  switch (op) {
    case kOpRead: {
      if (n != 7 || capacity < 5) return rejected(kRejectMalformed);
      poll();   // take what is waiting first
      const uint8_t from = p[0];
      const uint32_t arg = getU32(p + 1);
      uint32_t start = total_;
      if (from == 0) start = arg;
      else if (from == 1) start = oldest();
      else if (from == 3) {
        start = oldest();
        for (uint32_t i = mark_count_; i > 0 && i + kMarks > mark_count_; --i) {
          const Mark &mk = marks_[(i - 1) % kMarks];
          if ((arg & 0xff) == 0 || mk.kind == (arg & 0xff)) { start = mk.position; break; }
        }
      } else if (from != 2) {
        return rejected(kRejectMalformed);
      }
      uint8_t flags = 0;
      if (static_cast<int32_t>(start - oldest()) < 0) { start = oldest(); flags |= 2; }   // gap: pushed out
      if (static_cast<int32_t>(start - total_) > 0) start = total_;
      uint32_t count = total_ - start;
      uint32_t room = capacity - 5;
      uint16_t max = getU16(p + 5);
      if (max > max_read_) max = max_read_;
      if (room > max) room = max;
      if (count > room) { count = room; flags |= 1; }
      putU32(out, start);
      out[4] = flags;
      for (uint32_t i = 0; i < count; ++i) out[5 + i] = buffer_[(start + i) % kCapacity];
      return completed(5 + count);
    }
    case kOpMarks: {
      if (n != 4 || capacity < 1) return rejected(kRejectMalformed);
      const uint32_t from = getU32(p);
      uint8_t count = 0;
      size_t used = 1;
      const uint32_t first = mark_count_ > kMarks ? mark_count_ - kMarks : 0;
      for (uint32_t i = first; i < mark_count_ && used + 10 <= capacity; ++i) {
        const Mark &mk = marks_[i % kMarks];
        if (static_cast<int32_t>(mk.position - from) < 0) continue;
        putU32(out + used, mk.position);
        out[used + 4] = mk.kind;
        putU32(out + used + 5, mk.time_ms);
        out[used + 9] = mk.detail;
        used += 10;
        ++count;
      }
      out[0] = count;
      return completed(used);
    }
    case kOpClear:
      base_ = total_;
      mark(kMarkClear);
      return completed();
    case kOpMark:
      if (n != 1) return rejected(kRejectMalformed);
      mark(kMarkHost, p[0]);
      return completed();
    case kOpWrite: {
      if (!open_ || capacity < 2) return rejected(kRejectUnavailable);
      const size_t queued = driver_.queue(p, n);
      driver_.poll();   // start it on its way
      putU16(out, static_cast<uint16_t>(queued));
      return completed(2);
    }
    case kOpClose:
      driver_.stop();
      open_ = false;
      return completed();
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace v1
}  // namespace oep
