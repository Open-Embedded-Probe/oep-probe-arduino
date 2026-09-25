#include "OepV1Console.h"

#include <Arduino.h>

namespace oep {
namespace v1 {
namespace {
inline bool refused(const Result &r) { return r.resolution != kResolutionCompleted; }
}  // namespace

size_t TargetConsoleStream::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.u32(kTagFeatures, 0b0111);            // mechanisms SDI, DMDATA, dmseq (on the debug connection)
  w.u32(0x40, kCapacity);                 // buffer_bytes
  w.u8(0x41, kMarks);                     // mark_capacity
  w.u16(0x43, max_read_);                 // max_read
  return w.ok() ? w.length() : 0;
}

void TargetConsoleStream::poll() {
  if (!open_) return;
  if (!port_.connected) {                 // detached: the stream ends with the connection, and stays readable
    driver_.stop();
    stream_.mark(kMarkDetach);
    open_ = false;
    return;
  }
  if (port_.resets != seen_resets_) {     // a reset issued through riscv-dm
    seen_resets_ = port_.resets;
    stream_.mark(kMarkReset, 0);          // detail 0: ndmreset
  }
  if (cdc_.port()) {   // port -> target, as much as the driver takes now
    uint8_t chunk[64];
    const size_t k = cdc_.fromPort(chunk, driver_.room() < sizeof chunk ? driver_.room() : sizeof chunk);
    if (k) driver_.queue(chunk, k);
  }
  driver_.poll();
  if (driver_.resyncs() > seen_resyncs_) {   // the target's console started over: after the first, a restart
    if (seen_resyncs_ > 0) stream_.mark(kMarkRestart, 1);   // detail 1: console resync
    seen_resyncs_ = driver_.resyncs();
  }
  cdc_.toPort(stream_);
}

bool TargetConsoleStream::openStream(uint8_t mechanism) {
  if (!driver_.start(mechanism)) return false;
  exists_ = open_ = true;
  mechanism_ = mechanism;
  seen_resets_ = port_.resets;
  seen_resyncs_ = driver_.resyncs();
  stream_.mark(kMarkAttach, mechanism);
  return true;
}

bool TargetConsoleStream::bindOpen(uint8_t mechanism) {
  if (!port_.connected || mechanism > reg::target_console::kMechanismDmseq) return false;
  if (open_) return mechanism_ == mechanism;
  return openStream(mechanism);
}

Result TargetConsoleStream::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  if (op == kOpOpen) {   // connection(u8) mechanism(u8) [TLV]  ->  stream(u8) flags(u8: bit0 existing)
    const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
    if (refused(parsed)) return parsed;
    if (payload[0] != 1 || !port_.connected) return rejected(kRejectNoConnection);
    if (payload[1] > reg::target_console::kMechanismDmseq) return rejected(kRejectUnsupported);
    if (capacity < 2) return failed();
    if (open_ && mechanism_ == payload[1]) {   // the same stream, positions and marks as they are
      out[0] = 1;
      out[1] = 1;
      return tail.finish(completed(2), out, capacity);
    }
    if (open_) return rejected(kRejectUnavailable);   // one stream at a time on this connection
    if (!openStream(payload[1])) return failed();
    out[0] = 1;
    out[1] = 0;
    return tail.finish(completed(2), out, capacity);
  }
  if (length < 1) return rejected(kRejectMalformed);
  if (payload[0] != 1 || !exists_) return rejected(kRejectUnavailable);   // stream 1 only
  const uint8_t *p = payload + 1;
  const size_t n = length - 1;
  switch (op) {
    case kOpRead: {   // from(u8) arg(u32) max(u16) [TLV]  ->  start(u32) flags(u8) data (closed tail)
      const Result parsed = plainTail(tail, p, n, 7, out, capacity);
      if (refused(parsed)) return parsed;
      if (p[0] > reg::target_console::kReadFromLastMark) return rejected(kRejectUnsupported);
      poll();   // take what is waiting first
      return stream_.read(p, out, capacity, max_read_);
    }
    case kOpMarks: {   // from_serial(u32) [TLV]  ->  more(u8) count(u8) entries
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 2) return failed();
      poll();
      const size_t room = tail.anyIgnored() && capacity > 2 + Tail::kMaxIgnored ? capacity - 2 - Tail::kMaxIgnored : capacity;
      return tail.finish(completed(stream_.marks(getU32(p), out, room)), out, capacity);
    }
    case kOpClear:
    case kOpClose: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!open_) return rejected(kRejectUnavailable);
      if (op == kOpClear) {
        stream_.clear();
      } else {
        driver_.stop();
        stream_.mark(kMarkDetach);
        open_ = false;
      }
      return tail.finish(completed(), out, capacity);
    }
    case kOpMark: {   // value(u8) [TLV]
      const Result parsed = plainTail(tail, p, n, 1, out, capacity);
      if (refused(parsed)) return parsed;
      if (!open_) return rejected(kRejectUnavailable);
      stream_.mark(kMarkHost, p[0]);
      return tail.finish(completed(), out, capacity);
    }
    case kOpWrite: {   // count(u16) data [TLV]  ->  accepted(u16); what was not taken is the host's to send again
      if (n < 2) return rejected(kRejectMalformed);
      const uint16_t count = getU16(p);
      const Result parsed = plainTail(tail, p, n, 2u + count, out, capacity);
      if (refused(parsed)) return parsed;
      if (!open_) return rejected(kRejectUnavailable);
      if (capacity < 2) return failed();
      const size_t queued = driver_.queue(p + 2, count);
      driver_.poll();   // start it on its way
      putU16(out, static_cast<uint16_t>(queued));
      const Result r = queued == count ? completed(2) : queued ? partial(2) : failed(2);
      return tail.finish(r, out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace v1
}  // namespace oep
