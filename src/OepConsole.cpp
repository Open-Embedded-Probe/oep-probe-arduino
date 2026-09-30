// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepConsole.h"

#include <Arduino.h>

namespace oep {
namespace {
}  // namespace

size_t TargetConsoleStream::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  static const uint8_t kMechanisms[] = {reg::target_console::kMechanismSdi, reg::target_console::kMechanismDmdata,
                                       reg::target_console::kMechanismDmseq};
  w.put(reg::target_console::kTlvDescribeMechanisms, kMechanisms, sizeof kMechanisms);   // oep-if-console §1
  return w.ok() ? w.length() : 0;
}

size_t TargetConsoleStream::bindInput(const uint8_t *data, size_t length) {
  if (!open_) return 0;
  const size_t room = driver_.room();
  const size_t n = driver_.queue(data, length < room ? length : room);
  if (n) driver_.poll();
  return n;
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
  driver_.poll();
  if (driver_.resyncs() > seen_resyncs_) {   // the target's console started over: after the first, a restart
    if (seen_resyncs_ > 0) stream_.mark(kMarkRestart, 1);   // detail 1: console resync
    seen_resyncs_ = driver_.resyncs();
  }
}

bool TargetConsoleStream::openStream(uint8_t mechanism) {
  if (!driver_.start(mechanism)) return false;
  if (stream_number_ == 0xffff) return false;   // every number used (core §9: never reused within a boot)
  ++stream_number_;
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
  if (op == kOpOpen) {   // connection(u16) mechanism(u8) [TLV]  ->  stream(u16) flags(u8: bit0 existing)
    const Result parsed = plainTail(tail, payload, length, 3, out, capacity);
    if (refused(parsed)) return parsed;
    if (getU16(payload) != port_.number || !port_.connected) return rejected(kRejectNoConnection);
    const uint8_t mechanism = payload[2];
    if (mechanism > reg::target_console::kMechanismDmseq) return rejected(kRejectUnsupported);
    if (capacity < 3) return failed();
    if (open_ && mechanism_ == mechanism) {   // the same stream, positions and marks as they are
      putU16(out, stream_number_);
      out[2] = 1;
      return tail.finish(completed(3), out, capacity);
    }
    if (open_) return rejected(kRejectUnavailable);   // one stream at a time on this connection
    if (stream_number_ == 0xffff) return rejected(kRejectUnavailable);
    if (!openStream(mechanism)) return failed();
    putU16(out, stream_number_);
    out[2] = 0;
    return tail.finish(completed(3), out, capacity);
  }
  if (length < 2) return rejected(kRejectMalformed);
  if (getU16(payload) != stream_number_ || !exists_) return rejected(kRejectUnavailable);   // the current stream only
  const uint8_t *p = payload + 2;
  const size_t n = length - 2;
  switch (op) {
    case kOpRead: {   // from(u8) arg(u64) max(u16) [TLV]  ->  start(u64) flags(u8) data (closed tail)
      const Result parsed = plainTail(tail, p, n, PositionStream::kReadRequest, out, capacity);
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

}  // namespace oep
