// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepConsole.h"

#include <Arduino.h>

namespace oep {

namespace con = reg::target_console;

size_t TargetConsoleStream::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  static const uint8_t kMechanisms[] = {con::kMechanismSdi, con::kMechanismDmdata, con::kMechanismDmseq};
  w.put(con::kTlvDescribeMechanisms, kMechanisms, sizeof kMechanisms);   // oep-if-console §1
  return w.ok() ? w.length() : 0;
}

size_t TargetConsoleStream::bindInput(const uint8_t *data, size_t length) {
  if (!open_) return 0;
  const size_t room = driver_.slot();
  const size_t n = driver_.queue(data, length < room ? length : room);
  if (n) driver_.poll();
  return n;
}

// The stream closes (oep-if-console §2): the driver stops, the reason is marked, and what was collected stays readable.
void TargetConsoleStream::closeStream(uint8_t detail, bool link_lost) {
  if (!open_) return;
  driver_.stop();
  if (link_lost) stream_.mark(kMarkLinkLost);
  stream_.mark(kMarkClosed, detail);
  open_ = false;
  users_ = 0;
  ResourceNumbers::close(stream_number_);
}

void TargetConsoleStream::release(uint8_t user, uint8_t detail) {
  if (!open_ || !(users_ & user)) return;
  users_ &= static_cast<uint8_t>(~user);
  if (users_) return;
  closeStream(detail);
  // The stream was the connection's last user of its own: a connection nobody else uses closes with it
  // (oep-if-common §2). The wire's users say who else is there; the slot's share of the connection is the settings'.
}

void TargetConsoleStream::poll() {
  if (seen_closes_ != port_.closes) {     // the connection went: the stream ends with it, and stays readable
    seen_closes_ = port_.closes;
    closeStream(reg::common::kMarkDetailClosedConnectionClosed, port_.lost);
  }
  if (!open_) return;
  if (port_.resets != seen_resets_) {     // a reset the host asked for (riscv-dm reset, attach's reset TLV)
    seen_resets_ = port_.resets;
    reset_marked_ = true;                 // where a bind's port resumes after a session (probe.config §1.2)
    reset_mark_resets_ = port_.resets;
    reset_mark_position_ = stream_.end();
    stream_.mark(kMarkReset, port_.reset_detail);
  }
  if (port_.dm.restarts() != seen_restarts_) {   // havereset seen and acknowledged: the target restarted by itself
    seen_restarts_ = port_.dm.restarts();
    stream_.mark(kMarkRestart, reg::common::kMarkDetailRestartHavereset);
    driver_.unsync();
  }
  driver_.poll();
  if (driver_.lineLost()) {   // judged inside the console's reading (oep-if-debug §2): link-lost, then closed 4
    releaseConnection(port_, 0xff, true, true);
    seen_closes_ = port_.closes;
    closeStream(reg::common::kMarkDetailClosedConnectionClosed, true);
    return;
  }
  if (driver_.resyncs() > seen_resyncs_) {   // the target's console started over: after the first, a restart
    if (seen_resyncs_ > 0) stream_.mark(kMarkRestart, reg::common::kMarkDetailRestartResync);
    seen_resyncs_ = driver_.resyncs();
  }
}

// Open (or join) the stream of `mechanism` on the live connection for `user`. existing: the stream was already open, or
// is the closed stream of the same place and mechanism opened again under its own number (oep-if-console §2).
bool TargetConsoleStream::openStream(uint8_t mechanism, uint8_t user, bool &existing) {
  existing = false;
  if (open_) {
    if (mechanism_ != mechanism) return false;
    users_ |= user;
    existing = true;
    return true;
  }
  const bool same_place = exists_ && mechanism_ == mechanism && place_swdio_ == port_.swdio && place_swclk_ == port_.swclk;
  uint16_t number = stream_number_;
  if (same_place) {
    if (!ResourceNumbers::reopen(number, ResourceNumbers::kStream)) return false;
  } else {
    number = ResourceNumbers::take(ResourceNumbers::kStream);
    if (!number) return false;
  }
  if (!driver_.start(mechanism)) { ResourceNumbers::close(number); return false; }
  stream_number_ = number;
  connection_ = port_.number;
  place_swdio_ = port_.swdio;
  place_swclk_ = port_.swclk;
  exists_ = open_ = true;
  mechanism_ = mechanism;
  users_ = user;
  seen_resets_ = port_.resets;
  seen_restarts_ = port_.dm.restarts();
  seen_closes_ = port_.closes;
  seen_resyncs_ = driver_.resyncs();
  stream_.mark(kMarkAttach, mechanism);
  existing = same_place;
  return true;
}

bool TargetConsoleStream::bindOpen(uint8_t mechanism) {
  if (!port_.connected || mechanism > con::kMechanismDmseq) return false;
  bool existing = false;
  return openStream(mechanism, kUserSlot, existing);
}

Result TargetConsoleStream::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  if (op == kOpOpen) {   // connection(u16) mechanism(u8) [TLV]  ->  stream(u16) flags(u8: bit0 existing) [TLV]
    const Result parsed = plainTail(tail, payload, length, 3, out, capacity);
    if (refused(parsed)) return parsed;
    const uint8_t mechanism = payload[2];
    if (mechanism > con::kMechanismDmseq) return unsupportedValue(out, capacity);   // not a mechanism this probe opens
    if (!port_.connected || getU16(payload) != port_.number)
      return ResourceNumbers::refuse(getU16(payload), ResourceNumbers::kConnection, out, capacity);
    if (open_ && mechanism_ != mechanism) return wrongState(out, capacity);   // one live stream on a connection (§2)
    if (capacity < 3) return failed();
    bool existing = false;
    if (!openStream(mechanism, kUserHost, existing)) return failedStatus(kStatusLine, out, capacity);
    putU16(out, stream_number_);
    out[2] = existing ? con::kOpenFlagsExisting : 0;
    return tail.finish(completed(3), out, capacity);
  }
  if (op == kOpStreams) {   // first(u8) [TLV] -> more(u8) count(u8) count x (len(u8) stream(u16) connection(u16) mechanism(u8) users(u8) state(u8))
    const Result parsed = plainTail(tail, payload, length, 1, out, capacity);
    if (refused(parsed)) return parsed;
    if (capacity < 10) return failed();
    out[0] = 0;   // more: never (one stream at most)
    out[1] = exists_ && payload[0] == 0 ? 1 : 0;
    if (!out[1]) return tail.finish(completed(2), out, capacity);
    out[2] = 7;
    putU16(out + 3, stream_number_);
    putU16(out + 5, connection_);
    out[7] = mechanism_;
    out[8] = users_;
    out[9] = open_ ? con::kStreamStateOpen : con::kStreamStateClosed;
    return tail.finish(completed(10), out, capacity);
  }
  if (length < 2) return rejected(kRejectMalformed);
  if (!exists_ || getU16(payload) != stream_number_)   // a number it does not know (core §4.3)
    return ResourceNumbers::refuse(getU16(payload), ResourceNumbers::kStream, out, capacity);
  const uint8_t *p = payload + 2;
  const size_t n = length - 2;
  switch (op) {
    case kOpRead: {   // from(u8) arg(u64) max(u16) [TLV]  ->  start(u64) flags(u8) len(u16) data [TLV]
      const Result parsed = plainTail(tail, p, n, PositionStream::kReadRequest, out, capacity);
      if (refused(parsed)) return parsed;
      if (p[0] > reg::common::kReadFromLastMark) return unsupportedValue(out, capacity);   // from 4+ (common §1, core §2.5)
      poll();   // take what is waiting first
      const size_t reserve = tail.anyIgnored() ? 2 + Tail::kMaxIgnored : 0;
      return tail.finish(stream_.read(p, out, capacity, max_read_, reserve), out, capacity);
    }
    case kOpMarks: {   // from_serial(u32) [TLV]  ->  more(u8) count(u8) entries [TLV]
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 2) return failed();
      poll();
      const size_t room = tail.anyIgnored() && capacity > 2 + Tail::kMaxIgnored ? capacity - 2 - Tail::kMaxIgnored : capacity;
      return tail.finish(completed(stream_.marks(getU32(p), out, room)), out, capacity);
    }
    case kOpClose: {   // the host's share goes; a closed stream's close does nothing (oep-if-console §1)
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      release(kUserHost, reg::common::kMarkDetailClosedAllReleased);
      return tail.finish(completed(), out, capacity);
    }
    case kOpClear: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!open_) return wrongState(out, capacity);
      stream_.clear();
      return tail.finish(completed(), out, capacity);
    }
    case kOpMark: {   // value(u8) [TLV]
      const Result parsed = plainTail(tail, p, n, 1, out, capacity);
      if (refused(parsed)) return parsed;
      if (!open_) return wrongState(out, capacity);
      stream_.mark(kMarkHost, p[0]);
      return tail.finish(completed(), out, capacity);
    }
    case kOpWrite: {   // count(u16) data [TLV]  ->  accepted(u16) [TLV]: what went into the mechanism's send slot
      if (n < 2) return rejected(kRejectMalformed);
      const uint16_t count = getU16(p);
      const Result parsed = plainTail(tail, p, n, 2u + count, out, capacity);
      if (refused(parsed)) return parsed;
      if (count == 0) return rejected(kRejectMalformed);
      if (!open_) return wrongState(out, capacity);
      if (capacity < 2) return failed();
      const size_t slot = driver_.slot();
      const size_t queued = driver_.queue(p + 2, count < slot ? count : slot);
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
