// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepConsole.h"

#include <Arduino.h>

namespace oep {

namespace con = reg::target_console;

size_t TargetConsoleStream::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  // mechanisms: n(u8) n x mechanism (oep-if-console §1), the ones declareMechanisms left
  uint8_t v[1 + 8];
  uint8_t n = 0;
  for (uint8_t m = 0; m < 8; ++m) if (offersMechanism(m)) v[1 + n++] = m;
  v[0] = n;
  w.put(con::kTlvDescribeMechanisms, v, 1u + n);
  return w.ok() ? w.length() : 0;
}

// The old binds' input (the firmware's own pass-through): into the send queue as the write op, as much as fits.
size_t TargetConsoleStream::bindInput(const uint8_t *data, size_t length) {
  if (!open_) return 0;
  const size_t n = driver_.queue(data, length);
  if (n) driver_.poll();
  return n;
}

// The stream goes (oep-if-console §2: close, the connection released, the session's end): the driver stops, the stream,
// its buffer and send queue are freed, and its number is never used again (core §9). No mark: nothing reads it after.
void TargetConsoleStream::closeStream() {
  if (!open_) return;
  driver_.stop();
  open_ = exists_ = false;
  stream_.erase();
  ResourceNumbers::close(stream_number_);
}

void TargetConsoleStream::release(uint8_t, uint8_t) { closeStream(); }

void TargetConsoleStream::poll() {
  if (seen_closes_ != port_.closes) {     // the connection went: the stream goes with it (common §2)
    seen_closes_ = port_.closes;
    closeStream();
  }
  if (!open_) return;
  if (port_.resets != seen_resets_) {     // the reset op succeeded (debug §4.6): reset 0x40, detail 1 ndmreset
    seen_resets_ = port_.resets;
    stream_.mark(kMarkReset, reg::target_console::kMarkDetailResetNdmreset);
  }
  if (port_.dm.restarts() != seen_restarts_) {   // havereset seen and acknowledged: the target restarted by itself
    seen_restarts_ = port_.dm.restarts();
    stream_.mark(kMarkRestart, reg::target_console::kMarkDetailRestartHavereset);
    driver_.unsync();
  }
  driver_.poll();
  if (driver_.lineLost()) {   // judged inside the console's reading (oep-if-debug §2): connection and stream close, no mark
    releaseConnection(port_, 0xff, true, true);
    seen_closes_ = port_.closes;
    closeStream();
    return;
  }
  if (driver_.resyncs() > seen_resyncs_) {   // a new SYN after the first sync: restart, detail 2 resync (console §2)
    if (seen_resyncs_ > 0) stream_.mark(kMarkRestart, reg::target_console::kMarkDetailRestartResync);
    seen_resyncs_ = driver_.resyncs();
  }
}

// A new stream of `mechanism` on the live connection: its own number (0: none left, cause 2), positions and serials
// from 0 (common §1.1).
uint32_t TargetConsoleStream::openStream(uint8_t mechanism) {
  const uint32_t number = ResourceNumbers::take(ResourceNumbers::kStream);
  if (!number) return 0;
  if (!driver_.start(mechanism)) { ResourceNumbers::close(number); return 0xffffffffu; }
  stream_.restart();
  stream_number_ = number;
  connection_ = port_.number;
  exists_ = open_ = true;
  mechanism_ = mechanism;
  seen_resets_ = port_.resets;
  seen_restarts_ = port_.dm.restarts();
  seen_closes_ = port_.closes;
  seen_resyncs_ = driver_.resyncs();
  return number;
}

bool TargetConsoleStream::bindOpen(uint8_t) { return false; }   // the old binds: the pass-through takes their place

Result TargetConsoleStream::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  if (op == kOpOpen) {   // connection(u32) mechanism(u8) [TLV]  ->  stream(u32) [TLV] (oep-if-console §1)
    const Result parsed = plainTail(tail, payload, length, 5, out, capacity);
    if (refused(parsed)) return parsed;
    const uint8_t mechanism = payload[4];
    if (!offersMechanism(mechanism)) return unsupportedValue(out, capacity);   // not in describe's mechanisms (0x00)
    const uint32_t connection = getU32(payload);
    if (!port_.connected || connection != port_.number) {
      // another kind's number, or a connection this console does not ride on (arm-adi): unavailable cause 6; a number
      // nobody holds: no_resource (console §1)
      if (ResourceNumbers::kindOf(connection) != ResourceNumbers::kNone) return wrongState(out, capacity);
      return ResourceNumbers::refuse(connection, ResourceNumbers::kConnection, out, capacity);
    }
    if (open_) return wrongState(out, capacity);   // one live stream on a connection (console §2): cause 6
    if (capacity < 4) return failed();
    const uint32_t number = openStream(mechanism);
    if (!number) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);   // numbers used up (core §9)
    if (number == 0xffffffffu) return failed();
    putU32(out, number);
    return completed(4);
  }
  if (op == kOpStreams) {   // first(u16) [TLV] -> more(u8) count(u8) count x (stream u32, connection u32, mechanism u8) [TLV]
    const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
    if (refused(parsed)) return parsed;
    if (capacity < 11) return failed();
    out[0] = 0;   // more: never (one stream at most); first at or past the count: none (core §7.3)
    out[1] = open_ && getU16(payload) == 0 ? 1 : 0;
    if (!out[1]) return completed(2);
    putU32(out + 2, stream_number_);   // no element length (core §2.3)
    putU32(out + 6, connection_);
    out[10] = mechanism_;
    return completed(11);
  }
  if (length < 4) return rejected(kRejectMalformed);
  // The request's form and values first, nothing done, then the number it names (core §4.3: every check before
  // anything changes).
  checking_ = true;
  const Result checked = streamOp(op, payload + 4, length - 4, out, capacity);
  checking_ = false;
  if (refused(checked)) return checked;
  if (!open_ || getU32(payload) != stream_number_)   // a number it does not know, a closed one too (core §4.3, §9)
    return ResourceNumbers::refuse(getU32(payload), ResourceNumbers::kStream, out, capacity);
  return streamOp(op, payload + 4, length - 4, out, capacity);
}

// read, marks, close, clear, mark, write on the stream (after its number). checking_: only checked - a refusal, or
// completed when it would run.
Result TargetConsoleStream::streamOp(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case kOpRead: {   // from(u8) arg(u64) max(u16) [TLV]  ->  start(u64) flags(u8) len(u16) data [TLV]
      const Result parsed = plainTail(tail, p, n, PositionStream::kReadRequest, out, capacity);
      if (refused(parsed)) return parsed;
      const Result values = PositionStream::checkRead(p, out, capacity);   // from 4+ (common §1.2)
      if (refused(values)) return values;
      if (!optional_ && p[0] == reg::common::kReadFromLastMark) return unsupportedValue(out, capacity);   // no marks (§1)
      if (checking_) return completed();
      poll();   // take what is waiting first
      return stream_.read(p, out, capacity, max_read_);
    }
    case kOpMarks: {   // from_serial(u32) [TLV]  ->  more(u8) count(u8) entries [TLV]
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (checking_) return completed();
      if (capacity < 2) return failed();
      poll();
      return completed(stream_.marks(getU32(p), out, capacity));
    }
    case kOpClose: {   // the host's share goes; a closed stream's close does nothing (oep-if-console §1)
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (checking_) return completed();
      closeStream();
      return completed();
    }
    case kOpClear: {
      const Result parsed = plainTail(tail, p, n, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (checking_) return completed();
      if (!open_) return wrongState(out, capacity);
      stream_.clear();
      return completed();
    }
    case kOpMark: {   // value(u8) [TLV]
      const Result parsed = plainTail(tail, p, n, 1, out, capacity);
      if (refused(parsed)) return parsed;
      if (checking_) return completed();
      if (!open_) return wrongState(out, capacity);
      stream_.mark(kMarkHost, p[0]);
      return completed();
    }
    case kOpWrite: {   // count(u16) data [TLV]  ->  accepted(u16) [TLV]: what went into the send queue (§2)
      if (n < 2) return rejected(kRejectMalformed);
      const uint16_t count = getU16(p);
      const Result parsed = plainTail(tail, p, n, 2u + count, out, capacity);
      if (refused(parsed)) return parsed;
      if (checking_) return completed();
      if (!open_) return wrongState(out, capacity);
      if (mechanism_ == reg::target_console::kMechanismSdi) return unsupportedValue(out, capacity);   // one way (§1, §3.1)
      if (capacity < 2) return failed();
      poll();   // a frame waiting takes from the queue first, so its free space is as it is now
      if (!open_) return wrongState(out, capacity);
      // min(count, free space); 0 only when the queue is full, or always on SDI (one way, §3.1); count 0: success
      // (accepted = count, common §1.4)
      const size_t queued = driver_.queue(p + 2, count);
      driver_.poll();   // start it on its way
      putU16(out, static_cast<uint16_t>(queued));
      const Result r = queued == count ? completed(2) : queued ? partial(2) : failed(2);
      return r;
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep
