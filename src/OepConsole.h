// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.target.console revision 1 (oep-spec docs/oep-if-console.ja.md): a stream opened on
// the debug connection (SDI / DMDATA / dmseq through the DmConsole driver), kept in a position-addressed buffer that
// reads do not consume, with marks for what happened.
//
//   0x01 open(connection u16, mechanism u8) [TLV]  -> stream u16, flags u8 (bit0 an existing stream) [TLV]
//   0x02 read(stream, from u8, arg u64, max u16)   -> start u64, flags u8 (bit0 more, bit1 gap), len u16, data   no lock
//   0x03 marks(stream, from_serial u32)            -> more u8, count u8, count x (len u8, serial u32, position u64,
//                                                     kind u8, time_ns u64, detail u8)                   no lock
//   0x04 clear(stream)   0x05 mark(stream, value u8)   0x06 write(stream, count u16, data) -> accepted u16
//   0x07 close(stream)   0x08 streams(first u8) -> more u8, count u8, count x (len u8, stream u16, connection u16,
//                                                                      mechanism u8, users u8, state u8)   no lock
//
// One live stream on the one connection; its number is from the probe's one space (core §9). Opening the same mechanism
// again hands it back with its positions and marks; another mechanism while one is open is unavailable (cause 6). The
// stream is used by the host's session (open) and by a slot (its bind): close and a lapse take the host's share, the
// settings take the slot's, and the stream closes (mark closed) when nobody is left. A stream whose connection went
// away is closed (mark link-lost when the line was lost, then closed 4) and stays readable (read / marks) until the
// same mechanism is opened again at the same place, which gives the same number back (mark attach, flags bit0); another
// mechanism there starts a new stream. Collection goes on whatever the host does; bytes go only when the buffer
// overflows, on clear, or when the probe restarts, and positions never go back within a boot.
#pragma once

#include "OepDmConsole.h"
#include "OepStream.h"
#include "OepTarget.h"

namespace oep {

class TargetConsoleStream final : public Interface, public BindSource {
 public:
  enum : uint8_t {
    kOpOpen = reg::target_console::kOpOpen, kOpRead = reg::target_console::kOpRead,
    kOpMarks = reg::target_console::kOpMarks, kOpClear = reg::target_console::kOpClear,
    kOpMark = reg::target_console::kOpMark, kOpWrite = reg::target_console::kOpWrite,
    kOpClose = reg::target_console::kOpClose, kOpStreams = reg::target_console::kOpStreams,
  };
  enum : uint8_t {
    kMarkReset = reg::common::kMarkKindReset, kMarkRestart = reg::common::kMarkKindRestart,
    kMarkAttach = reg::common::kMarkKindAttach, kMarkDetach = reg::common::kMarkKindDetach,
    kMarkLost = reg::common::kMarkKindLost, kMarkClear = reg::common::kMarkKindClear,
    kMarkHost = reg::common::kMarkKindHost, kMarkLinkLost = reg::common::kMarkKindLinkLost,
    kMarkClosed = reg::common::kMarkKindClosed,
  };
  enum : uint8_t { kUserHost = reg::target_console::kStreamUsersHostSession, kUserSlot = reg::target_console::kStreamUsersSlot };
  TargetConsoleStream(DebugPort &port, DmConsole &driver, uint16_t instance)
      : port_(port), driver_(driver), instance_(instance), stream_(buffer_, kCapacity, marks_, kMarks) {
    driver_.setSink(&TargetConsoleStream::take, this);   // the driver writes straight into this buffer
  }
  const char *name() const override { return reg::target_console::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::target_console::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::target_console::kLockFreeOps, op); }
  bool offers(uint8_t op) const override { return opIn(op, reg::target_console::kOpOpen, reg::target_console::kOpStreams); }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  // The host's session lapsed or was taken over: its share of the stream goes (mark closed 2 when it was the last).
  void sessionLapsed() override { release(kUserHost, reg::common::kMarkDetailClosedExpired); }
  void poll();   // from loop()
  // What one read may return: declare it within the probe's frame (1000 suits a 1 KiB frame).
  void setMaxRead(uint16_t bytes) { max_read_ = bytes; }
  // A slot's bind opens the stream itself on the connection (the same stream a host's open gets back), taking the
  // slot's share. false: no connection, or another mechanism is open.
  bool bindOpen(uint8_t mechanism);
  // The slot's share goes (its bind no longer carries it, or the slot was replaced / removed: mark closed 3 when last).
  void bindClose(uint8_t detail = reg::common::kMarkDetailClosedAllReleased) { release(kUserSlot, detail); }
  bool isOpen() const { return open_; }
  uint8_t users() const { return users_; }
  DebugPort &port() const { return port_; }
  // BindSource: the stream while it exists (a closed one too, until another is opened), the target's input.
  const PositionStream *bindStream() const override { return exists_ ? &stream_ : nullptr; }
  uint16_t bindStreamNumber() const override { return stream_number_; }
  size_t bindInput(const uint8_t *data, size_t length) override;
  uint32_t hostResets() const override { return port_.resets; }

 private:
  static constexpr size_t kCapacity = 8192, kMarks = 16;   // both powers of two (wrapping positions / serials)
  DebugPort &port_;
  DmConsole &driver_;
  uint16_t instance_;
  bool exists_ = false;    // a stream was opened (and may be closed but still readable)
  uint16_t stream_number_ = 0;   // the stream's number (the probe's one space); kept while closed for the same place
  uint16_t connection_ = 0;      // the connection it was opened on (the streams entry)
  uint16_t place_swdio_ = 0, place_swclk_ = 0;   // where (the same place reopens the same number)
  bool open_ = false;
  uint8_t mechanism_ = 0;
  uint8_t users_ = 0;
  uint16_t max_read_ = 1000;
  uint8_t buffer_[kCapacity];
  PositionStream::Mark marks_[kMarks];
  PositionStream stream_;
  uint32_t seen_resets_ = 0, seen_resyncs_ = 0, seen_restarts_ = 0, seen_closes_ = 0;
  bool openStream(uint8_t mechanism, uint8_t user, bool &existing);
  void closeStream(uint8_t detail, bool link_lost = false);
  void release(uint8_t user, uint8_t detail);
  static void take(void *self, uint8_t byte) { static_cast<TargetConsoleStream *>(self)->stream_.put(byte); }
};

}  // namespace oep
