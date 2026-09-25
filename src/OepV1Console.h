// oep.target.console revision 1 (oep-spec v1-core-wire-delta.ja.md §5.7, console-stream.ja.md): a stream opened on
// the debug connection (SDI / DMDATA / dmseq through the DmConsole driver), kept in a position-addressed buffer that
// reads do not consume, with marks for what happened.
//
//   0x01 open(connection u8, mechanism u8) [TLV]   -> stream u8, flags u8 (bit0 an existing stream)
//   0x02 read(stream, from u8, arg u32, max u16)   -> start u32, flags u8 (bit0 more, bit1 gap), data      no lock
//   0x03 marks(stream, from_serial u32)            -> more u8, count u8, count x (serial u32, position u32, kind u8,
//                                                     time_ms u32, detail u8)                              no lock
//   0x04 clear(stream)   0x05 mark(stream, value u8)   0x06 write(stream, count u16, data) -> accepted u16
//   0x07 close(stream)
//
// One stream (number 1) on the one connection: opening the same mechanism again hands it back with its positions and
// marks; another mechanism while one is open is unavailable. A stream whose connection went away is closed (mark
// detach) and stays readable (read / marks) until a mechanism is opened again. Collection goes on whatever the host
// does (sessions and lock lapses change nothing); bytes go only when the buffer overflows, on clear, or when the
// probe restarts.
#pragma once

#include "OepDmConsole.h"
#include "OepV1Stream.h"
#include "OepV1Target.h"

namespace oep {
namespace v1 {

class TargetConsoleStream final : public Interface {
 public:
  enum : uint8_t {
    kOpOpen = reg::target_console::kOpOpen, kOpRead = reg::target_console::kOpRead,
    kOpMarks = reg::target_console::kOpMarks, kOpClear = reg::target_console::kOpClear,
    kOpMark = reg::target_console::kOpMark, kOpWrite = reg::target_console::kOpWrite,
    kOpClose = reg::target_console::kOpClose,
  };
  enum : uint8_t {
    kMarkReset = reg::target_console::kMarkKindReset, kMarkRestart = reg::target_console::kMarkKindRestart,
    kMarkAttach = reg::target_console::kMarkKindAttach, kMarkDetach = reg::target_console::kMarkKindDetach,
    kMarkLost = reg::target_console::kMarkKindLost, kMarkClear = reg::target_console::kMarkKindClear,
    kMarkHost = reg::target_console::kMarkKindHost, kMarkLinkLost = reg::target_console::kMarkKindLinkLost,
  };
  TargetConsoleStream(DebugPort &port, DmConsole &driver, uint16_t instance)
      : port_(port), driver_(driver), instance_(instance), stream_(buffer_, kCapacity, marks_, kMarks) {
    driver_.setSink(&TargetConsoleStream::take, this);   // the driver writes straight into this buffer
  }
  const char *name() const override { return reg::target_console::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::target_console::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::target_console::kLockFreeOps, op); }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  void poll();   // from loop()
  // What one read may return: declare it within the probe's frame (1000 suits a 1 KiB frame).
  void setMaxRead(uint16_t bytes) { max_read_ = bytes; }

 private:
  static constexpr size_t kCapacity = 8192, kMarks = 16;   // both powers of two (wrapping positions / serials)
  DebugPort &port_;
  DmConsole &driver_;
  uint16_t instance_;
  bool exists_ = false;    // a stream was opened (and may be closed but still readable)
  bool open_ = false;
  uint8_t mechanism_ = 0;
  uint16_t max_read_ = 1000;
  uint8_t buffer_[kCapacity];
  PositionStream::Mark marks_[kMarks];
  PositionStream stream_;
  uint32_t seen_resets_ = 0, seen_resyncs_ = 0;
  static void take(void *self, uint8_t byte) { static_cast<TargetConsoleStream *>(self)->stream_.put(byte); }
};

}  // namespace v1
}  // namespace oep
