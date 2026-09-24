// OEP v1 draft console stream (oep-spec docs/console-stream.ja.md, capability-name-hierarchy.ja.md):
// a stream opened on the debug connection (SDI / DMDATA / dmseq through the v0 TargetConsole driver),
// kept in a position-addressed buffer that reads do not consume, with marks for what happened.
//
//   0x01 open(conn u8, mechanism u8)              -> stream u8          mechanism 0 SDI, 1 DMDATA, 2 dmseq
//   0x02 read(stream, from u8, arg u32, max u16)  -> start u32, flags u8 (bit0 more, bit1 gap), data   no lock
//        from: 0 position = arg, 1 oldest, 2 now, 3 last mark of kind arg (0 = any)
//   0x03 marks(stream, from u32)                  -> count u8, (position u32, kind u8, time_ms u32, detail u8)...  no lock
//   0x04 clear(stream)   0x05 mark(stream, value u8)   0x06 write(stream, data) -> accepted u16   0x07 close(stream)
//
// Collection goes on whatever the host does (sessions and lock lapses change nothing); bytes go only when the
// buffer overflows, on clear, or when the probe restarts.
#pragma once

#include "OepTargetServices.h"
#include "OepV1Target.h"

namespace oep {
namespace v1 {

class TargetConsoleStream final : public Interface {
 public:
  enum : uint8_t { kOpOpen = 0x01, kOpRead = 0x02, kOpMarks = 0x03, kOpClear = 0x04, kOpMark = 0x05,
                   kOpWrite = 0x06, kOpClose = 0x07 };
  enum : uint8_t { kMarkReset = 0x01, kMarkRestart = 0x02, kMarkAttach = 0x03, kMarkDetach = 0x04,
                   kMarkLost = 0x05, kMarkClear = 0x06, kMarkHost = 0x07, kMarkLinkLost = 0x08 };
  TargetConsoleStream(DebugPort &port, TargetConsole &driver, uint16_t instance)
      : port_(port), driver_(driver), instance_(instance) {}
  const char *name() const override { return "oep.target.console"; }
  uint16_t instance() const override { return instance_; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return op == kOpRead || op == kOpMarks; }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  void poll();   // from loop()

 private:
  static constexpr size_t kCapacity = 8192, kMarks = 16;
  struct Mark { uint32_t position; uint8_t kind; uint32_t time_ms; uint8_t detail; };
  DebugPort &port_;
  TargetConsole &driver_;
  uint16_t instance_;
  bool open_ = false;
  uint8_t buffer_[kCapacity];
  uint32_t total_ = 0;   // bytes ever collected = the position of the next byte
  uint32_t base_ = 0;    // nothing before this position is kept (clear)
  Mark marks_[kMarks];
  uint32_t mark_count_ = 0;
  uint32_t seen_resets_ = 0, seen_resyncs_ = 0, seen_dropped_ = 0;
  uint32_t oldest() const;
  void mark(uint8_t kind, uint8_t detail = 0);
  void collect();
};

}  // namespace v1
}  // namespace oep
