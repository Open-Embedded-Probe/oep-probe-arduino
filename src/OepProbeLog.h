// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A log of the probe's own code, for an interface of the sketch's own to give the host (oep-spec docs/oep-if-common.ja.md
// §1): the lines go into a position stream in RAM - nothing is sent while the code runs - and each line's start gets a
// mark of kind kMarkKindLine with the probe's clock (time_ns: the clock of core §7.7's uptime_ns and of a capture's
// start_ns), so the host can place every line in a capture taken meanwhile: sample = (time_ns - the segment's start_ns) x
// its rate, within the segment's start_uncertainty_ns.
//
//   log.line("## write 0x50");               a line: a mark, the text, '\n' (a few hundred ns: a copy into the ring)
//   log.printf("result=%d", r);              the same, formatted (up to kLineMax characters)
//
// The host reads it with the stream ops of oep.target.console (same numbers and forms, without the stream number):
// read, marks, clear. Reading does not consume. When the ring is full the oldest bytes go and a lost mark (overflow)
// says so; marks that the mark ring pushed out show as a jump in their serials - a host that sees either knows lines
// are missing.
//
// Call it from loop() or from the interface's handle() (the endpoint's thread): it takes no lock.
#pragma once

#include <stdarg.h>
#include <stdio.h>

#include "Oep.h"
#include "OepStream.h"

namespace oep {

class ProbeLog {
 public:
  static constexpr uint8_t kMarkKindLine = 0x40;   // the interface's own mark kind (common §1.3): a line begins here
  static constexpr size_t kLineMax = 160;
  static constexpr uint8_t kOpRead = reg::target_console::kOpRead, kOpMarks = reg::target_console::kOpMarks,
                           kOpClear = reg::target_console::kOpClear;

  // capacity: a power of two
  ProbeLog(uint8_t *buffer, size_t capacity, PositionStream::Mark *marks, size_t mark_capacity)
      : stream_(buffer, capacity, marks, mark_capacity) {}

  void line(const char *text) {
    stream_.mark(kMarkKindLine);
    for (const char *c = text; *c; ++c) stream_.put(static_cast<uint8_t>(*c));
    stream_.put('\n');
  }
  void printf(const char *format, ...) __attribute__((format(printf, 2, 3))) {
    char text[kLineMax + 1];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof text, format, args);
    va_end(args);
    line(text);
  }

  // The stream ops (the interface's handle() passes them on); read and marks change nothing: lock-free.
  static bool handles(uint8_t op) { return op == kOpRead || op == kOpMarks || op == kOpClear; }
  static bool lockFree(uint8_t op) { return op == kOpRead || op == kOpMarks; }
  Result handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity, uint16_t max_read = 1000) {
    Tail tail;
    if (op == kOpRead) {   // from(u8) arg(u64) max(u16) [TLV]  ->  start(u64) flags(u8) len(u16) data [TLV]
      const Result parsed = plainTail(tail, p, n, PositionStream::kReadRequest, out, capacity);
      if (refused(parsed)) return parsed;
      const Result values = PositionStream::checkRead(p, out, capacity);
      if (refused(values)) return values;
      return stream_.read(p, out, capacity, max_read);
    }
    if (op == kOpMarks) {   // from_serial(u32) [TLV]  ->  more(u8) count(u8) count x mark [TLV]
      const Result parsed = plainTail(tail, p, n, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 2) return failed();
      return completed(stream_.marks(getU32(p), out, capacity));
    }
    const Result parsed = plainTail(tail, p, n, 0, out, capacity);   // clear
    if (refused(parsed)) return parsed;
    stream_.clear();
    return completed();
  }

 private:
  PositionStream stream_;
};

}  // namespace oep
