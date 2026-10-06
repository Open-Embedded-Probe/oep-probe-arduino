// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A bench's trace (off by default; never in a release build): with OEP_DEBUG_LOG defined to 1 at build time, a line
// for each request the endpoint answers (corr, fn, op, session_id, length, then resolution / detail / answer length)
// and the steps of riscv-dm's run and of the waits after a change of hart state (Ch32Dm), written to the Print the
// sketch names with oep::setLog() - a UART of its own, so the lines never mix with OEP frames. Each line starts with
// the probe's clock in us. Without OEP_DEBUG_LOG the calls compile to nothing.
#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

#ifndef OEP_DEBUG_LOG
#define OEP_DEBUG_LOG 0
#endif

namespace oep {

#if OEP_DEBUG_LOG
inline Print *g_log = nullptr;
inline void setLog(Print *out) { g_log = out; }
inline void logf(const char *format, ...) {
  if (!g_log) return;
  char line[160];
  int n = snprintf(line, sizeof line, "%lu ", static_cast<unsigned long>(micros()));
  if (n < 0) return;
  va_list args;
  va_start(args, format);
  const int m = vsnprintf(line + n, sizeof line - static_cast<size_t>(n) - 2, format, args);
  va_end(args);
  if (m > 0) n += m < static_cast<int>(sizeof line) - n - 2 ? m : static_cast<int>(sizeof line) - n - 3;
  line[n++] = '\r';
  line[n++] = '\n';
  g_log->write(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(n));
}
#define OEP_LOGF(...) ::oep::logf(__VA_ARGS__)
#else
inline void setLog(Print *) {}
#define OEP_LOGF(...) \
  do {                \
  } while (0)
#endif

}  // namespace oep
