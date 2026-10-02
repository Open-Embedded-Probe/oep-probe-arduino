// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The little of the Arduino API the portable core of the library uses, for host tests (g++): Stream, millis(), and
// enough of the pin / UART API for OepPlatform.h (pin modes do nothing).
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern uint32_t g_millis;
inline uint32_t millis() { return g_millis; }
inline uint32_t micros() { return g_millis * 1000u; }
inline void delay(uint32_t ms) { g_millis += ms; }
// A busy wait in the code under test: time goes on, and a test may complete USB transfers meanwhile.
extern void (*g_on_wait)();
inline void delayMicroseconds(uint32_t us) { static uint32_t acc = 0; acc += us; if (acc >= 1000) { g_millis += acc / 1000; acc %= 1000; } if (g_on_wait) g_on_wait(); }
template <class A, class B> inline auto min(A a, B b) -> decltype(a < b ? a : b) { return a < b ? a : b; }

class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t *b, size_t n) { size_t k = 0; while (k < n && write(b[k])) ++k; return k; }
  virtual int availableForWrite() { return 0; }
  virtual void flush() {}
};

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual size_t readBytes(char *b, size_t n) { size_t k = 0; int c; while (k < n && (c = read()) >= 0) b[k++] = static_cast<char>(c); return k; }
  size_t readBytes(uint8_t *b, size_t n) { return readBytes(reinterpret_cast<char *>(b), n); }
};

#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define INPUT_PULLDOWN 0x09
#define OUTPUT_OPEN_DRAIN 0x13
#define LOW 0
#define HIGH 1
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
enum : uint32_t {
  SERIAL_7N1 = 0x8000018, SERIAL_7N2 = 0x8000038, SERIAL_7E1 = 0x800001a, SERIAL_7E2 = 0x800003a, SERIAL_7O1 = 0x800001b,
  SERIAL_7O2 = 0x800003b, SERIAL_8N1 = 0x800001c, SERIAL_8N2 = 0x800003c, SERIAL_8E1 = 0x800001e, SERIAL_8E2 = 0x800003e,
  SERIAL_8O1 = 0x800001f, SERIAL_8O2 = 0x800003f,
};
class HardwareSerial {};
