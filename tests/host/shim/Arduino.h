// The little of the Arduino API the portable core of the library uses, for host tests (g++): Stream, millis().
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern uint32_t g_millis;
inline uint32_t millis() { return g_millis; }
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
