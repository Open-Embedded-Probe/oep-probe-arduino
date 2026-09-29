// The little of the Arduino API the portable core of the library uses, for host tests (g++): Stream, millis().
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern uint32_t g_millis;
inline uint32_t millis() { return g_millis; }
inline void delay(uint32_t ms) { g_millis += ms; }

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
