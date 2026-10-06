// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The little of the Arduino API the portable core of the library uses, for host tests (g++): Stream, millis(), and
// enough of the pin / UART API for OepPlatform.h and the fixtures (pin modes recorded, the UART does nothing).
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern uint32_t g_millis;
inline uint32_t g_micros_part = 0;   // the microseconds past g_millis (a test that keeps finer time: advanceMicros)
inline uint32_t millis() { return g_millis; }
inline uint32_t micros() { return g_millis * 1000u + g_micros_part; }
inline void advanceMicros(uint32_t us) { g_micros_part += us; g_millis += g_micros_part / 1000; g_micros_part %= 1000; }
inline void delay(uint32_t ms) { g_millis += ms; }
// A busy wait in the code under test: time goes on, and a test may complete USB transfers meanwhile.
extern void (*g_on_wait)();
inline void delayMicroseconds(uint32_t us) { advanceMicros(us); if (g_on_wait) g_on_wait(); }
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

// The host has no chip number: the build constant OepPlatform.h asks for on such a platform (core §7.5).
#if !defined(OEP_UNIT_ID) && !defined(OEP_HOST_NO_UNIT_ID)
#define OEP_UNIT_ID "host-test"
#endif

#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define INPUT_PULLDOWN 0x09
#define OUTPUT_OPEN_DRAIN 0x13
#define LOW 0
#define HIGH 1
// Pin modes and output levels are recorded (pin 0-63) so a test can see what a pad was left at: g_pin_mode (-1 never
// set), g_pin_level (the output latch), and g_pin_changes (every pinMode / digitalWrite that changed something).
inline int g_pin_mode[64] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                             -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                             -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
inline int g_pin_level[64] = {};
// The pad strength OepPlatform.h set (OEP_HOST_FAKE_DRIVE: four levels, default 2), -1 never set.
inline int g_pin_drive[64] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                              -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
inline int g_pin_changes = 0;
inline void pinMode(int pin, int mode) {
  if (pin < 0 || pin >= 64) return;
  if (g_pin_mode[pin] != mode) ++g_pin_changes;
  g_pin_mode[pin] = mode;
}
inline void (*g_on_pin)(int pin) = nullptr;   // a test's look at every output level written (a simulated target's reset line)
inline void digitalWrite(int pin, int level) {
  if (pin < 0 || pin >= 64) return;
  if (g_pin_level[pin] != level) ++g_pin_changes;
  g_pin_level[pin] = level;
  if (g_on_pin) g_on_pin(pin);
}
inline int digitalRead(int pin) { return pin >= 0 && pin < 64 && g_pin_level[pin]; }
enum : uint32_t {
  SERIAL_7N1 = 0x8000018, SERIAL_7N2 = 0x8000038, SERIAL_7E1 = 0x800001a, SERIAL_7E2 = 0x800003a, SERIAL_7O1 = 0x800001b,
  SERIAL_7O2 = 0x800003b, SERIAL_8N1 = 0x800001c, SERIAL_8N2 = 0x800003c, SERIAL_8E1 = 0x800001e, SERIAL_8E2 = 0x800003e,
  SERIAL_8O1 = 0x800001f, SERIAL_8O2 = 0x800003f,
};
class HardwareSerial {
 public:
  void end() {   // the driver (its ring / queue and events) goes with it
    fake_running = false;
    fake_head = fake_tail = 0;
    fake_ev_head = fake_ev_tail = 0;
  }
  int available() { return static_cast<int>(fake_tail - fake_head); }
  size_t readBytes(uint8_t *b, size_t n) {
    size_t k = 0;
    for (; k < n && fake_head < fake_tail; ++k) b[k] = fake_rx[fake_head++ % sizeof fake_rx];
    return k;
  }
  int availableForWrite() { return 0; }
  size_t write(const uint8_t *, size_t n) { return n; }
  // OEP_HOST_FAKE_UART (OepPlatform.h): a UART that platformUartBegin starts and whose receive a test feeds.
  // ESP-IDF style: the driver's ring and its events in order (fakeReceive / fakeChunk queue UART_DATA, fakeEvent any
  // other; fake_hold_events: the event task has not taken them yet; fake_event_queue: the queue's length, what is
  // posted to it full is dropped). arduino-pico style (OEP_HOST_FAKE_UART_RP2): the receive queue of fake_cap bytes
  // that drops what comes while it is full (overflow()), the PL011's overrun (fake_overrun, platformUartTakeOverrun)
  // and a break (fake_break, getBreakReceived).
  bool fakeBegin(uint32_t baud, int rx, int tx, uint32_t config, int irq_core) {
    (void)rx; (void)tx; (void)config;
    fake_baud = baud;
    fake_irq_core = irq_core;
    fake_running = true;
    ++fake_begins;
    return true;
  }
  bool fakeTakeOverrun() {
    const bool o = fake_overrun;
    fake_overrun = false;
    return o;
  }
  bool overflow() {
    const bool o = fake_queue_overflow;
    fake_queue_overflow = false;
    return o;
  }
  bool getBreakReceived() {
    const bool o = fake_break;
    fake_break = false;
    return o;
  }
  bool fakePush(uint8_t byte) {
    if (fake_tail - fake_head >= fake_cap) { fake_queue_overflow = true; return false; }
    fake_rx[fake_tail++ % sizeof fake_rx] = byte;
    return true;
  }
  void fakeEvent(uint8_t type, uint32_t size = 0) {
    if (fake_ev_tail - fake_ev_head >= fake_event_queue) return;   // the driver's queue full: dropped
    fake_events[fake_ev_tail % kFakeEvents] = {type, size};
    ++fake_ev_tail;
  }
  void fakeReceive(uint8_t byte) { fakeChunk(&byte, 1); }
  void fakeChunk(const uint8_t *b, size_t n, uint8_t type = 0 /* UartEvent::kData */) {
    size_t k = 0;
    while (k < n && fakePush(b[k])) ++k;
    if (k) fakeEvent(type, static_cast<uint32_t>(k));
  }
  bool fakeNextEvent(uint8_t &type, uint32_t &size, uint32_t &left) {
    if (fake_hold_events || fake_ev_head == fake_ev_tail) return false;
    type = fake_events[fake_ev_head % kFakeEvents].type;
    size = fake_events[fake_ev_head % kFakeEvents].size;
    ++fake_ev_head;
    left = static_cast<uint32_t>(fake_ev_tail - fake_ev_head);
    return true;
  }
  bool fakeEventsHeld() const { return fake_hold_events && fake_ev_head != fake_ev_tail; }
  static constexpr size_t kFakeEvents = 16384;
  struct FakeEvent { uint8_t type; uint32_t size; } fake_events[kFakeEvents];
  size_t fake_ev_head = 0, fake_ev_tail = 0;
  bool fake_hold_events = false;
  uint32_t fake_event_queue = kFakeEvents;   // the driver's event queue length (arduino-esp32: 20)
  uint8_t fake_rx[16384];
  size_t fake_head = 0, fake_tail = 0, fake_cap = 16384;
  bool fake_overrun = false, fake_running = false, fake_queue_overflow = false, fake_break = false;
  uint32_t fake_baud = 0;
  int fake_irq_core = -2, fake_begins = 0;
};
