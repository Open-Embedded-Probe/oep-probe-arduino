// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Running a library under test on the probe itself, captured as it runs: a file to put beside OepProbe.ino (see README).
// The firmware keeps every standard interface (the logic capture among them); this adds io.github.example.library-test:
//
//   op 0x10 run     case(u8)  ->  -                           starts the case and answers at once (busy: unavailable)
//   op 0x20 status  -         ->  state(u8: 0 idle, 1 running, 2 done) case(u8) result(i32)
//   op 0x11 read, 0x12 marks, 0x13 clear                       its log (OepProbeLog.h): every line the case wrote, each
//                                                              with the probe's clock, to place it in a capture
//
// A case runs on a thread of its own - a FreeRTOS task on an ESP32 / ESP32-P4, the second core (setup1 / loop1) on an
// RP2040 / RP2350 - so it may take as long as it needs (a display's init and drawing, a power IC's sequence, delays in the
// library) while the probe keeps answering: the host's capture, its lease, status. The case's code is plain sequential
// code; it writes its markers with mark() (the log is shared with the endpoint's thread: mark() and the log's ops take
// a lock for the copy).
//
// The cases use Wire on two pins the sketch keeps for itself (owner kSketch: no other interface is planned on them), and
// the logic capture listens to the same pins (a capture only listens, oep-if-capture §1.2). Nothing has to be wired: an
// address nobody answers still gives START, the address, a NACK and STOP on the lines.
#include <OepEndpoint.h>
#include <OepPinTable.h>
#include <OepProbeLog.h>
#include <Wire.h>

#include "OepFeatures.h"

#if defined(ARDUINO_ARCH_RP2040)
#include <pico/sync.h>
#endif

namespace {

#if defined(CONFIG_IDF_TARGET_ESP32P4)
constexpr int kSda = 32, kScl = 33;
TwoWire &bus = Wire;
#elif defined(CONFIG_IDF_TARGET_ESP32)
constexpr int kSda = 25, kScl = 26;
TwoWire &bus = Wire;
#elif defined(ARDUINO_ARCH_RP2040)
constexpr int kSda = 26, kScl = 27;   // I2C1's pins
TwoWire &bus = Wire1;
#else
#error "LibraryTest: an ESP32-P4, a classic ESP32 or an RP2040 / RP2350"
#endif
constexpr uint8_t kOwner = oep::owners::kSketch;
constexpr uint8_t kAbsent = 0x50;   // an address nobody answers on an empty bus

// ---- the log, shared by the case's thread and the endpoint's -------------------------------------------------------
// the ring and marks from the heap at setup (a classic ESP32's firmware leaves no static RAM for them)
constexpr size_t kLogBytes = 8192, kLogMarks = 256;
oep::ProbeLog *log_ = nullptr;
#if defined(ARDUINO_ARCH_RP2040)
critical_section_t logLock;
void lockLog() { critical_section_enter_blocking(&logLock); }
void unlockLog() { critical_section_exit(&logLock); }
#else
portMUX_TYPE logLock = portMUX_INITIALIZER_UNLOCKED;
void lockLog() { portENTER_CRITICAL(&logLock); }
void unlockLog() { portEXIT_CRITICAL(&logLock); }
#endif

void mark(const char *text) {
  lockLog();
  log_->line(text);
  unlockLog();
}

void markf(const char *format, ...) __attribute__((format(printf, 1, 2)));
void markf(const char *format, ...) {
  char text[oep::ProbeLog::kLineMax + 1];
  va_list args;
  va_start(args, format);
  vsnprintf(text, sizeof text, format, args);   // formatted outside the lock
  va_end(args);
  mark(text);
}

// ---- the cases: what a test of a library would do, here Wire itself --------------------------------------------------
int32_t writeAbsent() {
  mark("## write 2 bytes to 0x50");
  bus.beginTransmission(kAbsent);
  bus.write(0x00);
  bus.write(0x01);
  const int32_t r = bus.endTransmission();   // not 0: nobody answered (2 NACK; 4 on a bus with no pull-ups)
  markf("endTransmission=%ld", static_cast<long>(r));
  mark("##");
  return r;
}

int32_t readAbsent() {
  mark("## read 2 bytes from 0x50");
  const int32_t got = bus.requestFrom(kAbsent, static_cast<uint8_t>(2));   // 0: nobody answered
  markf("requestFrom=%ld", static_cast<long>(got));
  mark("##");
  return got;
}

int32_t scan() {
  mark("## scan 0x08 - 0x77");
  int32_t found = 0;
  for (uint8_t a = 0x08; a <= 0x77; ++a) {
    markf("addr 0x%02x", a);
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) {
      markf("found 0x%02x", a);
      ++found;
    }
  }
  markf("found=%ld", static_cast<long>(found));
  mark("##");
  return found;
}

// A display-like sequence: an init of a few commands with waits between them, then a frame drawn in rows - as a
// display or power IC library would run it, long and full of delays (here to the absent address: the lines show it all)
int32_t sequence() {
  mark("## init");
  static const uint8_t kInit[] = {0xAE, 0xD5, 0xA8, 0x8D, 0xAF};
  for (uint8_t cmd : kInit) {
    markf("cmd 0x%02x", cmd);
    bus.beginTransmission(kAbsent);
    bus.write(0x00);
    bus.write(cmd);
    bus.endTransmission();
    delay(10);
  }
  mark("##");
  mark("## draw");
  for (uint8_t row = 0; row < 8; ++row) {
    markf("row %u", row);
    bus.beginTransmission(kAbsent);
    bus.write(0x40);
    for (uint8_t i = 0; i < 16; ++i) bus.write(static_cast<uint8_t>(row * 16 + i));
    bus.endTransmission();
    delay(5);
  }
  mark("##");
  return 0;
}

// ---- the case's thread ---------------------------------------------------------------------------------------------
enum : uint8_t { kIdle = 0, kRunning = 1, kDone = 2 };
volatile uint8_t state = kIdle, current = 0;
volatile int32_t result = 0;

int32_t runCase(uint8_t which) {
  switch (which) {
    case 1: return writeAbsent();
    case 2: return readAbsent();
    case 3: return scan();
    default: return sequence();
  }
}

void finish(int32_t r) {
  result = r;
  __atomic_thread_fence(__ATOMIC_RELEASE);
  state = kDone;
}

#if !defined(ARDUINO_ARCH_RP2040)
void caseTask(void *) {
  finish(runCase(current));
  vTaskDelete(nullptr);
}
#endif

bool start(uint8_t which) {
  if (state == kRunning) return false;
  current = which;
  result = 0;
  __atomic_thread_fence(__ATOMIC_RELEASE);   // the case and its number before the state the other thread looks at
  state = kRunning;
#if defined(ARDUINO_ARCH_RP2040)
  return true;   // loop1 on the second core takes it
#else
  // below the endpoint's loop() and the capture's tasks: they keep their pace, the case gets the rest
  if (xTaskCreate(caseTask, "library_test", 8192, nullptr, 1, nullptr) == pdPASS) return true;
  state = kIdle;
  return false;
#endif
}

class LibraryTest final : public oep::Interface {
 public:
  static constexpr uint8_t kOpRun = 0x10, kOpStatus = 0x20;   // 0x01 - 0x0F: the ops every interface shares
  const char *name() const override { return "io.github.example.library-test"; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }
  bool offers(uint8_t op) const override { return op == kOpRun || op == kOpStatus || oep::ProbeLog::handles(op); }
  bool lockFree(uint8_t op) const override { return op == kOpStatus || oep::ProbeLog::lockFree(op); }
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 16 ? static_cast<uint16_t>(max_frame - 16) : 0; }

  oep::Result handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) override {
    if (oep::ProbeLog::handles(op)) {
      lockLog();
      const oep::Result r = log_->handle(op, p, n, out, capacity, max_read_);
      unlockLog();
      return r;
    }
    oep::Tail tail;
    const oep::Result parsed = oep::plainTail(tail, p, n, op == kOpRun ? 1 : 0, out, capacity);
    if (oep::refused(parsed)) return parsed;
    if (op == kOpRun) {
      if (p[0] < 1 || p[0] > 4) return oep::unsupportedValue(out, capacity);   // a case it does not have
      if (!start(p[0])) return oep::wrongState(out, capacity);                // one running already (cause 6)
      return oep::completed();
    }
    if (capacity < 6) return oep::failed();
    out[0] = state;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    out[1] = current;
    oep::putU32(out + 2, static_cast<uint32_t>(result));
    return oep::completed(6);
  }

 private:
  uint16_t max_read_ = 1000;
};

LibraryTest libraryTest;

}  // namespace

void oepUserSetup(oep::Endpoint &endpoint, oep::PinTable &pins) {
  static oep::ProbeLog log(static_cast<uint8_t *>(malloc(kLogBytes)), kLogBytes,
                           static_cast<oep::PositionStream::Mark *>(malloc(sizeof(oep::PositionStream::Mark) * kLogMarks)),
                           kLogMarks);
  log_ = &log;
#if defined(ARDUINO_ARCH_RP2040)
  critical_section_init(&logLock);
#endif
  pins.claim(kSda, kOwner);   // the bus is the sketch's: no other interface is planned on it (the capture listens)
  pins.claim(kScl, kOwner);
#if defined(ARDUINO_ARCH_RP2040)
  bus.setSDA(kSda);
  bus.setSCL(kScl);
  bus.begin();
#else
  bus.begin(kSda, kScl);
#endif
  bus.setClock(100000);
  endpoint.add(libraryTest);
}

#if defined(ARDUINO_ARCH_RP2040)
// The second core: a case when one is asked for (setup1 waits for setup's oepUserSetup: loop1 starts after it)
void loop1() {
  if (state == kRunning && log_) finish(runCase(current));
}
#endif
