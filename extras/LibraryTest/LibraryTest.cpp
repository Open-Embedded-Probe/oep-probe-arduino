// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Running a library under test on the probe itself, captured as it runs: a file to put beside a firmware sketch whose
// setup() calls libraryTestRegister() before oep::firmware.begin() (see README).
// The firmware keeps every standard interface (the logic capture among them); this adds io.github.example.library-test,
// an oep::SimpleInterface (its ops' names and fields in its describe, `oep call` and oep_client.call by name):
//
//   op 0x10 run     case(u8)  ->  -                 a job: starts the case on the worker and answers at once (busy while
//                                                   one runs; a case it does not have ends failed)
//   op 0x20 status, 0x21 cancel                     the worker's (addWorker): state, op, code, message, start_ns, end_ns
//   op 0x22 result  -         ->  case(u8) result(i32)    the last case's number and what it returned (lock-free)
//   op 0x11 log-read, 0x12 log-marks, 0x13 log-clear      its log (addLog): every line the case wrote, each with the
//                                                   probe's clock, to place it in a capture
//
// A case runs on the worker - a FreeRTOS task off loop()'s core on an ESP32 / ESP32-P4 (core 0 on a P4), the second core
// (loop1, worker.poll()) on an RP2040 / RP2350 - so it may take as long as it needs (a display's init and drawing, a
// power IC's sequence, delays in the library) while the probe keeps answering: the host's capture, its lease, status.
// The bus is begun there too, at the first case: its interrupts on the worker's core.
//
// The cases use Wire on two pins the sketch keeps for itself (owner kSketch: no other interface is planned on them), and
// the logic capture listens to the same pins (a capture only listens, oep-if-capture §1.2). Nothing has to be wired: an
// address nobody answers still gives START, the address, a NACK and STOP on the lines.
#include <OepFirmwareParts.h>
#include <OepProbeLog.h>
#include <OepSimpleInterface.h>
#include <OepWorker.h>
#include <Wire.h>

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

oep::SimpleInterface libraryTest("io.github.example.library-test", 1);
oep::Worker worker;
oep::ProbeLog *log_ = nullptr;   // from the heap at setup (a classic ESP32's firmware leaves no static RAM for it)

void mark(const char *text) { log_->line(text); }
#define markf(...) log_->printf(__VA_ARGS__)

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

// ---- the interface -------------------------------------------------------------------------------------------------
volatile uint8_t lastCase = 0;
volatile int32_t lastResult = 0;

void beginBus() {   // on the worker, at the first case
  static bool begun = false;
  if (begun) return;
#if defined(ARDUINO_ARCH_RP2040)
  bus.setSDA(kSda);
  bus.setSCL(kScl);
  bus.begin();
#else
  bus.begin(kSda, kScl);
#endif
  bus.setClock(100000);
  begun = true;
}

void run(oep::Args &in, oep::Worker &w) {
  const uint8_t which = in.u8();
  if (which < 1 || which > 4) {
    w.fail(1, "cases are 1 - 4");
    return;
  }
  beginBus();
  int32_t r = 0;
  switch (which) {
    case 1: r = writeAbsent(); break;
    case 2: r = readAbsent(); break;
    case 3: r = scan(); break;
    default: r = sequence(); break;
  }
  lastResult = r;
  lastCase = which;
}

oep::Result result(oep::Args &, oep::Reply &out) {
  out.u8(lastCase);
  out.i32(lastResult);
  return out.done();
}

}  // namespace

// The case interface and its bus pins, registered with the firmware (called before oep::firmware.begin())
void libraryTestRegister() {
  static oep::ProbeLog log(8192, 256);
  log_ = &log;
  libraryTest.addWorker(worker, 0x20);
  libraryTest.addLog(log);
  libraryTest.job(0x10, "run", {{"case", oep::u8}}, run);
  libraryTest.op(0x22, "result", {}, {{"case", oep::u8}, {"result", oep::i32}}, result, oep::kLockFree);
  oep::firmware.onSetup([](oep::Endpoint &, oep::PinTable &pins) {
    pins.claim(kSda, kOwner);   // the bus is the sketch's: no other interface is planned on it (the capture listens)
    pins.claim(kScl, kOwner);
  });
  oep::firmware.add(libraryTest);
}

#if defined(ARDUINO_ARCH_RP2040)
// The second core runs the cases (setup1 waits for setup's begin(): loop1 starts after it)
void loop1() { worker.poll(); }
#endif
