// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Running a library under test on the probe itself, captured as it runs: a file to put beside OepProbe.ino (see README).
// The firmware keeps every standard interface (the logic capture among them); this adds io.github.example.library-test:
//
//   op 0x10 run   case(u8)  ->  result(i32)     one test case, run to its end inside the request (keep it short: the
//                                                probe answers nothing else meanwhile)
//   op 0x11 read, 0x12 marks, 0x13 clear        its log (OepProbeLog.h): every line the case wrote, each with the probe's
//                                                clock, so the host places it in the capture taken meanwhile
//
// The cases use Wire on two pins the sketch keeps for itself (owner kSketch: no other interface is planned on them), and
// the logic capture listens to the same pins (a capture only listens, oep-if-capture §1.2): plan it on SDA and SCL, start
// it, run a case, stop it, read both. Nothing has to be wired: an address nobody answers still gives START, the address,
// a NACK and STOP on the lines.
#include <OepEndpoint.h>
#include <OepPinTable.h>
#include <OepProbeLog.h>
#include <Wire.h>

#include "OepFeatures.h"

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

// the log's ring and marks from the heap at setup (a classic ESP32's firmware leaves no static RAM for them)
constexpr size_t kLogBytes = 8192, kLogMarks = 256;
oep::ProbeLog *log_ = nullptr;

// The cases: what a test of a library would do, here Wire itself
int32_t writeAbsent() {
  log_->line("## write 2 bytes to 0x50");
  bus.beginTransmission(kAbsent);
  bus.write(0x00);
  bus.write(0x01);
  const int32_t r = bus.endTransmission();   // 2: NACK on the address
  log_->printf("endTransmission=%ld", static_cast<long>(r));
  log_->line("##");
  return r;
}

int32_t readAbsent() {
  log_->line("## read 2 bytes from 0x50");
  const int32_t got = bus.requestFrom(kAbsent, static_cast<uint8_t>(2));   // 0: nobody answered
  log_->printf("requestFrom=%ld", static_cast<long>(got));
  log_->line("##");
  return got;
}

int32_t scan() {
  log_->line("## scan 0x08 - 0x77");
  int32_t found = 0;
  for (uint8_t a = 0x08; a <= 0x77; ++a) {
    log_->printf("addr 0x%02x", a);
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) {
      log_->printf("found 0x%02x", a);
      ++found;
    }
  }
  log_->printf("found=%ld", static_cast<long>(found));
  log_->line("##");
  return found;
}

class LibraryTest final : public oep::Interface {
 public:
  static constexpr uint8_t kOpRun = 0x10;   // 0x01 - 0x0F: the ops every interface shares
  const char *name() const override { return "io.github.example.library-test"; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }
  bool offers(uint8_t op) const override { return op == kOpRun || oep::ProbeLog::handles(op); }
  bool lockFree(uint8_t op) const override { return oep::ProbeLog::lockFree(op); }
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 16 ? static_cast<uint16_t>(max_frame - 16) : 0; }

  oep::Result handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) override {
    if (oep::ProbeLog::handles(op)) return log_->handle(op, p, n, out, capacity, max_read_);
    oep::Tail tail;
    const oep::Result parsed = oep::plainTail(tail, p, n, 1, out, capacity);
    if (oep::refused(parsed)) return parsed;
    if (capacity < 4) return oep::failed();
    int32_t r;
    switch (p[0]) {
      case 1: r = writeAbsent(); break;
      case 2: r = readAbsent(); break;
      case 3: r = scan(); break;
      default: return oep::unsupportedValue(out, capacity);   // a case it does not have
    }
    oep::putU32(out, static_cast<uint32_t>(r));
    return oep::completed(4);
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
