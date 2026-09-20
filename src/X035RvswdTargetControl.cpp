#include "X035RvswdTargetControl.h"

#include <string.h>

#if defined(ARDUINO_ARCH_ESP32)

#include <driver/gpio.h>

namespace oep::prototype {
namespace {
constexpr uint8_t kData0 = 0x04;
constexpr uint8_t kData1 = 0x05;
constexpr uint8_t kDmControl = 0x10;
constexpr uint8_t kDmStatus = 0x11;
constexpr uint8_t kDmHartInfo = 0x12;
constexpr uint8_t kAbstractCs = 0x16;
constexpr uint8_t kCommand = 0x17;
constexpr uint8_t kAbstractAuto = 0x18;
constexpr uint8_t kProgBuf0 = 0x20;
constexpr uint32_t kFlashStatr = 0x4002200c;
constexpr uint32_t kFlashKeyr = 0x40022004;
constexpr uint32_t kFlashObkeyr = 0x40022008;
constexpr uint32_t kFlashCtlr = 0x40022010;
constexpr uint32_t kFlashAddr = 0x40022014;
constexpr uint32_t kFlashModekeyr = 0x40022024;

uint32_t get32(const uint8_t* p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 |
         uint32_t(p[3]) << 24;
}
void put32(uint8_t* p, uint32_t v) {
  p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}
}  // namespace

void X035RvswdTargetControl::begin() { releaseBus(); }

void X035RvswdTargetControl::releaseBus() {
  pinMode(swdio_, INPUT);
  pinMode(swclk_, INPUT);
}

void X035RvswdTargetControl::initializeBus() {
  pinMode(swclk_, OUTPUT);
  pinMode(swdio_, OUTPUT_OPEN_DRAIN | PULLUP);
  gpio_set_drive_capability(gpio_num_t(swdio_), GPIO_DRIVE_CAP_0);
  gpio_set_drive_capability(gpio_num_t(swclk_), GPIO_DRIVE_CAP_0);
  digitalWrite(swclk_, HIGH);
  digitalWrite(swdio_, HIGH);
  delayMicroseconds(20);
  for (int i = 0; i < 100; ++i) {
    digitalWrite(swclk_, LOW); digitalWrite(swdio_, HIGH);
    if (half_period_us_) delayMicroseconds(half_period_us_);
    digitalWrite(swclk_, HIGH);
    if (half_period_us_) delayMicroseconds(half_period_us_);
  }
  digitalWrite(swclk_, LOW); digitalWrite(swdio_, LOW);
  if (half_period_us_) delayMicroseconds(half_period_us_);
  digitalWrite(swclk_, HIGH);
  if (half_period_us_) delayMicroseconds(half_period_us_);
  digitalWrite(swdio_, HIGH);
  delayMicroseconds(20);
}

static void clockBit(uint8_t dio, uint8_t clk, unsigned delay_us, bool value) {
  pinMode(dio, OUTPUT_OPEN_DRAIN | PULLUP);
  digitalWrite(clk, LOW); digitalWrite(dio, value ? HIGH : LOW);
  if (delay_us) delayMicroseconds(delay_us);
  digitalWrite(clk, HIGH);
  if (delay_us) delayMicroseconds(delay_us);
}

static bool sampleBit(uint8_t dio, uint8_t clk, unsigned delay_us) {
  digitalWrite(clk, LOW); pinMode(dio, INPUT);
  if (delay_us) delayMicroseconds(delay_us);
  const bool value = digitalRead(dio);
  digitalWrite(clk, HIGH);
  if (delay_us) delayMicroseconds(delay_us);
  return value;
}

static void startFrame(uint8_t dio, uint8_t clk, unsigned delay_us) {
  pinMode(clk, OUTPUT); pinMode(dio, OUTPUT_OPEN_DRAIN | PULLUP);
  digitalWrite(clk, HIGH); digitalWrite(dio, HIGH);
  if (delay_us) delayMicroseconds(delay_us);
  digitalWrite(dio, LOW);
  if (delay_us) delayMicroseconds(delay_us);
}

static void sendHeader(uint8_t dio, uint8_t clk, unsigned delay_us,
                       uint8_t address, bool write) {
  bool parity = write;
  for (int bit = 6; bit >= 0; --bit) {
    const bool v = (address >> bit) & 1;
    parity ^= v; clockBit(dio, clk, delay_us, v);
  }
  clockBit(dio, clk, delay_us, write);
  clockBit(dio, clk, delay_us, parity);
}

static void auxiliary(uint8_t dio, uint8_t clk, unsigned delay_us,
                      uint8_t pattern) {
  for (int bit = 4; bit >= 0; --bit)
    clockBit(dio, clk, delay_us, (pattern >> bit) & 1);
}

static void stopFrame(uint8_t dio, uint8_t clk, unsigned delay_us) {
  clockBit(dio, clk, delay_us, false);
  digitalWrite(clk, HIGH); digitalWrite(dio, HIGH);
  if (delay_us) delayMicroseconds(delay_us);
  delayMicroseconds(20);
}

bool X035RvswdTargetControl::readDmi(uint8_t address, uint32_t& value) {
  startFrame(swdio_, swclk_, half_period_us_);
  sendHeader(swdio_, swclk_, half_period_us_, address, false);
  auxiliary(swdio_, swclk_, half_period_us_, 0x15);
  uint32_t result = 0; bool parity = false;
  for (int bit = 31; bit >= 0; --bit) {
    const bool v = sampleBit(swdio_, swclk_, half_period_us_);
    result |= uint32_t(v) << bit; parity ^= v;
  }
  const bool valid = sampleBit(swdio_, swclk_, half_period_us_) == parity;
  auxiliary(swdio_, swclk_, half_period_us_, 0x17);
  stopFrame(swdio_, swclk_, half_period_us_);
  value = result;
  return valid;
}

void X035RvswdTargetControl::writeDmi(uint8_t address, uint32_t value) {
  startFrame(swdio_, swclk_, half_period_us_);
  sendHeader(swdio_, swclk_, half_period_us_, address, true);
  auxiliary(swdio_, swclk_, half_period_us_, 0x15);
  bool parity = false;
  for (int bit = 31; bit >= 0; --bit) {
    const bool v = (value >> bit) & 1;
    parity ^= v; clockBit(swdio_, swclk_, half_period_us_, v);
  }
  clockBit(swdio_, swclk_, half_period_us_, parity);
  auxiliary(swdio_, swclk_, half_period_us_, 0x17);
  stopFrame(swdio_, swclk_, half_period_us_);
}

bool X035RvswdTargetControl::waitAbstract() {
  for (int i = 0; i < 1000; ++i) {
    uint32_t v;
    if (!readDmi(kAbstractCs, v)) return false;
    if (!(v & (1u << 12))) {
      if ((v >> 8) & 7) { writeDmi(kAbstractCs, 0x700); return false; }
      return true;
    }
  }
  return false;
}

bool X035RvswdTargetControl::attachAndHalt() {
  initializeBus();
  writeDmi(kDmControl, 1);
  writeDmi(kDmControl, 0x80000001);
  for (int i = 0; i < 100; ++i) {
    uint32_t v;
    if (readDmi(kDmStatus, v) && (v & (1u << 9))) return true;
  }
  return false;
}

bool X035RvswdTargetControl::readWord(uint32_t address, uint32_t& value) {
  writeDmi(kAbstractAuto, 0);
  writeDmi(kProgBuf0, 0x0004a403);
  writeDmi(kProgBuf0 + 1, 0x00100073);
  writeDmi(kData0, address);
  writeDmi(kCommand, 0x00231009);
  if (!waitAbstract()) return false;
  writeDmi(kCommand, 0x00241000);
  if (!waitAbstract()) return false;
  writeDmi(kCommand, 0x00221008);
  return waitAbstract() && readDmi(kData0, value);
}

bool X035RvswdTargetControl::writeWord(uint32_t address, uint32_t value) {
  writeDmi(kAbstractAuto, 0);
  writeDmi(kProgBuf0, 0x0084a023);
  writeDmi(kProgBuf0 + 1, 0x00100073);
  writeDmi(kData0, address);
  writeDmi(kCommand, 0x00231009);
  if (!waitAbstract()) return false;
  writeDmi(kData0, value);
  writeDmi(kCommand, 0x00271008);
  return waitAbstract();
}

bool X035RvswdTargetControl::waitFlash() {
  for (int i = 0; i < 2000; ++i) {
    uint32_t status;
    if (!readWord(kFlashStatr, status)) return false;
    if (!(status & 1)) return (status & 0x14) == 0;
    delayMicroseconds(20);
  }
  return false;
}

bool X035RvswdTargetControl::prepareFlashWriter() {
  uint32_t info;
  if (!readDmi(kDmHartInfo, info)) return false;
  const uint32_t data0_address = 0xe0000000u | (info & 0x7ff);
  writeDmi(kAbstractAuto, 0);
  writeDmi(kData0, data0_address); writeDmi(kCommand, 0x0023100a);
  if (!waitAbstract()) return false;
  writeDmi(kData0, data0_address + 4); writeDmi(kCommand, 0x0023100b);
  if (!waitAbstract()) return false;
  writeDmi(kData0, kFlashStatr); writeDmi(kCommand, 0x0023100c);
  if (!waitAbstract()) return false;
  writeDmi(kData0, 0x00050000); writeDmi(kCommand, 0x0023100d);
  if (!waitAbstract()) return false;
  writeDmi(kProgBuf0 + 0, 0x41844100);
  writeDmi(kProgBuf0 + 1, 0x0491c080);
  writeDmi(kProgBuf0 + 2, 0x0001c184);
  writeDmi(kProgBuf0 + 3, 0x4200c254);
  writeDmi(kProgBuf0 + 4, 0xfc758805);
  writeDmi(kProgBuf0 + 5, 0x90029002);
  return true;
}

BackendResult X035RvswdTargetControl::getStatus(TargetStatus& status) {
  if (!attachAndHalt()) { releaseBus(); return BackendResult::Unavailable; }
  uint32_t dmstatus = 0;
  const bool ok = readDmi(kDmStatus, dmstatus);
  status.flags = ok ? 0x03 : 0;
  status.start_mode = 0; status.boot_status = uint8_t(dmstatus >> 8);
  releaseBus();
  return ok ? BackendResult::Success : BackendResult::Failed;
}

BackendResult X035RvswdTargetControl::normalizeUser() {
  if (!attachAndHalt()) { releaseBus(); return BackendResult::Unavailable; }
  writeDmi(kDmControl, 0x40000001);
  writeDmi(kDmControl, 1);
  writeDmi(kDmControl, 0);
  releaseBus();
  return BackendResult::Success;
}

BackendResult X035RvswdTargetControl::enterProductBootloader() {
  return BackendResult::Unavailable;
}

BackendResult X035RvswdTargetControl::readMemory(uint32_t address,
                                                uint8_t* output,
                                                size_t length) {
  if (!attachAndHalt()) { releaseBus(); return BackendResult::Unavailable; }
  for (size_t i = 0; i < length; i += 4) {
    uint32_t value;
    if (!readWord(address + i, value)) { releaseBus(); return BackendResult::Failed; }
    put32(output + i, value);
  }
  releaseBus();
  return BackendResult::Success;
}

BackendResult X035RvswdTargetControl::programPage64(uint32_t address,
                                                   const uint8_t* data,
                                                   uint8_t& diagnostic) {
  diagnostic = 1;
  if ((address & 63) || (address & 0xff000000u) != 0x08000000u)
    return BackendResult::Failed;
  if (!attachAndHalt()) { releaseBus(); return BackendResult::Unavailable; }
  // OEP exposes a 64-byte logical unit, while X035 erases a 256-byte physical
  // page. Preserve the other three logical units inside that page.
  const uint32_t physical_page = address & ~uint32_t(255);
  uint8_t image[256];
  for (size_t i = 0; i < sizeof(image); i += 4) {
    uint32_t value;
    if (!readWord(physical_page + i, value)) goto failed;
    put32(image + i, value);
  }
  memcpy(image + (address - physical_page), data, 64);
  uint32_t ctlr;
  if (!readWord(kFlashCtlr, ctlr)) goto failed;
  if (ctlr & 0x8080) {
    if (!writeWord(kFlashKeyr, 0x45670123) ||
        !writeWord(kFlashKeyr, 0xcdef89ab) ||
        !writeWord(kFlashObkeyr, 0x45670123) ||
        !writeWord(kFlashObkeyr, 0xcdef89ab) ||
        !writeWord(kFlashModekeyr, 0x45670123) ||
        !writeWord(kFlashModekeyr, 0xcdef89ab)) goto failed;
  }
  diagnostic = 2;
  if (!waitFlash() || !writeWord(kFlashCtlr, 0x00020000) ||
      !writeWord(kFlashAddr, physical_page) ||
      !writeWord(kFlashCtlr, 0x00020040) || !waitFlash() ||
      !writeWord(kFlashCtlr, 0)) goto failed;
  diagnostic = 3;
  for (size_t group = 0; group < sizeof(image); group += 64) {
    if (!writeWord(kFlashCtlr, 0x00010000) ||
        !writeWord(kFlashCtlr, 0x00090000) || !waitFlash() ||
        !prepareFlashWriter()) goto failed;
    for (size_t i = 0; i < 64; i += 4) {
      writeDmi(kData1, physical_page + group + i);
      writeDmi(kData0, get32(image + group + i));
      writeDmi(kCommand, 0x00240000);
      if (!waitAbstract()) goto failed;
    }
    diagnostic = 4;
    if (!writeWord(kFlashAddr, physical_page + group) ||
        !writeWord(kFlashCtlr, 0x00010040) || !waitFlash() ||
        !writeWord(kFlashCtlr, 0)) goto failed;
  }
  for (size_t i = 0; i < sizeof(image); i += 4) {
    uint32_t value;
    if (!readWord(physical_page + i, value) || value != get32(image + i)) {
      diagnostic = 5; goto failed;
    }
  }
  releaseBus();
  diagnostic = 0;
  return BackendResult::Success;
failed:
  releaseBus();
  return BackendResult::Failed;
}

}  // namespace oep::prototype

#else
namespace oep::prototype {
void X035RvswdTargetControl::begin() {}
BackendResult X035RvswdTargetControl::getStatus(TargetStatus&) { return BackendResult::Unavailable; }
BackendResult X035RvswdTargetControl::normalizeUser() { return BackendResult::Unavailable; }
BackendResult X035RvswdTargetControl::enterProductBootloader() { return BackendResult::Unavailable; }
BackendResult X035RvswdTargetControl::readMemory(uint32_t, uint8_t*, size_t) { return BackendResult::Unavailable; }
BackendResult X035RvswdTargetControl::programPage64(uint32_t, const uint8_t*, uint8_t&) { return BackendResult::Unavailable; }
}  // namespace oep::prototype
#endif
