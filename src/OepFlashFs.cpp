// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepFlashFs.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)

#include <Arduino.h>
#include <LittleFS.h>

namespace oep {

bool FlashFs::tried_ = false, FlashFs::mounted_ = false;
uint32_t FlashFs::mount_ms_ = 0;

namespace {

#if defined(ARDUINO_ARCH_ESP32)
constexpr const char *kLabel = "spiffs", *kBase = "/oepfs";
bool mountOrFormat() { return LittleFS.begin(true, kBase, 10, kLabel); }
#else
extern "C" uint8_t _FS_start, _FS_end;

bool mountOrFormat() {
  if (&_FS_end == &_FS_start) return false;   // a build without an FS area
  return LittleFS.begin();                    // arduino-pico formats an area that does not mount (autoFormat)
}
#endif

}  // namespace

bool FlashFs::mount() {
  if (tried_) return mounted_;
  tried_ = true;
  const uint32_t t = millis();
  mounted_ = mountOrFormat();
  mount_ms_ = millis() - t;
  return mounted_;
}

size_t FlashFs::blockSize() { return 4096; }

size_t FlashFs::blocks() {
  if (!mounted_) return 0;
#if defined(ARDUINO_ARCH_ESP32)
  return LittleFS.totalBytes() / blockSize();
#else
  FSInfo i;
  return LittleFS.info(i) ? i.totalBytes / blockSize() : 0;
#endif
}

}  // namespace oep

#endif
