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

// A mount is tried kMountTries times before the area is formatted: a read that failed once (bench, an RP2040 probe
// 152a3f44e811780e whose flash reads failed now and then, 2026-11-18) lost the saved settings and the memo area to a
// format on the next boot. The core's own format-on-failure is off for that.
constexpr int kMountTries = 3;
#if defined(ARDUINO_ARCH_ESP32)
constexpr const char *kLabel = "spiffs", *kBase = "/oepfs";
bool mountOrFormat() {
  for (int i = 0; i < kMountTries; ++i) {
    if (LittleFS.begin(false, kBase, 10, kLabel)) return true;
    delay(10);
  }
  return LittleFS.begin(true, kBase, 10, kLabel);   // it does not mount: formatted
}
#else
extern "C" uint8_t _FS_start, _FS_end;

bool mountOrFormat() {
  if (&_FS_end == &_FS_start) return false;   // a build without an FS area
  LittleFS.setConfig(LittleFSConfig(false));  // no format on the first failure (arduino-pico's autoFormat)
  for (int i = 0; i < kMountTries; ++i) {
    if (LittleFS.begin()) return true;
    delay(10);
  }
  return LittleFS.format() && LittleFS.begin();   // it does not mount: formatted
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
