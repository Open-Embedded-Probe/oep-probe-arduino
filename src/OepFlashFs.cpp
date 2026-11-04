// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepFlashFs.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)

#include <Arduino.h>
#include <LittleFS.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_partition.h>
#endif

namespace oep {

bool FlashFs::tried_ = false, FlashFs::mounted_ = false, FlashFs::lost_ = false;
uint32_t FlashFs::mount_ms_ = 0;

namespace {

#if defined(ARDUINO_ARCH_ESP32)
constexpr const char *kLabel = "spiffs", *kBase = "/oepfs";

bool erased() {   // the partition reads all 0xFF: never used
  const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, kLabel);
  if (!p) return true;
  uint32_t chunk[64];
  for (size_t at = 0; at < p->size; at += sizeof chunk) {
    if (esp_partition_read(p, at, chunk, sizeof chunk) != ESP_OK) return false;
    for (uint32_t w : chunk)
      if (w != 0xFFFFFFFFu) return false;
  }
  return true;
}
bool tryMount() { return LittleFS.begin(false, kBase, 10, kLabel); }
bool formatAndMount() { return LittleFS.begin(true, kBase, 10, kLabel); }
#else
extern "C" uint8_t _FS_start, _FS_end;

bool erased() {   // the area (memory-mapped) reads all 0xFF: never used
  for (const uint8_t *p = &_FS_start; p < &_FS_end; ++p)
    if (*p != 0xFF) return false;
  return true;
}
bool tryMount() {
  if (&_FS_end == &_FS_start) return false;   // a build without an FS area
  LittleFS.setConfig(LittleFSConfig(false));  // no format behind the probe's back
  return LittleFS.begin();
}
bool formatAndMount() { return LittleFS.format() && LittleFS.begin(); }
#endif

}  // namespace

bool FlashFs::mount() {
  if (tried_) return mounted_;
  tried_ = true;
  const uint32_t t = millis();
  mounted_ = tryMount();
#if defined(ARDUINO_ARCH_RP2040)
  if (!mounted_ && &_FS_end == &_FS_start) return false;
#endif
  if (!mounted_) {
    lost_ = !erased();
    mounted_ = formatAndMount();
  }
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
