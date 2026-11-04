// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The probe's LittleFS, mounted once for whoever keeps files there: the memo area (OepMemo.h) and, on RP2040 / RP2350,
// the settings (OepSettings.cpp). ESP32: the data partition labelled "spiffs" (the default partition tables have one);
// RP2040 / RP2350: the core's FS area (the board's flash menu, sketch.yaml). Mounted without formatting first: an area
// that does not mount is formatted, and whether it held anything before (not all erased) is kept, so that the settings
// can say their save was there but could not be read (core §13.7, storage_state 2) rather than that there was none.
#pragma once

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)

#include <stddef.h>
#include <stdint.h>

namespace oep {

struct FlashFs {
  // Mounts on the first call (formatting an area that does not mount); true when the FS can be used.
  static bool mount();
  // The first mount found an area that did not mount and was not erased: what it held is gone.
  static bool lostOnMount() { return lost_; }
  static size_t blocks();          // the FS's blocks (0: not mounted)
  static size_t blockSize();
  static uint32_t mountMs() { return mount_ms_; }

 private:
  static bool tried_, mounted_, lost_;
  static uint32_t mount_ms_;
};

}  // namespace oep

#endif
