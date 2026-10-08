// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe
#pragma once
#include <stdint.h>

// At the ESP-IDF custom app descriptor offset (after esp_app_desc_t).
// Revision numbers use major * 100 + minor, as esp_chip_info does.
struct P4FirmwareIdentity {
  static constexpr uint32_t kMagic = 0x4650454f;   // "OEPF", little endian
  static constexpr uint16_t kVersion = 1, kP4 = 1, kP4X = 2;
  uint32_t magic;
  uint16_t version, variant, minRevision, maxRevision;
  uint32_t reserved;

  bool compatible(uint16_t runningVariant, uint16_t chipRevision) const {
    return magic == kMagic && version == kVersion && reserved == 0 &&
           (variant == kP4 || variant == kP4X) && variant == runningVariant &&
           minRevision <= chipRevision && chipRevision <= maxRevision;
  }
};
static_assert(sizeof(P4FirmwareIdentity) == 16, "fixed firmware identity layout");
