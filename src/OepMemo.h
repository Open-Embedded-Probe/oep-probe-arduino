// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.probe.memo (oep-if-memo): named objects the host keeps on the probe, as files on the probe's LittleFS
// (OepFlashFs.h). An object is written into a staging file kept open across the writes, closed and checked at the last
// one, then renamed over the object's file: LittleFS commits a rename whole, so a power loss leaves the old object or
// the new one (oep-if-memo §3 allows less). The old object stays readable until then. A file: name_len(u8) name crc(u32)
// data, at "/m/<the name's CRC-32 in hex>.<n>" (n: the next free slot when two names share a CRC).
#pragma once

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)

#include <Arduino.h>
#include <FS.h>

#include "Oep.h"

namespace oep {

class Memo final : public Interface {
 public:
  static constexpr uint32_t kMaxObject = 2048;
  static constexpr uint32_t kMostObjects = 64;
  // Mounts the FS and sizes the area; false (the interface not added) when the FS is too small for it.
  bool begin();
  uint32_t maxObjects() const { return max_objects_; }
  const char *name() const override { return reg::probe_memo::kName; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return reg::probe_memo::kRevision; }
  bool lockFree(uint8_t op) const override { return reg::probe_memo::isLockFree(op); }
  bool offers(uint8_t op) const override;
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  void sessionOver() override { dropStaging(); }

 private:
  static constexpr size_t kMaxName = 64;
  Result list(const uint8_t *p, size_t len, uint8_t *out, size_t capacity);
  Result read(const uint8_t *p, size_t len, uint8_t *out, size_t capacity);
  Result write(const uint8_t *p, size_t len, uint8_t *out, size_t capacity);
  Result remove(const uint8_t *p, size_t len, uint8_t *out, size_t capacity);
  void dropStaging();
  uint32_t max_objects_ = 0;
  bool staging_ = false;
  File stage_;
  char stage_name_[kMaxName + 1] = {};
  size_t stage_name_len_ = 0;
  uint32_t stage_size_ = 0, stage_have_ = 0, stage_crc_ = 0, stage_want_ = 0;
};

}  // namespace oep

#endif
