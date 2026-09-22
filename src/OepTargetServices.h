// target.control / target.memory / target.flash (owner 0, ids 0x10..0x12) over Ch32Dm.
// Failure is reported through outcomes and CRC/read-back, never through DMI parity alone.
#pragma once

#include "OepCh32Dm.h"
#include "OepService.h"

namespace oep {

uint32_t crc32Ieee(uint32_t crc, const uint8_t *data, size_t length);

class TargetControl final : public Service {
 public:
  TargetControl(Ch32Dm &dm, RvswdPhy &phy) : dm_(dm), phy_(phy) {}
  uint16_t owner() const override { return OEP_V0_DEF_TARGET_CONTROL_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_TARGET_CONTROL_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_TARGET_CONTROL_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t first, uint8_t *out, size_t capacity) override;
  void abandon() override;

 private:
  Ch32Dm &dm_;
  RvswdPhy &phy_;
};

class TargetMemory final : public Service {
 public:
  explicit TargetMemory(Ch32Dm &dm) : dm_(dm) {}
  uint16_t owner() const override { return OEP_V0_DEF_TARGET_MEMORY_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_TARGET_MEMORY_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_TARGET_MEMORY_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  Ch32Dm &dm_;
};

class TargetFlash final : public Service {
 public:
  explicit TargetFlash(Ch32Dm &dm) : dm_(dm) {}
  uint16_t owner() const override { return OEP_V0_DEF_TARGET_FLASH_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_TARGET_FLASH_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_TARGET_FLASH_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  Ch32Dm &dm_;
  uint32_t page_[64];  // read-back buffer for one 256-byte page
  bool inRange(uint32_t address, uint32_t bytes) const;
};

}  // namespace oep
