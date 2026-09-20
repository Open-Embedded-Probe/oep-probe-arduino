#pragma once

#include "OepPrototype.h"

namespace oep::prototype {

// Destructive ESP32/CH32V003 experiment backend. The wire protocol and this
// API are not stable. The target reset pin is deliberately not used.
class V003SwioTargetControl : public TargetControlBackend,
                              public TargetMemoryBackend,
                              public TargetFlashBackend {
 public:
  V003SwioTargetControl() = default;

  void begin();
  BackendResult getStatus(TargetStatus& status) override;
  BackendResult normalizeUser() override;
  BackendResult enterProductBootloader() override;
  BackendResult readMemory(uint32_t address, uint8_t* output,
                           size_t length) override;
  BackendResult programPage64(uint32_t address, const uint8_t* data,
                              uint8_t& diagnostic) override;

 private:
  bool runPayload(const uint32_t* words, size_t count);
};

}  // namespace oep::prototype
