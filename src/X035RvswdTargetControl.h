#pragma once

#include "OepPrototype.h"

namespace oep::prototype {

// Destructive ESP32-P4/CH32X035 prototype backend. The default fixture uses
// GPIO2 as SWDIO and GPIO54 as SWCLK. No compatibility is promised yet.
class X035RvswdTargetControl : public TargetControlBackend,
                               public TargetMemoryBackend,
                               public TargetFlashBackend {
 public:
  explicit X035RvswdTargetControl(uint8_t swdio = 2, uint8_t swclk = 54,
                                  unsigned half_period_us = 1)
      : swdio_(swdio), swclk_(swclk), half_period_us_(half_period_us) {}

  void begin();
  BackendResult getStatus(TargetStatus& status) override;
  BackendResult normalizeUser() override;
  BackendResult enterProductBootloader() override;
  BackendResult readMemory(uint32_t address, uint8_t* output,
                           size_t length) override;
  BackendResult programPage64(uint32_t address, const uint8_t* data,
                              uint8_t& diagnostic) override;

 private:
  uint8_t swdio_;
  uint8_t swclk_;
  unsigned half_period_us_;
  uint32_t recovery_page_ = 0;
  bool recovery_valid_ = false;
  uint8_t recovery_image_[256]{};

  void initializeBus();
  void releaseBus();
  bool readDmi(uint8_t address, uint32_t& value);
  void writeDmi(uint8_t address, uint32_t value);
  bool waitAbstract();
  bool attachAndHalt();
  bool readWord(uint32_t address, uint32_t& value);
  bool writeWord(uint32_t address, uint32_t value);
  bool waitFlash();
  bool prepareFlashWriter();
};

}  // namespace oep::prototype
