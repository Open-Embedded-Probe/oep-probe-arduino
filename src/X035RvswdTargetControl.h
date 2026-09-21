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
                                  unsigned half_period_us = 1,
                                  unsigned frame_settle_us = 20)
      : swdio_(swdio), swclk_(swclk), half_period_us_(half_period_us),
        frame_settle_us_(frame_settle_us) {}

  void begin();
  BackendResult getStatus(TargetStatus& status) override;
  BackendResult normalizeUser() override;
  BackendResult enterProductBootloader() override;
  BackendResult readMemory(uint32_t address, uint8_t* output,
                           size_t length) override;
  BackendResult programPage64(uint32_t address, const uint8_t* data,
                              uint8_t& diagnostic) override;
  BackendResult stagePage64(uint32_t address, const uint8_t* data,
                             uint8_t& diagnostic) override;
  BackendResult commitPage256(uint32_t address, uint8_t& diagnostic) override;

 private:
  uint8_t swdio_;
  uint8_t swclk_;
  unsigned half_period_us_;
  // Conservative post-frame guard.  A fixture may explicitly reduce this
  // only after full-image readback verification on its actual wiring.
  unsigned frame_settle_us_;
  // An OEP full-image operation is split into 88-byte read requests and
  // 64-byte program requests.  They form one host-owned transaction: keep
  // the target halted and the RVSWD bus configured between successful
  // requests, then invalidate this state on reset or any transport error.
  // This is deliberately private to the backend; the public OEP contract
  // still requires the client to finish with TargetControl normalize-user.
  bool attached_ = false;
  // Abstract-DM sequential read state.  The target program buffer increments
  // the address kept in DMDATA1 and writes every loaded word to DMDATA0;
  // reading DMDATA0 with ABSTRACTAUTO=1 launches the following word.
  bool sequential_read_valid_ = false;
  uint32_t sequential_read_next_ = 0;
  uint32_t recovery_page_ = 0;
  bool recovery_valid_ = false;
  uint8_t recovery_image_[256]{};
  uint32_t staged_page_ = 0;
  uint8_t staged_mask_ = 0;
  uint8_t staged_image_[256]{};
  bool staged_commit_active_ = false;

  void initializeBus();
  void releaseBus();
  bool readDmi(uint8_t address, uint32_t& value);
  void writeDmi(uint8_t address, uint32_t value);
  bool waitAbstract();
  bool attachAndHalt();
  bool readWord(uint32_t address, uint32_t& value);
  bool readSequentialWord(uint32_t address, uint32_t& value);
  bool prepareSequentialReader();
  bool writeWord(uint32_t address, uint32_t value);
  bool waitFlash();
  bool prepareFlashWriter();
};

}  // namespace oep::prototype
