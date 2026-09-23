// Single-wire SWIO physical layer for QingKe V2 (CH32V003) on the classic
// ESP32, ported from the E123-E137 experiments: software bit timing with a
// cycle coefficient (8 = 262.5/862.5 ns, the closest to the WCH-LinkE's
// 240/860 ns), read recharge pulses, DMCFGR/SHDWCFGR unlock at attach. The
// hot path constant-folds the pin, so the pin is fixed at compile time
// (GPIO16 on the UIAPduino jig). Other architectures get a stub.
#pragma once

#include <Arduino.h>

#include "OepDmiPhy.h"

namespace oep {

class SwioPhy final : public DmiPhy {
 public:
  static constexpr int kPin = 16;
  bool begin(int swio);   // must be kPin
  bool attach() override;
  void release() override;
  bool attached() const override { return attached_; }
  bool read(uint8_t address, uint32_t &value) override;   // bounded retry on a lost read
  void write(uint8_t address, uint32_t value) override;
  uint32_t dmiNs() const override { return dmi_ns_; }
  // 1 start + 7 address + 1 direction + 32 data bits per transaction
  uint32_t clockHz() const override { return dmi_ns_ ? uint32_t(41000000000ull / dmi_ns_) : 0; }
  uint32_t retries() const override { return retries_; }
  uint32_t transactions() const override { return transactions_; }

 private:
  bool ready_ = false, attached_ = false;
  uint32_t retries_ = 0, transactions_ = 0, dmi_ns_ = 0;
  bool readRaw(uint8_t address, uint32_t &value);   // IRAM_ATTR on the definition: the attribute is ESP32-only
};

}  // namespace oep
