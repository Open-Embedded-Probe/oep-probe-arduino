// RVSWD physical layer on ESP32 dedicated GPIO (E151/E153/E156/E157):
// push-pull SWDIO with explicit turnaround, per-session half-period margin
// check, bounded retry on DMI parity failure. Other architectures get a stub
// that reports Unavailable, so the same services compile everywhere.
#pragma once

#include <Arduino.h>

#include "OepDmiPhy.h"

namespace oep {

class RvswdPhy final : public DmiPhy {
 public:
  bool begin(int swdio, int swclk);
  // Drive the bus, write dmactive, and pick the smallest half period whose
  // DMSTATUS reads are all parity-clean and consistent. false = no target.
  bool attach() override;
  void release() override;
  bool attached() const override { return attached_; }
  bool read(uint8_t address, uint32_t &value) override;   // with bounded retry
  void write(uint8_t address, uint32_t value) override;
  uint32_t halfNs() const { return half_ns_; }
  // Measured during attach: wall time of one DMI read at the selected half period,
  // and the SWCLK rate it implies (53 clocked bits per transaction). 0 = not attached yet.
  uint32_t dmiNs() const override { return dmi_ns_; }
  uint32_t clockHz() const override { return dmi_ns_ ? uint32_t(53000000000ull / dmi_ns_) : 0; }
  uint32_t retries() const override { return retries_; }
  uint32_t transactions() const override { return transactions_; }

 private:
  int swdio_ = -1, swclk_ = -1;
  bool ready_ = false, attached_ = false;
  uint32_t half_cycles_ = 0, half_ns_ = 0, retries_ = 0, transactions_ = 0, dmi_ns_ = 0;
  void setHalf(uint32_t half_ns);
  void configureBus();
  bool readRaw(uint8_t address, uint32_t &value);
};

}  // namespace oep
