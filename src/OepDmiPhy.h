// The physical layer under Ch32Dm: one DMI register read/write plus attach and
// release. Two implementations so far: RvswdPhy (two-wire, X035 and other
// QingKe V4 parts) and SwioPhy (single-wire, CH32V003). The DM sequences above
// this line do not know which one they run on.
#pragma once

#include <stdint.h>

namespace oep {

class DmiPhy {
 public:
  virtual ~DmiPhy() = default;
  // Drive the bus and bring the debug module up (dmactive). false = no target answered.
  virtual bool attach() = 0;
  virtual void release() = 0;
  virtual bool attached() const = 0;
  virtual bool read(uint8_t address, uint32_t &value) = 0;   // with the PHY's own bounded retry
  virtual void write(uint8_t address, uint32_t value) = 0;
  // Measured at attach: wall time of one DMI read and the clock rate it implies (0 before attach).
  virtual uint32_t dmiNs() const = 0;
  virtual uint32_t clockHz() const = 0;
  virtual uint32_t retries() const = 0;
  virtual uint32_t transactions() const = 0;
};

}  // namespace oep
