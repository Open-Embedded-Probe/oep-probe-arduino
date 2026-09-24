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
  // Stop talking but leave the wires in a state the target is happy to sit in. Releasing
  // a CH32's two wires lets them float high together, which is the bus's reset condition;
  // parking keeps the clock low instead. Backends without that distinction just release.
  virtual void park() { release(); }
  virtual bool attached() const = 0;
  virtual bool read(uint8_t address, uint32_t &value) = 0;   // with the PHY's own bounded retry
  virtual void write(uint8_t address, uint32_t value) = 0;
  // Re-run the bus bring-up without touching any debug-module register. A CH32 drops the
  // DMI link when its state changes, so a caller whose write did not take can try again
  // from a known bus state. Backends with nothing to do leave this alone.
  virtual void reinit() {}
  // The speed attach() picks holds only for the target's clock at that moment. A reset drops a CH32 back to its
  // default clock - slower than a sketch that raised it - and a link tuned to the sketch then garbles writes: the
  // haltreq held through a reset was lost that way, and the hart ran into its image (2026-09-24, CH32X035). So a
  // reset runs at the slowest period, and once the hart has stopped the link is tuned again without a wake (which
  // would reset the target). Backends whose speed does not depend on the target leave these alone.
  virtual void useSafeSpeed() {}
  virtual bool retune() { return true; }
  // Measured at attach: wall time of one DMI read and the clock rate it implies (0 before attach).
  virtual uint32_t dmiNs() const = 0;
  virtual uint32_t clockHz() const = 0;
  virtual uint32_t retries() const = 0;
  virtual uint32_t transactions() const = 0;
};

}  // namespace oep
