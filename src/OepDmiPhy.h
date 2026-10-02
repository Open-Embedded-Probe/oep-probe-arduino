// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The physical layer under Ch32Dm: one DMI register read/write plus attach and
// release. Two implementations so far: RvswdPhy (two-wire, X035 and other
// QingKe V4 parts) and SwioPhy (single-wire, CH32V003). The DM sequences above
// this line do not know which one they run on.
#pragma once

#include <stdint.h>

namespace oep {

// DMSTATUS.version (bits 3:0) of a debug module the probe works with: 2 (debug spec 0.13) or 3 (1.0), as oep-if-debug
// §1 counts a module "found". Anything else (0 none, 1 0.11, 15 non-conforming, a floating bus) is not one.
inline bool dmVersionKnown(uint32_t dmstatus) {
  const uint32_t version = dmstatus & 0xf;
  return version == 2 || version == 3;
}

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
  // A ceiling on the link speed from the next attach on (and the retunes after it): the host's max_speed, in Hz of the
  // bit clock. 0 = none. false: this backend cannot stay under it (a fixed speed above it).
  virtual bool setMaxHz(uint32_t hz) { return hz == 0; }
  virtual bool keepsMaxHz(uint32_t hz) const { return hz == 0; }   // setMaxHz(hz) would succeed (nothing changed)
  // The slowest this link goes (describe min_clock_hz, oep-if-debug §1): a max_speed under it is rejected unsupported.
  // 0 = no floor to declare.
  virtual uint32_t minClockHz() const { return 0; }
  // How the bus rests between transactions: SWCLK low instead of both lines high. The target's (oep-if-debug §3): the
  // host says so at attach (idle_clock) or through a slot. false: this backend cannot rest that way.
  virtual bool setIdleClockLow(bool low) { return !low; }
  // Move the link to another pin pair (host-chosen pins, oep-if-debug §1): only while not attached. The old pins are left
  // released (Hi-Z). false: this backend's pins are fixed, or the pair cannot be used.
  virtual bool usePins(int swdio, int swclk) { (void)swdio; (void)swclk; return false; }
  virtual bool canIdleClockLow() const { return false; }
  // Measured at attach: wall time of one DMI read and the clock rate it implies (0 before attach).
  virtual uint32_t dmiNs() const = 0;
  virtual uint32_t clockHz() const = 0;
  virtual uint32_t retries() const = 0;
  virtual uint32_t transactions() const = 0;
};

}  // namespace oep
