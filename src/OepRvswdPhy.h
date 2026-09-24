// RVSWD physical layer: push-pull SWDIO with explicit turnaround, per-session half-period margin
// check, bounded retry on DMI parity failure. Backends: ESP32 dedicated GPIO (E151/E153/E156/E157)
// and RP2040 / RP2350 SIO (OepRp2BitBang.h). Other architectures get a stub that reports
// Unavailable, so the same services compile everywhere.
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
  void park() override;
  bool attached() const override { return attached_; }
  bool read(uint8_t address, uint32_t &value) override;   // with bounded retry
  // One cheap look for a debug module at this half period: drive the bus, set dmactive and
  // read DMSTATUS once. For sweeping candidate pin pairs, where attach()'s margin check
  // (six half periods x 1000 reads) is far too slow to be a search step.
  // keep_driven leaves the bus driven so readOnce() can continue where this left off.
  bool probeOnce(uint32_t half_ns, uint32_t &dmstatus, bool keep_driven = false);
  // One read, no retry: for measuring how often the link is clean.
  bool readOnce(uint8_t address, uint32_t &value) { return readRaw(address, value); }
  // Re-run the bus bring-up without touching any debug-module register. This part drops the
  // DMI link when its state changes, so a caller that has just written DMCONTROL may need it.
  void wakeBus() { configureBus(true); }
  void reinit() override { configureBus(false); }
  void useHalf(uint32_t half_ns) { setHalf(half_ns); }
  void useSafeSpeed() override;
  bool retune() override;
  // Refuse to attach faster than this. attach() measures the link, but a marginal one
  // (a bench jig) can pass both the read and the write check at a
  // period whose longer abstract-command sequences still break, and the period it lands
  // on then varies run to run. A jig that is known to be provisional says so here rather
  // than leaving the probe to guess: 0 = no floor.
  void setMinHalfNs(uint32_t half_ns) { min_half_ns_ = half_ns; }
  // Rest the bus with SWCLK low between transactions instead of both lines high. Which one
  // a target needs is a property of the target, and they disagree (2026-09-23): a CH32L103
  // resting high drops the link after about 1 ms and its debug module resets, letting a
  // halted hart run again, while resting low keeps both for seconds; a CH32X035 is the
  // other way round - it keeps the link across any idle resting high, and resting low
  // between frames stopped it attaching at all. Telling the two apart at run time means
  // provoking the reset on the kind that has it, so a probe built for one says so instead.
  void setIdleClockLow(bool low) { park_low_ = low; }
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
  uint32_t min_half_ns_ = 0;
  uint32_t half_cycles_ = 0, half_ns_ = 0, retries_ = 0, transactions_ = 0, dmi_ns_ = 0;
  // Longest quiet spell the target's debug interface tolerates before the link has to be
  // brought up again. It measured out at about 1 ms; this leaves margin.
  static constexpr uint32_t kIdleUs = 300;
  uint32_t last_activity_us_ = 0;
  bool park_low_ = false;     // idle with SWCLK low instead of both lines high (setIdleClockLow)
  void setHalf(uint32_t half_ns);
  bool readsStable(uint32_t &first);   // 1000 identical DMSTATUS reads at the current period
  bool writesLand();                   // a few hundred program-buffer write/read round trips
  void configureBus(bool with_wake);
  void writeRaw(uint8_t address, uint32_t data);
  void reviveIfIdle();
  bool readRaw(uint8_t address, uint32_t &value);
};

}  // namespace oep
