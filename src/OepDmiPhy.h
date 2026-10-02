// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The physical layer under Ch32Dm: one DMI register read/write plus attach and
// release. Two implementations so far: RvswdPhy (two-wire, X035 and other
// QingKe V4 parts) and SwioPhy (single-wire, CH32V003). The DM sequences above
// this line do not know which one they run on.
#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "OepRegistry.h"
#include "OepWireLoss.h"

namespace oep {

// DMSTATUS.version (bits 3:0) of a debug module the probe works with, as oep-if-debug §1 counts a module "found": 2 or
// more and not 15 (2 = debug spec 0.13, 3 = 1.0, later versions alike; the riscv-dm ops treat them all the same, §4).
// Anything else (0 none, 1 0.11, 15 non-conforming, a floating bus reading all ones) is not one.
inline bool dmVersionKnown(uint32_t dmstatus) {
  const uint32_t version = dmstatus & 0xf;
  return version >= 2 && version != 15;
}

// DMSTATUS.allhalted (bit 9) of a module: a line reading all ones has it set too, with no module behind it.
inline bool dmHalted(uint32_t dmstatus) { return dmVersionKnown(dmstatus) && (dmstatus & (1u << 9)); }

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
  // Nothing holds the pins any more (the connection closed, a scan's try, a failed attach): both to Hi-Z with no pull,
  // the free state of oep-core §8 - not the resting a live link uses (SWCLK low, SWDIO's pull-up). attach() sets them
  // up again. Backends whose release() already leaves no pull leave this alone.
  virtual void free() { release(); }
  virtual bool attached() const = 0;
  // One DMI read with the PHY's own bounded retry (readWire). Its outcome runs the connection's wire-loss clock
  // (oep-if-debug §2: lost after wire_lost_ms of exchanges with no answer from the wire, with no successful one between).
  // A good exchange is a read that came back with a value other than all zeros or all ones: a line held low, or one
  // that rises through its pull-up with no module behind it, reads one of those (a read cell, a parity bit included,
  // comes back as the line rests). Such a value is no answer on DMSTATUS - the probe answers status line for it
  // (TargetRiscvDm::failure, checkConnection) - so the clock runs on as for a read that got nothing; on any other
  // register it may be the register's value, so it leaves the clock as it is. A read that got nothing back starts it.
  bool read(uint8_t address, uint32_t &value) {
    const bool ok = readWire(address, value);
    const Outcome o = outcomeOf(address, ok, value);
    if (o == kNoAnswer) loss_.silent();
    else if (o == kAnswered) loss_.answered();
    return ok;
  }
  static constexpr uint8_t kDmStatusAddress = 0x11;
  // One read as an exchange of oep-if-debug §2: answered (a good exchange), no answer (nothing came back, or a DMSTATUS
  // of all zeros / all ones), or neither (another register's all zeros / all ones: it may be the register's value).
  enum Outcome : uint8_t { kNoAnswer, kAnswered, kNeither };
  static Outcome outcomeOf(uint8_t address, bool ok, uint32_t value) {
    if (!ok) return kNoAnswer;
    if (value != 0 && value != 0xffffffffu) return kAnswered;
    return address == kDmStatusAddress ? kNoAnswer : kNeither;
  }
  WireLossClock &loss() { return loss_; }
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

  // A scan's look at a combination (oep-if-debug §1, what scan writes): the wire's wake / configuration sequence and
  // dmactive (only when DMCONTROL does not read it set), then DMSTATUS read - no write check, no scratch register, the
  // link left not attached. false: nothing answered. The default (a backend with no separate bring-up) attaches.
  virtual bool bringUp(uint32_t &dmstatus) { return attach() && read(0x11, dmstatus); }

  // search_retries (oep-if-debug §1, attach answer TLV 0x12): the tries of the speed search that failed in the attaches
  // since clearSearchRetries() - a speed whose reads or writes did not check out, a pass that started again, and each
  // read or round trip retried inside a check at the slowest speed.
  uint32_t searchRetries() const { return search_retries_; }
  void clearSearchRetries() { search_retries_ = 0; }
  void countSearchRetry() { ++search_retries_; }   // a try above the PHY (a whole attach() again) failed
  // The attach budget (oep-if-debug §1, limits.attach_budget_ms): attach() starts no further step of its search once
  // millis() reaches `at_ms` and answers false. The caller sets it before and clears it after.
  // end_ms: the budget's own end - what follows the search (a halt's rounds, an abstract command's wait) stops there.
  void setDeadline(uint32_t at_ms, uint32_t end_ms) { deadline_ms_ = at_ms; end_ms_ = end_ms; has_deadline_ = true; }
  void setDeadline(uint32_t at_ms) { setDeadline(at_ms, at_ms); }
  void clearDeadline() { has_deadline_ = false; }
  bool pastDeadline() const { return has_deadline_ && static_cast<int32_t>(millis() - deadline_ms_) >= 0; }
  bool pastBudget() const { return has_deadline_ && static_cast<int32_t>(millis() - end_ms_) >= 0; }
  // Retries inside one request (oep-if-debug §2, limits.wire_retry_ms): a request starts with the whole allowance, and
  // read() retries no more once it is spent (the request then ends with status line).
  void beginRequest() { retry_us_ = 0; }

 protected:
  virtual bool readWire(uint8_t address, uint32_t &value) = 0;
  bool retryLeft() const { return retry_us_ < v1::reg::kLimitWireRetryMs * 1000u; }
  // One more retry that takes about cost_us still ends inside the allowance (so a request never spends more than
  // wire_retry_ms retrying, even when one retry is long - a wake at a slow max_speed takes tens of ms).
  bool retryFits(uint32_t cost_us) const { return retry_us_ + cost_us <= v1::reg::kLimitWireRetryMs * 1000u; }
  void spentRetrying(uint32_t us) { retry_us_ += us; }
  uint32_t search_retries_ = 0;

 private:
  uint32_t deadline_ms_ = 0, end_ms_ = 0;
  bool has_deadline_ = false;
  uint32_t retry_us_ = 0;
  WireLossClock loss_;
};

// The attach budget (oep-if-debug §1, limits.attach_budget_ms) over one attach: the PHY's deadline from construction to
// destruction. One attach answer takes at most the budget of the probe's time, a reset's hold_ms (extra_ms) aside; the
// PHY's search gets it less kTailMs - what follows the search (havereset, a halt, the target_id and dpc reads) and
// the request's wire retries (wire_retry_ms); a search step running when the deadline passes ends there, and the steps
// after the search end at the budget's end (pastBudget).
class AttachDeadline {
 public:
  static constexpr uint32_t kTailMs = v1::reg::kLimitWireRetryMs + 100;
  AttachDeadline(DmiPhy &phy, uint32_t extra_ms = 0) : phy_(phy) {
    const uint32_t end = millis() + v1::reg::kLimitAttachBudgetMs + extra_ms;
    phy_.setDeadline(end - kTailMs, end);
  }
  ~AttachDeadline() { phy_.clearDeadline(); }
  AttachDeadline(const AttachDeadline &) = delete;
  AttachDeadline &operator=(const AttachDeadline &) = delete;

 private:
  DmiPhy &phy_;
};

}  // namespace oep
