// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Single-wire SWIO physical layer for QingKe V2 (CH32V003) on the classic
// ESP32, ported from the E123-E137 experiments: software bit timing with a
// cycle coefficient (8 = 262.5/862.5 ns, the closest to the WCH-LinkE's
// 240/860 ns), read recharge pulses, DMCFGR/SHDWCFGR unlock at attach. Any
// GPIO0-31 (begin, or usePins for host-chosen pins); the hot path keeps the
// pin's mask in a register. The ESP32-P4 has the same frames on the CPU's
// dedicated GPIO, timed in nanoseconds against the cycle counter (any
// GPIO0-54). Other architectures get a stub.
#pragma once

#include <Arduino.h>

#include "OepDmiPhy.h"

namespace oep {

class SwioPhy final : public DmiPhy {
 public:
  bool begin(int swio);   // GPIO0-31 (ESP32-P4: 0-54)
  bool usePins(int swdio, int swclk) override;   // swclk -1: one wire
  // The wake / configuration and dmactive (only when DMCONTROL does not read it set), the configuration read back, then
  // the write check on PROGBUF0 (put back after) - oep-if-debug §1, §3. false: no target.
  bool attach() override;
  bool bringUp(uint32_t &dmstatus) override;   // a scan's look: no write check, released after
  void release() override;
  void free() override;   // released, without the pull-up the line has while a link waits
  bool attached() const override { return attached_; }
  bool read(uint8_t address, uint32_t &value) override;   // bounded retry on a lost read
  void write(uint8_t address, uint32_t value) override;
  uint32_t dmiNs() const override { return dmi_ns_; }
  // 1 start + 7 address + 1 direction + 32 data bits per transaction
  uint32_t clockHz() const override { return dmi_ns_ ? uint32_t(41000000000ull / dmi_ns_) : 0; }
  uint32_t retries() const override { return retries_; }
  // The bit timing is fixed (about 1.1 us a bit): a ceiling at or above it holds, a lower one cannot be kept.
  static constexpr uint32_t kNominalHz = 888888;
  bool setMaxHz(uint32_t hz) override { return keepsMaxHz(hz); }
  bool keepsMaxHz(uint32_t hz) const override { return hz == 0 || hz >= kNominalHz; }
  uint32_t minClockHz() const override { return kNominalHz; }
  uint32_t transactions() const override { return transactions_; }

 private:
  bool ready_ = false, attached_ = false;
  uint32_t retries_ = 0, transactions_ = 0, dmi_ns_ = 0;
  bool readRaw(uint8_t address, uint32_t &value);   // IRAM_ATTR on the definition: the attribute is ESP32-only
  bool readRetried(uint8_t address, uint32_t &value);   // up to 4 tries, within the request's wire_retry_ms
  bool lineUp();            // the pull-up look (2 ms), then the line driven high
  bool configureModule();   // the configuration pair twice, dmactive when not set, the configuration read back
  bool writesLand();        // PROGBUF0 round trips, its value put back
};

}  // namespace oep
