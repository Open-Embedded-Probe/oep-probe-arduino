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
//
// Interrupts are off for one frame at a time (portENTER_CRITICAL on the calling core, loop()'s): a write's 41 slots,
// at most 41 x 1125 ns = 46 us; a read's 9 slots and 32 read bits, about 50 us, plus at most one wait for the line to
// come back high per frame (not per bit) - the P4 100 us, the classic 1000 polls of GPIO.in - before the frame gives up.
#pragma once

#include <Arduino.h>

#include "OepDmiPhy.h"
#include "OepWireGate.h"

#ifndef OEP_SWIO_PAUSE_CONSOLE
#define OEP_SWIO_PAUSE_CONSOLE 0
#endif

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
  // Ch32Dm's relink: the link brought back in step (resync) when the last read got nothing back - a target that reset
  // itself through a system reset dropped the SWIO configuration and the debug module's dmactive.
  void reinit() override;
  void write(uint8_t address, uint32_t value) override;
  uint32_t dmiNs() const override { return dmi_ns_; }
  // The bit timing is fixed: a zero's slot, 862.5 ns low + 262.5 ns high = 1125 ns (oep-if-debug §3.2: the wire's speed
  // is 1 / (a zero's low + its high)). The link runs at it and at nothing else: describe declares it as min_clock_hz and
  // max_clock_hz, a connection's speed_hz is it, a ceiling at or above it holds and a lower one cannot be kept. speed_hz
  // was the rate one read's wall time implies over its 41 slots (41 s / dmi_ns: the read slots' waits for the line and
  // the frame's set-up counted in) - 732142 or 745454 Hz on the ESP32-P4, under the 888888 declared, so a max_speed
  // of 800 kHz was refused while the connections said they ran slower than that.
  // The longest a frame keeps interrupts off on its core (loop()'s), the bound a sketch sizes what interrupts must
  // serve against (the classic's UART0 RX FIFO threshold, Firmware/OepProbe/Esp32.h): a read about 60 us (the bench
  // fitted 70 us a read with the 8 us gap and the call, 0.0.29-dev+526a881) plus one frame's wait for the line to rise
  // (the P4 100 us; the classic 1000 polls of GPIO.in, about 0.1 us each - an estimate, not measured).
  static constexpr uint32_t kIrqOffMaxUs = 200;
  static constexpr uint32_t kNominalHz = 888888;
  uint32_t clockHz() const override { return dmi_ns_ ? kNominalHz : 0; }   // once an attach has run
  uint32_t retries() const override { return retries_; }
  bool setMaxHz(uint32_t hz) override { return keepsMaxHz(hz); }
  bool keepsMaxHz(uint32_t hz) const override { return hz == 0 || hz >= kNominalHz; }
  uint32_t minClockHz() const override { return kNominalHz; }
  uint32_t maxClockHz() const override { return kNominalHz; }
  uint32_t transactions() const override { return transactions_; }
  // The classic ESP32: no frame while the core-0 sampler has a window open (OepWireGate.h) - a request's frames wait
  // it out (at most one window: up to 164 ms immediate, 250 ms a burst of a trigger search) and then hold the wire until
  // loop() comes round. The console's reading (backgroundTurn) does not wait: oep-if-console §3 lets the probe stop
  // reading only for a riscv-dm request of the connection or a halted hart, so it reads on through a window as before
  // (its frames there may be garbled; dmseq's CRC and the reads twice take most of that). Test hook, off unless a build
  // defines it: OEP_SWIO_PAUSE_CONSOLE=1 pauses the reading instead (the wire's turn refused while a window is open) -
  // the bench's check of the mechanism; a spec change would be needed before it is the behaviour.
  bool backgroundTurn() override;
  void backgroundDone() override;

 protected:
  bool readWire(uint8_t address, uint32_t &value) override;   // with bounded retry (DmiPhy::read)

 private:
  bool ready_ = false, attached_ = false;
  // oep-if-debug §2 / §3.2: from a read with no answer until one answers, the line rests released to its pull-up
  // between frames and is never driven high there (each frame still drives it high and low)
  bool rest_free_ = false;
  uint32_t retries_ = 0, transactions_ = 0, dmi_ns_ = 0;
  bool readRaw(uint8_t address, uint32_t &value);   // IRAM_ATTR on the definition: the attribute is ESP32-only
  bool readRetried(uint8_t address, uint32_t &value);   // up to 4 tries, within the request's wire_retry_ms
  void resync();            // the configuration pair twice, dmactive when it reads clear (no line or write check)
  bool lineUp();            // the pull-up look (2 ms), then the line driven high
  bool configureModule();   // the configuration pair twice, dmactive when not set, the configuration read back
  bool writesLand();        // PROGBUF0 round trips, its value put back
};

}  // namespace oep
