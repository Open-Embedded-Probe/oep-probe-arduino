// RISC-V debug module operations for WCH CH32 over a DmiPhy: attach, halt / resume with the CH32 quirks, the reset
// sequences, abstract register access, the autoexec block reader / writer (E156), running host code until its ebreak,
// single step, attach under reset. Nothing chip-specific beyond the debug module: flashing is the host's job
// (oep-client-python ch32_flash) through these parts.
#pragma once

#include <Arduino.h>

#include "OepDmiPhy.h"

namespace oep {

class Ch32Dm {
 public:
  explicit Ch32Dm(DmiPhy &phy) : phy_(phy) {}
  bool attached() const { return phy_.attached(); }
  bool halted() const { return halted_; }
  bool attach();
  bool halt();            // attach + haltreq, waits for allhalted
  bool resume();          // resumereq, waits for allresumeack
  // Restart the target from its reset vector and let it run (reset-halt, then resume; the link stays attached).
  // flags: bit0 released and running, bit1 execution confirmed by a nonzero pc sample (confirm = true: a brief
  // halt, dpc read, resume), bit2 the sequence was redone, bit3 a confirmation halt / resume failed.
  struct ResetReport { uint8_t flags; uint8_t attempts; uint32_t pc; };
  ResetReport reset(bool confirm = true);
  void detach();          // dmactive = 0, lines Hi-Z (target keeps running or stays halted)
  // Memory (hart must be halted). readWords uses the autoexec reader with no
  // per-word poll; cmderr is checked once at the end.
  bool readWords(uint32_t address, uint32_t *out, size_t words, uint8_t *cmderr = nullptr);
  bool readWordScalar(uint32_t address, uint32_t &value);
  bool readRegister(uint16_t regno, uint32_t &value);
  bool writeRegister(uint16_t regno, uint32_t value);
  bool readDmi(uint8_t address, uint32_t &value) { return attach() && phy_.read(address, value); }
  // Raw DMI write for host-built step lists (v1 oep.target.riscv-dm). It bypasses this helper's view of the
  // hart: after raw dmcontrol writes, use halt()/resume() to bring halted() back in line.
  bool writeDmi(uint8_t address, uint32_t value) {
    if (!attach()) return false;
    host_raw_ = true;             // the host may be running abstract commands: DATA0 / DATA1 are its operands now
    phy_.write(address, value);   // a DMI write reports nothing; read back through the list to check
    return true;
  }
  // True from the host's first raw DMI write until the probe itself resumes, resets or detaches: the console must
  // not touch DATA0 / DATA1 then, whatever halted() says (the host may have halted the hart through the list).
  bool hostRaw() const { return host_raw_; }  // abstract access register (CSR 0x000-0xfff, GPR 0x1000+)
  bool writeWord(uint32_t address, uint32_t value);
  // Generic parts for host-driven flashing (experiment F3/F4, 2026-09-24). writeWordsFast
  // stores consecutive words through an autoexec program buffer (the write-side twin of
  // readWords); runUntilHalt sets registers and dpc, resumes, and waits for the hart to stop
  // on its own ebreak (ebreakm/s/u are set first), forcing a halt at the timeout.
  bool writeWordsFast(uint32_t address, const uint32_t *words, size_t count);
  struct RunReport { bool stopped; uint32_t dpc; uint32_t a0; uint32_t elapsed_us; };
  bool runUntilHalt(uint32_t pc, const uint16_t *regnos, const uint32_t *values, size_t count,
                    uint32_t timeout_us, RunReport &report);
  // Parts from the ch32rv review of the v1 draft (2026-09-24):
  // resetHalt: system reset with haltreq held through it, so the hart stops before its first instruction
  // (semihosting, gdb "monitor reset halt", flashing over a running watchdog). dpc = where it stopped.
  bool resetHalt(uint32_t &dpc);
  // step: one instruction with dcsr.step, resumed exactly once (a re-issued resume would step twice), the
  // privilege level left as it is; moved = dpc changed (the CH32L103 raises no allresumeack to go by).
  bool step(uint32_t &dpc_before, uint32_t &dpc_after, bool &moved);
  // Acknowledge a pending havereset: until then a V00x DM keeps DMSTATUS's halt / run bits frozen at their
  // reset values. true = one was pending.
  bool ackHaveReset();
  // Attach while the target is held in reset, then release it and stop the hart at once: the way back from
  // firmware that turns the debug pins into GPIOs or sleeps straight away. `release` lets go of the reset line;
  // the halt requests start before it and run through it.
  bool attachUnderReset(void (*hold)(void *), void (*release)(void *), void *ctx, uint32_t hold_ms, uint32_t &dpc);
 private:
  DmiPhy &phy_;
  bool halted_ = false;
  bool host_raw_ = false;
  uint8_t cmderr_ = 0;
  bool waitAbstract();
  void relink();                          // PHY re-sync + abstract-command block back to a known state
  void retune();                          // PHY speed search + the same
  void settleHalted(bool ack_reset);      // after the hart stopped: ack a pending reset, relink, halted_
  bool loadRegisters(uint32_t &data0_address);
};

}  // namespace oep
