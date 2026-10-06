// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// RISC-V debug module operations for WCH CH32 over a DmiPhy: attach, halt / resume with the CH32 quirks, the reset
// sequences, abstract register access, the autoexec block reader / writer (E156), running host code until its ebreak,
// single step, attach under reset. Nothing chip-specific beyond the debug module: flashing is the host's job
// (oep-client-python ch32_flash) through these parts.
//
// The op boundary invariant (oep-if-debug §4): nothing the probe changed in the target is carried past an operation.
// A block op keeps s0, s1, a0, a1, DATA0 / DATA1 and abstractauto as it found them and puts them back before it
// returns; step puts dcsr.step and DATA back; run puts abstractauto and haltreq back (pc, the host's registers and dcsr
// stay as the host asked). halt may keep haltreq asserted while the hart is halted (the L103 drops its DMI link on a
// change of hart state); resume, step, reset, detach lower it. A raw DMI write is just that: the host owns DATA then.
//
// Held groups: a link that drops (a CH32L103's, at a change of hart state - not always at once) loses writes and reads
// all ones or the last value read until it is brought up again. So what an op keeps and gives back, and the register
// accesses it acts on, are done in groups each followed by a look at the link (linkHeld): a good look says nothing in
// the group met a drop; a bad one brings the link up again (steady) and redoes the group. What goes back is read back
// and compared too. A link may also miss a single access and be up again at the next (a glitch: a write lost - cmderr 6
// if the module took it for a bad parity - or a read answering the read before it), which the look after does not see:
// so every value an op keeps, gives back or answers is read twice and taken only when both reads agree (readSure,
// readRegisterSure), every write the op relies on is read back before it is relied on (the block program's set-up,
// abstractauto off, the run's registers, dcsr), and write_block reads DATA1 after every store.
#pragma once

#include <Arduino.h>

#include "OepDmiPhy.h"

namespace oep {

class Ch32Dm {
 public:
  explicit Ch32Dm(DmiPhy &phy) : phy_(phy) {}
  bool attached() const { return phy_.attached(); }
  DmiPhy &phy() { return phy_; }
  // This helper's view of the hart; checkHalted() reads DMSTATUS and brings it in line (the host may have halted or
  // resumed the hart through raw DMI writes). false from checkHalted: the link did not answer.
  bool halted() const { return halted_; }
  bool checkHalted();
  // DMSTATUS, read by someone else (the console's poll), says the hart runs: this view follows.
  void noteRunning() { halted_ = false; }
  uint8_t lastCmderr() const { return cmderr_; }
  // DMCONTROL's hartsel (and hasel) back to 0 when the host's dmi left another there (oep-if-debug §4): before every
  // high-level op, which then return with it 0.
  void selectHart0();   // cmderr of the last abstract command that failed (0 = none)
  bool attach();
  // A scan's look (oep-if-debug §1): the PHY's bring-up (wake / configuration, dmactive) and DMSTATUS, no write check;
  // the link is not left attached. false: no module answered.
  bool probe(uint32_t &dmstatus);
  bool halt();            // attach + haltreq, waits for allhalted; true at once when already halted
  bool resume();          // one resumereq; ok = the hart left debug mode (allresumeack, or running and not halted)
  // Restart the target from its reset vector and let it run (reset-halt, then resume; the link stays attached).
  // flags: bit0 released and running, bit1 execution confirmed by a nonzero pc sample (confirm = true: a brief
  // halt, dpc read, resume), bit2 the sequence was redone, bit3 a confirmation halt / resume failed. It answers once the
  // module answers again after the release (a target that restarts itself on the way: awaitModule), at most
  // kResetSettleMs after it started; still silent then, bit0 is cleared (the caller answers status line).
  static constexpr uint32_t kResetSettleMs = v1::reg::kLimitResetSettleMs;   // limits.reset_settle_ms (oep-if-debug §4.3)
  struct ResetReport { uint8_t flags; uint8_t attempts; uint32_t pc; };
  ResetReport reset(bool confirm = true);
  void detach();          // haltreq and the rest lowered, dmactive kept, lines Hi-Z (target keeps running or stays halted)
  // Memory (hart must be halted). readWords uses the autoexec reader with no per-word poll; cmderr is checked once at
  // the end. Both put s0, s1, a0, a1, DATA1, DATA0 and abstractauto back before returning (§4.5), read back and seen
  // back over a held link; false when that could not be done.
  bool readWords(uint32_t address, uint32_t *out, size_t words, uint8_t *cmderr = nullptr);
  // writeWordsFast stores each word exactly once - never redone after its store may have run - and says how many it
  // stored (written: in order, seen over a held link; false and fewer than count after a cmderr or a link that did not
  // hold).
  bool writeWordsFast(uint32_t address, const uint32_t *words, size_t count, size_t *written = nullptr);
  // Abstract register access (hart halted). They use DATA0: an op that calls them restores DATA (keepMailbox /
  // giveMailbox) around the whole of itself.
  bool readRegister(uint16_t regno, uint32_t &value);
  bool writeRegister(uint16_t regno, uint32_t value);
  bool readDmi(uint8_t address, uint32_t &value) { return attach() && phy_.read(address, value); }
  // Raw DMI write for host-built step lists (v1 oep.target.riscv-dm): nothing of the probe's is restored first - the
  // probe carries nothing across operations (§4). After raw dmcontrol writes checkHalted() brings halted() in line.
  bool writeDmi(uint8_t address, uint32_t value) {
    if (!attach()) return false;
    phy_.write(address, value);   // a DMI write reports nothing; read back through the list to check
    return true;
  }
  // DATA0 / DATA1 as the target left them (a dmseq frame or the answer to one): kept at the start of an op that uses
  // them and written back (DATA1, then DATA0, read back) before the op returns. Public for the ops composed above this
  // class (run). false: the link did not hold for it (not kept / not seen back).
  bool keepMailbox();
  bool giveMailbox();
  // The halted hart's dpc, read in a held group with the mailbox kept and given back (attach's answer, the resets).
  bool readDpc(uint32_t &dpc);
  // runUntilHalt: sets registers and dpc (dcsr: ebreakm, prv = M), resumes once, waits for the hart to stop on its
  // ebreak; at the timeout it halts the hart itself. Then dpc and the registers `outs` are read. It never re-issues the
  // resume (oep-if-debug §4.4). report.halted: the hart is halted at the end (false: it could not be stopped - stopped 2).
  struct RunReport { bool stopped; bool halted; uint32_t dpc; uint32_t elapsed_us; };
  bool runUntilHalt(uint32_t pc, const uint16_t *regnos, const uint32_t *values, size_t count, uint32_t timeout_ms,
                    const uint16_t *outs, uint32_t *out_values, size_t out_count, RunReport &report);
  // resetHalt: system reset with haltreq held through it, so the hart stops before its first instruction
  // (semihosting, gdb "monitor reset halt", flashing over a running watchdog). dpc = where it stopped.
  bool resetHalt(uint32_t &dpc);
  // step: one instruction with dcsr.step, resumed exactly once (a re-issued resume would step twice), the
  // privilege level left as it is; moved = dpc changed (the CH32L103 raises no allresumeack to go by). end (oep-if-debug
  // §4.2): back in debug mode by itself within dm_wait_ms; halted by the probe's haltreq after that (dpc_after valid);
  // or still running after dm_wait_ms more (haltreq cleared, dcsr.step maybe still set: step_left; returns true).
  enum StepEnd : uint8_t { kStepped, kHaltedByProbe, kLeftRunning };
  bool step(uint32_t &dpc_before, uint32_t &dpc_after, bool &moved, StepEnd &end);
  // Acknowledge a pending havereset: until then a V00x DM keeps DMSTATUS's halt / run bits frozen at their
  // reset values. true = one was pending (restarts() counts them: the console marks a restart).
  bool ackHaveReset();
  uint32_t restarts() const { return restarts_; }
  // Attach while the target is held in reset, then release it and stop the hart at once: the way back from
  // firmware that turns the debug pins into GPIOs or sleeps straight away. `release` lets go of the reset line;
  // the halt requests start before it and run through it.
  bool attachUnderReset(void (*hold)(void *), void (*release)(void *), void *ctx, uint32_t hold_ms, uint32_t &dpc);
  // Hold the reset line hold_ms and let go, the target left running (attach's reset TLV with method 0). The link is
  // brought up again afterwards by the caller's attach.
  void pulseReset(void (*hold)(void *), void (*release)(void *), void *ctx, uint32_t hold_ms);
  // Lower haltreq (dmactive kept): what resume, step, reset, detach and a closing connection do (§4).
  void lowerHaltreq() { if (attached()) phy_.write(0x10, 0x00000001); }

 private:
  DmiPhy &phy_;
  bool halted_ = false;
  bool kept_ = false;
  uint32_t kept0_ = 0, kept1_ = 0;
  // s0, s1, a0, a1 as the target had them: kept by a block op before it uses them and written back (read back and
  // compared) before it returns.
  bool gprs_kept_ = false;
  uint32_t gprs_[4] = {};
  bool keepGprs();
  bool giveGprs();
  uint8_t cmderr_ = 0;
  bool auto_on_ = true;     // ABSTRACTAUTO may be set: autoOff() writes 0 only then (a block op's fixed cost is DMI round trips)
  void autoOff() { if (auto_on_) { phy_.write(0x18, 0); auto_on_ = false; } }
  void autoOn() { phy_.write(0x18, 1); auto_on_ = true; }
  // abstractauto as the op found it: read and cleared at its start, written back at its very end (oep-if-debug §4)
  bool auto_kept_ = false;
  uint32_t kept_auto_ = 0;
  bool keepAuto();
  bool giveAuto();
  struct AutoBack {   // giveAuto() on every way out of an op
    Ch32Dm &dm;
    explicit AutoBack(Ch32Dm &d) : dm(d) {}
    ~AutoBack() { dm.giveAuto(); }
  };
  // waitStatus: one DMSTATUS look of a wait for a change of hart state, the link brought up again on a read that is no
  // module's and every kRelinkUs (a link dropped at the change may read the last value again)
  static constexpr uint32_t kRelinkUs = 1000;
  bool waitStatus(uint32_t &status, uint32_t &relinked_us);
  uint32_t restarts_ = 0;
  // After a reset's release (oep-if-debug §4.3): DMSTATUS read again, relinking while it does not answer, until it
  // answers or until_ms. A target that restarts itself on its way out of a reset - a bootloader handing over to the
  // application with a system reset - leaves the module silent for a while and then answers with havereset (here
  // acknowledged) and the link to bring back in step. status: the last DMSTATUS that answered.
  enum ModuleWait : uint8_t { kModuleThere, kModuleBack, kModuleGone };
  ModuleWait awaitModule(uint32_t until_ms, uint32_t &status);
  bool waitAbstract();
  // PHY re-sync + abstract-command block back to a known state (clear_auto false: abstractauto left as it is - keepAuto
  // has not read it yet)
  void relink(bool clear_auto = true);
  // After a change of hart state: relink, then looks until the link stays up (kSteadyLooks good ones in a row)
  static constexpr int kSteadyLooks = 3;
  static constexpr uint32_t kSteadyMs = 20;
  bool steady(bool clear_auto = true);
  // One look: the link is up and has not dropped since it was last brought up - DMSTATUS a module's with authenticated
  // (bit 7) set, DMCONTROL with dmactive set and hart 0 selected (bit 7 clear). A dropped link reads all ones, or the
  // same last value for both, which cannot pass both.
  bool linkHeld();
  // held(group): the group, then a look; a group that failed or a look that did not pass - steady, the group again -
  // kHeldTries times at most. true: the group succeeded with the link held over it.
  static constexpr int kHeldTries = 4;
  template <typename Group> bool held(Group group, bool clear_auto = true);
  bool moduleStatus(uint32_t &status);    // DMSTATUS read and a module's (a found version); false: relink
  void retune();                          // PHY speed search + the same
  void settleHalted(bool ack_reset);      // after the hart stopped: ack a pending reset, relink, halted_
  // the block program's set-up (a0 / a1, the program buffer, DATA1 = address), each read back
  bool loadBlock(const uint32_t *program, size_t words, uint32_t address);
  // Reads and writes a link that misses one access now and then (the looks around it passing) cannot fake: a register
  // read twice with a different read between (readSure), an abstract register read twice over two sentinels in DATA0
  // (readRegisterSure, abstractauto off), a register write read back in `mask` (writeRegisterSeen), abstractauto
  // written 0 and read back 0 (autoOffSure), no cmderr left (cmderrClear)
  bool readSure(uint8_t address, uint32_t &value);
  bool readRegisterSure(uint16_t regno, uint32_t &value);
  bool writeRegisterSeen(uint16_t regno, uint32_t value, uint32_t mask = 0xffffffffu);
  bool autoOffSure();
  bool cmderrClear();
  bool keepBlock();                       // a block op's start: abstractauto, the mailbox, the GPRs (each held)
  bool restoreBlock();                    // a block op's exit: GPRs, the mailbox, then abstractauto (each held, read back)
};

}  // namespace oep
