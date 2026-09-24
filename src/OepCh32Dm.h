// RISC-V debug module operations for WCH CH32 over a DmiPhy. QingKe V4 (X035):
// E156 autoexec reader, E157 autoexec flash writer. QingKe V2 (CH32V003, RV32EC,
// 64-byte pages): the same reader, and flash through the RAM loader of E135/E137
// because host-driven flash-controller sequences left partial pages on that part.
#pragma once

#include <Arduino.h>

#include "OepDmiPhy.h"

namespace oep {

struct FlashGeometry {
  uint32_t base;
  uint32_t size;
  uint16_t page;          // physical erase page
  uint16_t program_unit;  // bytes per program transaction (== page for X035 and V003)
};

enum class DmProfile : uint8_t {
  kQingKeV4 = 0,  // X035 and friends: flash controller driven from the probe (E157)
  kQingKeV2 = 1,  // CH32V003: RV32EC, 2 KiB RAM, flash through the RAM loader (E135/E137)
};

class Ch32Dm {
 public:
  Ch32Dm(DmiPhy &phy, FlashGeometry geometry, DmProfile profile = DmProfile::kQingKeV4)
      : phy_(phy), geometry_(geometry), profile_(profile) {}
  DmProfile profile() const { return profile_; }
  const FlashGeometry &geometry() const { return geometry_; }
  bool attached() const { return phy_.attached(); }
  bool halted() const { return halted_; }
  bool readHalted();      // DMSTATUS.allhalted read from the target
  bool attach();
  bool halt();            // attach + haltreq, waits for allhalted
  bool resume();          // resumereq, waits for allresumeack
  // ndmreset then detach: the target restarts from its reset vector. The report
  // mirrors target.control reset: flags bit0 DM said running, bit1 execution
  // confirmed by a nonzero PC sample (confirm = true: brief halt, dpc read,
  // resume, re-sampled while the hart sits at the reset vector), bit2 the hart
  // was found parked at the reset vector or the sequence was redone, bit3 the
  // confirmation halt failed on the last attempt.
  struct ResetReport { uint8_t flags; uint8_t attempts; uint32_t pc; };
  ResetReport reset(bool confirm = true);
  // QingKe V2 only: run one of the E129/E130 RAM payloads on the target's own CPU. The payload
  // ends in a PFIC software reset, so there is nothing to confirm; true = injected and resumed.
  enum class Payload : uint8_t { kNormalizeUser, kPrepareBoot };
  bool hasPayloads() const { return profile_ == DmProfile::kQingKeV2; }
  bool runPayload(Payload which);
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
    phy_.write(address, value);   // a DMI write reports nothing; read back through the list to check
    return true;
  }  // abstract access register (CSR 0x000-0xfff, GPR 0x1000+)
  bool writeWord(uint32_t address, uint32_t value);
  bool writeHalfWord(uint32_t address, uint16_t value);
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
  // Flash (hart halted, page aligned).
  bool flashUnlock();
  bool flashErasePage(uint32_t page);   // V2 profile: no-op (the loader erases inside flashProgramPage)
  bool flashProgramPage(uint32_t page, const uint8_t *data);  // geometry.page bytes, autoexec writer
  bool flashLock();
  uint8_t lastCmderr() const { return cmderr_; }
  uint8_t resetDiag() const { return reset_diag_; }  // DMSTATUS snapshot right after the last reset()

 private:
  bool armFlash(uint32_t page, uint32_t mode);   // CTLR and ADDR written and read back before STRT
  DmiPhy &phy_;
  FlashGeometry geometry_;
  DmProfile profile_;
  bool halted_ = false;
  bool loader_resident_ = false;   // V2: the E135 loader sits at 0x20000000 until detach/reset
  uint8_t cmderr_ = 0;
  uint8_t reset_diag_ = 0;
  bool waitAbstract();
  bool loadRegisters(uint32_t &data0_address);
  ResetReport resetSequence(bool confirm);   // reset() without the final retune
  bool resetOnce();                 // one ndmreset state machine, ends released; true = DM said running
  bool confirmExecution(uint32_t &pc, bool &halt_failed);  // attach, halt, sample dpc, resume, release
  bool waitFlash();
  bool loaderLoad();                                          // V2: inject + verify the RAM loader
  bool loaderProgramPage(uint32_t page, const uint8_t *data);  // V2: unlock/erase/program/verify on the target
};

}  // namespace oep
