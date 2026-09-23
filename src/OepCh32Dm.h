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
  bool readDmi(uint8_t address, uint32_t &value) { return attach() && phy_.read(address, value); }  // abstract access register (CSR 0x000-0xfff, GPR 0x1000+)
  bool writeWord(uint32_t address, uint32_t value);
  bool writeHalfWord(uint32_t address, uint16_t value);
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
  bool resetOnce();                 // one ndmreset state machine, ends released; true = DM said running
  bool confirmExecution(uint32_t &pc, bool &halt_failed);  // attach, halt, sample dpc, resume, release
  bool waitFlash();
  bool loaderLoad();                                          // V2: inject + verify the RAM loader
  bool loaderProgramPage(uint32_t page, const uint8_t *data);  // V2: unlock/erase/program/verify on the target
};

}  // namespace oep
