// RISC-V debug module operations for WCH CH32 (QingKe V4, X035 first) over RvswdPhy.
// Sequences are the ones measured in E156 (autoexec reader) and E157 (autoexec writer).
#pragma once

#include "OepRvswdPhy.h"

namespace oep {

struct FlashGeometry {
  uint32_t base;
  uint32_t size;
  uint16_t page;          // physical erase page
  uint16_t program_unit;  // bytes per program transaction (== page for X035)
};

class Ch32Dm {
 public:
  explicit Ch32Dm(RvswdPhy &phy, FlashGeometry geometry) : phy_(phy), geometry_(geometry) {}
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
  void detach();          // dmactive = 0, lines Hi-Z (target keeps running or stays halted)
  // Memory (hart must be halted). readWords uses the autoexec reader with no
  // per-word poll; cmderr is checked once at the end.
  bool readWords(uint32_t address, uint32_t *out, size_t words, uint8_t *cmderr = nullptr);
  bool readWordScalar(uint32_t address, uint32_t &value);
  bool readRegister(uint16_t regno, uint32_t &value);
  bool readDmi(uint8_t address, uint32_t &value) { return attach() && phy_.read(address, value); }  // abstract access register (CSR 0x000-0xfff, GPR 0x1000+)
  bool writeWord(uint32_t address, uint32_t value);
  // Flash (hart halted, page aligned).
  bool flashUnlock();
  bool flashErasePage(uint32_t page);
  bool flashProgramPage(uint32_t page, const uint8_t *data);  // geometry.page bytes, autoexec writer
  bool flashLock();
  uint8_t lastCmderr() const { return cmderr_; }
  uint8_t resetDiag() const { return reset_diag_; }  // DMSTATUS snapshot right after the last reset()

 private:
  RvswdPhy &phy_;
  FlashGeometry geometry_;
  bool halted_ = false;
  uint8_t cmderr_ = 0;
  uint8_t reset_diag_ = 0;
  bool waitAbstract();
  bool loadRegisters(uint32_t &data0_address);
  bool resetOnce();                 // one ndmreset state machine, ends released; true = DM said running
  bool confirmExecution(uint32_t &pc, bool &halt_failed);  // attach, halt, sample dpc, resume, release
  bool waitFlash();
};

}  // namespace oep
