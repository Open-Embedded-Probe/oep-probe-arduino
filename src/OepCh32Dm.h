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
  void reset();           // ndmreset then detach: target restarts from its reset vector
  void detach();          // dmactive = 0, lines Hi-Z (target keeps running or stays halted)
  // Memory (hart must be halted). readWords uses the autoexec reader with no
  // per-word poll; cmderr is checked once at the end.
  bool readWords(uint32_t address, uint32_t *out, size_t words, uint8_t *cmderr = nullptr);
  bool readWordScalar(uint32_t address, uint32_t &value);
  bool writeWord(uint32_t address, uint32_t value);
  // Flash (hart halted, page aligned).
  bool flashUnlock();
  bool flashErasePage(uint32_t page);
  bool flashProgramPage(uint32_t page, const uint8_t *data);  // geometry.page bytes, autoexec writer
  bool flashLock();
  uint8_t lastCmderr() const { return cmderr_; }

 private:
  RvswdPhy &phy_;
  FlashGeometry geometry_;
  bool halted_ = false;
  uint8_t cmderr_ = 0;
  bool waitAbstract();
  bool loadRegisters(uint32_t &data0_address);
  bool waitFlash();
};

}  // namespace oep
