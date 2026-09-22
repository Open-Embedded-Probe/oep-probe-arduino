#include "OepCh32Dm.h"

namespace oep {
namespace {
constexpr uint8_t kData0 = 0x04, kData1 = 0x05, kDmControl = 0x10, kDmStatus = 0x11, kDmHartInfo = 0x12,
                  kAbstractCs = 0x16, kCommand = 0x17, kAbstractAuto = 0x18, kProgBuf0 = 0x20;
constexpr uint32_t kFlashKeyr = 0x40022004, kFlashStatr = 0x4002200c, kFlashCtlr = 0x40022010,
                   kFlashAddr = 0x40022014, kFlashModekeyr = 0x40022024;
constexpr uint32_t kFtpg = 1u << 16, kFter = 1u << 17, kBufload = 1u << 18, kBufrst = 1u << 19, kStrt = 1u << 6,
                   kLock = 1u << 7, kFlock = 1u << 15;
// E157 loader.S: lw s0,0(a1); lw s1,0(a0); sw s1,0(s0); sw a3,4(a2); 1: lw a4,0(a2);
// andi a4,a4,1; bnez a4,1b; addi s0,s0,4; sw s0,0(a1); ebreak
constexpr uint32_t kWriter[] = {0x41044180, 0xc254c004, 0x8b054218, 0x0411ff75, 0x9002c180};
// E156 reader: lw s0,0(a1); lw s1,0(s0); addi s0,4; sw s1,0(a0); sw s0,0(a1); ebreak
constexpr uint32_t kReader[] = {0x40044180, 0xc1040411, 0x9002c180};
}  // namespace

bool Ch32Dm::waitAbstract() {
  for (int i = 0; i < 1000; ++i) {
    uint32_t cs = 0;
    if (!phy_.read(kAbstractCs, cs)) return false;
    if (cs & (1u << 12)) continue;
    cmderr_ = (cs >> 8) & 7;
    if (cmderr_) { phy_.write(kAbstractCs, 0x700); return false; }
    return true;
  }
  return false;
}

bool Ch32Dm::attach() { return phy_.attach(); }

bool Ch32Dm::readHalted() {
  uint32_t status = 0;
  if (!attached() || !phy_.read(kDmStatus, status)) return false;
  return (status & (1u << 9)) != 0;  // allhalted
}

bool Ch32Dm::halt() {
  if (!attach()) return false;
  if (halted_) return true;
  phy_.write(kDmControl, 0x80000001);
  for (int i = 0; i < 100; ++i) {
    uint32_t status = 0;
    if (phy_.read(kDmStatus, status) && (status & (1u << 9))) {
      // Keep haltreq asserted while halted (E156/E157 ran this way); acknowledge any pending reset flag.
      if (status & (3u << 18)) phy_.write(kDmControl, 0x90000001);  // haltreq | ackhavereset | dmactive
      phy_.write(kAbstractCs, 0x700);
      halted_ = true;
      return true;
    }
  }
  return false;
}

bool Ch32Dm::resume() {
  if (!attached()) return false;
  phy_.write(kAbstractAuto, 0);
  phy_.write(kDmControl, 0x40000001);
  bool ok = false;
  for (int i = 0; i < 100; ++i) {
    uint32_t status = 0;
    if (phy_.read(kDmStatus, status) && (status & (1u << 17))) { ok = true; break; }
  }
  phy_.write(kDmControl, 0x00000001);
  halted_ = !ok;
  return ok;
}

bool Ch32Dm::resetOnce() {
  if (!attach()) return false;
  // ndmreset applied to a running hart left it stopped in most cycles, while a
  // halted hart always restarted (2026-09-22, 20-cycle alternation). Halt first.
  if (!halted_) halt();
  phy_.write(kAbstractAuto, 0);
  phy_.write(kDmControl, 0x00000003);  // dmactive | ndmreset
  delay(1);
  // The de-assert write issued 100 us after asserting ndmreset was lost every
  // second time (2026-09-22, strict good/bad alternation): the DM does not take
  // DMI writes for a while after ndmreset. Write, read back, repeat until the
  // ndmreset bit is really clear.
  bool released = false;
  for (int i = 0; i < 50 && !released; ++i) {
    phy_.write(kDmControl, 0x00000001);
    uint32_t control = 0;
    if (phy_.read(kDmControl, control) && (control & 0x3) == 0x1) released = true;
    else delay(1);
  }
  // Re-activating the debug module (dmactive 0 -> 1) is what reliably let the
  // hart out of reset on the next attach when a plain de-assert did not
  // (2026-09-22). Do it here so the target runs before the probe lets go.
  phy_.write(kDmControl, 0x00000000);
  delayMicroseconds(200);
  phy_.write(kDmControl, 0x00000001);
  delay(1);
  // After ndmreset the hart may report unavailable for a while, or come out
  // halted (2026-09-22: every second reset showed allhalted+anyunavail right
  // after the release). Wait for a consistent running state, resuming a halted
  // hart, before letting go of the debug module.
  uint32_t status = 0;
  int polls = 0;
  bool running = false;
  for (; polls < 200 && !running; ++polls) {
    if (!phy_.read(kDmStatus, status)) { delayMicroseconds(500); continue; }
    const bool unavail = status & (1u << 13), halted = status & (1u << 9), allrunning = status & (1u << 11);
    if (unavail) { delayMicroseconds(500); continue; }
    if (halted) { phy_.write(kDmControl, 0x40000001); delayMicroseconds(500); continue; }  // resumereq
    if (allrunning) running = true; else delayMicroseconds(500);
  }
  // bit0 running (consistent), bit1 last allhalted, bit2 last anyunavail, bit3 allhavereset, bit4 ndmreset released, bits5-7 polls/32
  reset_diag_ = (running ? 1 : 0) | (((status >> 9) & 1) << 1) | (((status >> 13) & 1) << 2) |
                (((status >> 19) & 1) << 3) | ((released ? 1 : 0) << 4) | ((polls / 32 > 7 ? 7 : polls / 32) << 5);
  phy_.write(kDmControl, 0x10000001);  // ackhavereset
  phy_.write(kDmControl, 0x00000001);
  delay(1);
  phy_.write(kDmControl, 0x00000000);
  halted_ = false;
  phy_.release();
  delay(2);
  // A hart that ndmreset left stopped was released every time by a fresh
  // attach (lines re-driven, init sequence, dmactive 0 -> 1) and a running hart
  // is not disturbed by it (2026-09-22). Do that once here, then let go.
  if (phy_.attach()) {
    phy_.write(kDmControl, 0x00000000);
    phy_.release();
    delay(2);
  }
  return running;
}

bool Ch32Dm::confirmExecution(uint32_t &pc, bool &halt_failed) {
  pc = 0;
  halt_failed = false;
  if (!phy_.attach()) return false;
  halted_ = false;
  if (!halt()) { halt_failed = true; detach(); return false; }
  const bool sampled = readRegister(0x7b1, pc);  // dpc
  const bool resumed = resume();
  detach();
  return sampled && resumed;
}

Ch32Dm::ResetReport Ch32Dm::reset(bool confirm) {
  ResetReport report = {0, 0, 0};
  if (!attach()) return report;
  const bool running = resetOnce();
  report.attempts = 1;
  report.flags = running ? 1 : 0;
  if (!confirm) return report;
  // Evidence (E158, 2026-09-22): after ndmreset the X035 hart sits at the reset
  // vector (dpc 0, CSRs at reset values) in about 4-5 % of cycles while DMSTATUS
  // says allrunning; a haltreq/resumereq pair released it 9/9 times. So a
  // sample at pc 0 is "parked, released by this resume", not execution: sample
  // again and count only a nonzero pc. A failed halt means the DM is not usable
  // at all; redo the reset sequence for that.
  for (int attempt = 0; attempt < 4; ++attempt) {
    bool halt_failed = false;
    uint32_t pc = 0;
    const bool ok = confirmExecution(pc, halt_failed);
    if (ok && pc != 0) {
      report.flags = (report.flags & ~8) | 2;
      report.pc = pc;
      return report;
    }
    if (ok) {  // parked at the reset vector: the resume above released it; re-sample
      report.flags |= 4;
      delayMicroseconds(500);
      continue;
    }
    report.flags = (report.flags & ~8) | (halt_failed ? 8 : 0) | 4;
    if (!attach()) break;
    resetOnce();
    ++report.attempts;
  }
  return report;
}

void Ch32Dm::detach() {
  if (attached()) { phy_.write(kAbstractAuto, 0); phy_.write(kDmControl, 0); }
  halted_ = false;
  phy_.release();
}

bool Ch32Dm::loadRegisters(uint32_t &data0_address) {
  uint32_t info = 0;
  if (!phy_.read(kDmHartInfo, info)) return false;
  data0_address = 0xe0000000u | (info & 0x7ff);
  phy_.write(kAbstractAuto, 0);
  phy_.write(kData0, data0_address);     phy_.write(kCommand, 0x0023100a);  // a0 = &DATA0
  if (!waitAbstract()) return false;
  phy_.write(kData0, data0_address + 4); phy_.write(kCommand, 0x0023100b);  // a1 = &DATA1
  return waitAbstract();
}

bool Ch32Dm::readWords(uint32_t address, uint32_t *out, size_t words, uint8_t *cmderr) {
  if (!halted_ || !words) return false;
  uint32_t data0_address = 0;
  if (!loadRegisters(data0_address)) return false;
  for (size_t i = 0; i < sizeof kReader / sizeof kReader[0]; ++i) phy_.write(kProgBuf0 + i, kReader[i]);
  phy_.write(kData1, address);
  phy_.write(kAbstractAuto, 1);
  phy_.write(kCommand, 0x00240000);  // first run
  bool ok = true;
  for (size_t i = 0; i < words; ++i) {
    // E156: one program-buffer run finishes within one DMI transaction; no poll.
    if (!phy_.read(kData0, out[i])) { ok = false; break; }
  }
  // The last read launched a look-ahead; let it finish and record its cmderr
  // (an exception there is expected at the end of flash and does not affect data).
  uint8_t err = 0;
  for (int i = 0; i < 1000; ++i) {
    uint32_t cs = 0;
    if (!phy_.read(kAbstractCs, cs)) break;
    if (cs & (1u << 12)) continue;
    err = (cs >> 8) & 7;
    break;
  }
  phy_.write(kAbstractAuto, 0);
  if (err) phy_.write(kAbstractCs, 0x700);
  cmderr_ = err;
  if (cmderr) *cmderr = err;
  return ok;
}

bool Ch32Dm::readWordScalar(uint32_t address, uint32_t &value) {
  if (!halted_) return false;
  phy_.write(kAbstractAuto, 0);
  phy_.write(kProgBuf0, 0x0004a403);      // lw s0, 0(s1)
  phy_.write(kProgBuf0 + 1, 0x00100073);  // ebreak
  phy_.write(kData0, address);
  phy_.write(kCommand, 0x00231009);
  if (!waitAbstract()) return false;
  phy_.write(kCommand, 0x00241000);
  if (!waitAbstract()) return false;
  phy_.write(kCommand, 0x00221008);
  if (!waitAbstract()) return false;
  return phy_.read(kData0, value);
}

bool Ch32Dm::readRegister(uint16_t regno, uint32_t &value) {
  if (!halted_) return false;
  phy_.write(kAbstractAuto, 0);
  phy_.write(kCommand, 0x00220000u | regno);  // aarsize=32, transfer, read
  if (!waitAbstract()) return false;
  return phy_.read(kData0, value);
}

bool Ch32Dm::writeWord(uint32_t address, uint32_t value) {
  if (!halted_) return false;
  phy_.write(kAbstractAuto, 0);
  phy_.write(kProgBuf0, 0x0084a023);      // sw s0, 0(s1)
  phy_.write(kProgBuf0 + 1, 0x00100073);  // ebreak
  phy_.write(kData0, address);
  phy_.write(kCommand, 0x00231009);
  if (!waitAbstract()) return false;
  phy_.write(kData0, value);
  phy_.write(kCommand, 0x00271008);
  return waitAbstract();
}

bool Ch32Dm::waitFlash() {
  for (int i = 0; i < 4000; ++i) {
    uint32_t status = 0;
    if (!readWordScalar(kFlashStatr, status)) return false;
    if (!(status & 1)) return (status & 0x10) == 0;
  }
  return false;
}

bool Ch32Dm::flashUnlock() {
  uint32_t ctlr = 0;
  if (!readWordScalar(kFlashCtlr, ctlr)) return false;
  if (!(ctlr & (kLock | kFlock))) return true;
  if (!writeWord(kFlashKeyr, 0x45670123) || !writeWord(kFlashKeyr, 0xcdef89ab) ||
      !writeWord(kFlashModekeyr, 0x45670123) || !writeWord(kFlashModekeyr, 0xcdef89ab)) return false;
  return readWordScalar(kFlashCtlr, ctlr) && !(ctlr & (kLock | kFlock));
}

bool Ch32Dm::flashLock() { return writeWord(kFlashCtlr, kLock | kFlock); }

bool Ch32Dm::flashErasePage(uint32_t page) {
  if (page % geometry_.page) return false;
  return writeWord(kFlashCtlr, kFter) && writeWord(kFlashAddr, page) &&
         writeWord(kFlashCtlr, kFter | kStrt) && waitFlash() && writeWord(kFlashCtlr, 0);
}

bool Ch32Dm::flashProgramPage(uint32_t page, const uint8_t *data) {
  if (page % geometry_.page) return false;
  if (!writeWord(kFlashCtlr, kFtpg) || !writeWord(kFlashCtlr, kFtpg | kBufrst) || !waitFlash()) return false;
  uint32_t data0_address = 0;
  if (!loadRegisters(data0_address)) return false;
  phy_.write(kData0, kFlashStatr);      phy_.write(kCommand, 0x0023100c);  // a2 = &STATR
  if (!waitAbstract()) return false;
  phy_.write(kData0, kFtpg | kBufload); phy_.write(kCommand, 0x0023100d);  // a3 = FTPG|BUFLOAD
  if (!waitAbstract()) return false;
  for (size_t i = 0; i < sizeof kWriter / sizeof kWriter[0]; ++i) phy_.write(kProgBuf0 + i, kWriter[i]);
  phy_.write(kData1, page);
  auto wordAt = [data](size_t off) {
    return uint32_t(data[off]) | uint32_t(data[off + 1]) << 8 | uint32_t(data[off + 2]) << 16 | uint32_t(data[off + 3]) << 24;
  };
  phy_.write(kData0, wordAt(0));
  phy_.write(kCommand, 0x00240000);  // run the writer for word 0 (also arms autoexec's command)
  if (!waitAbstract()) return false;
  phy_.write(kAbstractAuto, 1);
  for (size_t off = 4; off < geometry_.page; off += 4) phy_.write(kData0, wordAt(off));
  const bool ok = waitAbstract();
  phy_.write(kAbstractAuto, 0);
  if (!ok) return false;
  return writeWord(kFlashAddr, page) && writeWord(kFlashCtlr, kFtpg | kStrt) && waitFlash() &&
         writeWord(kFlashCtlr, 0);
}

}  // namespace oep
