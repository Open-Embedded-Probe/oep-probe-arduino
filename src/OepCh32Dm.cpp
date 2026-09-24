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
// Block writer (the reader turned round): lw s0,0(a1); lw s1,0(a0); sw s1,0(s0); addi s0,4; sw s0,0(a1); ebreak
constexpr uint32_t kBlockWriter[] = {0x41044180, 0x0411c004, 0x9002c180};
// QingKe V2 (CH32V003) RAM flash loader from ch32-rs/wlink (MIT/Apache-2.0), built from the WCH
// EVT flash routine. Runs on the target at 0x20000000 with a0 = operation flags (0x1d = unlock,
// erase, program, verify), a1 = flash address, a2 = 64, input at 0x20000200, sp = 0x20000800;
// ends in ebreak with a0 = 0 on success. E135/E137: 82/82 pages, no retry, reset-proof.
constexpr uint32_t kLoaderBase = 0x20000000u, kLoaderInput = 0x20000200u, kLoaderStack = 0x20000800u;
constexpr uint32_t kV003FlashLoader[] = {
0xcc221111u, 0xc802ca26u, 0x00157793u, 0x06b7cf99u, 0x27b74567u, 0x86934002u,
  0x97371236u, 0xc3d4cdefu, 0x9ab70713u, 0xd3d4c3d8u, 0x7793d3d8u, 0xc79d0025u,
  0x400227b7u, 0x66ad4b98u, 0x40003337u, 0x00476713u, 0x4b98cb98u, 0xaaa68693u,
  0x04076713u, 0x47d8cb98u, 0x16638b05u, 0x4b981007u, 0xcb989b6du, 0x00457793u,
  0x0793cba9u, 0x839903f6u, 0x632dc02eu, 0xc43e7681u, 0x400032b7u, 0x400227b7u,
  0xaaa30313u, 0x4b9816fdu, 0x000203b7u, 0x00776733u, 0x4702cb98u, 0x4b98cbd8u,
  0x04076713u, 0x47d8cb98u, 0xe7698b05u, 0x8f754b98u, 0x4702cb98u, 0x04070713u,
  0x4722c03au, 0xc43a177du, 0x7793f779u, 0xcff10085u, 0x03f60793u, 0x8399c02eu,
  0x40022737u, 0x4b1cc43eu, 0x632d66c1u, 0xcb1c8fd5u, 0x20000737u, 0x20070713u,
  0x400227b7u, 0x000803b7u, 0x400032b7u, 0xaaa30313u, 0xe6b34b94u, 0xcb940076u,
  0x8a8547d4u, 0x4682fef5u, 0x043784bau, 0xc2360004u, 0xc63646c1u, 0x40844692u,
  0xc2840711u, 0x8ec14b94u, 0x47d4cb94u, 0xeab18a85u, 0x84ba4692u, 0xc2360691u,
  0x16fd46b2u, 0xfef9c636u, 0xcbd44682u, 0xe6934b94u, 0xcb940406u, 0x8a8547d4u,
  0x47d4ee85u, 0xce858ac1u, 0x06b747d8u, 0x16fdfff3u, 0x01076713u, 0x4b98c7d8u,
  0x8f754521u, 0x4462cb98u, 0x017144d2u, 0x20239002u, 0xb5f500d3u, 0x0062a023u,
  0xa023b73du, 0xb7550062u, 0x0062a023u, 0x4682b7c1u, 0x04068693u, 0x46a2c036u,
  0xc43616fdu, 0x4b98f2b5u, 0xfff306b7u, 0x8f7516fdu, 0x8941cb98u, 0x4501e119u,
  0xc02ebf7du, 0xc402060du, 0x07b78209u, 0xc6322000u, 0x20078793u, 0x87134394u,
  0x47a20047u, 0x078a4602u, 0x439c97b2u, 0x02f69963u, 0x468247a2u, 0x97b6078au,
  0x47c24394u, 0xc83e97b6u, 0x078547a2u, 0x4622c43eu, 0x87ba46b2u, 0xfcd668e3u,
  0x200007b7u, 0x6107a703u, 0x06e347c2u, 0x4541faf7u, 0xffffb79du,
};
// E129/E130 payloads (wch-protocols, UIAP toolchain `riscv-none-embed-as -march=rv32imac`):
// executed by the V003 CPU from 0x20000000 because the same registers written from a halted
// debug session did not take (E127). kNormalizeUserReset: unlock FLASH, clear BOOT_MODE, PFIC
// SYSRST. kPrepareBootAndReset: unlock, set BOOT_MODE, PD4 (software USB D-) low for a detach
// window, PFIC SYSRST -> the UIAPduino bootloader enumerates as HID 1209:b803.
constexpr uint32_t kNormalizeUserReset[] = {
0x400222b7, 0x00428293, 0x45670337, 0x12330313, 0x0062a023,
  0xcdef9337, 0x9ab30313, 0x0062a023, 0x400222b7, 0x02428293,
  0x45670337, 0x12330313, 0x0062a023, 0xcdef9337, 0x9ab30313,
  0x0062a023, 0x400222b7, 0x02828293, 0x45670337, 0x12330313,
  0x0062a023, 0xcdef9337, 0x9ab30313, 0x0062a023, 0x400222b7,
  0x00c28293, 0x0002a303, 0xffffc3b7, 0xfff38393, 0x00737333,
  0x0062a023, 0xe000e2b7, 0x04828293, 0xbeef0337, 0x08030313,
  0x0062a023, 0x0000006f,
};
constexpr uint32_t kPrepareBootAndReset[] = {
0x400222b7, 0x00428293, 0x45670337, 0x12330313, 0x0062a023,
  0xcdef9337, 0x9ab30313, 0x0062a023, 0x400222b7, 0x02428293,
  0x45670337, 0x12330313, 0x0062a023, 0xcdef9337, 0x9ab30313,
  0x0062a023, 0x400222b7, 0x02828293, 0x45670337, 0x12330313,
  0x0062a023, 0xcdef9337, 0x9ab30313, 0x0062a023, 0x400222b7,
  0x00c28293, 0x0002a303, 0xffffc3b7, 0xfff38393, 0x00737333,
  0x000043b7, 0x00736333, 0x0062a023, 0x400212b7, 0x01828293,
  0x0002a303, 0x02036313, 0x0062a023, 0x400112b7, 0x40028293,
  0x0002a303, 0xfff103b7, 0xfff38393, 0x00737333, 0x000303b7,
  0x00736333, 0x0062a023, 0x400112b7, 0x41428293, 0x01000313,
  0x0062a023, 0x004c52b7, 0xb4028293, 0xfff28293, 0xfe029ee3,
  0xe000e2b7, 0x04828293, 0xbeef0337, 0x08030313, 0x0062a023,
  0x0000006f,
};
}  // namespace

bool Ch32Dm::runPayload(Payload which) {
  if (!hasPayloads() || !halt()) return false;
  const uint32_t *words = which == Payload::kPrepareBoot ? kPrepareBootAndReset : kNormalizeUserReset;
  const size_t count = which == Payload::kPrepareBoot ? sizeof kPrepareBootAndReset / 4 : sizeof kNormalizeUserReset / 4;
  loader_resident_ = false;   // the payload lives where the loader does
  for (size_t i = 0; i < count; ++i) {
    bool ok = false;
    for (int attempt = 0; attempt < 5 && !ok; ++attempt) {
      uint32_t back = 0;
      ok = writeWord(kLoaderBase + 4 * i, words[i]) && readWordScalar(kLoaderBase + 4 * i, back) && back == words[i];
    }
    if (!ok) return false;
  }
  // Interrupts stay armed while the application is halted; resumed into the payload with MIE set,
  // the application's SysTick handler ran and rewrote its globals under the payload (2026-09-22:
  // mcause 2 illegal instruction, dpc in the application). mstatus = 0 first, as the E135 loader
  // path always did.
  if (!writeRegister(0x0300, 0) || !writeRegister(0x07b1, kLoaderBase)) return false;   // mstatus, dpc
  phy_.write(kAbstractAuto, 0);
  phy_.write(kDmControl, 0x40000001);   // resumereq (E129: twice, then drop haltreq so the reset is not re-halted)
  phy_.write(kDmControl, 0x40000001);
  phy_.write(kDmControl, 0x00000001);
  halted_ = false;
  delay(20);
  detach();
  return true;
}

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
  // One halt request is not always enough. Measured on a CH32L103 over the Pico's flying
  // wires (2026-09-23): DMCONTROL reads the request back as set while the hart keeps
  // running, and abstract commands fail cmderr=4; repeating it makes the halt land every
  // time. minichlink writes it three or four times in a row for the same reason, so
  // re-issue between polls instead of only polling.
  for (int round = 0; round < 8; ++round) {
    // A request that does not take leaves the bus out of step, and every attempt that
    // worked on the bench had a fresh bring-up in front of it, so start each round from
    // one (2026-09-23, CH32L103: without this, halt landed on every other attempt).
    phy_.reinit();
    for (int i = 0; i < 4; ++i) phy_.write(kDmControl, 0x80000001);
    for (int i = 0; i < 25; ++i) {
      uint32_t status = 0;
      if (phy_.read(kDmStatus, status) && (status & (1u << 9))) {
        // Keep haltreq asserted while halted (E156/E157 ran this way); acknowledge any pending reset flag.
        if (status & (3u << 18)) phy_.write(kDmControl, 0x90000001);  // haltreq | ackhavereset | dmactive
        phy_.write(kAbstractCs, 0x700);
        // The hart changing state drops the DMI link on this part, and the first
        // transaction afterwards can be lost: the first word of the first memory read
        // after a halt came back as the previous operation's leftover (2026-09-23).
        // Start the caller from a freshly brought-up bus.
        phy_.reinit();
        halted_ = true;
        return true;
      }
    }
  }
  return false;
}

bool Ch32Dm::resume() {
  if (!attached()) return false;
  loader_resident_ = false;   // the application owns RAM once it runs
  phy_.write(kAbstractAuto, 0);
  // Same story as halt(): one request is not always enough, and the CH32L103 never raises
  // allresumeack at all (2026-09-23) - it just starts running. So repeat the request, and
  // accept either the acknowledgement or the hart plainly being back on its feet.
  bool ok = false;
  for (int round = 0; round < 8 && !ok; ++round) {
    phy_.reinit();
    for (int i = 0; i < 4; ++i) phy_.write(kDmControl, 0x40000001);
    for (int i = 0; i < 25 && !ok; ++i) {
      uint32_t status = 0;
      if (!phy_.read(kDmStatus, status)) continue;
      if (status & (1u << 17)) ok = true;                        // allresumeack
      else if ((status & 0xf) == 2 && (status & (1u << 11)) && !(status & (1u << 9)))
        ok = true;                                               // allrunning, not halted
    }
  }
  phy_.write(kDmControl, 0x00000001);
  halted_ = !ok;
  return ok;
}

bool Ch32Dm::resetOnce() {
  if (!attach()) return false;
  loader_resident_ = false;
  // ndmreset applied to a running hart left it stopped in most cycles, while a
  // halted hart always restarted (2026-09-22, 20-cycle alternation). Halt first.
  if (!halted_) halt();
  phy_.useSafeSpeed();                 // the part comes out of reset on its default, slower clock
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
  const ResetReport report = resetSequence(confirm);
  if (attached()) phy_.retune();   // resetOnce() dropped to the slowest speed for the reset itself
  return report;
}

Ch32Dm::ResetReport Ch32Dm::resetSequence(bool confirm) {
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
  loader_resident_ = false;
  phy_.park();   // floating both wires high is how this bus is told to reset
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
  // Long runs of these hiccup now and then - roughly one chunk in a couple of hundred on
  // the CH32L103 jig, which is a whole-flash verify failing every few tries. The words
  // already read are then meaningless, so redo the chunk from its own bring-up rather
  // than hand the caller a plausible-looking answer (2026-09-23).
  uint8_t err = 0;
  for (int attempt = 0; attempt < 3; ++attempt) {
    uint32_t data0_address = 0;
    if (!loadRegisters(data0_address)) { phy_.reinit(); continue; }
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
    err = 0;
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
    // The reader bumps DATA1 by 4 every run, so the address it left behind counts the runs:
    // the first one plus one per DATA0 read, or one fewer when the last look-ahead faulted
    // (cmderr 3) before its store. A mangled address write, a trigger the module missed or
    // one it took twice all leave a count that is off - and all of them otherwise hand back
    // plausible words with no error. Measured on the CH32L103 over the Pico's flying wires
    // (2026-09-23): a whole-flash CRC came out different about one read in three, with
    // every read reporting success.
    uint32_t next = 0;
    const uint32_t ran = address + 4u * static_cast<uint32_t>(words + (err == 3 ? 0 : 1));
    const bool counted = phy_.read(kData1, next) && next == ran;
    // cmderr 3 is the look-ahead walking off the end of a region and says nothing about
    // the words already read. Anything else means the program buffer did not run: the
    // reads then returned whatever was left in DATA0, which looks like data and is not.
    if (ok && counted && (err == 0 || err == 3)) return true;
    phy_.reinit();
  }
  return false;
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

bool Ch32Dm::writeRegister(uint16_t regno, uint32_t value) {
  if (!halted_) return false;
  phy_.write(kAbstractAuto, 0);
  phy_.write(kData0, value);
  phy_.write(kCommand, 0x00230000u | regno);  // aarsize=32, transfer, write
  return waitAbstract();
}

// Option bytes are programmed 16 bits at a time, so removing read protection needs a real
// half-word store on the target - a word store writes the neighbouring option byte too.
bool Ch32Dm::writeHalfWord(uint32_t address, uint16_t value) {
  if (!halted_) return false;
  phy_.write(kAbstractAuto, 0);
  phy_.write(kProgBuf0, 0x00849023);      // sh s0, 0(s1)
  phy_.write(kProgBuf0 + 1, 0x00100073);  // ebreak
  phy_.write(kData0, address);
  phy_.write(kCommand, 0x00231009);
  if (!waitAbstract()) return false;
  phy_.write(kData0, value);
  phy_.write(kCommand, 0x00271008);
  return waitAbstract();
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

bool Ch32Dm::writeWordsFast(uint32_t address, const uint32_t *words, size_t count) {
  if (!halted_ || !count || (address & 3)) return false;
  // Plain memory, so a failed attempt is simply redone from the start. As with readWords, the
  // address the writer leaves in DATA1 counts the runs and catches a missed or doubled trigger.
  for (int attempt = 0; attempt < 3; ++attempt) {
    uint32_t data0_address = 0;
    if (!loadRegisters(data0_address)) { phy_.reinit(); continue; }
    for (size_t i = 0; i < sizeof kBlockWriter / sizeof kBlockWriter[0]; ++i) phy_.write(kProgBuf0 + i, kBlockWriter[i]);
    phy_.write(kData1, address);
    phy_.write(kData0, words[0]);
    phy_.write(kCommand, 0x00240000);  // run the writer for word 0 (also arms autoexec's command)
    bool ok = waitAbstract();
    if (ok && count > 1) {
      phy_.write(kAbstractAuto, 1);
      for (size_t i = 1; i < count; ++i) phy_.write(kData0, words[i]);
      ok = waitAbstract();
      phy_.write(kAbstractAuto, 0);
    }
    uint32_t next = 0;
    if (ok && phy_.read(kData1, next) && next == address + 4u * static_cast<uint32_t>(count)) return true;
    phy_.write(kAbstractAuto, 0);
    phy_.write(kAbstractCs, 0x700);
    phy_.reinit();
  }
  return false;
}

bool Ch32Dm::runUntilHalt(uint32_t pc, const uint16_t *regnos, const uint32_t *values, size_t count,
                          uint32_t timeout_us, RunReport &report) {
  report = {false, 0, 0, 0};
  if (!halted_) return false;
  // Without ebreakm the final ebreak traps through mtvec and the application restarts
  // (the V003 loader finding, 2026-09-22). prv = M: the hart may have been stopped in U mode
  // (ArduinoCore-CH32 sketches on V3B/V4 run there), where interrupts cannot be masked; the
  // caller masks them with mstatus in the register list.
  uint32_t dcsr = 0;
  if (!readRegister(0x07b0, dcsr) || !writeRegister(0x07b0, dcsr | 0xb003u)) return false;
  for (size_t i = 0; i < count; ++i)
    if (!writeRegister(regnos[i], values[i])) return false;
  if (!writeRegister(0x07b1, pc)) return false;
  loader_resident_ = false;   // the host's code may have overwritten the V2 loader's RAM
  phy_.write(kAbstractAuto, 0);
  // The CH32L103 needs what halt()/resume() learned (2026-09-23): one resumereq does not always take and it never
  // raises allresumeack, and a change of hart state drops its DMI link, so a failed read is followed by a bus
  // bring-up. A stop with dpc still at `pc` means the code never ran - ask again (the host's code ends in an
  // ebreak somewhere else, so a real stop never sits at its first instruction). Measured on the L103 over flying
  // leads, 2026-09-24: 2 of 248 runs stopped at pc without running, 1 lost the link while polling.
  const uint32_t started = micros();
  for (int attempt = 0; attempt < 4 && !report.stopped; ++attempt) {
    phy_.write(kDmControl, 0x40000001);   // resumereq: run to the ebreak
    bool halted = false;
    while (micros() - started < timeout_us) {
      uint32_t status = 0;
      if (!phy_.read(kDmStatus, status)) { phy_.reinit(); continue; }
      if (status & (1u << 9)) { halted = true; break; }
    }
    phy_.write(kDmControl, 0x80000001);   // back to haltreq | dmactive, stopped or not
    phy_.write(kAbstractCs, 0x700);
    if (!halted) break;                   // the timeout: forced halt below
    phy_.reinit();                        // the stop changed the hart's state
    uint32_t dpc = 0;
    if (readRegister(0x07b1, dpc) && dpc == pc) continue;   // never ran: resume again
    report.stopped = true;
  }
  report.elapsed_us = micros() - started;
  if (!report.stopped) {
    halted_ = false;
    if (!halt()) return false;
  }
  // The caller judges success from dpc (its ebreak) and a0; a stop somewhere else is a fault.
  return readRegister(0x07b1, report.dpc) && readRegister(0x100a, report.a0);
}

bool Ch32Dm::ackHaveReset() {
  uint32_t status = 0;
  if (!attach() || !phy_.read(kDmStatus, status)) return false;
  if (!(status & (3u << 18))) return false;               // anyhavereset / allhavereset
  phy_.write(kDmControl, halted_ ? 0x90000001 : 0x10000001);   // ackhavereset, haltreq kept if we hold a halt
  // The CH32L103 drops its DMI link after this write and the next read fails (2026-09-24: every attach failed
  // until the bus was brought up again here, as halt() does after a change of state).
  phy_.reinit();
  return true;
}

bool Ch32Dm::resetHalt(uint32_t &dpc) {
  dpc = 0;
  if (!attach()) return false;
  loader_resident_ = false;
  if (!halted_) halt();                    // ndmreset on a running hart left it stopped oddly (2026-09-22)
  phy_.write(kAbstractAuto, 0);
  // haltreq is held through the reset, so the hart comes out of it into debug mode - provided the writes land. The
  // part leaves reset on its default clock, slower than a sketch that raised it, and at the speed attach() tuned to
  // the sketch the release was garbled and the hart ran into its image (2026-09-24, CH32X035 from a running
  // sketch: 0 of 28 at the vector; at the slowest period 28 of 28, as through a WCH-LinkE). So run the reset at
  // the slowest period and tune the link again once the hart has stopped. A DMSTATUS without version 2 is noise.
  phy_.useSafeSpeed();
  phy_.write(kDmControl, 0x80000003);      // haltreq | ndmreset | dmactive
  phy_.write(kDmControl, 0x80000001);
  // Out of reset the hart may be unavailable for a while; it should then come up halted at the reset vector.
  // The release is written again with haltreq in case the DM ignored it (it ignores DMI writes for a while after
  // ndmreset, 2026-09-22), and ndmreset is checked clear at the end.
  bool halted = false;
  for (int poll = 0; poll < 400 && !halted; ++poll) {
    uint32_t status = 0;
    if (!phy_.read(kDmStatus, status) || (status & 0xf) != 2) { phy_.reinit(); continue; }
    if (status & (1u << 13)) { delayMicroseconds(250); continue; }   // anyunavail
    if (status & (1u << 9)) { halted = true; break; }
    phy_.write(kDmControl, 0x80000001);
    delayMicroseconds(250);
  }
  uint32_t control = 0;
  const bool released = phy_.read(kDmControl, control) && (control & 0x3) == 0x1;
  phy_.write(kDmControl, 0x90000001);      // ackhavereset, haltreq kept
  phy_.write(kAbstractCs, 0x700);
  phy_.reinit();
  halted_ = halted;
  phy_.retune();                           // at the default clock this time
  return released && halted && readRegister(0x07b1, dpc);
}

bool Ch32Dm::step(uint32_t &dpc_before, uint32_t &dpc_after, bool &moved) {
  dpc_before = dpc_after = 0;
  moved = false;
  if (!halted_) return false;
  uint32_t dcsr = 0;
  if (!readRegister(0x07b1, dpc_before) || !readRegister(0x07b0, dcsr)) return false;
  if (!writeRegister(0x07b0, dcsr | 0x4u)) return false;   // dcsr.step, the privilege level as it is
  phy_.write(kAbstractAuto, 0);
  phy_.write(kDmControl, 0x40000001);      // resumereq, once
  bool halted = false;
  const uint32_t started = micros();
  while (micros() - started < 50000u) {
    uint32_t status = 0;
    if (!phy_.read(kDmStatus, status)) { phy_.reinit(); continue; }
    if (status & (1u << 9)) { halted = true; break; }
  }
  phy_.write(kDmControl, 0x80000001);
  phy_.write(kAbstractCs, 0x700);
  phy_.reinit();
  if (!halted) {
    halted_ = false;
    if (!halt()) return false;
  }
  const bool ok = readRegister(0x07b1, dpc_after) && writeRegister(0x07b0, dcsr & ~0x4u);
  moved = dpc_after != dpc_before;
  return ok;
}

bool Ch32Dm::attachUnderReset(void (*hold)(void *), void (*release)(void *), void *ctx, uint32_t hold_ms,
                              uint32_t &dpc) {
  dpc = 0;
  halted_ = false;
  loader_resident_ = false;
  hold(ctx);
  delay(hold_ms);
  // Bring the link up while the target is held (it may or may not answer yet), queue the halt request, then let
  // go and keep asking until the hart reports halted - before firmware that kills the debug pins gets that far.
  phy_.reinit();
  phy_.attach();
  phy_.write(kDmControl, 0x80000001);
  release(ctx);
  bool halted = false;
  const uint32_t started = micros();
  while (!halted && micros() - started < 200000u) {
    phy_.write(kDmControl, 0x80000001);
    uint32_t status = 0;
    if (phy_.read(kDmStatus, status)) halted = (status & (1u << 9)) != 0;
    else phy_.reinit();
  }
  if (!halted) return false;
  phy_.write(kDmControl, 0x90000001);      // ackhavereset, haltreq kept
  phy_.write(kAbstractCs, 0x700);
  phy_.reinit();
  halted_ = true;
  return readRegister(0x07b1, dpc);
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

// Before a flash operation is started, make sure the controller will act on the page we
// mean and in the mode we mean. Both registers are written through abstract commands, and a
// DMI write that goes astray does not fail - it writes a different value, or to a different
// place. On the CH32L103 over flying wires an erase landed on a page that had been programmed
// and verified a moment earlier (2026-09-23); only the whole-flash CRC at the end noticed.
// Reading both back before STRT also means no mode bit other than the one asked for - mass
// erase least of all - is ever set when the operation starts.
bool Ch32Dm::armFlash(uint32_t page, uint32_t mode) {
  static constexpr uint32_t kModeMask = kFtpg | kFter | kBufload | kBufrst | kStrt | 0x7u;
  for (int attempt = 0; attempt < 3; ++attempt) {
    uint32_t ctlr = 0, addr = 0;
    if (writeWord(kFlashCtlr, mode) && writeWord(kFlashAddr, page) &&
        readWordScalar(kFlashCtlr, ctlr) && readWordScalar(kFlashAddr, addr) &&
        (ctlr & kModeMask) == mode && addr == page) return true;
  }
  return false;
}

bool Ch32Dm::flashErasePage(uint32_t page) {
  if (page % geometry_.page) return false;
  if (profile_ == DmProfile::kQingKeV2) return halted_;  // the loader erases the page it programs
  return armFlash(page, kFter) && writeWord(kFlashCtlr, kFter | kStrt) && waitFlash() &&
         writeWord(kFlashCtlr, 0);
}

bool Ch32Dm::loaderLoad() {
  if (loader_resident_) return true;
  for (size_t i = 0; i < sizeof kV003FlashLoader / sizeof kV003FlashLoader[0]; ++i) {
    const uint32_t address = kLoaderBase + 4 * i;
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {   // E132: the flying SWIO lead shows rare 1-bit errors
      uint32_t back = 0;
      ok = writeWord(address, kV003FlashLoader[i]) && readWordScalar(address, back) && back == kV003FlashLoader[i];
    }
    if (!ok) return false;
  }
  loader_resident_ = true;
  return true;
}

bool Ch32Dm::loaderProgramPage(uint32_t page, const uint8_t *data) {
  if (!halted_ || !loaderLoad()) return false;
  for (size_t off = 0; off < geometry_.page; off += 4) {
    const uint32_t word = uint32_t(data[off]) | uint32_t(data[off + 1]) << 8 | uint32_t(data[off + 2]) << 16 | uint32_t(data[off + 3]) << 24;
    if (!writeWord(kLoaderInput + off, word)) return false;
  }
  // The loader ends in ebreak. That only re-enters debug mode with dcsr.ebreakm set; otherwise it
  // traps through mtvec (0 on a fresh part = the reset vector) and the application restarts,
  // clobbering the loader (2026-09-22: RAM at 0x20000000 held .data afterwards, pages were
  // programmed but a0 read garbage). Set ebreakm/s/u before the run.
  uint32_t dcsr = 0;
  if (!readRegister(0x07b0, dcsr) || !writeRegister(0x07b0, dcsr | 0xb000u)) return false;
  if (!writeRegister(0x100a, 0x1d) || !writeRegister(0x100b, page) || !writeRegister(0x100c, geometry_.page) ||
      !writeRegister(0x0300, 0) ||                      // proven E135 sequence writes CSR 0x300 = 0 before the run
      !writeRegister(0x1002, kLoaderStack) || !writeRegister(0x07b1, kLoaderBase)) return false;
  phy_.write(kDmControl, 0x40000001);                   // resumereq: the loader runs to its ebreak
  bool halted = false;
  const uint32_t started = micros();
  while (micros() - started < 50000u) {
    uint32_t status = 0;
    if (phy_.read(kDmStatus, status) && (status & (3u << 8)) == (3u << 8)) { halted = true; break; }
  }
  phy_.write(kDmControl, 0x80000001);                   // back to haltreq | dmactive (the loader stopped, or force it)
  phy_.write(kAbstractCs, 0x700);
  if (halted) {
    // Done when the hart sits on the loader's ebreak. a0 is not a status: it read 0x10 (the word
    // count) after every good page on 2026-09-22; E135's "a0 = 0" was sampled after the ebreak
    // had trapped into the restarted application. The caller's read-back is the data proof.
    static uint32_t ebreak_offset = 0;
    if (!ebreak_offset) {
      for (size_t i = 0; i < sizeof kV003FlashLoader / sizeof kV003FlashLoader[0] && !ebreak_offset; ++i)
        for (unsigned half = 0; half < 2; ++half)
          if (((kV003FlashLoader[i] >> (16 * half)) & 0xffffu) == 0x9002u) { ebreak_offset = 4 * i + 2 * half; break; }
    }
    uint32_t dpc = 0;
    return readRegister(0x07b1, dpc) && dpc == kLoaderBase + ebreak_offset;
  }
  // E135: completion reads can be lost on SWIO. Force a halt as the fence, assume the loader is
  // gone, and let the page itself decide: it is programmed iff it reads back as written.
  halted_ = false;
  loader_resident_ = false;
  if (!halt()) return false;
  uint32_t back[64 / 4];
  if (geometry_.page > sizeof back || !readWords(page, back, geometry_.page / 4)) return false;
  for (size_t off = 0; off < geometry_.page; off += 4) {
    const uint32_t word = uint32_t(data[off]) | uint32_t(data[off + 1]) << 8 | uint32_t(data[off + 2]) << 16 | uint32_t(data[off + 3]) << 24;
    if (back[off / 4] != word) return false;
  }
  return true;
}

bool Ch32Dm::flashProgramPage(uint32_t page, const uint8_t *data) {
  if (page % geometry_.page) return false;
  if (profile_ == DmProfile::kQingKeV2) return loaderProgramPage(page, data);
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
  return armFlash(page, kFtpg) && writeWord(kFlashCtlr, kFtpg | kStrt) && waitFlash() &&
         writeWord(kFlashCtlr, 0);
}

}  // namespace oep
