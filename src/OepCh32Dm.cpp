// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepCh32Dm.h"

namespace oep {
namespace {
constexpr uint32_t kDmWaitMs = v1::reg::kLimitDmWaitMs;   // one wait for DM state inside a high-level op (oep-if-debug §4)
constexpr uint8_t kData0 = 0x04, kData1 = 0x05, kDmControl = 0x10, kDmStatus = 0x11, kDmHartInfo = 0x12,
                  kAbstractCs = 0x16, kCommand = 0x17, kAbstractAuto = 0x18, kProgBuf0 = 0x20;
// E156 reader: lw s0,0(a1); lw s1,0(s0); addi s0,4; sw s1,0(a0); sw s0,0(a1); ebreak
constexpr uint32_t kReader[] = {0x40044180, 0xc1040411, 0x9002c180};
// Block writer (the reader turned round): lw s0,0(a1); lw s1,0(a0); sw s1,0(s0); addi s0,4; sw s0,0(a1); ebreak
constexpr uint32_t kBlockWriter[] = {0x41044180, 0x0411c004, 0x9002c180};
}  // namespace

bool Ch32Dm::waitAbstract() {
  for (int i = 0; i < 1000 && !(i && phy_.pastBudget()); ++i) {
    uint32_t cs = 0;
    if (!phy_.read(kAbstractCs, cs)) return false;
    if (cs & (1u << 12)) continue;
    cmderr_ = (cs >> 8) & 7;
    if (cmderr_) { phy_.write(kAbstractCs, 0x700); return false; }
    return true;
  }
  return false;
}

bool Ch32Dm::attach() {
  if (phy_.attached()) return true;
  if (!phy_.attach()) return false;
  // Leave the abstract-command block in a known state. A session that ended mid-sequence can leave autoexec armed
  // on DATA0 or a sticky cmderr behind (2026-09-23).
  phy_.write(kAbstractAuto, 0);
  auto_on_ = false;
  phy_.write(kAbstractCs, 0x700);
  return true;
}

// A scan's look at a pair (oep-if-debug §1, what scan writes): the PHY's bring-up - the wake / configuration sequence
// and dmactive - and DMSTATUS read; no write check, no scratch, nothing else written, the link left down. A debug module
// answers with a "found" version in DMSTATUS[3:0]; anything else is noise. An attached link is only read.
bool Ch32Dm::probe(uint32_t &dmstatus) {
  if (phy_.attached() ? !phy_.read(kDmStatus, dmstatus) : !phy_.bringUp(dmstatus)) return false;
  return dmVersionKnown(dmstatus);
}

// Bring the link up again (the CH32 drops it on a change of state) and put the abstract-command block back in a
// known state: autoexec off, cmderr cleared.
void Ch32Dm::relink() {
  phy_.reinit();
  phy_.write(kAbstractAuto, 0);
  auto_on_ = false;
  phy_.write(kAbstractCs, 0x700);
}

// Measure the link speed again (the target's clock may have changed), then put the abstract-command block back in
// a known state: the search re-syncs the bus once per candidate and leaves whatever the probes did behind.
void Ch32Dm::retune() {
  phy_.retune();
  phy_.write(kAbstractAuto, 0);
  auto_on_ = false;
  phy_.write(kAbstractCs, 0x700);
}

// The hart has just stopped: acknowledge a pending reset (haltreq kept), clear cmderr, start the caller from a
// freshly brought-up bus (the first transaction after a change of state can be lost on this part).
void Ch32Dm::settleHalted(bool ack_reset) {
  if (ack_reset) { phy_.write(kDmControl, 0x90000001); ++restarts_; }   // haltreq | ackhavereset | dmactive
  relink();
  halted_ = true;
}

// What DMSTATUS says now: allhalted (bit 9) of a version-2 or -3 module, with no reset pending (a V00x freezes the
// halt / run bits until it is acknowledged, so a pending one is acknowledged first).
bool Ch32Dm::checkHalted() {
  if (!attach()) return false;
  uint32_t status = 0;
  if (!phy_.read(kDmStatus, status) || !dmVersionKnown(status)) return false;
  if (status & (3u << 18)) {   // havereset: acknowledge, then read again
    ackHaveReset();
    if (!phy_.read(kDmStatus, status) || !dmVersionKnown(status)) return false;
  }
  const bool now = (status & (1u << 9)) != 0;
  if (now && !halted_) settleHalted(false);
  halted_ = now;
  return true;
}

// The high-level ops work on hart 0 (oep-if-debug §4): a hartsel (or hasel) the host left in DMCONTROL through dmi goes
// back to 0 before the op, which then writes DMCONTROL with hartsel 0 only - it starts and returns with it 0. haltreq
// reads 0 (write-only), so the write sets dmactive alone; a halted hart 0 stays halted.
void Ch32Dm::selectHart0() {
  uint32_t control = 0;
  if (!attach() || !phy_.read(kDmControl, control) || control == 0xffffffffu) return;
  if (control & 0x07ffffc0u) phy_.write(kDmControl, 0x00000001);   // hasel (26), hartsello (25:16), hartselhi (15:6)
}

// The target's mailbox across an op (oep-if-debug §4.2). A client's halt -> read_block -> resume left the abstract
// command's word in DATA0; the target, missing its dmseq frame, read that as silence and waited out its timeout, and
// the console went quiet for seconds (2026-09-30, CH32X035, ch32rv monitor + another client).
void Ch32Dm::keepMailbox() {
  if (kept_) return;
  kept_ = phy_.read(kData0, kept0_) && phy_.read(kData1, kept1_);
}

void Ch32Dm::giveMailbox() {   // abstractauto is off here: writing DATA0 runs no command
  if (!kept_) return;
  phy_.write(kData1, kept1_);   // the order the target writes them in (dmseq: DATA1 before DATA0)
  phy_.write(kData0, kept0_);
  kept_ = false;
}

// GPRs x8..x11 (s0, s1, a0, a1): what the block ops use (oep-if-debug §4.5). A sketch whose loop was stopped
// 109 times by halt -> read_block -> resume died when they were not put back (2026-09-30, CH32X035).
bool Ch32Dm::keepGprs() {
  if (gprs_kept_) return true;
  for (uint16_t k = 0; k < 4; ++k)
    if (!readRegister(0x1008 + k, gprs_[k])) return false;   // not kept: the op does not run (it would break the target)
  gprs_kept_ = true;
  return true;
}

void Ch32Dm::giveGprs() {   // while still halted, before the mailbox (these writes go through DATA0)
  if (!gprs_kept_) return;
  // Four register writes, one completion check at the end (E156: an abstract command finishes within one DMI
  // transaction, and nothing here acts on a failure beyond clearing cmderr). 9 round trips instead of 16.
  autoOff();
  for (uint16_t k = 0; k < 4; ++k) {
    phy_.write(kData0, gprs_[k]);
    phy_.write(kCommand, 0x00230000u | (0x1008 + k));
  }
  waitAbstract();
  gprs_kept_ = false;
}

// A block op's exit, whatever happened inside it: the GPRs back, autoexec off, cmderr clear, then the mailbox
// (oep-if-debug §4 table: the probe carries nothing past the answer).
void Ch32Dm::restoreBlock() {
  autoOff();
  giveGprs();
  autoOff();
  if (cmderr_) phy_.write(kAbstractCs, 0x700);
  giveMailbox();
}

bool Ch32Dm::halt() {
  if (!attach()) return false;
  // Already stopped: nothing to do (v1 riscv-dm halt is idempotent). Asked of the debug module rather than halted_,
  // since the host may have resumed or halted the hart through raw DMI writes since. Not while a reset is pending:
  // a V00x keeps DMSTATUS's halt / run bits frozen until it is acknowledged.
  {
    uint32_t status = 0;
    if (phy_.read(kDmStatus, status) && dmVersionKnown(status) && (status & (1u << 9)) && !(status & (3u << 18))) {
      if (!halted_) settleHalted(false);
      return true;
    }
  }
  // One halt request is not always enough. Measured on a CH32L103 through the RP2350
  // probe (2026-09-23): DMCONTROL reads the request back as set while the hart keeps
  // running, and abstract commands fail cmderr=4; repeating it makes the halt land every
  // time. minichlink writes it three or four times in a row for the same reason, so
  // re-issue between polls instead of only polling - for dm_wait_ms of time at most (oep-if-debug §4: the wait for
  // allhalted; the attach budget's end too, inside an attach).
  const uint32_t started = millis();
  auto waited = [&]() { return millis() - started >= kDmWaitMs || phy_.pastBudget(); };
  for (int round = 0; round == 0 || !waited(); ++round) {
    // A request that does not take leaves the bus out of step, and every attempt that
    // worked on the bench had a fresh bring-up in front of it, so start each round from
    // one (2026-09-23, CH32L103: without this, halt landed on every other attempt).
    relink();
    for (int i = 0; i < 4; ++i) phy_.write(kDmControl, 0x80000001);
    for (int i = 0; i < 25 && (i == 0 || !waited()); ++i) {
      uint32_t status = 0;
      if (phy_.read(kDmStatus, status) && dmHalted(status)) {
        // Keep haltreq asserted while halted (E156/E157 ran this way; oep-if-debug §4 allows it). The hart changing
        // state drops the DMI link on this part, and the first word of the first memory read after a halt came back as
        // the previous operation's leftover (2026-09-23): settleHalted starts the caller from a freshly brought-up bus.
        settleHalted(status & (3u << 18));
        return true;
      }
    }
  }
  // Not halted within the wait: haltreq cleared before the answer (oep-if-debug §4.2, status timeout), so the hart is
  // not stopped later behind the host's back.
  phy_.write(kDmControl, 0x00000001);
  return false;
}

bool Ch32Dm::resume() {
  if (!attached()) return false;
  phy_.write(kAbstractAuto, 0);
  // oep-if-debug §4.2: one resumereq; ok = the hart left debug mode (allresumeack, or running and not halted). A
  // target that needs the request again (the CH32V006 now and then) or never raises allresumeack and stops again at
  // once (a breakpoint straight ahead on a CH32L103) comes back as not ok: the host, which knows the part, reads dpc and
  // asks again (the CH32 rule lives in the host, not in this generic operation). Nothing is kept across ops, so
  // nothing is given back here (§4).
  relink();                                                      // a change of state drops the CH32's link
  phy_.write(kDmControl, 0x40000001);                            // resumereq, once (haltreq lowered)
  // Looked for in DMSTATUS for dm_wait_ms of time (oep-if-debug §4, §4.2: not seen by then, status state).
  bool ok = false;
  const uint32_t started = millis();
  do {
    uint32_t status = 0;
    if (!phy_.read(kDmStatus, status) || !dmVersionKnown(status)) continue;   // all ones has allresumeack set too
    if (status & (1u << 17)) ok = true;                          // allresumeack
    else if ((status & (1u << 11)) && !(status & (1u << 9)))
      ok = true;                                                 // allrunning, not halted
  } while (!ok && millis() - started < kDmWaitMs);
  phy_.write(kDmControl, 0x00000001);
  halted_ = !ok;
  return ok;
}

// The module again after a reset's release: answering at once (kModuleThere), or silent first and answering by
// until_ms (kModuleBack: the target restarted itself - havereset acknowledged, the hart's view taken from DMSTATUS), or
// silent until then (kModuleGone). The reads bring the link back in step between them (relink: a CH32V003's system
// reset drops its SWIO configuration and dmactive); each poll yields for 1 ms.
Ch32Dm::ModuleWait Ch32Dm::awaitModule(uint32_t until_ms, uint32_t &status) {
  bool silent = false;
  while (!(phy_.read(kDmStatus, status) && dmVersionKnown(status))) {
    silent = true;
    if (static_cast<int32_t>(millis() - until_ms) >= 0) return kModuleGone;
    delay(1);
    relink();
  }
  if (!silent) return kModuleThere;
  halted_ = false;   // the target started over: no halt of the probe's is held (ackHaveReset keeps haltreq only then)
  if ((status & (3u << 18)) && ackHaveReset() && !(phy_.read(kDmStatus, status) && dmVersionKnown(status))) status = 0;
  halted_ = dmHalted(status);
  return kModuleBack;
}

Ch32Dm::ResetReport Ch32Dm::reset(bool confirm) {
  // Stop the hart at its reset vector (resetHalt: haltreq held through ndmreset, at the slowest speed, retuned once
  // stopped), then let it run. The link stays attached the whole way. The old sequence - ndmreset with the hart
  // running, then letting go of the bus and attaching again (a wake burst each time) to release and to confirm it -
  // came through first time in only 9 of 20 resets on the CH32L103 and failed 5, while reset-halt + resume ran
  // 20 of 20 on the L103, the X035 and the V003 alike (2026-09-25). Stopping at the vector first also takes care of
  // the X035's parked-at-vector resets (E158).
  // The procedure is redone at most reset_retries (1) times (oep-if-debug §4.3: flags bit2).
  //
  // A target may restart itself on its way out of the reset: a CH32V003 that boots through its bootloader (BOOT_MODE
  // set, as after a power-on) runs it from the vector and then hands over to the application with a system reset; its
  // module answered nothing for a few hundred ms from just after the resume (P4 bench, 0.0.28+1c940ca: the confirmation
  // failed at 212 ms with status line, a mode 0 reset answered ok and the next requests got line). So after the resume
  // the op looks at the module once more (after the same 1 ms the confirmation gives the image) and, when it is silent,
  // waits for it to answer again - relinking, its havereset acknowledged - before it confirms or answers: a host's next
  // request finds the module answering. The wait ends kResetSettleMs after the op started, so that with the
  // confirmation's halt and resume (dm_wait_ms each) the answer still comes within the host's wait for a request with
  // no time argument (host_wait_add_ms, core §4.4). Still silent then: bit0 cleared, no redo (a redo restarts the
  // target into the same hand-over), and the caller answers status line.
  ResetReport report = {0, 0, 0};
  cmderr_ = 0;   // a cmderr of this reset's own abstract commands says fault (§4.3)
  const uint32_t settle_end = millis() + kResetSettleMs;
  for (uint8_t attempt = 1; attempt <= 1 + v1::reg::kLimitResetRetries; ++attempt) {
    report.attempts = attempt;
    if (attempt > 1) report.flags |= 4;                     // redone
    uint32_t dpc = 0;
    if (!resetHalt(dpc)) continue;
    const bool resumed = resume();
    delay(1);                                               // a moment for the image to start
    uint32_t status = 0;
    const ModuleWait module = awaitModule(settle_end, status);
    if (module == kModuleGone) { report.flags &= static_cast<uint8_t>(~1u); return report; }
    // A resume whose acknowledgement the silence swallowed: the module back and the hart running is the same thing.
    if (!resumed && !(module == kModuleBack && (status & (1u << 11)) && !(status & (1u << 9)))) continue;
    report.flags |= 1;                                      // released and running
    if (!confirm) return report;
    // Confirm execution: a brief halt for the pc. A pc still at the vector is not execution yet: resume and sample
    // again. The halt is the probe's own: DATA goes back before the resume (oep-if-debug §4: as step).
    for (int sample = 0; sample < 3; ++sample) {
      if (sample) delay(1);
      if (awaitModule(settle_end, status) == kModuleGone) { report.flags &= static_cast<uint8_t>(~1u); return report; }
      uint32_t pc = 0;
      if (!halt()) { report.flags |= 8; break; }            // the confirmation halt failed
      keepMailbox();
      const bool read = readRegister(0x7b1, pc);
      giveMailbox();
      if (!resume()) { report.flags |= 8; break; }
      if (read && pc != 0) {
        report.flags = static_cast<uint8_t>((report.flags & ~8) | 2);
        report.pc = pc;
        return report;
      }
    }
  }
  return report;
}

void Ch32Dm::detach() {
  gprs_kept_ = kept_ = false;   // an op that failed midway left nothing to give back here (it restored its own)
  // dmactive stays set (haltreq / resumereq / ndmreset go): writing 0 reset the debug module, which wiped a dmseq
  // frame the target had out in DATA0, and the target, reading 0 as silence, then waited out its timeout - counted in
  // its own reads of DATA0, which the probe's polling slows, so 1-10 s - before posting again (the console came back
  // that late after a refused automatic attach; with dmactive kept, 8 of 8 came at once. probe-cdc-and-persistence
  // §7.5.1). The same frame is at stake after a host's flash, reset and detach.
  if (attached()) { phy_.write(kAbstractAuto, 0); phy_.write(kDmControl, 1); }
  halted_ = false;
  phy_.park();   // floating both wires high is how this bus is told to reset
}

bool Ch32Dm::loadRegisters(uint32_t &data0_address) {
  if (!keepGprs()) return false;
  uint32_t info = 0;
  if (!phy_.read(kDmHartInfo, info)) return false;
  data0_address = 0xe0000000u | (info & 0x7ff);
  autoOff();
  phy_.write(kData0, data0_address);     phy_.write(kCommand, 0x0023100a);  // a0 = &DATA0
  if (!waitAbstract()) return false;
  phy_.write(kData0, data0_address + 4); phy_.write(kCommand, 0x0023100b);  // a1 = &DATA1
  return waitAbstract();
}

bool Ch32Dm::readWords(uint32_t address, uint32_t *out, size_t words, uint8_t *cmderr) {
  if (!halted_ || !words) return false;
  keepMailbox();   // first: everything below goes through DATA0 / DATA1
  // Long runs of these hiccup now and then - roughly one chunk in a couple of hundred on
  // the CH32L103 jig, which is a whole-flash verify failing every few tries. The words
  // already read are then meaningless, so redo the chunk from its own bring-up rather
  // than hand the caller a plausible-looking answer (2026-09-23).
  uint8_t err = 0;
  bool done = false;
  for (int attempt = 0; attempt < 3 && !done; ++attempt) {
    uint32_t data0_address = 0;
    if (!loadRegisters(data0_address)) { relink(); continue; }
    for (size_t i = 0; i < sizeof kReader / sizeof kReader[0]; ++i) phy_.write(kProgBuf0 + i, kReader[i]);
    phy_.write(kData1, address);
    autoOn();
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
    autoOff();
    if (err) phy_.write(kAbstractCs, 0x700);
    cmderr_ = err;
    if (cmderr) *cmderr = err;
    // The reader bumps DATA1 by 4 every run, so the address it left behind counts the runs:
    // the first one plus one per DATA0 read, or one fewer when the last look-ahead faulted
    // (cmderr 3) before its store. A mangled address write, a trigger the module missed or
    // one it took twice all leave a count that is off - and all of them otherwise hand back
    // plausible words with no error. Measured on the CH32L103 through the RP2350 probe
    // (2026-09-23): a whole-flash CRC came out different about one read in three, with
    // every read reporting success.
    uint32_t next = 0;
    const uint32_t ran = address + 4u * static_cast<uint32_t>(words + (err == 3 ? 0 : 1));
    const bool counted = phy_.read(kData1, next) && next == ran;
    // cmderr 3 is the look-ahead walking off the end of a region and says nothing about
    // the words already read. Anything else means the program buffer did not run: the
    // reads then returned whatever was left in DATA0, which looks like data and is not.
    done = ok && counted && (err == 0 || err == 3);
    if (!done) relink();
  }
  restoreBlock();
  return done;
}

bool Ch32Dm::readRegister(uint16_t regno, uint32_t &value) {
  if (!halted_) return false;
  autoOff();
  phy_.write(kCommand, 0x00220000u | regno);  // aarsize=32, transfer, read
  if (!waitAbstract()) return false;
  return phy_.read(kData0, value);
}

bool Ch32Dm::writeRegister(uint16_t regno, uint32_t value) {
  if (!halted_) return false;
  autoOff();
  phy_.write(kData0, value);
  phy_.write(kCommand, 0x00230000u | regno);  // aarsize=32, transfer, write
  return waitAbstract();
}

bool Ch32Dm::writeWordsFast(uint32_t address, const uint32_t *words, size_t count) {
  if (!halted_ || !count || (address & 3)) return false;
  keepMailbox();
  // Plain memory, so a failed attempt is simply redone from the start. As with readWords, the
  // address the writer leaves in DATA1 counts the runs and catches a missed or doubled trigger.
  bool done = false;
  for (int attempt = 0; attempt < 3 && !done; ++attempt) {
    uint32_t data0_address = 0;
    if (!loadRegisters(data0_address)) { relink(); continue; }
    for (size_t i = 0; i < sizeof kBlockWriter / sizeof kBlockWriter[0]; ++i) phy_.write(kProgBuf0 + i, kBlockWriter[i]);
    phy_.write(kData1, address);
    phy_.write(kData0, words[0]);
    phy_.write(kCommand, 0x00240000);  // run the writer for word 0 (also arms autoexec's command)
    bool ok = waitAbstract();
    if (ok && count > 1) {
      autoOn();
      for (size_t i = 1; i < count; ++i) phy_.write(kData0, words[i]);
      ok = waitAbstract();
      autoOff();
    }
    uint32_t next = 0;
    done = ok && phy_.read(kData1, next) && next == address + 4u * static_cast<uint32_t>(count);
    if (!done) {
      phy_.write(kAbstractAuto, 0);
      phy_.write(kAbstractCs, 0x700);
      relink();
    }
  }
  restoreBlock();
  return done;
}

bool Ch32Dm::runUntilHalt(uint32_t pc, const uint16_t *regnos, const uint32_t *values, size_t count,
                          uint32_t timeout_ms, const uint16_t *outs, uint32_t *out_values, size_t out_count,
                          RunReport &report) {
  report = {false, false, 0, 0};
  if (!halted_) return false;
  // Without ebreakm the final ebreak traps through mtvec and the application restarts
  // (the V003 loader finding, 2026-09-22). prv = M: the hart may have been stopped in U mode
  // (ArduinoCore-CH32 sketches on V3B/V4 run there), where interrupts cannot be masked; the
  // caller masks them with mstatus in the register list. ebreaks / ebreaku are set for the run as well and put back
  // as they were once the hart has stopped (oep-if-debug §4.4: what the run leaves changed is ebreakm and prv only).
  // Without ebreaku the loader's ebreak trapped through mtvec whenever the hart still ran it in U mode - a CH32L103
  // stopped in its sketch (U mode) - and the run timed out: uploads by ch32rv through the RP2350 failed now and then
  // ("run: timeout"), where 0.0.28, which left them set, passed. Restoring them keeps the target as §4.4 says.
  constexpr uint32_t kEbreakSU = 0x3000u;   // ebreaks (13), ebreaku (12)
  uint32_t dcsr = 0;
  if (!readRegister(0x07b0, dcsr) || !writeRegister(0x07b0, dcsr | 0x8003u | kEbreakSU)) return false;
  const uint32_t ebreak_su = dcsr & kEbreakSU;
  for (size_t i = 0; i < count; ++i)
    if (!writeRegister(regnos[i], values[i])) return false;
  if (!writeRegister(0x07b1, pc)) return false;
  phy_.write(kAbstractAuto, 0);
  // A change of hart state drops the CH32L103's DMI link, so a failed read is followed by a bus bring-up. One
  // resumereq, never re-issued (oep-if-debug §4.4): a stop with dpc still at `pc` may be a run that never started
  // (the L103 did it 2 of 248 times, 2026-09-24) or one that came back - the host, which knows its code, decides.
  const uint32_t started = micros(), started_ms = millis();
  // micros() up to 4000 s (its u32 holds that many us), millis() beyond
  auto expired = [&]() {
    if (timeout_ms <= 4000000u) return micros() - started >= timeout_ms * 1000u;
    return millis() - started_ms >= timeout_ms;
  };
  phy_.write(kDmControl, 0x40000001);   // resumereq: run to the ebreak
  bool halted = false;
  while (!expired()) {
    uint32_t status = 0;
    if (!phy_.read(kDmStatus, status)) { relink(); continue; }
    if (dmHalted(status)) { halted = true; break; }
  }
  phy_.write(kDmControl, 0x80000001);   // back to haltreq | dmactive, stopped or not
  phy_.write(kAbstractCs, 0x700);
  report.elapsed_us = micros() - started;
  report.stopped = halted;
  if (halted) {
    relink();                           // the stop changed the hart's state
    halted_ = true;
  } else {
    halted_ = false;
    if (!halt()) return false;          // the timeout: the hart could not be stopped (stopped 2)
  }
  report.halted = true;
  // The caller judges success from dpc (its ebreak) and the registers it asked for; a stop somewhere else is a fault.
  // DATA0 carries them and goes back to what the hart left when it stopped (§4: as step).
  keepMailbox();
  bool ok = readRegister(0x07b1, report.dpc);
  for (size_t i = 0; i < out_count; ++i) {
    out_values[i] = 0;
    if (!readRegister(outs[i], out_values[i])) ok = false;
  }
  uint32_t now = 0;   // ebreaks / ebreaku back as they were (cause, prv: where it stopped)
  if (!readRegister(0x07b0, now) || ((now & kEbreakSU) != ebreak_su && !writeRegister(0x07b0, (now & ~kEbreakSU) | ebreak_su)))
    ok = false;
  phy_.write(kAbstractAuto, 0);
  giveMailbox();
  return ok;
}

bool Ch32Dm::ackHaveReset() {
  uint32_t status = 0;
  if (!attach() || !phy_.read(kDmStatus, status)) return false;
  if (!dmVersionKnown(status) || !(status & (3u << 18))) return false;   // anyhavereset / allhavereset of a module
  phy_.write(kDmControl, halted_ ? 0x90000001 : 0x10000001);   // ackhavereset, haltreq kept if we hold a halt
  ++restarts_;
  // The CH32L103 drops its DMI link after this write and the next read fails (2026-09-24: every attach failed
  // until the bus was brought up again here, as halt() does after a change of state).
  relink();
  return true;
}

bool Ch32Dm::resetHalt(uint32_t &dpc) {
  dpc = 0;
  cmderr_ = 0;
  if (!attach()) return false;
  if (!halted_) halt();                    // ndmreset on a running hart left it stopped oddly (2026-09-22)
  phy_.write(kAbstractAuto, 0);
  // haltreq is held through the reset, so the hart comes out of it into debug mode - provided the writes land. The
  // part leaves reset on its default clock, slower than a sketch that raised it, and at the speed attach() tuned to
  // the sketch the release was garbled and the hart ran into its image (2026-09-24, CH32X035 from a running
  // sketch: 0 of 28 at the vector; at the slowest period 28 of 28, as through a WCH-LinkE). So run the reset at
  // the slowest period and tune the link again once the hart has stopped. A DMSTATUS without version 2 or 3 is noise.
  phy_.useSafeSpeed();
  kept_ = gprs_kept_ = false;              // the target starts over: nothing of before is wanted back
  phy_.write(kDmControl, 0x80000003);      // haltreq | ndmreset | dmactive
  phy_.write(kDmControl, 0x80000001);
  // Out of reset the hart may be unavailable for a while; it should then come up halted at the reset vector.
  // The release is written again with haltreq in case the DM ignored it (it ignores DMI writes for a while after
  // ndmreset, 2026-09-22), and ndmreset is checked clear at the end.
  // The wait for the hart to halt after the release is dm_wait_ms of time per procedure (oep-if-debug §4.3).
  bool halted = false;
  const uint32_t released_at = millis();
  do {
    uint32_t status = 0;
    if (!phy_.read(kDmStatus, status) || !dmVersionKnown(status)) { relink(); continue; }
    if (status & (1u << 13)) { delayMicroseconds(250); continue; }   // anyunavail
    if (status & (1u << 9)) { halted = true; break; }
    phy_.write(kDmControl, 0x80000001);
    delayMicroseconds(250);
  } while (millis() - released_at < kDmWaitMs);
  uint32_t control = 0;
  const bool released = phy_.read(kDmControl, control) && (control & 0x3) == 0x1;
  if (halted) settleHalted(true);
  else { relink(); halted_ = false; }
  retune();                                // at the default clock this time
  if (!(released && halted)) return false;
  keepMailbox();                           // what the fresh image put there (nothing, usually) goes back after the read
  const bool ok = readRegister(0x07b1, dpc);
  giveMailbox();
  return ok;
}

bool Ch32Dm::step(uint32_t &dpc_before, uint32_t &dpc_after, bool &moved, StepEnd &end) {
  dpc_before = dpc_after = 0;
  moved = false;
  end = kStepped;
  if (!halted_) return false;
  keepMailbox();                           // the reads below go through DATA0
  uint32_t dcsr = 0;
  if (!readRegister(0x7b1, dpc_before) || !readRegister(0x7b0, dcsr)) { giveMailbox(); return false; }
  if (!writeRegister(0x7b0, dcsr | 0x4u)) { giveMailbox(); return false; }   // dcsr.step, the privilege level as it is
  phy_.write(kAbstractAuto, 0);
  giveMailbox();                           // the instruction runs on the target's own mailbox
  phy_.write(kDmControl, 0x40000001);      // resumereq, once
  // Back in debug mode by itself within dm_wait_ms of time (oep-if-debug §4.2)?
  bool halted = false;
  const uint32_t started = millis();
  do {
    uint32_t status = 0;
    if (!phy_.read(kDmStatus, status)) { relink(); continue; }
    if (dmHalted(status)) { halted = true; break; }
  } while (millis() - started < kDmWaitMs);
  if (halted) {
    phy_.write(kDmControl, 0x80000001);
    phy_.write(kAbstractCs, 0x700);
    relink();
  } else {
    // Not back: haltreq, and dm_wait_ms more (halt()). Halted by the probe, dcsr.step is cleared and DATA put back as
    // below and the answer is status state with dpc_after; still running, halt() has cleared haltreq and the answer is
    // status state with step_left - dcsr.step may still be set, the host halts the hart and clears it (§4.2).
    halted_ = false;
    if (!halt()) { end = kLeftRunning; return true; }
    end = kHaltedByProbe;
  }
  keepMailbox();                           // before the dpc / dcsr reads below overwrite them again
  const bool ok = readRegister(0x07b1, dpc_after) && writeRegister(0x07b0, dcsr & ~0x4u);
  phy_.write(kDmControl, 0x00000001);      // haltreq lowered after a step (§4); the hart stays halted
  giveMailbox();
  moved = dpc_after != dpc_before;
  return ok;
}

bool Ch32Dm::attachUnderReset(void (*hold)(void *), void (*release)(void *), void *ctx, uint32_t hold_ms,
                              uint32_t &dpc) {
  dpc = 0;
  halted_ = false;
  kept_ = gprs_kept_ = false;              // the target starts over
  hold(ctx);
  delay(hold_ms);
  // Bring the link up while the target is held (it may or may not answer yet), queue the halt request, then let
  // go and keep asking until the hart reports halted - before firmware that kills the debug pins gets that far.
  relink();
  phy_.attach();
  // The part comes out of reset on its default clock: a speed tuned earlier to a sketch's raised clock garbles the
  // halt requests below and the hart runs into its image (2026-09-24, CH32L103: 2 in 10 stopped mid-sketch after a
  // plain attach had tuned the link). Same rule as the other resets - slowest period, retune once stopped.
  phy_.useSafeSpeed();
  phy_.write(kDmControl, 0x80000001);
  release(ctx);
  bool halted = false;
  const uint32_t started = micros();
  while (!halted && micros() - started < 200000u && !phy_.pastDeadline()) {
    phy_.write(kDmControl, 0x80000001);
    uint32_t status = 0;
    if (phy_.read(kDmStatus, status)) halted = dmHalted(status);
    else relink();
  }
  if (!halted) return false;
  settleHalted(true);
  retune();
  keepMailbox();
  const bool ok = readRegister(0x07b1, dpc);
  giveMailbox();
  return ok;
}

void Ch32Dm::pulseReset(void (*hold)(void *), void (*release)(void *), void *ctx, uint32_t hold_ms) {
  halted_ = false;
  kept_ = gprs_kept_ = false;
  if (attached()) phy_.write(kDmControl, 0x00000001);   // haltreq down: the target comes out of reset running
  hold(ctx);
  delay(hold_ms);
  release(ctx);
  phy_.park();   // the bus is brought up again by the attach that follows (the part is back on its default clock)
}

}  // namespace oep
