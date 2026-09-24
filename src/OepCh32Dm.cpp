#include "OepCh32Dm.h"

namespace oep {
namespace {
constexpr uint8_t kData0 = 0x04, kData1 = 0x05, kDmControl = 0x10, kDmStatus = 0x11, kDmHartInfo = 0x12,
                  kAbstractCs = 0x16, kCommand = 0x17, kAbstractAuto = 0x18, kProgBuf0 = 0x20;
// E156 reader: lw s0,0(a1); lw s1,0(s0); addi s0,4; sw s1,0(a0); sw s0,0(a1); ebreak
constexpr uint32_t kReader[] = {0x40044180, 0xc1040411, 0x9002c180};
// Block writer (the reader turned round): lw s0,0(a1); lw s1,0(a0); sw s1,0(s0); addi s0,4; sw s0,0(a1); ebreak
constexpr uint32_t kBlockWriter[] = {0x41044180, 0x0411c004, 0x9002c180};
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

bool Ch32Dm::attach() {
  if (phy_.attached()) return true;
  if (!phy_.attach()) return false;
  // Leave the abstract-command block in a known state. A session that ended mid-sequence can leave autoexec armed
  // on DATA0 or a sticky cmderr behind (2026-09-23).
  phy_.write(kAbstractAuto, 0);
  phy_.write(kAbstractCs, 0x700);
  return true;
}

// Bring the link up again (the CH32 drops it on a change of state) and put the abstract-command block back in a
// known state: autoexec off, cmderr cleared.
void Ch32Dm::relink() {
  phy_.reinit();
  phy_.write(kAbstractAuto, 0);
  phy_.write(kAbstractCs, 0x700);
}

// Measure the link speed again (the target's clock may have changed), then put the abstract-command block back in
// a known state: the search re-syncs the bus once per candidate and leaves whatever the probes did behind.
void Ch32Dm::retune() {
  phy_.retune();
  phy_.write(kAbstractAuto, 0);
  phy_.write(kAbstractCs, 0x700);
}

// The hart has just stopped: acknowledge a pending reset (haltreq kept), clear cmderr, start the caller from a
// freshly brought-up bus (the first transaction after a change of state can be lost on this part).
void Ch32Dm::settleHalted(bool ack_reset) {
  if (ack_reset) phy_.write(kDmControl, 0x90000001);   // haltreq | ackhavereset | dmactive
  relink();
  halted_ = true;
}

bool Ch32Dm::halt() {
  if (!attach()) return false;
  if (halted_) return true;
  // One halt request is not always enough. Measured on a CH32L103 through the RP2350
  // probe (2026-09-23): DMCONTROL reads the request back as set while the hart keeps
  // running, and abstract commands fail cmderr=4; repeating it makes the halt land every
  // time. minichlink writes it three or four times in a row for the same reason, so
  // re-issue between polls instead of only polling.
  for (int round = 0; round < 8; ++round) {
    // A request that does not take leaves the bus out of step, and every attempt that
    // worked on the bench had a fresh bring-up in front of it, so start each round from
    // one (2026-09-23, CH32L103: without this, halt landed on every other attempt).
    relink();
    for (int i = 0; i < 4; ++i) phy_.write(kDmControl, 0x80000001);
    for (int i = 0; i < 25; ++i) {
      uint32_t status = 0;
      if (phy_.read(kDmStatus, status) && (status & (1u << 9))) {
        // Keep haltreq asserted while halted (E156/E157 ran this way). The hart changing state drops the DMI link
        // on this part, and the first word of the first memory read after a halt came back as the previous
        // operation's leftover (2026-09-23): settleHalted starts the caller from a freshly brought-up bus.
        settleHalted(status & (3u << 18));
        return true;
      }
    }
  }
  return false;
}

bool Ch32Dm::resume() {
  if (!attached()) return false;
  host_raw_ = false;          // the probe has the hart back
  phy_.write(kAbstractAuto, 0);
  // Same story as halt(): one request is not always enough, and the CH32L103 never raises
  // allresumeack at all (2026-09-23) - it just starts running. So repeat the request, and
  // accept either the acknowledgement or the hart plainly being back on its feet.
  bool ok = false;
  for (int round = 0; round < 8 && !ok; ++round) {
    relink();
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

Ch32Dm::ResetReport Ch32Dm::reset(bool confirm) {
  // Stop the hart at its reset vector (resetHalt: haltreq held through ndmreset, at the slowest speed, retuned once
  // stopped), then let it run. The link stays attached the whole way. The old sequence - ndmreset with the hart
  // running, then letting go of the bus and attaching again (a wake burst each time) to release and to confirm it -
  // came through first time in only 9 of 20 resets on the CH32L103 and failed 5, while reset-halt + resume ran
  // 20 of 20 on the L103, the X035 and the V003 alike (2026-09-25). Stopping at the vector first also takes care of
  // the X035's parked-at-vector resets (E158).
  host_raw_ = false;
  ResetReport report = {0, 0, 0};
  for (uint8_t attempt = 1; attempt <= 3; ++attempt) {
    report.attempts = attempt;
    if (attempt > 1) report.flags |= 4;                     // redone
    uint32_t dpc = 0;
    if (!resetHalt(dpc) || !resume()) continue;
    report.flags |= 1;                                      // released and running
    if (!confirm) return report;
    // Confirm execution: a moment for the image to start, then a brief halt for the pc. A pc still at the vector
    // is not execution yet: resume and sample again.
    for (int sample = 0; sample < 3; ++sample) {
      delay(1);
      uint32_t pc = 0;
      if (!halt()) { report.flags |= 8; break; }            // the confirmation halt failed
      const bool read = readRegister(0x7b1, pc);
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
  host_raw_ = false;
  if (attached()) { phy_.write(kAbstractAuto, 0); phy_.write(kDmControl, 0); }
  halted_ = false;
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
    if (!loadRegisters(data0_address)) { relink(); continue; }
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
    // plausible words with no error. Measured on the CH32L103 through the RP2350 probe
    // (2026-09-23): a whole-flash CRC came out different about one read in three, with
    // every read reporting success.
    uint32_t next = 0;
    const uint32_t ran = address + 4u * static_cast<uint32_t>(words + (err == 3 ? 0 : 1));
    const bool counted = phy_.read(kData1, next) && next == ran;
    // cmderr 3 is the look-ahead walking off the end of a region and says nothing about
    // the words already read. Anything else means the program buffer did not run: the
    // reads then returned whatever was left in DATA0, which looks like data and is not.
    if (ok && counted && (err == 0 || err == 3)) return true;
    relink();
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
    if (!loadRegisters(data0_address)) { relink(); continue; }
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
    relink();
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
  phy_.write(kAbstractAuto, 0);
  // The CH32L103 needs what halt()/resume() learned (2026-09-23): one resumereq does not always take and it never
  // raises allresumeack, and a change of hart state drops its DMI link, so a failed read is followed by a bus
  // bring-up. A stop with dpc still at `pc` means the code never ran - ask again (the host's code ends in an
  // ebreak somewhere else, so a real stop never sits at its first instruction). Measured on the L103 through the RP2350 probe over
  // leads, 2026-09-24: 2 of 248 runs stopped at pc without running, 1 lost the link while polling.
  const uint32_t started = micros();
  for (int attempt = 0; attempt < 4 && !report.stopped; ++attempt) {
    phy_.write(kDmControl, 0x40000001);   // resumereq: run to the ebreak
    bool halted = false;
    while (micros() - started < timeout_us) {
      uint32_t status = 0;
      if (!phy_.read(kDmStatus, status)) { relink(); continue; }
      if (status & (1u << 9)) { halted = true; break; }
    }
    phy_.write(kDmControl, 0x80000001);   // back to haltreq | dmactive, stopped or not
    phy_.write(kAbstractCs, 0x700);
    if (!halted) break;                   // the timeout: forced halt below
    relink();                        // the stop changed the hart's state
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
  relink();
  return true;
}

bool Ch32Dm::resetHalt(uint32_t &dpc) {
  dpc = 0;
  if (!attach()) return false;
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
    if (!phy_.read(kDmStatus, status) || (status & 0xf) != 2) { relink(); continue; }
    if (status & (1u << 13)) { delayMicroseconds(250); continue; }   // anyunavail
    if (status & (1u << 9)) { halted = true; break; }
    phy_.write(kDmControl, 0x80000001);
    delayMicroseconds(250);
  }
  uint32_t control = 0;
  const bool released = phy_.read(kDmControl, control) && (control & 0x3) == 0x1;
  if (halted) settleHalted(true);
  else { relink(); halted_ = false; }
  retune();                                // at the default clock this time
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
    if (!phy_.read(kDmStatus, status)) { relink(); continue; }
    if (status & (1u << 9)) { halted = true; break; }
  }
  phy_.write(kDmControl, 0x80000001);
  phy_.write(kAbstractCs, 0x700);
  relink();
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
  while (!halted && micros() - started < 200000u) {
    phy_.write(kDmControl, 0x80000001);
    uint32_t status = 0;
    if (phy_.read(kDmStatus, status)) halted = (status & (1u << 9)) != 0;
    else relink();
  }
  if (!halted) return false;
  settleHalted(true);
  retune();
  return readRegister(0x07b1, dpc);
}

}  // namespace oep
