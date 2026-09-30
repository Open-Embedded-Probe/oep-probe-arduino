// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 target interfaces over Ch32Dm (oep-spec docs/oep-if-debug.ja.md §1-§4, revision 1):
//
//   oep.wire.rvswd       scan / attach / detach / attach_under_reset on the probe's fixed RVSWD pair; the one
//                        connection is number 1, and attaching an attached wire hands it back (flags bit1)
//   oep.wire.swio        the same on a single SWIO wire (CH32V003); one class, told which it is
//   oep.target.riscv-dm  RISC-V Debug Module over DMI on that connection: a DMI step list plus the parts that
//                        make host-driven flashing fast (block read/write through autoexec, run until halt,
//                        halt / resume with the CH32 re-issue and bus bring-up, reset, step)
//
// Failures that happened while executing are completed failed / partial with the success-shaped payload and a
// status byte (ok / wait / line / fault / timeout / state); rejected is kept for requests not accepted.
// The probe knows nothing about the target: no chip names, no flash controller. The host composes
// everything else from these (experiments/flash-primitives F4).
#pragma once

#include "OepCh32Dm.h"
#include "Oep.h"
#include "OepDebug.h"

namespace oep {

class PinTable;

// What both interfaces share: the one debug connection of this wire.
struct DebugPort {
  Ch32Dm &dm;
  uint16_t swdio, swclk;   // probe channels, for scan results and the describe pin set (swclk 0xffff: one wire)
  bool connected = false;
  uint32_t resets = 0;     // resets issued through riscv-dm (the console marks them)
  // The target's reset line for attach-under-reset: the host names the channel every time - there is no default
  // (oep-if-debug §3); it can find it by pulsing candidates and watching where the hart stops. Only channels in
  // reset_allowed may be pulled (describe role_channels, role reset), and not one another interface holds in `pins`.
  uint64_t reset_allowed = 0;
  PinTable *pins = nullptr;
  // Who uses the connection (oep-if-common §2): the host through attach, a slot (oep.probe.config) through its
  // automatic attach or the console its bind opened. A host's detach drops only its own use; the link goes when nobody
  // uses it, or on a forced detach. The bits are the connections entry's users (registry connection_users).
  enum : uint8_t { kUserHost = reg::wire_rvswd::kConnectionUsersHostSession, kUserSlot = reg::wire_rvswd::kConnectionUsersSlot };
  uint8_t users = 0;
  // The live connection's target_id as its attach read it (oep-if-debug §1; has_tid false: none).
  bool has_tid = false;
  uint32_t tid = 0;
  // The slot registered on this place (oep.probe.config), 0xff: none - the connections entry's slot.
  uint8_t slot = 0xff;
  // The connection's number (u16): a new connection takes the next, never reused within a boot, so a host holding the
  // number of an earlier one gets no_connection instead of reaching this one (core §9). One connection at a time on
  // this wire. exhausted(): every number used - a new connection is refused (unavailable).
  uint16_t number = 0;
  void numberNew() { ++number; }
  bool exhausted() const { return !connected && number == 0xffff; }
};

// Attach without stopping the hart (method 0), for a probe's own use (a bind's automatic attach): the same as the
// host's attach, havereset acknowledged. Joins an existing connection. Adds `user`. false: the target did not answer.
// A new connection takes the line settings given (oep-if-debug §3: a slot's max_speed / idle_clock); an existing one keeps
// its own.
bool attachRunning(DebugPort &port, uint8_t user, uint32_t &dmstatus, uint32_t max_hz = 0, bool idle_low = false);
// The attach result's target_id TLV (oep-if-debug §1) into out: its length, 0 when the target gives none.
size_t targetId(DebugPort &port, uint8_t *out, size_t room);
// Drop `user`'s use; the link is closed when nobody is left (or `force`).
void releaseConnection(DebugPort &port, uint8_t user, bool force);
// The connections answer (oep-if-debug §2.1) for a wire with this one place.
Result connectionsOf(DebugPort &port, uint32_t speed_hz, uint8_t *out, size_t capacity);

class WireRvswd final : public Interface {
 public:
  enum : uint8_t {
    kOpScan = reg::wire_rvswd::kOpScan, kOpAttach = reg::wire_rvswd::kOpAttach, kOpDetach = reg::wire_rvswd::kOpDetach,
    kOpAttachUnderReset = reg::wire_rvswd::kOpAttachUnderReset, kOpConnections = reg::wire_rvswd::kOpConnections,
  };
  WireRvswd(DebugPort &port, uint16_t instance, const char *name = reg::wire_rvswd::kName)
      : port_(port), instance_(instance), name_(name) {}
  const char *name() const override { return name_; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::wire_rvswd::kRevision; }   // oep.wire.swio: the same (1)
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::wire_rvswd::kLockFreeOps, op); }
  size_t describe(uint8_t *out, size_t capacity) override;
  DebugPort &port() const { return port_; }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  void sessionLapsed() override { releaseConnection(port_, DebugPort::kUserHost, false); }

 private:
  DebugPort &port_;
  uint16_t instance_;
  const char *name_;
};

class TargetRiscvDm final : public Interface {
 public:
  enum : uint8_t {
    kOpDmi = reg::target_riscv_dm::kOpDmi, kOpHalt = reg::target_riscv_dm::kOpHalt,
    kOpResume = reg::target_riscv_dm::kOpResume, kOpReset = reg::target_riscv_dm::kOpReset,
    kOpReadBlock = reg::target_riscv_dm::kOpReadBlock, kOpWriteBlock = reg::target_riscv_dm::kOpWriteBlock,
    kOpRun = reg::target_riscv_dm::kOpRun, kOpStep = reg::target_riscv_dm::kOpStep,
  };
  enum : uint8_t {
    kStepWrite = reg::target_riscv_dm::kDmiStepWrite, kStepRead = reg::target_riscv_dm::kDmiStepRead,
    kStepPoll = reg::target_riscv_dm::kDmiStepPollReads, kStepDelay = reg::target_riscv_dm::kDmiStepWaitUs,
    kStepPollTime = reg::target_riscv_dm::kDmiStepPollUs,
  };
  enum : uint8_t {
    kResetRun = reg::target_riscv_dm::kResetModeRun, kResetRunConfirm = reg::target_riscv_dm::kResetModeRunVerified,
    kResetHalt = reg::target_riscv_dm::kResetModeHaltAtReset,
  };
  TargetRiscvDm(DebugPort &port, uint16_t instance) : port_(port), instance_(instance) {}
  const char *name() const override { return reg::target_riscv_dm::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::target_riscv_dm::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  void setFrameLimit(size_t max_frame) override { max_frame_ = max_frame; }

 private:
  static constexpr size_t kMaxRegs = 16;
  size_t max_frame_ = 0;
  Result dmi(const uint8_t *p, size_t length, uint8_t *out, size_t capacity);
  uint8_t failure(uint8_t otherwise);   // line when the link does not answer, else `otherwise`
  DebugPort &port_;
  uint16_t instance_;
  uint32_t words_[256];
};

}  // namespace oep
