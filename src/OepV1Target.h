// OEP v1 target interfaces over Ch32Dm (oep-spec docs/v1-core-wire-delta.ja.md §5.4 / §5.5, revision 1):
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
#include "OepV1.h"

namespace oep {
namespace v1 {

// What both interfaces share: the one debug connection of this wire.
struct DebugPort {
  Ch32Dm &dm;
  uint16_t swdio, swclk;   // probe channels, for scan results and the describe pin set (swclk 0xffff: one wire)
  bool connected = false;
  uint32_t resets = 0;     // resets issued through riscv-dm (the console marks them)
  // The target's reset line for attach-under-reset: the host names the channel (it can find it by pulsing
  // candidates and watching havereset); a probe with fixed wiring may offer a default. Only channels in
  // reset_allowed may be pulled (the same idea as the scan allow-list: never drive a pin the jig did not clear).
  int16_t reset_default = -1;
  uint64_t reset_allowed = 0;
};

class WireRvswd final : public Interface {
 public:
  enum : uint8_t {
    kOpScan = reg::wire_rvswd::kOpScan, kOpAttach = reg::wire_rvswd::kOpAttach, kOpDetach = reg::wire_rvswd::kOpDetach,
    kOpAttachUnderReset = reg::wire_rvswd::kOpAttachUnderReset,
  };
  WireRvswd(DebugPort &port, uint16_t instance, const char *name = reg::wire_rvswd::kName)
      : port_(port), instance_(instance), name_(name) {}
  const char *name() const override { return name_; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::wire_rvswd::kRevision; }   // oep.wire.swio: the same (1)
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

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

}  // namespace v1
}  // namespace oep
