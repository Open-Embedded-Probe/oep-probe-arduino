// OEP v1 draft target interfaces over Ch32Dm (oep-spec docs/capability-name-hierarchy.ja.md):
//
//   oep.wire.rvswd       scan / attach / detach on the probe's fixed RVSWD pair; attach returns connection 1
//   oep.wire.swio        the same on a single SWIO wire (CH32V003); one class, told which it is
//   oep.target.riscv-dm  RISC-V Debug Module over DMI on that connection: a DMI step list plus the parts that
//                        make host-driven flashing fast (block read/write through autoexec, run until halt,
//                        halt / resume with the CH32 re-issue and bus bring-up, ndmreset)
//
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
  enum : uint8_t { kOpScan = 0x01, kOpAttach = 0x02, kOpDetach = 0x03, kOpAttachUnderReset = 0x04 };
  WireRvswd(DebugPort &port, uint16_t instance, const char *name = "oep.wire.rvswd")
      : port_(port), instance_(instance), name_(name) {}
  const char *name() const override { return name_; }
  uint16_t instance() const override { return instance_; }
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
    kOpDmi = 0x01, kOpHalt = 0x02, kOpResume = 0x03, kOpReset = 0x04,
    kOpReadBlock = 0x05, kOpWriteBlock = 0x06, kOpRun = 0x07, kOpStep = 0x08,
  };
  enum : uint8_t { kStepWrite = 0x01, kStepRead = 0x02, kStepPoll = 0x03, kStepDelay = 0x04, kStepPollTime = 0x05 };
  enum : uint8_t { kResetRun = 0, kResetRunConfirm = 1, kResetHalt = 2 };
  TargetRiscvDm(DebugPort &port, uint16_t instance) : port_(port), instance_(instance) {}
  const char *name() const override { return "oep.target.riscv-dm"; }
  uint16_t instance() const override { return instance_; }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  Result dmi(const uint8_t *p, size_t length, uint8_t *out, size_t capacity);
  DebugPort &port_;
  uint16_t instance_;
  uint32_t words_[256];
};

}  // namespace v1
}  // namespace oep
