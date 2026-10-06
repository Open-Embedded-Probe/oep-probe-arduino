// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 target interfaces over Ch32Dm (oep-spec docs/oep-if-debug.ja.md §1-§4, revision 1):
//
//   oep.wire.rvswd       scan / attach / detach / connections on the probe's RVSWD pair (fixed, or host-chosen); the
//                        connection takes a number from the probe's one space, and attaching an attached wire hands it
//                        back (flags bit1). attach's reset TLV holds the target's reset line first (method 1: stopped
//                        before its first instruction; method 0: left running)
//   oep.wire.swio        the same on a single SWIO wire (CH32V003); one class, told which it is
//   oep.target.riscv-dm  RISC-V Debug Module over DMI on that connection: a DMI step list plus the parts that
//                        make host-driven flashing fast (block read/write through autoexec, run until halt,
//                        halt / resume with the CH32 re-issue and bus bring-up, reset, step). Every op puts back what it
//                        changed in the target before it answers (oep-if-debug §4).
//
// Failures that happened while executing are completed failed / partial with the success-shaped payload and a
// status byte (ok / wait / line / fault / timeout / state); rejected is kept for requests not accepted. A request that
// gets nothing back from the wire within its wire_retry_ms answers status line and keeps the connection; the
// connection closes (after that answer) only when the wire has failed for wire_lost_ms of real time with no good
// exchange between (§2: the PHY's wire-loss clock, which the console's reads and a slot's liveness check share).
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
  // Host resets of the target that the console marks and last-reset follows: riscv-dm's reset and attach's reset TLV.
  // reset_detail: how the last one was done (mark_detail_reset: 1 ndmreset, 3 attach's reset TLV).
  uint32_t resets = 0;
  uint8_t reset_detail = 0;
  // Times the connection closed, and whether the last close was the line lost (the console marks link-lost, then
  // closed 4), a host's detach with force (detach, then closed 4: oep-if-debug §2's table) or another detach / release
  // (closed 4).
  uint32_t closes = 0;
  bool lost = false;
  bool detached = false;
  // The target's reset line (attach's reset TLV): the host names the channel every time - there is no default
  // (oep-if-debug §3). Only channels in reset_allowed may be pulled (describe role_channels, role reset), and not one
  // another interface holds in `pins`.
  uint64_t reset_allowed = 0;
  PinTable *pins = nullptr;
  // Host-chosen pins (oep-if-debug §1): the channels this wire may take as SWDIO / SWCLK, declared as role_channels;
  // 0 = the one fixed pair in swdio / swclk (a channel group). swdio / swclk are then the pair the link is on now, and a
  // live connection holds them in `pins` under pin_owner (core §8.1). A one-wire link keeps swclk 0xffff.
  uint64_t pin_choice = 0;
  uint8_t pin_owner = 0xf0;
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
  // The connection's number: from the probe's one space (core §9, ResourceNumbers), taken when a connection comes up,
  // closed when it goes. A host holding an old number gets no_connection.
  uint16_t number = 0;
};

// The reset line of a probe's own attach (oep-if-probe-config §3.1, the retry with reset): pulled low for hold_ms and
// released before the attach, as attach's reset TLV with method 0 (oep-if-debug §3). Not a host reset: `resets` is not
// counted, so a last-reset bind keeps its selection (probe.config §3.1 / §1.2). held_at_ns: when the pull started.
struct AttachReset {
  uint16_t channel;
  uint16_t hold_ms;
  uint64_t held_at_ns;
};
// Attach without stopping the hart (method 0), for a probe's own use (a bind's automatic attach): the same as the
// host's attach, havereset acknowledged. Joins an existing connection. Adds `user`. false: the target did not answer
// (no_answer set: the wire got nothing back - status line), or no connection number was left.
// A new connection takes the line settings given (oep-if-debug §3: a slot's max_speed / idle_clock); an existing one keeps
// its own. `reset`: a new connection pulls that reset line first (an existing one is joined without it).
bool attachRunning(DebugPort &port, uint8_t user, uint32_t &dmstatus, uint32_t max_hz = 0, bool idle_low = false,
                   AttachReset *reset = nullptr, bool *no_answer = nullptr);
// The attach result's target_id TLV (oep-if-debug §1) into out: its length, 0 when the target gives none.
size_t targetId(DebugPort &port, uint8_t *out, size_t room);
// Drop `user`'s use; the link is closed when nobody is left (or `force`). lost: the line was found gone (the console
// marks link-lost before closed); detached: a host's detach with its force TLV (the console marks detach before closed).
void releaseConnection(DebugPort &port, uint8_t user, bool force, bool lost = false, bool detached = false);
// No connection holds the wire's pins (closed, a scan's try, a failed attach): to their free state (oep-core §8).
void freeWire(DebugPort &port);
// An at-boot slot's liveness check (oep-if-probe-config §3.1): DMSTATUS read once; false = the wire is lost (no good
// exchange for wire_lost_ms, oep-if-debug §2) and the connection was closed. A single failed check keeps it.
bool checkConnection(DebugPort &port);
// Host-chosen pins (oep-if-debug §1). pairAllowed: a pair this wire may use at all; pairFree: none of its channels held
// by anything but this wire's live connection on that pair; usePair: move the link there (no live connection; the pair
// allowed and free) - the same pair is always fine; holdPins: the live connection takes its pins (after it came up).
bool pairAllowed(const DebugPort &port, uint16_t swdio, uint16_t swclk);
bool pairFree(const DebugPort &port, uint16_t swdio, uint16_t swclk);
bool usePair(DebugPort &port, uint16_t swdio, uint16_t swclk);
// The first channel of the pair held by anything but this wire's own connection (0xFFFF: none), and the
// unavailable refusal for it (core §4.3: cause 1, the channel, its holder_kind).
uint16_t pairHeld(const DebugPort &port, uint16_t swdio, uint16_t swclk);
Result pairHeldRefusal(const DebugPort &port, uint16_t swdio, uint16_t swclk, uint8_t *out, size_t capacity);
void holdPins(DebugPort &port);
// The connections answer (oep-if-debug §2.1: first(u8) -> more(u8) count(u8) count x entry) for a wire with
// this one place.
Result connectionsOf(DebugPort &port, uint8_t first, uint32_t speed_hz, uint8_t *out, size_t capacity);

class WireRvswd final : public Interface {
 public:
  enum : uint8_t {
    kOpScan = reg::wire_rvswd::kOpScan, kOpAttach = reg::wire_rvswd::kOpAttach, kOpDetach = reg::wire_rvswd::kOpDetach,
    kOpConnections = reg::wire_rvswd::kOpConnections,
  };
  WireRvswd(DebugPort &port, uint16_t instance, const char *name = reg::wire_rvswd::kName)
      : port_(port), instance_(instance), name_(name) {}
  const char *name() const override { return name_; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::wire_rvswd::kRevision; }   // oep.wire.swio: the same (1)
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::wire_rvswd::kLockFreeOps, op); }
  // scan, attach, detach, connections: required on every wire (oep-if-debug §0)
  bool offers(uint8_t op) const override {
    return opIn(op, reg::wire_rvswd::kOpScan, reg::wire_rvswd::kOpDetach) || op == reg::wire_rvswd::kOpConnections;
  }
  size_t describe(uint8_t *out, size_t capacity) override;
  DebugPort &port() const { return port_; }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  void sessionOver() override { releaseConnection(port_, DebugPort::kUserHost, false); }
  // oep.probe.restart (oep-if-restart §2): the connection closes whoever uses it (a slot too); the target is left as it is
  void probeRestart() override { releaseConnection(port_, 0xff, true); }

 private:
  DebugPort &port_;
  uint16_t instance_;
  const char *name_;
  bool isRvswd() const { return strcmp(name_, reg::wire_rvswd::kName) == 0; }
  struct PinRefusal { uint8_t cause; uint16_t channel; uint8_t holder_kind; };   // an unavailable's payload (core §4.3)
  uint8_t choosePair(const uint8_t *pins, size_t len, PinRefusal &why);
  Result scan(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result attach(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
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
  TargetRiscvDm(DebugPort &port, uint16_t instance) : instance_(instance) { ports_[0] = &port; }
  // Another wire's port (a probe with two wires, e.g. oep.wire.rvswd and oep.wire.swio): this one interface serves the
  // connections of both - a request goes to the wire whose live connection it names (oep-if-debug: oep.wire.* creates
  // connections, oep.target.* works over one).
  void addPort(DebugPort &port) { ports_[1] = &port; }
  const char *name() const override { return reg::target_riscv_dm::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::target_riscv_dm::kRevision; }
  // dmi, halt, resume always; reset, read_block / write_block (a pair), run and step are optional (oep-if-debug §4) -
  // all offered unless offerOptional(false) leaves them out: the ops tag declares what offers says (core §1.2)
  bool offers(uint8_t op) const override {
    return opIn(op, reg::target_riscv_dm::kOpDmi, reg::target_riscv_dm::kOpResume) ||
           (optional_ && opIn(op, reg::target_riscv_dm::kOpReset, reg::target_riscv_dm::kOpStep));
  }
  // A probe without the optional ops (a smaller build): they answer unknown_operation and are not in the ops tag.
  void offerOptional(bool on) { optional_ = on; }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  void setFrameLimit(size_t max_frame) override { max_frame_ = max_frame; }

 private:
  static constexpr size_t kMaxRegs = 16;
  size_t max_frame_ = 0;
  bool optional_ = true;
  // describe's max_length (oep-if-debug §4.5): bytes of one block op that fit the frame and the word buffer
  uint16_t maxLength() const { return blockMaxLength(max_frame_, sizeof words_); }
  Result dispatch(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity);
  Result dmi(const uint8_t *p, size_t length, uint8_t *out, size_t capacity);
  uint8_t failure(uint8_t otherwise);   // line when the link does not answer, else `otherwise`
  static Result asLine(uint8_t op, uint8_t *out, const Result &r);   // a completed answer as status line, form kept
  bool line_lost_ = false;              // set by failure() once the wire is lost (§2): the answer goes out, then the close
  DebugPort *ports_[2] = {};
  DebugPort *port_ = nullptr;           // the port of the request being handled (chosen in handle)
  uint16_t instance_;
  uint32_t words_[256];
};

}  // namespace oep
