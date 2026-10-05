// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 ARM SWD interfaces (oep-spec docs/oep-if-debug.ja.md §1, §5-§6, revision 1):
//
//   oep.wire.swd         scan / attach / detach / connections on the probe's SWD pair (fixed or host-chosen); attach
//                        (method 0 only) wakes the port (JTAG-to-SWD, then the dormant wake: flags bit2), sends
//                        TARGETSEL when the host gives one, and returns DPIDR; attaching an attached port hands its
//                        connection back (flags bit1). The wake is tried again within wire_retry_ms (search_retries
//                        counts the failed tries), and a transfer that got nothing back is tried again after the line
//                        reset / dormant wake (oep-if-debug §2, §5). The reset TLV is not offered (rejected unsupported). The
//                        connections entry's tid is scheme 2 = TARGETSEL (0 when none).
//   oep.target.arm-adi   ADI (v5 / v6) access on that connection: a list of raw DP / AP transfers (done, status, the last
//                        ACK, nvals, values), and MEM-AP block reads / writes through TAR / DRW
//
// Like the RISC-V side, the probe knows nothing about the target: power-up requests, SELECT, CSW, the AP layout, the
// Cortex-M debug registers are all the host's. RP2040 / RP2350 SIO bit-bang only for now.
#pragma once

#if defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_SWD)

#if defined(OEP_HOST_FAKE_SWD)
#include <fake_swd_io.h>   // host tests (tests/host): rp2::BitBang over a simulated SWD target
#else
#include "OepRp2BitBang.h"
#endif
#include "Oep.h"
#include "OepDebug.h"
#include "OepPinTable.h"
#include "OepWireLoss.h"

namespace oep {

struct SwdPort {
  uint16_t swdio, swclk;       // probe channels (GPIO numbers)
  uint32_t half_ns = 500;      // SWCLK half period (the fastest this probe uses)
  bool connected = false;
  uint16_t number = 0;         // the live connection's number, from the probe's one space (core §9, ResourceNumbers)
  rp2::BitBang io{};
  uint32_t active_half_ns = 0; // the half period of the live connection (half_ns, or slower for a max_speed)
  bool active_targetsel = false;   // the live connection's TARGETSEL (part of its identity, oep-if-debug §5)
  uint32_t targetsel = 0;
  // Host-chosen pins (oep-if-debug §1), as DebugPort: the channels SWDIO / SWCLK may take (role_channels); 0 = the fixed
  // pair above. swdio / swclk are then the pair the link is on, held in `pins` under pin_owner while it is live.
  uint64_t pin_choice = 0;
  PinTable *pins = nullptr;
  uint8_t pin_owner = 0xf1;
  WireLossClock loss{};        // oep-if-debug §2: the live connection closes only once the wire is lost
  bool io_driven = false;      // the lines are driven (from a wake on, until they are let go)
  bool rest_free = false;      // oep-if-debug §2: no answer since the last exchange - the lines free between exchanges
};

// One request's wire retries (oep-if-debug §2): at most wire_retry_ms of it goes to retrying the wire. fits: one more
// round that takes about cost_us still ends inside it.
struct WireRetry {
  uint32_t spent_us = 0;
  bool fits(uint32_t cost_us) const { return spent_us + cost_us <= reg::kLimitWireRetryMs * 1000u; }
};

// The live connection goes: pins released (Hi-Z), its number closed, let go of in the pin table.
void closePort(SwdPort &port);

class WireSwd final : public Interface {
 public:
  enum : uint8_t { kOpScan = reg::wire_swd::kOpScan, kOpAttach = reg::wire_swd::kOpAttach,
                   kOpDetach = reg::wire_swd::kOpDetach, kOpConnections = reg::wire_swd::kOpConnections };
  WireSwd(SwdPort &port, uint16_t instance) : port_(port), instance_(instance) {}
  const char *name() const override { return reg::wire_swd::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::wire_swd::kRevision; }
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::wire_swd::kLockFreeOps, op); }
  // scan, attach, detach, connections: required on every wire (oep-if-debug §0)
  bool offers(uint8_t op) const override {
    return opIn(op, reg::wire_swd::kOpScan, reg::wire_swd::kOpDetach) || op == reg::wire_swd::kOpConnections;
  }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  // core §9 / oep-if-debug §2: a host that fell away (its lease lapsed) keeps nothing open; the host is this link's only user
  void sessionLapsed() override {
    if (port_.connected) close();
  }

 private:
  bool wake(const uint32_t *targetsel, uint32_t half_ns, uint32_t &dpidr, bool &dormant);
  bool xferDpidr(uint32_t &dpidr);   // one DPIDR read on the current, live port
  void close();                      // the live connection goes: pins released (Hi-Z), let go of in the pin table
  bool allowed(uint16_t swdio, uint16_t swclk) const;
  bool free(uint16_t swdio, uint16_t swclk) const;   // nothing but this link's live connection on that pair holds them
  // unavailable for a pair free() refuses: cause 1, the held channel and its holder_kind (core §4.3)
  Result heldRefusal(uint16_t swdio, uint16_t swclk, uint8_t *out, size_t capacity) const;
  uint16_t disabledOf(uint16_t swdio, uint16_t swclk) const;   // a channel the settings disable, or 0xFFFF
  // a channel with an idle item in the settings (outputs: an output idle only), or 0xFFFF (oep-if-debug §1)
  uint16_t idleOf(uint16_t swdio, uint16_t swclk, bool outputs) const;
  bool move(uint16_t swdio, uint16_t swclk);          // the link to that pair (no live connection)
  SwdPort &port_;
  uint16_t instance_;
};

class TargetArmAdi final : public Interface {
 public:
  enum : uint8_t { kOpTransfer = reg::target_arm_adi::kOpTransfer, kOpReadBlock = reg::target_arm_adi::kOpReadBlock,
                   kOpWriteBlock = reg::target_arm_adi::kOpWriteBlock };
  TargetArmAdi(SwdPort &port, uint16_t instance) : port_(port), instance_(instance) {}
  const char *name() const override { return reg::target_arm_adi::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::target_arm_adi::kRevision; }
  // transfer, read_block, write_block: all required, no features (oep-if-debug §5)
  bool offers(uint8_t op) const override { return opIn(op, reg::target_arm_adi::kOpTransfer, reg::target_arm_adi::kOpWriteBlock); }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  void setFrameLimit(size_t max_frame) override { max_frame_ = max_frame; }

 private:
  // with WAIT retries and the request's wire retries (retry_); returns the last ACK
  uint8_t xfer(bool ap, bool read, uint8_t a23, uint32_t &data);
  WireRetry retry_;
  // describe's max_length (oep-if-debug §6): bytes of one block op that fit the frame (the words go straight
  // between the frame and the line, no buffer of this interface's own)
  uint16_t maxLength() const { return blockMaxLength(max_frame_, 0); }
  SwdPort &port_;
  uint16_t instance_;
  size_t max_frame_ = 0;
};

}  // namespace oep

#endif
