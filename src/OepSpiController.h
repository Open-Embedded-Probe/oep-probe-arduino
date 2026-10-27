// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.spi-controller revision 1 (oep-spec oep-if-fixture §6): the probe as the SPI controller, for testing a
// DUT's SPI target.
//   0x10 configure(mode u8 0-3, bit_order u8, clock_hz u32) -> clock_hz(u32): the SCK used
//   0x11 transfer(n u8, n x (gap_ns u32, setup_ns u32, hold_ns u32, count u16, tx))
//        -> n x (gap_ns u32, setup_ns u32, hold_ns u32, count u16, rx): the times kept
//   0x12 status -> state(u8) mode(u8) bit_order(u8) transfers(u32)   (no lock)
// Every request takes a TLV tail (oep-core §2.3). Roles: 1 SCK, 2 MOSI, 3 MISO, 4 CS (low active). The transfers of one
// request run back to back, CS's gap / setup / hold each at least the time asked: the host's round trip never sits
// between them, so a target's boundaries (CS to the first clock, CS to CS) are measured in one request.
//
// The lines are GPIOs, a transfer at a time (lineTransfer). On an ESP32 the GPIO registers, every edge timed on the CPU's
// cycle counter from the edge before: a time is whole cycles at or above the one asked, and the value answered is
// those cycles in ns, rounded down (an interrupt only makes a time longer, which the answer allows: at least what it
// says). An SCK half an interrupt stretched is longer too (the clock is the controller's). OEP_HOST_FAKE_SPI_BUS: a host
// test's target answers the transfers (SpiHostBus).
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

namespace oep {

#if defined(OEP_HOST_FAKE_SPI_BUS)
// The host test's bus: one CS-framed transfer, its times in CPU cycles (the gap unused on the request's first)
struct SpiHostBus {
  virtual void transfer(bool first, uint32_t gap, uint32_t setup, uint32_t hold, const uint8_t *tx, uint8_t *rx,
                        size_t count) = 0;
};
#endif

class SpiController final : public Interface {
 public:
  static constexpr uint8_t kOwnerId = 9;
  enum Role : uint8_t { kRoleSck = 1, kRoleMosi = 2, kRoleMiso = 3, kRoleCs = 4 };
  static constexpr size_t kMaxLength = 256;   // the bytes of one request's transfers
  static constexpr uint32_t kMaxClockHz = 2000000;
  static constexpr uint32_t kMinClockHz = 1000;
  static constexpr uint32_t kMaxDelayNs = 100000000;   // 100 ms: gap, setup or hold, well inside max_op_ms
  static constexpr uint8_t kModes = 0x0f;   // all four

  enum : uint8_t {
    kOpConfigure = reg::fixture_spi_controller::kOpConfigure, kOpTransfer = reg::fixture_spi_controller::kOpTransfer,
    kOpStatus = reg::fixture_spi_controller::kOpStatus,
  };

  SpiController(PinTable &pins, uint16_t instance = 0) : pins_(pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_spi_controller::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_spi_controller::kRevision; }
  bool lockFree(uint8_t op) const override { return op == kOpStatus; }
  bool offers(uint8_t op) const override { return opIn(op, kOpConfigure, kOpStatus); }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t *out, size_t capacity) override;
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  uint8_t planRefusalCause() const override { return refusal_cause_; }
  void planRefusalDetail(uint16_t &channel) const override { channel = refusal_channel_; }
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;

#if defined(OEP_HOST_FAKE_SPI_BUS)
  static inline SpiHostBus *host_bus = nullptr;
  static inline bool fake_fail_start = false;   // host tests: the next start fails
#endif

 private:
  PinTable &pins_;
  uint16_t instance_;
  uint8_t refusal_cause_ = reg::core::kUnavailableCauseWrongState;   // the last planCheck's unavailable (core §4.3)
  uint16_t refusal_channel_ = 0xffff;
  int sck_ = -1, mosi_ = -1, miso_ = -1, cs_ = -1;
  uint8_t mode_ = 0, bit_order_ = 0;
  bool started_ = false;
  uint32_t half_ = 0;   // a half SCK period, CPU cycles
  uint32_t transfers_ = 0;
#if defined(ARDUINO_ARCH_ESP32)
  int sck_pad_ = -1, mosi_pad_ = -1, miso_pad_ = -1, cs_pad_ = -1;
  uint32_t edge_ = 0;   // the cycle count at the last edge
  void waitFor(uint32_t cycles);
  void set(int pad, bool high) const;
  bool miso() const;
#endif
  bool start();
  void stop();
  void lineTransfer(bool first, uint32_t gap, uint32_t setup, uint32_t hold, const uint8_t *tx, uint8_t *rx, size_t count);
  Result transfer(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
};

}  // namespace oep
