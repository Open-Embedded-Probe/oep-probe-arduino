// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.spi-target revision 1 (oep-spec oep-if-fixture §4) on a WCH CH32's SPI as a slave (ArduinoCore-CH32RV's
// CH32RVSPISlave, 0.4.0): the same ops as the ESP32's (OepP4SpiTarget.h).
//   0x10 configure(mode u8 0-3, bit_order u8)   0x11 arm(length u16, count u16, tx)
//   0x12 read_rx -> pending(u8) bits(u32) count(u16) data   0x13 status (no lock)
// Roles: 1 SCK, 2 MOSI, 3 MISO, 4 CS. SCK / MISO / MOSI have to be one route of an SPI (configure fails otherwise: the
// core's begin() refuses the pins), CS any pin with an EXTI line. The slave sends tx and then 0 (its fill byte), lets
// MISO go while CS is high, and takes nothing up for a CS with no clock. A CS with clocks and nothing armed is the
// core's missed(): counted in transactions and errors (fixture §4).
//
// What this slave cannot tell: the bits of a byte cut short by CS - it counts whole bytes, so bits is the bytes x 8.
// Each byte is an interrupt and so is CS: the master leaves a few microseconds after CS falls (describe cs_setup_ns)
// and keeps the clock low enough (max_clock_hz: 200 kHz, what the core's self test measured on V003 / V103 / V307 /
// X035 / L103, 2026-10-28).
// Not on the X035: once a source includes the core's SPI slave, its interrupt handlers are linked (640 bytes) whether
// or not a slave is made, and the X035's flash has no room for them. OEP_HOST_FAKE_CH32_SPI_SLAVE: a host test's fake
// of the core's class (tests/host/shim).
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

#if (defined(ARDUINO_ARCH_CH32RV) && !defined(ARDUINO_CH32X035)) || defined(OEP_HOST_FAKE_CH32_SPI_SLAVE)
#include <CH32RVSPISlave.h>

namespace oep {

class Ch32SpiTarget final : public Interface {
 public:
  static constexpr uint8_t kOwnerId = 6;
  enum Role : uint8_t { kRoleSck = 1, kRoleMosi = 2, kRoleMiso = 3, kRoleCs = 4 };
  static constexpr size_t kMaxFrame = 64;
  static constexpr size_t kQueueDepth = 4;
  static constexpr uint32_t kMaxClockHz = 200000;
  static constexpr uint32_t kCsSetupNs = 5000;   // CS falling to the slave selected, its first byte on MISO
  static constexpr uint8_t kModes = 0x0f;

  enum : uint8_t {
    kOpConfigure = reg::fixture_spi_target::kOpConfigure, kOpArm = reg::fixture_spi_target::kOpArm,
    kOpReadRx = reg::fixture_spi_target::kOpReadRx, kOpStatus = reg::fixture_spi_target::kOpStatus,
  };

  Ch32SpiTarget(PinTable &pins, uint16_t instance = 0) : pins_(pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_spi_target::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_spi_target::kRevision; }
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
  void service();   // from loop(): the finished transfers into the queue, the unarmed ones counted

#if defined(OEP_HOST_FAKE_CH32_SPI_SLAVE)
  CH32RVSPISlave &hostSlave() { return slave_; }
#endif

 private:
  PinTable &pins_;
  uint16_t instance_;
  uint8_t refusal_cause_ = reg::core::kUnavailableCauseWrongState;
  uint16_t refusal_channel_ = 0xffff;
  int sck_ = -1, mosi_ = -1, miso_ = -1, cs_ = -1;
  uint8_t mode_ = 0, bit_order_ = 0;
  bool started_ = false, armed_ = false;
  uint32_t transactions_ = 0, errors_ = 0;
  uint32_t missed_ = 0;   // the slave's missed() already counted
  CH32RVSPISlave slave_;
  uint8_t tx_[kMaxFrame] = {};
  uint8_t rx_[kMaxFrame] = {};
  size_t armed_length_ = 0;
  uint8_t queue_[kQueueDepth][kMaxFrame];
  uint8_t queue_length_[kQueueDepth] = {};
  uint32_t queue_bits_[kQueueDepth] = {};
  uint8_t queue_count_ = 0;
  bool start();
  void stop();
  void count(uint32_t n, bool error);   // transactions (and errors) up by n, both stopping at 0xFFFFFFFF
};

}  // namespace oep

#endif
