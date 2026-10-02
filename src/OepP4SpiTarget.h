// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.spi-target revision 1 (oep-spec oep-if-fixture §4), on an ESP32's hardware SPI target (the ESP-IDF
// spi_slave driver, SPI2_HOST, no DMA: 64-byte FIFO transactions).
//   0x01 configure(mode u8 0-3, bit_order u8: 0 MSB first, 1 LSB first)   0x02 arm(length u16, count u16, tx)
//   0x03 read_rx -> pending(u8) bits(u32) count(u16) data
//   0x04 status -> state mode bit_order armed queued (u8 each) transactions(u32) errors(u32) (no lock)   0x05 reset.   Every request takes a TLV tail (oep-core §2.3). Roles: 1 SCK, 2 MOSI, 3 MISO, 4 CS.
// One CS-framed transaction is armed at a time with the MISO bytes to send;
// after the master raises CS the result (MOSI bytes, length in bits) is queued
// for read_rx. Polled from service() in loop(); nothing runs in an ISR.
// While nothing is armed a discard transaction (MISO 0, MOSI to a scratch buffer) waits in the driver, so a transfer
// the host did not arm is seen and counted in transactions and errors (fixture §4); arm replaces it (the driver cannot
// take a queued transaction back: arm restarts the target). A CS frame with no SCK edge (0 bits) is no transfer: it
// counts nothing and leaves the arm waiting (fixture §4).
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

// The ESP-IDF spi_slave driver; OEP_HOST_FAKE_SPI_SLAVE: a host test's fake of it (tests/host/shim)
#if defined(ARDUINO_ARCH_ESP32) || defined(OEP_HOST_FAKE_SPI_SLAVE)
#define OEP_SPI_SLAVE_DRIVER 1
#include <driver/spi_slave.h>
#endif

namespace oep {

class P4SpiTarget final : public Interface {
 public:
  static constexpr uint8_t kOwnerId = 6;
  enum Role : uint8_t { kRoleSck = 1, kRoleMosi = 2, kRoleMiso = 3, kRoleCs = 4 };
  static constexpr size_t kMaxFrame = 64;
  static constexpr size_t kQueueDepth = 4;

  enum : uint8_t { kOpConfigure = 0x01, kOpArm = 0x02, kOpReadRx = 0x03, kOpStatus = 0x04, kOpReset = 0x05 };

  P4SpiTarget(PinTable &pins, uint16_t instance = 0) : pins_(pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_spi_target::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_spi_target::kRevision; }
  bool lockFree(uint8_t op) const override { return op == kOpStatus; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void service();  // call from loop(): collects the finished transaction into the queue

 private:
  PinTable &pins_;
  uint16_t instance_;
  int sck_ = -1, mosi_ = -1, miso_ = -1, cs_ = -1;
  uint8_t mode_ = 0, bit_order_ = 0;
  bool started_ = false, armed_ = false;
  uint32_t transactions_ = 0;
  uint32_t errors_ = 0;
  // the one in-flight transaction (word aligned for the driver)
  alignas(4) uint8_t tx_buffer_[kMaxFrame];
  alignas(4) uint8_t rx_buffer_[kMaxFrame];
  size_t armed_length_ = 0;
  // the discard transaction while nothing is armed: MISO 0, MOSI dropped
  alignas(4) uint8_t idle_tx_[kMaxFrame] = {};
  alignas(4) uint8_t idle_rx_[kMaxFrame];
  bool idle_queued_ = false;
  // finished transactions, oldest first
  uint8_t queue_[kQueueDepth][kMaxFrame];
  uint8_t queue_length_[kQueueDepth] = {};
  uint32_t queue_bits_[kQueueDepth] = {};
  uint8_t queue_count_ = 0;
#if defined(OEP_SPI_SLAVE_DRIVER)
  spi_slave_transaction_t trans_ = {};
  spi_slave_transaction_t idle_trans_ = {};
  bool begin();        // the driver alone
  bool queueIdle();
#endif
  bool start();
  void stop();
  bool arm(const uint8_t *tx, size_t tx_length, size_t length);
};

}  // namespace oep
