// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// io.github.ch32-riscv-ug.esp32.spi-target revision 1 (a custom OEP v1 interface): an ESP32 hardware SPI target on
// the ESP-IDF spi_slave driver (SPI2_HOST, no DMA: 64-byte FIFO transactions).
//   0x01 configure(mode u8 0-3, bit_order u8: 0 MSB first, 1 LSB first)   0x02 arm(length u16, count u16, tx)
//   0x03 read_rx -> pending(u8) bits(u32) count(u16) data   0x04 status -> flags(u8) transactions(u32) errors(u16)
//   (no lock)   0x05 reset.   Every request takes a TLV tail (oep-core §2.3). Roles: 1 SCK, 2 MOSI, 3 MISO, 4 CS.
// One CS-framed transaction is armed at a time with the MISO bytes to send;
// after the master raises CS the result (MOSI bytes, length in bits) is queued
// for read_rx. Polled from service() in loop(); nothing runs in an ISR.
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

#if defined(ARDUINO_ARCH_ESP32)
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
  const char *name() const override { return "io.github.ch32-riscv-ug.esp32.spi-target"; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return 1; }
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
  uint16_t errors_ = 0;
  // the one in-flight transaction (word aligned for the driver)
  alignas(4) uint8_t tx_buffer_[kMaxFrame];
  alignas(4) uint8_t rx_buffer_[kMaxFrame];
  size_t armed_length_ = 0;
  // finished transactions, oldest first
  uint8_t queue_[kQueueDepth][kMaxFrame];
  uint8_t queue_length_[kQueueDepth] = {};
  uint32_t queue_bits_[kQueueDepth] = {};
  uint8_t queue_count_ = 0;
#if defined(ARDUINO_ARCH_ESP32)
  spi_slave_transaction_t trans_ = {};
#endif
  bool start();
  void stop();
  bool arm(const uint8_t *tx, size_t tx_length, size_t length);
};

}  // namespace oep
