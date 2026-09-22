// Vendor tool (owner 0x0100, id 0x0002): ESP32-P4 hardware SPI target on the
// ESP-IDF spi_slave driver (SPI2_HOST, no DMA: 64-byte FIFO transactions).
// One CS-framed transaction is armed at a time with the MISO bytes to send;
// after the master raises CS the result (MOSI bytes, length in bits) is queued
// for read_rx. Polled from service() in loop(); nothing runs in an ISR.
#pragma once

#include <Arduino.h>

#include "OepFixtureServices.h"
#include "OepService.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/spi_slave.h>
#endif

namespace oep {

class P4SpiTarget final : public Service {
 public:
  static constexpr uint8_t kOwnerId = 6;
  enum Role : uint8_t { kRoleSck = 1, kRoleMosi = 2, kRoleMiso = 3, kRoleCs = 4 };
  static constexpr size_t kMaxFrame = 64;
  static constexpr size_t kQueueDepth = 4;

  explicit P4SpiTarget(PinTable &pins) : pins_(pins) {}
  uint16_t owner() const override { return OEP_V0_DEF_P4_SPI_TARGET_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_P4_SPI_TARGET_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_P4_SPI_TARGET_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t first, uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void service();  // call from loop(): collects the finished transaction into the queue

 private:
  PinTable &pins_;
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
