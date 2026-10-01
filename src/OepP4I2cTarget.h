// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.i2c-target revision 1 (oep-spec oep-if-fixture §3), on an ESP32's hardware I2C target (the ESP-IDF
// slave v1 driver), for testing a DUT's I2C controller.
//   0x01 configure(address u8, mode u8: 1 fixed rx, 2 framed rx, 3 preloaded tx)   0x02 arm_rx(length u16)
//   0x03 read_rx -> pending(u8) count(u16) data          0x04 preload_tx(count u16, data) -> slots(u8)
//   0x05 status -> state(u8) mode(u8) armed(u8) queued(u8) rx_frames(u32) tx_slots(u8) errors(u32)   (no lock)
//   0x06 reset   0x07 stretch(stretch_us u32)
// Every request takes a TLV tail (oep-core §2.3). Roles: 1 SDA, 2 SCL.
// Contract measured in E147-E150 (wch-protocols): Contract measured in E147-E150: a receive job is
// armed with the exact transaction length; framed mode takes a 1-byte length
// header transaction then a payload transaction; preloaded TX slots need one
// filler byte after the payload. The ISR only flags completion; re-arming,
// queueing and TX preloads happen in service() from loop().
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/i2c_slave.h>
#endif

namespace oep {

class P4I2cTarget final : public Interface {
 public:
  static constexpr uint8_t kOwnerId = 3;
  enum Mode : uint8_t { kModeNone = 0, kModeFixedRx = 1, kModeFramedRx = 2, kModePreloadedTx = 3 };
  enum Role : uint8_t { kRoleSda = 1, kRoleScl = 2 };
  static constexpr size_t kMaxFrame = 128;
  static constexpr size_t kQueueDepth = 4;

  enum : uint8_t { kOpConfigure = 0x01, kOpArmRx = 0x02, kOpReadRx = 0x03, kOpPreloadTx = 0x04, kOpStatus = 0x05,
                   kOpReset = 0x06, kOpStretch = 0x07 };

  P4I2cTarget(PinTable &pins, uint16_t instance = 0) : pins_(pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_i2c_target::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_i2c_target::kRevision; }
  bool lockFree(uint8_t op) const override { return op == kOpStatus; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void service();  // call from loop(): drains the ISR flag, queues frames, re-arms

 private:
  PinTable &pins_;
  uint16_t instance_;
  int sda_ = -1, scl_ = -1;
  uint8_t address_ = 0, mode_ = kModeNone;
  bool started_ = false;
  uint32_t rx_frames_ = 0;
  uint32_t errors_ = 0;
  uint8_t tx_slots_ = 0;
  // receive job
  uint8_t rx_buffer_[kMaxFrame];
  size_t armed_ = 0;
  bool framed_header_ = true;
  volatile bool rx_done_ = false;
  // completed frames, oldest first
  uint8_t queue_[kQueueDepth][kMaxFrame];
  uint8_t queue_length_[kQueueDepth] = {};
  uint8_t queue_count_ = 0;
#if defined(ARDUINO_ARCH_ESP32)
  i2c_slave_dev_handle_t slave_ = nullptr;
  static bool receiveDone(i2c_slave_dev_handle_t, const i2c_slave_rx_done_event_data_t *, void *context);
#endif
  // set_stretch: hold every hardware stretch (address match on read / TX empty / RX full) this long from service(), 0 = off
  uint32_t stretch_us_ = 0;
  volatile uint32_t stretch_events_ = 0;
  bool start();
  void stop();
  bool arm(size_t length);
  void pushFrame(const uint8_t *data, size_t length);
};

}  // namespace oep
