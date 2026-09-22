// Vendor tool (owner 0x0100, id 0x0001): ESP32-P4 hardware I2C target on the
// ESP-IDF slave v1 driver. Contract measured in E147-E150: a receive job is
// armed with the exact transaction length; framed mode takes a 1-byte length
// header transaction then a payload transaction; preloaded TX slots need one
// filler byte after the payload. The ISR only flags completion; re-arming,
// queueing and TX preloads happen in service() from loop().
#pragma once

#include <Arduino.h>

#include "OepFixtureServices.h"
#include "OepService.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/i2c_slave.h>
#endif

namespace oep {

class P4I2cTarget final : public Service {
 public:
  static constexpr uint8_t kOwnerId = 3;
  enum Mode : uint8_t { kModeNone = 0, kModeFixedRx = 1, kModeFramedRx = 2, kModePreloadedTx = 3 };
  enum Role : uint8_t { kRoleSda = 1, kRoleScl = 2 };
  static constexpr size_t kMaxFrame = 128;
  static constexpr size_t kQueueDepth = 4;

  explicit P4I2cTarget(PinTable &pins) : pins_(pins) {}
  uint16_t owner() const override { return OEP_V0_DEF_P4_I2C_TARGET_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_P4_I2C_TARGET_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_P4_I2C_TARGET_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t first, uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void service();  // call from loop(): drains the ISR flag, queues frames, re-arms

 private:
  PinTable &pins_;
  int sda_ = -1, scl_ = -1;
  uint8_t address_ = 0, mode_ = kModeNone;
  bool started_ = false;
  uint32_t rx_frames_ = 0;
  uint16_t errors_ = 0;
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
  bool start();
  void stop();
  bool arm(size_t length);
  void pushFrame(const uint8_t *data, size_t length);
};

}  // namespace oep
