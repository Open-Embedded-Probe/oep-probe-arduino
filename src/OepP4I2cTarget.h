// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.i2c-target revision 1 (oep-spec oep-if-fixture §3), on an ESP32's hardware I2C target, for testing a DUT's
// I2C controller.
//   0x01 configure(address u8)   0x03 read_rx -> pending(u8) count(u16) data   0x04 preload_tx(count u16, data)
//   0x05 status -> state(u8) queued(u8) rx_frames(u32) tx_slots(u8) errors(u32)   (no lock)
//   0x07 stretch(stretch_us u32; ESP32-P4 only, declared in the ops tag)
// Every request takes a TLV tail (oep-core §2.3). Roles: 1 SDA, 2 SCL. A write with data in one transaction is one
// frame (bytes past max_length dropped, errors + 1); a read is answered from the preload slots in order, 0xFF when
// there is none.
//
// The ESP-IDF slave driver (v1, the one the arduino-esp32 libraries are built with) only sets the peripheral up: its
// receive job reports no byte count (i2c_slave_rx_done_event_data_t has only the buffer; the v2 driver that has a
// length is not compiled into the libraries) and reads the job's length from the FIFO whatever the controller sent,
// and its transmit ring is a byte stream with no slot boundaries. So the FIFOs are this class's own: a handler shared
// on the peripheral's interrupt (registered after the driver's, so it runs first and clears what it handled) reads
// every received byte of a transaction and keeps the TX FIFO topped up from the current preload slot (then 0xFF). At
// STOP the transaction's write becomes a frame (fixture §3), and a transaction that took bytes from the TX FIFO used up
// the slot. The logic past the FIFOs (onTransaction, txByte) is portable and
// host-tested through OEP_HOST_FAKE_I2C_SLAVE (tests/host).
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/i2c_slave.h>
#include <esp_intr_alloc.h>
#endif

namespace oep {

class P4I2cTarget final : public Interface {
 public:
  static constexpr uint8_t kOwnerId = 3;
  enum Role : uint8_t { kRoleSda = 1, kRoleScl = 2 };
  static constexpr size_t kMaxFrame = 128;
  static constexpr size_t kQueueDepth = 4;   // received frames kept, and unread preload slots (fixture §3)
#if defined(CONFIG_IDF_TARGET_ESP32P4) || defined(OEP_HOST_FAKE_I2C_SLAVE)
  // stretch: the P4's slave holds SCL itself (slave_scl_stretch_en, with slave_byte_ack_ctl_en a hold at every
  // received byte's ACK) and service() lets go after stretch_us, so the limit is the bench's: 0.1-20 ms measured with
  // a CH32 Wire controller (2026-09-22), 30 ms used to make it time out.
  static constexpr bool kStretch = true;
  static constexpr uint32_t kMaxStretchUs = 100000;
#else
  static constexpr bool kStretch = false;   // the classic ESP32's I2C slave cannot hold SCL
  static constexpr uint32_t kMaxStretchUs = 0;
#endif

  // start() enables the pads' internal pull-ups on SDA / SCL while configured (fixture §3: declared with features bit2).
  // The typical R_PU of both chips' datasheets is 45 kOhm: ESP32-P4 Series Datasheet v1.2 / Pre-release v0.7, Table 5-4
  // DC Characteristics (3.3 V, 25 °C), "R_PU Internal weak pull-up resistor - 45 - kOhm"; ESP32 Series Datasheet v5.3,
  // Table 5-3 DC Characteristics (3.3 V, 25 °C), "R_PU Resistance of internal pull-up resistor - 45 - kOhm" (typical
  // only; no min / max given).

  enum : uint8_t {
    kOpConfigure = reg::fixture_i2c_target::kOpConfigure, kOpReadRx = reg::fixture_i2c_target::kOpReadRx,
    kOpPreloadTx = reg::fixture_i2c_target::kOpPreloadTx, kOpStatus = reg::fixture_i2c_target::kOpStatus,
    kOpStretch = reg::fixture_i2c_target::kOpStretch,
  };

  P4I2cTarget(PinTable &pins, uint16_t instance = 0) : pins_(pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_i2c_target::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_i2c_target::kRevision; }
  bool lockFree(uint8_t op) const override { return op == kOpStatus; }
  // configure, read_rx, preload_tx, status (fixture §3); stretch (optional) only where the target can hold SCL
  bool offers(uint8_t op) const override {
    return op == kOpConfigure || opIn(op, kOpReadRx, kOpStatus) || (op == kOpStretch && kStretch);
  }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t *out, size_t capacity) override;
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void service();  // call from loop(): ends a stretch hold, reloads the TX FIFO when a preload waited for an idle bus

#if defined(OEP_HOST_FAKE_I2C_SLAVE)
  // The host test's controller: one transaction to this address - a write of `count` bytes (0: the address alone),
  // then, with `read` > 0, a repeated START and a read of `read` bytes into `got`. false: the target is not running.
  bool hostTransaction(const uint8_t *data, size_t count, size_t read = 0, uint8_t *got = nullptr);
  uint32_t hostStretchUs() const { return stretch_us_; }
#endif

 private:
  PinTable &pins_;
  uint16_t instance_;
  int sda_ = -1, scl_ = -1;
  uint8_t address_ = 0;
  bool started_ = false;
  uint32_t stretch_us_ = 0;   // kept by configure, 0 again when the plan goes (fixture §3)
  // the state the interrupt handler shares (under lock_)
  uint32_t rx_frames_ = 0;
  uint32_t errors_ = 0;
  uint8_t queue_[kQueueDepth][kMaxFrame];
  uint8_t queue_length_[kQueueDepth] = {};
  uint8_t queue_count_ = 0;
  uint8_t slot_[kQueueDepth][kMaxFrame];   // preload slots, a ring: slot_head_ is what the next read sends
  uint8_t slot_length_[kQueueDepth] = {};
  uint8_t slot_head_ = 0, slot_count_ = 0;
  // the transaction under way: its write bytes (kept up to max_length) and how many came
  uint8_t rx_[kMaxFrame];
  size_t rx_count_ = 0;
  // the TX FIFO: bytes put in since its last reset, and how far into the head slot (then 0xFF) they reached
  uint32_t tx_loaded_ = 0;
  size_t tx_pos_ = 0;
  bool tx_stale_ = false;   // a preload went into an empty set while the bus was busy: reload when it is idle
  bool stretch_held_ = false;
  uint32_t stretch_since_ = 0;
#if defined(ARDUINO_ARCH_ESP32)
  portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  i2c_slave_dev_handle_t slave_ = nullptr;
  intr_handle_t intr_ = nullptr;
  static void isr(void *context);
  void fillTx();
  void reloadTx();
  void drainRx();
  void applyStretch();
#endif
  void lock();
  void unlock();
  bool start();
  void stop();
  void clearTarget();   // frames, slots, counts: configure's fresh state
  uint8_t txByte(size_t pos) const;   // what a read sends at pos: the head slot, then (and with no slot) 0xFF
  void onTransaction(bool read);   // STOP: the write (rx_, rx_count_) a frame, and a read uses up the head slot
  bool pushFrame(const uint8_t *data, size_t length);   // false: the queue was full
};

}  // namespace oep
