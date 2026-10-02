// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.spi-target revision 1 (oep-spec oep-if-fixture §4), on an ESP32's hardware SPI target (the ESP-IDF
// spi_slave driver, SPI2_HOST, no DMA: 64-byte FIFO transactions).
//   0x01 configure(mode u8 0-3, bit_order u8: 0 MSB first, 1 LSB first)   0x02 arm(length u16, count u16, tx)
//   0x03 read_rx -> pending(u8) bits(u32) count(u16) data
//   0x04 status -> state mode bit_order armed queued (u8 each) transactions(u32) errors(u32) (no lock)   0x05 reset.   Every request takes a TLV tail (oep-core §2.3). Roles: 1 SCK, 2 MOSI, 3 MISO, 4 CS.
// One CS-framed transaction is armed at a time with the MISO bytes to send;
// after the master raises CS the result (MOSI bytes, length in bits) is queued
// for read_rx. The driver's interrupt loads transactions (below); service() in loop() does the accounting.
// While nothing is armed a discard transaction (MISO 0, MOSI to a scratch buffer) waits in the driver, so a transfer
// the host did not arm is seen and counted in transactions and errors (fixture §4); arm takes it back (the driver's
// queue reset) and loads the armed one. A CS frame with no SCK edge (0 bits) is no transfer: it counts nothing and
// leaves the arm waiting (fixture §4).
// The next transaction is loaded by the driver's interrupt, in post_trans_cb, at the CS rising edge that ended the last
// one - never later from loop(): on the classic ESP32 without DMA a load is a sync reset of the slave and a rewrite of
// its buffer, so a load while CS is low restarts the frame from that bit (MISO from byte 0 again, MOSI kept from there).
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

// The ESP-IDF spi_slave driver; OEP_HOST_FAKE_SPI_SLAVE: a host test's fake of it (tests/host/shim)
#if defined(ARDUINO_ARCH_ESP32) || defined(OEP_HOST_FAKE_SPI_SLAVE)
#define OEP_SPI_SLAVE_DRIVER 1
#include <driver/spi_slave.h>
#include <esp_private/spi_slave_internal.h>   // spi_slave_queue_trans_isr, spi_slave_queue_reset
#include <freertos/FreeRTOS.h>
#endif
// The classic ESP32's slave drives MISO from spi_slave_initialize to spi_slave_free, CS high as well (bench,
// 2026-10-02: low while CS is high, configured, armed and between frames; the ESP32-P4's leaves it undriven). A target
// that drives the line while it is not selected fights any other device on it, so there the pad's output enable is taken
// from the slave and follows CS (a CS edge interrupt): on while CS is low, off while it is high. The fake spi_slave
// models the classic, so the host tests run with the gate.
// The gate only switches the pad's output enable; the pad's value stays the slave's MISO signal throughout, so once
// enabled after CS fell it carries what the slave shifts - nothing the gate holds of its own. Its handler runs on core 0
// (kGateCore): the SWIO wire's frames run in loop() on core 1 with that core's interrupts masked for up to about 45 us
// each, and a handler there waited them out. The delay from CS falling to MISO driven - the time of one level-3 GPIO
// interrupt on a core that is not masked - is declared as describe cs_setup_ns (kCsSetupNs): during it MISO is
// undriven. Not reachable in software below about 1 us on this chip: the level-5 vector is ESP-IDF's own (xt_highint5,
// the interrupt watchdog and the cache-lock fix) and the level-4 one the Bluetooth controller's dispatcher, whose C
// entry alone costs more than the 0.3 us asked for.
#if (defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32)) || defined(OEP_HOST_FAKE_SPI_SLAVE)
#define OEP_SPI_MISO_GATE 1
#include <driver/gpio.h>
#include <esp_ipc.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <hal/gpio_ll.h>
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
  // finished transactions, oldest first
  uint8_t queue_[kQueueDepth][kMaxFrame];
  uint8_t queue_length_[kQueueDepth] = {};
  uint32_t queue_bits_[kQueueDepth] = {};
  uint8_t queue_count_ = 0;
#if defined(OEP_SPI_SLAVE_DRIVER)
  spi_slave_transaction_t trans_ = {};
  spi_slave_transaction_t idle_trans_ = {};
  // What the interrupt saw since service() last looked (under lock_): the armed one ended (its bits), unarmed
  // transfers, and a load that failed (service() retries it).
  portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  volatile bool isr_armed_done_ = false;
  volatile uint32_t isr_armed_bits_ = 0;
  volatile uint32_t isr_unarmed_ = 0;
  volatile bool isr_load_failed_ = false;
  static void onDone(spi_slave_transaction_t *done);   // post_trans_cb, in the driver's interrupt
  bool begin();        // the driver alone
#endif
#if defined(OEP_SPI_MISO_GATE)
 public:
  // The core the CS handler runs on (not loop()'s, which the SWIO frames mask), and the declared worst-case delay from
  // CS falling to MISO driven (describe cs_setup_ns): one level-3 GPIO interrupt there, about 1.5 us (1-2 us measured on
  // the bench with nothing masking the core), plus the longest the core masks level 3 itself - FreeRTOS's and ESP-IDF's
  // critical sections on core 0, a few us. An estimate from the code, to be measured on the bench. Not covered: a logic
  // capture of the core-0 sampler running at the same time (it masks core 0 for its whole window, up to 164 ms).
  static constexpr uint32_t kGateCore = 0;
  static constexpr uint32_t kCsSetupNs = 10000;

 private:
  bool gated_ = false;
  static void onCs(void *self);   // a CS edge, in the GPIO interrupt: MISO's output enable = CS low
  static constexpr uint32_t kServiceStack = 4096;   // the install task's stack (bytes): esp_intr_alloc and a log
  static bool gateService();             // the GPIO ISR service on kGateCore, once per boot (an ordinary task)
  static void gateInstall(void *self);   // on kGateCore (esp_ipc): the handler, and MISO set from CS at once
  static void gateRemove(void *self);    // on kGateCore: the handler out, MISO undriven
  bool gate_ok_ = false;
  bool gateBegin();
  void gateEnd();
#endif
  bool start();
  void stop();
  bool arm(const uint8_t *tx, size_t tx_length, size_t length);
};

}  // namespace oep
