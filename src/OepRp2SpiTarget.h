// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.spi-target revision 1 (oep-spec oep-if-fixture §4) on an RP2040 / RP2350: a PIO state machine as the SPI
// slave, the same ops as the ESP32's (OepP4SpiTarget.h). NOT YET RUN ON HARDWARE (no SPI master on the bench when it was
// written, 2026-10-27): built, to be checked once one is wired.
//   0x10 configure(mode u8 0-3, bit_order u8)   0x11 arm(length u16, count u16, tx)
//   0x12 read_rx -> pending(u8) bits(u32) count(u16) data   0x13 status (no lock)
// Roles: 1 SCK, 2 MOSI, 3 MISO, 4 CS, any GPIOs (0-31).
//
// The program (one per CPHA; CPOL 1 inverts SCK's input while configured): after CS falls it drives MISO, takes a tx byte
// each 8 bits (`pull noblock`: past tx the X register, 0), puts a MISO bit out and takes a MOSI bit in at the mode's
// edges, counting the bits down in Y. Its RX words go by DMA into the transfer's buffer, its tx by DMA from the armed bytes.
// CS rising is a GPIO interrupt: it stops the state machine, lets MISO go, takes the partial byte and the bit count out of
// it, queues the transfer (or counts it unarmed) and starts it again for the next CS. So MISO is driven a little after CS
// falls and released a little after it rises: kCsSetupNs, a cautious figure until measured.
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

#if defined(ARDUINO_ARCH_RP2040)
#include <hardware/pio.h>

namespace oep {

class Rp2SpiTarget final : public Interface {
 public:
  static constexpr uint8_t kOwnerId = 6;
  enum Role : uint8_t { kRoleSck = 1, kRoleMosi = 2, kRoleMiso = 3, kRoleCs = 4 };
  static constexpr size_t kMaxFrame = 64;
  static constexpr size_t kRxRoom = 256;        // a master past the armed length still clocked in (then the PIO stalls)
  static constexpr size_t kQueueDepth = 4;
  static constexpr uint32_t kMaxClockHz = 10000000;   // a PIO wait a bit at clk_sys: cautious until measured
  static constexpr uint32_t kCsSetupNs = 20000;       // the CS interrupt's latency both ways: cautious until measured
  static constexpr uint8_t kModes = 0x0f;

  enum : uint8_t {
    kOpConfigure = reg::fixture_spi_target::kOpConfigure, kOpArm = reg::fixture_spi_target::kOpArm,
    kOpReadRx = reg::fixture_spi_target::kOpReadRx, kOpStatus = reg::fixture_spi_target::kOpStatus,
  };

  Rp2SpiTarget(PinTable &pins, uint16_t instance = 0) : pins_(pins), instance_(instance) {}
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
  void onCsRise();   // the GPIO interrupt's: a transfer ended

 private:
  PinTable &pins_;
  uint16_t instance_;
  uint8_t refusal_cause_ = reg::core::kUnavailableCauseWrongState;
  uint16_t refusal_channel_ = 0xffff;
  int sck_ = -1, mosi_ = -1, miso_ = -1, cs_ = -1;
  uint8_t mode_ = 0, bit_order_ = 0;
  bool started_ = false;
  // the PIO and DMA, taken by configure, given back by stop
  PIO pio_ = nullptr;
  int sm_ = -1, offset_ = -1, dma_rx_ = -1, dma_tx_ = -1;
  uint8_t program_ = 0xff;   // which program is loaded (CPHA)
  // the transfer under way: its MOSI bytes, and the tx it was armed with (armed_ false: unarmed, MISO 0)
  uint8_t rx_[kRxRoom];
  uint8_t tx_[kMaxFrame];
  size_t armed_length_ = 0, armed_count_ = 0;
  volatile bool armed_ = false;        // the transfer under way (or next) is the armed one
  volatile bool arm_pending_ = false;  // armed while CS was low: loaded at its rise
  // the finished transfers, oldest first (the interrupt adds, loop() takes: under noInterrupts)
  uint8_t queue_[kQueueDepth][kMaxFrame];
  uint8_t queue_length_[kQueueDepth] = {};
  uint32_t queue_bits_[kQueueDepth] = {};
  volatile uint8_t queue_count_ = 0;
  volatile uint32_t transactions_ = 0, errors_ = 0;
  bool start();
  void stop();
  void restart();   // the state machine and the DMA ready for the next CS (armed_: from tx_)
  void count(bool error);
};

}  // namespace oep

#endif
