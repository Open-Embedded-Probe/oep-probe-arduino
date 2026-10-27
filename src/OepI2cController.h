// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.i2c-controller revision 1 (oep-spec oep-if-fixture §5): the probe as the I2C controller, for testing a
// DUT's I2C target.
//   0x10 configure(clock_hz u32, stretch_timeout_us u32) -> clock_hz(u32): the SCL used
//   0x11 transfer(flags u8, n u8, n x (dir u8, address u8, count u16, [data])) -> result(u8) at(u8) index(u16) count(u16) data
//   0x12 status -> state(u8) held(u8) transfers(u32) errors(u32)   (no lock)
// Every request takes a TLV tail (oep-core §2.3). Roles: 1 SDA, 2 SCL. The segments are joined by repeated STARTs; with
// flags bit0 a STOP ends the transfer, without it the bus is held and the next transfer starts with a repeated START.
// A NACK, a bus error or a stretch past the timeout ends the transfer there with a STOP (a bus error: both lines let
// go), and is the answer's result, not a failure.
//
// The lines are GPIOs driven open drain, a byte at a time (lineStart / lineWrite / lineRead / lineStop): the NACKed byte's
// place and the transfer left without a STOP are the controller's own to see, which an I2C driver does not tell. On an
// ESP32 the GPIO registers, each half period counted on the CPU's cycle counter from the edge before (an interrupt only
// stretches a half, which I2C allows: the clock is the controller's); a high SCL is waited for (the target's stretch). The
// pads' internal pull-ups are on while configured (features bit0), as the I2C target's. OEP_HOST_FAKE_I2C_BUS: a host
// test's target answers the bytes (I2cHostBus).
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"

namespace oep {

#if defined(OEP_HOST_FAKE_I2C_BUS)
// The host test's bus, a byte at a time: each returns a line result (I2cController::Line)
struct I2cHostBus {
  virtual uint8_t start(bool repeated) = 0;
  virtual uint8_t write(uint8_t byte, bool &ack) = 0;
  virtual uint8_t read(uint8_t &byte, bool ack) = 0;
  virtual void stop() = 0;
  virtual void release() = 0;   // both lines let go (a bus error)
};
#endif

class I2cController final : public Interface {
 public:
  static constexpr uint8_t kOwnerId = 8;
  enum Role : uint8_t { kRoleSda = 1, kRoleScl = 2 };
  static constexpr size_t kMaxLength = 256;   // the bytes written and read in one transfer
  static constexpr uint32_t kMaxClockHz = 400000;   // fast mode, on the pads' pull-ups and the bus's own
  static constexpr uint32_t kMinClockHz = 1000;
  static constexpr uint32_t kMaxStretchTimeoutUs = 100000;   // one wait for SCL, well inside max_op_ms
  // What a line step ends with: the result values of the transfer's answer (fixture §5), 1 a NACK
  enum Line : uint8_t {
    kLineOk = reg::fixture_i2c_controller::kResultDone, kLineNack = reg::fixture_i2c_controller::kResultAddressNack,
    kLineBusError = reg::fixture_i2c_controller::kResultBusError,
    kLineStretch = reg::fixture_i2c_controller::kResultStretchTimeout,
  };

  enum : uint8_t {
    kOpConfigure = reg::fixture_i2c_controller::kOpConfigure, kOpTransfer = reg::fixture_i2c_controller::kOpTransfer,
    kOpStatus = reg::fixture_i2c_controller::kOpStatus,
  };

  I2cController(PinTable &pins, uint16_t instance = 0) : pins_(pins), instance_(instance) {}
  const char *name() const override { return reg::fixture_i2c_controller::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_i2c_controller::kRevision; }
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
  void sessionOver() override { if (held_) { lineStop(); held_ = false; } }   // the bus not left held (fixture §5)

#if defined(OEP_HOST_FAKE_I2C_BUS)
  static inline I2cHostBus *host_bus = nullptr;
  static inline bool fake_fail_start = false;   // host tests: the next start fails
#endif

 private:
  PinTable &pins_;
  uint16_t instance_;
  uint8_t refusal_cause_ = reg::core::kUnavailableCauseWrongState;   // the last planCheck's unavailable (core §4.3)
  uint16_t refusal_channel_ = 0xffff;
  int sda_ = -1, scl_ = -1;
  bool started_ = false, held_ = false;
  uint32_t clock_hz_ = 0, stretch_timeout_us_ = 0;
  uint32_t transfers_ = 0, errors_ = 0;
#if defined(ARDUINO_ARCH_ESP32)
  int sda_pad_ = -1, scl_pad_ = -1;
  uint32_t half_ = 0;       // a half SCL period, CPU cycles
  uint32_t timeout_ = 0;    // stretch_timeout_us, CPU cycles
  uint32_t edge_ = 0;       // the cycle count at the last edge
  void sda(bool high) const;
  void scl(bool high) const;
  bool sdaHigh() const;
  bool sclHigh() const;
  void waitHalf();
  bool sclRelease();        // SCL let go and waited high: false, the target held it past the timeout
  uint8_t bitOut(bool bit);
  uint8_t bitIn(bool &bit);
#endif
  bool start();
  void stop();
  uint8_t lineStart(bool repeated);
  uint8_t lineWrite(uint8_t byte, bool &ack);
  uint8_t lineRead(uint8_t &byte, bool ack);
  void lineStop();
  void lineRelease();
  Result transfer(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
};

}  // namespace oep
