// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 fixtures on the probe's own pins (oep-spec docs/oep-if-fixture.ja.md, revision 1):
//
//   oep.fixture.gpio   plan role 1 = a line (any number of them). Only planned channels may be set or read.
//     0x01 set(n u8, n x (channel u16, mode u8)) [TLV]      modes 0 input, 1 pull-up, 2 pull-down, 3 output low,
//                                                          4 output high, 5 open-drain low, 6 open-drain release,
//                                                          7 input with pull-up and pull-down
//     0x02 read(n u8, n x channel u16) [TLV] -> n(u8) n x level [TLV]   no lock
//   oep.fixture.uart   plan roles 1 = RX, 2 = TX. A position stream like oep.target.console (without the stream
//                      byte): the plan makes the stream and the UART runs from then on (the probe.config uart item's
//                      settings, else 115200 8N1) until plan_release; reading does not consume. Positions and mark
//                      serials never go back within a boot (a plan made again goes on from where the last left off).
//                      TX idles high while planned (before configure too); released, it goes to its idle state
//                      (PinTable, default Hi-Z).
//     0x01 configure(baud u32) [TLV 0x01 format, critical] -> baud (actual) [TLV]
//     0x02 read(from, arg, max) -> start flags len data [TLV] (no lock)   0x03 marks(from_serial) (no lock)
//     0x04 clear   0x05 mark(value)   0x06 write(count u16, data) -> accepted
//     0x07 status -> configured(u8: 0 default, 1 session configure, 2 settings item, 3 item whose baud could not be made,
//                    the default applied) baud format (no lock)
//
// The ESP32 I2C / SPI targets are in OepP4I2cTarget.h / OepP4SpiTarget.h.
#pragma once

#include "OepPinTable.h"
#include "Oep.h"
#include "OepStream.h"

namespace oep {

// The plan roles and extra describe TLVs of the fixtures every probe offers the same way.
constexpr uint8_t kGpioRoles[] = {reg::fixture_gpio::kRoleLine};                               // line
constexpr uint8_t kUartRoles[] = {reg::fixture_uart::kRoleRx, reg::fixture_uart::kRoleTx};    // RX, TX
constexpr uint8_t kImplementationPeripheral[] = {kTagImplementation, 1, 2};                     // implementation: peripheral

class FixtureGpio final : public Interface {
 public:
  enum : uint8_t { kOpSet = reg::fixture_gpio::kOpSet, kOpRead = reg::fixture_gpio::kOpRead };
  static constexpr uint32_t kModes = 0xff;   // modes 0-7, all of them (describe modes, u32 bit set)
  // owner: this instance's PinTable owner id (keeps it off the channels other fixtures hold).
  FixtureGpio(PinTable &pins, uint16_t instance, uint8_t owner = 1) : pins_(pins), instance_(instance), owner_(owner) {}
  const char *name() const override { return reg::fixture_gpio::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_gpio::kRevision; }
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::fixture_gpio::kLockFreeOps, op); }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;   // every planned channel back to its idle state

 private:
  PinTable &pins_;
  uint16_t instance_;
  uint8_t owner_;
  uint64_t planned_ = 0;
  bool planned(uint16_t channel) const { return channel < 64 && ((planned_ >> channel) & 1); }
};

class FixtureUart final : public Interface, public BindSource {
 public:
  enum : uint8_t {
    kOpConfigure = reg::fixture_uart::kOpConfigure, kOpRead = reg::fixture_uart::kOpRead,
    kOpMarks = reg::fixture_uart::kOpMarks, kOpClear = reg::fixture_uart::kOpClear,
    kOpMark = reg::fixture_uart::kOpMark, kOpWrite = reg::fixture_uart::kOpWrite, kOpStatus = reg::fixture_uart::kOpStatus,
  };
  static constexpr uint32_t kDefaultBaud = 115200, kMinBaud = 1200, kMaxBaud = 2000000;
  // owner: this instance's PinTable owner id (each UART instance needs its own so a release returns only its pins).
  FixtureUart(PinTable &pins, OepUart &serial, uint16_t instance, uint8_t owner = 2)
      : pins_(pins), serial_(serial), instance_(instance), owner_(owner), stream_(buffer_, kCapacity, marks_, kMarks) {}
  const char *name() const override { return reg::fixture_uart::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_uart::kRevision; }
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::fixture_uart::kLockFreeOps, op); }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override;
  bool planApply(const RoleAssignment *roles, size_t count) override;
  void planRelease() override;
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 14 ? static_cast<uint16_t>(max_frame - 14) : 0; }
  void poll();   // from loop(): moves what the UART received into the stream
  uint32_t baud() const { return running_ ? baud_ : 0; }
  // The probe.config uart item (oep-if-probe-config §1): applied when the plan gives the UART its pins, unless a
  // session's configure came since (that wins until the plan is released). format: the configure TLV's byte.
  void setItem(uint32_t baud, uint8_t format);
  void clearItem();
  // What probe.config checks before taking a uart item: the format's bits defined (else malformed), the baud this UART
  // can run within 5 % (else unsupported).
  static bool formatDefined(uint8_t format);
  static bool baudWithinReach(uint32_t baud) { return baud >= kMinBaud && baud <= kMaxBaud; }
  // BindSource (oep.probe.config §1.2): the receive stream while the UART is planned; a port's raw bytes go out on TX,
  // as much as the UART takes without waiting.
  const PositionStream *bindStream() const override { return rx_ >= 0 || tx_ >= 0 ? &stream_ : nullptr; }
  uint16_t bindStreamNumber() const override { return 1; }   // one stream for good: positions only grow
  size_t bindInput(const uint8_t *data, size_t length) override;

 private:
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  // a port forwarded over USB may not be drained for 90 ms and more (usbip): 8 KiB overflowed at 921600 (P7)
  static constexpr size_t kCapacity = 32768, kMarks = 16;
#else
  static constexpr size_t kCapacity = 8192, kMarks = 16;   // both powers of two (wrapping positions / serials)
#endif
  PinTable &pins_;
  OepUart &serial_;
  uint16_t instance_;
  uint8_t owner_;
  int rx_ = -1, tx_ = -1;
  bool running_ = false;            // the UART runs (planned and begun)
  bool session_configured_ = false; // configure since the plan: it wins over the item until the plan is released
  uint8_t configured_ = reg::fixture_uart::kUartConfiguredDefault;   // what the running settings came from (status)
  bool item_set_ = false;
  uint32_t item_baud_ = kDefaultBaud;
  uint8_t item_format_ = 0;
  uint16_t max_read_ = 1000;
  // Receive errors the UART driver reports (ESP32: onReceiveError, from its event task): counted there, marked lost in
  // poll() at the stream's position (fixture §2 / common §1.3: detail 1 overflow, 2 framing, 3 parity). Before this a
  // byte lost in the hardware FIFO at 2 Mbaud left no mark (X035, 2026-10-01).
  volatile uint16_t rx_overflows_ = 0, rx_framing_ = 0, rx_parity_ = 0;
  uint16_t marked_overflows_ = 0, marked_framing_ = 0, marked_parity_ = 0;
  void markReceiveErrors();
  uint8_t buffer_[kCapacity];
  PositionStream::Mark marks_[kMarks];
  PositionStream stream_;
  void idleHigh();   // TX at the UART idle level, driven
  bool begin(uint32_t baud, uint8_t format);
  void applySettings();   // the plan's start: the session's configure, else the item, else the default
  uint32_t baud_ = 0;
  uint8_t format_ = 0;
};

}  // namespace oep
