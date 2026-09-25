// OEP v1 fixtures on the probe's own pins (oep-spec v1-core-wire-delta.ja.md §5.8, revision 1):
//
//   oep.fixture.gpio   plan role 1 = a line (any number of them). Only planned channels may be set or read.
//     0x01 set(n u8, n x (channel u16, mode u8)) [TLV]      modes 0 input, 1 pull-up, 2 pull-down, 3 output low,
//                                                          4 output high, 5 open-drain low, 6 open-drain release
//     0x02 read(n u8, n x channel u16) [TLV] -> n x level   no lock
//   oep.fixture.uart   plan roles 1 = RX, 2 = TX. A position stream like oep.target.console (without the stream
//                      byte): received bytes are kept from configure until plan_release whatever the sessions do,
//                      and reading does not consume them. TX idles high before configure and after plan_release.
//     0x01 configure(baud u32) [TLV 0x01 format] -> baud    0x02 read(from, arg, max) -> start flags data (no lock)
//     0x03 marks(from_serial) (no lock)   0x04 clear   0x05 mark(value)   0x06 write(count u16, data) -> accepted
//
// V0Fixture offers a v0 service (its payloads unchanged) under a project-specific name, revision 0: the ESP-IDF I2C /
// SPI slave tools. The oep.fixture.* names are revision 1 only (the v0-shape gpio / uart / capture are not offered).
#pragma once

#include "OepFixtureServices.h"
#include "OepV1.h"
#include "OepV1Stream.h"

namespace oep {
namespace v1 {

// The plan roles and extra describe TLVs of the fixtures every probe offers the same way.
constexpr uint8_t kGpioRoles[] = {reg::fixture_gpio::kRoleLine};                               // line
constexpr uint8_t kUartRoles[] = {reg::fixture_uart::kRoleRx, reg::fixture_uart::kRoleTx};    // RX, TX
constexpr uint8_t kImplementationPeripheral[] = {kTagImplementation, 1, 2};                     // implementation: peripheral

class FixtureGpio final : public Interface {
 public:
  enum : uint8_t { kOpSet = reg::fixture_gpio::kOpSet, kOpRead = reg::fixture_gpio::kOpRead };
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
  void planRelease() override;   // every planned channel back to a floating input

 private:
  PinTable &pins_;
  uint16_t instance_;
  uint8_t owner_;
  uint64_t planned_ = 0;
  bool planned(uint16_t channel) const { return channel < 64 && ((planned_ >> channel) & 1); }
};

class FixtureUart final : public Interface {
 public:
  enum : uint8_t {
    kOpConfigure = reg::fixture_uart::kOpConfigure, kOpRead = reg::fixture_uart::kOpRead,
    kOpMarks = reg::fixture_uart::kOpMarks, kOpClear = reg::fixture_uart::kOpClear,
    kOpMark = reg::fixture_uart::kOpMark, kOpWrite = reg::fixture_uart::kOpWrite,
  };
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
  void setFrameLimit(size_t max_frame) override { max_read_ = max_frame > 10 ? static_cast<uint16_t>(max_frame - 10) : 0; }
  void poll();   // from loop(): moves what the UART received into the stream (and to the port, if any)

  // Experimental (oep-spec probe-cdc-and-persistence P3): forward this UART to a serial port (a USB CDC port) both
  // ways. What the UART receives still goes into the stream, so OEP reads are unaffected; the port takes it from a
  // position of its own, as fast as the port accepts it, only while the port is open (its availableForWrite() > 0;
  // what arrives while it is closed stays in the stream only), and falling a whole buffer behind skips to the oldest
  // byte kept (portGaps). Bytes from the port go out on TX while the UART runs. nullptr stops forwarding.
  void setPort(Stream *port);
  // Run the UART at a port's line coding (the same as configure). false: no plan, or the UART refused.
  bool setLineCoding(uint32_t baud, uint8_t data_bits, uint8_t parity, uint8_t stop_bits);
  uint32_t portGaps() const { return port_.gaps(); }
  uint32_t baud() const { return configured_ ? baud_ : 0; }

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
  bool configured_ = false;
  uint16_t max_read_ = 1000;
  uint8_t buffer_[kCapacity];
  PositionStream::Mark marks_[kMarks];
  PositionStream stream_;
  void idleHigh();   // TX at the UART idle level, driven
  bool begin(uint32_t baud, uint8_t data_bits, uint8_t parity, uint8_t stop_bits);
  void forward();
  StreamPort port_;
  uint32_t baud_ = 0;
};

class V0Fixture final : public Interface {
 public:
  // roles: the plan roles this fixture takes (each may use any allowed channel); lock_free: bit n = op n
  // changes nothing; extra: more describe TLVs (limits, implementation), kept by the caller.
  V0Fixture(Service &service, const char *name, uint16_t instance, PinTable &pins, const uint8_t *roles,
            uint8_t role_count, uint32_t lock_free, const uint8_t *extra = nullptr, size_t extra_length = 0)
      : service_(service), name_(name), instance_(instance), pins_(pins), roles_(roles), role_count_(role_count),
        lock_free_(lock_free), extra_(extra), extra_length_(extra_length) {}

  const char *name() const override { return name_; }
  uint16_t instance() const override { return instance_; }
  bool lockFree(uint8_t op) const override { return op < 32 && ((lock_free_ >> op) & 1); }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override {
    return service_.handle(op, payload, length, out, capacity);
  }
  size_t describe(uint8_t *out, size_t capacity) override {
    TlvWriter w(out, capacity);
    w.roleChannels(roles_, role_count_, pins_.allowedMask());
    if (extra_length_ && w.ok() && w.length() + extra_length_ <= capacity) {
      memcpy(out + w.length(), extra_, extra_length_);
      return w.length() + extra_length_;
    }
    return w.ok() ? w.length() : 0;
  }
  uint8_t planCheck(const RoleAssignment *roles, size_t count) override { return service_.planCheck(roles, count); }
  bool planApply(const RoleAssignment *roles, size_t count) override { return service_.planApply(roles, count); }
  void planRelease() override { service_.planRelease(); }

 private:
  Service &service_;
  const char *name_;
  uint16_t instance_;
  PinTable &pins_;
  const uint8_t *roles_;
  uint8_t role_count_;
  uint32_t lock_free_;
  const uint8_t *extra_;
  size_t extra_length_;
};

}  // namespace v1
}  // namespace oep
