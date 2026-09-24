// A v0 fixture service offered under a v1 name (oep-spec capability-name-hierarchy.ja.md: v1 starts with
// oep.fixture.gpio / uart / capture). Operations and payloads stay the v0 ones for now; the describe is
// rebuilt in the v1 vocabulary (role_channels from the probe's pin table instead of channel_candidate).
#pragma once

#include "OepFixtureServices.h"
#include "OepV1.h"

namespace oep {
namespace v1 {

// The plan roles and extra describe TLVs of the fixtures every probe offers the same way.
constexpr uint8_t kGpioRoles[] = {1};                                    // line
constexpr uint8_t kUartRoles[] = {1, 2};                                 // RX, TX
constexpr uint8_t kCaptureRoles[] = {0, 1, 2, 3, 4, 5, 6, 7};            // line k
constexpr uint8_t kImplementationPeripheral[] = {kTagImplementation, 1, 2};        // implementation: peripheral
constexpr uint32_t kGpioLockFree = 1u << 2;                              // read_bank changes nothing
constexpr uint32_t kCaptureLockFree = (1u << 3) | (1u << 4);             // status, read

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
    const uint64_t mask = pins_.allowedMask();
    uint8_t value[3 + 8];
    int top = 63;
    while (top >= 0 && !((mask >> top) & 1)) --top;
    const size_t bytes = top < 0 ? 0 : static_cast<size_t>(top / 8 + 1);
    for (uint8_t r = 0; r < role_count_; ++r) {   // role(u8) base(u16 = 0) bitmap
      value[0] = roles_[r];
      value[1] = value[2] = 0;
      for (size_t i = 0; i < bytes; ++i) value[3 + i] = static_cast<uint8_t>(mask >> (8 * i));
      w.put(kTagRoleChannels, value, 3 + bytes);
    }
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
