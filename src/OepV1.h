// OEP v1 draft (oep-spec docs/v1-core-wire-delta.ja.md, provisional 2026-09-24): interfaces found by
// name, the probe described by oep.core, a lock held by a host-chosen session id.
// Results reuse the v0 Result helpers: resolutions and reject reasons 0x01..0x06 keep their values.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "OepResult.h"

namespace oep {
namespace v1 {

constexpr uint8_t kRoleRequest = 0x01, kRoleResult = 0x02, kRoleSession = 0x80;
// Experimental (2026-09-25): probe-initiated data, sent only while a host has subscribed and given credit.
//   role(0x06) fn(u16) seq(u16) position(u32) data        seq counts this fn's frames, position its bytes
constexpr uint8_t kRolePush = 0x06;
constexpr size_t kPushHeader = 9;
constexpr size_t kRequestHeader = 6, kResultHeader = 5, kSessionBytes = 4;

// Reject reasons added in v1 (0x01..0x06 as in v0).
constexpr uint8_t kRejectNoSession = 0x07;        // lock free, not the last session id: open again
constexpr uint8_t kRejectLocked = 0x08;           // another session holds it; payload = remaining ms (u32)
constexpr uint8_t kRejectSessionRequired = 0x09;  // a state-changing request without a session id

// core (fn 0)
constexpr uint8_t kOpConfirm = 0x01, kOpList = 0x02, kOpDescribe = 0x03;
constexpr uint8_t kOpOpen = 0x10, kOpEnd = 0x11, kOpKeepalive = 0x12, kOpLockState = 0x13;
constexpr uint8_t kOpStatus = 0x20, kOpCancel = 0x21;
// Experimental: subscribe(fn u16, credit u32), credit(fn u16, add u32), unsubscribe(fn u16).
constexpr uint8_t kOpSubscribe = 0x30, kOpCredit = 0x31, kOpUnsubscribe = 0x32;

// common describe tags (capability-declaration-model.ja.md §3)
constexpr uint8_t kTagRoleChannels = 0x01, kTagMaxClockHz = 0x02, kTagMaxLength = 0x03, kTagFeatures = 0x06,
                  kTagImplementation = 0x07, kTagChannelGroup = 0x08;
// oep.core's own tags: the probe itself
constexpr uint8_t kCoreFirmware = 0x40, kCoreModel = 0x41, kCoreUnitId = 0x42, kCoreChannels = 0x43,
                  kCoreReserved = 0x44, kCoreProfile = 0x45, kCoreLabel = 0x46, kCoreResetsOnOpen = 0x47,
                  kCoreUartRates = 0x48;

inline uint16_t getU16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }
inline uint32_t getU32(const uint8_t *p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
inline void putU16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
inline void putU32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

// The firmware string every v1 draft probe reports (oep.core describe tag 0x40).
constexpr const char *kFirmwareVersion = "3.1.0-v1draft";

// Appends TLVs (tag, len, value) to a fixed buffer; ok() stays false once something did not fit.
class TlvWriter {
 public:
  TlvWriter(uint8_t *buffer, size_t capacity) : p_(buffer), cap_(capacity) {}
  bool put(uint8_t tag, const void *value, size_t length) {
    if (!ok_ || length > 253 || n_ + 2 + length > cap_) return ok_ = false;
    p_[n_++] = tag;
    p_[n_++] = static_cast<uint8_t>(length);
    if (length) memcpy(p_ + n_, value, length);
    n_ += length;
    return true;
  }
  bool u8(uint8_t tag, uint8_t v) { return put(tag, &v, 1); }
  bool u16(uint8_t tag, uint16_t v) { uint8_t b[2]; putU16(b, v); return put(tag, b, 2); }
  bool u32(uint8_t tag, uint32_t v) { uint8_t b[4]; putU32(b, v); return put(tag, b, 4); }
  bool text(uint8_t tag, const char *s) { return put(tag, s, strlen(s)); }
  bool label(uint16_t channel, const char *name) {
    uint8_t b[2 + 32];
    const size_t n = strlen(name) < 32 ? strlen(name) : 32;
    putU16(b, channel);
    memcpy(b + 2, name, n);
    return put(kCoreLabel, b, 2 + n);
  }
  // A wire's fixed pin set as a channel group: role 1 SWDIO (or SWIO), role 2 SWCLK (none on a one-wire link).
  bool pinGroup(uint16_t swdio, uint16_t swclk) {
    uint8_t group[7] = {1, 1, 0, 0, 2, 0, 0};
    putU16(group + 2, swdio);
    putU16(group + 5, swclk);
    return put(kTagChannelGroup, group, swclk == 0xffff ? 4 : sizeof group);
  }
  size_t length() const { return n_; }
  bool ok() const { return ok_; }

 private:
  uint8_t *p_;
  size_t cap_;
  size_t n_ = 0;
  bool ok_ = true;
};

// One offered interface. The endpoint gives it fn = 1, 2, ... in registration order.
class Interface {
 public:
  virtual ~Interface() = default;
  virtual const char *name() const = 0;
  virtual uint16_t instance() const = 0;
  virtual uint8_t revision() const { return 0; }
  virtual uint8_t flags() const { return 0; }
  // The whole describe as TLV bytes; the endpoint pages it by whole TLVs.
  virtual size_t describe(uint8_t *out, size_t capacity) { (void)out; (void)capacity; return 0; }
  // Operations that change nothing may run without the lock (and without a session id).
  virtual bool lockFree(uint8_t op) const { (void)op; return false; }
  virtual Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) = 0;
  // Pin plan (core plan_apply / plan_release, the v0 shape): check without side effects (0 = acceptable, else a
  // reject reason), apply, undo. The plan is probe state: it outlives sessions until released.
  virtual uint8_t planCheck(const RoleAssignment *roles, size_t count) {
    (void)roles;
    return count ? kRejectUnavailable : 0;
  }
  virtual bool planApply(const RoleAssignment *roles, size_t count) { (void)roles; (void)count; return true; }
  virtual void planRelease() {}
  // The endpoint's frame limit, told when the interface is added: what a describe may promise.
  virtual void setFrameLimit(size_t max_frame) { (void)max_frame; }
  // Experimental push: while subscribed, the endpoint asks for bytes to send. Return up to `capacity` bytes and set
  // `position` to the stream position of the first one (a jump past the previous end tells the host what was lost).
  virtual bool subscribe(bool on) { (void)on; return false; }   // false: this interface does not push
  virtual size_t pull(uint32_t &position, uint8_t *out, size_t capacity) {
    (void)position; (void)out; (void)capacity;
    return 0;
  }
};

// The part of oep.core's describe every probe writes the same way: firmware, model, unit id, channel count and
// the reserved-channel bitmap. The sketch adds its profile and labels after it.
inline bool describeCore(TlvWriter &w, const char *model, const uint8_t *unit_id, size_t unit_id_length,
                         uint16_t channels, uint64_t reserved) {
  w.text(kCoreFirmware, kFirmwareVersion);
  w.text(kCoreModel, model);
  if (unit_id_length) w.put(kCoreUnitId, unit_id, unit_id_length);
  w.u16(kCoreChannels, channels);
  uint8_t bitmap[2 + 8] = {0, 0};                       // first channel (u16) = 0, then one bit per channel
  const size_t bytes = (channels + 7) / 8 < 8 ? (channels + 7) / 8 : 8;
  for (size_t i = 0; i < bytes; ++i) bitmap[2 + i] = static_cast<uint8_t>(reserved >> (8 * i));
  return w.put(kCoreReserved, bitmap, 2 + bytes);
}

constexpr uint8_t kOpPlanApply = 0x04, kOpPlanRelease = 0x05;
constexpr uint8_t kTagRoleAssignment = 0x90;   // fn(u16) role(u8) channel(u16), critical

}  // namespace v1
}  // namespace oep
