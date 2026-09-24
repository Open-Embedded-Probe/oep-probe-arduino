// OEP v1 draft (oep-spec docs/v1-core-wire-delta.ja.md, provisional 2026-09-24): interfaces found by
// name, the probe described by oep.core, a lock held by a host-chosen session id.
// Results reuse the v0 Result helpers: resolutions and reject reasons 0x01..0x06 keep their values.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "OepService.h"

namespace oep {
namespace v1 {

constexpr uint8_t kRoleRequest = 0x01, kRoleResult = 0x02, kRoleSession = 0x80;
constexpr size_t kRequestHeader = 6, kResultHeader = 5, kSessionBytes = 4;

// Reject reasons added in v1 (0x01..0x06 as in v0).
constexpr uint8_t kRejectNoSession = 0x07;        // lock free, not the last session id: open again
constexpr uint8_t kRejectLocked = 0x08;           // another session holds it; payload = remaining ms (u32)
constexpr uint8_t kRejectSessionRequired = 0x09;  // a state-changing request without a session id

// core (fn 0)
constexpr uint8_t kOpConfirm = 0x01, kOpList = 0x02, kOpDescribe = 0x03;
constexpr uint8_t kOpOpen = 0x10, kOpEnd = 0x11, kOpKeepalive = 0x12, kOpLockState = 0x13;
constexpr uint8_t kOpStatus = 0x20, kOpCancel = 0x21;

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
};

}  // namespace v1
}  // namespace oep
