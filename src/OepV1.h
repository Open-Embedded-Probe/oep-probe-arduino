// OEP v1 (oep-spec docs/v1-core-wire-delta.ja.md, frozen candidate 2026-09-25): interfaces found by name, the probe
// described by oep.core, a lock held by a host-chosen session id. Every number comes from the registry
// (OepV1Registry.h, generated from oep-spec registry/oep-v1.toml); the names below are the library's aliases.
// Results reuse the v0 Result helpers: resolutions and reject reasons 0x01..0x06 keep their values.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "OepResult.h"
#include "OepV1Registry.h"

namespace oep {
namespace v1 {

constexpr uint8_t kRoleRequest = reg::kRoleRequest, kRoleResult = reg::kRoleResult,
                  kRoleSession = reg::kRoleSessionFlag;
// Probe-initiated frames (§4.5), sent only to the lock holder that subscribed, after results.
//   role(0x06) fn(u16) seq(u16) position(u32) data        seq counts this fn's frames, position its bytes
constexpr uint8_t kRolePush = reg::kRoleData;
constexpr size_t kPushHeader = 9;
//   role(0x05) fn(u16) seq(u16) kind(u8) payload            events; fn 0 kind 1 = heartbeat (boot_id u32, uptime_ms u32)
constexpr uint8_t kRoleEvent = reg::kRoleEvent;
constexpr size_t kEventHeader = 6;
constexpr uint8_t kEventHeartbeat = reg::core::kEventHeartbeat;
constexpr size_t kRequestHeader = 6, kResultHeader = 5, kSessionBytes = 4;

// Reject reasons added in v1 (0x01..0x06 as in v0).
constexpr uint8_t kRejectNoSession = reg::kRejectNoSession;              // lock free, not the last session id: open again
constexpr uint8_t kRejectLocked = reg::kRejectLocked;                    // another session holds it; payload = remaining ms
constexpr uint8_t kRejectSessionRequired = reg::kRejectSessionRequired;  // a state-changing request without a session id
constexpr uint8_t kRejectNoConnection = reg::kRejectNoConnection;        // the request's connection is not known
constexpr uint8_t kRejectUnsupported = reg::kRejectUnsupported;          // a critical TLV (payload: tag) or value (none)

// The status byte of wire and target results (§5.4). A failure is completed failed / partial with this in it.
constexpr uint8_t kStatusOk = reg::kStatusOk, kStatusWait = reg::kStatusWait, kStatusLine = reg::kStatusLine,
                  kStatusFault = reg::kStatusFault, kStatusTimeout = reg::kStatusTimeout, kStatusState = reg::kStatusState;

// core (fn 0)
constexpr uint8_t kOpConfirm = reg::core::kOpConfirm, kOpList = reg::core::kOpList, kOpDescribe = reg::core::kOpDescribe;
constexpr uint8_t kOpOpen = reg::core::kOpOpen, kOpEnd = reg::core::kOpEnd, kOpKeepalive = reg::core::kOpKeepalive,
                  kOpLockState = reg::core::kOpLockState;
constexpr uint8_t kOpLinkSource = reg::core::kOpLinkSource, kOpLinkSink = reg::core::kOpLinkSink;
constexpr uint8_t kOpStatus = reg::core::kOpStatus, kOpCancel = reg::core::kOpCancel;
constexpr uint8_t kOpSubscribe = reg::core::kOpSubscribe, kOpUnsubscribe = reg::core::kOpUnsubscribe;
constexpr uint8_t kOpPlanApply = reg::core::kOpPlanApply, kOpPlanRelease = reg::core::kOpPlanRelease;
constexpr uint8_t kTagRoleAssignment = reg::core::kTlvPlanApplyRoleAssignment;   // fn(u16) role(u8) channel(u16), critical

// common describe tags (capability-declaration-model.ja.md §3)
constexpr uint8_t kTagRoleChannels = reg::kDescribeRoleChannels, kTagMaxClockHz = reg::kDescribeMaxClockHz,
                  kTagMaxLength = reg::kDescribeMaxLength, kTagFeatures = reg::kDescribeFeatures,
                  kTagImplementation = reg::kDescribeImplementation, kTagChannelGroup = reg::kDescribeChannelGroup;
// oep.core's own tags: the probe itself
constexpr uint8_t kCoreFirmware = reg::core::kTlvDescribeFirmware, kCoreModel = reg::core::kTlvDescribeModel,
                  kCoreUnitId = reg::core::kTlvDescribeUnitId, kCoreChannels = reg::core::kTlvDescribeChannels,
                  kCoreReserved = reg::core::kTlvDescribeReserved, kCoreProfile = reg::core::kTlvDescribeProfile,
                  kCoreLabel = reg::core::kTlvDescribeLabel, kCoreResetsOnOpen = reg::core::kTlvDescribeResetsOnOpen,
                  kCoreUartRates = reg::core::kTlvDescribeUartRates;

// TLV tag bits (§0): bit 7 = critical (in requests); 0x7F = ignored (in every result); 0xFF invalid.
constexpr uint8_t kTagCritical = reg::kTagCritical, kTagIgnored = reg::kTagIgnored, kTagInvalid = reg::kTagInvalid;

inline uint16_t getU16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }
inline uint32_t getU32(const uint8_t *p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
inline void putU16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
inline void putU32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

// bit n of a registry kLockFreeOps mask = op n needs no lock
inline bool lockFreeIn(uint64_t mask, uint8_t op) { return op < 64 && ((mask >> op) & 1); }

// A rejection with a one-byte payload (unsupported: the tag as received; gpio unavailable: the position).
inline Result rejectedWith(uint8_t reason, uint8_t *out, size_t capacity, uint8_t value) {
  if (capacity < 1) return rejected(reason);
  out[0] = value;
  return {kResolutionRejected, reason, 1};
}

// A wire / target result that did not go all the way (§5.4): nothing done = failed, some done = partial. The payload
// has the success shape; its status byte says why.
inline Result outcome(uint8_t status, size_t done, size_t length) {
  if (status == kStatusOk) return completed(length);
  return done ? partial(length) : failed(length);
}

// The firmware string every v1 probe reports (oep.core describe tag 0x40).
constexpr const char *kFirmwareVersion = "3.2.0-v1rc";

// The TLVs after a request's fixed part (§0). A handler checks the fixed part (shorter = malformed), then:
//
//   Tail tail;
//   const Result r = tail.parse(p + fixed, n - fixed, kKnown, out, capacity);   // kKnown: tags (bit 7 clear) it reads
//   if (r.resolution != kResolutionCompleted) return r;                         // malformed, or unsupported + tag
//   ... tail.find(tag, len, &critical) for a known tag; tail.refuse(tag, critical, ...) for a value it cannot honour
//   return tail.finish(result, out, capacity);                                  // appends ignored (0x7F) if any
//
// Unknown critical tags reject the request (unsupported, payload = the tag byte as received); unknown
// non-critical ones are ignored and listed in the result's ignored TLV. Tag 0xFF anywhere is malformed.
class Tail {
 public:
  static constexpr size_t kMaxIgnored = 16;

  Result parse(const uint8_t *p, size_t n, const uint8_t *known, size_t known_count, uint8_t *out, size_t capacity) {
    p_ = p;
    n_ = n;
    ignored_count_ = 0;
    size_t at = 0;
    uint8_t raw = 0, len = 0;
    const uint8_t *value = nullptr;
    while (at < n) {
      if (!step(at, raw, value, len)) return rejected(kRejectMalformed);
      if (raw == kTagInvalid || raw == kTagIgnored) return rejected(kRejectMalformed);   // 0x7F: results only (§0)
      const uint8_t tag = raw & ~kTagCritical;
      bool is_known = false;
      for (size_t i = 0; i < known_count && !is_known; ++i) is_known = (known[i] & ~kTagCritical) == tag;
      if (is_known) continue;
      if (raw & kTagCritical) return rejectedWith(kRejectUnsupported, out, capacity, raw);
      ignore(tag);
    }
    return completed();
  }
  Result parse(const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
    return parse(p, n, nullptr, 0, out, capacity);
  }
  template <size_t N>
  Result parse(const uint8_t *p, size_t n, const uint8_t (&known)[N], uint8_t *out, size_t capacity) {
    return parse(p, n, known, N, out, capacity);
  }

  // A known tag's value (its last occurrence), nullptr when absent. critical: whether the host marked it.
  const uint8_t *find(uint8_t tag, uint8_t &length, bool *critical = nullptr) const {
    const uint8_t *found = nullptr;
    size_t at = 0;
    uint8_t raw = 0, len = 0;
    const uint8_t *value = nullptr;
    while (at < n_ && step(at, raw, value, len)) {
      if ((raw & ~kTagCritical) != (tag & ~kTagCritical)) continue;
      found = value;
      length = len;
      if (critical) *critical = raw & kTagCritical;
    }
    return found;
  }
  // Every TLV in order (for a request that is a list of them, like plan_apply). false at the end.
  bool next(size_t &at, uint8_t &raw, const uint8_t *&value, uint8_t &length) const {
    return at < n_ && step(at, raw, value, length);
  }
  // A known tag whose value this probe cannot honour: critical -> the rejection (unsupported + the tag as received,
  // critical bit set) to return; otherwise it goes on the ignored list and the result is completed() (go on without).
  Result refuse(uint8_t tag, bool critical, uint8_t *out, size_t capacity) {
    tag &= ~kTagCritical;
    if (critical) return rejectedWith(kRejectUnsupported, out, capacity, tag | kTagCritical);
    ignore(tag);
    return completed();
  }
  bool anyIgnored() const { return ignored_count_ != 0; }
  // Append the ignored TLV after a completed result's payload (not after a closed tail: those ops skip this).
  Result finish(Result result, uint8_t *out, size_t capacity) const {
    if (result.resolution != kResolutionCompleted || !ignored_count_) return result;
    if (result.length + 2 + ignored_count_ > capacity) return result;
    out[result.length] = kTagIgnored;
    out[result.length + 1] = ignored_count_;
    memcpy(out + result.length + 2, ignored_, ignored_count_);
    result.length += 2 + ignored_count_;
    return result;
  }

 private:
  const uint8_t *p_ = nullptr;
  size_t n_ = 0;
  uint8_t ignored_[kMaxIgnored];
  uint8_t ignored_count_ = 0;
  bool step(size_t &at, uint8_t &raw, const uint8_t *&value, uint8_t &length) const {
    if (at + 2 > n_ || at + 2 + p_[at + 1] > n_) return false;
    raw = p_[at];
    length = p_[at + 1];
    value = p_ + at + 2;
    at += 2 + length;
    return true;
  }
  void ignore(uint8_t tag) {
    for (uint8_t i = 0; i < ignored_count_; ++i) if (ignored_[i] == tag) return;
    if (ignored_count_ < kMaxIgnored) ignored_[ignored_count_++] = tag;
  }
};

// A wire op that failed before it had anything of its success shape to report (scan, attach, attach_under_reset,
// detach): completed failed with the status byte alone.
inline Result failedStatus(uint8_t status, uint8_t *out, size_t capacity) {
  if (capacity < 1) return failed();
  out[0] = status;
  return failed(1);
}

// The common case: a request with a fixed part of `fixed` bytes and a tail of TLVs none of which this op reads.
inline Result plainTail(Tail &tail, const uint8_t *p, size_t n, size_t fixed, uint8_t *out, size_t capacity) {
  if (n < fixed) return rejected(kRejectMalformed);
  return tail.parse(p + fixed, n - fixed, out, capacity);
}

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
  // role_channels for each role: role(u8) base(u16 = 0) bitmap of the channels it may take.
  bool roleChannels(const uint8_t *roles, size_t count, uint64_t mask) {
    uint8_t value[3 + 8];
    int top = 63;
    while (top >= 0 && !((mask >> top) & 1)) --top;
    const size_t bytes = top < 0 ? 0 : static_cast<size_t>(top / 8 + 1);
    for (size_t r = 0; r < count; ++r) {
      value[0] = roles[r];
      value[1] = value[2] = 0;
      for (size_t i = 0; i < bytes; ++i) value[3 + i] = static_cast<uint8_t>(mask >> (8 * i));
      put(kTagRoleChannels, value, 3 + bytes);
    }
    return ok_;
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
  // The payload shapes are fixed by (name, revision) (§0); every oep.* interface in this library is revision 1.
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
  // Push (§4.5): while subscribed, the endpoint asks for bytes to send. Return up to `capacity` bytes and set
  // `position` to the stream position of the first one (a jump past the previous end tells the host what was lost).
  virtual bool subscribe(bool on) { (void)on; return false; }   // false: this interface does not push
  virtual size_t pull(uint32_t &position, uint8_t *out, size_t capacity) {
    (void)position; (void)out; (void)capacity;
    return 0;
  }
  // Bytes waiting to be pulled, for the subscriber's "at least n bytes or t ms" batching (0 when unknown: no batching).
  virtual size_t pending() { return 0; }
};

// A transport that sends caller-owned buffers without copying them (USB writeDirect). A buffer holds whole frames
// (length prefix included), is 64-byte aligned in DMA-capable internal RAM, and stays untouched until
// done(context, buffer) runs (on the transport's task; keep it short). Results still go through the Stream.
class DirectTransport {
 public:
  using Done = void (*)(void *context, const uint8_t *buffer);
  virtual bool queueData(const uint8_t *buffer, size_t length, Done done, void *context) = 0;
  // Data transfers queued or in flight (0: the link is idle - a buffer queued now goes straight out, alone).
  virtual size_t queued() const = 0;

 protected:
  ~DirectTransport() = default;
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

}  // namespace v1
}  // namespace oep
