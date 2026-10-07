// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 core (oep-spec docs/oep-core.ja.md, 0f455a0): interfaces found by name, the probe described by fn 0 (the core,
// which has no name and is not listed), a lock held by a host-chosen session id, one clock (ns since boot, u64) and one
// space of resource numbers. The interfaces' shared parts are in OepStream.h (position streams) and OepDebug.h (wire /
// target status, pin pairs). Every number comes from the registry (OepRegistry.h, generated from oep-spec
// registry/oep-v1.toml); the names below are the library's aliases.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "OepResult.h"
#include "OepRegistry.h"
#include "openembeddedprobe_version.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_random.h>
#include <esp_timer.h>
#elif defined(ARDUINO_ARCH_RP2040)
#include <Arduino.h>
#include <pico/time.h>
#else
#include <Arduino.h>
#endif

namespace oep {

// The wire numbers (generated from oep-spec registry/oep-v1.toml - protocol revision 1 - into OepRegistry.h).
namespace reg = v1::reg;

// oep.probe.restart's handler (Endpoint::setRestart, oep::platformRestart): a probe on USB first takes its device off the
// bus (the pull-up off: the host records an unplug, not a device that stopped answering mid-transfer), waits
// kRestartDetachMs for the host to see it, and only then resets the chip, which takes kRestartResetMs at most to start
// (an RP2's watchdog reset fires 10 ms on). A reset with the device still on the bus left an RP2350 and an ESP32-P4
// failing their device descriptor request after they came back, until a replug (bench, 0.0.29-dev+3c0cd99: Windows'
// "unknown USB device (device descriptor request failed)"). Endpoint::kRestartSettleMs (the answer's last packet taken)
// before the handler, these after it: the reset starts soon after the answer (oep-if-restart §2: nothing is answered
// until it does).
constexpr uint32_t kRestartDetachMs = 60, kRestartResetMs = 10;

constexpr uint8_t kRoleRequest = reg::kRoleRequest, kRoleResult = reg::kRoleResult;
// Probe-initiated frames (core §11), sent only to the lock holder that subscribed to the emitting fn, after results.
//   role(0x06) fn(u16) seq(u16) payload     data: payload = position(u64) len(u16) data [TLV] (core §11.2)
constexpr uint8_t kRolePush = reg::kRoleData;
constexpr size_t kPushHeader = 5;
//   role(0x05) fn(u16) seq(u16) kind(u8) fixed part [TLV]   events, the kinds the interface's own (fn 0 sends none)
constexpr uint8_t kRoleEvent = reg::kRoleEvent;
constexpr size_t kEventHeader = 6;
// A request: role(0x01) corr(u16) fn(u16) op(u8) session_id(u32) payload - one 10-byte header, session_id 0 = no
// session (core §4.1). A result: role(0x02) corr(u16) resolution(u8) detail(u8) payload (core §4.2).
constexpr size_t kRequestHeader = 10, kResultHeader = 5;

// Reject reasons added in v1 (0x01..0x06 as in v0).
constexpr uint8_t kRejectResultLost = reg::kRejectResultLost;
constexpr uint8_t kRejectNoSession = reg::kRejectNoSession;              // a session_id while no session holds the lock: open again
constexpr uint8_t kRejectLocked = reg::kRejectLocked;                    // another session holds it; payload = remaining ms
constexpr uint8_t kRejectSessionRequired = reg::kRejectSessionRequired;  // an op that needs the lock with session_id 0
constexpr uint8_t kRejectNoConnection = reg::kRejectNoConnection;        // the request's connection is not known
constexpr uint8_t kRejectUnsupported = reg::kRejectUnsupported;          // defined, not handled here: payload tag(u8) [TLV]

// The longest one request may take (core §7.5 max_op_ms, declared in fn 0's describe): this implementation's value
// (docs/implementation-limits.ja.md §1.1; the specification leaves it to the probe).
constexpr uint32_t kMaxOpMs = 10000;
static_assert(kMaxOpMs >= 1 && kMaxOpMs <= reg::kLimitMaxOpMsMax, "max_op_ms is 1 to max_op_ms_max (core §7.5)");

// core (fn 0, core §12): every op required, none optional
constexpr uint8_t kOpConfirm = reg::core::kOpConfirm, kOpList = reg::core::kOpList, kOpDescribe = reg::core::kOpDescribe,
                  kOpClock = reg::core::kOpClock;
constexpr uint8_t kOpOpen = reg::core::kOpOpen, kOpEnd = reg::core::kOpEnd, kOpKeepalive = reg::core::kOpKeepalive,
                  kOpLockState = reg::core::kOpLockState;
// subscribe / unsubscribe: ops of the interface that sends notifications, the same numbers in every interface's op space
// (core §11.3); the endpoint answers them for an interface that notifies()
constexpr uint8_t kOpSubscribe = reg::kOpSubscribe, kOpUnsubscribe = reg::kOpUnsubscribe;
// oep.probe.plan (oep-if-plan) and oep.probe.restart (oep-if-restart): their own fns, which the endpoint lists itself
constexpr uint8_t kOpPlanApply = reg::probe_plan::kOpPlanApply, kOpPlanRelease = reg::probe_plan::kOpPlanRelease;
constexpr uint8_t kTagRoleAssignment = reg::probe_plan::kTlvPlanApplyRoleAssignment;   // fn(u16) role(u8) channel(u16)
constexpr uint8_t kOpRestart = reg::probe_restart::kOpRestart;

// common describe tags (core §7.4)
constexpr uint8_t kTagRoleChannels = reg::kDescribeRoleChannels, kTagMaxClockHz = reg::kDescribeMaxClockHz,
                  kTagMaxLength = reg::kDescribeMaxLength, kTagMinClockHz = reg::kDescribeMinClockHz,
                  kTagFeatures = reg::kDescribeFeatures,
                  kTagChannelGroup = reg::kDescribeChannelGroup, kTagOps = reg::kDescribeOps;
// fn 0's own describe tags: the probe itself (core §7.5)
constexpr uint8_t kCoreFirmware = reg::core::kTlvDescribeFirmware, kCoreModel = reg::core::kTlvDescribeModel,
                  kCoreUnitId = reg::core::kTlvDescribeUnitId, kCoreChannels = reg::core::kTlvDescribeChannels,
                  kCoreLabel = reg::core::kTlvDescribeLabel, kCoreChip = reg::core::kTlvDescribeChip;

// TLV tag bits (core §2.2): bit 7 = critical (in requests); 0x00 and 0x7F are never tags (0x00: the rejected
// unsupported payload's "fixed-part value" marker).
constexpr uint8_t kTagCritical = reg::kTagCritical;
constexpr uint8_t kTagValue = reg::kTagReservedZero;

inline uint16_t getU16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }
inline uint32_t getU32(const uint8_t *p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
inline void putU16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
inline void putU32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
inline uint64_t getU64(const uint8_t *p) { return getU32(p) | (static_cast<uint64_t>(getU32(p + 4)) << 32); }
inline void putU64(uint8_t *p, uint64_t v) { putU32(p, static_cast<uint32_t>(v)); putU32(p + 4, static_cast<uint32_t>(v >> 32)); }
// bit n of a registry kLockFreeOps mask = op n needs no lock
inline bool lockFreeIn(uint64_t mask, uint8_t op) { return op < 64 && ((mask >> op) & 1); }
// op among first .. last (an interface's op table, for Interface::offers)
constexpr bool opIn(uint8_t op, uint8_t first, uint8_t last) { return op >= first && op <= last; }

// One TLV (core §2.2): tag(u8) len(u16) value - one form whatever the value's length (0 to 65535 bytes).
constexpr size_t kTlvHeader = 3;
inline void putTlvHeader(uint8_t *p, uint8_t tag, uint16_t length) { p[0] = tag; putU16(p + 1, length); }
// The bytes a TLV of `length` takes with its header.
constexpr size_t tlvSize(size_t length) { return length + kTlvHeader; }

// The probe's one clock (core §2.6a): ns since boot, u64; it does not decrease and does not wrap while the boot_id is
// the same. fn 0's clock op (core §7.7), marks, segments and the slots' "last tried" all use it; "not yet" is all ones.
constexpr uint64_t kNeverNs = ~uint64_t{0};
#if !defined(ARDUINO_ARCH_ESP32) && !defined(ARDUINO_ARCH_RP2040)
// A platform with a 32-bit micros() (it wraps every 71.6 minutes): extended with a count of its wraps. Every read
// counts one it sees; the endpoint's poll() reads it each pass, far more often than once per wrap.
struct MicrosExtender {
  uint32_t last = 0, wraps = 0;
  uint64_t read(uint32_t us) {
    if (us < last) ++wraps;
    last = us;
    return static_cast<uint64_t>(wraps) << 32 | us;
  }
};
inline MicrosExtender g_micros_extender;
#endif
inline uint64_t nowNs() {
#if defined(ARDUINO_ARCH_ESP32)
  return static_cast<uint64_t>(esp_timer_get_time()) * 1000u;   // 64-bit
#elif defined(ARDUINO_ARCH_RP2040)
  return time_us_64() * 1000u;                                   // 64-bit
#else
  return g_micros_extender.read(micros()) * 1000u;
#endif
}

// The boot_id (core §6.5), in the order of preference the core gives: a hardware random source where the platform has
// one (ESP32 esp_random, RP2 hwrand32); elsewhere the count of the free-running microsecond timer when the first
// message arrives (an external event: the endpoint picks the value then, not at a fixed point of the start-up code),
// mixed so that nearby counts give unrelated values. Such a probe may repeat a boot_id; the host accepts that.
inline uint32_t mix32(uint32_t x) {   // MurmurHash3's finaliser
  x ^= x >> 16;
  x *= 0x85ebca6bu;
  x ^= x >> 13;
  x *= 0xc2b2ae35u;
  x ^= x >> 16;
  return x;
}
inline uint32_t bootIdSource() {
#if defined(ARDUINO_ARCH_ESP32)
  return esp_random();
#elif defined(ARDUINO_ARCH_RP2040)
  return rp2040.hwrand32();
#else
  return mix32(micros());
#endif
}

// A rejection with a one-byte payload (the tag of a rejected unsupported).
inline Result rejectedWith(uint8_t reason, uint8_t *out, size_t capacity, uint8_t value) {
  if (capacity < 1) return rejected(reason);
  out[0] = value;
  return {kResolutionRejected, reason, 1};
}
// rejected unsupported (core §4.3): payload tag(u8) [TLV] - 0x00 for a value of the fixed part, else the critical TLV's tag
// as received. unsupportedAt names which element of the request's list (TLV channel 0x02, index 0x40).
inline Result unsupportedValue(uint8_t *out, size_t capacity) { return rejectedWith(kRejectUnsupported, out, capacity, kTagValue); }
inline Result unsupportedTag(uint8_t *out, size_t capacity, uint8_t raw_tag) { return rejectedWith(kRejectUnsupported, out, capacity, raw_tag); }
inline Result unsupportedAt(uint8_t *out, size_t capacity, uint16_t channel, uint8_t index) {
  if (capacity < 10) return unsupportedValue(out, capacity);
  out[0] = kTagValue;
  putTlvHeader(out + 1, reg::core::kTlvUnsupportedPayloadChannel, 2);
  putU16(out + 4, channel);
  putTlvHeader(out + 6, reg::core::kTlvUnsupportedPayloadIndex, 1);
  out[9] = index;
  return {kResolutionRejected, kRejectUnsupported, 10};
}
// The same with the index alone (a list whose elements are not channels: a scan's combinations, oep-if-debug §1).
inline Result unsupportedIndex(uint8_t *out, size_t capacity, uint8_t index) {
  if (capacity < 5) return unsupportedValue(out, capacity);
  out[0] = kTagValue;
  putTlvHeader(out + 1, reg::core::kTlvUnsupportedPayloadIndex, 1);
  out[4] = index;
  return {kResolutionRejected, kRejectUnsupported, 5};
}

// One read_block / write_block's length (core §7.4 max_length; oep-if-debug §4.5, §6): bytes, a multiple of 4. The
// value a target declares must let both the read_block answer (header 5, done 2, status 1, words) and the write_block
// request (header 10, connection 2, address 4, count 2, words) fit the endpoint's max_frame - the spec's bound is
// max_frame - 24 - and fit the target's own word buffer (`buffer_bytes`; 0 = none, the words go straight through).
// Before the endpoint has told the frame limit (max_frame 0) the buffer alone bounds it.
inline uint16_t blockMaxLength(size_t max_frame, size_t buffer_bytes) {
  size_t bytes = max_frame ? (max_frame > 24 ? max_frame - 24 : 0) : 0xFFFC;
  if (buffer_bytes && bytes > buffer_bytes) bytes = buffer_bytes;
  if (bytes > 0xFFFC) bytes = 0xFFFC;
  return static_cast<uint16_t>(bytes / 4 * 4);
}
// A block op's count against the declared max_length: count x 4 over it is rejected unsupported with the fixed
// part's tag (unsupportedValue, payload 0x00); count 0 fits (success, done 0). An address that is not a multiple of 4
// is malformed, which the caller checks first (core §4.3's order).
inline bool blockCountFits(uint16_t count, uint16_t max_length) { return 4u * count <= max_length; }

// The firmware string every probe reports (fn 0's describe tag 0x40): the library's release version
// (openembeddedprobe_version.h, written by the release). A build between releases reports the last release.
constexpr const char *kFirmwareVersion = OPENEMBEDDEDPROBE_VERSION_STR;

// A token of `a-z 0-9 -` only, 1 to `max` bytes (core §7.5: unit_id; interface names' labels).
inline bool lowerToken(const uint8_t *p, size_t n, size_t max) {
  if (n == 0 || n > max) return false;
  for (size_t i = 0; i < n; ++i)
    if (!((p[i] >= 'a' && p[i] <= 'z') || (p[i] >= '0' && p[i] <= '9') || p[i] == '-')) return false;
  return true;
}
// An interface name (core §13 rule 1): 1 to 64 bytes, at least two labels separated by `.`, each label 1 or more of
// `a-z 0-9 -` that does not start or end with `-`.
inline bool interfaceName(const char *name) {
  const size_t n = name ? strlen(name) : 0;
  if (n == 0 || n > reg::kLimitInterfaceNameMaxBytes) return false;
  size_t labels = 0, start = 0;
  for (size_t i = 0; i <= n; ++i) {
    if (i < n && name[i] != '.') continue;
    const size_t len = i - start;
    if (len == 0 || name[start] == '-' || name[i - 1] == '-' ||
        !lowerToken(reinterpret_cast<const uint8_t *>(name + start), len, len))
      return false;
    ++labels;
    start = i + 1;
  }
  return labels >= 2;
}

// The TLV at `at` of p[0..n) (core §2.2): its tag, value and length, and where the next one starts. false: its header
// or its value runs past the end.
inline bool tlvAt(const uint8_t *p, size_t n, size_t at, uint8_t &tag, const uint8_t *&value, size_t &length, size_t &next) {
  if (at + kTlvHeader > n) return false;
  tag = p[at];
  length = getU16(p + at + 1);
  value = p + at + kTlvHeader;
  next = at + kTlvHeader + length;
  return next <= n;
}

// The TLVs after a request's fixed part (core §2.3). A handler checks the fixed part (shorter = malformed), then:
//
//   Tail tail;
//   const Result r = tail.parse(p + fixed, n - fixed, kKnown, out, capacity);   // kKnown: the tags (bit 7 clear) it implements
//   if (r.resolution != kResolutionCompleted) return r;                         // malformed, or unsupported + tag
//   ... tail.fixed(tag, size, value, ...) / tail.find(tag, len, &critical) for an implemented tag;
//   Tail::refuse(tag, critical, ...) for a value it does not handle
//
// An unknown tag (one this probe does not implement) that is critical rejects the request (unsupported, payload = the
// tag byte as received); an unknown non-critical one is ignored, nothing said about it. An implemented TLV is checked the
// same with or without bit 7: a value whose length differs from its definition, or a value the definition excludes, is
// malformed; a value the definition leaves unused or this probe does not handle is unsupported with the tag as received
// (core §2.3). A tag that appears twice where its definition does not say it repeats: the first one is used.
class Tail {
 public:
  // The whole tail's form is checked before any unknown critical tag is refused (one answer per request, core §4.3).
  // deferred: the unsupported refusal is not returned but stored there (the first one; completed when there is none),
  // for a handler with checks of its own that come first (plan_apply's fns).
  Result parse(const uint8_t *p, size_t n, const uint8_t *known, size_t known_count, uint8_t *out, size_t capacity,
               Result *deferred = nullptr) {
    p_ = p;
    n_ = n;
    size_t at = 0;
    uint8_t raw = 0;
    size_t len = 0;
    const uint8_t *value = nullptr;
    while (at < n)
      if (!step(at, raw, value, len)) return rejected(kRejectMalformed);
    if (deferred) *deferred = completed();
    for (at = 0; at < n && step(at, raw, value, len);) {
      const uint8_t tag = raw & ~kTagCritical;
      if (listed(known, known_count, tag) || !(raw & kTagCritical)) continue;   // unknown non-critical: ignored
      if (!deferred) return unsupportedTag(out, capacity, raw);
      if (deferred->resolution == kResolutionCompleted) *deferred = unsupportedTag(out, capacity, raw);
    }
    return completed();
  }
  Result parse(const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
    return parse(p, n, nullptr, 0, out, capacity);
  }
  template <size_t N>
  Result parse(const uint8_t *p, size_t n, const uint8_t (&known)[N], uint8_t *out, size_t capacity,
               Result *deferred = nullptr) {
    return parse(p, n, known, N, out, capacity, deferred);
  }

  // An implemented tag's value (its first occurrence, core §2.3), nullptr when absent. critical: whether the host marked it.
  const uint8_t *find(uint8_t tag, size_t &length, bool *critical = nullptr) const {
    size_t at = 0;
    uint8_t raw = 0;
    size_t len = 0;
    const uint8_t *value = nullptr;
    while (at < n_ && step(at, raw, value, len)) {
      if ((raw & ~kTagCritical) != (tag & ~kTagCritical)) continue;
      length = len;
      if (critical) *critical = raw & kTagCritical;
      return value;
    }
    return nullptr;
  }
  // An implemented TLV of one fixed `size` (core §2.3): `value` it when present with that length; any other length is
  // malformed, critical or not. completed() unless refused.
  Result fixed(uint8_t tag, size_t size, const uint8_t *&value, uint8_t *out, size_t capacity, bool *critical = nullptr) {
    (void)out;
    (void)capacity;
    size_t len = 0;
    bool crit = false;
    value = find(tag, len, &crit);
    if (critical) *critical = crit;
    if (!value) return completed();
    if (len != size) { value = nullptr; return rejected(kRejectMalformed); }
    return completed();
  }
  // Every TLV in order (for a request that is a list of them, like plan_apply). false at the end.
  bool next(size_t &at, uint8_t &raw, const uint8_t *&value, size_t &length) const {
    return at < n_ && step(at, raw, value, length);
  }
  // An implemented tag whose value this probe does not handle (or the definition leaves unused): rejected unsupported,
  // the tag byte as received (bit 7 as the host sent it), whether or not it was critical (core §2.3).
  static Result refuse(uint8_t tag, bool critical, uint8_t *out, size_t capacity) {
    return unsupportedTag(out, capacity, critical ? tag | kTagCritical : tag & ~kTagCritical);
  }

 private:
  const uint8_t *p_ = nullptr;
  size_t n_ = 0;
  static bool listed(const uint8_t *tags, size_t count, uint8_t tag) {
    for (size_t i = 0; i < count; ++i) if ((tags[i] & ~kTagCritical) == tag) return true;
    return false;
  }
  bool step(size_t &at, uint8_t &raw, const uint8_t *&value, size_t &length) const {
    size_t next = 0;
    if (!tlvAt(p_, n_, at, raw, value, length, next)) return false;
    at = next;
    return true;
  }
};

// The common case: a request with a fixed part of `fixed` bytes and a tail of TLVs none of which this op reads.
inline Result plainTail(Tail &tail, const uint8_t *p, size_t n, size_t fixed, uint8_t *out, size_t capacity) {
  if (n < fixed) return rejected(kRejectMalformed);
  return tail.parse(p + fixed, n - fixed, out, capacity);
}

// Appends TLVs (tag, len(u16), value; core §2.2) to a fixed buffer; ok() stays false once something did not fit.
class TlvWriter {
 public:
  TlvWriter(uint8_t *buffer, size_t capacity) : p_(buffer), cap_(capacity) {}
  bool put(uint8_t tag, const void *value, size_t length) {
    if (!ok_ || length > 0xFFFF || n_ + tlvSize(length) > cap_) return ok_ = false;
    putTlvHeader(p_ + n_, tag, static_cast<uint16_t>(length));
    n_ += kTlvHeader;
    if (length) memcpy(p_ + n_, value, length);
    n_ += length;
    return true;
  }
  bool u8(uint8_t tag, uint8_t v) { return put(tag, &v, 1); }
  bool u16(uint8_t tag, uint16_t v) { uint8_t b[2]; putU16(b, v); return put(tag, b, 2); }
  bool u32(uint8_t tag, uint32_t v) { uint8_t b[4]; putU32(b, v); return put(tag, b, 4); }
  bool text(uint8_t tag, const char *s) { return put(tag, s, strlen(s)); }
  void fail() { ok_ = false; }   // a value the caller found it cannot write: the whole answer is unusable
  // A list of u8 with its count first (core §2.3: n(u8), n x u8).
  bool u8List(uint8_t tag, const uint8_t *values, size_t count) {
    uint8_t b[1 + 255];
    if (count > 255) return ok_ = false;
    b[0] = static_cast<uint8_t>(count);
    memcpy(b + 1, values, count);
    return put(tag, b, 1 + count);
  }
  // fn 0's label (0x46): a channel name the firmware (the wiring) fixes; the settings' labels are probe.config's.
  bool label(uint16_t channel, const char *name) {
    uint8_t b[2 + 32];
    const size_t n = strlen(name) < 32 ? strlen(name) : 32;
    putU16(b, channel);
    memcpy(b + 2, name, n);
    return put(kCoreLabel, b, 2 + n);
  }
  // A wire's fixed pin set as a channel group: group 1, n roles, role 1 SWDIO (or SWIO), role 2 SWCLK (none on a one-wire
  // link). core §7.4: group(u8) n(u8) n x (role(u8) channel(u16)).
  bool pinGroup(uint16_t swdio, uint16_t swclk) {
    uint8_t group[8] = {1, 2, 1, 0, 0, 2, 0, 0};
    putU16(group + 3, swdio);
    putU16(group + 6, swclk);
    if (swclk == 0xffff) group[1] = 1;
    return put(kTagChannelGroup, group, swclk == 0xffff ? 5 : sizeof group);
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

// The resource numbers (core §9): one u16 space for the whole probe (connections, streams, whatever an interface numbers),
// each new one the previous plus 1 (65535 then 1 again), a number in use skipped. Live numbers are remembered with their
// kind so a request naming a live number of another kind is refused unavailable cause 6, and a number nobody holds
// no_connection.
class ResourceNumbers {
 public:
  enum Kind : uint8_t { kNone = 0, kConnection = 1, kStream = 2 };
  static constexpr size_t kLive = 16;
  // The next free number, registered live as `kind` (0 when every number is live - not in practice).
  static uint16_t take(Kind kind) {
    for (uint32_t tries = 0; tries < 0x10000; ++tries) {
      const uint16_t n = next_++;
      if (next_ == 0) next_ = 1;
      if (n == 0 || kindOf(n) != kNone) continue;
      for (Live &l : live_) if (l.kind == kNone) { l = {n, kind}; return n; }
      return 0;
    }
    return 0;
  }
  static void close(uint16_t number) {
    for (Live &l : live_) if (l.kind != kNone && l.number == number) l.kind = kNone;
  }
  // Re-register a number that was closed (a console stream opened again at the same place keeps its number).
  static bool reopen(uint16_t number, Kind kind) {
    if (kindOf(number) != kNone) return kindOf(number) == kind;
    for (Live &l : live_) if (l.kind == kNone) { l = {number, kind}; return true; }
    return false;
  }
  static Kind kindOf(uint16_t number) {
    for (const Live &l : live_) if (l.kind != kNone && l.number == number) return l.kind;
    return kNone;
  }
  // As at boot: nothing live, the next number 1 (a host test that plays a probe from boot).
  static void reset() {
    next_ = 1;
    for (Live &l : live_) l = {0, kNone};
  }
  // The rejection for a number that is not the resource asked for: another live kind = unavailable cause 6, else
  // no_connection (core §4.3).
  static Result refuse(uint16_t number, Kind wanted, uint8_t *out, size_t capacity) {
    const Kind k = kindOf(number);
    if (k != kNone && k != wanted) {
      if (capacity < 4) return rejected(kRejectUnavailable);
      putTlvHeader(out, reg::core::kTlvUnavailablePayloadCause, 1);
      out[3] = reg::core::kUnavailableCauseWrongState;
      return {kResolutionRejected, kRejectUnavailable, 4};
    }
    return rejected(kRejectNoConnection);
  }

 private:
  struct Live { uint16_t number; Kind kind; };
  static inline uint16_t next_ = 1;
  static inline Live live_[kLive] = {};
};

// One offered interface. The endpoint gives it fn = 1, 2, ... in registration order.
class Interface {
 public:
  virtual ~Interface() = default;
  virtual const char *name() const = 0;
  virtual uint16_t instance() const = 0;
  // The payload shapes are fixed by (name, revision) (core §2.7); every oep.* interface in this library is revision 1.
  virtual uint8_t revision() const { return 0; }
  virtual uint8_t flags() const { return 0; }
  // The whole describe as TLV bytes (declarations only: nothing that changes while the probe runs, core §7.3); the
  // endpoint pages it by whole TLVs.
  virtual size_t describe(uint8_t *out, size_t capacity) { (void)out; (void)capacity; return 0; }
  // Operations that change nothing may run without the lock (and without a session id).
  virtual bool lockFree(uint8_t op) const { (void)op; return false; }
  // Whether this interface offers op (core §1.2): every required op of its document's table, an optional one only when
  // this probe has it. The endpoint declares exactly these in the describe's ops tag (0x09, core §7.4) - it writes
  // that tag itself, first, in its one canonical encoding (base = the lowest op, the last bitmap byte non-zero) - and
  // answers any other op unknown_operation before it looks at the session (core §4.3 order 1). Every interface says
  // which ops it has (an interface of the sketch's own too): the default offers none, and an interface that offers no op
  // at all has no ops encoding and is not added (Endpoint::add). subscribe / unsubscribe (0x30 / 0x32) are not asked
  // here: notifies() puts both in the ops.
  virtual bool offers(uint8_t op) const { (void)op; return false; }
  virtual Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) = 0;
  // Whether this interface has plan roles (oep-if-plan: roles its document assigns through the plan; the pins a wire's
  // attach selects by argument are not). The endpoint lists oep.probe.plan when one of its interfaces has some, and not
  // otherwise; a plan_apply naming a fn without them is refused unsupported. An interface that overrides planCheck /
  // planApply says true.
  virtual bool planRoles() const { return false; }
  // Pin plan (oep.probe.plan plan_apply / plan_release): check without side effects (0 = acceptable, else a reject
  // reason), apply, undo. A session's plan goes with its session; one the settings put in stays.
  virtual uint8_t planCheck(const RoleAssignment *roles, size_t count) {
    (void)roles;
    return count ? kRejectUnavailable : 0;
  }
  // The cause (core §4.3 unavailable payload) of the last planCheck that answered unavailable when none of its channels
  // is held by something else (the endpoint reports a held channel itself: cause 1, the channel).
  virtual uint8_t planRefusalCause() const { return reg::core::kUnavailableCauseWrongState; }
  // ... and the channel it met, when the cause has one (0xFFFF: left out) - an analog channel whose idle is an output:
  // cause 5, the channel (core §8, oep-if-capture §1.2).
  virtual void planRefusalDetail(uint16_t &channel) const { (void)channel; }
  virtual bool planApply(const RoleAssignment *roles, size_t count) { (void)roles; (void)count; return true; }
  virtual void planRelease() {}
  // false: this interface's planned channels are shared with no other fn's plan (core §8.1) - an analog input that
  // takes its pad from the digital side (oep-if-capture §1.2). The endpoint refuses the overlap whichever comes second.
  virtual bool planShares() const { return true; }
  // The capture-group fn this interface is bound into, 0 = none (oep-if-capture §4.1): plan_apply and plan_release of
  // its fn are refused unavailable cause 4, before anything changes.
  virtual uint16_t boundTo() const { return 0; }
  // The lock holder's session ended - end, its lease lapsed, or another host took the lock by force, all alike (core
  // §6.4, §9): drop everything that session created or shared (a wire: the host's use of its connections; a console:
  // the host's share of its streams; a capture-group: its bind). What the settings keep (a slot's connection, a bind's
  // stream) stays. Nothing passes to the next session.
  virtual void sessionOver() {}
  // true: this interface's resources sit on another interface's (a console's streams on a wire's connections): at the
  // session's end the endpoint releases its share first (oep-if-console §2: the stream closes with session_ended).
  virtual bool sessionOverFirst() const { return false; }
  // The probe restarts right after this (oep.probe.restart, oep-if-restart §2), its session already over (sessionOver ran): let go
  // of what the settings keep as well - a slot's connection, a bind's stream - without touching the target (no reset;
  // a halted hart stays halted), so that no line is driven when the chip resets. sessionOverFirst orders it the same
  // way. The endpoint releases every plan after it, the settings' too.
  virtual void probeRestart() {}
  // The endpoint's frame limit, told when the interface is added: what a describe may promise.
  virtual void setFrameLimit(size_t max_frame) { (void)max_frame; }
  // Notifications (core §11): an interface that sends them says true here, and the endpoint puts subscribe and
  // unsubscribe (0x30 / 0x32, core §11.3) in its ops and answers them itself (the lock holder's; min_bytes and
  // max_delay_ms batch the data frames only, an event goes out as soon as the answers ahead of it have). One that sends
  // none has neither op: a subscribe to it is unknown_operation.
  virtual bool notifies() const { return false; }
  // The lock holder subscribed (true) or unsubscribed / its session ended (false); called only when notifies(). While
  // subscribed, the endpoint asks for a data frame's payload (pull): write up to `capacity` bytes of it in core §11.2's
  // form - position(u64) len(u16) data [TLV] - and return the length; 0 = nothing now. The return value is not used.
  virtual bool subscribe(bool on) { (void)on; return false; }
  virtual size_t pull(uint8_t *out, size_t capacity) {
    (void)out; (void)capacity;
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

// The part of fn 0's describe every probe writes the same way: firmware, model, unit id and channel count. The sketch
// adds its fixed labels and chip after it; the endpoint adds the transports and max_op_ms (plan_roles and restart_max_ms
// are oep.probe.plan's and oep.probe.restart's describes). unit_id is mandatory (core §7.5, 1 to 32 bytes of a-z 0-9 -):
// without one the writer fails rather than send a describe that leaves it out. model is free text (left out when empty).
// firmware_note: put after the version in the firmware text, in parentheses, when not empty (the reference firmware's
// BootGuard::lastBoot(): what ended the boot before this one - this implementation's, docs/implementation-limits §2.2).
inline bool describeCore(TlvWriter &w, const char *model, const uint8_t *unit_id, size_t unit_id_length,
                         uint16_t channels, const char *firmware_note = nullptr) {
  if (firmware_note && *firmware_note) {
    char firmware[128];
    snprintf(firmware, sizeof firmware, "%s (%s)", kFirmwareVersion, firmware_note);
    w.text(kCoreFirmware, firmware);
  } else {
    w.text(kCoreFirmware, kFirmwareVersion);
  }
  if (model && *model) w.text(kCoreModel, model);
  if (!lowerToken(unit_id, unit_id_length, reg::kLimitUnitIdMaxBytes)) {
    w.fail();
    return false;
  }
  w.put(kCoreUnitId, unit_id, unit_id_length);
  return w.u16(kCoreChannels, channels);
}

// rejected unavailable with core §4.3's payload: why (cause, 0 = left out), the channel it met (0xFFFF = left out), then
// an interface's own TLVs or the capture-group's fn (`extra`). Hosts may ignore all of it.
inline Result unavailable(uint8_t *out, size_t capacity, uint8_t cause, uint16_t channel = 0xFFFF,
                          const uint8_t *extra = nullptr, size_t extra_length = 0) {
  TlvWriter w(out, capacity);
  if (cause) w.u8(reg::core::kTlvUnavailablePayloadCause, cause);
  if (channel != 0xFFFF) w.u16(reg::core::kTlvUnavailablePayloadChannel, channel);
  if (extra && extra_length && w.length() + extra_length <= capacity) { memcpy(out + w.length(), extra, extra_length); }
  const size_t n = w.ok() ? w.length() + (extra && w.length() + extra_length <= capacity ? extra_length : 0) : 0;
  return {kResolutionRejected, kRejectUnavailable, n};
}
// rejected unavailable cause 6: not in the state for it (not configured, running, bound, another kind of resource).
inline Result wrongState(uint8_t *out, size_t capacity) {
  return unavailable(out, capacity, reg::core::kUnavailableCauseWrongState);
}

}  // namespace oep
