#include "OepV1Endpoint.h"

#include <string.h>

namespace oep {
namespace v1 {
namespace {

Result lockedFor(uint32_t remaining_ms, uint8_t *out, size_t capacity) {
  if (capacity < 4) return rejected(kRejectLocked);
  putU32(out, remaining_ms);
  return {kResolutionRejected, kRejectLocked, 4};
}

// Label-boundary prefix match: "oep.fixture.uart" matches "oep.fixture.uart" and "oep.fixture.uart.stream",
// not "oep.fixture.uart2". An empty prefix matches everything.
bool nameMatches(const char *name, const uint8_t *prefix, size_t n, bool exact) {
  const size_t len = strlen(name);
  if (exact) return len == n && memcmp(name, prefix, n) == 0;
  if (n == 0) return true;
  return len >= n && memcmp(name, prefix, n) == 0 && (len == n || name[n] == '.');
}

// Page a TLV stream by whole TLVs: skip `first`, then copy while they fit. Returns bytes, sets more.
size_t pageTlv(const uint8_t *tlv, size_t length, uint8_t first, uint8_t *out, size_t capacity, bool &more) {
  size_t at = 0, n = 0, used = 0;
  uint8_t index = 0;
  more = false;
  while (at + 2 <= length) {
    const size_t size = 2 + tlv[at + 1];
    if (at + size > length) break;
    if (index >= first) {
      if (used + size > capacity) { more = true; break; }
      memcpy(out + used, tlv + at, size);
      used += size;
      ++n;
    }
    at += size;
    ++index;
  }
  (void)n;
  return used;
}

}  // namespace

bool Endpoint::add(Interface &interface) {
  if (count_ >= kMaxInterfaces) return false;
  interfaces_[count_++] = &interface;
  interface.setFrameLimit(limits_.max_frame);
  return true;
}

void Endpoint::poll() {
  while (stream_.available()) {
    const uint8_t byte = static_cast<uint8_t>(stream_.read());
    if (framing_ == Framing::kCobsCrc) {
      if (!cobs_.push(byte)) continue;
      handleMessage(cobs_.message(), cobs_.length());
      cobs_.consume();
    } else {
      if (!reader_.push(byte)) continue;
      handleMessage(reader_.message(), reader_.length());
      reader_.consume();
    }
  }
}

void Endpoint::lapse() {
  if (locked_ && static_cast<int32_t>(millis() - expires_ms_) >= 0) locked_ = false;   // the last id stays
}

uint32_t Endpoint::remaining() const {
  if (!locked_) return 0;
  const int32_t left = static_cast<int32_t>(expires_ms_ - millis());
  return left > 0 ? static_cast<uint32_t>(left) : 0;
}

Result Endpoint::checkSession(bool has_session, uint32_t session, uint8_t *out, size_t capacity) {
  if (!has_session) return rejected(kRejectSessionRequired);
  if (!locked_) {
    if (have_last_ && session == last_) {   // resume: nobody else came in between
      locked_ = true;
      holder_ = session;
      return completed();
    }
    return rejected(kRejectNoSession);
  }
  if (session == holder_) return completed();
  return lockedFor(remaining(), out, capacity);   // never the holder's id
}

void Endpoint::handleMessage(const uint8_t *message, size_t length) {
  if (length < kRequestHeader || (message[0] & ~kRoleSession) != kRoleRequest) return;   // other roles: drop
  const bool has_session = message[0] & kRoleSession;
  const uint16_t corr = getU16(message + 1), fn = getU16(message + 3);
  const uint8_t op = message[5];
  size_t header = kRequestHeader;
  uint32_t session = 0;
  if (has_session) {
    if (length < kRequestHeader + kSessionBytes) return;
    session = getU32(message + kRequestHeader);
    header += kSessionBytes;
  }
  const uint8_t *payload = message + header;
  const size_t payload_length = length - header;
  uint8_t *out = tx_ + kResultHeader;
  size_t capacity = tx_capacity_ - kResultHeader;
  if (capacity > static_cast<size_t>(limits_.max_frame) - kResultHeader) capacity = limits_.max_frame - kResultHeader;

  lapse();
  Result result;
  if (fn == 0) {
    result = core(op, has_session, session, payload, payload_length, out, capacity);
  } else if (fn > count_) {
    result = rejected(kRejectUnknownFunction);
  } else {
    Interface &it = *interfaces_[fn - 1];
    result = it.lockFree(op) ? completed() : checkSession(has_session, session, out, capacity);
    if (result.resolution == kResolutionCompleted) result = it.handle(op, payload, payload_length, out, capacity);
  }
  if (result.length > capacity) result = failed(0);
  // The lease runs from when the holder's request completed (a long verify must not lapse its own lock).
  if (has_session && locked_ && session == holder_) expires_ms_ = millis() + lease_ms_;
  tx_[0] = kRoleResult;
  putU16(tx_ + 1, corr);
  tx_[3] = result.resolution;
  tx_[4] = result.detail;
  if (framing_ == Framing::kCobsCrc) writeCobsFrame(stream_, tx_, kResultHeader + result.length);
  else writeFrame(stream_, tx_, kResultHeader + result.length);
}

Result Endpoint::core(uint8_t op, bool has_session, uint32_t session, const uint8_t *payload, size_t length,
                      uint8_t *out, size_t capacity) {
  switch (op) {
    case kOpConfirm:   // -> magic, revision, max_frame, window, max_inflight
      if (capacity < 10) return failed();
      memcpy(out, "OEP!", 4);
      out[4] = 1;
      putU16(out + 5, limits_.max_frame);
      putU16(out + 7, limits_.window_bytes);
      out[9] = limits_.max_inflight;
      return completed(10);
    case kOpList: return list(payload, length, out, capacity);
    case kOpDescribe: return describe(payload, length, out, capacity);
    case kOpOpen: return open(payload, length, out, capacity);
    case kOpLockState:
      if (capacity < 5) return failed();
      out[0] = locked_;
      putU32(out + 1, remaining());
      return completed(5);
    case kOpStatus: return rejected(kRejectUnavailable);   // no long operations yet (they block)
    case kOpEnd:
    case kOpKeepalive:
    case kOpCancel:
    case kOpPlanApply:
    case kOpPlanRelease: {
      const Result check = checkSession(has_session, session, out, capacity);
      if (check.resolution != kResolutionCompleted) return check;
      if (op == kOpEnd) locked_ = false;   // the last id stays: the same host may resume
      if (op == kOpCancel) return rejected(kRejectUnavailable);
      if (op == kOpPlanApply) return planApply(payload, length, out, capacity);
      if (op == kOpPlanRelease) planRelease();
      return completed();
    }
    default: return rejected(kRejectUnknownOperation);
  }
}

Result Endpoint::open(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length != 9 || capacity < 9) return rejected(kRejectMalformed);
  const uint32_t session = getU32(payload), lease = getU32(payload + 4);
  const bool force = payload[8];
  if (locked_ && holder_ != session && !force) return lockedFor(remaining(), out, capacity);
  const bool resumed = (locked_ && holder_ == session) || (have_last_ && last_ == session);
  locked_ = true;
  holder_ = last_ = session;
  have_last_ = true;
  lease_ms_ = lease == 0 ? kLeaseDefaultMs : (lease > kLeaseMaxMs ? kLeaseMaxMs : lease);
  expires_ms_ = millis() + lease_ms_;
  putU32(out, lease_ms_);
  putU32(out + 4, boot_id_);
  out[8] = resumed;
  return completed(9);
}

// The v0 plan, unchanged: role assignments as critical TLVs 0x90 = fn(u16) role(u8) channel(u16); every
// interface checks its own roles without side effects, then all apply or none does. One plan at a time; it is
// probe state and stays until plan_release (no session end or lapse releases it).
Result Endpoint::planApply(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  (void)out; (void)capacity;
  if (plan_active_) return rejected(kRejectUnavailable);
  constexpr size_t kMaxRoles = 16;
  RoleAssignment roles[kMaxRoles];
  size_t count = 0, at = 0;
  while (at + 2 <= length) {
    const uint8_t tag = payload[at], len = payload[at + 1];
    if (at + 2 + len > length) return rejected(kRejectMalformed);
    const uint8_t *v = payload + at + 2;
    if (tag == kTagRoleAssignment) {
      if (len != 5 || count >= kMaxRoles) return rejected(kRejectMalformed);
      roles[count] = {getU16(v), v[2], getU16(v + 3)};
      if (roles[count].function == 0 || roles[count].function > count_) return rejected(kRejectUnknownFunction);
      ++count;
    } else if (tag & 0x80) {
      return rejected(kRejectMalformed);   // critical and unknown: refuse the whole plan
    }
    at += 2 + len;
  }
  if (!count || at != length) return rejected(kRejectMalformed);
  bool wants[kMaxInterfaces] = {};
  for (size_t i = 0; i < count_; ++i) {
    RoleAssignment mine[kMaxRoles];
    size_t n = 0;
    for (size_t r = 0; r < count; ++r) if (roles[r].function == i + 1) mine[n++] = roles[r];
    if (!n) continue;
    const uint8_t reason = interfaces_[i]->planCheck(mine, n);
    if (reason) return rejected(reason);
    wants[i] = true;
  }
  for (size_t i = 0; i < count_; ++i) {
    if (!wants[i]) continue;
    RoleAssignment mine[kMaxRoles];
    size_t n = 0;
    for (size_t r = 0; r < count; ++r) if (roles[r].function == i + 1) mine[n++] = roles[r];
    if (!interfaces_[i]->planApply(mine, n)) {
      for (size_t j = 0; j < i; ++j) if (planned_[j]) { interfaces_[j]->planRelease(); planned_[j] = false; }
      return failed();
    }
    planned_[i] = true;
  }
  plan_active_ = true;
  return completed();
}

void Endpoint::planRelease() {
  for (size_t i = 0; i < count_; ++i) {
    if (planned_[i]) interfaces_[i]->planRelease();
    planned_[i] = false;
  }
  plan_active_ = false;
}

Result Endpoint::list(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // flags(u8, bit0 exact) first(u8) prefix_len(u8) prefix  ->  total(u8) count(u8) entries
  if (length < 3 || length != 3u + payload[2] || capacity < 2) return rejected(kRejectMalformed);
  const bool exact = payload[0] & 1;
  const uint8_t first = payload[1], n = payload[2];
  const uint8_t *prefix = payload + 3;
  uint8_t total = 0, count = 0;
  size_t used = 2;
  bool full = false;
  auto consider = [&](uint16_t fn, uint16_t instance, uint8_t revision, uint8_t flags, const char *name) {
    if (!nameMatches(name, prefix, n, exact)) return;
    if (total++ < first || full) return;
    const size_t name_len = strlen(name);
    if (used + 7 + name_len > capacity) { full = true; return; }
    uint8_t *e = out + used;
    putU16(e, fn);
    putU16(e + 2, instance);
    e[4] = revision;
    e[5] = flags;
    e[6] = static_cast<uint8_t>(name_len);
    memcpy(e + 7, name, name_len);
    used += 7 + name_len;
    ++count;
  };
  consider(0, 0, 1, 0, "oep.core");
  for (size_t i = 0; i < count_; ++i) {
    Interface &it = *interfaces_[i];
    consider(static_cast<uint16_t>(i + 1), it.instance(), it.revision(), it.flags(), it.name());
  }
  out[0] = total;
  out[1] = count;
  return completed(used);
}

Result Endpoint::describe(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // fn(u16) first(u8)  ->  more(u8) TLVs
  if (length != 3 || capacity < 1) return rejected(kRejectMalformed);
  const uint16_t fn = getU16(payload);
  const uint8_t first = payload[2];
  const uint8_t *tlv = nullptr;
  size_t tlv_length = 0;
  if (fn == 0) {
    tlv = probe_tlv_;
    tlv_length = probe_tlv_length_;
  } else if (fn <= count_) {
    tlv_length = interfaces_[fn - 1]->describe(scratch_, sizeof scratch_);
    tlv = scratch_;
  } else {
    return rejected(kRejectUnknownFunction);
  }
  bool more = false;
  const size_t used = tlv ? pageTlv(tlv, tlv_length, first, out + 1, capacity - 1, more) : 0;
  out[0] = more;
  return completed(1 + used);
}

}  // namespace v1
}  // namespace oep
