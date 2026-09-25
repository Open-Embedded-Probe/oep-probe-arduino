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

void Endpoint::send(size_t length) {
  wrote_ = true;
  if (framing_ == Framing::kCobsCrc) writeCobsFrame(stream_, tx_, length);
  else writeFrame(stream_, tx_, length);
}

// Pushes go at a lower priority than results: poll() answers what has arrived first, and a push is only written
// into the room the transport has free right now, so it never blocks and a result is never queued behind more than
// what already sits in the transport's own buffer. One push frame per subscribed fn per poll.
bool Endpoint::queueEvent(uint16_t fn, uint8_t kind, const uint8_t *payload, size_t length) {
  if (length > sizeof(Event::payload)) return false;
  if (event_head_ - event_tail_ >= kEvents) ++event_tail_;   // full: the oldest goes (its seq never appears)
  Event &e = events_[event_head_ % kEvents];
  e.fn = fn;
  e.seq = fn == 0 ? core_seq_++ : takeSeq(fn);   // numbered when queued: one dropped from the queue leaves a gap
  e.kind = kind;
  e.length = static_cast<uint8_t>(length);
  memcpy(e.payload, payload, length);
  ++event_head_;
  return true;
}

bool Endpoint::event(Interface &from, uint8_t kind, const uint8_t *payload, size_t length) {
  for (size_t i = 0; i < count_; ++i)
    if (interfaces_[i] == &from) return subscribed_[i] && queueEvent(static_cast<uint16_t>(i + 1), kind, payload, length);
  return false;
}

void Endpoint::eventsLost(Interface &from, uint16_t count) {
  for (size_t i = 0; i < count_; ++i)
    if (interfaces_[i] == &from && subscribed_[i]) __atomic_fetch_add(&push_seq_[i], count, __ATOMIC_RELAXED);
}

bool Endpoint::directPush(const Interface &from, uint16_t &fn, uint16_t &min_bytes, uint16_t &max_delay_ms) const {
  for (size_t i = 0; i < count_; ++i) {
    if (interfaces_[i] != &from) continue;
    if (!subscribed_[i] || !locked_) return false;
    fn = static_cast<uint16_t>(i + 1);
    min_bytes = min_bytes_[i];
    max_delay_ms = max_delay_ms_[i];
    return true;
  }
  return false;
}

// Events go before data (they are small and usually what someone waits for). false: no room right now.
bool Endpoint::sendEvents() {
  while (event_tail_ != event_head_) {
    const Event &e = events_[event_tail_ % kEvents];
    const int need = static_cast<int>(kEventHeader + e.length) + (framing_ == Framing::kCobsCrc ? 8 : 2);
    if (stream_.availableForWrite() < need) return false;
    tx_[0] = kRoleEvent;
    putU16(tx_ + 1, e.fn);
    putU16(tx_ + 3, e.seq);
    tx_[5] = e.kind;
    memcpy(tx_ + kEventHeader, e.payload, e.length);
    ++event_tail_;
    send(kEventHeader + e.length);
  }
  return true;
}

void Endpoint::push() {
  lapse();
  if (!locked_) return;
  if (heartbeat_ && static_cast<uint32_t>(millis() - heartbeat_last_) >= heartbeat_ms_) {
    heartbeat_last_ = millis();
    uint8_t hb[8];
    putU32(hb, boot_id_);
    putU32(hb + 4, heartbeat_last_);
    queueEvent(0, kEventHeartbeat, hb, sizeof hb);
  }
  if (!sendEvents()) return;
  size_t room = tx_capacity_ - kPushHeader;
  if (room > static_cast<size_t>(limits_.max_frame) - kPushHeader) room = limits_.max_frame - kPushHeader;
  for (size_t i = 0; i < count_; ++i) {
    if (!subscribed_[i]) continue;
    // batching: wait for min_bytes, or max_delay_ms after the first byte became pending
    if (min_bytes_[i] || max_delay_ms_[i]) {
      const size_t ready = interfaces_[i]->pending();
      if (ready == 0) { waiting_[i] = false; continue; }
      if (!waiting_[i]) { waiting_[i] = true; waiting_since_[i] = millis(); }
      if (ready < min_bytes_[i] && static_cast<uint32_t>(millis() - waiting_since_[i]) < max_delay_ms_[i]) continue;
    }
    const int writable = stream_.availableForWrite();
    // Keep at most push_queue_ bytes waiting in the transport: a result queues behind no more than that. The
    // transport's capacity is taken as the most room ever seen free (idle).
    if (writable > tx_room_max_) tx_room_max_ = writable;
    const int queued = tx_room_max_ - writable;
    if (push_queue_ && queued >= static_cast<int>(push_queue_)) return;
    // length prefix (2) or COBS/CRC overhead (about 1 per 254 + 2 CRC + delimiter) on top of the header
    const size_t overhead = kPushHeader + (framing_ == Framing::kCobsCrc ? 8 : 2);
    if (writable <= static_cast<int>(overhead) + 16) return;
    size_t free_room = static_cast<size_t>(writable) - overhead;
    if (framing_ == Framing::kCobsCrc) free_room -= free_room / 254;
    size_t cap = room < free_room ? room : free_room;
    if (push_queue_ && cap + overhead + queued > push_queue_) {
      const int left = static_cast<int>(push_queue_) - queued - static_cast<int>(overhead);
      if (left < 16) return;
      cap = static_cast<size_t>(left);
    }
    uint32_t position = 0;
    const size_t n = interfaces_[i]->pull(position, tx_ + kPushHeader, cap);
    if (n == 0) continue;
    tx_[0] = kRolePush;
    putU16(tx_ + 1, static_cast<uint16_t>(i + 1));
    putU16(tx_ + 3, push_seq_[i]++);
    putU32(tx_ + 5, position);
    send(kPushHeader + n);
    waiting_[i] = false;
  }
}

// subscribe(fn u16 [, min_bytes u16, max_delay_ms u16]); fn 0 = heartbeat events, max_delay_ms = the period.
Result Endpoint::subscription(uint8_t op, const uint8_t *payload, size_t length) {
  if (length != 2 && !(op == kOpSubscribe && length == 6)) return rejected(kRejectMalformed);
  const uint16_t fn = getU16(payload);
  if (fn == 0) {
    heartbeat_ = op == kOpSubscribe;
    if (length == 6 && getU16(payload + 4)) heartbeat_ms_ = getU16(payload + 4);
    heartbeat_last_ = millis() - heartbeat_ms_;
    return completed();
  }
  if (fn > count_) return rejected(kRejectUnknownFunction);
  const size_t i = fn - 1;
  if (op == kOpUnsubscribe) {
    if (subscribed_[i]) interfaces_[i]->subscribe(false);
    subscribed_[i] = false;
    return completed();
  }
  if (!interfaces_[i]->subscribe(true)) return rejected(kRejectUnavailable);
  subscribed_[i] = true;
  push_seq_[i] = 0;
  min_bytes_[i] = length == 6 ? getU16(payload + 2) : 0;
  max_delay_ms_[i] = length == 6 ? getU16(payload + 4) : 0;
  waiting_[i] = false;
  return completed();
}

void Endpoint::poll() {
  if (framing_ == Framing::kCobsCrc) {
    while (stream_.available()) {
      if (!cobs_.push(static_cast<uint8_t>(stream_.read()))) continue;
      handleMessage(cobs_.message(), cobs_.length());
      cobs_.consume();
    }
  } else {
    uint8_t chunk[512];
    for (int avail; (avail = stream_.available()) > 0;) {
      size_t n = static_cast<size_t>(avail) < sizeof chunk ? static_cast<size_t>(avail) : sizeof chunk;
      for (size_t i = 0; i < n; ++i) chunk[i] = static_cast<uint8_t>(stream_.read());
      const uint8_t *p = chunk;
      while (n) {
        if (!reader_.feed(p, n)) continue;
        handleMessage(reader_.message(), reader_.length());
        reader_.consume();
      }
    }
  }
  push();   // after the results for everything that has arrived
  if (flush_after_burst_ && wrote_) stream_.flush();
  wrote_ = false;
}

void Endpoint::lapse() {
  if (locked_ && static_cast<int32_t>(millis() - expires_ms_) >= 0) {
    locked_ = false;   // the last id stays
    endSubscriptions();
  }
}

// Subscriptions belong to the lock: a host that vanished stops being pushed to when its lease lapses.
void Endpoint::endSubscriptions() {
  heartbeat_ = false;
  event_tail_ = event_head_;
  for (size_t i = 0; i < count_; ++i) {
    if (subscribed_[i]) interfaces_[i]->subscribe(false);
    subscribed_[i] = false;
  }
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
  send(kResultHeader + result.length);
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
    case kOpLinkSource: {   // length(u32) -> that many bytes (as many as fit one frame), byte k = k & 0xff
      if (length != 4) return rejected(kRejectMalformed);
      size_t n = getU32(payload);
      if (n > capacity) n = capacity;
      static uint8_t pattern[256];
      if (pattern[255] != 255) for (size_t k = 0; k < 256; ++k) pattern[k] = static_cast<uint8_t>(k);
      for (size_t at = 0; at < n; at += 256) memcpy(out + at, pattern, n - at < 256 ? n - at : 256);
      return completed(n);
    }
    case kOpLinkSink:       // any bytes -> how many arrived (u32)
      if (capacity < 4) return failed();
      putU32(out, static_cast<uint32_t>(length));
      return completed(4);
    case kOpDescribe: return describe(payload, length, out, capacity);
    case kOpOpen: return open(payload, length, out, capacity);
    case kOpLockState:
      if (capacity < 5) return failed();
      out[0] = locked_;
      putU32(out + 1, remaining());
      return completed(5);
    case kOpStatus: return rejected(kRejectUnavailable);   // no long operations yet (they block)
    case kOpSubscribe:
    case kOpUnsubscribe: {   // experimental: the lock holder only, and they end with the lock
      const Result check = checkSession(has_session, session, out, capacity);
      if (check.resolution != kResolutionCompleted) return check;
      return subscription(op, payload, length);
    }
    case kOpEnd:
    case kOpKeepalive:
    case kOpCancel:
    case kOpPlanApply:
    case kOpPlanRelease: {
      const Result check = checkSession(has_session, session, out, capacity);
      if (check.resolution != kResolutionCompleted) return check;
      if (op == kOpEnd) { locked_ = false; endSubscriptions(); }   // the last id stays: the same host may resume
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
