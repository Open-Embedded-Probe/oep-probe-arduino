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
size_t pageTlv(const uint8_t *tlv, size_t length, uint16_t first, uint8_t *out, size_t capacity, bool &more) {
  size_t at = 0, n = 0, used = 0;
  uint16_t index = 0;
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

bool Endpoint::addTransport(Stream &stream, uint8_t *rx_buffer, size_t rx_capacity, Framing framing,
                            bool flush_after_burst) {
  if (transport_count_ >= kMaxTransports) return false;
  Transport &t = transports_[transport_count_++];
  t.stream = &stream;
  t.reader.reset(rx_buffer, rx_capacity, limits_.max_frame);
  t.cobs.reset(rx_buffer, rx_capacity);
  t.framing = framing;
  t.flush_after_burst = flush_after_burst;
  return true;
}

void Endpoint::send(size_t length) {
  Transport &t = transports_[current_];
  t.wrote = true;
  if (t.framing == Framing::kCobsCrc) writeCobsFrame(*t.stream, tx_, length);
  else writeFrame(*t.stream, tx_, length);
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
    const Transport &t = transports_[current_];
    const int need = static_cast<int>(kEventHeader + e.length) + (t.framing == Framing::kCobsCrc ? 8 : 2);
    if (t.stream->availableForWrite() < need) return false;
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
      // 0 = that condition is not used (v1 wire §4.5): send on min_bytes ready, or max_delay_ms after the first byte
      const bool enough = min_bytes_[i] && ready >= min_bytes_[i];
      const bool due = max_delay_ms_[i] && static_cast<uint32_t>(millis() - waiting_since_[i]) >= max_delay_ms_[i];
      if (!enough && !due) continue;
    }
    Transport &t = transports_[current_];
    const int writable = t.stream->availableForWrite();
    // Keep at most push_queue_ bytes waiting in the transport: a result queues behind no more than that. The
    // transport's capacity is taken as the most room ever seen free (idle).
    if (writable > t.tx_room_max) t.tx_room_max = writable;
    const int queued = t.tx_room_max - writable;
    if (push_queue_ && queued >= static_cast<int>(push_queue_)) return;
    // length prefix (2) or COBS/CRC overhead (about 1 per 254 + 2 CRC + delimiter) on top of the header
    const size_t overhead = kPushHeader + (t.framing == Framing::kCobsCrc ? 8 : 2);
    if (writable <= static_cast<int>(overhead) + 16) return;
    size_t free_room = static_cast<size_t>(writable) - overhead;
    if (t.framing == Framing::kCobsCrc) free_room -= free_room / 254;
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

// subscribe(fn u16, min_bytes u16, max_delay_ms u16) [TLV]; unsubscribe(fn u16) [TLV]. fn 0 = heartbeat events,
// max_delay_ms = the period (0: 1000 ms). seq starts again at 0 with every subscribe. No field is optional (v1 wire
// §0: an optional fixed field could not be told apart from a tail).
Result Endpoint::subscription(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  const bool batching = op == kOpSubscribe;
  const size_t fixed = batching ? 6 : 2;
  if (length < fixed) return rejected(kRejectMalformed);
  Tail tail;
  const Result parsed = tail.parse(payload + fixed, length - fixed, out, capacity);
  if (parsed.resolution != kResolutionCompleted) return parsed;
  const uint16_t fn = getU16(payload);
  const uint16_t min_bytes = batching ? getU16(payload + 2) : 0, max_delay = batching ? getU16(payload + 4) : 0;
  if (op == kOpSubscribe) push_ = current_;   // pushes and events go where the subscription came from
  if (fn == 0) {
    heartbeat_ = op == kOpSubscribe;
    if (heartbeat_) {
      heartbeat_ms_ = max_delay ? max_delay : reg::kHeartbeatDefaultMs;
      heartbeat_last_ = millis() - heartbeat_ms_;
      core_seq_ = 0;
    }
    return tail.finish(completed(), out, capacity);
  }
  if (fn > count_) return rejected(kRejectUnknownFunction);
  const size_t i = fn - 1;
  if (op == kOpUnsubscribe) {
    if (subscribed_[i]) interfaces_[i]->subscribe(false);
    subscribed_[i] = false;
    return tail.finish(completed(), out, capacity);
  }
  if (!interfaces_[i]->subscribe(true)) return rejected(kRejectUnavailable);
  subscribed_[i] = true;
  push_seq_[i] = 0;
  min_bytes_[i] = min_bytes;
  max_delay_ms_[i] = max_delay;
  waiting_[i] = false;
  return tail.finish(completed(), out, capacity);
}

void Endpoint::poll() {
  for (size_t i = 0; i < transport_count_; ++i) {
    Transport &t = transports_[i];
    current_ = i;   // results go back on this transport
    if (t.framing == Framing::kCobsCrc) {
      while (t.stream->available()) {
        if (!t.cobs.push(static_cast<uint8_t>(t.stream->read()))) continue;
        handleMessage(t.cobs.message(), t.cobs.length());
        t.cobs.consume();
      }
    } else {
      uint8_t chunk[2048];
      for (int avail; (avail = t.stream->available()) > 0;) {
        size_t n = static_cast<size_t>(avail) < sizeof chunk ? static_cast<size_t>(avail) : sizeof chunk;
        n = t.stream->readBytes(chunk, n);   // a stream that copies in bulk (DirectBulkStream, CDC) does it here
        if (!n) break;
        const uint8_t *p = chunk;
        while (n) {
          if (!t.reader.feed(p, n)) continue;
          handleMessage(t.reader.message(), t.reader.length());
          t.reader.consume();
        }
      }
    }
  }
  current_ = push_;
  push();   // after the results for everything that has arrived, on the subscriber's transport
  for (size_t i = 0; i < transport_count_; ++i) {
    Transport &t = transports_[i];
    if (t.flush_after_burst && t.wrote) t.stream->flush();
    t.wrote = false;
  }
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

namespace {
inline bool refused(const Result &r) { return r.resolution != kResolutionCompleted; }
}  // namespace

Result Endpoint::core(uint8_t op, bool has_session, uint32_t session, const uint8_t *payload, size_t length,
                      uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case kOpConfirm: {
      // "OEP?" min_rev(u8) max_rev(u8) [TLV]  ->  "OEP!" revision(u8) flags(u8) max_frame(u16) window(u32) max_inflight(u8)
      if (length < 6 || memcmp(payload, reg::kConfirmRequestMagic, 4) != 0) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 6, length - 6, out, capacity);
      if (refused(parsed)) return parsed;
      if (reg::kProtocolRevision < payload[4] || reg::kProtocolRevision > payload[5]) return rejected(kRejectUnsupported);
      if (capacity < 13) return failed();
      memcpy(out, reg::kConfirmResultMagic, 4);
      out[4] = reg::kProtocolRevision;
      out[5] = 0;                                     // flags: reserved
      putU16(out + 6, limits_.max_frame);
      putU32(out + 8, limits_.window_bytes);
      out[12] = limits_.max_inflight;
      return tail.finish(completed(13), out, capacity);
    }
    case kOpList: return list(payload, length, out, capacity);
    case kOpLinkSource: {   // length(u32) [TLV] -> that many bytes (as many as fit one frame), byte k = k & 0xff
      const Result parsed = plainTail(tail, payload, length, 4, out, capacity);
      if (refused(parsed)) return parsed;
      size_t n = getU32(payload);
      if (n > capacity) n = capacity;
      static uint8_t pattern[256];
      if (pattern[255] != 255) for (size_t k = 0; k < 256; ++k) pattern[k] = static_cast<uint8_t>(k);
      for (size_t at = 0; at < n; at += 256) memcpy(out + at, pattern, n - at < 256 ? n - at : 256);
      return completed(n);   // closed tail: nothing can follow the bytes
    }
    case kOpLinkSink:       // any bytes (no tail) -> how many arrived (u32)
      if (capacity < 4) return failed();
      putU32(out, static_cast<uint32_t>(length));
      return completed(4);
    case kOpDescribe: return describe(payload, length, out, capacity);
    case kOpOpen: return open(payload, length, out, capacity);
    case kOpLockState: {
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 5) return failed();
      out[0] = locked_;
      putU32(out + 1, remaining());
      return tail.finish(completed(5), out, capacity);
    }
    case kOpStatus: {   // activity(u16): no long operations yet (they block)
      const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
      if (refused(parsed)) return parsed;
      return rejected(kRejectUnavailable);
    }
    case kOpSubscribe:
    case kOpUnsubscribe: {   // the lock holder only, and they end with the lock
      const Result check = checkSession(has_session, session, out, capacity);
      if (refused(check)) return check;
      return subscription(op, payload, length, out, capacity);
    }
    case kOpEnd:
    case kOpKeepalive:
    case kOpCancel:
    case kOpPlanApply:
    case kOpPlanRelease: {
      const Result check = checkSession(has_session, session, out, capacity);
      if (refused(check)) return check;
      if (op == kOpPlanApply) return planApply(payload, length, out, capacity);
      const Result parsed = plainTail(tail, payload, length, op == kOpCancel ? 2 : 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (op == kOpCancel) return rejected(kRejectUnavailable);   // nothing that could be stopped
      if (op == kOpEnd) { locked_ = false; endSubscriptions(); }   // the last id stays: the same host may resume
      if (op == kOpPlanRelease) planRelease();
      return tail.finish(completed(), out, capacity);
    }
    default: return rejected(kRejectUnknownOperation);
  }
}

// session_id(u32) lease_ms(u32) force(u8) [TLV]  ->  lease_ms(u32) boot_id(u32) resumed(u8)
Result Endpoint::open(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  const Result parsed = plainTail(tail, payload, length, 9, out, capacity);
  if (refused(parsed)) return parsed;
  if (capacity < 9) return failed();
  const uint32_t session = getU32(payload), lease = getU32(payload + 4);
  const bool force = payload[8];
  if (locked_ && holder_ != session && !force) return lockedFor(remaining(), out, capacity);
  // Taken over by force: the previous holder's subscriptions end with its lock (§4.5).
  if (locked_ && holder_ != session) endSubscriptions();
  const bool resumed = (locked_ && holder_ == session) || (have_last_ && last_ == session);
  locked_ = true;
  holder_ = last_ = session;
  have_last_ = true;
  lease_ms_ = lease == 0 ? kLeaseDefaultMs : (lease > kLeaseMaxMs ? kLeaseMaxMs : lease);
  expires_ms_ = millis() + lease_ms_;
  putU32(out, lease_ms_);
  putU32(out + 4, boot_id_);
  out[8] = resumed;
  return tail.finish(completed(9), out, capacity);
}

// Role assignments as TLVs 0x90 = fn(u16) role(u8) channel(u16) (critical); every interface checks its own roles
// without side effects, then all apply or none does. An unknown critical TLV refuses the plan (unsupported), an
// unknown non-critical one is ignored and listed. One plan at a time; it is probe state and stays until
// plan_release (no session end or lapse releases it).
Result Endpoint::planApply(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (plan_active_) return rejected(kRejectUnavailable);
  static const uint8_t kKnown[] = {kTagRoleAssignment};
  Tail tail;
  const Result parsed = tail.parse(payload, length, kKnown, out, capacity);
  if (refused(parsed)) return parsed;
  RoleAssignment roles[kMaxRoles];
  size_t count = 0, at = 0;
  uint8_t raw = 0, len = 0;
  const uint8_t *v = nullptr;
  while (tail.next(at, raw, v, len)) {
    if ((raw & ~kTagCritical) != (kTagRoleAssignment & ~kTagCritical)) continue;
    if (len != 5 || count >= kMaxRoles) return rejected(kRejectMalformed);
    roles[count] = {getU16(v), v[2], getU16(v + 3)};
    if (roles[count].function == 0 || roles[count].function > count_) return rejected(kRejectUnknownFunction);
    ++count;
  }
  if (!count) return rejected(kRejectMalformed);
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
  memcpy(plan_roles_, roles, count * sizeof roles[0]);
  plan_count_ = count;
  return tail.finish(completed(), out, capacity);
}

void Endpoint::planRelease() {
  for (size_t i = 0; i < count_; ++i) {
    if (planned_[i]) interfaces_[i]->planRelease();
    planned_[i] = false;
  }
  plan_active_ = false;
  plan_count_ = 0;
}

size_t Endpoint::plan(RoleAssignment *out, size_t max) const {
  const size_t n = plan_count_ < max ? plan_count_ : max;
  memcpy(out, plan_roles_, n * sizeof out[0]);
  return n;
}

uint8_t Endpoint::replacePlan(const RoleAssignment *roles, size_t count) {
  if (count > kMaxRoles) return kRejectMalformed;
  auto apply = [this](const RoleAssignment *r, size_t n) -> uint8_t {
    if (!n) return 0;
    uint8_t tlv[kMaxRoles * 7], scratch[16];
    for (size_t i = 0; i < n; ++i) {
      uint8_t *t = tlv + i * 7;
      t[0] = kTagRoleAssignment;
      t[1] = 5;
      putU16(t + 2, r[i].function);
      t[4] = r[i].role;
      putU16(t + 5, r[i].channel);
    }
    const Result res = planApply(tlv, n * 7, scratch, sizeof scratch);
    if (res.resolution == kResolutionRejected) return res.detail;
    return res.detail == kOutcomeSuccess ? 0 : kRejectUnavailable;
  };
  RoleAssignment before[kMaxRoles];
  const size_t had = plan(before, kMaxRoles);
  planRelease();
  const uint8_t reason = apply(roles, count);
  if (reason) apply(before, had);
  return reason;
}

Result Endpoint::list(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // flags(u8, bit0 exact) first(u16) prefix_len(u8) prefix [TLV]  ->  total(u16) count(u8) entries
  // entry: fn(u16) instance(u16) revision(u8) flags(u8) name_len(u8) name; oep.core (fn 0) is the first one
  if (length < 4 || length < 4u + payload[3] || capacity < 3) return rejected(kRejectMalformed);
  Tail tail;
  const Result parsed = tail.parse(payload + 4 + payload[3], length - 4 - payload[3], out, capacity);
  if (refused(parsed)) return parsed;
  const bool exact = payload[0] & 1;
  const uint16_t first = getU16(payload + 1);
  const uint8_t n = payload[3];
  const uint8_t *prefix = payload + 4;
  uint16_t total = 0;
  uint8_t count = 0;
  size_t used = 3;
  bool full = false;
  // room for the ignored TLV after the entries, if there is one to report
  const size_t room = tail.anyIgnored() && capacity > 2 + Tail::kMaxIgnored ? capacity - 2 - Tail::kMaxIgnored : capacity;
  auto consider = [&](uint16_t fn, uint16_t instance, uint8_t revision, uint8_t flags, const char *name) {
    if (!nameMatches(name, prefix, n, exact)) return;
    if (total++ < first || full) return;
    const size_t name_len = strlen(name);
    if (used + 7 + name_len > room || count == 255) { full = true; return; }
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
  consider(0, 0, reg::core::kRevision, 0, reg::core::kName);
  for (size_t i = 0; i < count_; ++i) {
    Interface &it = *interfaces_[i];
    consider(static_cast<uint16_t>(i + 1), it.instance(), it.revision(), it.flags(), it.name());
  }
  putU16(out, total);
  out[2] = count;
  return tail.finish(completed(used), out, capacity);
}

Result Endpoint::describe(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // fn(u16) first(u16) [TLV]  ->  more(u8) TLVs (first = the index of the first TLV)
  if (length < 4 || capacity < 1) return rejected(kRejectMalformed);
  Tail tail;
  const Result parsed = tail.parse(payload + 4, length - 4, out, capacity);
  if (refused(parsed)) return parsed;
  const uint16_t fn = getU16(payload);
  const uint16_t first = getU16(payload + 2);
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
  const size_t room = tail.anyIgnored() && capacity > 3 + Tail::kMaxIgnored ? capacity - 2 - Tail::kMaxIgnored : capacity;
  bool more = false;
  const size_t used = tlv ? pageTlv(tlv, tlv_length, first, out + 1, room - 1, more) : 0;
  out[0] = more;
  return tail.finish(completed(1 + used), out, capacity);
}

}  // namespace v1
}  // namespace oep
