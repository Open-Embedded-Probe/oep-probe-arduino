#include "OepEndpoint.h"

#include <string.h>

namespace oep {
namespace {

// rejected locked: remaining ms (u32), then the holder's owner TLV when it gave one (core §4.3, §6.4)
Result lockedFor(uint32_t remaining_ms, const uint8_t *owner, size_t owner_length, uint8_t *out, size_t capacity) {
  if (capacity < 4) return rejected(kRejectLocked);
  putU32(out, remaining_ms);
  size_t n = 4;
  if (owner_length && capacity >= 6 + owner_length) {
    out[4] = reg::core::kTlvLockedPayloadOwner;
    out[5] = static_cast<uint8_t>(owner_length);
    memcpy(out + 6, owner, owner_length);
    n += 2 + owner_length;
  }
  return {kResolutionRejected, kRejectLocked, n};
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

bool Endpoint::addTransport(Stream &stream, uint8_t *rx_buffer, size_t rx_capacity, uint8_t kind, uint8_t usb_interface,
                            bool flush_after_burst) {
  if (transport_count_ >= kMaxTransports) return false;
  if (serialKind(kind) && limits_.max_frame > kMaxSerialFrame) return false;
  Transport &t = transports_[transport_count_];
  t.stream = &stream;
  t.kind = kind;
  t.usb_interface = usb_interface;
  t.index = static_cast<uint8_t>(transport_count_);
  t.owner = this;
  if (serialKind(kind)) t.serial.reset(rx_buffer, rx_capacity, decode_, sizeof decode_);
  else t.reader.reset(rx_buffer, rx_capacity, limits_.max_frame);
  t.flush_after_burst = flush_after_burst;
  ++transport_count_;
  return true;
}

void Endpoint::send(size_t length) {
  Transport &t = transports_[current_];
  t.wrote = true;
  if (serialKind(t.kind)) writeCobsFrame(*t.stream, tx_, length);
  else writeFrame(*t.stream, tx_, length);
}

// A serial port's raw bytes (core §3.4): to the binds, unless a session holds the port (then dropped).
void Endpoint::rawSink(void *context, const uint8_t *data, size_t length) {
  Transport &t = *static_cast<Transport *>(context);
  Endpoint &e = *t.owner;
  if (e.raw_ && !e.held(t.index)) e.raw_->rawIn(t.index, data, length);
}

// The binds' bytes out on every serial port not held, into the room the port has now beyond what a frame needs (a
// result never waits behind raw bytes, and a port nobody reads holds its position instead of losing bytes).
void Endpoint::rawOut() {
  if (!raw_) return;
  raw_->poll(locked_);
  const int reserve = static_cast<int>(cobsFrameMax(limits_.max_frame));
  for (size_t i = 0; i < transport_count_; ++i) {
    Transport &t = transports_[i];
    if (!serialKind(t.kind) || held(i)) continue;
    int room = t.stream->availableForWrite() - reserve;
    uint8_t chunk[64];
    while (room > 0) {
      const size_t n = raw_->rawOut(static_cast<uint8_t>(i), chunk, static_cast<size_t>(room) < sizeof chunk ? room : sizeof chunk);
      if (!n) break;
      const size_t sent = t.stream->write(chunk, n);
      raw_->rawSent(static_cast<uint8_t>(i), sent);
      t.wrote = true;
      if (sent < n) break;   // the port is full: the rest waits (its position holds)
      room -= static_cast<int>(n);
    }
  }
}

// The session ended (end, lapse, force): the ports it held take up the raw transfer again (from its last host reset).
void Endpoint::sessionEnded() {
  if (raw_) raw_->sessionOver(held_);
  held_ = 0;
}

size_t Endpoint::appendOwner(uint8_t *out, size_t room, uint8_t tag) const {
  if (!owner_length_ || room < 2u + owner_length_) return 0;
  out[0] = tag;
  out[1] = owner_length_;
  memcpy(out + 2, owner_, owner_length_);
  return 2u + owner_length_;
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
    const int need = static_cast<int>(serialKind(t.kind) ? cobsFrameMax(kEventHeader + e.length) : kEventHeader + e.length + 2);
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
      // 0 = that condition is not used (core §11.3): send on min_bytes ready, or max_delay_ms after the first byte
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
    const size_t overhead = kPushHeader + (serialKind(t.kind) ? 9 : 2);
    if (writable <= static_cast<int>(overhead) + 16) return;
    size_t free_room = static_cast<size_t>(writable) - overhead;
    if (serialKind(t.kind)) free_room -= free_room / 254;
    size_t cap = room < free_room ? room : free_room;
    if (push_queue_ && cap + overhead + queued > push_queue_) {
      const int left = static_cast<int>(push_queue_) - queued - static_cast<int>(overhead);
      if (left < 16) return;
      cap = static_cast<size_t>(left);
    }
    const size_t n = interfaces_[i]->pull(tx_ + kPushHeader, cap);   // the payload is the interface's
    if (n == 0) continue;
    tx_[0] = kRolePush;
    putU16(tx_ + 1, static_cast<uint16_t>(i + 1));
    putU16(tx_ + 3, push_seq_[i]++);
    send(kPushHeader + n);
    waiting_[i] = false;
  }
}

// subscribe(fn u16, min_bytes u16, max_delay_ms u16) [TLV]; unsubscribe(fn u16) [TLV]. fn 0 = heartbeat events,
// max_delay_ms = the period (0: 1000 ms). seq starts again at 0 with every subscribe. No field is optional (core
// §2.3: an optional fixed field could not be told apart from a tail).
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
    if (serialKind(t.kind)) {
      uint8_t chunk[512];
      for (int avail; (avail = t.stream->available()) > 0;) {
        size_t n = static_cast<size_t>(avail) < sizeof chunk ? static_cast<size_t>(avail) : sizeof chunk;
        n = t.stream->readBytes(chunk, n);
        if (!n) break;
        const uint8_t *p = chunk;
        while (n) {
          if (t.serial.feed(p, n, rawSink, &t)) handleMessage(t.serial.message(), t.serial.length());
        }
      }
      t.serial.idle(rawSink, &t);
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
  rawOut();   // then the serial ports' raw bytes, never inside a frame
  for (size_t i = 0; i < transport_count_; ++i) {
    Transport &t = transports_[i];
    if (t.flush_after_burst && t.wrote) t.stream->flush();
    t.wrote = false;
  }
}

void Endpoint::lapse() {
  if (locked_ && static_cast<int32_t>(millis() - expires_ms_) >= 0) {
    locked_ = false;   // the last id stays
    loseSession();
    sessionEnded();
  }
}

// The lock holder is gone (its lease lapsed, or another host took the lock by force): what its session made goes -
// subscriptions, its use of connections, and a plan it applied (the pins back to safe: released). A plan set through
// oep.probe.config stays (probe settings, not a session's). An explicit end keeps all of it for the next session.
// core §9 (decided 2026-09-26).
void Endpoint::loseSession() {
  endSubscriptions();
  for (size_t i = 0; i < count_; ++i) interfaces_[i]->sessionLapsed();
  uint16_t lapsed[kMaxInterfaces];
  size_t n = 0;
  for (size_t i = 0; i < count_; ++i) if (planned_[i] && !persistent_[i]) lapsed[n++] = static_cast<uint16_t>(i + 1);
  if (n) planRelease(lapsed, n);   // n == 0 would mean every fn: only the session's plans go
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
  return lockedFor(remaining(), owner_, owner_length_, out, capacity);   // never the holder's id
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
  // The last session's request sent again (the host lost the result): answer from what was kept, never run it twice,
  // and before the session is judged (core §5.2) - an end sent again must not take the lock back. The host numbers
  // its requests in order (core §4.1): a corr no newer than the newest seen and not in the table was dropped from it.
  const bool mine = has_session && have_last_ && session == last_ && !(fn == 0 && op == kOpOpen);
  const uint32_t crc = mine ? crc32(payload, payload_length) : 0;
  if (mine) {
    for (Dedup &d : dedup_) {
      if (!d.used || d.corr != corr) continue;
      if (d.fn != fn || d.op != op || d.crc != crc) { sendReject(corr, kRejectCorrReused); return; }
      if (!d.kept) { sendReject(corr, kRejectResultLost); return; }
      memcpy(tx_, d.result, d.length);
      send(d.length);
      return;
    }
    if (have_newest_ && static_cast<int16_t>(corr - newest_corr_) <= 0) { sendReject(corr, kRejectResultLost); return; }
  }
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
  // core §3.4: a serial port the lock holder's requests come in on (its open too) stops its raw transfer
  if (serialKind(transports_[current_].kind) && locked_ &&
      ((fn == 0 && op == kOpOpen && result.resolution == kResolutionCompleted) || (has_session && session == holder_)))
    held_ |= uint32_t{1} << current_;
  // The lease runs from when the holder's request completed (a long verify must not lapse its own lock).
  if (has_session && locked_ && session == holder_) expires_ms_ = millis() + lease_ms_;
  tx_[0] = kRoleResult;
  putU16(tx_ + 1, corr);
  tx_[3] = result.resolution;
  tx_[4] = result.detail;
  const size_t total = kResultHeader + result.length;
  if (mine) {   // keep it for a request sent again (an open drops the table)
    newest_corr_ = corr;
    have_newest_ = true;
    Dedup &d = dedup_[dedup_next_];
    dedup_next_ = (dedup_next_ + 1) % kDedupEntries;
    d.used = true;
    d.corr = corr;
    d.fn = fn;
    d.op = op;
    d.crc = crc;
    d.kept = total <= kDedupBytes;
    d.length = d.kept ? static_cast<uint8_t>(total) : 0;
    if (d.kept) memcpy(d.result, tx_, total);
  }
  send(total);
}

void Endpoint::sendReject(uint16_t corr, uint8_t reason) {
  tx_[0] = kRoleResult;
  putU16(tx_ + 1, corr);
  tx_[3] = kResolutionRejected;
  tx_[4] = reason;
  send(kResultHeader);
}

uint32_t Endpoint::crc32(const uint8_t *data, size_t length) {   // IEEE, reflected (the key of a kept result)
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    c ^= data[i];
    for (int b = 0; b < 8; ++b) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
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
      out[0] = locked_;   // any session holds it (who asked is not known: lock_state goes without one)
      putU32(out + 1, remaining());
      const size_t owner = locked_ ? appendOwner(out + 5, capacity - 5, reg::core::kTlvLockStateAnswerOwner) : 0;
      return tail.finish(completed(5 + owner), out, capacity);
    }
    case kOpSubscribe:
    case kOpUnsubscribe: {   // the lock holder only, and they end with the lock
      const Result check = checkSession(has_session, session, out, capacity);
      if (refused(check)) return check;
      return subscription(op, payload, length, out, capacity);
    }
    case kOpEnd:
    case kOpKeepalive:
    case kOpPlanApply:
    case kOpPlanRelease: {
      const Result check = checkSession(has_session, session, out, capacity);
      if (refused(check)) return check;
      if (op == kOpPlanApply) return planApply(payload, length, out, capacity);
      if (op == kOpPlanRelease) {   // n(u8) n x fn(u16) [TLV]; n = 0: every fn
        if (length < 1) return rejected(kRejectMalformed);
        const size_t n = payload[0];
        const Result parsed = plainTail(tail, payload, length, 1 + 2 * n, out, capacity);
        if (refused(parsed)) return parsed;
        uint16_t fns[256];
        for (size_t k = 0; k < n; ++k) fns[k] = getU16(payload + 1 + 2 * k);
        planRelease(fns, n);
        return tail.finish(completed(), out, capacity);
      }
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (op == kOpEnd) { locked_ = false; endSubscriptions(); sessionEnded(); }   // the last id stays: it may resume
      return tail.finish(completed(), out, capacity);
    }
    default: return rejected(kRejectUnknownOperation);
  }
}

// session_id(u32) lease_ms(u32) force(u8) [TLV 0x01 owner]  ->  lease_ms(u32) boot_id(u32) resumed(u8)
// lease_ms 0 = the default; 1000..60000 are taken as asked (core §6.4), longer ones cut to kLeaseMaxMs.
Result Endpoint::open(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  static const uint8_t kKnown[] = {reg::core::kTlvOpenOwner};
  if (length < 9) return rejected(kRejectMalformed);
  Tail tail;
  const Result parsed = tail.parse(payload + 9, length - 9, kKnown, out, capacity);
  if (refused(parsed)) return parsed;
  if (capacity < 9) return failed();
  const uint32_t session = getU32(payload), lease = getU32(payload + 4);
  const bool force = payload[8];
  if (locked_ && holder_ != session && !force) return lockedFor(remaining(), owner_, owner_length_, out, capacity);
  uint8_t owner_len = 0;
  bool owner_critical = false;
  const uint8_t *owner = tail.find(reg::core::kTlvOpenOwner, owner_len, &owner_critical);
  if (owner && (owner_len == 0 || owner_len > sizeof owner_)) {   // 1..32 bytes: a value this probe cannot keep
    const Result r = tail.refuse(reg::core::kTlvOpenOwner, owner_critical, out, capacity);
    if (refused(r)) return r;
    owner = nullptr;
  }
  // Taken over by force: the previous holder loses what its session made, as at a lapse (core §6.4).
  if (locked_ && holder_ != session) { loseSession(); sessionEnded(); }
  if (!(have_last_ && last_ == session)) owner_length_ = 0;   // another session: the old owner goes
  if (owner) { memcpy(owner_, owner, owner_len); owner_length_ = owner_len; }
  const bool resumed = (locked_ && holder_ == session) || (have_last_ && last_ == session);
  // Every open (a resume too) drops the kept results: a one-shot CLI resumes the session with corr from 1 again, and
  // must not get a previous process's result (core §5.2).
  for (Dedup &d : dedup_) d.used = false;
  have_newest_ = false;
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

// Role assignments as TLVs 0x90 = fn(u16) role(u8) channel(u16) (critical). The plan is per fn (oep-core §8): the fns
// the request names are replaced, every other fn keeps its plan. An unknown critical TLV refuses the plan (unsupported),
// an unknown non-critical one is ignored and listed. A session's plan: an explicit end keeps it, a lapse or a takeover
// releases it; a plan set through oep.probe.config (replacePlan, persistent) stays.
Result Endpoint::planApply(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
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
  uint16_t fns[kMaxInterfaces];
  size_t nfns = 0;
  for (size_t r = 0; r < count; ++r) {
    bool seen = false;
    for (size_t k = 0; k < nfns; ++k) seen |= fns[k] == roles[r].function;
    if (!seen) fns[nfns++] = roles[r].function;
  }
  const uint16_t reason = replaceFns(roles, count, fns, nfns, false);
  if (reason > 0xff) return failed();
  if (reason) return rejected(static_cast<uint8_t>(reason));
  return tail.finish(completed(), out, capacity);
}

// Replaces the plan of the fns listed (a listed fn with no role is released). All or nothing: the old plans of those
// fns are taken off first (so a replacement may reuse its own pins), every interface checks without side effects, then
// all apply; a refusal or a failed apply puts the old plans back. 0 = done; a reject reason; or
// kRejectUnavailable + 0x100 when an interface accepted the check and then failed to apply (completed failed).
uint16_t Endpoint::replaceFns(const RoleAssignment *roles, size_t count, const uint16_t *fns, size_t nfns,
                              bool persistent) {
  bool listed[kMaxInterfaces] = {};
  for (size_t k = 0; k < nfns; ++k) if (fns[k] >= 1 && fns[k] <= count_) listed[fns[k] - 1] = true;
  RoleAssignment old[kMaxRoles];
  size_t old_count = 0;
  bool old_persistent[kMaxInterfaces] = {};
  size_t keep = 0;
  for (size_t r = 0; r < plan_count_; ++r) {   // the listed fns' old roles come out of the table
    const uint16_t f = plan_roles_[r].function;
    if (listed[f - 1]) old[old_count++] = plan_roles_[r];
    else plan_roles_[keep++] = plan_roles_[r];
  }
  plan_count_ = keep;
  for (size_t i = 0; i < count_; ++i) {
    if (!listed[i]) continue;
    old_persistent[i] = persistent_[i];
    if (planned_[i]) { interfaces_[i]->planRelease(); planned_[i] = false; }
  }
  auto applyAll = [this, &listed](const RoleAssignment *set, size_t n, bool check) -> uint16_t {
    for (size_t i = 0; i < count_; ++i) {
      if (!listed[i]) continue;
      RoleAssignment mine[kMaxRoles];
      size_t m = 0;
      for (size_t r = 0; r < n; ++r) if (set[r].function == i + 1) mine[m++] = set[r];
      if (!m) continue;
      if (check) {
        if (const uint8_t reason = interfaces_[i]->planCheck(mine, m)) return reason;
      } else {
        if (!interfaces_[i]->planApply(mine, m)) return kRejectUnavailable + 0x100;
        planned_[i] = true;
      }
    }
    return 0;
  };
  auto undo = [&]() {   // the new ones off, the old ones back (they fitted before)
    for (size_t i = 0; i < count_; ++i)
      if (listed[i] && planned_[i]) { interfaces_[i]->planRelease(); planned_[i] = false; }
    applyAll(old, old_count, false);
    for (size_t r = 0; r < old_count && plan_count_ < kMaxRoles; ++r) plan_roles_[plan_count_++] = old[r];
    for (size_t i = 0; i < count_; ++i) if (listed[i]) persistent_[i] = old_persistent[i];
  };
  if (plan_count_ + count > kMaxRoles) { undo(); return kRejectMalformed; }
  if (const uint16_t reason = applyAll(roles, count, true)) { undo(); return reason; }
  if (const uint16_t reason = applyAll(roles, count, false)) { undo(); return reason; }
  for (size_t r = 0; r < count; ++r) plan_roles_[plan_count_++] = roles[r];
  for (size_t i = 0; i < count_; ++i) if (listed[i]) persistent_[i] = persistent && planned_[i];
  return 0;
}

// plan_release: the fns listed (none = every fn); a fn with no plan is left alone.
void Endpoint::planRelease(const uint16_t *fns, size_t nfns) {
  for (size_t i = 0; i < count_; ++i) {
    bool hit = nfns == 0;
    for (size_t k = 0; k < nfns; ++k) hit |= fns[k] == i + 1;
    if (!hit) continue;
    if (planned_[i]) interfaces_[i]->planRelease();
    planned_[i] = persistent_[i] = false;
  }
  size_t keep = 0;
  for (size_t r = 0; r < plan_count_; ++r)
    if (planned_[plan_roles_[r].function - 1]) plan_roles_[keep++] = plan_roles_[r];
  plan_count_ = keep;
}

size_t Endpoint::plan(RoleAssignment *out, size_t max, bool persistent_only) const {
  size_t n = 0;
  for (size_t r = 0; r < plan_count_ && n < max; ++r)
    if (!persistent_only || persistent_[plan_roles_[r].function - 1]) out[n++] = plan_roles_[r];
  return n;
}

uint8_t Endpoint::replacePlan(const RoleAssignment *roles, size_t count, const uint16_t *fns, size_t nfns) {
  const uint16_t reason = replaceFns(roles, count, fns, nfns, true);
  return reason > 0xff ? static_cast<uint8_t>(kRejectUnavailable) : static_cast<uint8_t>(reason);
}

uint32_t Endpoint::listHash() const {
  uint32_t c = 0xFFFFFFFFu;
  auto feed = [&c](const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      c ^= p[i];
      for (int b = 0; b < 8; ++b) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
  };
  auto entry = [&feed](uint16_t fn, uint16_t instance, uint8_t revision, const char *name) {
    uint8_t e[5];
    putU16(e, fn);
    putU16(e + 2, instance);
    e[4] = revision;
    feed(e, 5);
    feed(reinterpret_cast<const uint8_t *>(name), strlen(name));
  };
  entry(0, 0, reg::core::kRevision, reg::core::kName);
  for (size_t i = 0; i < count_; ++i)
    entry(static_cast<uint16_t>(i + 1), interfaces_[i]->instance(), interfaces_[i]->revision(), interfaces_[i]->name());
  return ~c;
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
  if (fn == 0) {   // the sketch's part, then the transports and oep_pid (core §7.5)
    if (probe_tlv_length_ <= sizeof scratch_) { memcpy(scratch_, probe_tlv_, probe_tlv_length_); }
    size_t at = probe_tlv_length_ <= sizeof scratch_ ? probe_tlv_length_ : 0;
    for (size_t i = 0; i < transport_count_ && at + 5 <= sizeof scratch_; ++i) {
      const uint8_t v[5] = {reg::core::kTlvDescribeTransport, 3, static_cast<uint8_t>(i), transports_[i].kind,
                            transports_[i].usb_interface};
      memcpy(scratch_ + at, v, 5);
      at += 5;
    }
    if (oep_pid_ && at + 3 <= sizeof scratch_) {
      const uint8_t v[3] = {reg::core::kTlvDescribeOepPid, 1, 1};
      memcpy(scratch_ + at, v, 3);
      at += 3;
    }
    tlv = scratch_;
    tlv_length = at;
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

}  // namespace oep
