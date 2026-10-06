// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepEndpoint.h"
#include "OepLog.h"
#include "OepPinTable.h"

#include <string.h>

namespace oep {
namespace {

// rejected locked: remaining ms (u32), then the holder's owner TLV when it gave one (core §4.3, §6.4)
Result lockedFor(uint32_t remaining_ms, const uint8_t *owner, size_t owner_length, uint8_t *out, size_t capacity) {
  if (capacity < 4) return rejected(kRejectLocked);
  putU32(out, remaining_ms);
  size_t n = 4;
  if (owner_length && capacity >= 4 + kTlvHeader + owner_length) {
    putTlvHeader(out + 4, reg::core::kTlvLockedPayloadOwner, static_cast<uint16_t>(owner_length));
    memcpy(out + 4 + kTlvHeader, owner, owner_length);
    n += kTlvHeader + owner_length;
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
  size_t at = 0, used = 0;
  uint16_t index = 0;
  more = false;
  uint8_t tag = 0;
  const uint8_t *value = nullptr;
  size_t vlen = 0, next = 0;
  while (tlvAt(tlv, length, at, tag, value, vlen, next)) {
    const size_t size = next - at;
    if (index >= first) {
      if (used + size > capacity) { more = true; break; }
      memcpy(out + used, tlv + at, size);
      used += size;
    }
    at = next;
    ++index;
  }
  return used;
}

}  // namespace

bool Endpoint::add(Interface &interface) {
  // a name outside core §13 rule 1 is refused: list would show what no host may rely on
  if (count_ >= kMaxInterfaces || polled_ || !interfaceName(interface.name())) return false;
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
  t.reader.setTcp(kind == kTcp);   // no restart on a pause inside a frame (transports §2)
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

// A serial port's raw bytes (transports §4): to the binds, unless a session holds the port (then dropped).
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
  if (speed_state_ != kSpeedBase) speed_pending_ = kSpeedRevert;   // oep-if-link §3: after the answer that ended it, if any
  if (raw_) raw_->sessionOver(held_);
  held_ = 0;
}

size_t Endpoint::appendOwner(uint8_t *out, size_t room, uint8_t tag) const {
  if (!owner_length_ || room < kTlvHeader + owner_length_) return 0;
  putTlvHeader(out, tag, owner_length_);
  memcpy(out + kTlvHeader, owner_, owner_length_);
  return kTlvHeader + owner_length_;
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

bool Endpoint::directPush(const Interface &from, uint16_t &fn, uint16_t &min_bytes, uint32_t &max_delay_ms) const {
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
    uint8_t hb[12];   // boot_id(u32) uptime_ns(u64) (core §11.2)
    putU32(hb, bootId());
    putU64(hb + 4, nowNs());
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

// subscribe(fn u16, min_bytes u16, max_delay_ms u32) [TLV]; unsubscribe(fn u16) [TLV]. fn 0 = heartbeat events,
// max_delay_ms = the period (0: 1000 ms). seq starts again at 0 with every subscribe. A fn that emits nothing is
// rejected unsupported; an unsubscribe of a fn not subscribed does nothing (core §11.3).
Result Endpoint::subscription(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  const bool batching = op == kOpSubscribe;
  const size_t fixed = batching ? 8 : 2;
  if (length < fixed) return rejected(kRejectMalformed);
  Tail tail;
  const Result parsed = tail.parse(payload + fixed, length - fixed, out, capacity);
  if (parsed.resolution != kResolutionCompleted) return parsed;
  const uint16_t fn = getU16(payload);
  const uint16_t min_bytes = batching ? getU16(payload + 2) : 0;
  const uint32_t max_delay = batching ? getU32(payload + 4) : 0;
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
  if (!interfaces_[i]->subscribe(true)) return unsupportedValue(out, capacity);   // it emits nothing (core §11.3)
  subscribed_[i] = true;
  push_seq_[i] = 0;
  min_bytes_[i] = min_bytes;
  max_delay_ms_[i] = max_delay;
  waiting_[i] = false;
  return tail.finish(completed(), out, capacity);
}

void Endpoint::poll() {
  if (restarting_) return;   // fn 0 restart answered: nothing more is served or sent (core §6.6)
  polled_ = true;
  (void)nowNs();   // the clock counts the wraps of a 32-bit timer as it reads it (core §2.6a): read it every pass
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
          const uint32_t bad = t.serial.badCandidates();
          const bool message = t.serial.feed(p, n, rawSink, &t);
          if (i == speed_port_ && speed_state_ != kSpeedBase) {   // the sped-up port's line (oep-if-link §3)
            if (t.serial.badCandidates() != bad) speedBad();
            if (message) { speed_good_ms_ = millis(); speed_heard_ = true; speed_bad_run_ = 0; }   // a good frame: the run ends
          }
          if (message) handleMessage(t.serial.message(), t.serial.length());
          if (restarting_) return;   // what came after the restart is not served (core §6.6)
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
          if (restarting_) return;   // what came after the restart is not served (core §6.6)
          t.reader.consume();
        }
      }
    }
  }
  speedPoll();
  current_ = push_;
  push();   // after the results for everything that has arrived, on the subscriber's transport
  rawOut();   // then the serial ports' raw bytes, never inside a frame
  speedPoll();   // a lapse push() found: back to the boot speed
  for (size_t i = 0; i < transport_count_; ++i) {
    Transport &t = transports_[i];
    if (t.flush_after_burst && t.wrote) t.stream->flush();
    t.wrote = false;
  }
}

void Endpoint::lapse() {
  if (locked_ && static_cast<int32_t>(millis() - expires_ms_) >= 0) releaseLock();   // as an end (core §6.4, §9)
}

// The session's lock ends - end, lease expiry or force, all alike (core §6.4, §9): everything the session created is
// released - its subscriptions, its shares of the interfaces' resources (a console's stream before the wire's
// connection, so a stream it was the last user of closes with detail session_ended, oep-if-console §2), its plans (the
// pins to their idle state). A plan set through oep.probe.config stays (probe settings, not a session's), as do a
// slot's connection and a bind's stream. The owner is forgotten; the last id and the resend table stay (a resent end
// is answered from it, core §5.2). Nothing passes to the next session: no resume.
void Endpoint::releaseLock() {
  locked_ = false;
  owner_length_ = 0;
  loseSession();
  sessionEnded();
}

void Endpoint::loseSession() {
  endSubscriptions();
  for (size_t i = 0; i < count_; ++i) if (interfaces_[i]->sessionOverFirst()) interfaces_[i]->sessionOver();
  for (size_t i = 0; i < count_; ++i) if (!interfaces_[i]->sessionOverFirst()) interfaces_[i]->sessionOver();
  uint16_t lapsed[kMaxInterfaces];
  size_t n = 0;
  for (size_t i = 0; i < count_; ++i) if (planned_[i] && !persistent_[i]) lapsed[n++] = static_cast<uint16_t>(i + 1);
  if (n) planRelease(lapsed, n);   // n == 0 would mean every fn: only the session's plans go
}

// Subscriptions belong to the lock: they end with it (core §11.3).
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

// core §6.2 for a request (not open) with a session_id: no session holds the lock -> no_session (whatever the id: no
// resume); another holds it -> locked with the time left and the owner (never the holder's id); the holder's -> go on.
Result Endpoint::checkSession(uint32_t session, uint8_t *out, size_t capacity) {
  if (!locked_) return rejected(kRejectNoSession);
  if (session == holder_) return completed();
  return lockedFor(remaining(), owner_, owner_length_, out, capacity);
}

bool Endpoint::lockFreeOp(uint16_t fn, uint8_t op) const {
  return fn == 0 ? lockFreeIn(reg::core::kLockFreeOps, op) : interfaces_[fn - 1]->lockFree(op);
}

void Endpoint::handleMessage(const uint8_t *message, size_t length) {
  // other roles, and a request shorter than its header: dropped unanswered (core §2.4)
  if (length < kRequestHeader || message[0] != kRoleRequest) return;
  bootId();   // picked now if the sketch set none: the first message is the external event (core §6.5)
  const uint16_t corr = getU16(message + 1), fn = getU16(message + 3);
  const uint8_t op = message[5];
  const uint32_t session = getU32(message + 6);   // 0 = no session (core §4.1)
  const uint8_t *payload = message + kRequestHeader;
  const size_t payload_length = length - kRequestHeader;
  uint8_t *out = tx_ + kResultHeader;
  size_t capacity = tx_capacity_ - kResultHeader;
  if (capacity > static_cast<size_t>(limits_.max_frame) - kResultHeader) capacity = limits_.max_frame - kResultHeader;

  lapse();
  OEP_LOGF("req corr %u fn %u op 0x%02x sid %08lx len %u", corr, fn, op, static_cast<unsigned long>(session),
           static_cast<unsigned>(payload_length));
  // core §4.3 order 1, the header: a fn this probe does not have, an op that fn does not offer (its ops, core §1.2), an
  // op that needs the lock with session_id 0 (open's own id 0 is malformed: open checks it) - before the resend table,
  // so such a refusal is neither kept nor restarts the lease (core §5.2, §6.1)
  const bool is_open = fn == 0 && op == kOpOpen;
  if (fn != 0 && fn > count_) { sendReject(corr, kRejectUnknownFunction); return; }
  if (!offersOp(fn, op)) { sendReject(corr, kRejectUnknownOperation); return; }
  if (session == 0 && !is_open && !lockFreeOp(fn, op)) { sendReject(corr, kRejectSessionRequired); return; }
  // The last session's request sent again (the host lost the result): answer from what was kept, never run it twice,
  // and before the session is judged (core §5.2) - a resent end is answered from here and does not take the lock back.
  // The host numbers its requests in order (core §4.1): a corr no newer than the newest seen and not in the table was
  // dropped from it. An open is never looked up (a resent open is decided by §6.2).
  const bool mine = session != 0 && have_last_ && session == last_ && !is_open;
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
  restart_taken_ = false;
  g_request_ignored.reset();   // this request's ignored tags (core §2.3), from its tail's parse
  // core §6.2 (order 3): every request with a session_id but open - lock-free ones too (core §4.1) - goes through it
  if (session != 0 && !is_open) result = checkSession(session, out, capacity);
  else result = completed();
  if (result.resolution == kResolutionCompleted) {
    if (fn == 0) result = core(op, session, payload, payload_length, out, capacity);
    else result = interfaces_[fn - 1]->handle(op, payload, payload_length, out, capacity);
  }
  if (result.length > capacity) result = failed(0);
  // core §2.3: every completed answer carries ignored, a failed status too - for a handler that returned without finish
  result = g_request_ignored.append(result, out, capacity);
  // transports §4: a serial port the lock holder's requests come in on (its open too) stops its raw transfer
  if (serialKind(transports_[current_].kind) && locked_ && session == holder_ &&
      (!is_open || result.resolution == kResolutionCompleted))
    held_ |= uint32_t{1} << current_;
  // The lease runs from when the holder's request completed (a long verify must not lapse its own lock); every answer
  // to the holder that passed order 3, rejected ones too (core §6.1).
  if (session != 0 && locked_ && session == holder_) expires_ms_ = millis() + lease_ms_;
  // oep-if-link §3 condition 3: idle_ms is not counted while a request runs (the same rule as the lease) - it runs
  // from the answer.
  if (speed_state_ == kSpeedCommitted) speed_good_ms_ = millis();
  tx_[0] = kRoleResult;
  putU16(tx_ + 1, corr);
  tx_[3] = result.resolution;
  tx_[4] = result.detail;
  const size_t total = kResultHeader + result.length;
  if (mine) {   // kept for a request sent again, rejected answers too (core §5.2; a successful open drops the table)
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
    d.length = d.kept ? static_cast<uint16_t>(total) : 0;
    if (d.kept) memcpy(d.result, tx_, total);
  }
  if (restart_taken_) {
    // fn 0 restart (core §6.6): no notification after its answer - the session's end, and the zero-copy data already
    // queued (it would follow the answer) goes out before it
    endSubscriptions();
    if (direct_)
      for (const uint32_t start = millis(); direct_->queued() && static_cast<uint32_t>(millis() - start) < kRestartDrainMs;)
        delay(1);
  }
  OEP_LOGF("ans corr %u res %u detail %u len %u", corr, result.resolution, result.detail, static_cast<unsigned>(result.length));
  send(total);
  if (restart_taken_) { restartNow(); return; }
  speedApply();   // port_speed: the answer went out at the old speed; now switch (or revert)
}

// After restart's answer (core §6.6): it leaves the probe, every line is let go of, and the chip restarts as from
// power-on - a new boot_id, the saved settings applied, every serial port at its boot speed, no session, connection, plan
// or table kept.
void Endpoint::restartNow() {
  restart_taken_ = false;
  restarting_ = true;
  Transport &t = transports_[current_];
  // The answer out first: a UART's flush returns once its last bit is sent, a USB stream's starts the transfer. Not
  // USB-Serial/JTAG's: HWCDC's flush with no TX timeout drops what still waits; its interrupt sends it as the host reads.
  if (t.kind != kUsbSerialJtag) t.stream->flush();
  t.wrote = false;
  const uint32_t out = millis();
  // The lines let go of (core §6.6, §8): the session ends as at its end (everything it created), every interface closes
  // what the settings keep too (a slot's connection, a bind's stream) without touching the target, and every plan goes,
  // the settings' included - each channel to its free state.
  if (locked_) releaseLock();
  for (size_t i = 0; i < count_; ++i) if (interfaces_[i]->sessionOverFirst()) interfaces_[i]->probeRestart();
  for (size_t i = 0; i < count_; ++i) if (!interfaces_[i]->sessionOverFirst()) interfaces_[i]->probeRestart();
  planRelease(nullptr, 0);
  // The host's USB stack takes the last packet meanwhile; the restart still starts well within restart_after_answer_ms.
  while (static_cast<uint32_t>(millis() - out) < kRestartSettleMs) delay(1);
  OEP_LOGF("restart");
  if (restart_) restart_();   // does not return on a chip
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

// The ops fn 0 offers (core §12): the required ones, plan_apply / plan_release with an interface that has plan roles
// (core §1.2).
bool Endpoint::coreOffers(uint8_t op) const {
  switch (op) {
    case kOpConfirm: case kOpList: case kOpDescribe: case kOpOpen: case kOpEnd: case kOpKeepalive: case kOpLockState:
    case kOpSubscribe: case kOpUnsubscribe:
      return true;
    case kOpPlanApply: case kOpPlanRelease: return anyPlanRoles();
    case kOpRestart: return restart_ != nullptr;   // optional: with a handler (setRestart)
    default: return false;
  }
}

bool Endpoint::offersOp(uint16_t fn, uint8_t op) const {
  if (op >= kExperimentalOps) return false;   // never declared (core §1.2)
  return fn == 0 ? coreOffers(op) : interfaces_[fn - 1]->offers(op);
}

// The describe's ops tag (core §1.2, §7.4): base = the lowest op offered, then the bitmap as short as it can be (no op:
// base 0, no bitmap byte). Written first in every fn's describe, fn 0 included.
size_t Endpoint::opsTlv(uint16_t fn, uint8_t *out, size_t capacity) const {
  int lo = -1, hi = -1;
  for (int op = 1; op < kExperimentalOps; ++op)
    if (offersOp(fn, static_cast<uint8_t>(op))) { if (lo < 0) lo = op; hi = op; }
  const size_t bytes = lo < 0 ? 0 : static_cast<size_t>((hi - lo) / 8 + 1);
  if (capacity < kTlvHeader + 1 + bytes) return 0;
  putTlvHeader(out, kTagOps, static_cast<uint16_t>(1 + bytes));
  out[kTlvHeader] = lo < 0 ? 0 : static_cast<uint8_t>(lo);
  memset(out + kTlvHeader + 1, 0, bytes);
  for (int op = lo; lo >= 0 && op <= hi; ++op)
    if (offersOp(fn, static_cast<uint8_t>(op))) out[kTlvHeader + 1 + (op - lo) / 8] |= static_cast<uint8_t>(1u << ((op - lo) % 8));
  return kTlvHeader + 1 + bytes;
}

Result Endpoint::core(uint8_t op, uint32_t session, const uint8_t *payload, size_t length, uint8_t *out,
                      size_t capacity) {
  Tail tail;
  switch (op) {
    case kOpConfirm: {
      // "OEP?" min_rev(u8) max_rev(u8) [TLV]
      //   ->  "OEP!" revision(u8) flags(u8) max_frame(u16) window(u32) max_inflight(u8) boot_id(u32) [TLV]  (core §7.1)
      // TLV 0x01 transport (u8): the index (§7.5) of the transport this confirm came on, always attached. A range
      // with min_rev > max_rev is malformed; one without this probe's revision is unsupported with the supported range.
      if (length < 6 || memcmp(payload, reg::kConfirmRequestMagic, 4) != 0) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 6, length - 6, out, capacity);
      if (refused(parsed)) return parsed;
      if (payload[4] > payload[5]) return rejected(kRejectMalformed);
      if (reg::kProtocolRevision < payload[4] || reg::kProtocolRevision > payload[5]) {
        if (capacity < 6) return unsupportedValue(out, capacity);
        out[0] = kTagValue;
        putTlvHeader(out + 1, reg::core::kTlvUnsupportedPayloadSupported, 2);
        out[4] = reg::kProtocolRevision;   // min
        out[5] = reg::kProtocolRevision;   // max
        return {kResolutionRejected, kRejectUnsupported, 6};
      }
      if (capacity < 21) return failed();
      memcpy(out, reg::kConfirmResultMagic, 4);
      out[4] = reg::kProtocolRevision;
      out[5] = 0;                                     // flags: reserved
      putU16(out + 6, limits_.max_frame);
      putU32(out + 8, limits_.window_bytes);
      out[12] = limits_.max_inflight;
      putU32(out + 13, bootId());                     // a host without the lock learns of a restart here (core §6.5)
      putTlvHeader(out + 17, reg::core::kTlvConfirmAnswerTransport, 1);
      out[20] = transports_[current_].index;          // the entry of describe's transport list it came on
      return tail.finish(completed(21), out, capacity);
    }
    case kOpList: return list(payload, length, out, capacity);
    case kOpDescribe: return describe(payload, length, out, capacity);
    case kOpOpen: return open(session, payload, length, out, capacity);
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
    case kOpUnsubscribe:   // the lock holder only (handleMessage checked the session), and they end with the lock
      return subscription(op, payload, length, out, capacity);
    case kOpRestart: {
      // [TLV] -> nothing (core §6.6): the lock holder's (handleMessage checked the session), in fn 0's ops only with a
      // handler. completed success, sent first; the restart follows the answer (handleMessage, restartNow).
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      restart_taken_ = true;
      return tail.finish(completed(), out, capacity);
    }
    case kOpEnd:
    case kOpKeepalive:
    case kOpPlanApply:
    case kOpPlanRelease: {
      // the holder's (handleMessage checked the session); plan_apply / plan_release are in fn 0's ops only when an
      // interface has plan roles (core §1.2, §12)
      if (op == kOpPlanApply) return planApply(payload, length, out, capacity);
      if (op == kOpPlanRelease) {   // n(u8) n x fn(u16) [TLV]; n = 0: every fn
        if (length < 1) return rejected(kRejectMalformed);
        const size_t n = payload[0];
        const Result parsed = plainTail(tail, payload, length, 1 + 2 * n, out, capacity);
        if (refused(parsed)) return parsed;
        // the session's plans only: a plan the settings put in stays (core §8), n = 0 included
        uint16_t fns[kMaxInterfaces];
        size_t m = 0;
        for (size_t i = 0; i < count_; ++i) {
          if (!planned_[i] || persistent_[i]) continue;
          bool hit = n == 0;
          for (size_t k = 0; k < n; ++k) hit |= getU16(payload + 1 + 2 * k) == i + 1;
          if (hit) fns[m++] = static_cast<uint16_t>(i + 1);
        }
        for (size_t k = 0; k < m; ++k)   // a track bound into a capture-group: the group's (oep-if-capture §4.1)
          if (const uint16_t group = interfaces_[fns[k] - 1]->boundTo())
            return unavailable(out, capacity, reg::core::kUnavailableCauseBoundInGroup, 0xFFFF, group);
        if (m) planRelease(fns, m);   // m == 0 must not reach it: there an empty list means every fn
        return tail.finish(completed(), out, capacity);
      }
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (op == kOpEnd) releaseLock();   // the lock and everything the session created (core §6.4, §9); no resume
      return tail.finish(completed(), out, capacity);
    }
    default: return rejected(kRejectUnknownOperation);
  }
}

// oep.link port_speed (oep-if-link §3): port(u8) baud(u32) step(u8: 0 try, 1 commit, 2 revert) verify_ms(u16) idle_ms(u32) [TLV]
//   ->  baud(u32: the rate that applies). A step over 2 is unsupported. Only the UART bridge the request came in on
// (else unavailable cause 6); a baud this UART cannot make is unsupported. The port is in one of three states - boot,
// trying, committed - and a step that does not fit its state is unavailable cause 6 too (the same refusal as the wrong
// port): try is taken at the boot speed only, commit while trying (that baud, that port), revert while trying or
// committed. try: answered at the old speed, then switched; verify_ms later the probe goes back unless a commit came
// (on the new speed), and a broken candidate after the first good frame at the new speed sends it back at once.
// commit: then idle_ms (this request's, at most kPortSpeedIdleMaxMs: 0 and anything longer count as that maximum, so a
// host that died leaves the port at its boot speed soon) with no good frame, or kSpeedBadRun broken candidates in a row
// with no good frame between, revert. revert: answered (the boot speed) at the speed now, then back.
Result Endpoint::portSpeed(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (!port_speed_) return rejected(kRejectUnknownOperation);   // not in oep.link's ops (Link::offers)
  Tail tail;
  const Result parsed = plainTail(tail, payload, length, 12, out, capacity);
  if (refused(parsed)) return parsed;
  if (capacity < 4) return failed();
  const uint8_t port = payload[0], step = payload[5];
  const uint32_t baud = getU32(payload + 1), idle = getU32(payload + 8);
  const uint16_t verify = getU16(payload + 6);
  // verify_ms 0 in a try is malformed (order 5); a step of 3 or more is a value a later revision may define:
  // unsupported, tag 0x00 (core §2.5, oep-if-link §3, order 6)
  if (step == reg::link::kPortSpeedStepTry && verify == 0) return rejected(kRejectMalformed);
  if (step > reg::link::kPortSpeedStepRevert) return unsupportedValue(out, capacity);
  if (port != current_ || transports_[port].kind != kUartBridge) return wrongState(out, capacity);
  const bool this_port = speed_state_ != kSpeedBase && speed_port_ == port;   // this port is off its boot speed
  uint32_t answer = 0;
  if (step == reg::link::kPortSpeedStepTry) {
    if (this_port) return wrongState(out, capacity);   // trying or committed already: revert first
    answer = port_speed_(port, baud, false);
    if (!answer) return unsupportedValue(out, capacity);
    speed_pending_ = kSpeedSwitch;
    speed_pending_port_ = port;
    speed_pending_baud_ = baud;
    speed_pending_verify_ = verify;
  } else if (step == reg::link::kPortSpeedStepCommit) {
    // at the boot speed, or committed already, or another baud than the one trying: not the state for it
    if (!this_port || speed_state_ != kSpeedTry || baud != speed_asked_) return wrongState(out, capacity);
    speed_state_ = kSpeedCommitted;
    speed_idle_ms_ = (idle == 0 || idle > reg::kPortSpeedIdleMaxMs) ? reg::kPortSpeedIdleMaxMs : idle;
    speed_good_ms_ = millis();
    speed_bad_run_ = 0;
    answer = speed_rate_;
  } else {
    if (!this_port) return wrongState(out, capacity);   // at the boot speed already: nothing to go back from
    speed_pending_ = kSpeedRevert;
    answer = speed_base_;
  }
  putU32(out, answer);
  return tail.finish(completed(4), out, capacity);
}

void Endpoint::speedApply() {
  const uint8_t pending = speed_pending_;
  speed_pending_ = kSpeedNone;
  if (pending == kSpeedRevert) { speedRevert(); return; }
  if (pending != kSpeedSwitch || !port_speed_) return;
  if (speed_state_ != kSpeedBase && speed_port_ != speed_pending_port_) speedRevert();   // one port at a time
  Transport &t = transports_[speed_pending_port_];
  t.stream->flush();   // the answer out at the old speed first
  speed_rate_ = port_speed_(speed_pending_port_, speed_pending_baud_, true);
  speed_port_ = speed_pending_port_;
  speed_asked_ = speed_pending_baud_;
  speed_state_ = kSpeedTry;
  speed_until_ = millis() + speed_pending_verify_;
  speed_good_ms_ = millis();
  speed_heard_ = false;
  speed_bad_run_ = 0;
}

void Endpoint::speedRevert() {
  if (speed_state_ == kSpeedBase) return;
  transports_[speed_port_].stream->flush();
  port_speed_(speed_port_, speed_base_, true);
  speed_state_ = kSpeedBase;
  speed_port_ = 0xff;
  speed_rate_ = speed_asked_ = 0;
  speed_bad_run_ = 0;
}

void Endpoint::speedPoll() {
  if (speed_pending_ == kSpeedRevert) { speed_pending_ = kSpeedNone; speedRevert(); }
  const uint32_t now = millis();
  if (speed_state_ == kSpeedTry && static_cast<int32_t>(now - speed_until_) >= 0) speedRevert();   // no commit in time
  else if (speed_state_ == kSpeedCommitted && now - speed_good_ms_ >= speed_idle_ms_) speedRevert();
}

void Endpoint::speedBad() {
  // The bytes in flight while both ends switch (the host's last write at the old speed, the line settling) close as a
  // broken candidate: the ATOM's FTDI gave one on every switch, and reverting on it threw away rates that verified
  // cleanly (2026-10-01). So in the try state only a broken candidate after a good frame at the new speed counts; a
  // rate that never carries a good frame runs out its verify_ms instead.
  if (speed_state_ == kSpeedTry) { if (speed_heard_) speedRevert(); return; }   // the new speed breaks frames: not this one
  // committed (condition 4): kSpeedBadRun broken candidates in a row, no good frame between them (poll() resets the
  // run on each good frame). No time window: a host back at the boot speed sends confirms that arrive as broken
  // candidates, and the third takes the port back whatever their spacing (host duty 5).
  if (++speed_bad_run_ >= kSpeedBadRun) speedRevert();
}

// open (core §6.4): lease_ms(u32) force(u8) [TLV 0x01 owner], the session's id in the header  ->  lease_ms(u32)
// boot_id(u32). lease_ms 0 = the default; 1000..60000 are taken as asked, the rest rounded into that range. core §6.2's
// table: the lock free -> taken; held by this id -> a resent open: the lease starts again, nothing released, the
// subscriptions follow this transport; held by another -> locked, or with force that session is released first (as at
// its end, §9). The owner comes from the open that takes the lock and stays while it is held. Every successful open
// drops the resend table (§5.2).
Result Endpoint::open(uint32_t session, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  static const uint8_t kKnown[] = {reg::core::kTlvOpenOwner};
  if (length < 5) return rejected(kRejectMalformed);
  const uint32_t lease = getU32(payload);
  const uint8_t force = payload[4];
  // The open's own form first (it decides nothing about the lock until it is whole): its TLVs, session_id 0 (core §4.1),
  // force a boolean (core §2.1), owner text of 1 to 32 bytes (core §6.4: another length, or text that is not valid,
  // is a value the definition excludes - malformed, critical or not, core §2.1, §2.3).
  Tail tail;
  const Result parsed = tail.parse(payload + 5, length - 5, kKnown, out, capacity);
  if (refused(parsed)) return parsed;
  if (session == 0 || force > 1) return rejected(kRejectMalformed);
  size_t owner_len = 0;
  const uint8_t *owner = tail.find(reg::core::kTlvOpenOwner, owner_len);
  if (owner && (owner_len == 0 || owner_len > sizeof owner_ || !requestText(owner, owner_len)))
    return rejected(kRejectMalformed);
  if (capacity < 8) return failed();
  if (locked_ && holder_ != session && !force) return lockedFor(remaining(), owner_, owner_length_, out, capacity);
  if (locked_ && holder_ != session) releaseLock();   // force: the old session is released first (core §6.4, §9)
  if (locked_) {
    push_ = current_;   // a resent open: the notifications go to this transport (core §6.2)
  } else {
    if (owner) memcpy(owner_, owner, owner_len);
    owner_length_ = owner ? static_cast<uint8_t>(owner_len) : 0;
  }
  for (Dedup &d : dedup_) d.used = false;
  have_newest_ = false;
  locked_ = true;
  holder_ = last_ = session;
  have_last_ = true;
  lease_ms_ = lease == 0 ? kLeaseDefaultMs : (lease > kLeaseMaxMs ? kLeaseMaxMs : lease < kLeaseMinMs ? kLeaseMinMs : lease);
  expires_ms_ = millis() + lease_ms_;
  putU32(out, lease_ms_);
  putU32(out + 4, bootId());
  return tail.finish(completed(8), out, capacity);
}

// Role assignments as TLVs 0x10 (sent critical, 0x90) = fn(u16) role(u8) channel(u16) (critical). The plan is per fn (oep-core §8): the fns
// the request names are replaced, every other fn keeps its plan. An unknown critical TLV refuses the plan (unsupported),
// an unknown non-critical one is ignored and listed. A session's plan goes when its lock ends (end, a lapse, a
// takeover); a plan set through oep.probe.config (replacePlan, persistent) stays.
Result Endpoint::planApply(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // core §4.3's order over the whole request: its form (order 5: every role_assignment 5 bytes, fn 0 malformed as §8's
  // table says, at least one), then at the end of order 5 the fns named inside it (unknown_function), then an unknown
  // critical tag (unsupported, order 6), then the count against plan_roles (unavailable cause 2).
  static const uint8_t kKnown[] = {kTagRoleAssignment};
  Tail tail;
  tail.repeats(kKnown);   // role_assignment repeats (core §8)
  Result unknown_critical;
  const Result parsed = tail.parse(payload, length, kKnown, out, capacity, &unknown_critical);
  if (refused(parsed)) return parsed;
  size_t total = 0, at = 0;
  uint8_t raw = 0;
  size_t len = 0;
  const uint8_t *v = nullptr;
  while (tail.next(at, raw, v, len)) {
    if ((raw & ~kTagCritical) != (kTagRoleAssignment & ~kTagCritical)) continue;
    if (len != 5 || getU16(v) == 0) return rejected(kRejectMalformed);
    ++total;
  }
  if (!total) return rejected(kRejectMalformed);
  for (at = 0; tail.next(at, raw, v, len);)
    if ((raw & ~kTagCritical) == (kTagRoleAssignment & ~kTagCritical) && getU16(v) > count_)
      return rejected(kRejectUnknownFunction);
  if (refused(unknown_critical)) return unknown_critical;
  if (total > kMaxRoles) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);   // over plan_roles (core §8)
  RoleAssignment roles[kMaxRoles];
  size_t count = 0;
  for (at = 0; tail.next(at, raw, v, len);)
    if ((raw & ~kTagCritical) == (kTagRoleAssignment & ~kTagCritical)) roles[count++] = {getU16(v), v[2], getU16(v + 3)};
  uint16_t fns[kMaxInterfaces];
  size_t nfns = 0;
  for (size_t r = 0; r < count; ++r) {
    bool seen = false;
    for (size_t k = 0; k < nfns; ++k) seen |= fns[k] == roles[r].function;
    if (!seen) fns[nfns++] = roles[r].function;
  }
  // a fn whose plan the settings put in is the settings' to change (core §8)
  for (size_t k = 0; k < nfns; ++k)
    if (persistent_[fns[k] - 1])
      return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, 0xFFFF, fns[k],
                         reg::core::kHolderKindSettingsPlan);
  for (size_t r = 0; r < count; ++r)   // a channel the settings disable (probe.config §1)
    if (disabled(roles[r].channel))
      return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, roles[r].channel, 0xFFFF,
                                           reg::core::kHolderKindDisabled);
  const uint16_t reason = replaceFns(roles, count, fns, nfns, false);
  if (reason > 0xff) return failed();
  // another plan's channel, a wire connection's, the interface's own state: what it met (core §4.3)
  if (reason == kRejectUnavailable) return planUnavailable(out, capacity);
  // a role, channel or combination the interface does not declare: unsupported, tag 0x90 (core §8's table)
  if (reason == kRejectUnsupported) return unsupportedTag(out, capacity, kTagRoleAssignment | kTagCritical);
  if (reason) return rejected(static_cast<uint8_t>(reason));
  return tail.finish(completed(), out, capacity);
}

// Replaces the plan of the fns listed (a listed fn with no role is released). All or nothing: the old plans of those
// fns are taken off first (so a replacement may reuse its own pins), every interface checks without side effects, then
// all apply; a refusal or a failed apply puts the old plans back. 0 = done; a reject reason; or
// kRejectUnavailable + 0x100 when an interface accepted the check and then failed to apply (completed failed).
uint16_t Endpoint::replaceFns(const RoleAssignment *roles, size_t count, const uint16_t *fns, size_t nfns,
                              bool persistent) {
  // The old plans are released and the new ones applied as one change: a channel in both keeps its state and drive
  // (a power line through a gpio plan does not blink off), one leaving goes to its idle state at the end, a new one
  // stays in its idle state until its first set (oep-core §8, oep-if-fixture §1). The undo path settles the same way.
  for (size_t k = 0; k < nfns; ++k) {   // a track bound into a capture-group: the group's (oep-if-capture §4.1)
    const uint16_t group = fns[k] >= 1 && fns[k] <= count_ ? interfaces_[fns[k] - 1]->boundTo() : 0;
    if (group) {
      plan_refusal_ = {reg::core::kUnavailableCauseBoundInGroup, 0xffff, group, 0};
      return kRejectUnavailable;
    }
  }
  struct Settle {
    PinTable *pins;
    explicit Settle(PinTable *p) : pins(p) { if (pins) pins->deferIdle(); }
    ~Settle() { if (pins) pins->settleIdle(); }
  } settle(pins_);
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
  plan_refusal_ = {reg::core::kUnavailableCausePinInUse, 0xffff, 0xffff, 0};
  auto applyAll = [this, &listed](const RoleAssignment *set, size_t n, bool check) -> uint16_t {
    for (size_t i = 0; i < count_; ++i) {
      if (!listed[i]) continue;
      RoleAssignment mine[kMaxRoles];
      size_t m = 0;
      for (size_t r = 0; r < n; ++r) if (set[r].function == i + 1) mine[m++] = set[r];
      if (!m) continue;
      if (check) {
        if (const uint8_t reason = interfaces_[i]->planCheck(mine, m)) {
          if (reason == kRejectUnavailable) {   // a channel held now (a wire's connection, a resource), else its state
            plan_refusal_ = {interfaces_[i]->planRefusalCause(), 0xffff, 0xffff, 0};
            interfaces_[i]->planRefusalDetail(plan_refusal_.channel, plan_refusal_.holder_kind);
            for (size_t r = 0; r < m && pins_; ++r) {
              const uint16_t ch = mine[r].channel;
              if (!pins_->owner(ch) || pins_->owner(ch) == 0xff) continue;
              plan_refusal_ = {reg::core::kUnavailableCausePinInUse, ch, 0xffff, pins_->holderKind(ch)};
              for (size_t k = 0; k < plan_count_; ++k)   // another fn's plan: which, and whether the settings put it in
                if (plan_roles_[k].channel == ch) {
                  const uint16_t g = plan_roles_[k].function;
                  plan_refusal_.holder_fn = g;
                  plan_refusal_.holder_kind =
                      persistent_[g - 1] ? reg::core::kHolderKindSettingsPlan : reg::core::kHolderKindPlan;
                  break;
                }
              break;
            }
          }
          return reason;
        }
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
  if (plan_count_ + count > kMaxRoles) {   // over plan_roles (core §8)
    plan_refusal_ = {reg::core::kUnavailableCauseLimit, 0xffff, 0xffff, 0};
    undo();
    return kRejectUnavailable;
  }
  // a channel of an interface that shares none may be in no other fn's plan, old (kept) or new (core §8.1)
  for (size_t r = 0; r < count; ++r) {
    const uint16_t f = roles[r].function;
    if (f < 1 || f > count_) continue;
    const bool shares = interfaces_[f - 1]->planShares();
    auto clash = [&](const RoleAssignment &o) {
      return o.function != f && o.channel == roles[r].channel &&
             (!shares || (o.function >= 1 && o.function <= count_ && !interfaces_[o.function - 1]->planShares()));
    };
    for (size_t k = 0; k < plan_count_; ++k)
      if (clash(plan_roles_[k])) {   // another fn's plan: a session's (1), or one the settings put in (5)
        const uint16_t g = plan_roles_[k].function;
        plan_refusal_ = {reg::core::kUnavailableCausePinInUse, roles[r].channel, g,
                         persistent_[g - 1] ? reg::core::kHolderKindSettingsPlan : reg::core::kHolderKindPlan};
        undo();
        return kRejectUnavailable;
      }
    for (size_t k = 0; k < count; ++k)
      if (clash(roles[k])) {   // two fns of this very replacement
        plan_refusal_ = {reg::core::kUnavailableCausePinInUse, roles[r].channel, roles[k].function,
                         reg::core::kHolderKindPlan};
        undo();
        return kRejectUnavailable;
      }
  }
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
    entry(static_cast<uint16_t>(i + 1), instanceOf(static_cast<uint16_t>(i + 1)), interfaces_[i]->revision(),
          interfaces_[i]->name());
  return ~c;
}

Result Endpoint::list(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // flags(u8, bit0 exact) first(u16) prefix_len(u8) prefix [TLV]  ->  total(u16) count(u8) count x entry (core §2.3)
  // entry: fn(u16) instance(u16) revision(u8) flags(u8) name_len(u8) name; oep.core (fn 0) is the first one
  if (length < 4 || length < 4u + payload[3] || capacity < 3) return rejected(kRejectMalformed);
  Tail tail;
  const Result parsed = tail.parse(payload + 4 + payload[3], length - 4 - payload[3], out, capacity);
  if (refused(parsed)) return parsed;
  if (!requestText(payload + 4, payload[3])) return rejected(kRejectMalformed);   // the prefix is text (core §2.1)
  if (payload[0] & ~1u) return unsupportedValue(out, capacity);   // flags bits 1 to 7 are reserved (core §7.2)
  const bool exact = payload[0] & 1;
  const uint16_t first = getU16(payload + 1);
  const uint8_t n = payload[3];
  const uint8_t *prefix = payload + 4;
  uint16_t total = 0;
  uint8_t count = 0;
  size_t used = 3;
  bool full = false;
  // room for the ignored TLV after the entries, if there is one to report
  const size_t room = capacity > tail.room() ? capacity - tail.room() : 0;
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
    consider(static_cast<uint16_t>(i + 1), instanceOf(static_cast<uint16_t>(i + 1)), it.revision(), it.flags(), it.name());
  }
  putU16(out, total);
  out[2] = count;
  return tail.finish(completed(used), out, capacity);
}

Result Endpoint::describe(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // fn(u16) first(u16)  ->  more(u8) TLVs (first = the index of the first TLV). No TLV in the request (core §7.3: the
  // answer is a TLV list itself, so ignored never appears in it): anything after the fixed part is malformed.
  if (length != 4 || capacity < 1) return rejected(kRejectMalformed);
  const uint16_t fn = getU16(payload);
  const uint16_t first = getU16(payload + 2);
  const uint8_t *tlv = nullptr;
  size_t tlv_length = 0;
  if (fn > count_) return rejected(kRejectUnknownFunction);
  // every fn's describe starts with its ops (core §1.2, §7.4)
  const size_t ops = opsTlv(fn, scratch_, sizeof scratch_);
  if (fn == 0) {   // the ops, the sketch's part, then the transports, discoverable, plan_roles, max_op_ms, restart_max_ms (core §7.5)
    const size_t sketch = probe_tlv_length_ <= sizeof scratch_ - ops ? probe_tlv_length_ : 0;
    if (sketch) memcpy(scratch_ + ops, probe_tlv_, sketch);
    TlvWriter w(scratch_ + ops + sketch, sizeof scratch_ - ops - sketch);
    for (size_t i = 0; i < transport_count_; ++i) {
      const uint8_t v[3] = {static_cast<uint8_t>(i), transports_[i].kind, transports_[i].usb_interface};
      w.put(reg::core::kTlvDescribeTransport, v, sizeof v);
    }
    w.u8(reg::core::kTlvDescribeDiscoverable, discoverable_ ? 1 : 0);   // always: 0 without the project's VID:PID
    if (anyPlanRoles()) w.u32(reg::core::kTlvDescribePlanRoles, kMaxRoles);   // no plan ops, no plan to count
    w.u32(reg::core::kTlvDescribeMaxOpMs, max_op_ms_);
    if (restart_) w.u32(reg::core::kTlvDescribeRestartMaxMs, restart_max_ms_);   // with restart only (core §6.6, §7.5)
    tlv = scratch_;
    tlv_length = ops + sketch + w.length();
  } else {
    tlv_length = ops + interfaces_[fn - 1]->describe(scratch_ + ops, sizeof scratch_ - ops);
    tlv = scratch_;
  }
  bool more = false;
  const size_t used = tlv ? pageTlv(tlv, tlv_length, first, out + 1, capacity - 1, more) : 0;
  out[0] = more;
  return completed(1 + used);
}

// oep.link source / sink / port_speed (oep-if-link §2, §3).
Result Link::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case reg::link::kOpSource: {   // length(u32) [TLV]  ->  len(u16) data [TLV]
      const Result parsed = plainTail(tail, payload, length, 4, out, capacity);
      if (refused(parsed)) return parsed;
      // at most max_frame - 26 (header 5, len 2, the room for ignored 19 - kept even when nothing is ignored, so len
      // does not depend on the request's TLVs)
      const size_t most = endpoint_.maxFrameIn() - reg::kLimitLinkSourceOverheadBytes;
      size_t n = getU32(payload);
      if (n > most) n = most;
      if (capacity < 2 + n) return failed();
      putU16(out, static_cast<uint16_t>(n));
      for (size_t k = 0; k < n; ++k) out[2 + k] = static_cast<uint8_t>(k);
      return tail.finish(completed(2 + n), out, capacity);
    }
    case reg::link::kOpSink: {   // count(u16) data [TLV]  ->  (empty); a count past the bytes that follow is malformed
      if (length < 2) return rejected(kRejectMalformed);
      const Result parsed = plainTail(tail, payload, length, 2u + getU16(payload), out, capacity);
      if (refused(parsed)) return parsed;
      return tail.finish(completed(), out, capacity);
    }
    case reg::link::kOpPortSpeed: return endpoint_.portSpeed(payload, length, out, capacity);
    default: return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep
