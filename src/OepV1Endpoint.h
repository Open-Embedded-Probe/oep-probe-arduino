// OEP v1 endpoint: frames in, interfaces by name, the session lock, results out.
//
// The lock follows oep-spec docs/session-and-exclusivity.ja.md: a host-chosen u32 session id, a lease
// extended by every request of its holder and counted from when that request completed; when it lapses
// or ends the last id is remembered and may resume. Losing the host changes nothing else - no detach,
// no pin release, no target reset (the v0 idle abandon is gone on purpose).
#pragma once

#include <Arduino.h>

#include "OepFrame.h"
#include "OepV1.h"

namespace oep {
namespace v1 {

class Endpoint {
 public:
  static constexpr size_t kMaxInterfaces = 16;
  static constexpr uint32_t kLeaseDefaultMs = 3000, kLeaseMaxMs = 600000;

  // Framing: length-prefixed on reliable streams (USB CDC, USB-Serial/JTAG, TCP); COBS + CRC-16 on a UART
  // (through a USB-UART bridge the bytes are not protected). The probe knows which transport it has.
  enum class Framing : uint8_t { kLengthPrefixed, kCobsCrc };

  static constexpr size_t kMaxTransports = 4;

  Endpoint(Stream &stream, uint8_t *rx_buffer, size_t rx_capacity, uint8_t *tx_buffer, size_t tx_capacity,
           Limits limits, Framing framing = Framing::kLengthPrefixed)
      : tx_(tx_buffer), tx_capacity_(tx_capacity), limits_(limits) {
    addTransport(stream, rx_buffer, rx_capacity, framing, false);
  }

  // Another way in to the same probe (v1 wire §1): vendor bulk, HID, CDC, USB-Serial/JTAG, UART. Every transport shares
  // the one session and lock; a result goes back on the transport its request came from, pushes and events go to the
  // transport the subscription came from. rx: one whole frame (max_frame) for this transport. The transport the
  // constructor took is transport 0 (the one a DirectTransport, if any, belongs to).
  bool addTransport(Stream &stream, uint8_t *rx_buffer, size_t rx_capacity, Framing framing, bool flush_after_burst);

  bool add(Interface &interface);
  // The plan as probe state (oep.probe.config's plan item): the roles now applied, and a replacement that is all or
  // nothing (0: applied; else the reject reason, with the plan before it applied again).
  static constexpr size_t kMaxRoles = 16;
  size_t plan(RoleAssignment *out, size_t max) const;
  uint8_t replacePlan(const RoleAssignment *roles, size_t count);
  // oep.core's describe: the probe itself, as TLV bytes (kept by the caller).
  void setProbeDescription(const uint8_t *tlv, size_t length) { probe_tlv_ = tlv; probe_tlv_length_ = length; }
  // A random-ish value per boot; 0 = unknown (then hosts treat every no-session as a possible reboot).
  void setBootId(uint32_t boot_id) { boot_id_ = boot_id; }
  // Experimental event from an interface (sent to the lock holder if it subscribed to that interface), after
  // results; kept in a small queue until then (oldest dropped when full - the seq gap shows it).
  bool event(Interface &from, uint8_t kind, const uint8_t *payload, size_t length);
  // Events the interface lost before handing them over (its own queue overflowed): the seq skips them, so the
  // host sees the gap. Every event generated must take a seq number, sent or not.
  void eventsLost(Interface &from, uint16_t count);
  // Experimental: at most this many bytes of pushes may wait in the transport (0 = no limit). Size it to the link:
  // a result waits behind up to this much (256 B is about 0.3 ms on USB-Serial/JTAG, 23 ms on a 115200 UART).
  void setPushQueue(size_t bytes) { push_queue_ = bytes; }
  // Flush the stream after each poll that wrote something: a buffered USB vendor interface sends a short frame only
  // when flushed (E160). Off for streams that send by themselves (USB-Serial/JTAG, UART).
  void setFlushAfterBurst(bool on) { transports_[0].flush_after_burst = on; }
  // Experimental: a zero-copy path for an interface's data pushes (length-prefixed framing only). The interface builds
  // whole push frames itself, from its own task: directPush says whether it may send now (the lock holder subscribed
  // its fn) with the fn and the subscriber's batching; takeSeq numbers a frame (shared with that fn's events).
  void setDirect(DirectTransport *direct) { direct_ = direct; }
  // the zero-copy path belongs to transport 0: pushes use it only while the subscriber came in on transport 0
  DirectTransport *direct() const {
    return push_ == 0 && transports_[0].framing == Framing::kLengthPrefixed ? direct_ : nullptr;
  }
  bool directPush(const Interface &from, uint16_t &fn, uint16_t &min_bytes, uint16_t &max_delay_ms) const;
  uint16_t takeSeq(uint16_t fn) { return __atomic_fetch_add(&push_seq_[fn - 1], 1, __ATOMIC_RELAXED); }
  void poll();

 private:
  struct Transport {
    Stream *stream = nullptr;
    FrameReader reader;
    CobsReader cobs;
    Framing framing = Framing::kLengthPrefixed;
    bool flush_after_burst = false, wrote = false;
    int tx_room_max = 0;
  };
  Transport transports_[kMaxTransports];
  size_t transport_count_ = 0;
  size_t current_ = 0;   // the transport the message being handled came in on (results go back there)
  size_t push_ = 0;      // the transport the push subscriptions came in on
  uint8_t *tx_;
  size_t tx_capacity_;
  Limits limits_;
  Interface *interfaces_[kMaxInterfaces] = {};
  size_t count_ = 0;
  const uint8_t *probe_tlv_ = nullptr;
  size_t probe_tlv_length_ = 0;
  uint32_t boot_id_ = 0;
  volatile bool locked_ = false;
  uint32_t holder_ = 0, last_ = 0;
  bool have_last_ = false;
  uint32_t lease_ms_ = kLeaseDefaultMs, expires_ms_ = 0;
  uint8_t scratch_[512];   // one interface's full describe before paging

  void handleMessage(const uint8_t *message, size_t length);
  Result core(uint8_t op, bool has_session, uint32_t session, const uint8_t *payload, size_t length,
              uint8_t *out, size_t capacity);
  Result list(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result describe(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result open(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result planApply(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  void planRelease();
  bool planned_[kMaxInterfaces] = {};
  // Experimental push subscriptions, per fn.
  volatile bool subscribed_[kMaxInterfaces] = {};
  DirectTransport *direct_ = nullptr;
  uint16_t push_seq_[kMaxInterfaces] = {};
  uint16_t min_bytes_[kMaxInterfaces] = {}, max_delay_ms_[kMaxInterfaces] = {};
  uint32_t waiting_since_[kMaxInterfaces] = {};
  bool waiting_[kMaxInterfaces] = {};
  // events: fn (0 = core), seq per fn shared with data frames
  struct Event { uint16_t fn; uint16_t seq; uint8_t kind; uint8_t length; uint8_t payload[24]; };
  static constexpr size_t kEvents = 32;
  Event events_[kEvents];
  uint32_t event_head_ = 0, event_tail_ = 0;
  uint16_t core_seq_ = 0;
  bool heartbeat_ = false;
  uint32_t heartbeat_ms_ = reg::kHeartbeatDefaultMs;
  uint32_t heartbeat_last_ = 0;
  bool queueEvent(uint16_t fn, uint8_t kind, const uint8_t *payload, size_t length);
  bool sendEvents();
  size_t push_queue_ = 1024;
  Result subscription(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  void push();
  void endSubscriptions();
  void send(size_t length);
  bool plan_active_ = false;
  RoleAssignment plan_roles_[kMaxRoles] = {};
  size_t plan_count_ = 0;
  Result checkSession(bool has_session, uint32_t session, uint8_t *out, size_t capacity);
  void lapse();
  uint32_t remaining() const;
};

}  // namespace v1
}  // namespace oep
