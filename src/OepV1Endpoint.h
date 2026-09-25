// OEP v1 draft endpoint: frames in, interfaces by name, the session lock, results out.
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

  Endpoint(Stream &stream, uint8_t *rx_buffer, size_t rx_capacity, uint8_t *tx_buffer, size_t tx_capacity,
           Limits limits, Framing framing = Framing::kLengthPrefixed)
      : stream_(stream), reader_(rx_buffer, rx_capacity, limits.max_frame), cobs_(rx_buffer, rx_capacity),
        tx_(tx_buffer), tx_capacity_(tx_capacity), limits_(limits), framing_(framing) {}

  bool add(Interface &interface);
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
  void poll();

 private:
  Stream &stream_;
  FrameReader reader_;
  CobsReader cobs_;
  uint8_t *tx_;
  size_t tx_capacity_;
  Limits limits_;
  Framing framing_;
  Interface *interfaces_[kMaxInterfaces] = {};
  size_t count_ = 0;
  const uint8_t *probe_tlv_ = nullptr;
  size_t probe_tlv_length_ = 0;
  uint32_t boot_id_ = 0;
  bool locked_ = false;
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
  bool subscribed_[kMaxInterfaces] = {};
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
  uint16_t heartbeat_ms_ = 1000;
  uint32_t heartbeat_last_ = 0;
  bool queueEvent(uint16_t fn, uint8_t kind, const uint8_t *payload, size_t length);
  bool sendEvents();
  size_t push_queue_ = 1024;
  int tx_room_max_ = 0;
  Result subscription(uint8_t op, const uint8_t *payload, size_t length);
  void push();
  void endSubscriptions();
  void send(size_t length);
  bool plan_active_ = false;
  Result checkSession(bool has_session, uint32_t session, uint8_t *out, size_t capacity);
  void lapse();
  uint32_t remaining() const;
};

}  // namespace v1
}  // namespace oep
