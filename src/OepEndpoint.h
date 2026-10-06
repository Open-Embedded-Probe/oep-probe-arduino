// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 endpoint: frames in, interfaces by name, the session lock, results out.
//
// The lock follows oep-core §6: a host-chosen u32 session id, a lease extended by every request of its holder and
// counted from when that request completed (a long request never lapses its own lock). An explicit end keeps the
// session's resources for the next open (the same id resumes them); a lapse sweeps them, and the next request with
// that id is rejected expired until it opens again (resumed 2); a takeover by force sweeps them too and forgets the id
// (the taken host meets locked, then no_session).
#pragma once

#include <Arduino.h>

#include "OepFrame.h"
#include "Oep.h"

namespace oep {

// The raw side of the serial ports (core §3.4): what a serial port carries outside the frames. The binds implement it
// (oep.probe.config §1.2); the endpoint calls it from poll(), the one writer of every port.
class PinTable;

class RawPorts {
 public:
  // Bytes that came in on serial port `port` outside any frame (not called while a session holds the port).
  virtual void rawIn(uint8_t port, const uint8_t *data, size_t length) = 0;
  // Up to `room` bytes serial port `port` would send now (not asked while a session holds the port); nothing moves
  // until rawSent says how many of them the port took.
  virtual size_t rawOut(uint8_t port, uint8_t *out, size_t room) = 0;
  virtual void rawSent(uint8_t port, size_t length) = 0;
  // Every poll, before rawOut: `session` = a session holds the lock now.
  virtual void poll(bool session) { (void)session; }
  // The session ended (end, lapse, force): the ports it held (bit n = port n) take up the raw transfer again.
  virtual void sessionOver(uint32_t held) = 0;

 protected:
  ~RawPorts() = default;
};

class Endpoint {
 public:
  static constexpr size_t kMaxInterfaces = 16;
  static constexpr uint32_t kLeaseDefaultMs = 3000, kLeaseMinMs = reg::kLimitLeaseMinMs, kLeaseMaxMs = reg::kLimitLeaseMaxMs;

  // A transport's kind (core §7.5, registry transport_kind): the serial ports (UART bridge, USB CDC, USB-Serial/JTAG)
  // frame as 0x00 <COBS> 0x00 and share the line with raw bytes (core §3.4); vendor bulk, HID and TCP are
  // length(u16) message. The endpoint lists the transports in oep.core's describe, in the order they were added.
  enum : uint8_t {
    kUartBridge = reg::core::kTransportKindUartBridge, kUsbCdc = reg::core::kTransportKindUsbCdc,
    kUsbSerialJtag = reg::core::kTransportKindUsbSerialJtag, kVendorBulk = reg::core::kTransportKindVendorBulk,
    kHid = reg::core::kTransportKindHid, kTcp = reg::core::kTransportKindTcp,
  };
  static constexpr bool serialKind(uint8_t kind) { return kind == kUartBridge || kind == kUsbCdc || kind == kUsbSerialJtag; }
  static constexpr size_t kMaxTransports = 4;
  // A serial port's frames are decoded here (one at a time): max_frame may not exceed this with a serial port.
  static constexpr size_t kMaxSerialFrame = 2048;

  // usb_interface (describe transport, core §7.5): for USB CDC the bInterfaceNumber of the CDC communication interface
  // (the first of the function); for built-in USB serial that number as the hardware presents it, or 0xFF when the
  // probe cannot know it; for vendor bulk and HID that interface's number; 0xFF for a UART bridge and TCP.
  Endpoint(Stream &stream, uint8_t *rx_buffer, size_t rx_capacity, uint8_t *tx_buffer, size_t tx_capacity,
           Limits limits, uint8_t kind, uint8_t usb_interface = 0xff)
      : tx_(tx_buffer), tx_capacity_(tx_capacity), limits_(bounded(limits)) {
    addTransport(stream, rx_buffer, rx_capacity, kind, usb_interface, false);
    setPushQueue(1024);
  }
  // confirm's values within core §7.1's bounds, whatever the sketch gave: max_frame 64 or more, window max_frame or
  // more, max_inflight 1 or more (a host treats a transport answering outside them as not usable). max_inflight is at
  // most the resend table's entries: the table remembers at least max_inflight requests (core §5.2).
  static constexpr Limits bounded(Limits l) {
    if (l.max_frame < reg::kMinMaxFrame) l.max_frame = reg::kMinMaxFrame;
    if (l.window_bytes < l.max_frame) l.window_bytes = l.max_frame;
    if (l.max_inflight < 1) l.max_inflight = 1;
    if (l.max_inflight > kDedupEntries) l.max_inflight = kDedupEntries;
    return l;
  }

  // Another way in to the same probe (core §3.3). Every transport shares the one session and lock; a result goes back
  // on the transport its request came from, pushes and events go to the transport the subscription came from.
  // rx: one whole frame for this transport (a serial port: its encoded candidate, cobsFrameMax(max_frame)). The
  // transport the constructor took is transport 0 (the one a DirectTransport, if any, belongs to). The index is the
  // order of adding, as the describe lists them and the binds name the serial ports.
  bool addTransport(Stream &stream, uint8_t *rx_buffer, size_t rx_capacity, uint8_t kind, uint8_t usb_interface = 0xff,
                    bool flush_after_burst = false);
  size_t transportCount() const { return transport_count_; }
  bool isSerialPort(size_t index) const { return index < transport_count_ && serialKind(transports_[index].kind); }
  // The raw side of the serial ports (the binds); nullptr: raw bytes are dropped.
  void setRawPorts(RawPorts *raw) { raw_ = raw; }
  // A session holds the lock / holds serial port `port` (its raw transfer is stopped, core §3.4).
  bool locked() const { return locked_; }
  // A session has taken the lock at least once since boot (oep-if-probe-config §3.1: no retry with reset after that).
  bool lockEverTaken() const { return have_last_; }
  bool held(size_t port) const { return locked_ && ((held_ >> port) & 1); }
  // The probe enumerates with the project's USB VID:PID, registry usb project_vid / project_pid (describe discoverable,
  // core §3.3 / §7.5). Set it only once it actually does: a probe behind a UART bridge or on a fixed-ID built-in USB
  // serial leaves it 0.
  void setDiscoverable(bool on) { discoverable_ = on; }
  // The longest one request takes (describe max_op_ms, core §7.5): what every interface's long op is bounded by.
  static constexpr uint32_t kMaxOpMs = oep::kMaxOpMs;
  // fn of an interface added (0: not added), and the interface at a fn (nullptr: none).
  uint16_t fnOf(const Interface &interface) const {
    for (size_t i = 0; i < count_; ++i) if (interfaces_[i] == &interface) return static_cast<uint16_t>(i + 1);
    return 0;
  }
  Interface *interfaceAt(uint16_t fn) const { return fn >= 1 && fn <= count_ ? interfaces_[fn - 1] : nullptr; }

  // An interface, numbered fn 1, 2, ... in the order added. The list stays the same for a boot (core §7.2): every add
  // comes before the first poll(), and one after it is refused (false).
  bool add(Interface &interface);
  bool anyPlanRoles() const {
    for (size_t i = 0; i < count_; ++i) if (interfaces_[i]->planRoles()) return true;
    return false;
  }
  // The plan (oep-core §8, per fn): the roles now applied (persistent_only: those set through oep.probe.config), and
  // a replacement of the fns listed that is all or nothing and outlives sessions (0: applied; else the reject reason,
  // with the plans before it applied again). A listed fn with no role is released.
  static constexpr size_t kMaxRoles = 64;   // the plan's role assignments, every fn together (describe plan_roles)
  size_t plan(RoleAssignment *out, size_t max, bool persistent_only = false) const;
  uint8_t replacePlan(const RoleAssignment *roles, size_t count, const uint16_t *fns, size_t nfns);
  // The unavailable answer for the last replacement refused unavailable (core §4.3: the cause, the channel that met
  // something, who holds it).
  Result planUnavailable(uint8_t *out, size_t capacity) const {
    return unavailable(out, capacity, plan_refusal_.cause, plan_refusal_.channel, plan_refusal_.holder_fn,
                       plan_refusal_.holder_kind);
  }
  // The channels oep.probe.config's disable items take away (channel < 64): a plan_apply naming one is refused
  // unavailable cause 5 (held by settings) with the channel (probe.config §1).
  void setDisabled(uint64_t mask) { disabled_ = mask; }
  // The pin table the interfaces' plans claim from: a plan replacement releases only the channels leaving the plan
  // (PinTable::deferIdle; oep-core §8). ProbeConfig::setPins sets it too.
  void setPins(PinTable *pins) { pins_ = pins; }
  bool disabled(uint16_t channel) const { return channel < 64 && (disabled_ >> channel) & 1; }
  // The identity of the interface list (oep-if-probe-config §2): CRC-32 of every entry (fn u16, instance u16,
  // revision u8, name) in fn order, oep.core first.
  uint32_t listHash() const;
  // oep.core's describe: the probe itself, as TLV bytes (kept by the caller). Declarations only (core §7.3).
  void setProbeDescription(const uint8_t *tlv, size_t length) { probe_tlv_ = tlv; probe_tlv_length_ = length; }
  // The boot_id returned by confirm, open and the heartbeat (core §6.5): a value that changes every boot. Without a
  // call the endpoint picks it itself when the first message arrives (bootIdSource: a hardware random source, or on a
  // platform without one the timer's count at that external event). A sketch with a better source of its own (a
  // counter it keeps in non-volatile storage) sets it in setup(); 0 is as good as any other value.
  void setBootId(uint32_t boot_id) { boot_id_ = boot_id; boot_id_set_ = true; }
  uint32_t bootId() {
    if (!boot_id_set_) setBootId(bootIdSource());
    return boot_id_;
  }
  // The optional port_speed (core §3.5, fn 0 op 0x14): a host raises a UART bridge's baud for its session. Setting a
  // handler turns the feature on (describe port_speed 1, the op taken; without one the op is unknown_operation).
  // fn(port, baud, apply): apply false = the rate the port would run at for `baud` (0: this UART cannot make it, the
  // request is unsupported); apply true = switch the port to `baud` (its output already flushed) and return the rate it
  // runs at. `base` is the boot speed every revert goes back to.
  using PortSpeedFn = uint32_t (*)(uint8_t port, uint32_t baud, bool apply);
  void setPortSpeed(PortSpeedFn fn, uint32_t base) { port_speed_ = fn; speed_base_ = base; }
  // The rate a sped-up port runs at now (0: every port at its boot speed), and whether it is committed (else trying).
  uint32_t portSpeedNow() const { return speed_state_ == kSpeedBase ? 0 : speed_rate_; }
  bool portSpeedCommitted() const { return speed_state_ == kSpeedCommitted; }
  // Experimental event from an interface (sent to the lock holder if it subscribed to that interface), after
  // results; kept in a small queue until then (oldest dropped when full - the seq gap shows it).
  bool event(Interface &from, uint8_t kind, const uint8_t *payload, size_t length);
  // Events the interface lost before handing them over (its own queue overflowed): the seq skips them, so the
  // host sees the gap. Every event generated must take a seq number, sent or not.
  void eventsLost(Interface &from, uint16_t count);
  // Experimental: at most this many bytes of pushes may wait in the transport. Size it to the link: a result waits
  // behind up to this much (256 B is about 0.3 ms on USB-Serial/JTAG, 23 ms on a 115200 UART). Never more than
  // max_frame x 2 (core §11.4 obligation 2): 0, or a larger value, is that bound.
  void setPushQueue(size_t bytes) {
    const size_t bound = 2u * limits_.max_frame;
    push_queue_ = bytes == 0 || bytes > bound ? bound : bytes;
  }
  // Flush the stream after each poll that wrote something: a buffered USB vendor interface sends a short frame only
  // when flushed (E160). Off for streams that send by themselves (USB-Serial/JTAG, UART).
  void setFlushAfterBurst(bool on) { transports_[0].flush_after_burst = on; }
  // Experimental: a zero-copy path for an interface's data pushes (length-prefixed framing only). The interface builds
  // whole push frames itself, from its own task: directPush says whether it may send now (the lock holder subscribed
  // its fn) with the fn and the subscriber's batching; takeSeq numbers a frame (shared with that fn's events).
  void setDirect(DirectTransport *direct) { direct_ = direct; }
  // the zero-copy path belongs to transport 0: pushes use it only while the subscriber came in on transport 0
  DirectTransport *direct() const {
    return push_ == 0 && !serialKind(transports_[0].kind) ? direct_ : nullptr;
  }
  bool directPush(const Interface &from, uint16_t &fn, uint16_t &min_bytes, uint32_t &max_delay_ms) const;
  uint16_t takeSeq(uint16_t fn) { return __atomic_fetch_add(&push_seq_[fn - 1], 1, __ATOMIC_RELAXED); }
  void poll();

 private:
  struct Transport {
    Stream *stream = nullptr;
    FrameReader reader;
    SerialReader serial;
    uint8_t kind = kVendorBulk, usb_interface = 0xff, index = 0;
    Endpoint *owner = nullptr;
    bool flush_after_burst = false, wrote = false;
    int tx_room_max = 0;
  };
  static void rawSink(void *context, const uint8_t *data, size_t length);
  void rawOut();
  void sessionEnded();
  RawPorts *raw_ = nullptr;
  uint32_t held_ = 0;   // serial ports the lock holder's requests came in on (core §3.4)
  bool discoverable_ = false;
  uint8_t decode_[kMaxSerialFrame + 2];
  uint8_t owner_[32];   // the lock holder's owner text (core §6.4)
  uint8_t owner_length_ = 0;
  size_t appendOwner(uint8_t *out, size_t room, uint8_t tag) const;
  Transport transports_[kMaxTransports];
  size_t transport_count_ = 0;
  size_t current_ = 0;   // the transport the message being handled came in on (results go back there)
  size_t push_ = 0;      // the transport the push subscriptions came in on
  uint8_t *tx_;
  size_t tx_capacity_;
  Limits limits_;
  Interface *interfaces_[kMaxInterfaces] = {};
  size_t count_ = 0;
  bool polled_ = false;   // poll() ran: the interface list is fixed (core §7.2)
  const uint8_t *probe_tlv_ = nullptr;
  size_t probe_tlv_length_ = 0;
  uint64_t disabled_ = 0;                        // the settings' disabled channels (setDisabled)
  PinTable *pins_ = nullptr;                     // setPins
  // What the last plan replacement refused unavailable met (core §4.3's payload: cause, channel, holder_fn,
  // holder_kind; 0xFFFF / 0 = left out). replaceFns fills it.
  struct PlanRefusal { uint8_t cause; uint16_t channel, holder_fn; uint8_t holder_kind; };
  PlanRefusal plan_refusal_ = {0, 0xffff, 0xffff, 0};
  uint32_t boot_id_ = 0;
  bool boot_id_set_ = false;
  // port_speed (core §3.5): one UART bridge at a time is off its boot speed, trying (verify_ms to be committed; one
  // broken candidate after the first good frame at the new speed reverts) or committed (idle_ms, at most
  // kPortSpeedIdleMaxMs, with no good frame - not counted while a request runs, like the lease - or kSpeedBadRun broken
  // candidates in a row with no good frame between them, revert). A step that does not fit the port's state is
  // unavailable cause 6. A switch or a revert asked by a request happens after its answer is out.
  enum : uint8_t { kSpeedBase, kSpeedTry, kSpeedCommitted };
  enum : uint8_t { kSpeedNone, kSpeedSwitch, kSpeedRevert };
  static constexpr uint8_t kSpeedBadRun = 3;
  PortSpeedFn port_speed_ = nullptr;
  uint32_t speed_base_ = 115200;
  uint8_t speed_state_ = kSpeedBase, speed_port_ = 0xff;
  uint32_t speed_asked_ = 0, speed_rate_ = 0, speed_until_ = 0, speed_idle_ms_ = 0, speed_good_ms_ = 0;
  bool speed_heard_ = false;   // a good frame came at the new speed (until then a broken candidate is the switch-over's)
  uint8_t speed_bad_run_ = 0;  // broken candidates since the last good frame on the sped-up port (committed)
  uint8_t speed_pending_ = kSpeedNone, speed_pending_port_ = 0xff;
  uint32_t speed_pending_baud_ = 0, speed_pending_verify_ = 0;
  Result portSpeed(bool has_session, uint32_t session, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  void speedApply();             // the switch or revert a request asked for, after its answer
  void speedRevert();            // back to the boot speed (nothing when there already)
  void speedPoll();              // the try deadline, the idle limit, a revert the session's end asked for
  void speedBad();               // a broken candidate on the sped-up port
  volatile bool locked_ = false;
  uint32_t holder_ = 0, last_ = 0;
  bool have_last_ = false;
  bool swept_ = false;   // the last session's lock lapsed and its resources went (core §6.2: expired until it opens again)
  uint32_t lease_ms_ = kLeaseDefaultMs, expires_ms_ = 0;
  uint8_t scratch_[512];   // one interface's full describe before paging

  void handleMessage(const uint8_t *message, size_t length);
  bool coreOffers(uint8_t op) const;
  Result core(uint8_t op, bool has_session, uint32_t session, const uint8_t *payload, size_t length,
              uint8_t *out, size_t capacity);
  Result list(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result describe(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result open(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result planApply(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  uint16_t replaceFns(const RoleAssignment *roles, size_t count, const uint16_t *fns, size_t nfns, bool persistent);
  void planRelease(const uint16_t *fns, size_t nfns);
  bool planned_[kMaxInterfaces] = {};
  bool persistent_[kMaxInterfaces] = {};   // planned through replacePlan (oep.probe.config): a lapse does not release it
  // Experimental push subscriptions, per fn.
  volatile bool subscribed_[kMaxInterfaces] = {};
  DirectTransport *direct_ = nullptr;
  uint16_t push_seq_[kMaxInterfaces] = {};
  uint16_t min_bytes_[kMaxInterfaces] = {};
  uint32_t max_delay_ms_[kMaxInterfaces] = {};
  uint32_t waiting_since_[kMaxInterfaces] = {};
  bool waiting_[kMaxInterfaces] = {};
  // events: fn (0 = core), seq per fn shared with data frames
  struct Event { uint16_t fn; uint16_t seq; uint8_t kind; uint8_t length; uint8_t payload[40]; };
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
  // Dedup of the last session's requests sent again (core §5.2): the last kDedupEntries results, keyed on corr
  // and checked against fn, op and a CRC of the payload; dropped at every open. Results over kDedupBytes are not kept
  // (rejected result_lost): kDedupBytes covers a whole frame of the largest profile, so a 1 KiB read_block whose answer
  // was corrupted on a CP2102 link comes back from here instead of being read again (V003 jig, 2026-10-01).
  static constexpr size_t kDedupEntries = 8, kDedupBytes = 1024 + 5;
  struct Dedup {
    bool used = false, kept = false;
    uint16_t corr = 0, fn = 0;
    uint8_t op = 0;
    uint16_t length = 0;
    uint32_t crc = 0;
    uint8_t result[kDedupBytes];
  };
  Dedup dedup_[kDedupEntries];
  size_t dedup_next_ = 0;
  uint16_t newest_corr_ = 0;   // the last session's newest request (core §4.1: the host numbers them in order)
  bool have_newest_ = false;
  RoleAssignment plan_roles_[kMaxRoles] = {};
  size_t plan_count_ = 0;
  Result checkSession(bool has_session, uint32_t session, uint8_t *out, size_t capacity);
  void lapse();
  void loseSession();
  void sendReject(uint16_t corr, uint8_t reason);
  static uint32_t crc32(const uint8_t *data, size_t length);
  uint32_t remaining() const;
};

}  // namespace oep
