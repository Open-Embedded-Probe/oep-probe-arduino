// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 endpoint: frames in, interfaces by name, the session lock, results out.
//
// The lock follows oep-core §6: a host-chosen u32 session id in every request's header (0 = no session), a lease
// extended by every request of its holder and counted from when that request completed (a long request never lapses
// its own lock). An end, a lapse and a takeover by force all release everything the session created (core §9); no
// resume - a request with any id while the lock is free is no_session, and a resent end is answered from the resend
// table. Every fn's describe starts with its ops (core §1.2, §7.4), written here from Interface::offers. fn 0 is the core
// itself (no name, not in list); oep.probe.plan and oep.probe.restart are interfaces the endpoint lists itself, after
// the sketch's (ProbePlan, ProbeRestart below).
#pragma once

#include <Arduino.h>

#include "OepFrame.h"
#include "Oep.h"

#include <string.h>

namespace oep {

// The raw side of the serial ports (transports §4): what a serial port carries outside the frames. The binds implement it
// (oep.probe.config §1.2); the endpoint calls it from poll(), the one writer of every port.
class PinTable;

class Endpoint;

// One accepted TCP connection at a time (transports §1): a Stream while it lasts. A listening socket is one transport
// entry of fn 0's describe (kind 6, interface 0xFF); each connection accepted on it is a transport of its own -
// confirm's transport TLV names the listener's entry, while the revision, max_frame / window / max_inflight and the
// notifications' destination are the connection's (core §7.1, §11.4). A slot holds one connection; the endpoint takes
// a few slots per listener (Endpoint::addTcpListener). OepTcp.h has the ESP32's (lwIP sockets).
class Connection : public Stream {
 public:
  // A number that changes with every connection the slot takes (0: none now). The endpoint starts that connection's
  // reader afresh, and notifications subscribed on a connection go nowhere once it changes.
  virtual uint32_t connection() const = 0;
  // Close the connection now (a length over max_frame, transports §1). What it held is dropped.
  virtual void drop() = 0;
};

// oep.probe.plan (oep-if-plan): plan_apply (0x01) and plan_release (0x02), describe plan_roles (0x40). The endpoint lists
// it itself, after every interface the sketch added, when one of them has plan roles (Interface::planRoles), and not
// otherwise (oep-if-plan: listed by a probe whose interfaces have plan roles, at most one). A sketch never adds it.
class ProbePlan final : public Interface {
 public:
  explicit ProbePlan(Endpoint &endpoint) : endpoint_(endpoint) {}
  const char *name() const override { return reg::probe_plan::kName; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return reg::probe_plan::kRevision; }
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::probe_plan::kLockFreeOps, op); }
  bool offers(uint8_t op) const override { return op == kOpPlanApply || op == kOpPlanRelease; }   // both required
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  Endpoint &endpoint_;
};

// oep.probe.restart (oep-if-restart): restart (0x01), describe restart_max_ms (0x40). An optional interface the endpoint
// lists itself, after oep.probe.plan, when the sketch gave it a handler (Endpoint::setRestart). A sketch never adds it.
class ProbeRestart final : public Interface {
 public:
  explicit ProbeRestart(Endpoint &endpoint) : endpoint_(endpoint) {}
  const char *name() const override { return reg::probe_restart::kName; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return reg::probe_restart::kRevision; }
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::probe_restart::kLockFreeOps, op); }
  bool offers(uint8_t op) const override { return op == kOpRestart; }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  Endpoint &endpoint_;
};

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
  // The fns a probe lists: the sketch's interfaces (at most kMaxInterfaces - 2) and the endpoint's own two (oep.probe.plan,
  // oep.probe.restart) after them.
  static constexpr size_t kMaxInterfaces = OEP_SMALL_RAM ? 10 : 24;
  static constexpr uint32_t kLeaseDefaultMs = 3000, kLeaseMinMs = reg::kLimitLeaseMinMs, kLeaseMaxMs = reg::kLimitLeaseMaxMs;

  // A transport's kind (core §7.5, registry transport_kind): the serial ports (UART bridge, USB CDC, USB-Serial/JTAG)
  // frame as 0x00 <COBS> 0x00 and share the line with raw bytes (transports §4); vendor bulk, HID and TCP are
  // length(u16) message. The endpoint lists the transports in fn 0's describe, in the order they were added.
  enum : uint8_t {
    kUartBridge = reg::core::kTransportKindUartBridge, kUsbCdc = reg::core::kTransportKindUsbCdc,
    kUsbSerialJtag = reg::core::kTransportKindUsbSerialJtag, kVendorBulk = reg::core::kTransportKindVendorBulk,
    kHid = reg::core::kTransportKindHid, kTcp = reg::core::kTransportKindTcp,
  };
  // A small-RAM build (OEP_SMALL_RAM) takes no serial transport (COBS, port_speed, raw ports): a USB vendor bulk one only.
  static constexpr bool serialKind(uint8_t kind) {
    return !OEP_SMALL_RAM && (kind == kUartBridge || kind == kUsbCdc || kind == kUsbSerialJtag);
  }
  // Transports, a TCP listener's connection slots each counted (addTcpListener).
  static constexpr size_t kMaxTransports = OEP_SMALL_RAM ? 2 : 8;
  // A serial port's frames are decoded here (one at a time): a serial port's max_frame is at most this (the
  // endpoint's own may be larger: its other transports use it, core §7.1).
  static constexpr size_t kMaxSerialFrame = OEP_SMALL_RAM ? 512 : 2048;

  // usb_interface (describe transport, core §7.5): for USB CDC the bInterfaceNumber of the CDC communication interface
  // (the first of the function); for built-in USB serial that number as the hardware presents it, or 0xFF when the
  // probe cannot know it; for vendor bulk and HID that interface's number; 0xFF for a UART bridge and TCP.
  Endpoint(Stream &stream, uint8_t *rx_buffer, size_t rx_capacity, uint8_t *tx_buffer, size_t tx_capacity,
           Limits limits, uint8_t kind, uint8_t usb_interface = 0xff)
      : tx_(tx_buffer), tx_capacity_(tx_capacity), limits_(bounded(limits)) {
    addTransport(stream, rx_buffer, rx_capacity, kind, usb_interface, false);
    setPushQueue(1024);
  }
  // How many requests the resend record keeps (core §5.1: the probe's choice).
  static constexpr size_t kDedupEntriesOf() { return kDedupEntries; }
  // The largest max_frame of this endpoint (the constructor's; a serial port's is at most kMaxSerialFrame - each
  // transport answers confirm with its own, core §7.1).
  uint16_t maxFrame() const { return limits_.max_frame; }
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

  // Another way in to the same probe (transports §3). Every transport shares the one session and lock; a result goes back
  // on the transport its request came from, pushes and events go to the transport the subscription came from.
  // rx: one whole frame for this transport (a serial port: its encoded candidate, cobsFrameMax(max_frame)). The
  // transport the constructor took is transport 0 (the one a DirectTransport, if any, belongs to). The index is the
  // order of adding, as the describe lists them and the binds name the serial ports.
  // After a TCP listener (below) no transport is added: the serial ports' indexes are the describe entries' (binds).
  bool addTransport(Stream &stream, uint8_t *rx_buffer, size_t rx_capacity, uint8_t kind, uint8_t usb_interface = 0xff,
                    bool flush_after_burst = false);
  // Buffers given later than the constructor - memory that is up only by setup() (an ESP32's PSRAM): transport 0's
  // receive buffer (one whole frame of its max_frame, + 2) and the answer buffer (an answer is at most its size: a probe
  // may answer within less than its max_frame, core §4.4). Before the first poll. false: the receive buffer too small
  // for the max_frame (nothing changed).
  bool setBuffers(uint8_t *rx_buffer, size_t rx_capacity, uint8_t *tx_buffer, size_t tx_capacity) {
    if (polled_ || serialKind(transports_[0].kind) || rx_capacity < transports_[0].max_frame + 2u || tx_capacity < 64)
      return false;
    transports_[0].reader.reset(rx_buffer, rx_capacity, transports_[0].max_frame);
    transports_[0].reader.setTcp(transports_[0].kind == kTcp);
    tx_ = tx_buffer;
    tx_capacity_ = tx_capacity;
    return true;
  }
  // A TCP listening socket (transports §1): one entry in fn 0's describe (kind 6, interface 0xFF), added after every
  // other transport, with `count` connection slots - each its own transport here (rx[k]: max_frame + 2 bytes). A
  // connection's requests are answered on it, its subscriptions' notifications go to it alone, a length over
  // max_frame closes it, and a pause inside a frame never restarts its read (transports §2). The session and the lock
  // are the probe's one, shared with every other transport (transports §3).
  bool addTcpListener(Connection *const *slots, uint8_t *const *rx, size_t rx_capacity, size_t count);
  size_t transportCount() const { return transport_count_; }
  bool isSerialPort(size_t index) const { return index < transport_count_ && serialKind(transports_[index].kind); }
  // The raw side of the serial ports (the binds); nullptr: raw bytes are dropped.
  void setRawPorts(RawPorts *raw) { raw_ = raw; }
  // A session holds the lock / holds serial port `port` (its raw transfer is stopped, transports §4).
  bool locked() const { return locked_; }
  bool held(size_t port) const { return locked_ && ((held_ >> port) & 1); }
  // The longest one request takes (describe max_op_ms, core §7.5): what every interface's long op is bounded by.
  static constexpr uint32_t kMaxOpMs = oep::kMaxOpMs;
  // The max_op_ms the describe declares: kMaxOpMs unless set. A probe whose ops all finish sooner may declare less
  // (1 to max_op_ms_max); its interfaces' own bounds must then fit within it.
  void setMaxOpMs(uint32_t ms) { if (ms >= 1 && ms <= reg::kLimitMaxOpMsMax) max_op_ms_ = ms; }
  // fn of an interface added (0: not added), and the interface at a fn (nullptr: none).
  uint16_t fnOf(const Interface &interface) const {
    for (size_t i = 0; i < count_; ++i) if (interfaces_[i] == &interface) return static_cast<uint16_t>(i + 1);
    return 0;
  }
  Interface *interfaceAt(uint16_t fn) const { return fn >= 1 && fn <= count_ ? interfaces_[fn - 1] : nullptr; }
  // The list's instance: the interface's own fixed identifier (core §7.2: kept across firmware updates, never counted
  // from the fn order or the list position; unique within one (name, revision), which add checks).
  uint16_t instanceOf(uint16_t fn) const {
    const Interface *it = interfaceAt(fn);
    return it ? it->instance() : 0;
  }

  // An interface, numbered fn 1, 2, ... in the order added. The list stays the same for a boot (core §7.2): every add
  // comes before the first poll(), and one after it is refused (false), as is a name outside core §13 rule 1 and an
  // interface that offers no op (its ops tag would have no valid encoding, core §7.4). At the first poll() the endpoint
  // lists its own interfaces after the sketch's, in this order: oep.probe.plan when an interface has plan roles,
  // oep.probe.restart when setRestart gave a handler - so the same firmware gives every interface the same fn and
  // instance at every boot, and a firmware that adds an interface moves only these two (the saved settings name their
  // interfaces by name, instance and revision and are renumbered, oep-if-probe-config §2; they never name these two).
  bool add(Interface &interface);
  bool anyPlanRoles() const {
    for (size_t i = 0; i < count_; ++i) if (interfaces_[i]->planRoles()) return true;
    return false;
  }
  // The fns of oep.probe.plan and oep.probe.restart (0: not listed) - before the first poll(), the fns they will have.
  uint16_t planFn() const {
    if (polled_) return fnOf(plan_iface_);
    return anyPlanRoles() ? static_cast<uint16_t>(count_ + 1) : 0;
  }
  uint16_t restartFn() const {
    if (polled_) return fnOf(restart_iface_);
    return restart_ ? static_cast<uint16_t>(count_ + 1 + (anyPlanRoles() ? 1 : 0)) : 0;
  }
  // The plan (oep-if-plan, per fn): the roles now applied (persistent_only: those set through oep.probe.config), and
  // a replacement of the fns listed that is all or nothing and outlives sessions (0: applied; else the reject reason,
  // with the plans before it applied again). A listed fn with no role is released.
  static constexpr size_t kMaxRoles = 64;   // the plan's role assignments, every fn together (describe plan_roles)
  size_t plan(RoleAssignment *out, size_t max, bool persistent_only = false) const;
  uint8_t replacePlan(const RoleAssignment *roles, size_t count, const uint16_t *fns, size_t nfns);
  // The unavailable answer for the last replacement refused unavailable (core §4.3: the cause, the channel that met
  // something).
  Result planUnavailable(uint8_t *out, size_t capacity) const {
    return unavailable(out, capacity, plan_refusal_.cause, plan_refusal_.channel);
  }
  // The channels oep.probe.config's disable items take away (channel < 64): a plan_apply naming one is refused
  // unavailable cause 5 (held by settings) with the channel (probe.config §1).
  void setDisabled(uint64_t mask) { disabled_ = mask; }
  // The pin table the interfaces' plans claim from: a plan replacement releases only the channels leaving the plan
  // (PinTable::deferIdle; oep-core §8). ProbeConfig::setPins sets it too.
  void setPins(PinTable *pins) { pins_ = pins; }
  bool disabled(uint16_t channel) const { return channel < 64 && (disabled_ >> channel) & 1; }
  // fn 0's describe: the probe itself, as TLV bytes (kept by the caller). Declarations only (core §7.3).
  void setProbeDescription(const uint8_t *tlv, size_t length) { probe_tlv_ = tlv; probe_tlv_length_ = length; }
  const uint8_t *probeDescription(size_t &length) const { length = probe_tlv_length_; return probe_tlv_; }
  // The boot_id returned by confirm, clock and open (core §6.5): a value that changes every boot. Without a
  // call the endpoint picks it itself when the first message arrives (bootIdSource: a hardware random source, or on a
  // platform without one the timer's count at that external event). A sketch with a better source of its own (a
  // counter it keeps in non-volatile storage) sets it in setup(); 0 is as good as any other value.
  void setBootId(uint32_t boot_id) { boot_id_ = boot_id; boot_id_set_ = true; }
  uint32_t bootId() {
    if (!boot_id_set_) setBootId(bootIdSource());
    return boot_id_;
  }
  // The optional port_speed (oep-if-link §3, oep.probe.link op 0x03): a host raises a UART bridge's baud for its session.
  // Setting a handler turns the op on in the probe's oep.probe.link (Link below: in its ops; without one it is
  // unknown_operation).
  // fn(port, baud, apply): apply false = the rate the port would run at for `baud` (0: this UART cannot make it, the
  // request is unsupported); apply true = switch the port to `baud` (its output already flushed) and return the rate it
  // runs at. `base` is the boot speed every revert goes back to. A rate further than port_speed_tolerance_pct from the
  // baud asked is refused unsupported (the endpoint checks it).
  using PortSpeedFn = uint32_t (*)(uint8_t port, uint32_t baud, bool apply);
  void setPortSpeed(PortSpeedFn fn, uint32_t base) { port_speed_ = fn; speed_base_ = base; }
  // TEST HOOK (host tests): port `port` at `rate` in phase `phase` (1 trying, 2 committed, 3 a return pending after a
  // try - `committed` false - or a committed rate), as if a port_speed had got it there.
  void testSpeed(uint8_t port, uint32_t rate, uint8_t phase, bool committed = false) {
    speed_port_ = port;
    speed_rate_ = speed_asked_ = rate;
    speed_state_ = phase == 1 || (phase == 3 && !committed) ? kSpeedTry : kSpeedCommitted;
    speed_fallback_ = phase == 3;
    speed_until_ = millis() + 1000;
    speed_good_ms_ = millis();
  }
  // The optional oep.probe.restart (oep-if-restart). Setting a handler before the first poll() lists the interface (after
  // the sketch's and oep.probe.plan; restartFn) with restart (0x01) and restart_max_ms (describe 0x40); without one
  // the probe does not list it. A call after the first poll() changes nothing (the list is fixed for the boot, core §7.2).
  // fn restarts the chip as from power-on and does not return (oep::platformRestart: esp_restart on an ESP32, the
  // watchdog on an RP2). max_ms: the longest from the answer leaving the transport until the probe answers confirm on
  // that transport again - boot, USB re-enumeration included.
  // A restart taken: the session's notifications end and the zero-copy data already queued goes out first (at most
  // kRestartDrainMs), then the answer; it is flushed (a UART's flush waits for its last bit), the session ends, every
  // interface lets go of its connections (probeRestart) and every plan goes, the settings' too, so each channel is in
  // its free state (core §8); kRestartSettleMs after the flush - the host's USB stack takes the last packet meanwhile -
  // fn is called. A probe on USB takes its device off the bus in fn, waits kRestartDetachMs and resets the chip
  // (kRestartResetMs to start; Oep.h): the reset starts within about 100 ms of the answer. From the answer on nothing
  // is served or sent (restarting()).
  using RestartFn = void (*)();
  static constexpr uint32_t kRestartSettleMs = 20, kRestartDrainMs = 200;
  void setRestart(RestartFn fn, uint32_t max_ms) {
    if (polled_) return;
    restart_ = fn;
    restart_max_ms_ = max_ms ? max_ms : 1;
  }
  bool restarting() const { return restarting_; }
  // The rate a sped-up port runs at now (0: every port at its boot speed), and whether it is committed (else trying).
  uint32_t portSpeedNow() const { return speed_state_ == kSpeedBase ? 0 : speed_rate_; }
  // port_state's phase of the port a request came on (oep-if-link §1): 0 boot, 1 trying, 2 committed, 3 a return pending
  uint8_t portPhase(uint8_t port) const {
    if (speed_state_ == kSpeedBase || port != speed_port_) return 0;
    if (speed_fallback_) return 3;
    return speed_state_ == kSpeedTry ? 1 : 2;
  }
  bool portSpeedCommitted() const { return speed_state_ == kSpeedCommitted; }
  // An event from an interface that notifies() (sent to the lock holder if it subscribed to that fn): not batched, it
  // goes out as soon as the answers to what has arrived have (core §11.3, §11.4); kept in a small queue until then
  // (oldest dropped when full - the seq gap shows it).
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
  // A zero-copy path for an interface's data pushes (length-prefixed framing only). The interface builds
  // whole push frames itself, from its own task: directPush says whether it may send now (the lock holder subscribed
  // its fn) with the fn and the subscriber's batching; takeSeq numbers a frame (shared with that fn's events).
  void setDirect(DirectTransport *direct) { direct_ = direct; }
  // A zero-copy path exists (whoever subscribes): a streaming configure can plan on it without a store of its own.
  bool directCapable() const { return directT() != nullptr && !serialKind(transports_[0].kind); }
  // the zero-copy path belongs to transport 0: pushes use it only while the subscriber came in on transport 0
  DirectTransport *direct() const {
    return push_ == 0 && !serialKind(transports_[0].kind) ? directT() : nullptr;
  }
  bool directPush(const Interface &from, uint16_t &fn, uint16_t &min_bytes, uint32_t &max_delay_ms) const;
  uint16_t takeSeq(uint16_t fn) { return __atomic_fetch_add(&push_seq_[fn - 1], 1, __ATOMIC_RELAXED); }
  void poll();

 private:
  struct Transport {
    Stream *stream = nullptr;
    Connection *conn = nullptr;   // a TCP listener's slot (addTcpListener)
    uint32_t seen = 0;            // the slot's connection the reader belongs to
    FrameReader reader;
    SerialReader serial;
    uint8_t kind = kVendorBulk, usb_interface = 0xff, index = 0;
    Endpoint *owner = nullptr;
    bool flush_after_burst = false, wrote = false;
    int tx_room_max = 0;
    uint16_t max_frame = 0;       // confirm's max_frame on this transport (core §4.4, §7.1: each route its own)
  };
  static void rawSink(void *context, const uint8_t *data, size_t length);
  void rawOut();
  void sessionEnded();
  void releaseLock();
  RawPorts *raw_ = nullptr;
  uint32_t held_ = 0;   // serial ports the lock holder's requests came in on (transports §4)
  uint8_t decode_[kMaxSerialFrame + 2];
  uint8_t owner_[32];   // the lock holder's owner text (core §6.4)
  uint8_t owner_length_ = 0;
  size_t appendOwner(uint8_t *out, size_t room, uint8_t tag) const;
  Transport transports_[kMaxTransports];
  size_t transport_count_ = 0;
  uint8_t entries_ = 0;   // fn 0's describe transport entries (a listener's slots are one)
  bool listener_ = false;
  size_t current_ = 0;   // the transport the message being handled came in on (results go back there)
  size_t push_ = 0;      // the transport the push subscriptions came in on
  uint32_t push_conn_ = 0;   // and its connection, on a TCP slot (another connection there gets none of them)
  void pushHere() { push_ = current_; push_conn_ = transports_[current_].seen; }
  bool pushGone() const { return connOf(transports_[push_]) && connOf(transports_[push_])->connection() != push_conn_; }
  uint8_t *tx_;
  size_t tx_capacity_;
  Limits limits_;
  Interface *interfaces_[kMaxInterfaces] = {};
  size_t count_ = 0;
  bool polled_ = false;   // poll() ran: the interface list is fixed (core §7.2)
  ProbePlan plan_iface_{*this};         // oep.probe.plan, listed by freeze() when an interface has plan roles
  ProbeRestart restart_iface_{*this};   // oep.probe.restart, listed by freeze() with a restart handler
  friend class ProbePlan;
  friend class ProbeRestart;
  void freeze();   // the first poll(): the endpoint's own interfaces after the sketch's, the list fixed
  const uint8_t *probe_tlv_ = nullptr;
  size_t probe_tlv_length_ = 0;
  uint64_t disabled_ = 0;                        // the settings' disabled channels (setDisabled)
  PinTable *pins_ = nullptr;                     // setPins
  // What the last plan replacement refused unavailable met (core §4.3's payload: cause, channel; 0xFFFF = left out).
  // replaceFns fills it.
  struct PlanRefusal { uint8_t cause; uint16_t channel; };
  PlanRefusal plan_refusal_ = {0, 0xffff};
  uint32_t boot_id_ = 0;
  bool boot_id_set_ = false;
  // port_speed (oep-if-link §3): one UART bridge at a time is off its boot speed, trying (verify_ms to be committed) or
  // committed (port_speed_idle_ms with no good frame - not counted while a request runs, like the lease - reverts). A
  // step that does not fit the port's state is unavailable cause 6. A switch or a revert asked by a request happens
  // after its answer is out.
  enum : uint8_t { kSpeedBase, kSpeedTry, kSpeedCommitted };
  enum : uint8_t { kSpeedNone, kSpeedSwitch, kSpeedRevert };
  PortSpeedFn port_speed_ = nullptr;
  uint32_t speed_base_ = 115200;
  uint8_t speed_state_ = kSpeedBase, speed_port_ = 0xff;
  uint32_t speed_asked_ = 0, speed_rate_ = 0, speed_until_ = 0, speed_good_ms_ = 0;
  bool speed_fallback_ = false;   // phase 3: a return condition holds, the frame on the line is let finish (link §3)
  uint8_t speed_pending_ = kSpeedNone, speed_pending_port_ = 0xff;
  uint32_t speed_pending_baud_ = 0, speed_pending_verify_ = 0;
  friend class Link;
  Result portSpeed(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result portState(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  size_t maxFrameIn() const { return transports_[current_].max_frame; }   // the request's route
  void speedApply();             // the switch or revert a request asked for, after its answer
  void speedRevert();            // back to the boot speed (nothing when there already)
  void speedPoll();              // the try deadline, the idle limit, a revert the session's end asked for
  void speedFall();              // a return condition: now, or phase 3 while a frame is being received
  static bool speedWithin(uint32_t rate, uint32_t baud);   // port_speed_tolerance_pct
  RestartFn restart_ = nullptr;  // setRestart: oep.probe.restart listed
  uint32_t restart_max_ms_ = 0;
  bool restart_taken_ = false;   // this request is a restart answered success: the probe restarts after the answer
  volatile bool restarting_ = false;
  void restartNow();
  volatile bool locked_ = false;
  uint32_t holder_ = 0, last_ = 0;
  bool have_last_ = false;
  uint32_t lease_ms_ = kLeaseDefaultMs, expires_ms_ = 0;
  uint8_t scratch_[OEP_SMALL_RAM ? 512 : 768];   // one interface's full describe (after the ops tag) before paging

  uint32_t max_op_ms_ = kMaxOpMs;

  static constexpr int kReservedOps = 0xF0;   // 0xF0..0xFF: reserved, never in ops (core §2.5)
  void handleMessage(const uint8_t *message, size_t length);
  bool coreOffers(uint8_t op) const;
  bool offersOp(uint16_t fn, uint8_t op) const;
  bool lockFreeOp(uint16_t fn, uint8_t op) const;
  size_t opsTlv(uint16_t fn, uint8_t *out, size_t capacity) const;
  Result core(uint8_t op, uint32_t session, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result list(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result describe(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result open(uint32_t session, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result planApply(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result planReleaseRequest(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  uint16_t replaceFns(const RoleAssignment *roles, size_t count, const uint16_t *fns, size_t nfns, bool persistent);
  void planRelease(const uint16_t *fns, size_t nfns);
  bool planned_[kMaxInterfaces] = {};
  bool persistent_[kMaxInterfaces] = {};   // planned through replacePlan (oep.probe.config): a lapse does not release it
  // Subscriptions, per fn (core §11.3: one per fn, the lock holder's).
  volatile bool subscribed_[kMaxInterfaces] = {};
  DirectTransport *direct_ = nullptr;
  // A small-RAM build (OEP_SMALL_RAM: one USB vendor bulk transport) has no TCP listener and no zero-copy transport:
  // these are constant nulls there, and what they lead to compiles out
  DirectTransport *directT() const { return OEP_SMALL_RAM ? nullptr : direct_; }
  static Connection *connOf(const Transport &t) { return OEP_SMALL_RAM ? nullptr : t.conn; }
  uint16_t push_seq_[kMaxInterfaces] = {};
  uint16_t min_bytes_[kMaxInterfaces] = {};
  uint32_t max_delay_ms_[kMaxInterfaces] = {};
  uint32_t waiting_since_[kMaxInterfaces] = {};
  bool waiting_[kMaxInterfaces] = {};
  size_t draining_[kMaxInterfaces] = {};   // a batch due: the bytes still to send of what was pending then
  // events: fn (1 and up: fn 0 sends none, core §11.2), seq per fn shared with data frames
  struct Event { uint16_t fn; uint16_t seq; uint8_t kind; uint8_t length; uint8_t payload[40]; };
  static constexpr size_t kEvents = OEP_SMALL_RAM ? 8 : 32;
  Event events_[kEvents];
  uint32_t event_head_ = 0, event_tail_ = 0;
  bool queueEvent(uint16_t fn, uint8_t kind, const uint8_t *payload, size_t length);
  bool sendEvents();
  size_t push_queue_ = 1024;
  Result subscription(uint16_t fn, uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  void push();
  void endSubscriptions();
  void send(size_t length);
  // Dedup of the last session's requests sent again (core §5.2): the last kDedupEntries (corr, answer), keyed on corr
  // alone; dropped at every successful open (kept over end, lapse, force). Results over kDedupBytes are not kept
  // (rejected result_lost): kDedupBytes covers a whole frame of the largest profile, so a 1 KiB read_block whose answer
  // was corrupted on a CP2102 link comes back from here instead of being read again (V003 jig, 2026-10-01).
  // A small-RAM build: 4 (over its max_inflight) of a 512-byte frame.
  static constexpr size_t kDedupEntries = OEP_SMALL_RAM ? 4 : 8, kDedupBytes = (OEP_SMALL_RAM ? 512 : 1024) + 5;
  // The request is kept too, up to kDedupRequestBytes (core §5.1: a resent request is compared byte for byte with the
  // one recorded - a different one is malformed; one whose bytes are not kept is result_lost).
  static constexpr size_t kDedupRequestBytes = 64;
  struct Dedup {
    bool used = false, kept = false;
    uint16_t corr = 0;
    uint16_t length = 0, request_length = 0;   // request_length 0: the request's bytes not kept
    uint8_t result[kDedupBytes];
    uint8_t request[kDedupRequestBytes];
  };
  bool fresh_session_ = false;   // this request is an open that began a new session (its corr starts the order, §5.1)
  Dedup dedup_[kDedupEntries];
  size_t dedup_next_ = 0;
  uint16_t newest_corr_ = 0;   // the last session's newest request (core §4.1: the host numbers them in order)
  bool have_newest_ = false;
  RoleAssignment plan_roles_[kMaxRoles] = {};
  size_t plan_count_ = 0;
  Result checkSession(uint32_t session, uint8_t *out, size_t capacity);
  void lapse();
  void loseSession();
  void sendReject(uint16_t corr, uint8_t reason);
  uint32_t remaining() const;
};

// oep.probe.link (oep-if-link): the link test - source (length(u32) -> len(u16) data, byte k = k & 0xFF, at most
// max_frame - 7: the answer's header and len) and sink (count(u16) data -> nothing; at most max_frame - 12: the
// request's header 10 and count 2) - and, when the endpoint has a port_speed handler (setPortSpeed), port_speed on a
// UART bridge. An optional interface: a probe lists at most one; add it like any other (endpoint.add(link)).
class Link final : public Interface {
 public:
  explicit Link(Endpoint &endpoint) : endpoint_(endpoint) {}
  const char *name() const override { return reg::probe_link::kName; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return reg::probe_link::kRevision; }
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::probe_link::kLockFreeOps, op); }
  bool offers(uint8_t op) const override {
    return op == reg::probe_link::kOpSource || op == reg::probe_link::kOpSink ||
           ((op == reg::probe_link::kOpPortSpeed || op == reg::probe_link::kOpPortState) && !OEP_SMALL_RAM &&
            endpoint_.port_speed_ != nullptr);
  }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  Endpoint &endpoint_;
};

}  // namespace oep
