// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.target.console revision 1 (oep-spec docs/oep-if-console.ja.md): a stream opened on
// the debug connection (SDI / DMDATA / dmseq through the DmConsole driver), kept in a position-addressed buffer that
// reads do not consume, with marks for what happened.
//
//   open(connection u16, mechanism u8) [TLV]  -> stream u16, flags u8 (bit0 an existing stream) [TLV]
//   read(stream, from u8, arg u64, max u16)   -> start u64, flags u8 (bit0 more, bit1 gap), len u16, data   no lock
//   marks(stream, from_serial u32)            -> more u8, count u8, count x (serial u32, position u64,
//                                                     kind u8, time_ns u64, detail u8)                   no lock
//   clear(stream)   mark(stream, value u8)   write(stream, count u16, data) -> accepted u16
//   close(stream)   streams(first u16) -> more u8, count u8, count x (stream u16, connection u16,
//                                                                      mechanism u8, users u8, state u8)   no lock
//
// One live stream on the one connection; its number is from the probe's one space (core §9). Opening the same mechanism
// again hands it back with its positions and marks; another mechanism while one is open is unavailable (cause 6). The
// stream is used by the host's session (open) and by a slot (its bind): close and the session's end take the host's share, the
// settings take the slot's, and the stream closes (mark closed) when nobody is left. A stream whose connection went
// away is closed (mark link-lost when the line was lost, then closed 4) and stays readable (read / marks) until the
// same mechanism is opened again at the same place, which gives the same number back (mark attach, flags bit0); another
// mechanism there starts a new stream. Collection goes on whatever the host does; bytes go only when the buffer
// overflows, on clear, or when the probe restarts, and positions never go back within a boot.
#pragma once

#include "OepDmConsole.h"
#include "OepStream.h"
#include "OepTarget.h"

namespace oep {

class TargetConsoleStream final : public Interface, public BindSource {
 public:
  enum : uint8_t {
    kOpOpen = reg::target_console::kOpOpen, kOpRead = reg::target_console::kOpRead,
    kOpMarks = reg::target_console::kOpMarks, kOpClear = reg::target_console::kOpClear,
    kOpMark = reg::target_console::kOpMark, kOpWrite = reg::target_console::kOpWrite,
    kOpClose = reg::target_console::kOpClose, kOpStreams = reg::target_console::kOpStreams,
  };
  enum : uint8_t {
    kMarkReset = reg::target_console::kMarkKindReset, kMarkRestart = reg::target_console::kMarkKindRestart,
    kMarkLost = reg::common::kMarkKindLost, kMarkClear = reg::common::kMarkKindClear, kMarkHost = reg::common::kMarkKindHost,
  };
  enum : uint8_t { kUserHost = legacy::kStreamUsersHostSession, kUserSlot = legacy::kStreamUsersSlot };
  TargetConsoleStream(DebugPort &port, DmConsole &driver, uint16_t instance)
      : port_(port), driver_(driver), instance_(instance), stream_(buffer_, kCapacity, marks_, kMarks) {
    driver_.setSink(&TargetConsoleStream::take, this, &TargetConsoleStream::takeMark);   // straight into this buffer
  }
  const char *name() const override { return reg::target_console::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::target_console::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::target_console::kLockFreeOps, op); }
  // open / read / close / streams always; marks, clear, mark and write while offerOptional (console §1: the minimum is
  // open / read / close / streams, without the marks' ring or a send queue)
  bool offers(uint8_t op) const override {
    if (!opIn(op, reg::target_console::kOpOpen, reg::target_console::kOpStreams)) return false;
    return optional_ || op == kOpOpen || op == kOpRead || op == kOpClose || op == kOpStreams;
  }
  void offerOptional(bool on) { optional_ = on; }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  // The host's session ended (end, a lapse, a takeover): its share of the stream goes - before the wire's share of the
  // connection (sessionOverFirst) - and a stream it was the last user of closes with mark closed 2 (session_ended),
  // readable until the same place and mechanism is opened again (oep-if-console §2).
  void sessionOver() override { closeStream(); }   // the session's stream goes with it (console §2)
  bool sessionOverFirst() const override { return true; }
  // oep.probe.restart (oep-if-restart §2): the slot's share goes too, before the wire closes the connection
  void probeRestart() override { closeStream(); }
  void poll();   // from loop()
  // What one read may return: declare it within the probe's frame (1000 suits a 1 KiB frame).
  void setMaxRead(uint16_t bytes) { max_read_ = bytes; }
  // A slot's bind opens the stream itself on the connection (the same stream a host's open gets back), taking the
  // slot's share. false: no connection, or another mechanism is open.
  bool bindOpen(uint8_t mechanism);
  // The slot's share goes (its bind no longer carries it, or the slot was replaced / removed: mark closed 3 when last).
  void bindClose(uint8_t = 0) {}   // the old binds (the pass-through takes their place)
  bool isOpen() const { return open_; }
  uint8_t users() const { return open_ ? kUserHost : 0; }
  // The driver's mechanisms (DmConsole's own ids, not wire numbers): describe lists their names in this order and the
  // k-th listed is mechanism number k on the wire (console §1, core §7.7).
  enum : uint8_t { kDriverSdi = 0, kDriverDmdata = 1, kDriverDmseq = 2, kDrivers = 3 };
  static constexpr const char *kDriverNames[kDrivers] = {reg::target_console::kNameMechanismSdi,
                                                         reg::target_console::kNameMechanismDmdata,
                                                         reg::target_console::kNameMechanismDmseq};
  // The mechanisms describe declares (at least one): bit d = driver mechanism d.
  void declareMechanisms(uint8_t mask) { mechanisms_ = mask; }
  // Wire number (from 1) <-> driver mechanism; -1 / 0: not declared.
  int8_t driverOf(uint8_t number) const;
  uint8_t numberOf(uint8_t driver) const;
  // console §2.1: the console read for a pass-through on the port's resident link, its output to `sink` (no
  // stream, no number, no mark); false: a session's stream is open, or the mechanism did not start.
  using PassSink = void (*)(void *context, const uint8_t *data, size_t length);
  bool passStart(uint8_t number, PassSink sink, void *context);   // number: the wire's mechanism number
  void passStop();
  bool passing() const { return pass_sink_ != nullptr; }
  bool passLost() const { return pass_lost_; }
  bool offersMechanism(uint8_t number) const { return driverOf(number) >= 0; }   // a wire number
  DebugPort &port() const { return port_; }
  // The interface the host sees it through: a TargetConsole that holds it as one of its places, or itself. The settings'
  // pass-through items belong to that one (common §2: one oep.target.console per probe).
  const Interface &front() const { return front_ ? *front_ : *this; }
  void setFront(const Interface *front) { front_ = front; }
  uint32_t streamNumber() const { return stream_number_; }
  // Its streams entry (stream u32, connection u32, mechanism u8: 9 bytes) while a stream is open; false: none.
  bool streamEntry(uint8_t *out) const;
  // BindSource: the stream while it exists (a closed one too, until another is opened), the target's input.
  const PositionStream *bindStream() const override { return exists_ ? &stream_ : nullptr; }
  uint16_t bindStreamNumber() const override { return static_cast<uint16_t>(stream_number_); }
  size_t bindInput(const uint8_t *data, size_t length) override;

 private:
  // both powers of two (wrapping positions / serials); a small-RAM build (OEP_SMALL_RAM) keeps 1 KiB of console
  static constexpr size_t kCapacity = OEP_SMALL_RAM ? 1024 : 8192, kMarks = OEP_SMALL_RAM ? 8 : 16;
  DebugPort &port_;
  DmConsole &driver_;
  uint16_t instance_;
  const Interface *front_ = nullptr;
  bool exists_ = false;    // a stream was opened (and may be closed but still readable)
  uint32_t stream_number_ = 0;   // the live stream's number (the probe's one space, core §9)
  uint32_t connection_ = 0;      // the connection it was opened on (the streams entry)
  uint8_t mechanisms_ = 0x07;    // SDI, DMDATA, dmseq
  bool optional_ = true;         // marks / clear / mark / write offered
  PassSink pass_sink_ = nullptr;
  void *pass_ctx_ = nullptr;
  bool pass_lost_ = false;
  bool open_ = false;
  uint8_t mechanism_ = 0;
  uint16_t max_read_ = 1000;
  uint8_t buffer_[kCapacity];
  PositionStream::Mark marks_[kMarks];
  PositionStream stream_;
  uint32_t seen_resets_ = 0, seen_resyncs_ = 0, seen_restarts_ = 0, seen_closes_ = 0;
  bool checking_ = false;   // streamOp only checks the request's form and values (handle: the number after)
  Result streamOp(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity);
  uint32_t openStream(uint8_t mechanism);   // 0: no number left; 0xFFFFFFFF: the driver did not start
  void closeStream();
  void release(uint8_t user, uint8_t detail);
  static void take(void *self, uint8_t byte) {
    TargetConsoleStream *t = static_cast<TargetConsoleStream *>(self);
    if (t->pass_sink_) t->pass_sink_(t->pass_ctx_, &byte, 1);   // a pass-through: to its port only
    else t->stream_.put(byte);
  }
  static void takeMark(void *self, uint8_t kind, uint8_t detail) {
    TargetConsoleStream *t = static_cast<TargetConsoleStream *>(self);
    if (!t->pass_sink_) t->stream_.mark(kind, detail);
  }
};

// oep.target.console as the probe's one (common §2: one per name, taking the connections of every wire mapped to it) over
// a place per wire: each place a TargetConsoleStream on its own port. A request goes to the place it names - open by its
// connection, the stream ops by their stream - and streams lists every place's. Add this one to the endpoint, not the
// places; the places' poll() stays the sketch's. A probe with one wire may add its TargetConsoleStream alone.
class TargetConsole final : public Interface {
 public:
  static constexpr size_t kMaxPlaces = 2;
  explicit TargetConsole(TargetConsoleStream &first) { add(first); }
  bool add(TargetConsoleStream &place);
  const char *name() const override { return reg::target_console::kName; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return reg::target_console::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override { return places_[0]->describe(out, capacity); }
  bool lockFree(uint8_t op) const override { return places_[0]->lockFree(op); }
  bool offers(uint8_t op) const override { return places_[0]->offers(op); }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  void sessionOver() override { for (size_t k = 0; k < count_; ++k) places_[k]->sessionOver(); }
  bool sessionOverFirst() const override { return true; }
  void probeRestart() override { for (size_t k = 0; k < count_; ++k) places_[k]->probeRestart(); }

 private:
  TargetConsoleStream *places_[kMaxPlaces] = {};
  size_t count_ = 0;
};

}  // namespace oep
