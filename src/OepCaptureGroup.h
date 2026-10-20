// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.capture-group revision 1 (oep-spec docs/oep-if-capture.ja.md §4): capture / analog tracks started
// together. Each track keeps its own configure and is read as usual; the group binds them, starts and stops them at once
// and says when it started. Every track stamps its own first sample on the probe's one clock (segment start_ns), so a
// track's offset is its first segment's start_ns minus the group's.
//
//   0x01 bind(n u8, n x fn u16, [TLV 0x01 trigger_track fn]) -> -
//   0x02 start -> start_ns u64, generation u32, n u8, n x (fn u16, generation u32) (bind order)
//   0x03 stop   0x04 force   0x05 status -> state u8, start_ns u64, trigger_ns u64, trigger_fn u16, generation u32 (no lock)
//   events: 0x03 triggered (trigger_fn u16, trigger_ns u64, generation u32), 0x02 stopped (reason u8, error u8,
//   generation u32) - the group's generation, one up at every start of the group (capture §4.1, §4.2)
//
// With trigger_track, that track waits for its own trigger and the others follow: they run into their rings from the
// start (the trigger track starts once each holds its pretrigger), and when the trigger's time is known (trackTriggerNs)
// the group hands it to them (trackTriggerAt), and each cuts its segment around the sample nearest to it, with its own
// pretrigger. A track that cannot follow (no ring) is refused at bind, and so is a set of tracks over a budget they
// share (addBudget): the probe's own limit, declared nowhere (capture §4.3: a host tries bind).
#pragma once
#include <Arduino.h>
#include "Oep.h"

namespace oep {

class Endpoint;

// What a capture interface gives a group: whether it can be bound, and a start / stop that the group drives. While
// bound, the interface refuses its own configure / start / stop / force (rejected unavailable cause 4).
class GroupTrack {
 public:
  virtual ~GroupTrack() = default;
  virtual bool trackReady() const = 0;        // configured and not running
  // a start would be taken now (capture §3.2: configured, complete or error; streaming: subscribed) - the group checks
  // every track before it starts any (§4.1)
  virtual bool trackCanStart() const { return trackReady(); }
  virtual uint8_t trackMode() const = 0;      // the configured mode (one-shot, repeat, streaming)
  virtual bool trackTriggered() const = 0;    // a trigger other than immediate is configured
  virtual uint32_t trackLoad() const = 0;     // channels x rate, sample/s (for the group's budgets)
  virtual bool trackStart() = 0;              // start now; false: it could not
  virtual void trackStop() = 0;
  virtual uint8_t trackState() const = 0;     // the capture state (oep-if-capture §3.2)
  // in state 6, why (status's error): 1 the peripheral, 2 a segment lost to the queue or the ring (capture §2.2)
  virtual uint8_t trackError() const { return reg::fixture_logic::kErrorPeripheral; }
  virtual uint32_t trackGeneration() const { return 0; }   // the generation the last start made (the group's start answer)
  // Following another track's trigger (the group's trigger_track): trackCanFollow at bind (no side effects; false:
  // this track cannot, as configured), trackStartFollowing at start (running into its ring, waiting for
  // trackTriggerAt), trackTriggerAt(ns) with the trigger's time on the probe's clock. The trigger track:
  // trackTriggerNs once its trigger is found, trackForce.
  virtual bool trackCanFollow() const { return false; }
  virtual bool trackStartFollowing() { return false; }
  virtual void trackTriggerAt(uint64_t ns) { (void)ns; }
  virtual bool trackTriggerNs(uint64_t &ns) const { (void)ns; return false; }
  virtual void trackForce() {}
  virtual bool trackArmed() const { return true; }   // following: it holds its pretrigger's worth of samples
  // The group's pretrigger (capture §4.1): the trigger track's own P samples (trackPretrigger) at its actual rate
  // (trackRate), kept by every follower k as the same time, P_k samples at its rate (groupPretrigger). trackCanKeep: a
  // follower can keep p samples before the trigger (within its max_pretrigger and, mode 1 / 2, below its samples);
  // trackKeep(p) sets what its following keeps.
  virtual bool trackRate(uint32_t &num, uint32_t &den) const { num = den = 0; return false; }
  virtual uint32_t trackPretrigger() const { return 0; }
  // The trigger is known to the group only some time after its sample (trackLatencyNs, the trigger track's most: a
  // DMA chunk to be searched, a loop round to come by); until then a follower's samples run on into its ring, so it
  // keeps late_ns more than P_k and its segment - bind refuses one that cannot, cause 2 like a P_k it cannot keep.
  virtual bool trackCanKeep(uint32_t p, uint64_t late_ns) const { (void)late_ns; return p == 0; }
  virtual void trackKeep(uint32_t p, uint64_t late_ns) { (void)p; (void)late_ns; }
  // The longest from the trigger's sample to trackTriggerNs giving it here and the group's poll handing it on: the
  // track's own search delay plus kLoopNs for loop() to come round (a stop of another track, a long request).
  static constexpr uint64_t kLoopNs = 100000000;
  virtual uint64_t trackLatencyNs() const { return kLoopNs; }
  void setBound(bool on, uint16_t group_fn = 0) { bound_ = on; group_fn_ = on ? group_fn : 0; if (!on) following_ = false; }
  bool bound() const { return bound_; }
  uint16_t groupFn() const { return group_fn_; }   // the group's fn while bound (0: not bound)

 protected:
  bool bound_ = false;
  uint16_t group_fn_ = 0;
  bool following_ = false;   // bound as a follower: start with trackStartFollowing
  friend class CaptureGroup;
};

// A bound track's own configure / start / stop / force, and plan changes of its fn: rejected unavailable cause 4, the
// cause alone (oep-if-capture §4.1, core §4.3).
inline Result boundInGroup(const GroupTrack &track, uint8_t *out, size_t capacity) {
  (void)track;
  return unavailable(out, capacity, reg::core::kUnavailableCauseBoundInGroup);
}

// A capture's next generation (capture §3.2 / §4.1): one up at every start, 1 after 0xFFFFFFFF - 0 is only before the
// first start of the boot, and generations are compared for equality only.
constexpr uint32_t nextGeneration(uint32_t generation) { return generation == 0xFFFFFFFFu ? 1u : generation + 1u; }

// capture §3.3's contract, the same for configure and query and for the logic and analog tracks (one tag space), on
// the TLVs as parsed (value or nullptr after their lengths were checked, and whether bit 7 was set).
struct CaptureAsk { const uint8_t *v; bool critical; };
namespace capture_tag {
constexpr uint8_t kMode = reg::fixture_logic::kTlvConfigureMode, kRate = reg::fixture_logic::kTlvConfigureRate,
                  kSamples = reg::fixture_logic::kTlvConfigureSamples, kSegments = reg::fixture_logic::kTlvConfigureSegments,
                  kTrigger = reg::fixture_logic::kTlvConfigureTrigger, kPretrigger = reg::fixture_logic::kTlvConfigurePretrigger,
                  kTriggerOpts = reg::fixture_logic::kTlvConfigureTriggerOpts;
static_assert(kMode == reg::fixture_analog::kTlvConfigureMode && kRate == reg::fixture_analog::kTlvConfigureRate &&
              kSamples == reg::fixture_analog::kTlvConfigureSamples && kSegments == reg::fixture_analog::kTlvConfigureSegments &&
              kTrigger == reg::fixture_analog::kTlvConfigureTrigger && kPretrigger == reg::fixture_analog::kTlvConfigurePretrigger &&
              kTriggerOpts == reg::fixture_analog::kTlvConfigureTriggerOpts,
              "logic and analog share the configure tags");
}  // namespace capture_tag
// trigger_opts (capture §3.3): arm(u8) rearm(u8) when(u8) hysteresis(u32), 7 bytes. The defaults - arm 1 (looked at
// once the pretrigger is in), rearm 0, when 0 (a level counts from false to true), hysteresis 0 - are every probe's;
// the rest only as describe's trigger opts say (`opts`: bit 0 arm 0, bit 1 rearm 1, bit 2 when 1, bit 3 hysteresis).
// false: a value it does not handle (refused unsupported, the tag as received).
struct TriggerOpts { uint8_t arm = 1, rearm = 0, when = 0; uint32_t hysteresis = 0; };
constexpr uint8_t kTriggerOptsBytes = 7;
constexpr uint8_t kTriggerOptsArm0 = 1, kTriggerOptsRearm = 2, kTriggerOptsWhen1 = 4, kTriggerOptsHysteresis = 8;
inline bool triggerOpts(const uint8_t *v, uint8_t opts, TriggerOpts &o) {
  o = TriggerOpts();
  if (!v) return true;
  o.arm = v[0];
  o.rearm = v[1];
  o.when = v[2];
  o.hysteresis = getU32(v + 3);
  if (o.arm > 1 || o.rearm > 1 || o.when > 1) return false;
  return (o.arm || (opts & kTriggerOptsArm0)) && (!o.rearm || (opts & kTriggerOptsRearm)) &&
         (!o.when || (opts & kTriggerOptsWhen1)) && (!o.hysteresis || (opts & kTriggerOptsHysteresis));
}
// The triggered event's fixed part (capture §3.4): serial trigger_index record(0xFFFFFFFF: none, rearm 0)
// sample(u64: from the generation's start) trigger_ns(u64) uncertainty_ns generation - 36 bytes.
constexpr size_t kTriggeredBytes = 36;
inline void putTriggered(uint8_t *e, uint32_t serial, uint32_t index, uint64_t sample, uint64_t ns, uint32_t uncertainty,
                         uint32_t generation) {
  putU32(e, serial);
  putU32(e + 4, index);
  putU32(e + 8, 0xFFFFFFFFu);
  putU64(e + 12, sample);
  putU64(e + 20, ns);
  putU32(e + 28, uncertainty);
  putU32(e + 32, generation);
}
// One sample period at num / den Hz in ns, rounded up (a triggered event's uncertainty: +- a sample).
inline uint32_t periodNs(uint32_t num, uint32_t den) {
  return num ? static_cast<uint32_t>((1000000000ull * den + num - 1) / num) : 0;
}
// describe's trigger (capture §3.5): types(u32) max_pretrigger(u32) opts(u8), 9 bytes
inline void putTriggerDecl(uint8_t *v, uint32_t types, uint32_t max_pretrigger, uint8_t opts) {
  putU32(v, types);
  putU32(v + 4, max_pretrigger);
  v[8] = opts;
}

// Data format 4, the test sequence (capture §1): the byte at stream position k is k & 0xFF. Written over what a frame
// or a read carries, so the bytes go the real path at format 0's volume and pace.
inline void putTestCounter(uint8_t *d, uint64_t position, size_t n) {
  size_t j = 0;   // to an aligned address first: a misaligned word store is a trap the P4 emulates (15 cycles a byte)
  for (; j < n && (reinterpret_cast<uintptr_t>(d + j) & 3); ++j) d[j] = static_cast<uint8_t>(position + j);
  uint32_t word = 0;
  for (int b = 0; b < 4; ++b) word |= static_cast<uint32_t>(static_cast<uint8_t>(position + j + b)) << (8 * b);
  for (; j + 4 <= n; j += 4) {   // 4 bytes at a time: each byte up 4, wrapping in its own lane (no carry across)
    memcpy(__builtin_assume_aligned(d + j, 4), &word, 4);
    word = ((word & 0x7F7F7F7Fu) + 0x04040404u) ^ (word & 0x80808080u);
  }
  for (; j < n; ++j) d[j] = static_cast<uint8_t>(position + j);
}

// Malformed (core §2.3): mode or rate missing, a value the definition excludes (rate 0, samples or segments 0), samples
// missing in mode 1 / 2.
inline bool captureMalformed(CaptureAsk mode, CaptureAsk rate, CaptureAsk samples, CaptureAsk segments) {
  if (!mode.v || !rate.v || getU32(rate.v) == 0) return true;
  if ((samples.v && getU32(samples.v) == 0) || (segments.v && getU32(segments.v) == 0)) return true;
  return (mode.v[0] == reg::fixture_logic::kModeOneShot || mode.v[0] == reg::fixture_logic::kModeRepeat) && !samples.v;
}
// Unsupported whatever the value, the tag as received: samples in mode 3, segments outside mode 2, pretrigger without a
// trigger or with an immediate one. `mode` is a mode the probe takes (refused before this otherwise). Completed: none.
inline Result captureMisplaced(uint8_t mode, CaptureAsk samples, CaptureAsk segments, CaptureAsk trigger,
                               CaptureAsk pretrigger, uint8_t *out, size_t capacity) {
  if (samples.v && mode == reg::fixture_logic::kModeStreaming)
    return Tail::refuse(capture_tag::kSamples, samples.critical, out, capacity);
  if (segments.v && mode != reg::fixture_logic::kModeRepeat)
    return Tail::refuse(capture_tag::kSegments, segments.critical, out, capacity);
  if (pretrigger.v && (!trigger.v || trigger.v[0] == reg::fixture_logic::kTriggerImmediate))
    return Tail::refuse(capture_tag::kPretrigger, pretrigger.critical, out, capacity);
  return completed();
}

// capture §4.1: P_k = ceil(p x num_k x den_t / (den_k x num_t)) - the products may pass 64 bits (96 at most), so a
// long division of the 128-bit product. false: no rate known, or more than a u32.
inline bool groupPretrigger(uint32_t p, uint32_t num_t, uint32_t den_t, uint32_t num_k, uint32_t den_k, uint32_t &out) {
  if (!num_t || !den_t || !num_k || !den_k) return false;
  const uint64_t a = static_cast<uint64_t>(num_k) * den_t, b = static_cast<uint64_t>(den_k) * num_t;
  // p x a as hi:lo (u64 each) from 32-bit halves
  const uint64_t a_lo = a & 0xFFFFFFFFu, a_hi = a >> 32;
  const uint64_t x = p * a_lo, y = p * a_hi;   // p < 2^32: each fits u64
  uint64_t lo = x + (y << 32), hi = (y >> 32) + (lo < x ? 1 : 0);
  uint64_t q_hi = 0, q_lo = 0, r = 0;
  for (int i = 127; i >= 0; --i) {   // restoring division by b; r < b < 2^64, r << 1 checked for the carry
    const uint64_t bit = i >= 64 ? (hi >> (i - 64)) & 1 : (lo >> i) & 1;
    const bool carry = r >> 63;
    r = (r << 1) | bit;
    if (carry || r >= b) {
      r -= b;
      if (i >= 64) q_hi |= uint64_t{1} << (i - 64);
      else q_lo |= uint64_t{1} << i;
    }
  }
  if (r) { if (++q_lo == 0) ++q_hi; }   // ceil
  if (q_hi || q_lo > 0xFFFFFFFFu) return false;
  out = static_cast<uint32_t>(q_lo);
  return true;
}

// The sample (of a rate num / den a second) nearest to dns ns after the first one.
inline uint64_t samplesIn(uint64_t dns, uint32_t num, uint32_t den) {
  const uint64_t q = dns / 1000000000u, r = dns % 1000000000u;
  const uint64_t a = q * num;   // whole seconds' samples x den
  return a / den + ((a % den) * 1000000000u + r * num + static_cast<uint64_t>(den) * 500000000u) / (static_cast<uint64_t>(den) * 1000000000u);
}

class CaptureGroup final : public Interface {
 public:
  static constexpr size_t kMaxTracks = 4;
  CaptureGroup(Endpoint &endpoint, uint16_t instance = 0) : endpoint_(endpoint), instance_(instance) {}
  // A track the group may bind (add both to the endpoint first; describe's tracks).
  bool addTrack(Interface &interface, GroupTrack &track);
  // A budget the listed tracks share when bound together: channels x rate summed, sample/s. Not declared: a bind over
  // it is refused unavailable cause 2 with the track's fn (capture §4.1).
  bool addBudget(uint32_t max_rate, GroupTrack &a, GroupTrack &b);
  const char *name() const override { return reg::fixture_capture_group::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_capture_group::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::fixture_capture_group::kLockFreeOps, op); }
  // bind .. status, force (optional) included: the ops tag declares them (core §1.2)
  bool offers(uint8_t op) const override {
    using namespace reg::fixture_capture_group;
    return op == kOpBind || op == kOpStart || op == kOpStop || op == kOpStatus;
  }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  bool notifies() const override { return true; }   // subscribe / unsubscribe in its ops (core §11.3)
  bool subscribe(bool on) override { subscribed_ = on; return true; }
  // The bind is the session's (core §9): its end (end, a lapse, a takeover) unbinds.
  void sessionOver() override { unbind(); }
  void poll();   // from loop(): the stopped event once every bound track is done

 private:
  struct Entry { Interface *interface; GroupTrack *track; };
  struct Budget { uint32_t max_rate; uint8_t a, b; };
  Endpoint &endpoint_;
  uint16_t instance_;
  Entry tracks_[kMaxTracks] = {};
  size_t count_ = 0;
  Budget budgets_[kMaxTracks] = {};
  size_t budget_count_ = 0;
  uint8_t bound_[kMaxTracks] = {};   // indexes into tracks_
  size_t bound_count_ = 0;
  uint64_t start_ns_ = ~uint64_t{0};
  uint32_t generation_ = 0;   // the group's (capture §4.1): one up at every start, kept across binds
  bool subscribed_ = false;
  bool started_ = false;   // started since the bind: the state is the tracks' (done once all are)
  bool running_ = false;   // the stopped event is still to come
  int trigger_ = -1;       // the trigger track (index into tracks_), or none
  bool trigger_pending_ = false;   // started, the trigger track not yet: the followers' pretriggers are filling
  bool startTrigger();
  uint64_t trigger_ns_ = ~uint64_t{0};
  bool forced_ = false;    // the trigger was forced: triggered says trigger_fn 0
  int indexOf(uint16_t fn) const;
  int indexOf(const GroupTrack &track) const;
  uint8_t state() const;
  bool failed_ = false;    // a track failed after the start: the group is in state 6 until the next start or bind
  Result refuseTrack(uint8_t cause, uint16_t fn, uint8_t *out, size_t capacity) const;
  void unbind();
  void stopAll(uint8_t reason, uint8_t error);
};

}  // namespace oep
