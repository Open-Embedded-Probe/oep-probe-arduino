// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.capture-group revision 1 (oep-spec docs/oep-if-capture.ja.md §4): capture / analog tracks started
// together. Each track keeps its own configure and is read as usual; the group binds them, starts and stops them at once
// and says when it started. Every track stamps its own first sample on the probe's one clock (segment start_ns), so a
// track's offset is its first segment's start_ns minus the group's.
//
//   0x01 bind(n u8, n x fn u16, [TLV 0x01 trigger_track fn]) -> -
//   0x02 start -> blocking_ms u32, start_ns u64, [TLV 0x01 generations: n x (fn u16, generation u32)]
//   0x03 stop   0x04 force   0x05 status -> state u8, start_ns u64, trigger_ns u64, trigger_fn u16 (no lock)
//   events: 0x03 triggered (trigger_fn u16, trigger_ns u64), 0x02 stopped (reason u8, error u8)
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
  virtual uint32_t trackGeneration() const { return 0; }   // the generation the last start made (the start answer's TLV)
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
  bool offers(uint8_t op) const override { return opIn(op, reg::fixture_capture_group::kOpBind, reg::fixture_capture_group::kOpStatus); }
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
