// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// oep.fixture.capture-group revision 1 (oep-spec docs/oep-if-capture.ja.md §4): capture / analog tracks started
// together. Each track keeps its own configure and is read as usual; the group binds them, starts and stops them at once
// and says when it started. Every track stamps its own first sample on the probe's one clock (segment start_ns), so a
// track's offset is its first segment's start_ns minus the group's.
//
//   0x01 bind(n u8, n x fn u16, [TLV 0x01 trigger_track fn]) -> -   0x02 start -> blocking_ms u32, start_ns u64
//   0x03 stop   0x04 force   0x05 status -> state u8, start_ns u64, trigger_ns u64, trigger_fn u16 (no lock)
//
// With trigger_track, that track waits for its own trigger and the others follow: they run into their rings from the
// start (the trigger track starts once each holds its pretrigger), and when the trigger's time is known (trackTriggerNs) the group hands it to them (trackTriggerAt), and each
// cuts its segment around the sample nearest to it, with its own pretrigger. A track that cannot follow (no ring)
// is refused at bind.
#pragma once
#include <Arduino.h>
#include "Oep.h"

namespace oep {

class Endpoint;

// What a capture interface gives a group: whether it can be bound, and a start / stop that the group drives. While
// bound, the interface refuses its own configure / start / stop / force (rejected unavailable).
class GroupTrack {
 public:
  virtual ~GroupTrack() = default;
  virtual bool trackReady() const = 0;        // configured and not running
  virtual uint8_t trackMode() const = 0;      // the configured mode (one-shot, repeat, streaming)
  virtual bool trackTriggered() const = 0;    // a trigger other than immediate is configured
  virtual uint32_t trackLoad() const = 0;     // channels x rate, sample/s (for budgets)
  virtual bool trackStart() = 0;              // start now; false: it could not
  virtual void trackStop() = 0;
  virtual uint8_t trackState() const = 0;     // the capture state (oep-if-capture §3.2)
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
  void setBound(bool on) { bound_ = on; if (!on) following_ = false; }
  bool bound() const { return bound_; }

 protected:
  bool bound_ = false;
  bool following_ = false;   // bound as a follower: start with trackStartFollowing
  friend class CaptureGroup;
};

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
  // A track the group may bind (add both to the endpoint first). start_skew_ns: how late it typically starts after
  // the group (describe, for display).
  bool addTrack(Interface &interface, GroupTrack &track, uint32_t start_skew_ns = 0);
  // A budget the listed tracks share when bound together: channels x rate summed, sample/s.
  bool addBudget(uint32_t max_rate, GroupTrack &a, GroupTrack &b);
  const char *name() const override { return reg::fixture_capture_group::kName; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return reg::fixture_capture_group::kRevision; }
  size_t describe(uint8_t *out, size_t capacity) override;
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::fixture_capture_group::kLockFreeOps, op); }
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  bool subscribe(bool on) override { subscribed_ = on; return true; }
  void poll();   // from loop(): the stopped event once every bound track is done

 private:
  struct Entry { Interface *interface; GroupTrack *track; uint32_t skew_ns; };
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
  int indexOf(uint16_t fn) const;
  int indexOf(const GroupTrack &track) const;
  uint8_t state() const;
  void unbind();
};

}  // namespace oep
