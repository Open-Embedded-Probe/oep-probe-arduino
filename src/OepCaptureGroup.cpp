// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepCaptureGroup.h"

#include "OepEndpoint.h"

namespace oep {
namespace grp = reg::fixture_capture_group;
namespace cap = reg::fixture_logic;

bool CaptureGroup::addTrack(Interface &interface, GroupTrack &track) {
  if (count_ >= kMaxTracks) return false;
  tracks_[count_++] = {&interface, &track};
  return true;
}

bool CaptureGroup::addBudget(uint32_t max_rate, GroupTrack &a, GroupTrack &b) {
  const int ia = indexOf(a), ib = indexOf(b);
  if (budget_count_ >= kMaxTracks || ia < 0 || ib < 0) return false;
  budgets_[budget_count_++] = {max_rate, static_cast<uint8_t>(ia), static_cast<uint8_t>(ib)};
  return true;
}

int CaptureGroup::indexOf(uint16_t fn) const {
  for (size_t i = 0; i < count_; ++i) if (endpoint_.fnOf(*tracks_[i].interface) == fn) return static_cast<int>(i);
  return -1;
}

int CaptureGroup::indexOf(const GroupTrack &track) const {
  for (size_t i = 0; i < count_; ++i) if (tracks_[i].track == &track) return static_cast<int>(i);
  return -1;
}

size_t CaptureGroup::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  // no features: revision 1 defines no bit (force, subscribe and unsubscribe are in the ops tag)
  uint8_t fns[1 + 2 * kMaxTracks] = {static_cast<uint8_t>(count_)};   // n(u8) n x fn(u16)
  for (size_t i = 0; i < count_; ++i) putU16(fns + 1 + 2 * i, endpoint_.fnOf(*tracks_[i].interface));
  w.put(grp::kTlvDescribeTracks, fns, 1 + 2 * count_);
  // which sets bind together (the budgets, a track that cannot follow) is found by trying bind (§4.3)
  return w.ok() ? w.length() : 0;
}

void CaptureGroup::unbind() {
  for (size_t k = 0; k < bound_count_; ++k) tracks_[bound_[k]].track->setBound(false);
  bound_count_ = 0;
  running_ = started_ = false;
  start_ns_ = trigger_ns_ = ~uint64_t{0};
  trigger_ = -1;
  trigger_pending_ = forced_ = failed_ = false;
}

// unavailable `cause` about one track: TLV fn (0x05) names it (§4.1, core §4.3)
Result CaptureGroup::refuseTrack(uint8_t cause, uint16_t fn, uint8_t *out, size_t capacity) const {
  uint8_t extra[5] = {reg::core::kTlvUnavailablePayloadFn, 2, 0, 0, 0};   // tag len(u16) fn(u16)
  putU16(extra + 3, fn);
  return unavailable(out, capacity, cause, 0xFFFF, extra, sizeof extra);
}

uint8_t CaptureGroup::state() const {
  if (!bound_count_) return cap::kStateUnconfigured;
  if (failed_) return cap::kStateError;
  // §4.1: 6 if a track is 6; else 4 if started and every track is 4; else 2 if the trigger track has not fired and a
  // track is 2 or 3; else 3 if a track is 2, 3 or 5; else 1
  bool all_done = true, any_active = false, any_running = false;
  for (size_t k = 0; k < bound_count_; ++k) {
    const uint8_t s = tracks_[bound_[k]].track->trackState();
    all_done &= s == cap::kStateDone;
    any_active |= s == cap::kStateCapturing || s == cap::kStateWaiting;
    any_running |= s == cap::kStateCapturing || s == cap::kStateWaiting || s == cap::kStatePaused;
    if (s == cap::kStateError) return cap::kStateError;
  }
  if (started_ && all_done) return cap::kStateDone;
  if (running_ && trigger_ >= 0 && trigger_ns_ == ~uint64_t{0} && any_active) return cap::kStateWaiting;
  return any_running ? cap::kStateCapturing : cap::kStateConfigured;
}

// Every bound track stopped by the group (the host's stop, or a track that failed after the start), and the stopped
// event with its reason (oep-if-capture §4.1, §4.2).
void CaptureGroup::stopAll(uint8_t reason, uint8_t error) {
  trigger_pending_ = false;
  for (size_t k = 0; k < bound_count_; ++k) tracks_[bound_[k]].track->trackStop();
  if (running_ && subscribed_) {   // reason(u8) error(u8) generation(u32: the group's)
    uint8_t e[6] = {reason, error};
    putU32(e + 2, generation_);
    endpoint_.event(*this, grp::kEventStopped, e, sizeof e);
  }
  running_ = false;
}

Result CaptureGroup::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case grp::kOpBind: {   // n(u8) n x fn(u16) [TLV] (capture §4.1: immediate tracks only)
      if (length < 1 || length < 1u + 2u * payload[0]) return rejected(kRejectMalformed);
      const uint8_t n = payload[0];
      const Result parsed = plainTail(tail, payload, length, 1u + 2u * n, out, capacity);
      if (refused(parsed)) return parsed;
      uint8_t chosen[kMaxTracks];
      for (uint8_t k = 0; k < n; ++k)   // the same fn twice: malformed (core §4.3)
        for (uint8_t j = 0; j < k; ++j)
          if (getU16(payload + 1 + 2 * k) == getU16(payload + 1 + 2 * j)) return rejected(kRejectMalformed);
      if (n == 0) {   // unbind: not while capturing (state 3); nothing bound: a no-op
        if (state() == cap::kStateCapturing) return wrongState(out, capacity);
        unbind();
        return completed();
      }
      for (uint8_t k = 0; k < n; ++k) {
        const int i = indexOf(getU16(payload + 1 + 2 * k));
        if (i < 0) {   // not a track of this group (describe tracks): unsupported, the fn in TLV 0x05
          if (capacity < 6) return unsupportedValue(out, capacity);
          out[0] = kTagValue;
          putTlvHeader(out + 1, reg::core::kTlvUnsupportedPayloadFn, 2);
          putU16(out + 4, getU16(payload + 1 + 2 * k));
          return {kResolutionRejected, kRejectUnsupported, 6};
        }
        chosen[k] = static_cast<uint8_t>(i);
      }
      if (n > kMaxTracks) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
      // the start's answer (5 + 13 + 6 n) must fit this route's frame (§4.1)
      if (5u + 13u + 6u * n > endpoint_.maxFrame()) return unsupportedValue(out, capacity);
      bool same = n == bound_count_;
      for (uint8_t k = 0; same && k < n; ++k) same = bound_[k] == chosen[k];
      if (same) return completed();   // the same list again: kept as it is
      if (state() == cap::kStateCapturing) return wrongState(out, capacity);   // another list while capturing
      auto refuseAt = [&](uint8_t cause, uint8_t k) { return refuseTrack(cause, getU16(payload + 1 + 2 * k), out, capacity); };
      for (uint8_t k = 0; k < n; ++k) {
        const GroupTrack &t = *tracks_[chosen[k]].track;
        // state 1 / 4 / 6, immediate (no trigger), one mode for all
        if (!t.trackReady() || t.trackTriggered() || (k && t.trackMode() != tracks_[chosen[0]].track->trackMode()))
          return refuseAt(reg::core::kUnavailableCauseWrongState, k);
      }
      // short of the resources to capture together (a budget the tracks share): cause 2, the track whose load goes over
      for (size_t b = 0; b < budget_count_; ++b) {
        uint64_t load = 0;
        for (uint8_t k = 0; k < n; ++k) {
          if (chosen[k] != budgets_[b].a && chosen[k] != budgets_[b].b) continue;
          load += tracks_[chosen[k]].track->trackLoad();
          if (load > budgets_[b].max_rate) return refuseAt(reg::core::kUnavailableCauseLimit, k);
        }
      }
      const uint64_t start_ns = start_ns_;
      unbind();
      start_ns_ = start_ns;   // bind changes neither the group's generation nor its last start
      const uint16_t me = endpoint_.fnOf(*this);
      for (uint8_t k = 0; k < n; ++k) {
        bound_[k] = chosen[k];
        tracks_[chosen[k]].track->setBound(true, me);
        tracks_[chosen[k]].track->following_ = false;
      }
      bound_count_ = n;
      return completed();
    }
    case grp::kOpStart: {   // -> start_ns(u64) generation(u32) n(u8) n x (fn(u16) generation(u32)) [TLV]
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!bound_count_) return wrongState(out, capacity);
      // every track's prerequisites before any starts (§4.1): its state, a streaming track's subscription
      for (size_t k = 0; k < bound_count_; ++k) {
        const GroupTrack &t = *tracks_[bound_[k]].track;
        if (!(t.following_ ? t.trackCanFollow() : t.trackCanStart()))
          return refuseTrack(reg::core::kUnavailableCauseWrongState, endpoint_.fnOf(*tracks_[bound_[k]].interface), out, capacity);
      }
      if (capacity < 13 + 6 * bound_count_) return failed();
      failed_ = false;
      generation_ = nextGeneration(generation_);   // the group's: one up at every start, never 0 (§4.1)
      start_ns_ = nowNs();
      trigger_ns_ = ~uint64_t{0};
      forced_ = false;
      // The followers first: their rings run before the trigger can come. The trigger track starts once each of them
      // holds its pretrigger (poll): a trigger right at the start found the analog with no values yet (0.0.15, the P4).
      for (size_t k = 0; k < bound_count_; ++k) {
        GroupTrack &t = *tracks_[bound_[k]].track;
        if (static_cast<int>(bound_[k]) == trigger_) continue;
        if (!(t.following_ ? t.trackStartFollowing() : t.trackStart())) {   // the rest must not run alone
          for (size_t j = 0; j < bound_count_; ++j) tracks_[bound_[j]].track->trackStop();
          failed_ = true;   // state 6 (§4.1)
          return failed();
        }
      }
      trigger_pending_ = trigger_ >= 0;
      running_ = started_ = true;
      putU64(out, start_ns_);
      putU32(out + 8, generation_);
      out[12] = static_cast<uint8_t>(bound_count_);
      for (size_t k = 0; k < bound_count_; ++k) {   // in bind order
        putU16(out + 13 + 6 * k, endpoint_.fnOf(*tracks_[bound_[k]].interface));
        putU32(out + 15 + 6 * k, tracks_[bound_[k]].track->trackGeneration());
      }
      return completed(13 + 6 * bound_count_);
    }
    case grp::kOpStop: {
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      stopAll(cap::kStoppedReasonHost, 0);
      return completed();
    }
    case grp::kOpStatus: {   // -> state(u8) start_ns(u64) generation(u32) [TLV]
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 13) return failed();
      poll();
      out[0] = state();
      putU64(out + 1, start_ns_);       // all ones before the first start
      putU32(out + 9, generation_);     // the group's (0 before its first start)
      return completed(13);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

// The trigger track starts once the followers hold their pretrigger. Failing after the group's start: the group goes to
// state 6, every track stops, stopped reason 3 (§4.1).
bool CaptureGroup::startTrigger() {
  trigger_pending_ = false;
  if (tracks_[trigger_].track->trackStart()) return true;
  failed_ = true;
  stopAll(cap::kStoppedReasonError, cap::kErrorPeripheral);
  return false;
}

void CaptureGroup::poll() {
  if (running_ && trigger_pending_) {
    bool armed = true;
    for (size_t k = 0; k < bound_count_; ++k)
      if (static_cast<int>(bound_[k]) != trigger_) armed &= tracks_[bound_[k]].track->trackArmed();
    if (armed) startTrigger();
  }
  uint64_t ns = 0;
  // not before the trigger track has started: until then it still holds the last run's trigger (the second run in one
  // boot took that time and the followers cut at once, 0.0.17)
  if (running_ && trigger_ >= 0 && !trigger_pending_ && trigger_ns_ == ~uint64_t{0} &&
      tracks_[trigger_].track->trackTriggerNs(ns)) {
    trigger_ns_ = ns;
    for (size_t k = 0; k < bound_count_; ++k)
      if (tracks_[bound_[k]].track->following_) tracks_[bound_[k]].track->trackTriggerAt(ns);
    if (subscribed_) {   // trigger_fn(u16: 0 when forced) trigger_ns(u64) generation(u32: the group's)
      uint8_t e[14];
      putU16(e, forced_ ? 0 : endpoint_.fnOf(*tracks_[trigger_].interface));
      putU64(e + 2, ns);
      putU32(e + 10, generation_);
      endpoint_.event(*this, legacy::kEventGroupTriggered, e, sizeof e);
    }
  }
  if (!running_) return;
  for (size_t k = 0; k < bound_count_; ++k)   // a track failed after the start: the group fails, the rest stop
    if (tracks_[bound_[k]].track->trackState() == cap::kStateError) {
      failed_ = true;
      stopAll(cap::kStoppedReasonError, tracks_[bound_[k]].track->trackError());   // the track's own reason
      return;
    }
  if (state() != cap::kStateDone) return;
  running_ = false;
  if (subscribed_) {
    uint8_t e[6] = {cap::kStoppedReasonComplete, 0};
    putU32(e + 2, generation_);
    endpoint_.event(*this, grp::kEventStopped, e, sizeof e);
  }
}

}  // namespace oep
