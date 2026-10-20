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
  w.u32(kTagFeatures, grp::kFeaturesTriggerSource);   // bit 0: the trigger source (§4.4; force in the ops tag)
  // trigger_caps: max_stages max_terms max_conds cross_track (u8 each) types(u32) protocols(u8) trigger_out_latency_ns
  // (u32: none) - one condition on one track: a logic level or edge, an analog crossing
  uint8_t caps[13] = {1, 1, 1, 0};
  putU32(caps + 4, kCondTypes);
  caps[8] = 0;
  putU32(caps + 9, 0xFFFFFFFFu);
  w.put(grp::kTlvDescribeTriggerCaps, caps, sizeof caps);
  uint8_t fns[1 + 2 * kMaxTracks] = {static_cast<uint8_t>(count_)};   // n(u8) n x fn(u16)
  for (size_t i = 0; i < count_; ++i) putU16(fns + 1 + 2 * i, endpoint_.fnOf(*tracks_[i].interface));
  w.put(grp::kTlvDescribeTracks, fns, 1 + 2 * count_);
  // which sets bind together (the budgets, a track that cannot follow) is found by trying bind (§4.3)
  return w.ok() ? w.length() : 0;
}

void CaptureGroup::unbind() {
  if (trigger_ >= 0) tracks_[trigger_].track->trackUnjudge();
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
    case grp::kOpBind: {   // n(u8) n x fn(u16) [TLV] (§4.1; with a trigger source §4.4)
      if (length < 1 || length < 1u + 2u * payload[0]) return rejected(kRejectMalformed);
      const uint8_t n = payload[0];
      static const uint8_t kKnown[] = {grp::kTlvBindTriggerSource, grp::kTlvBindStage, grp::kTlvBindCond};
      const Result parsed = tail.parse(payload + 1 + 2 * n, length - 1 - 2 * n, kKnown, out, capacity);
      if (refused(parsed)) return parsed;   // trigger_out among them: unsupported, the tag as received
      Source src;
      const Result read = readSource(tail, n, payload + 1, src, out, capacity);
      if (refused(read)) return read;
      uint8_t chosen[kMaxTracks];
      for (uint8_t k = 0; k < n; ++k)   // the same fn twice: malformed (core §4.3)
        for (uint8_t j = 0; j < k; ++j)
          if (getU16(payload + 1 + 2 * k) == getU16(payload + 1 + 2 * j)) return rejected(kRejectMalformed);
      if (n == 0) {   // unbind: not while capturing (state 3); nothing bound: a no-op
        if (src.present) return rejected(kRejectMalformed);   // a trigger source with no track
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
      // the condition's track (one of the bound) and its type and values on it: unsupported, the cond's tag
      int judge = -1;
      if (src.present) {
        for (uint8_t k = 0; k < n; ++k) if (getU16(payload + 1 + 2 * k) == src.fn) judge = chosen[k];
        if (judge < 0 ||
            tracks_[judge].track->trackCanJudge(src.type, src.role, src.args) == GroupTrack::kJudgeUnsupported)
          return unsupportedTag(out, capacity, src.cond_raw);
      }
      bool same = n == bound_count_ && !src.present && trigger_ < 0;
      for (uint8_t k = 0; same && k < n; ++k) same = bound_[k] == chosen[k];
      if (same) return completed();   // the same list again (immediate): kept as it is
      if (state() == cap::kStateCapturing || state() == cap::kStateWaiting) return wrongState(out, capacity);
      auto refuseAt = [&](uint8_t cause, uint8_t k) { return refuseTrack(cause, getU16(payload + 1 + 2 * k), out, capacity); };
      for (uint8_t k = 0; k < n; ++k) {
        const GroupTrack &t = *tracks_[chosen[k]].track;
        // state 1 / 4 / 6, one mode for all; immediate - or, with a trigger source, linked (§4.4)
        const bool trigger_ok = src.present ? t.trackLinked() : !t.trackTriggered();
        if (!t.trackReady() || !trigger_ok || (k && t.trackMode() != tracks_[chosen[0]].track->trackMode()))
          return refuseAt(reg::core::kUnavailableCauseWrongState, k);
      }
      if (src.present && tracks_[judge].track->trackCanJudge(src.type, src.role, src.args) == GroupTrack::kJudgeNoRole)
        return refuseTrack(reg::core::kUnavailableCauseWrongState, src.fn, out, capacity);   // the role not in its plan
      // short of the resources to capture together (a budget the tracks share): cause 2, the track whose load goes over
      for (size_t b = 0; b < budget_count_; ++b) {
        uint64_t load = 0;
        for (uint8_t k = 0; k < n; ++k) {
          if (chosen[k] != budgets_[b].a && chosen[k] != budgets_[b].b) continue;
          load += tracks_[chosen[k]].track->trackLoad();
          if (load > budgets_[b].max_rate) return refuseAt(reg::core::kUnavailableCauseLimit, k);
        }
      }
      // every other track runs into its ring and cuts at the event: its own pretrigger and the time the judging track
      // takes to tell the event (its search, loop()'s round) - a ring too short for that: cause 2
      const uint64_t late_ns = src.present ? tracks_[judge].track->trackLatencyNs() : 0;
      for (uint8_t k = 0; src.present && k < n; ++k) {
        const GroupTrack &t = *tracks_[chosen[k]].track;
        if (static_cast<int>(chosen[k]) != judge && !t.trackCanKeep(t.trackPretrigger(), late_ns))
          return refuseAt(reg::core::kUnavailableCauseLimit, k);
      }
      const uint64_t start_ns = start_ns_;
      unbind();
      start_ns_ = start_ns;   // bind changes neither the group's generation nor its last start
      const uint16_t me = endpoint_.fnOf(*this);
      for (uint8_t k = 0; k < n; ++k) {
        GroupTrack &t = *tracks_[chosen[k]].track;
        bound_[k] = chosen[k];
        t.setBound(true, me);
        t.following_ = src.present && static_cast<int>(chosen[k]) != judge;
        if (t.following_) t.trackKeep(t.trackPretrigger(), late_ns);
      }
      bound_count_ = n;
      if (src.present) {
        trigger_ = judge;
        arm_ = src.arm;
        tracks_[judge].track->trackJudge(src.type, src.role, src.args, src.arm);
      }
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
      running_ = started_ = true;
      trigger_pending_ = trigger_ >= 0;
      // arm 1: the judging track starts once every other holds its pretrigger (poll); arm 0: now (§4.4)
      if (trigger_pending_ && !arm_ && !startTrigger()) return failed();
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
      if (trigger_ns_ == ~uint64_t{0}) return completed(13);
      TlvWriter w(out + 13, capacity - 13);   // trigger: fn(u16: 0 forced) ns(u64) uncertainty_ns(u32) (§4.4)
      uint8_t t[14];
      putU16(t, triggerFn());
      putU64(t + 2, trigger_ns_);
      putU32(t + 10, triggerUncertainty());
      w.put(grp::kTlvStatusAnswerTrigger, t, sizeof t);
      return w.ok() ? completed(13 + w.length()) : failed();
    }
    case grp::kOpForce: {   // waiting for the event: it happens now; otherwise nothing (§4.4)
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!running_ || trigger_ < 0 || trigger_ns_ != ~uint64_t{0}) return completed();
      if (trigger_pending_ && !startTrigger()) return completed();   // the judging track not started yet: now
      forced_ = true;
      tracks_[trigger_].track->trackForce();
      return completed();
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

uint16_t CaptureGroup::triggerFn() const { return forced_ || trigger_ < 0 ? 0 : endpoint_.fnOf(*tracks_[trigger_].interface); }

// The judging track's sample period (§4.4: the track the deciding term looks at)
uint32_t CaptureGroup::triggerUncertainty() const {
  uint32_t num = 0, den = 0;
  return trigger_ >= 0 && tracks_[trigger_].track->trackRate(num, den) ? periodNs(num, den) : 0;
}

// bind's trigger source TLVs (§4.4), the form first (malformed: a length not the definition's, a stage or cond with
// no source or past its stages, a cond with no stage, arm / rearm / on_timeout past their values, on_timeout 1 with
// within 0), then what this probe takes (unsupported, the TLV as received): one stage counted once with no time
// limit, one condition (term 0, no abort) on a bound track, rearm 0.
Result CaptureGroup::readSource(const Tail &tail, uint8_t n, const uint8_t *fns, Source &src, uint8_t *out,
                                size_t capacity) const {
  size_t at = 0, len = 0;
  uint8_t raw = 0;
  const uint8_t *v = nullptr;
  uint8_t source_raw = 0, stages = 0, stage_raw = 0, cond_count = 0;
  bool stage_bad = false, source_bad = false, cond_bad = false;
  uint8_t bad_raw = 0;
  while (tail.next(at, raw, v, len)) {   // the form
    const uint8_t tag = raw & ~kTagCritical;
    if (tag == grp::kTlvBindTriggerSource) {
      if (src.present || len != 11 || v[0] == 0 || v[9] > 1 || v[10] > 1) return rejected(kRejectMalformed);
      src.present = true;
      source_raw = raw;
      stages = v[0];
      src.arm = v[9];
      source_bad = stages != 1 || v[10] != 0;   // rearm 1: one-shot only here
    }
  }
  at = 0;
  while (tail.next(at, raw, v, len)) {
    const uint8_t tag = raw & ~kTagCritical;
    if (tag == grp::kTlvBindStage) {   // stage(u8) count(u32) within_ns(u64) on_timeout(u8)
      if (!src.present || len != 14 || v[0] >= stages || getU32(v + 1) == 0 || v[13] > 1 ||
          (v[13] == 1 && getU64(v + 5) == 0))
        return rejected(kRejectMalformed);
      if (!stage_bad && (getU32(v + 1) != 1 || getU64(v + 5) != 0)) { stage_bad = true; stage_raw = raw; }
    } else if (tag == grp::kTlvBindCond) {   // stage part term src fn(u16) role type args
      if (!src.present || len < 8 || v[0] >= stages || v[1] > 1) return rejected(kRejectMalformed);
      const uint8_t type = v[7];
      size_t want = 0;
      switch (type) {
        case grp::kTriggerCondLevel: want = 2; break;
        case grp::kTriggerCondEdge: want = 1; break;
        case grp::kTriggerCondCrossUp: case grp::kTriggerCondCrossDown: want = 8; break;
        case grp::kTriggerCondAbove: case grp::kTriggerCondBelow: want = 9; break;
        case grp::kTriggerCondPulse: want = 19; break;
        case grp::kTriggerCondAnalogPulse: want = 26; break;
        case grp::kTriggerCondTimer: want = 8; break;
        default: want = len - 8; break;   // pattern, serial and later ones: variable, not taken here (below)
      }
      if (len != 8 + want) return rejected(kRejectMalformed);
      if (++cond_count == 1) {
        src.cond_raw = raw;
        src.fn = getU16(v + 4);
        src.role = v[6];
        src.type = type;
        memcpy(src.args, v + 8, want < sizeof src.args ? want : sizeof src.args);
        bool bound_fn = false;
        for (uint8_t k = 0; k < n; ++k) bound_fn |= getU16(fns + 2 * k) == src.fn;
        cond_bad = v[1] != 0 || v[2] != 0 || v[3] != grp::kCondSrcTrack || !bound_fn || type >= 32 ||
                   !((kCondTypes >> type) & 1);
      } else if (!bad_raw) {
        bad_raw = raw;   // a second condition: one is the most (max_conds / max_terms 1)
      }
    }
  }
  if (src.present && cond_count == 0) return rejected(kRejectMalformed);   // an expression with no condition
  if (!src.present) return completed();
  if (source_bad) return unsupportedTag(out, capacity, source_raw);
  if (stage_bad) return unsupportedTag(out, capacity, stage_raw);
  if (cond_bad) return unsupportedTag(out, capacity, src.cond_raw);
  if (bad_raw) return unsupportedTag(out, capacity, bad_raw);
  return completed();
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
    if (subscribed_) {   // trigger_fn(u16: 0 forced) record(u32: none, rearm 0) trigger_ns(u64) uncertainty_ns(u32)
      uint8_t e[22];     // generation(u32: the group's) (§4.4)
      putU16(e, triggerFn());
      putU32(e + 2, 0xFFFFFFFFu);
      putU64(e + 6, ns);
      putU32(e + 14, triggerUncertainty());
      putU32(e + 18, generation_);
      endpoint_.event(*this, grp::kEventTriggered, e, sizeof e);
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
