// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepCaptureGroup.h"

#include "OepEndpoint.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_timer.h>
#endif

namespace oep {
namespace grp = reg::fixture_capture_group;
namespace cap = reg::fixture_capture;

namespace {
uint64_t nowNs() {   // the probe's one clock (the captures stamp their segments with it)
#if defined(ARDUINO_ARCH_ESP32)
  return static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
#elif defined(ARDUINO_ARCH_RP2040)
  return time_us_64() * 1000u;
#else
  return static_cast<uint64_t>(micros()) * 1000u;
#endif
}
}  // namespace

bool CaptureGroup::addTrack(Interface &interface, GroupTrack &track, uint32_t start_skew_ns) {
  if (count_ >= kMaxTracks) return false;
  tracks_[count_++] = {&interface, &track, start_skew_ns};
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
  uint8_t fns[2 * kMaxTracks];
  for (size_t i = 0; i < count_; ++i) putU16(fns + 2 * i, endpoint_.fnOf(*tracks_[i].interface));
  w.put(grp::kTlvDescribeTracks, fns, 2 * count_);
  w.u8(grp::kTlvDescribeMaxTracks, static_cast<uint8_t>(count_));
  for (size_t k = 0; k < budget_count_; ++k) {
    uint8_t v[8];
    putU32(v, budgets_[k].max_rate);
    putU16(v + 4, endpoint_.fnOf(*tracks_[budgets_[k].a].interface));
    putU16(v + 6, endpoint_.fnOf(*tracks_[budgets_[k].b].interface));
    w.put(grp::kTlvDescribeBudget, v, sizeof v);
  }
  for (size_t i = 0; i < count_; ++i) {
    if (!tracks_[i].skew_ns) continue;
    uint8_t v[6];
    putU16(v, endpoint_.fnOf(*tracks_[i].interface));
    putU32(v + 2, tracks_[i].skew_ns);
    w.put(grp::kTlvDescribeStartSkew, v, sizeof v);
  }
  return w.ok() ? w.length() : 0;
}

void CaptureGroup::unbind() {
  for (size_t k = 0; k < bound_count_; ++k) tracks_[bound_[k]].track->setBound(false);
  bound_count_ = 0;
  running_ = started_ = false;
  start_ns_ = ~uint64_t{0};
}

uint8_t CaptureGroup::state() const {
  if (!bound_count_) return cap::kStateUnconfigured;
  bool all_done = true, any_running = false;
  for (size_t k = 0; k < bound_count_; ++k) {
    const uint8_t s = tracks_[bound_[k]].track->trackState();
    all_done &= s == cap::kStateDone;
    any_running |= s == cap::kStateCapturing || s == cap::kStateWaiting || s == cap::kStatePaused;
    if (s == cap::kStateError) return cap::kStateError;
  }
  if (started_ && all_done) return cap::kStateDone;
  return any_running ? cap::kStateCapturing : cap::kStateConfigured;
}

Result CaptureGroup::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case grp::kOpBind: {   // n(u8) n x fn(u16) [TLV 0x01 trigger_track fn(u16)]
      if (length < 1 || length < 1u + 2u * payload[0]) return rejected(kRejectMalformed);
      const uint8_t n = payload[0];
      static const uint8_t kKnown[] = {grp::kTlvBindTriggerTrack};
      const Result parsed = tail.parse(payload + 1 + 2 * n, length - 1 - 2 * n, kKnown, out, capacity);
      if (refused(parsed)) return parsed;
      uint8_t chosen[kMaxTracks];
      if (n > kMaxTracks) return rejected(kRejectUnavailable);
      for (uint8_t k = 0; k < n; ++k) {
        const int i = indexOf(getU16(payload + 1 + 2 * k));
        if (i < 0) return rejected(kRejectUnavailable);                            // not a track of this group
        for (uint8_t j = 0; j < k; ++j) if (chosen[j] == i) return rejected(kRejectUnavailable);   // twice
        const GroupTrack &t = *tracks_[i].track;
        if (!t.trackReady() || (k && t.trackMode() != tracks_[chosen[0]].track->trackMode()))
          return rejected(kRejectUnavailable);                                     // not configured, modes differ
        chosen[k] = static_cast<uint8_t>(i);
      }
      uint8_t len = 0;
      int trigger = -1;
      if (const uint8_t *v = tail.find(grp::kTlvBindTriggerTrack, len)) {
        if (len != 2) return rejected(kRejectMalformed);
        trigger = indexOf(getU16(v));
        bool in = false;
        for (uint8_t k = 0; k < n; ++k) in |= chosen[k] == trigger;
        if (!in) return rejected(kRejectUnavailable);
      }
      for (uint8_t k = 0; k < n; ++k)   // only the trigger track may have a trigger (none here has one)
        if (chosen[k] != trigger && tracks_[chosen[k]].track->trackTriggered()) return rejected(kRejectUnavailable);
      for (size_t b = 0; b < budget_count_; ++b) {
        uint64_t load = 0;
        for (uint8_t k = 0; k < n; ++k)
          if (chosen[k] == budgets_[b].a || chosen[k] == budgets_[b].b) load += tracks_[chosen[k]].track->trackLoad();
        if (load > budgets_[b].max_rate) return rejected(kRejectUnavailable);
      }
      unbind();
      for (uint8_t k = 0; k < n; ++k) {
        bound_[k] = chosen[k];
        tracks_[chosen[k]].track->setBound(true);
      }
      bound_count_ = n;
      return tail.finish(completed(), out, capacity);
    }
    case grp::kOpStart: {   // -> blocking_ms(u32) start_ns(u64)
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!bound_count_) return rejected(kRejectUnavailable);
      if (capacity < 12) return failed();
      start_ns_ = nowNs();
      for (size_t k = 0; k < bound_count_; ++k) {
        if (!tracks_[bound_[k]].track->trackStart()) {   // the rest must not run alone
          for (size_t j = 0; j < k; ++j) tracks_[bound_[j]].track->trackStop();
          return failed();
        }
      }
      running_ = started_ = true;
      putU32(out, 0);   // blocking_ms: every track keeps the probe answering
      putU64(out + 4, start_ns_);
      return tail.finish(completed(12), out, capacity);
    }
    case grp::kOpStop: {
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      for (size_t k = 0; k < bound_count_; ++k) tracks_[bound_[k]].track->trackStop();
      if (running_ && subscribed_) {
        const uint8_t reason = cap::kStoppedReasonHost;
        endpoint_.event(*this, grp::kEventStopped, &reason, 1);
      }
      running_ = false;
      return tail.finish(completed(), out, capacity);
    }
    case grp::kOpForce: {   // nothing waits for a trigger here
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      return tail.finish(completed(), out, capacity);
    }
    case grp::kOpStatus: {   // -> state(u8) start_ns(u64) trigger_ns(u64) trigger_fn(u16)
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 19) return failed();
      out[0] = state();
      putU64(out + 1, start_ns_);
      putU64(out + 9, ~uint64_t{0});   // no trigger: immediate
      putU16(out + 17, 0);
      return tail.finish(completed(19), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

void CaptureGroup::poll() {
  if (!running_ || state() != cap::kStateDone) return;
  running_ = false;
  if (subscribed_) {
    const uint8_t reason = cap::kStoppedReasonComplete;
    endpoint_.event(*this, grp::kEventStopped, &reason, 1);
  }
}

}  // namespace oep
