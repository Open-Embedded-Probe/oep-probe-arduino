// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.fixture.capture-group (oep-spec oep-if-capture §4) over fake tracks, and the endpoint's plan rules for
// a bound track (§4.1: plan_apply / plan_release of its fn refused unavailable cause 4 with the group's fn).
#include <stdio.h>

#include <algorithm>
#include <vector>

#include "OepCaptureGroup.h"
#include "OepEndpoint.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using Bytes = std::vector<uint8_t>;
using namespace oep;
namespace grp = reg::fixture_capture_group;
namespace cap = reg::fixture_logic;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

class MemStream final : public Stream {
 public:
  Bytes rx, tx;
  size_t at = 0;
  int available() override { return static_cast<int>(rx.size() - at); }
  int read() override { return at < rx.size() ? rx[at++] : -1; }
  int peek() override { return at < rx.size() ? rx[at] : -1; }
  size_t write(uint8_t c) override { tx.push_back(c); return 1; }
  size_t write(const uint8_t *b, size_t n) override { tx.insert(tx.end(), b, b + n); return n; }
  int availableForWrite() override { return 4096; }
};

static Bytes u32(uint32_t v) { return {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)}; }
// One request: role 0x01 corr fn op session_id (0 = none) payload, the 10-byte header (core §4.1). An open written
// without a session carries its id at the front of its payload here (openPayload): it goes into the header.
static Bytes request(uint16_t corr, uint16_t fn, uint8_t op, const Bytes &payload, bool session = false, uint32_t id = 0) {
  Bytes p = payload;
  if (fn == 0 && op == 0x10 && !session && p.size() >= 4) {
    id = uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
    p.erase(p.begin(), p.begin() + 4);
  } else if (!session) {
    id = 0;
  }
  Bytes m = {0x01, uint8_t(corr), uint8_t(corr >> 8), uint8_t(fn), uint8_t(fn >> 8), op};
  const Bytes s = u32(id);
  m.insert(m.end(), s.begin(), s.end());
  m.insert(m.end(), p.begin(), p.end());
  return m;
}
static Bytes openPayload(uint32_t id, uint32_t lease) {
  Bytes p = u32(id);
  const Bytes l = u32(lease);
  p.insert(p.end(), l.begin(), l.end());
  p.push_back(0);
  return p;
}
static bool contains(const Bytes &a, const Bytes &b) { return std::search(a.begin(), a.end(), b.begin(), b.end()) != a.end(); }

// A track for the group's rules: its state, mode, subscription and start are the test's; it has a plan role.
class Track final : public Interface, public GroupTrack {
 public:
  explicit Track(bool trigger = false) : trigger_(trigger) {}
  const char *name() const override { return "io.github.test.track"; }
  uint16_t instance() const override { return 0; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return rejected(kRejectUnknownOperation); }
  bool planRoles() const override { return true; }
  uint8_t planCheck(const RoleAssignment *, size_t) override { return 0; }
  bool planApply(const RoleAssignment *, size_t) override { return true; }
  uint16_t boundTo() const override { return groupFn(); }
  bool trackReady() const override { return state == 1 || state == 4; }
  bool trackCanStart() const override { return (state == 1 || state == 4 || state == 6) && (mode != 3 || subscribed); }
  uint8_t trackMode() const override { return mode; }
  bool trackTriggered() const override { return trigger_; }
  uint32_t trackLoad() const override { return load; }
  bool trackStart() override {
    ++starts;
    if (fail_start) return false;   // its state as it was (a driver that would not start)
    state = trigger_ ? 2 : 3;
    ++generation;
    return true;
  }
  uint32_t trackGeneration() const override { return generation; }
  void trackStop() override { ++stops; if (state != 6) state = 1; }
  uint8_t trackState() const override { return state; }
  bool trackCanFollow() const override { return !trigger_; }
  bool trackStartFollowing() override { ++starts; state = 2; return true; }
  uint8_t state = 1, mode = 1;
  bool subscribed = false, fail_start = false;
  uint32_t load = 0, generation = 0;
  int starts = 0, stops = 0;

 private:
  bool trigger_;
};

struct Rig {
  MemStream bulk;
  uint8_t rx[1100], tx[1100];
  Endpoint ep{bulk, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, Endpoint::kVendorBulk, 0};
  Track a, b{false};
  CaptureGroup group{ep};
  uint8_t out[128];
  Rig(bool trigger_a = false) : a(trigger_a) {
    ep.add(a);       // fn 1
    ep.add(b);       // fn 2
    ep.add(group);   // fn 3
    group.addTrack(a, a);
    group.addTrack(b, b);
  }
  Bytes send(const Bytes &m) {   // vendor bulk: length(u16) message; the answer: length(2) role corr(2) resolution detail payload
    const Bytes f = {uint8_t(m.size()), uint8_t(m.size() >> 8)};
    bulk.rx.insert(bulk.rx.end(), f.begin(), f.end());
    bulk.rx.insert(bulk.rx.end(), m.begin(), m.end());
    bulk.tx.clear();
    ep.poll();
    return bulk.tx;
  }
  Result op(uint8_t code, const Bytes &p = {}) { return group.handle(code, p.data(), p.size(), out, sizeof out); }
  Bytes payload(const Result &r) const { return Bytes(out, out + r.length); }
};

static bool rejectedAs(const Result &r, uint8_t reason) { return r.resolution == kResolutionRejected && r.detail == reason; }
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }

// A bound track's plan is the group's (§4.1): plan_apply and plan_release of its fn are refused unavailable cause 4,
// holder_fn the group, and nothing changes; unbound, they go through again.
static void testBoundPlan() {
  Rig rig;
  CHECK(rig.send(request(1, 0, 0x10, openPayload(7, 3000)))[5] == 1);
  CHECK(rig.send(request(2, 0, 0x04, {0x90, 5, 0, 1, 0, 0, 12, 0, 0x90, 5, 0, 2, 0, 0, 13, 0}, true, 7))[5] == 1);
  CHECK(ok(rig.op(grp::kOpBind, {2, 1, 0, 2, 0})));
  const Bytes cause4 = {0x01, 1, 0, 4, 0x03, 2, 0, 3, 0};   // cause 4, holder_fn 3
  Bytes r = rig.send(request(3, 0, 0x04, {0x90, 5, 0, 1, 0, 0, 14, 0}, true, 7));
  CHECK(r.size() > 7 && r[5] == kResolutionRejected && r[6] == kRejectUnavailable && contains(r, cause4));
  r = rig.send(request(4, 0, 0x05, {1, 2, 0}, true, 7));
  CHECK(r.size() > 7 && r[5] == kResolutionRejected && r[6] == kRejectUnavailable && contains(r, cause4));
  r = rig.send(request(5, 0, 0x05, {0}, true, 7));   // every fn: refused whole
  CHECK(r.size() > 7 && r[5] == kResolutionRejected && r[6] == kRejectUnavailable && contains(r, cause4));
  RoleAssignment now[4];
  CHECK(rig.ep.plan(now, 4) == 2 && now[0].channel == 12 && now[1].channel == 13);   // nothing changed
  CHECK(ok(rig.op(grp::kOpBind, {0})));
  CHECK(rig.send(request(6, 0, 0x04, {0x90, 5, 0, 1, 0, 0, 14, 0}, true, 7))[5] == 1);
  CHECK(rig.send(request(7, 0, 0x05, {0}, true, 7))[5] == 1);
  CHECK(rig.ep.plan(now, 4) == 0);
}

// start checks every track's prerequisites before it starts any (§4.1): a streaming track without a subscription is
// refused unavailable cause 6 with TLV fn (0x05) naming it, and no track has started.
static void testStartPrerequisites() {
  Rig rig;
  rig.a.mode = rig.b.mode = 3;
  rig.a.subscribed = true;   // b is not
  CHECK(ok(rig.op(grp::kOpBind, {2, 1, 0, 2, 0})));
  Result r = rig.op(grp::kOpStart);
  CHECK(rejectedAs(r, kRejectUnavailable) && rig.payload(r) == (Bytes{0x01, 1, 0, 6, 0x05, 2, 0, 2, 0}));
  CHECK(rig.a.starts == 0 && rig.b.starts == 0);
  rig.b.subscribed = true;
  CHECK(ok(rig.op(grp::kOpStart)) && rig.a.starts == 1 && rig.b.starts == 1);
  r = rig.op(grp::kOpStart);   // running: refused before anything starts again
  CHECK(rejectedAs(r, kRejectUnavailable) && rig.payload(r) == (Bytes{0x01, 1, 0, 6, 0x05, 2, 0, 1, 0}) && rig.a.starts == 1);
}

// The trigger track failing to start after the group's start (it waits for the followers' pretrigger): the group goes
// to state 6 and every track is stopped (§4.1).
static void testTriggerTrackFails() {
  Rig rig(true);   // a holds the trigger, b follows
  CHECK(ok(rig.op(grp::kOpBind, {2, 1, 0, 2, 0, 0x81, 2, 0, 1, 0})));
  rig.a.fail_start = true;
  CHECK(ok(rig.op(grp::kOpStart)) && rig.b.state == 2);
  rig.group.poll();   // b armed: the trigger track starts, and fails
  CHECK(rig.a.starts == 1 && rig.b.stops >= 1 && rig.b.state == 1);
  Result r = rig.op(grp::kOpStatus);
  CHECK(ok(r) && rig.out[0] == cap::kStateError);
  rig.a.fail_start = false;
  rig.a.state = 1;   // the track configured again: the group starts again
  CHECK(ok(rig.op(grp::kOpStart)));
  r = rig.op(grp::kOpStatus);
  CHECK(ok(r) && rig.out[0] == cap::kStateWaiting);
}

// bind over a budget (§4.1): unavailable cause 2 with TLV fn naming the track whose load goes over it.
static void testBudgetNamesTrack() {
  Rig rig;
  CHECK(rig.group.addBudget(100, rig.a, rig.b));
  rig.a.load = rig.b.load = 60;
  const Result r = rig.op(grp::kOpBind, {2, 1, 0, 2, 0});
  CHECK(rejectedAs(r, kRejectUnavailable) && rig.payload(r) == (Bytes{0x01, 1, 0, 2, 0x05, 2, 0, 2, 0}));
  rig.b.load = 40;
  CHECK(ok(rig.op(grp::kOpBind, {2, 1, 0, 2, 0})));
}

// describe start_skew (§4.3): one per track, a typical 0 included.
static void testStartSkewPerTrack() {
  Rig rig;
  uint8_t d[256];
  const size_t n = rig.group.describe(d, sizeof d);
  const Bytes all(d, d + n);
  CHECK(contains(all, {grp::kTlvDescribeStartSkew, 6, 0, 1, 0, 0, 0, 0, 0}));
  CHECK(contains(all, {grp::kTlvDescribeStartSkew, 6, 0, 2, 0, 0, 0, 0, 0}));
}

// status state (§4.1): 2 only while the trigger has not fired and a track is 2 or 3; tracks paused (5) make it 3.
static void testStatusState() {
  Rig rig(true);
  CHECK(ok(rig.op(grp::kOpBind, {2, 1, 0, 2, 0, 0x81, 2, 0, 1, 0})));
  CHECK(ok(rig.op(grp::kOpStart)));
  CHECK(ok(rig.op(grp::kOpStatus)) && rig.out[0] == cap::kStateWaiting);
  rig.a.state = rig.b.state = 5;
  CHECK(ok(rig.op(grp::kOpStatus)) && rig.out[0] == cap::kStateCapturing);
  rig.a.state = rig.b.state = 1;
  CHECK(ok(rig.op(grp::kOpStatus)) && rig.out[0] == cap::kStateConfigured);
}

int main() {
  testStartPrerequisites();
  testTriggerTrackFails();
  testBudgetNamesTrack();
  testStartSkewPerTrack();
  testStatusState();
  testBoundPlan();
  printf("capture-group: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
