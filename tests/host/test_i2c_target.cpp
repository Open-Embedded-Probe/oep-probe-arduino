// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.fixture.i2c-target (oep-spec oep-if-fixture §3) with a fake controller (OEP_HOST_FAKE_I2C_SLAVE:
// hostTransaction runs a transaction through the same judging code the interrupt handler calls at STOP) - the plan's
// roles, configure / reset / plan release, mode 1's wait and write lengths, mode 2's one-transaction frame, mode 3's
// slots, an address-only write, queue overflow, stretch's limit, read_rx in state 0.
#include <stdio.h>

#include <vector>

#include "OepP4I2cTarget.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

struct Status {
  uint8_t state, mode, armed, queued;
  uint32_t rx_frames;
  uint8_t tx_slots;
  uint32_t errors;
};

static Result call(P4I2cTarget &t, uint8_t op, const Bytes &payload, Bytes &out) {
  out.assign(512, 0);
  const Result r = t.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static bool rejectedAs(const Result &r, uint8_t reason) { return r.resolution == kResolutionRejected && r.detail == reason; }
static bool unavailableCause(const Result &r, const Bytes &out, uint8_t cause) {
  return rejectedAs(r, kRejectUnavailable) && out.size() >= 3 && out[0] == 0x01 && out[1] == 1 && out[2] == cause;
}
static Status status(P4I2cTarget &t) {
  Bytes out;
  const Result r = call(t, P4I2cTarget::kOpStatus, {}, out);
  Status s = {};
  if (!ok(r) || out.size() < 13) { CHECK(false); return s; }
  s = {out[0], out[1], out[2], out[3], getU32(out.data() + 4), out[8], getU32(out.data() + 9)};
  return s;
}
static Result configure(P4I2cTarget &t, uint8_t address, uint8_t mode) {
  Bytes out;
  return call(t, P4I2cTarget::kOpConfigure, {address, mode}, out);
}
static Result armRx(P4I2cTarget &t, uint16_t length) {
  Bytes p(2), out;
  putU16(p.data(), length);
  return call(t, P4I2cTarget::kOpArmRx, p, out);
}
static Result preload(P4I2cTarget &t, const Bytes &data, Bytes &out) {
  Bytes p(2);
  putU16(p.data(), static_cast<uint16_t>(data.size()));
  p.insert(p.end(), data.begin(), data.end());
  return call(t, P4I2cTarget::kOpPreloadTx, p, out);
}
static Result stretch(P4I2cTarget &t, uint32_t us) {
  Bytes p(4), out;
  putU32(p.data(), us);
  return call(t, P4I2cTarget::kOpStretch, p, out);
}
static Bytes readRx(P4I2cTarget &t, uint8_t *pending = nullptr) {
  Bytes out;
  const Result r = call(t, P4I2cTarget::kOpReadRx, {}, out);
  if (!ok(r) || out.size() < 3) { CHECK(false); return {}; }
  if (pending) *pending = out[0];
  CHECK(getU16(out.data() + 1) == out.size() - 3);
  return Bytes(out.begin() + 3, out.end());
}
static void write(P4I2cTarget &t, const Bytes &data) { CHECK(t.hostTransaction(data.data(), data.size())); }
static Bytes read(P4I2cTarget &t, size_t n, const Bytes &first = {}) {
  Bytes got(n);
  CHECK(t.hostTransaction(first.data(), first.size(), n, got.data()));
  return got;
}

int main() {
  static PinTable pins((1ull << 4) | (1ull << 5));
  static P4I2cTarget t(pins);
  Bytes out;

  // the plan: each role once, on distinct channels
  {
    const RoleAssignment twice[] = {{0, P4I2cTarget::kRoleSda, 4}, {0, P4I2cTarget::kRoleSda, 5}};
    const RoleAssignment same[] = {{0, P4I2cTarget::kRoleSda, 4}, {0, P4I2cTarget::kRoleScl, 4}};
    const RoleAssignment other[] = {{0, P4I2cTarget::kRoleSda, 4}, {0, 3, 5}};
    CHECK(t.planCheck(twice, 2) == kRejectMalformed);
    CHECK(t.planCheck(same, 2) == kRejectMalformed);
    CHECK(t.planCheck(other, 2) == kRejectMalformed);
    CHECK(t.planCheck(twice, 1) == kRejectMalformed);
  }
  // no plan: configure is unavailable cause 6; stretch is taken in any state
  CHECK(unavailableCause(call(t, P4I2cTarget::kOpConfigure, {0x42, 1}, out), out, 6));
  CHECK(ok(stretch(t, 50)));
  CHECK(t.hostStretchUs() == 50);
  const RoleAssignment roles[] = {{0, P4I2cTarget::kRoleSda, 4}, {0, P4I2cTarget::kRoleScl, 5}};
  CHECK(t.planCheck(roles, 2) == 0);
  CHECK(t.planApply(roles, 2));

  // describe: max_stretch_us (0x41) with features bit1
  {
    uint8_t d[128];
    const size_t n = t.describe(d, sizeof d);
    bool found = false;
    for (size_t i = 0; i + 1 < n; i += 2 + d[i + 1])
      if (d[i] == reg::fixture_i2c_target::kTlvDescribeMaxStretchUs && d[i + 1] == 4)
        found = getU32(d + i + 2) == P4I2cTarget::kMaxStretchUs;
    CHECK(found && P4I2cTarget::kMaxStretchUs >= 30000);
    CHECK(P4I2cTarget::kQueueDepth >= 2);
  }

  // state 0: read_rx and reset are unavailable cause 6
  CHECK(unavailableCause(call(t, P4I2cTarget::kOpReadRx, {}, out), out, 6));
  CHECK(unavailableCause(call(t, P4I2cTarget::kOpReset, {}, out), out, 6));
  CHECK(rejectedAs(configure(t, 0x80, 1), kRejectMalformed));
  CHECK(rejectedAs(configure(t, 0x42, 0), kRejectMalformed));
  CHECK(rejectedAs(configure(t, 0x42, 4), kRejectMalformed));

  // ---- mode 1
  CHECK(ok(configure(t, 0x42, P4I2cTarget::kModeFixedRx)));
  Status s = status(t);
  CHECK(s.state == 1 && s.mode == 1 && !s.armed && s.queued == 0 && s.rx_frames == 0 && s.tx_slots == 0 && s.errors == 0);
  write(t, {1, 2, 3});   // not armed: dropped, an error
  s = status(t);
  CHECK(s.queued == 0 && s.rx_frames == 0 && s.errors == 1);
  write(t, {});   // the address alone: nothing
  CHECK(status(t).errors == 1);
  CHECK(rejectedAs(armRx(t, 0), kRejectMalformed));
  CHECK(rejectedAs(armRx(t, 129), kRejectUnsupported));
  CHECK(ok(armRx(t, 3)));
  CHECK(status(t).armed == 1);
  write(t, {0x11, 0x22, 0x33});
  s = status(t);
  CHECK(s.armed && s.queued == 1 && s.rx_frames == 1 && s.errors == 1);   // the wait goes on after a frame
  write(t, {0x44, 0x55});   // another length: dropped, an error, still armed
  write(t, {0x44, 0x55, 0x66, 0x77});
  s = status(t);
  CHECK(s.armed && s.queued == 1 && s.rx_frames == 1 && s.errors == 3);
  write(t, {});   // address only, armed: nothing
  CHECK(status(t).errors == 3);
  write(t, {0xa1, 0xa2, 0xa3});
  uint8_t pending = 0xEE;
  CHECK(readRx(t, &pending) == Bytes({0x11, 0x22, 0x33}) && pending == 1);
  CHECK(readRx(t, &pending) == Bytes({0xa1, 0xa2, 0xa3}) && pending == 0);
  CHECK(readRx(t, &pending).empty() && pending == 0);
  // a new arm_rx replaces the wait
  CHECK(ok(armRx(t, 2)));
  write(t, {0x01, 0x02, 0x03});
  write(t, {0x05, 0x06});
  s = status(t);
  CHECK(s.queued == 1 && s.rx_frames == 3 && s.errors == 4);
  // overflow: the frame past queue_depth is dropped, an error, not in rx_frames
  for (int i = 0; i < 4; ++i) write(t, {static_cast<uint8_t>(i), 0});
  s = status(t);
  CHECK(s.queued == 4 && s.rx_frames == 6 && s.errors == 5);
  CHECK(readRx(t) == Bytes({0x05, 0x06}));
  // a read in mode 1 sends 0xFF and counts nothing
  CHECK(read(t, 3) == Bytes({0xFF, 0xFF, 0xFF}));
  s = status(t);
  CHECK(s.queued == 3 && s.errors == 5);
  // reset: as right after configure (mode and address stay, the wait goes)
  CHECK(ok(call(t, P4I2cTarget::kOpReset, {}, out)));
  s = status(t);
  CHECK(s.state == 1 && s.mode == 1 && !s.armed && s.queued == 0 && s.rx_frames == 0 && s.errors == 0);
  // configure clears frames, the wait and the counts, and keeps stretch
  CHECK(ok(armRx(t, 1)));
  write(t, {9});
  write(t, {9, 9});
  CHECK(ok(stretch(t, 30000)));
  CHECK(ok(configure(t, 0x42, P4I2cTarget::kModeFixedRx)));
  s = status(t);
  CHECK(s.state == 1 && !s.armed && s.queued == 0 && s.rx_frames == 0 && s.errors == 0);
  CHECK(t.hostStretchUs() == 30000);

  // ---- mode 2: the length byte and the body in one write
  CHECK(ok(configure(t, 0x42, P4I2cTarget::kModeFramedRx)));
  CHECK(rejectedAs(armRx(t, 4), kRejectUnavailable));
  write(t, {3, 0xb1, 0xb2, 0xb3});
  s = status(t);
  CHECK(!s.armed && s.queued == 1 && s.rx_frames == 1 && s.errors == 0);
  write(t, {3, 0xb1, 0xb2});   // body short
  write(t, {2, 0xb1, 0xb2, 0xb3});   // body long
  write(t, {0});   // L = 0
  {
    Bytes big(1 + 129, 0x5a);
    big[0] = 129;   // over max_length (the byte can say up to 255)
    write(t, big);
    Bytes over(1 + 200, 0x5a);
    over[0] = 200;
    write(t, over);
  }
  write(t, {});   // address only
  s = status(t);
  CHECK(s.queued == 1 && s.rx_frames == 1 && s.errors == 5);
  {
    Bytes full(1 + 128);
    for (size_t i = 0; i < full.size(); ++i) full[i] = static_cast<uint8_t>(i);
    full[0] = 128;
    write(t, full);
    CHECK(readRx(t) == Bytes({0xb1, 0xb2, 0xb3}));
    CHECK(readRx(t) == Bytes(full.begin() + 1, full.end()));
  }

  // ---- mode 3: slots
  CHECK(ok(configure(t, 0x42, P4I2cTarget::kModePreloadedTx)));
  CHECK(read(t, 2) == Bytes({0xFF, 0xFF}));   // no slot: 0xFF
  CHECK(rejectedAs(preload(t, {}, out), kRejectMalformed));
  CHECK(rejectedAs(preload(t, Bytes(129, 1), out), kRejectUnsupported));
  for (uint8_t i = 1; i <= P4I2cTarget::kQueueDepth; ++i) {
    CHECK(ok(preload(t, {static_cast<uint8_t>(0xA0 + i), i}, out)) && out.size() == 1 && out[0] == i);   // serial
    CHECK(status(t).tx_slots == i);
  }
  CHECK(unavailableCause(preload(t, {0x99}, out), out, 2));   // queue_depth unread slots: nothing placed
  s = status(t);
  CHECK(s.tx_slots == P4I2cTarget::kQueueDepth && s.errors == 0);
  // a read of another length still moves to the next slot; past the slot's end 0xFF
  CHECK(read(t, 1) == Bytes({0xA1}));
  CHECK(read(t, 4) == Bytes({0xA2, 0x02, 0xFF, 0xFF}));
  CHECK(status(t).tx_slots == 2);
  CHECK(ok(preload(t, {0xC5}, out)) && out[0] == 5);
  // write-then-read (repeated START): the write is dropped with an error, the read is answered
  CHECK(read(t, 2, {0x10}) == Bytes({0xA3, 0x03}));
  s = status(t);
  CHECK(s.tx_slots == 2 && s.errors == 1 && s.queued == 0 && s.rx_frames == 0);
  write(t, {0x20, 0x21});   // a write alone: dropped, an error
  write(t, {});   // address only: nothing
  CHECK(status(t).errors == 2);
  CHECK(read(t, 2) == Bytes({0xA4, 0x04}));
  CHECK(read(t, 1) == Bytes({0xC5}));
  CHECK(read(t, 1) == Bytes({0xFF}));
  CHECK(status(t).tx_slots == 0);
  // reset clears the slots and their serial
  CHECK(ok(preload(t, {1}, out)));
  CHECK(ok(call(t, P4I2cTarget::kOpReset, {}, out)));
  s = status(t);
  CHECK(s.mode == 3 && s.tx_slots == 0 && s.errors == 0);
  CHECK(ok(preload(t, {1}, out)) && out[0] == 1);
  // preload_tx is mode 3's only
  CHECK(ok(configure(t, 0x42, P4I2cTarget::kModeFixedRx)));
  CHECK(rejectedAs(preload(t, {1}, out), kRejectUnavailable));

  // ---- stretch: up to max_stretch_us, in any state
  CHECK(ok(stretch(t, 0)));
  CHECK(ok(stretch(t, P4I2cTarget::kMaxStretchUs)));
  CHECK(rejectedAs(stretch(t, P4I2cTarget::kMaxStretchUs + 1), kRejectUnsupported));
  CHECK(t.hostStretchUs() == P4I2cTarget::kMaxStretchUs);

  // ---- releasing the plan, then a new one: describe's state (state 0, mode 0, nothing kept, stretch 0)
  CHECK(ok(armRx(t, 2)));
  write(t, {1, 2});
  t.planRelease();
  CHECK(t.planApply(roles, 2));
  s = status(t);
  CHECK(s.state == 0 && s.mode == 0 && !s.armed && s.queued == 0 && s.rx_frames == 0 && s.tx_slots == 0 && s.errors == 0);
  CHECK(t.hostStretchUs() == 0);
  CHECK(!t.hostTransaction(nullptr, 0));
  CHECK(unavailableCause(call(t, P4I2cTarget::kOpReadRx, {}, out), out, 6));

  t.planRelease();
  printf("i2c-target: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
