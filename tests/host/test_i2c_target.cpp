// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.fixture.i2c-target (oep-spec oep-if-fixture §3) with a fake controller (OEP_HOST_FAKE_I2C_SLAVE:
// hostTransaction runs a transaction through the same code the interrupt handler calls at STOP) - the plan's roles,
// configure / plan release, a write with data one frame (past max_length cut, an error), the preload slots answering
// reads in order (0xFF without one), an address-only write, queue overflow, stretch's limit, read_rx / preload_tx in
// state 0, ops 0x02 / 0x06 (gone) unknown.
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
  uint8_t state, queued;
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
  // cause TLV: 01 len(u16) cause
  return rejectedAs(r, kRejectUnavailable) && out.size() >= 4 && out[0] == 0x01 && out[1] == 1 && out[2] == 0 &&
         out[3] == cause;
}
static Status status(P4I2cTarget &t) {
  Bytes out;
  const Result r = call(t, P4I2cTarget::kOpStatus, {}, out);
  Status s = {};
  if (!ok(r) || out.size() != 11) { CHECK(false); return s; }
  s = {out[0], out[1], getU32(out.data() + 2), out[6], getU32(out.data() + 7)};
  return s;
}
static Result configure(P4I2cTarget &t, uint8_t address) {
  Bytes out;
  return call(t, P4I2cTarget::kOpConfigure, {address}, out);
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

// The channel bitmap role_channels (role u8, base u16, bitmap) declares for `role` in a describe (base 0), 0 when absent.
// TLVs are tag(u8) len(u16 LE) value.
static uint64_t roleMask(const uint8_t *d, size_t n, uint8_t role) {
  for (size_t i = 0; i + kTlvHeader <= n; i += kTlvHeader + getU16(d + i + 1)) {
    const size_t len = getU16(d + i + 1);
    if (d[i] != kTagRoleChannels || len < 3 || d[i + 3] != role) continue;
    uint64_t m = 0;
    for (size_t k = 0; k < len - 3 && k < 8; ++k) m |= uint64_t{d[i + 6 + k]} << (8 * k);
    return m;
  }
  return 0;
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
    // a channel outside role_channels: unsupported; a declared one something else holds: unavailable (core §8)
    const RoleAssignment undeclared[] = {{0, P4I2cTarget::kRoleSda, 4}, {0, P4I2cTarget::kRoleScl, 6}};
    CHECK(t.planCheck(undeclared, 2) == kRejectUnsupported);
    CHECK(pins.claim(5, 99));
    const RoleAssignment held[] = {{0, P4I2cTarget::kRoleSda, 4}, {0, P4I2cTarget::kRoleScl, 5}};
    CHECK(t.planCheck(held, 2) == kRejectUnavailable);
    pins.release(99);
  }
  // no plan: configure is unavailable cause 6; stretch is taken in any state
  CHECK(unavailableCause(call(t, P4I2cTarget::kOpConfigure, {0x42}, out), out, 6));
  CHECK(ok(stretch(t, 50)));
  CHECK(t.hostStretchUs() == 50);
  const RoleAssignment roles[] = {{0, P4I2cTarget::kRoleSda, 4}, {0, P4I2cTarget::kRoleScl, 5}};
  CHECK(t.planCheck(roles, 2) == 0);
  CHECK(t.planApply(roles, 2));

  // describe: max_stretch_us (0x41); stretch itself is declared by the endpoint's ops tag (offers), not by features
  {
    uint8_t d[128];
    const size_t n = t.describe(d, sizeof d);
    bool found = false;
    for (size_t i = 0; i + kTlvHeader <= n; i += kTlvHeader + getU16(d + i + 1))
      if (d[i] == reg::fixture_i2c_target::kTlvDescribeMaxStretchUs && getU16(d + i + 1) == 4)
        found = getU32(d + i + 3) == P4I2cTarget::kMaxStretchUs;
    CHECK(found && P4I2cTarget::kMaxStretchUs >= 30000);
    CHECK(t.offers(P4I2cTarget::kOpStretch) && t.offers(P4I2cTarget::kOpPreloadTx));
    // ops 0x02 (arm_rx) and 0x06 (reset) are gone: not offered, unknown_operation
    CHECK(!t.offers(0x02) && !t.offers(0x06));
    CHECK(rejectedAs(call(t, 0x02, {0, 1}, out), kRejectUnknownOperation));
    CHECK(rejectedAs(call(t, 0x06, {}, out), kRejectUnknownOperation));
    // features: only bit2, the internal pull-ups start() enables (fixture §3); no other tag past queue_depth (0x40) and
    // max_stretch_us (0x41)
    uint32_t features = 0xffffffff;
    bool other = false;
    for (size_t i = 0; i + kTlvHeader <= n; i += kTlvHeader + getU16(d + i + 1)) {
      if (d[i] == reg::kDescribeFeatures && getU16(d + i + 1) == 4) features = getU32(d + i + 3);
      if (d[i] > reg::fixture_i2c_target::kTlvDescribeMaxStretchUs) other = true;
    }
    CHECK(features == reg::fixture_i2c_target::kFeaturesInternalPullups && !other);
    CHECK(P4I2cTarget::kQueueDepth >= 2);
  }

  {   // an input-only channel (the classic ESP32's GPIO34-39): not in SDA / SCL's role_channels, refused unsupported
    static PinTable in_pins((1ull << 4) | (1ull << 5) | (1ull << 34));
    in_pins.setInputOnly(1ull << 34);
    static P4I2cTarget t2(in_pins);
    uint8_t d[128];
    const size_t n = t2.describe(d, sizeof d);
    CHECK(roleMask(d, n, P4I2cTarget::kRoleSda) == 0x30 && roleMask(d, n, P4I2cTarget::kRoleScl) == 0x30);
    const RoleAssignment on34[] = {{0, P4I2cTarget::kRoleSda, 34}, {0, P4I2cTarget::kRoleScl, 5}};
    CHECK(t2.planCheck(on34, 2) == kRejectUnsupported);
  }

  {   // the classic ESP32 firmware's channels with the library's input-only mask (GPIO34-39; the firmware had 36-39
      // only and offered 34 / 35): every channel role_channels declares for SDA / SCL passes planCheck, every other one
      // is refused unsupported - the plan check and the declaration agree channel by channel
    CHECK(kEsp32InputOnlyPins == (0x3full << 34));
    static PinTable esp_pins((1ull << 4) | (1ull << 5) | (1ull << 13) | (1ull << 14) | (1ull << 16) | (1ull << 17) | (1ull << 18) |
      (1ull << 19) | (1ull << 21) | (1ull << 22) | (1ull << 23) | (1ull << 25) | (1ull << 26) | (1ull << 27) | (1ull << 32) |
      (1ull << 33) | (1ull << 34) | (1ull << 35) | (1ull << 36) | (1ull << 39));
    esp_pins.setInputOnly(kEsp32InputOnlyPins);
    static P4I2cTarget t3(esp_pins);
    uint8_t d[128];
    const size_t n = t3.describe(d, sizeof d);
    const uint64_t sda = roleMask(d, n, P4I2cTarget::kRoleSda), scl = roleMask(d, n, P4I2cTarget::kRoleScl);
    CHECK(sda == scl && !(sda & kEsp32InputOnlyPins) && (sda & (1ull << 33)) && (sda & (1ull << 4)));
    int disagree = 0;
    for (uint16_t c = 0; c < 64; ++c) {
      const uint16_t other = c == 4 ? 5 : 4;
      const RoleAssignment as_sda[] = {{0, P4I2cTarget::kRoleSda, c}, {0, P4I2cTarget::kRoleScl, other}};
      const RoleAssignment as_scl[] = {{0, P4I2cTarget::kRoleSda, other}, {0, P4I2cTarget::kRoleScl, c}};
      const bool declared = (sda >> c) & 1;
      if ((t3.planCheck(as_sda, 2) != kRejectUnsupported) != declared) ++disagree;
      if ((t3.planCheck(as_scl, 2) != kRejectUnsupported) != declared) ++disagree;
    }
    CHECK(disagree == 0);
    const RoleAssignment on3435[] = {{0, P4I2cTarget::kRoleSda, 34}, {0, P4I2cTarget::kRoleScl, 35}};   // the harness's pick
    CHECK(t3.planCheck(on3435, 2) == kRejectUnsupported);
  }

  // state 0: read_rx and preload_tx are unavailable cause 6
  CHECK(unavailableCause(call(t, P4I2cTarget::kOpReadRx, {}, out), out, 6));
  CHECK(unavailableCause(preload(t, {1}, out), out, 6));
  CHECK(rejectedAs(configure(t, 0x80), kRejectMalformed));
  CHECK(rejectedAs(call(t, P4I2cTarget::kOpConfigure, {}, out), kRejectMalformed));   // no address
  // the reserved addresses 0x00 - 0x07 and 0x78 - 0x7F: unsupported, payload 0x00 (fixture §3); 0x08 / 0x77 taken
  for (const uint8_t a : {0x00, 0x01, 0x07, 0x78, 0x7C, 0x7F}) {
    CHECK(rejectedAs(call(t, P4I2cTarget::kOpConfigure, {a}, out), kRejectUnsupported) && out.size() == 1 && out[0] == 0);
  }
  CHECK(ok(configure(t, 0x08)));
  CHECK(ok(configure(t, 0x77)));

  // ---- writes: one with data is one frame
  CHECK(ok(configure(t, 0x42)));
  Status s = status(t);
  CHECK(s.state == 1 && s.queued == 0 && s.rx_frames == 0 && s.tx_slots == 0 && s.errors == 0);
  write(t, {1, 2, 3});
  write(t, {});   // the address alone: nothing
  write(t, {0x44});
  s = status(t);
  CHECK(s.queued == 2 && s.rx_frames == 2 && s.errors == 0);
  uint8_t pending = 0xEE;
  CHECK(readRx(t, &pending) == Bytes({1, 2, 3}) && pending == 1);
  CHECK(readRx(t, &pending) == Bytes({0x44}) && pending == 0);
  CHECK(readRx(t, &pending).empty() && pending == 0);
  {   // max_length bytes: a frame; past it: the first max_length bytes, an error
    Bytes full(P4I2cTarget::kMaxFrame);
    for (size_t i = 0; i < full.size(); ++i) full[i] = static_cast<uint8_t>(i);
    write(t, full);
    Bytes over(P4I2cTarget::kMaxFrame + 72);
    for (size_t i = 0; i < over.size(); ++i) over[i] = static_cast<uint8_t>(0x80 + i);
    write(t, over);
    s = status(t);
    CHECK(s.queued == 2 && s.rx_frames == 4 && s.errors == 1);
    CHECK(readRx(t) == full);
    CHECK(readRx(t) == Bytes(over.begin(), over.begin() + P4I2cTarget::kMaxFrame));
  }
  // overflow: the frame past queue_depth is dropped, an error, not in rx_frames; one too long as well: one error
  for (int i = 0; i < 4; ++i) write(t, {static_cast<uint8_t>(i), 0});
  write(t, {0x55});
  s = status(t);
  CHECK(s.queued == 4 && s.rx_frames == 8 && s.errors == 2);
  write(t, Bytes(P4I2cTarget::kMaxFrame + 1, 0x66));
  s = status(t);
  CHECK(s.queued == 4 && s.rx_frames == 8 && s.errors == 3);
  CHECK(readRx(t) == Bytes({0, 0}));
  // a read with no slot sends 0xFF and counts nothing
  CHECK(read(t, 3) == Bytes({0xFF, 0xFF, 0xFF}));
  s = status(t);
  CHECK(s.queued == 3 && s.errors == 3 && s.tx_slots == 0);
  // configure clears frames, slots and the counts, and keeps stretch
  CHECK(ok(preload(t, {7}, out)));
  CHECK(ok(stretch(t, 30000)));
  CHECK(ok(configure(t, 0x42)));
  s = status(t);
  CHECK(s.state == 1 && s.queued == 0 && s.rx_frames == 0 && s.tx_slots == 0 && s.errors == 0);
  CHECK(t.hostStretchUs() == 30000);
  CHECK(read(t, 1) == Bytes({0xFF}));

  // ---- reads: the slots in order
  CHECK(rejectedAs(preload(t, {}, out), kRejectMalformed));
  CHECK(rejectedAs(preload(t, Bytes(P4I2cTarget::kMaxFrame + 1, 1), out), kRejectUnsupported));
  for (uint8_t i = 1; i <= P4I2cTarget::kQueueDepth; ++i) {
    CHECK(ok(preload(t, {static_cast<uint8_t>(0xA0 + i), i}, out)) && out.empty());   // no payload
    CHECK(status(t).tx_slots == i);
  }
  CHECK(unavailableCause(preload(t, {0x99}, out), out, 2));   // queue_depth unread slots: nothing placed
  s = status(t);
  CHECK(s.tx_slots == P4I2cTarget::kQueueDepth && s.errors == 0);
  // a read of another length still moves to the next slot; past the slot's end 0xFF
  CHECK(read(t, 1) == Bytes({0xA1}));
  CHECK(read(t, 4) == Bytes({0xA2, 0x02, 0xFF, 0xFF}));
  CHECK(status(t).tx_slots == 2);
  CHECK(ok(preload(t, {0xC5}, out)));
  // write-then-read (repeated START): the write is a frame, the read is answered from the next slot
  CHECK(read(t, 2, {0x10}) == Bytes({0xA3, 0x03}));
  s = status(t);
  CHECK(s.tx_slots == 2 && s.errors == 0 && s.queued == 1 && s.rx_frames == 1);
  CHECK(readRx(t) == Bytes({0x10}));
  CHECK(read(t, 2) == Bytes({0xA4, 0x04}));
  CHECK(read(t, 1) == Bytes({0xC5}));
  CHECK(read(t, 1) == Bytes({0xFF}));
  CHECK(status(t).tx_slots == 0);

  // ---- stretch: up to max_stretch_us, in any state
  CHECK(ok(stretch(t, 0)));
  CHECK(ok(stretch(t, P4I2cTarget::kMaxStretchUs)));
  CHECK(rejectedAs(stretch(t, P4I2cTarget::kMaxStretchUs + 1), kRejectUnsupported));
  CHECK(t.hostStretchUs() == P4I2cTarget::kMaxStretchUs);

  // ---- releasing the plan, then a new one: describe's state (state 0, nothing kept, stretch 0)
  write(t, {1, 2});
  CHECK(ok(preload(t, {3}, out)));
  t.planRelease();
  CHECK(t.planApply(roles, 2));
  s = status(t);
  CHECK(s.state == 0 && s.queued == 0 && s.rx_frames == 0 && s.tx_slots == 0 && s.errors == 0);
  CHECK(t.hostStretchUs() == 0);
  CHECK(!t.hostTransaction(nullptr, 0));
  CHECK(unavailableCause(call(t, P4I2cTarget::kOpReadRx, {}, out), out, 6));

  t.planRelease();
  printf("i2c-target: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
