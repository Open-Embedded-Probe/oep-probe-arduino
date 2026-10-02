// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the idle states of the pin table (oep-spec oep-if-probe-config §1 idle, oep-core §8) and the gpio
// fixture's take (oep-if-fixture §1) - modes 3 / 4 drive their level while a channel is free, an output idle on a
// channel that cannot drive is refused, every release goes to the idle state, a channel taken by a gpio plan keeps its
// idle drive until the first set (no glitch on a power line), a debug wire's release touches only channels with an
// idle, the reset line goes back to its idle, and a plan replaced through the endpoint releases only the channels that
// leave it (a power line kept in both plans never blinks off). The output drive strength (oep-if-fixture §1.1): describe's
// drive_levels, set's drive TLV per element (malformed / ignored), the inheritance set -> idle -> default, read's drive.
#include <stdio.h>

#include <vector>

#include "OepEndpoint.h"
#include "OepFixture.h"

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

static bool drives(int pin, int level) { return g_pin_mode[pin] == OUTPUT && g_pin_level[pin] == level; }
static bool floating(int pin) { return g_pin_mode[pin] == INPUT; }

static Result gpioCall(FixtureGpio &g, uint8_t op, const Bytes &payload, Bytes &out) {
  out.assign(64, 0);
  const Result r = g.handle(op, payload.data(), payload.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static Bytes setOne(uint16_t channel, uint8_t mode) {
  Bytes p(4);
  p[0] = 1;
  putU16(p.data() + 1, channel);
  p[3] = mode;
  return p;
}
static Bytes readOne(uint16_t channel) {
  Bytes p(3);
  p[0] = 1;
  putU16(p.data() + 1, channel);
  return p;
}

static void testIdleModes() {
  PinTable pins((1ull << 4) | (1ull << 5) | (1ull << 6) | (1ull << 34));
  pins.setInputOnly(1ull << 34);
  CHECK(pins.setIdle(4, PinTable::kIdleOutputHigh));
  CHECK(drives(4, HIGH));   // free: driven now
  CHECK(pins.setIdle(5, PinTable::kIdleOutputLow));
  CHECK(drives(5, LOW));
  CHECK(pins.setIdle(6, PinTable::kIdlePullUp));
  CHECK(g_pin_mode[6] == INPUT_PULLUP);
  CHECK(!pins.setIdle(34, PinTable::kIdleOutputHigh));   // input only: no output idle
  CHECK(!pins.setIdle(34, PinTable::kIdleOutputLow));
  CHECK(pins.idle(34) == PinTable::kIdleUnset);
  CHECK(pins.setIdle(34, PinTable::kIdlePullDown));   // the input modes still are
  CHECK(!pins.canOutput(34) && pins.canOutput(4));
  CHECK(!pins.setIdle(4, 5));   // not a mode
  CHECK(pins.idle(4) == PinTable::kIdleOutputHigh);
  CHECK(pins.setIdle(4, PinTable::kIdleUnset));   // removed: Hi-Z
  CHECK(floating(4));
}

static void testReleaseGoesToIdle() {
  PinTable pins((1ull << 8) | (1ull << 9));
  CHECK(pins.claim(8, 3));
  const int before = g_pin_changes;
  CHECK(pins.setIdle(8, PinTable::kIdleOutputHigh));   // owned: kept, not applied
  CHECK(g_pin_changes == before);
  pins.release(3);
  CHECK(drives(8, HIGH));   // released: to the output idle, not Hi-Z
  // a channel to be disabled: the mode kept without touching the pad
  const int before2 = g_pin_changes;
  CHECK(pins.setIdle(9, PinTable::kIdleOutputLow, false));
  CHECK(g_pin_changes == before2);
  pins.setDisabled(1ull << 9);
  CHECK(g_pin_changes == before2);
  pins.setDisabled(0);   // enabled again: its idle now
  CHECK(drives(9, LOW));
}

static void testGpioTakeKeepsIdle() {
  PinTable pins((1ull << 10) | (1ull << 11));
  FixtureGpio gpio(pins, 0, 1);
  CHECK(pins.setIdle(10, PinTable::kIdleOutputHigh));   // a power_hi line, powered while idle
  CHECK(drives(10, HIGH));
  const RoleAssignment roles[] = {{1, reg::fixture_gpio::kRoleLine, 10}, {1, reg::fixture_gpio::kRoleLine, 11}};
  CHECK(gpio.planCheck(roles, 2) == 0);
  const int before = g_pin_changes;
  CHECK(gpio.planApply(roles, 2));
  CHECK(g_pin_changes == before);   // taking it changes nothing
  CHECK(drives(10, HIGH));
  Bytes out;
#if defined(OEP_HOST_FAKE_DRIVE)
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpRead, readOne(10), out)) && out.size() == 5 && out[1] == 1 && out[4] == 2);
#else
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpRead, readOne(10), out)) && out.size() == 2 && out[1] == 1);
#endif
  // the first set changes it (output low: power off), and nothing before it did
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setOne(10, reg::fixture_gpio::kModeOutputLow), out)));
  CHECK(drives(10, LOW));
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setOne(11, reg::fixture_gpio::kModeOutputHigh), out)));
  CHECK(drives(11, HIGH));
  // plan_release: 10 back to its output idle (powered), 11 (no idle) to Hi-Z
  gpio.planRelease();
  CHECK(drives(10, HIGH));
  CHECK(floating(11));
  CHECK(pins.free(10) && pins.free(11));
}

static void testWireReleaseAndReset() {
  PinTable pins((1ull << 12) | (1ull << 13) | (1ull << 14));
  CHECK(pins.setIdle(13, PinTable::kIdlePullUp));
  CHECK(pins.claim(12, 0xf0) && pins.claim(13, 0xf0));
  pinMode(12, OUTPUT);   // what the PHY left (the test's stand-in)
  pinMode(13, OUTPUT);
  pins.releaseToIdle(0xf0);
  CHECK(g_pin_mode[12] == OUTPUT);   // no idle set: the PHY's state kept
  CHECK(g_pin_mode[13] == INPUT_PULLUP);   // its idle
  CHECK(pins.free(12) && pins.free(13));
  // the reset line: used without a claim, then rested to its idle
  CHECK(pins.setIdle(14, PinTable::kIdleOutputHigh));
  pinMode(14, INPUT);
  pins.rest(14);
  CHECK(drives(14, HIGH));
  CHECK(pins.claim(14, 1));
  pinMode(14, INPUT);
  pins.rest(14);   // held by someone: untouched
  CHECK(floating(14));
}

class NullStream final : public Stream {
 public:
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  size_t write(uint8_t) override { return 1; }
};

// plan_apply replacing a gpio plan (the same fn, a new channel set): 0.0.27 put every old channel to Hi-Z before
// claiming the new set, so a power line in both plans glitched off (ESP32-P4, the target lost power).
static void testReplaceKeepsSharedChannels() {
  static NullStream stream;
  static uint8_t rx[512], tx[512];
  static Endpoint ep(stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kVendorBulk, 0);
  static PinTable pins((1ull << 40) | (1ull << 41) | (1ull << 42) | (1ull << 43));
  static FixtureGpio gpio(pins, 0, 1);
  ep.add(gpio);
  ep.setPins(&pins);
  const uint16_t fn = 1;
  CHECK(pins.setIdle(43, PinTable::kIdleOutputLow));
  const RoleAssignment first[] = {{fn, reg::fixture_gpio::kRoleLine, 40}, {fn, reg::fixture_gpio::kRoleLine, 41}};
  CHECK(ep.replacePlan(first, 2, &fn, 1) == 0);
  Bytes out;
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setOne(40, reg::fixture_gpio::kModeOutputHigh), out)));   // the power
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setOne(41, reg::fixture_gpio::kModeOutputHigh), out)));
  CHECK(drives(40, HIGH) && drives(41, HIGH));
  // replaced: 40 kept, 41 leaves, 42 and 43 new
  const RoleAssignment second[] = {{fn, reg::fixture_gpio::kRoleLine, 40}, {fn, reg::fixture_gpio::kRoleLine, 42},
                                   {fn, reg::fixture_gpio::kRoleLine, 43}};
  int changes40 = 0;
  const int mode40 = g_pin_mode[40], level40 = g_pin_level[40];
  g_pin_mode[42] = INPUT;   // its idle state (Hi-Z)
  CHECK(ep.replacePlan(second, 3, &fn, 1) == 0);
  changes40 += g_pin_mode[40] != mode40 || g_pin_level[40] != level40;
  CHECK(changes40 == 0 && drives(40, HIGH));   // in both plans: its drive untouched
  CHECK(floating(41) && pins.free(41));         // left the plan: to its idle state (Hi-Z)
  CHECK(floating(42) && drives(43, LOW));       // new: their idle states until the first set
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpRead, readOne(40), out)) && out[1] == 1);
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setOne(43, reg::fixture_gpio::kModeOutputHigh), out)));
  CHECK(drives(43, HIGH));
  // a refused replacement (a channel this gpio does not have) changes nothing either
  const RoleAssignment bad[] = {{fn, reg::fixture_gpio::kRoleLine, 40}, {fn, reg::fixture_gpio::kRoleLine, 50}};
  CHECK(ep.replacePlan(bad, 2, &fn, 1) != 0);
  CHECK(drives(40, HIGH) && drives(43, HIGH) && !pins.free(40) && !pins.free(43));
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpRead, readOne(43), out)) && out[1] == 1);
  // released for good (no new plan): each channel to its idle state
  CHECK(ep.replacePlan(nullptr, 0, &fn, 1) == 0);   // the plan removed
  CHECK(floating(40) && floating(42) && drives(43, LOW));
}

// ---- output drive strength (oep-if-fixture §1.1; OEP_HOST_FAKE_DRIVE: levels 5 / 10 / 20 / 40 mA, default 2) ----

static Bytes setWithDrive(std::vector<std::pair<uint16_t, uint8_t>> elements, std::vector<std::vector<uint8_t>> drives) {
  Bytes p;
  p.push_back(static_cast<uint8_t>(elements.size()));
  for (auto &e : elements) { p.push_back(e.first & 0xff); p.push_back(e.first >> 8); p.push_back(e.second); }
  for (auto &d : drives) {   // index kind value(u16) - or whatever bytes a test gives
    p.push_back(reg::fixture_gpio::kTlvSetDrive);
    p.push_back(static_cast<uint8_t>(d.size()));
    p.insert(p.end(), d.begin(), d.end());
  }
  return p;
}
#if defined(OEP_HOST_FAKE_DRIVE)
static bool ignoredDrive(const Bytes &out, size_t at) {
  return out.size() == at + 3 && out[at] == kTagIgnored && out[at + 1] == 1 && out[at + 2] == reg::fixture_gpio::kTlvSetDrive;
}

static void testDriveDescribe() {
  PinTable pins(1ull << 20);
  FixtureGpio gpio(pins, 0, 1);
  uint8_t d[64];
  const size_t n = gpio.describe(d, sizeof d);
  bool found = false;
  for (size_t at = 0; at + 2 <= n; at += 2u + d[at + 1]) {
    if (d[at] != reg::fixture_gpio::kTlvDescribeDriveLevels) continue;
    found = d[at + 1] == 10 && d[at + 2] == 2 && d[at + 3] == 4 && getU16(d + at + 4) == 5 && getU16(d + at + 6) == 10 &&
            getU16(d + at + 8) == 20 && getU16(d + at + 10) == 40;
  }
  CHECK(found);
}

static void testDriveSet() {
  PinTable pins((1ull << 20) | (1ull << 21) | (1ull << 22) | (1ull << 23));
  FixtureGpio gpio(pins, 0, 1);
  CHECK(pins.setIdle(23, PinTable::kIdleOutputHigh, true, 0));   // an output idle at level 0
  CHECK(drives(23, HIGH) && g_pin_drive[23] == 0);
  const RoleAssignment roles[] = {{1, reg::fixture_gpio::kRoleLine, 20}, {1, reg::fixture_gpio::kRoleLine, 21},
                                  {1, reg::fixture_gpio::kRoleLine, 22}, {1, reg::fixture_gpio::kRoleLine, 23}};
  CHECK(gpio.planApply(roles, 4));
  Bytes out;
  const uint8_t hi = reg::fixture_gpio::kModeOutputHigh, lo = reg::fixture_gpio::kModeOutputLow;
  // taken: the idle's level read back, the others not driven
  Bytes rd = {4, 20, 0, 21, 0, 22, 0, 23, 0};
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpRead, rd, out)) && out.size() == 5 + 6 && out[5] == reg::fixture_gpio::kTlvReadAnswerDrive &&
        out[6] == 4 && out[7] == 0xff && out[8] == 0xff && out[9] == 0xff && out[10] == 0);
  // one request, a strength per element: 20 at level 3, 21 by mA (12 mA -> 10 mA, level 1), 22 without (default 2),
  // 23 without (its idle's, 0)
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet,
                    setWithDrive({{20, hi}, {21, lo}, {22, hi}, {23, lo}}, {{0, 0, 3, 0}, {1, 1, 12, 0}}), out)) && out.empty());
  CHECK(drives(20, HIGH) && g_pin_drive[20] == 3);
  CHECK(drives(21, LOW) && g_pin_drive[21] == 1);
  CHECK(drives(22, HIGH) && g_pin_drive[22] == 2);
  CHECK(drives(23, LOW) && g_pin_drive[23] == 0);
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpRead, rd, out)) && out[7] == 3 && out[8] == 1 && out[9] == 2 && out[10] == 0);
  // two ignored: 0x01 listed for each; an unknown tag once, before them
  Bytes two = setWithDrive({{20, hi}, {21, hi}}, {{0, 0, 9, 0}, {1, 0, 4, 0}});
  two.insert(two.end(), {0x30, 0});
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, two, out)) && out.size() == 5 && out[0] == kTagIgnored && out[1] == 3 &&
        out[2] == 0x30 && out[3] == reg::fixture_gpio::kTlvSetDrive && out[4] == reg::fixture_gpio::kTlvSetDrive);
  // critical where it would be ignored: rejected unsupported with the tag as received; a usable one critical is taken
  Bytes crit = setWithDrive({{20, hi}}, {{0, 0, 4, 0}});
  crit[4] |= kTagCritical;
  CHECK(gpioCall(gpio, FixtureGpio::kOpSet, crit, out).detail == kRejectUnsupported && out.size() == 1 &&
        out[0] == (reg::fixture_gpio::kTlvSetDrive | kTagCritical));
  crit = setWithDrive({{20, hi}}, {{0, 0, 1, 0}});
  crit[4] |= kTagCritical;
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, crit, out)) && out.empty() && g_pin_drive[20] == 1);
  // a mA under every level: level 0
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{21, lo}}, {{0, 1, 1, 0}}), out)) && g_pin_drive[21] == 0);
  // kept until the channel is set again: a set without drive takes the idle's / default again
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{20, lo}}, {}), out)) && g_pin_drive[20] == 2);
  // a level the probe does not have: that TLV ignored (listed), the set done at the default
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{20, hi}}, {{0, 0, 4, 0}}), out)) && ignoredDrive(out, 0));
  CHECK(drives(20, HIGH) && g_pin_drive[20] == 2);
  // an undefined kind (2+): a value this probe cannot handle - that TLV ignored, or unsupported when critical (C-02)
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{20, hi}}, {{0, 2, 0, 0}}), out)) && ignoredDrive(out, 0));
  CHECK(drives(20, HIGH) && g_pin_drive[20] == 2);
  Bytes kind2 = setWithDrive({{20, hi}}, {{0, 2, 0, 0}});
  kind2[4] |= kTagCritical;
  CHECK(gpioCall(gpio, FixtureGpio::kOpSet, kind2, out).detail == kRejectUnsupported && out.size() == 1 &&
        out[0] == (reg::fixture_gpio::kTlvSetDrive | kTagCritical));
  // malformed, nothing done: index out of range, the same index twice, an element not 3 / 4, length
  const int before = g_pin_changes;
  CHECK(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{20, lo}}, {{1, 0, 0, 0}}), out).detail == kRejectMalformed);
  CHECK(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{20, lo}}, {{0, 0, 0, 0}, {0, 0, 1, 0}}), out).detail == kRejectMalformed);
  // an undefined mode (8+) is unsupported like an undeclared one, with the channel and its index (C-02)
  CHECK(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{20, 8}}, {}), out).detail == kRejectUnsupported && out.size() >= 1 &&
        out[0] == kTagValue);
  CHECK(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{20, lo}, {21, 0}}, {{1, 0, 0, 0}}), out).detail == kRejectMalformed);
  CHECK(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{20, lo}}, {{0, 0, 0}}), out).detail == kRejectMalformed);
  // malformed wins wherever it is: after an ignored one, and after a critical one that would be unsupported
  CHECK(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{20, lo}}, {{0, 0, 9, 0}, {5, 0, 0, 0}}), out).detail == kRejectMalformed);
  Bytes late = setWithDrive({{20, lo}}, {{0, 0, 9, 0}, {5, 0, 0, 0}});
  late[4] |= kTagCritical;
  CHECK(gpioCall(gpio, FixtureGpio::kOpSet, late, out).detail == kRejectMalformed);
  CHECK(g_pin_changes == before && drives(20, HIGH) && g_pin_drive[20] == 2);
  // an input: not driven (0xFF), and the pad back at the default strength for whoever takes it next
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{20, hi}}, {{0, 0, 3, 0}}), out)) && g_pin_drive[20] == 3);
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setOne(20, reg::fixture_gpio::kModeInput), out)) && g_pin_drive[20] == 2);
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpRead, readOne(20), out)) && out.size() == 5 && out[4] == 0xff);
  // released: each to its idle state with its strength (23: output high at level 0; 21 at level 0 -> Hi-Z, default)
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{23, lo}}, {{0, 0, 3, 0}}), out)) && g_pin_drive[23] == 3);
  gpio.planRelease();
  CHECK(drives(23, HIGH) && g_pin_drive[23] == 0 && pins.drivenLevel(23) == 0);
  CHECK(floating(21) && g_pin_drive[21] == 2 && pins.drivenLevel(21) == PinTable::kNotDriven);
}

#else
// A chip without drive_levels: drive is an unknown tag - no form checked, each one ignored (0x01 listed once per TLV),
// a critical one rejected unsupported; read has no drive TLV.
static void testNoDriveLevels() {
  PinTable pins(1ull << 20);
  FixtureGpio gpio(pins, 0, 1);
  uint8_t d[64];
  const size_t n = gpio.describe(d, sizeof d);
  for (size_t at = 0; at + 2 <= n; at += 2u + d[at + 1]) CHECK(d[at] != reg::fixture_gpio::kTlvDescribeDriveLevels);
  const RoleAssignment roles[] = {{1, reg::fixture_gpio::kRoleLine, 20}};
  CHECK(gpio.planApply(roles, 1));
  Bytes out;
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpSet, setWithDrive({{20, 0}}, {{5, 9}, {0, 0, 3, 0}}), out)) && out.size() == 4 &&
        out[0] == kTagIgnored && out[1] == 2 && out[2] == reg::fixture_gpio::kTlvSetDrive && out[3] == reg::fixture_gpio::kTlvSetDrive);
  Bytes crit = setWithDrive({{20, reg::fixture_gpio::kModeOutputHigh}}, {{0, 0, 3, 0}});
  crit[4] |= kTagCritical;
  CHECK(gpioCall(gpio, FixtureGpio::kOpSet, crit, out).detail == kRejectUnsupported);
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpRead, readOne(20), out)) && out.size() == 2);
}
#endif

int main() {
  testIdleModes();
  testReleaseGoesToIdle();
  testGpioTakeKeepsIdle();
  testWireReleaseAndReset();
  testReplaceKeepsSharedChannels();
#if defined(OEP_HOST_FAKE_DRIVE)
  testDriveDescribe();
  testDriveSet();
#else
  testNoDriveLevels();
#endif
  printf("idle: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
