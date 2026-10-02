// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the idle states of the pin table (oep-spec oep-if-probe-config §1 idle, oep-core §8) and the gpio
// fixture's take (oep-if-fixture §1) - modes 3 / 4 drive their level while a channel is free, an output idle on a
// channel that cannot drive is refused, every release goes to the idle state, a channel taken by a gpio plan keeps its
// idle drive until the first set (no glitch on a power line), a debug wire's release touches only channels with an
// idle, the reset line goes back to its idle, and a plan replaced through the endpoint releases only the channels that
// leave it (a power line kept in both plans never blinks off).
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
  CHECK(ok(gpioCall(gpio, FixtureGpio::kOpRead, readOne(10), out)) && out.size() == 2 && out[1] == 1);
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

int main() {
  testIdleModes();
  testReleaseGoesToIdle();
  testGpioTakeKeepsIdle();
  testWireReleaseAndReset();
  testReplaceKeepsSharedChannels();
  printf("idle: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
