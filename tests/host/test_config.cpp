// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.probe.config set's refusals of item values (oep-if-probe-config §1, C-02): a value a later revision
// may define - a slot's attach policy or idle_clock of 2 or more, a lock scheme not in the table, a bind stream kind
// other than 1 / 2 - is refused unsupported with the item's tag as received (they were malformed); a form that is wrong
// stays malformed (retry_ms on a host slot, a lock whose n is not the scheme's length, boot_reset 2+).
#include <stdio.h>

#include <vector>

#include "OepConfig.h"
#include "OepConsole.h"
#include "OepDmConsole.h"
#include "OepEndpoint.h"
#include "OepTarget.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using Bytes = std::vector<uint8_t>;
namespace cfg = reg::probe_config;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

class NullStream final : public Stream {
 public:
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  size_t write(uint8_t) override { return 1; }
};

// A debug module that never answers: set only checks the items here.
class SilentPhy final : public DmiPhy {
 public:
  bool attach() override { return false; }
  void release() override {}
  bool attached() const override { return false; }
  void write(uint8_t, uint32_t) override {}
  bool setIdleClockLow(bool) override { return true; }
  bool canIdleClockLow() const override { return true; }
  bool setMaxHz(uint32_t) override { return true; }
  bool keepsMaxHz(uint32_t) const override { return true; }
  uint32_t dmiNs() const override { return 0; }
  uint32_t clockHz() const override { return 0; }
  uint32_t retries() const override { return 0; }
  uint32_t transactions() const override { return 0; }

 protected:
  bool readWire(uint8_t, uint32_t &) override { return false; }
};

// A slot item: slot 0 on wire fn 1, pins 0 / 1, the attach / idle_clock / lock / boot_reset given.
static Bytes slotItem(uint8_t attach, uint8_t idle, const Bytes &lock = {}, int boot_reset = -1, uint32_t retry_ms = 0) {
  Bytes v = {0, 1, 0, 0, 0, 1, 0, attach};
  for (int i = 0; i < 4; ++i) v.push_back(uint8_t(retry_ms >> (8 * i)));
  v.insert(v.end(), {0, 0, 0, 0, idle, 0xff, 3, 'd', 'u', 't', uint8_t(lock.size())});
  v.insert(v.end(), lock.begin(), lock.end());
  if (boot_reset >= 0) v.push_back(uint8_t(boot_reset));
  Bytes item = {uint8_t(cfg::kTlvItemSlot | kTagCritical), uint8_t(v.size())};
  item.insert(item.end(), v.begin(), v.end());
  return item;
}
static Result set(Interface &config, const Bytes &items, Bytes &out) {
  out.assign(256, 0);
  const Result r = config.handle(cfg::kOpSet, items.data(), items.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
static bool unsupportedWith(const Result &r, const Bytes &out, uint8_t raw) {
  return r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out.size() >= 1 && out[0] == raw;
}
static bool malformed(const Result &r) { return r.resolution == kResolutionRejected && r.detail == kRejectMalformed; }
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }

int main() {
  static NullStream stream;
  static uint8_t rx[512], tx[512];
  static Endpoint ep(stream, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kUartBridge);
  static SilentPhy phy;
  static Ch32Dm dm(phy);
  static DebugPort port{dm, 0, 1};
  static WireRvswd wire(port, 0);
  static DmConsole driver(dm, phy);
  static TargetConsoleStream console(port, driver, 0);
  static Binds binds;
  static ProbeConfig config(ep, binds);
  ep.add(wire);      // fn 1
  ep.add(console);
  ep.add(config);
  config.addPlace(wire, console);
  const uint8_t slot_raw = cfg::kTlvItemSlot | kTagCritical;
  Bytes out;

  CHECK(ok(set(config, slotItem(cfg::kSlotAttachHost, 0), out)));                 // a host slot, no lock
  CHECK(unsupportedWith(set(config, slotItem(2, 0), out), out, slot_raw));         // attach 2: 0.0.28 malformed
  CHECK(unsupportedWith(set(config, slotItem(0xff, 0), out), out, slot_raw));
  CHECK(unsupportedWith(set(config, slotItem(0, 2), out), out, slot_raw));         // idle_clock 2: 0.0.28 malformed
  // a lock scheme not in the table: unsupported (0.0.28 malformed); targetsel (2) too - these wires do not read it
  const Bytes lock9 = {9, 0xff, 0xff, 0xff, 0xff, 1, 2, 3, 4};
  CHECK(unsupportedWith(set(config, slotItem(0, 0, lock9), out), out, slot_raw));
  const Bytes lock2 = {2, 0xff, 0xff, 0xff, 0xff, 1, 2, 3, 4};
  CHECK(unsupportedWith(set(config, slotItem(0, 0, lock2), out), out, slot_raw));
  const Bytes lock1 = {1, 0xff, 0xff, 0xff, 0xff, 1, 2, 3, 4};
  CHECK(ok(set(config, slotItem(0, 0, lock1), out)));
  // the form stays malformed: a scheme-1 lock with n 2, retry_ms on a host slot, boot_reset 2, boot_reset 1 on a host slot
  const Bytes lock_short = {1, 0xff, 0xff, 1, 2};
  CHECK(malformed(set(config, slotItem(0, 0, lock_short), out)));
  CHECK(malformed(set(config, slotItem(0, 0, {}, -1, 100), out)));
  CHECK(malformed(set(config, slotItem(1, 0, {}, 2), out)));
  CHECK(malformed(set(config, slotItem(0, 0, {}, 1), out)));
  // and malformed before unsupported: an undefined attach with a lock of the wrong length is malformed
  CHECK(malformed(set(config, slotItem(2, 0, lock_short), out)));

  // a bind stream kind other than 1 / 2: unsupported with the item's tag (0.0.28 malformed); a cut element malformed
  const uint8_t bind_raw = cfg::kTlvItemBind | kTagCritical;
  CHECK(ok(set(config, slotItem(0, 0), out)));
  const Bytes mechanism_slot = [] {
    Bytes s = slotItem(0, 0);
    s[2 + 17] = reg::target_console::kMechanismSdi;   // a console, so a bind may carry it
    return s;
  }();
  CHECK(ok(set(config, mechanism_slot, out)));
  Bytes bind = {bind_raw, 8, 0, 0, 0, 1, 3, 3, 0, 0};   // port 0, last-reset, one stream (len 3): kind 3, id 0
  CHECK(unsupportedWith(set(config, bind, out), out, bind_raw));
  bind[7] = 1;   // kind 1, slot 0: taken
  CHECK(ok(set(config, bind, out)));
  const Bytes cut = {bind_raw, 6, 0, 0, 0, 1, 3, 3};   // the element's 3 bytes are not all there
  CHECK(malformed(set(config, cut, out)));

  printf("config: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
