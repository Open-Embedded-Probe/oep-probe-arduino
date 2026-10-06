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
#include "OepPinTable.h"
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

  // core §4.3 (C-21) over the whole set: an earlier item's unknown_function or unsupported does not win over a later
  // item's malformed; an fn named inside an item that does not exist (unknown_function) comes before another item's
  // unsupported, which carries that item's tag as received
  auto concat = [](Bytes a, const Bytes &b) { a.insert(a.end(), b.begin(), b.end()); return a; };
  const uint8_t plan_raw = cfg::kTlvItemPlan | kTagCritical;
  const Bytes plan_fn9 = {plan_raw, 5, 9, 0, 0, 1, 0};   // fn 9: no such fn
  const Bytes plan_fn0 = {plan_raw, 5, 0, 0, 0, 1, 0};   // fn 0: malformed
  CHECK(malformed(set(config, concat(plan_fn9, plan_fn0), out)));
  CHECK(malformed(set(config, concat(slotItem(2, 0), plan_fn0), out)));   // an unsupported slot before it
  CHECK(malformed(set(config, concat(plan_fn9, slotItem(0, 0, lock_short)), out)));
  Result r = set(config, concat(slotItem(2, 0), plan_fn9), out);
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnknownFunction);
  CHECK(unsupportedWith(set(config, concat(slotItem(2, 0), Bytes{0xB5, 0}), out), out, slot_raw));
  CHECK(unsupportedWith(set(config, concat(Bytes{0xB5, 0}, slotItem(2, 0)), out), out, 0xB5));   // the first one
  // a slot naming a wire fn that does not exist: unknown_function before its undefined attach (unsupported)
  Bytes slot_fn9 = slotItem(2, 0);
  slot_fn9[2 + 1] = 9;
  r = set(config, slot_fn9, out);
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnknownFunction);

  {   // label (probe.config §1): text 1-32 bytes of valid UTF-8 without C0 controls / 0x7F, else malformed; a channel
      // not below channels or reserved (not in the pin table) unsupported with the item's tag
    static NullStream s2;
    static uint8_t rx2[512], tx2[512];
    static Endpoint ep2(s2, rx2, sizeof rx2, tx2, sizeof tx2, {512, 1024, 2}, Endpoint::kUartBridge);
    static Binds binds2;
    static ProbeConfig pinned(ep2, binds2);
    static PinTable pins(uint64_t{0xff});   // channels 0-7
    ep2.add(pinned);
    pinned.setPins(&pins);
    const uint8_t label_raw = cfg::kTlvItemLabel | kTagCritical;
    auto label = [&](uint16_t channel, const Bytes &text) {
      Bytes item = {label_raw, uint8_t(2 + text.size()), uint8_t(channel), uint8_t(channel >> 8)};
      item.insert(item.end(), text.begin(), text.end());
      return item;
    };
    CHECK(ok(set(pinned, label(1, {'n', 'r', 's', 't'}), out)));
    CHECK(ok(set(pinned, label(2, {0xC3, 0xA9}), out)));             // U+00E9
    CHECK(ok(set(pinned, label(3, Bytes(32, 'a')), out)));            // label_max_bytes
    CHECK(malformed(set(pinned, label(1, {}), out)));                 // no text
    CHECK(malformed(set(pinned, label(1, Bytes(33, 'a')), out)));
    CHECK(malformed(set(pinned, label(1, {'a', 0x0A}), out)));        // a C0 control
    CHECK(malformed(set(pinned, label(1, {'a', 0x7F}), out)));
    CHECK(malformed(set(pinned, label(1, {'a', 0xC3}), out)));        // cut short
    CHECK(malformed(set(pinned, label(1, {0xC0, 0x80}), out)));       // over-long form
    CHECK(unsupportedWith(set(pinned, label(8, {'x'}), out), out, label_raw));   // beyond the channels
    CHECK(malformed(set(pinned, label(8, {0x01}), out)));             // malformed before unsupported
    // idle (probe.config §1, core §4.3 "Contradictions and undefined values"): mode 5 with a drive is unsupported for the
    // mode, not malformed for a drive on a mode other than 3 / 4; mode 2 with a drive stays malformed
    const uint8_t idle_raw = cfg::kTlvItemIdle | kTagCritical;
    CHECK(unsupportedWith(set(pinned, Bytes{idle_raw, 6, 1, 0, 5, 0, 0, 0}, out), out, idle_raw));
    CHECK(unsupportedWith(set(pinned, Bytes{idle_raw, 3, 1, 0, 5}, out), out, idle_raw));
    CHECK(malformed(set(pinned, Bytes{idle_raw, 6, 1, 0, 2, 0, 0, 0}, out)));
    CHECK(malformed(set(pinned, Bytes{idle_raw, 4, 1, 0, 5, 0}, out)));   // the length still first
    // slots_max 0 (no wire place): describe's items leave slot out, and a slot item is unsupported with its tag
    uint8_t d[64];
    const size_t dn = pinned.describe(d, sizeof d);
    bool listed = false, slots_max0 = false;
    for (size_t at = 0; at + 2 <= dn; at += 2u + d[at + 1]) {
      if (d[at] == cfg::kTlvDescribeItems)
        for (size_t k = 0; k < d[at + 1]; ++k) listed |= d[at + 2 + k] == cfg::kTlvItemSlot;
      if (d[at] == cfg::kTlvDescribeSlotsMax) slots_max0 = d[at + 1] == 1 && d[at + 2] == 0;
    }
    CHECK(dn > 0 && !listed && slots_max0);
    CHECK(unsupportedWith(set(pinned, slotItem(0, 0), out), out, slot_raw));
    CHECK(unsupportedWith(set(pinned, Bytes{slot_raw, 1, 0}, out), out, slot_raw));   // its form is not looked at
    // a probe without a pin table has no channels to name: label is not declared (unsupported, its tag)
    CHECK(unsupportedWith(set(config, label(1, {'x'}), out), out, label_raw));
  }

  printf("config: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
