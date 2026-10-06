// Host tests: oep.probe.config set's refusals of item values (oep-if-probe-config §1, C-02): a value a later revision
// may define - a slot's attach policy or idle_clock of 2 or more, a lock scheme not in the table, a bind stream kind
// other than 1 / 2 - is refused unsupported with the item's tag as received (they were malformed); a form that is wrong
// stays malformed (retry_ms on a host slot, a lock whose n is not the scheme's length, boot_reset 2+).
// Every item has one closed form (probe.config §1, core §2.3): idle 6 bytes (an input mode carries drive_kind 2 and
// value 0), a slot's boot_reset after attach and the item ending at its lock, a bind's streams 3 bytes each with no
// element length; an item longer than its form is refused unsupported with its tag as received when critical, else not
// applied and listed in the answer's ignored (the endpoint appends it). The state answer has no element lengths and
// reset_at_ns before the tid (§3.3); setStorage(false) leaves save / erase out of the ops and the storage tag out (§2, §4).
#include <stdio.h>

#include <algorithm>
#include <vector>

#include "OepConfig.h"
#include "OepConsole.h"
#include "OepDmConsole.h"
#include "OepEndpoint.h"
#include "OepFixture.h"
#include "OepFrame.h"
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

// A serial port in memory: what the host sent, what the probe answered.
class MemStream final : public Stream {
 public:
  Bytes rx, tx;
  size_t at = 0;
  int available() override { return static_cast<int>(rx.size() - at); }
  int read() override { return at < rx.size() ? rx[at++] : -1; }
  int peek() override { return at < rx.size() ? rx[at] : -1; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override { tx.insert(tx.end(), b, b + n); return n; }
  int availableForWrite() override { return 4096; }
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

static Bytes cat(Bytes a, const Bytes &b) { a.insert(a.end(), b.begin(), b.end()); return a; }
static Bytes u32(uint32_t v) { return {uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)}; }
// An item: tag len(u16) value (core §2.2).
static Bytes item(uint8_t raw, const Bytes &value) {
  return cat({raw, uint8_t(value.size()), uint8_t(value.size() >> 8)}, value);
}
// A slot item: slot 0 on wire fn 1, pins 0 / 1, the attach / idle_clock / lock / boot_reset given (probe.config §1.1:
// slot wire_fn swdio swclk attach boot_reset retry_ms max_speed_hz idle_clock mechanism name_len name lock_len [lock]);
// `extra`: bytes after the lock (a value longer than the form).
static Bytes slotItem(uint8_t attach, uint8_t idle, const Bytes &lock = {}, uint8_t boot_reset = 0, uint32_t retry_ms = 0,
                      bool critical = true, const Bytes &extra = {}) {
  Bytes v = {0, 1, 0, 0, 0, 1, 0, attach, boot_reset};
  v = cat(v, u32(retry_ms));
  v.insert(v.end(), {0, 0, 0, 0, idle, 0xff, 3, 'd', 'u', 't', uint8_t(lock.size())});
  v = cat(cat(v, lock), extra);
  return item(uint8_t(cfg::kTlvItemSlot | (critical ? kTagCritical : 0)), v);
}
static constexpr size_t kSlotMechanismAt = kTlvHeader + 18;   // the mechanism byte in slotItem's bytes
static Result set(Interface &config, const Bytes &items, Bytes &out) {
  out.assign(256, 0);
  const Result r = config.handle(cfg::kOpSet, items.data(), items.size(), out.data(), out.size());
  out.resize(r.length);
  return r;
}
// The current items (get from 0; they fit one answer here) and their hash.
static Bytes getItems(Interface &config, uint32_t *hash = nullptr) {
  Bytes out(512);
  const uint8_t first[2] = {0, 0};
  const Result r = config.handle(cfg::kOpGet, first, sizeof first, out.data(), out.size());
  if (r.resolution != kResolutionCompleted || r.length < 5) return {};
  if (hash) *hash = uint32_t(out[1] | out[2] << 8 | out[3] << 16 | uint32_t(out[4]) << 24);
  return Bytes(out.begin() + 5, out.begin() + r.length);
}
// The value of the item of `tag` whose value starts with `key` in a get's items (found: whether there is one).
static Bytes itemValue(const Bytes &items, uint8_t tag, const Bytes &key, bool *found = nullptr) {
  if (found) *found = false;
  for (size_t at = 0; at + kTlvHeader <= items.size();) {
    const size_t len = items[at + 1] | items[at + 2] << 8;
    const Bytes v(items.begin() + at + kTlvHeader, items.begin() + at + kTlvHeader + len);
    if (items[at] == tag && v.size() >= key.size() && std::equal(key.begin(), key.end(), v.begin())) {
      if (found) *found = true;
      return v;
    }
    at += kTlvHeader + len;
  }
  return {};
}
static bool unsupportedWith(const Result &r, const Bytes &out, uint8_t raw) {
  return r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out.size() >= 1 && out[0] == raw;
}
static bool malformed(const Result &r) { return r.resolution == kResolutionRejected && r.detail == kRejectMalformed; }
static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }

// The host side of a serial port's framing (0x00 <COBS(message + CRC-16)> 0x00): one request (core §4.1: role 0x01,
// corr, fn, op, session_id), the answer message (role, corr, resolution, detail, payload) back; empty when none.
static bool unframe(const Bytes &enc, Bytes &out) {
  out.clear();
  size_t in = 0;
  while (in < enc.size()) {
    const uint8_t code = enc[in++];
    if (code == 0 || in + code - 1 > enc.size()) return false;
    out.insert(out.end(), enc.begin() + in, enc.begin() + in + code - 1);
    in += code - 1;
    if (code != 0xff && in < enc.size()) out.push_back(0);
  }
  if (out.size() < 3) return false;
  const uint16_t got = static_cast<uint16_t>(out[out.size() - 2] | out[out.size() - 1] << 8);
  out.resize(out.size() - 2);
  return crc16Ccitt(out.data(), out.size()) == got;
}
static Bytes exchange(Endpoint &ep, MemStream &s, uint16_t corr, uint16_t fn, uint8_t op, const Bytes &payload,
                      uint32_t session) {
  const Bytes m = cat(cat({0x01, uint8_t(corr), uint8_t(corr >> 8), uint8_t(fn), uint8_t(fn >> 8), op}, u32(session)), payload);
  MemStream f;
  writeCobsFrame(f, m.data(), m.size());
  s.tx.clear();
  s.rx.insert(s.rx.end(), f.tx.begin(), f.tx.end());
  ep.poll();
  if (s.tx.size() < 3 || s.tx.front() != 0 || s.tx.back() != 0) return {};
  Bytes msg;
  return unframe(Bytes(s.tx.begin() + 1, s.tx.end() - 1), msg) ? msg : Bytes{};
}
static bool answeredOk(const Bytes &a) {
  return a.size() >= kResultHeader && a[3] == kResolutionCompleted && a[4] == kOutcomeSuccess;
}
// A describe answer's TLV of `tag` (more(u8), then the TLVs): its value; found whether it was there.
static Bytes describeTlv(const Bytes &tlvs, size_t from, uint8_t tag, bool *found = nullptr) {
  if (found) *found = false;
  for (size_t at = from; at + kTlvHeader <= tlvs.size();) {
    const size_t len = tlvs[at + 1] | tlvs[at + 2] << 8;
    if (tlvs[at] == tag) {
      if (found) *found = true;
      return Bytes(tlvs.begin() + at + kTlvHeader, tlvs.begin() + at + kTlvHeader + len);
    }
    at += kTlvHeader + len;
  }
  return {};
}

int main() {
  static MemStream link;
  static uint8_t rx[512], tx[512];
  static Endpoint ep(link, rx, sizeof rx, tx, sizeof tx, {512, 1024, 2}, Endpoint::kUartBridge);
  static SilentPhy phy;
  static Ch32Dm dm(phy);
  static DebugPort port{dm, 0, 1};
  static WireRvswd wire(port, 0);
  static DmConsole driver(dm, phy);
  static TargetConsoleStream console(port, driver, 0);
  static Binds binds;
  static ProbeConfig config(ep, binds);
  ep.add(wire);      // fn 1
  ep.add(console);   // fn 2
  ep.add(config);    // fn 3
  config.addPlace(wire, console);
  const uint16_t config_fn = 3;
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
  CHECK(malformed(set(config, slotItem(0, 0, {}, 0, 100), out)));
  CHECK(malformed(set(config, slotItem(1, 0, {}, 2), out)));
  CHECK(malformed(set(config, slotItem(0, 0, {}, 1), out)));
  // and malformed before unsupported: an undefined attach with a lock of the wrong length is malformed
  CHECK(malformed(set(config, slotItem(2, 0, lock_short), out)));

  // ---- the slot's closed form (probe.config §1.1): boot_reset right after attach, the item ends at its lock ----
  {
    CHECK(ok(set(config, slotItem(cfg::kSlotAttachAtBoot, 0, {}, cfg::kSlotBootResetRetryWithReset), out)));
    bool found = false;
    const Bytes v = itemValue(getItems(config), cfg::kTlvItemSlot, {0}, &found);
    CHECK(found && v.size() == 24 && v[7] == cfg::kSlotAttachAtBoot && v[8] == cfg::kSlotBootResetRetryWithReset);
    // the form before (boot_reset after the lock): one byte longer than the form - critical, unsupported with the tag
    // as received (not malformed, not read as a boot_reset)
    CHECK(unsupportedWith(set(config, slotItem(1, 0, {}, 0, 0, true, {1}), out), out, slot_raw));
    CHECK(unsupportedWith(set(config, slotItem(0, 0, lock1, 0, 0, true, {0}), out), out, slot_raw));   // after a lock
    // shorter than its counts make it: malformed (a name_len past the end, a lock_len past the end)
    Bytes cut = slotItem(0, 0, lock1);
    cut.resize(cut.size() - 1);
    cut[1] = uint8_t(cut.size() - kTlvHeader);
    CHECK(malformed(set(config, cut, out)));
    CHECK(malformed(set(config, item(slot_raw, {0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 3, 'd'}), out)));
    CHECK(malformed(set(config, item(slot_raw, Bytes(19, 0)), out)));   // the fixed head cut short
  }

  // a bind stream kind other than 1 / 2: unsupported with the item's tag (0.0.28 malformed); a cut element malformed
  const uint8_t bind_raw = cfg::kTlvItemBind | kTagCritical;
  CHECK(ok(set(config, slotItem(0, 0), out)));
  const Bytes mechanism_slot = [] {
    Bytes s = slotItem(0, 0);
    s[kSlotMechanismAt] = reg::target_console::kMechanismSdi;   // a console, so a bind may carry it
    return s;
  }();
  CHECK(ok(set(config, mechanism_slot, out)));
  // port 0, last-reset, one stream of 3 bytes - kind(u8) id(u16), no element length (probe.config §1.2): kind 3, id 0
  Bytes bind = item(bind_raw, {0, 0, 0, 1, 3, 0, 0});
  CHECK(unsupportedWith(set(config, bind, out), out, bind_raw));
  bind[kTlvHeader + 4] = 1;   // kind 1, slot 0: taken
  CHECK(ok(set(config, bind, out)));
  const Bytes cut = item(bind_raw, {0, 0, 0, 1, 1, 0});   // the element's 3 bytes are not all there
  CHECK(malformed(set(config, cut, out)));
  CHECK(malformed(set(config, item(bind_raw, {0, 0, 0, 2, 1, 0, 0}), out)));   // n 2, one element
  // the form before (a len byte per element): longer than n x 3 - critical, unsupported with the tag as received
  CHECK(unsupportedWith(set(config, item(bind_raw, {0, 0, 0, 1, 3, 1, 0, 0}), out), out, bind_raw));

  // core §4.3 (C-21) over the whole set: an earlier item's unknown_function or unsupported does not win over a later
  // item's malformed; an fn named inside an item that does not exist (unknown_function) comes before another item's
  // unsupported, which carries that item's tag as received
  const uint8_t plan_raw = cfg::kTlvItemPlan | kTagCritical;
  const Bytes plan_fn9 = item(plan_raw, {9, 0, 0, 1, 0});   // fn 9: no such fn
  const Bytes plan_fn0 = item(plan_raw, {0, 0, 0, 1, 0});   // fn 0: malformed
  CHECK(malformed(set(config, cat(plan_fn9, plan_fn0), out)));
  CHECK(malformed(set(config, cat(slotItem(2, 0), plan_fn0), out)));   // an unsupported slot before it
  CHECK(malformed(set(config, cat(plan_fn9, slotItem(0, 0, lock_short)), out)));
  Result r = set(config, cat(slotItem(2, 0), plan_fn9), out);
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnknownFunction);
  CHECK(unsupportedWith(set(config, cat(slotItem(2, 0), Bytes{0xB5, 0, 0}), out), out, slot_raw));
  CHECK(unsupportedWith(set(config, cat(Bytes{0xB5, 0, 0}, slotItem(2, 0)), out), out, 0xB5));   // the first one
  // a slot naming a wire fn that does not exist: unknown_function before its undefined attach (unsupported)
  Bytes slot_fn9 = slotItem(2, 0);
  slot_fn9[kTlvHeader + 1] = 9;
  r = set(config, slot_fn9, out);
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnknownFunction);

  // ---- state (probe.config §3.3): more storage_state storage_hash unreadable_reason n_slots slot_state... n_binds
  // bind_state..., no element lengths; slot_state = slot state connection last_try_at_ns reset_at_ns tid_scheme tid_len
  // tid (reset_at_ns before the tid); bind_state = port mode selected flow ----
  {
    const uint8_t first[2] = {0, 0};
    out.assign(128, 0);
    Result st = config.handle(cfg::kOpState, first, sizeof first, out.data(), out.size());
    out.resize(st.length);
    // one slot without a connection (22 bytes, no tid), one bind (4 bytes)
    CHECK(ok(st) && out.size() == 8 + 22 + 1 + 4 && out[0] == 0 && out[7] == 1);
    if (out.size() == 35) {
      CHECK(out[8] == 0 && out[9] == cfg::kSlotStateAbsent && out[10] == 0 && out[11] == 0);
      CHECK(std::all_of(out.begin() + 20, out.begin() + 28, [](uint8_t b) { return b == 0xff; }));   // reset_at_ns: never
      CHECK(out[28] == 0 && out[29] == 0);                     // tid_scheme 0, tid_len 0
      CHECK(out[30] == 1 && out[31] == 0 && out[32] == cfg::kBindModeLastReset && out[33] == 0 && out[34] <= cfg::kBindFlowHeld);
    }
    // the slot's pair connected with a target_id (a stand-in for an attach): the tid after reset_at_ns
    port.connected = true;
    port.number = ResourceNumbers::take(ResourceNumbers::kConnection);
    port.has_tid = true;
    port.tid = 0x04030201u;
    out.assign(128, 0);
    st = config.handle(cfg::kOpState, first, sizeof first, out.data(), out.size());
    out.resize(st.length);
    CHECK(ok(st) && out.size() == 8 + 26 + 1 + 4 && out[7] == 1);
    if (out.size() == 39) {
      CHECK(out[9] == cfg::kSlotStateConnected && uint16_t(out[10] | out[11] << 8) == port.number);
      CHECK(std::all_of(out.begin() + 20, out.begin() + 28, [](uint8_t b) { return b == 0xff; }));
      CHECK(out[28] == reg::common::kTargetIdSchemeWchDmi7f && out[29] == 4 && out[30] == 1 && out[31] == 2 &&
            out[32] == 3 && out[33] == 4);
      CHECK(out[34] == 1 && out[35] == 0);   // n_binds, then the bind's port
    }
    ResourceNumbers::close(port.number);
    port.connected = port.has_tid = false;
    port.tid = 0;
  }

  // ---- a non-critical item longer than its form: not applied (its key neither replaced nor created), listed in the
  // answer's ignored (0x7F len(u16) tags, in request order) that the endpoint appends; the other items are applied
  // (probe.config §1, core §2.3) ----
  {
    uint16_t corr = 1;
    const uint32_t session = 0x51;
    Bytes a = exchange(ep, link, corr++, 0, reg::core::kOpOpen, cat(u32(3000), {0}), session);
    CHECK(answeredOk(a));
    uint32_t before = 0;
    const Bytes items_before = getItems(config, &before);
    // a longer slot (as at boot, another name would show) with a bind that changes the mode to manual
    Bytes longer_slot = slotItem(cfg::kSlotAttachAtBoot, 0, {}, 0, 0, false, {0});
    const Bytes manual = item(bind_raw, {0, cfg::kBindModeManual, 0, 1, 1, 0, 0});
    a = exchange(ep, link, corr++, config_fn, cfg::kOpSet, cat(longer_slot, manual), session);
    uint32_t after = 0;
    const Bytes items_after = getItems(config, &after);
    CHECK(answeredOk(a) && a.size() == kResultHeader + 4 + kTlvHeader + 1);
    if (a.size() == kResultHeader + 8) {
      CHECK(uint32_t(a[5] | a[6] << 8 | a[7] << 16 | uint32_t(a[8]) << 24) == after);   // the hash after the set
      CHECK(a[9] == kTagIgnored && a[10] == 1 && a[11] == 0 && a[12] == cfg::kTlvItemSlot);
    }
    CHECK(itemValue(items_after, cfg::kTlvItemSlot, {0}) == itemValue(items_before, cfg::kTlvItemSlot, {0}));   // slot kept
    const Bytes b = itemValue(items_after, cfg::kTlvItemBind, {0});
    CHECK(b.size() == 7 && b[1] == cfg::kBindModeManual);   // the bind applied
    // a longer item for a key that does not exist is not created: slot 1
    Bytes slot1 = slotItem(0, 0, {}, 0, 0, false, {0, 0});
    slot1[kTlvHeader] = 1;
    a = exchange(ep, link, corr++, config_fn, cfg::kOpSet, slot1, session);
    bool found = true;
    itemValue(getItems(config), cfg::kTlvItemSlot, {1}, &found);
    CHECK(answeredOk(a) && !found && a.size() == kResultHeader + 8 && a[12] == cfg::kTlvItemSlot);
    // two of them: both listed, in request order; nothing changes
    const Bytes longer_bind = item(cfg::kTlvItemBind, {0, cfg::kBindModeLastReset, 0, 1, 1, 0, 0, 9});
    uint32_t hash0 = 0, hash1 = 0;
    getItems(config, &hash0);
    a = exchange(ep, link, corr++, config_fn, cfg::kOpSet, cat(longer_bind, longer_slot), session);
    getItems(config, &hash1);
    CHECK(answeredOk(a) && a.size() == kResultHeader + 4 + kTlvHeader + 2 && hash0 == hash1);
    if (a.size() == kResultHeader + 9) CHECK(a[10] == 2 && a[12] == cfg::kTlvItemBind && a[13] == cfg::kTlvItemSlot);
    // the same item critical: the whole set refused unsupported with the tag as received, the other item not applied
    longer_slot[0] |= kTagCritical;
    const Bytes last_reset = item(bind_raw, {0, cfg::kBindModeLastReset, 0, 1, 1, 0, 0});
    a = exchange(ep, link, corr++, config_fn, cfg::kOpSet, cat(last_reset, longer_slot), session);
    CHECK(a.size() == kResultHeader + 1 && a[3] == kResolutionRejected && a[4] == kRejectUnsupported && a[5] == slot_raw);
    CHECK(itemValue(getItems(config), cfg::kTlvItemBind, {0})[1] == cfg::kBindModeManual);
    CHECK(answeredOk(exchange(ep, link, corr++, 0, reg::core::kOpEnd, {}, session)));
  }

  {   // a saved bind whose port is not a serial port of this firmware (probe.config §2): unreadable reason 2, not 3.
      // The settings above (a slot, a bind on port 0) saved on `ep` (port 0 a UART bridge), read on a firmware with the
      // same interfaces whose port 0 is TCP.
    out.assign(64, 0);
    CHECK(ok(config.handle(cfg::kOpSave, nullptr, 0, out.data(), out.size())));
    static NullStream s3;
    static uint8_t rx3[512], tx3[512];
    static Endpoint ep3(s3, rx3, sizeof rx3, tx3, sizeof tx3, {512, 1024, 2}, Endpoint::kTcp);
    static Binds binds3;
    static ProbeConfig other(ep3, binds3);
    ep3.add(wire);
    ep3.add(console);
    ep3.add(other);
    other.addPlace(wire, console);
    other.load();
    other.applySaved();
    const uint8_t first[2] = {0, 0};
    out.assign(64, 0);
    const Result st = other.handle(cfg::kOpState, first, sizeof first, out.data(), out.size());
    CHECK(ok(st) && st.length == 9 && out[1] == cfg::kStorageStateUnreadable && out[6] == cfg::kStorageUnreadableInterface &&
          out[7] == 0 && out[8] == 0);   // nothing applied: no slot, no bind
  }

  {   // label (probe.config §1): text 1-32 bytes of valid UTF-8 without C0 controls / 0x7F, else malformed; a channel
      // not below channels or reserved (not in the pin table) unsupported with the item's tag
    static MemStream s2;
    static uint8_t rx2[512], tx2[512];
    static Endpoint ep2(s2, rx2, sizeof rx2, tx2, sizeof tx2, {512, 1024, 2}, Endpoint::kUartBridge);
    static Binds binds2;
    static ProbeConfig pinned(ep2, binds2);
    static PinTable pins(uint64_t{0xff});   // channels 0-7
    ep2.add(pinned);                         // fn 1
    pinned.setPins(&pins);
    const uint8_t label_raw = cfg::kTlvItemLabel | kTagCritical;
    auto label = [&](uint16_t channel, const Bytes &text) {
      return item(label_raw, cat({uint8_t(channel), uint8_t(channel >> 8)}, text));
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

    // idle (probe.config §1): channel(u16) mode(u8) drive_kind(u8) drive_value(u16), always 6 bytes; drive_kind 2 is the
    // default with value 0, which an input idle (mode 0-2) carries; core §4.3 "Contradictions and undefined values": an
    // undefined mode (5+) or kind (3+) is unsupported and the rules that read it are not checked
    const uint8_t idle_raw = cfg::kTlvItemIdle | kTagCritical;
    auto idle = [&](uint16_t channel, uint8_t mode, uint8_t kind, uint16_t value, bool critical = true) {
      return item(uint8_t(cfg::kTlvItemIdle | (critical ? kTagCritical : 0)),
                  {uint8_t(channel), uint8_t(channel >> 8), mode, kind, uint8_t(value), uint8_t(value >> 8)});
    };
    CHECK(ok(set(pinned, idle(1, cfg::kIdleModeHiZ, 2, 0), out)));
    CHECK(ok(set(pinned, idle(1, cfg::kIdleModePullUp, 2, 0), out)));
    CHECK(ok(set(pinned, idle(2, cfg::kIdleModeOutputLow, 2, 0), out)));   // an output at the default strength
    CHECK(ok(set(pinned, idle(2, cfg::kIdleModeOutputHigh, 1, 8), out)));  // at most 8 mA (no drive_levels: kept)
    CHECK(ok(set(pinned, idle(2, cfg::kIdleModeOutputLow, 0, 1), out)));   // level 1
    CHECK(malformed(set(pinned, idle(1, cfg::kIdleModeHiZ, 0, 0), out)));        // an input idle with a drive
    CHECK(malformed(set(pinned, idle(1, cfg::kIdleModePullDown, 1, 4), out)));
    CHECK(malformed(set(pinned, idle(1, cfg::kIdleModeHiZ, 2, 1), out)));        // kind 2 with a value
    CHECK(malformed(set(pinned, idle(2, cfg::kIdleModeOutputHigh, 2, 1), out)));
    CHECK(unsupportedWith(set(pinned, idle(2, cfg::kIdleModeOutputHigh, 3, 0), out), out, idle_raw));   // kind 3
    CHECK(unsupportedWith(set(pinned, idle(1, cfg::kIdleModeHiZ, 0xff, 7), out), out, idle_raw));        // not read
    CHECK(unsupportedWith(set(pinned, idle(1, 5, 2, 0), out), out, idle_raw));                          // mode 5
    CHECK(unsupportedWith(set(pinned, idle(1, 5, 0, 0), out), out, idle_raw));   // not malformed for the drive
    // shorter than 6 bytes: malformed, before an undefined mode (the length first); the 3-byte form before too
    CHECK(malformed(set(pinned, item(idle_raw, {1, 0, 5}), out)));
    CHECK(malformed(set(pinned, item(idle_raw, {1, 0, 0, 2, 0}), out)));
    CHECK(malformed(set(pinned, item(idle_raw, {1, 0, 5, 0, 0}), out)));
    // longer than 6 bytes: critical, unsupported with the tag as received
    CHECK(unsupportedWith(set(pinned, item(idle_raw, {1, 0, 0, 2, 0, 0, 0}), out), out, idle_raw));
    {   // non-critical: not applied, listed in ignored (through the endpoint)
      Bytes a = exchange(ep2, s2, 1, 0, reg::core::kOpOpen, cat(u32(3000), {0}), 0x52);
      CHECK(answeredOk(a));
      const Bytes before = itemValue(getItems(pinned), cfg::kTlvItemIdle, {1, 0});
      a = exchange(ep2, s2, 2, 1, cfg::kOpSet, item(cfg::kTlvItemIdle, {1, 0, cfg::kIdleModePullDown, 2, 0, 0, 0}), 0x52);
      CHECK(answeredOk(a) && a.size() == kResultHeader + 8 && a[9] == kTagIgnored && a[10] == 1 && a[11] == 0 &&
            a[12] == cfg::kTlvItemIdle);
      CHECK(itemValue(getItems(pinned), cfg::kTlvItemIdle, {1, 0}) == before && before.size() == 6 &&
            before[2] == cfg::kIdleModePullUp);
      a = exchange(ep2, s2, 3, 1, cfg::kOpSet, idle(1, cfg::kIdleModePullDown, 2, 0, false), 0x52);   // the form: applied
      CHECK(answeredOk(a) && a.size() == kResultHeader + 4);
      CHECK(itemValue(getItems(pinned), cfg::kTlvItemIdle, {1, 0}) == Bytes({1, 0, cfg::kIdleModePullDown, 2, 0, 0}));
      CHECK(answeredOk(exchange(ep2, s2, 4, 0, reg::core::kOpEnd, {}, 0x52)));
    }

    // slots_max 0 (no wire place): describe's items leave slot out, and a slot item is unsupported with its tag
    uint8_t d[64];
    const size_t dn = pinned.describe(d, sizeof d);
    bool listed = false, slots_max0 = false;
    for (size_t at = 0; at + kTlvHeader <= dn; at += kTlvHeader + (d[at + 1] | d[at + 2] << 8)) {
      const size_t len = d[at + 1] | d[at + 2] << 8;
      if (d[at] == cfg::kTlvDescribeItems)
        for (size_t k = 0; k < len; ++k) listed |= d[at + kTlvHeader + k] == cfg::kTlvItemSlot;
      if (d[at] == cfg::kTlvDescribeSlotsMax) slots_max0 = len == 1 && d[at + kTlvHeader] == 0;
    }
    CHECK(dn > 0 && !listed && slots_max0);
    CHECK(unsupportedWith(set(pinned, slotItem(0, 0), out), out, slot_raw));
    CHECK(unsupportedWith(set(pinned, item(slot_raw, {0}), out), out, slot_raw));   // its form is not looked at
    // a channel without pulls (setNoPull): an idle with mode 1 / 2 unsupported with the item's tag, Hi-Z taken
    pins.setNoPull(1ull << 6);
    CHECK(unsupportedWith(set(pinned, idle(6, cfg::kIdleModePullUp, 2, 0), out), out, idle_raw));
    CHECK(unsupportedWith(set(pinned, idle(6, cfg::kIdleModePullDown, 2, 0), out), out, idle_raw));
    CHECK(ok(set(pinned, idle(6, cfg::kIdleModeHiZ, 2, 0), out)) && ok(set(pinned, idle(5, cfg::kIdleModePullUp, 2, 0), out)));
    // a probe without a pin table has no channels to name: label is not declared (unsupported, its tag)
    CHECK(unsupportedWith(set(config, label(1, {'x'}), out), out, label_raw));
  }

  // ---- setStorage(false) (probe.config §2, §4, core §1.2): save and erase are not offered - not in the ops tag the
  // endpoint writes first in the describe, unknown_operation - and the describe has no storage tag; with storage, both
  // ----
  {
    static MemStream s4;
    static uint8_t rx4[512], tx4[512];
    static Endpoint ep4(s4, rx4, sizeof rx4, tx4, sizeof tx4, {512, 1024, 2}, Endpoint::kUartBridge);
    static Binds binds4;
    static ProbeConfig nostore(ep4, binds4);
    nostore.setStorage(false);
    ep4.add(nostore);   // fn 1
    CHECK(!nostore.offers(cfg::kOpSave) && !nostore.offers(cfg::kOpErase));
    CHECK(nostore.offers(cfg::kOpGet) && nostore.offers(cfg::kOpSet) && nostore.offers(cfg::kOpUnset) &&
          nostore.offers(cfg::kOpState));
    CHECK(config.offers(cfg::kOpSave) && config.offers(cfg::kOpErase));
    uint8_t d[64];
    bool storage = true, items = false;
    const Bytes own(d, d + nostore.describe(d, sizeof d));
    describeTlv(own, 0, cfg::kTlvDescribeStorage, &storage);
    describeTlv(own, 0, cfg::kTlvDescribeItems, &items);
    CHECK(!storage && items);
    // through the endpoint: the ops tag first (base 1: get set unset state = bits 0 1 4 5), no storage tag
    Bytes a = exchange(ep4, s4, 1, 0, reg::core::kOpDescribe, {1, 0, 0, 0}, 0);
    CHECK(answeredOk(a) && a.size() > kResultHeader + 1 + kTlvHeader + 2);
    if (a.size() > kResultHeader + 1 + kTlvHeader + 2) {
      CHECK(a[6] == kTagOps && a[7] == 2 && a[8] == 0 && a[9] == cfg::kOpGet && a[10] == 0x33);
      describeTlv(a, kResultHeader + 1, cfg::kTlvDescribeStorage, &storage);
      CHECK(!storage);
    }
    CHECK(answeredOk(exchange(ep4, s4, 2, 0, reg::core::kOpOpen, cat(u32(3000), {0}), 0x53)));
    a = exchange(ep4, s4, 3, 1, cfg::kOpSave, {}, 0x53);
    CHECK(a.size() == kResultHeader && a[3] == kResolutionRejected && a[4] == kRejectUnknownOperation);
    a = exchange(ep4, s4, 4, 1, cfg::kOpErase, {}, 0x53);
    CHECK(a.size() == kResultHeader && a[3] == kResolutionRejected && a[4] == kRejectUnknownOperation);
    CHECK(answeredOk(exchange(ep4, s4, 5, 0, reg::core::kOpEnd, {}, 0x53)));
    // with storage (`config` on `ep`): save and erase in the ops (bits 0-5), the storage tag max_bytes
    a = exchange(ep, link, 100, 0, reg::core::kOpDescribe, {uint8_t(config_fn), 0, 0, 0}, 0);
    CHECK(answeredOk(a) && a.size() > kResultHeader + 1 + kTlvHeader + 2);
    if (a.size() > kResultHeader + 1 + kTlvHeader + 2) {
      CHECK(a[6] == kTagOps && a[7] == 2 && a[8] == 0 && a[9] == cfg::kOpGet && a[10] == 0x3f);
      const Bytes v = describeTlv(a, kResultHeader + 1, cfg::kTlvDescribeStorage, &storage);
      CHECK(storage && v.size() == 4 && uint32_t(v[0] | v[1] << 8 | v[2] << 16 | uint32_t(v[3]) << 24) == ProbeConfig::kMaxItems);
    }
  }

  {   // probe.config §2 with the endpoint's own interfaces (oep.probe.plan, oep.probe.restart): listed after the sketch's,
      // in the same order at every boot of a firmware; saved settings name their interfaces by (name, instance, revision)
      // and are renumbered on a firmware that added an interface before them, which moves the endpoint's two as well.
    static NullStream sa, sb, sc;
    static uint8_t rxa[512], txa[512], rxb[512], txb[512], rxc[512], txc[512];
    static PinTable pins(uint64_t{0xff});
    auto restart = []() {};
    // firmware A: gpio fn 1, config fn 2, then oep.probe.plan fn 3 and oep.probe.restart fn 4
    static Endpoint epa(sa, rxa, sizeof rxa, txa, sizeof txa, {512, 1024, 2}, Endpoint::kUartBridge);
    static FixtureGpio gpioa(pins, 0);
    static Binds bindsa;
    static ProbeConfig cfga(epa, bindsa);
    epa.add(gpioa);
    epa.add(cfga);
    cfga.setPins(&pins);
    epa.setRestart(restart, 1500);
    CHECK(epa.planFn() == 3 && epa.restartFn() == 4);
    epa.poll();   // the list fixed
    CHECK(epa.planFn() == 3 && epa.restartFn() == 4 && epa.instanceOf(3) == 0 && epa.instanceOf(4) == 0);
    CHECK(strcmp(epa.interfaceAt(3)->name(), "oep.probe.plan") == 0 && strcmp(epa.interfaceAt(4)->name(), "oep.probe.restart") == 0);
    const Bytes plan_item = item(cfg::kTlvItemPlan | kTagCritical, {1, 0, reg::fixture_gpio::kRoleLine, 5, 0});
    CHECK(ok(set(cfga, plan_item, out)));
    out.assign(64, 0);
    CHECK(ok(cfga.handle(cfg::kOpSave, nullptr, 0, out.data(), out.size())));
    cfga.handle(cfg::kOpUnset, nullptr, 0, out.data(), out.size());
    // the same firmware booted again: the same fns and instances, the saved plan on fn 1
    static Endpoint epc(sc, rxc, sizeof rxc, txc, sizeof txc, {512, 1024, 2}, Endpoint::kUartBridge);
    static PinTable pinsc(uint64_t{0xff});
    static FixtureGpio gpioc(pinsc, 0);
    static Binds bindsc;
    static ProbeConfig cfgc(epc, bindsc);
    epc.add(gpioc);
    epc.add(cfgc);
    cfgc.setPins(&pinsc);
    epc.setRestart(restart, 1500);
    cfgc.load();
    cfgc.applySaved();
    epc.poll();
    RoleAssignment now[4];
    CHECK(epc.plan(now, 4) == 1 && now[0].function == 1 && now[0].channel == 5);
    CHECK(epc.planFn() == 3 && epc.restartFn() == 4);
    for (uint16_t f = 1; f <= 4; ++f)
      CHECK(strcmp(epc.interfaceAt(f)->name(), epa.interfaceAt(f)->name()) == 0 && epc.instanceOf(f) == epa.instanceOf(f));
    // firmware B adds oep.probe.link first: gpio fn 2, config fn 3, oep.probe.plan fn 4, oep.probe.restart fn 5; the saved
    // plan item (fn 1 = oep.fixture.gpio#0 rev 1) lands on fn 2
    static Endpoint epb(sb, rxb, sizeof rxb, txb, sizeof txb, {512, 1024, 2}, Endpoint::kUartBridge);
    static PinTable pinsb(uint64_t{0xff});
    static Link linkb(epb);
    static FixtureGpio gpiob(pinsb, 0);
    static Binds bindsb;
    static ProbeConfig cfgb(epb, bindsb);
    epb.add(linkb);
    epb.add(gpiob);
    epb.add(cfgb);
    cfgb.setPins(&pinsb);
    epb.setRestart(restart, 1500);
    cfgb.load();
    cfgb.applySaved();
    epb.poll();
    CHECK(epb.plan(now, 4) == 1 && now[0].function == 2 && now[0].channel == 5);
    CHECK(epb.planFn() == 4 && epb.restartFn() == 5);
    const uint8_t first[2] = {0, 0};
    out.assign(64, 0);
    const Result st = cfgb.handle(cfg::kOpState, first, sizeof first, out.data(), out.size());
    CHECK(ok(st) && out[1] == cfg::kStorageStateApplied);
    // a plan item naming oep.probe.plan's fn (no plan role): refused unsupported, nothing saved changes
    CHECK(unsupportedWith(set(cfgb, item(cfg::kTlvItemPlan | kTagCritical, {4, 0, 1, 6, 0}), out), out,
                          cfg::kTlvItemPlan | kTagCritical));
  }

  {   // the at-boot attach's gate (setAttachGate) and a boot that skips the saved at-boot slots (skipBootAttach): an
      // at-boot slot saved, then read by firmwares with the same interfaces; the slot's last_try_at_ns (state, §3.3)
      // tells whether the probe tried its automatic attach (the wire never answers here)
    static NullStream s4, s5, s6;
    static uint8_t rx4[512], tx4[512], rx5[512], tx5[512], rx6[512], tx6[512];
    static Endpoint ep4(s4, rx4, sizeof rx4, tx4, sizeof tx4, {512, 1024, 2}, Endpoint::kUartBridge);
    static Endpoint ep5(s5, rx5, sizeof rx5, tx5, sizeof tx5, {512, 1024, 2}, Endpoint::kUartBridge);
    static Endpoint ep6(s6, rx6, sizeof rx6, tx6, sizeof tx6, {512, 1024, 2}, Endpoint::kUartBridge);
    static Binds binds4, binds5, binds6;
    static ProbeConfig cfg4(ep4, binds4), cfg5(ep5, binds5), cfg6(ep6, binds6);
    Endpoint *eps[] = {&ep4, &ep5, &ep6};
    ProbeConfig *cfgs[] = {&cfg4, &cfg5, &cfg6};
    for (int k = 0; k < 3; ++k) {
      eps[k]->add(wire);
      eps[k]->add(console);
      eps[k]->add(*cfgs[k]);
      cfgs[k]->addPlace(wire, console);
    }
    auto tried = [&](ProbeConfig &c) {   // slot 0's last_try_at_ns is not all ones
      const uint8_t first[2] = {0, 0};
      Bytes st(64, 0);
      const Result r = c.handle(cfg::kOpState, first, sizeof first, st.data(), st.size());
      if (!ok(r) || r.length < 28 || st[7] != 1) return false;
      return !std::all_of(st.begin() + 12, st.begin() + 20, [](uint8_t b) { return b == 0xff; });
    };
    CHECK(ok(set(cfg4, slotItem(cfg::kSlotAttachAtBoot, 0), out)));
    CHECK(tried(cfg4));   // a set at-boot slot: attached at once (no gate)
    out.assign(64, 0);
    CHECK(ok(cfg4.handle(cfg::kOpSave, nullptr, 0, out.data(), out.size())));
    // the gate closed: applySaved and the polls after it do not attach; once open, the next poll does
    static bool gate = false;
    cfg5.setAttachGate([]() { return gate; });
    cfg5.load();
    cfg5.applySaved();
    for (int i = 0; i < 3; ++i) cfg5.poll();
    CHECK(!tried(cfg5));
    gate = true;
    cfg5.poll();
    CHECK(tried(cfg5));
    // a boot that skips them: never tried by the probe (state 1, last_try_at_ns all ones); a set of the slot attaches
    cfg6.skipBootAttach();
    cfg6.load();
    cfg6.applySaved();
    for (int i = 0; i < 3; ++i) cfg6.poll();
    CHECK(cfg6.bootAttachSkipped() && !tried(cfg6));
    CHECK(ok(set(cfg6, slotItem(cfg::kSlotAttachAtBoot, 0, {}, 0, 100), out)));
    CHECK(tried(cfg6));
  }

  printf("config: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
