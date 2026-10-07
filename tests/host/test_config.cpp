// Host tests: oep.probe.config's items (oep-if-probe-config §1): every item has one length - any other is malformed,
// critical or not (no item is ever skipped); a value this probe cannot take is refused unsupported with the item's tag
// as received; a set is checked whole before anything changes. The slot item (ending at its name; retry_ms not read
// on a host slot), the bind of one stream (port kind id, 4 bytes), the idle of 4 bytes (drive read only for an output:
// past drive_levels, or anything but 0xFF without them, unsupported), the label's length rule. The state answer
// (§3.3): slot_state slot state connection last_try_at_ns, bind_state port flow; storage_hash; the hash changing with
// the settings; setStorage(false) leaving save / erase out of the ops and the storage tag out (§2, §4); disable's
// cause 5; the saved settings renumbered; the at-boot attach's gate and a boot that skips it.
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <string>
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
// A slot item (probe.config §1.1: slot wire_fn swdio swclk attach retry_ms max_speed_hz idle_clock mechanism name_len
// name): slot `n` on wire fn 1, pins 0 / 1, no console unless `mechanism` says; `extra`: bytes after the name.
static Bytes slotValue(uint8_t attach, uint8_t idle, uint32_t retry_ms = 0, uint8_t mechanism = 0xff, const char *name = "dut",
                       uint8_t n = 0, const Bytes &extra = {}) {
  Bytes v = {n, 1, 0, 0, 0, 1, 0, attach};
  v = cat(v, u32(retry_ms));
  v = cat(v, u32(0));
  v.insert(v.end(), {idle, mechanism, uint8_t(strlen(name))});
  v.insert(v.end(), name, name + strlen(name));
  return cat(v, extra);
}
constexpr size_t kSlotAttachAt = 7;   // the attach byte in slotValue's bytes
static Bytes slotItem(uint8_t attach, uint8_t idle, uint32_t retry_ms = 0, bool critical = true) {
  return item(uint8_t(cfg::kTlvItemSlot | (critical ? kTagCritical : 0)), slotValue(attach, idle, retry_ms));
}
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
// rejected unavailable whose payload has cause `cause` (and channel `channel`, 0xFFFF: not looked at)
static bool unavailableWith(const Result &r, const Bytes &out, uint8_t cause, uint16_t channel = 0xFFFF) {
  if (r.resolution != kResolutionRejected || r.detail != kRejectUnavailable) return false;
  bool cause_ok = false, channel_ok = channel == 0xFFFF;
  for (size_t at = 0; at + kTlvHeader <= out.size();) {
    const size_t len = out[at + 1] | out[at + 2] << 8;
    if (at + kTlvHeader + len > out.size()) break;
    if (out[at] == reg::core::kTlvUnavailablePayloadCause && len == 1) cause_ok = out[at + kTlvHeader] == cause;
    if (out[at] == reg::core::kTlvUnavailablePayloadChannel && len == 2)
      channel_ok = channel_ok || (out[at + kTlvHeader] | out[at + kTlvHeader + 1] << 8) == channel;
    at += kTlvHeader + len;
  }
  return cause_ok && channel_ok;
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

static uint32_t hashIn(const Bytes &out) {
  return out.size() >= 4 ? uint32_t(out[0] | out[1] << 8 | out[2] << 16 | uint32_t(out[3]) << 24) : 0;
}
// The state answer from (0, 0).
static Bytes stateOf(Interface &config) {
  const uint8_t first[2] = {0, 0};
  Bytes out(128, 0);
  const Result r = config.handle(cfg::kOpState, first, sizeof first, out.data(), out.size());
  out.resize(ok(r) ? r.length : 0);
  return out;
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

  // ---- the slot (probe.config §1.1) ----
  CHECK(ok(set(config, slotItem(cfg::kSlotAttachHost, 0), out)));
  CHECK(ok(set(config, slotItem(cfg::kSlotAttachHost, 0, 100), out)));            // retry_ms: not read on a host slot
  CHECK(unsupportedWith(set(config, slotItem(2, 0), out), out, slot_raw));         // an attach policy not defined
  CHECK(unsupportedWith(set(config, slotItem(0xff, 0), out), out, slot_raw));
  CHECK(unsupportedWith(set(config, slotItem(0, 2), out), out, slot_raw));         // idle_clock 2
  CHECK(unsupportedWith(set(config, slotItem(0, 2, 0, false), out), out, cfg::kTlvItemSlot));   // the tag as received
  {
    bool found = false;
    const Bytes v = itemValue(getItems(config), cfg::kTlvItemSlot, {0}, &found);
    CHECK(found && v.size() == 19 + 3 && v[7] == cfg::kSlotAttachHost && v[8] == 100 && v[18] == 3 && v[19] == 'd');
    // any other length is malformed, critical or not: a byte after the name (as the forms before had)
    CHECK(malformed(set(config, item(slot_raw, slotValue(0, 0, 0, 0xff, "dut", 0, {0})), out)));
    CHECK(malformed(set(config, item(cfg::kTlvItemSlot, slotValue(0, 0, 0, 0xff, "dut", 0, {0})), out)));
    Bytes cut = slotValue(0, 0);
    cut.pop_back();   // name_len past the end
    CHECK(malformed(set(config, item(slot_raw, cut), out)));
    CHECK(malformed(set(config, item(slot_raw, Bytes(18, 0)), out)));   // the fixed head cut short
    // the name: 1-32 of a-z 0-9 - _, unique; the slot number below slots_max (pins the host chooses, 0-3, for a second pair)
    port.pin_choice = 0x0f;
    CHECK(malformed(set(config, item(slot_raw, slotValue(0, 0, 0, 0xff, "Dut")), out)));
    CHECK(malformed(set(config, item(slot_raw, slotValue(0, 0, 0, 0xff, "")), out)));
    CHECK(ok(set(config, item(slot_raw, slotValue(0, 0, 0, 0xff, "a-b_9")), out)));
    CHECK(malformed(set(config, item(slot_raw, slotValue(0, 0, 0, 0xff, "dut", uint8_t(ProbeConfig::kMaxSlots))), out)));
    Bytes other = slotValue(0, 0, 0, 0xff, "a-b_9", 1);   // slot 1, another pair, the same name
    other[3] = 2;
    other[5] = 3;
    CHECK(malformed(set(config, item(slot_raw, other), out)));
    other = slotValue(0, 0, 0, 0xff, "dut", 1);           // slot 1 on slot 0's pair
    CHECK(malformed(set(config, item(slot_raw, other), out)));
    // the wire: one that does not exist is unknown_function; one slots do not ride on (the console, config) unsupported
    Bytes fn9 = slotValue(0, 0);
    fn9[1] = 9;
    Result r = set(config, item(slot_raw, fn9), out);
    CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnknownFunction);
    Bytes fn2 = slotValue(0, 0);
    fn2[1] = 2;
    CHECK(unsupportedWith(set(config, item(slot_raw, fn2), out), out, slot_raw));
    fn2[1] = uint8_t(config_fn);
    CHECK(unsupportedWith(set(config, item(slot_raw, fn2), out), out, slot_raw));
    // two at-boot slots on one wire (max_connections 1): unavailable
    other = slotValue(cfg::kSlotAttachAtBoot, 0, 0, 0xff, "two", 1);
    other[3] = 2;
    other[5] = 3;
    CHECK(ok(set(config, item(slot_raw, slotValue(cfg::kSlotAttachAtBoot, 0, 0, 0xff, "dut")), out)));
    CHECK(unavailableWith(set(config, item(slot_raw, other), out), out, reg::core::kUnavailableCauseLimit));
    other[kSlotAttachAt] = cfg::kSlotAttachHost;   // a host slot on that pair: taken
    CHECK(ok(set(config, item(slot_raw, other), out)));
    const uint8_t unset1[] = {1, 1, cfg::kTlvItemSlot, 1};
    out.assign(64, 0);
    CHECK(ok(config.handle(cfg::kOpUnset, unset1, sizeof unset1, out.data(), out.size())));
    port.pin_choice = 0;
    CHECK(ok(set(config, slotItem(cfg::kSlotAttachHost, 0), out)));
  }

  // ---- the bind (probe.config §1.2): port kind id, 4 bytes ----
  const uint8_t bind_raw = cfg::kTlvItemBind | kTagCritical;
  CHECK(ok(set(config, item(slot_raw, slotValue(0, 0, 0, reg::target_console::kMechanismSdi)), out)));   // a console
  CHECK(unsupportedWith(set(config, item(bind_raw, {0, 3, 0, 0}), out), out, bind_raw));            // kind 3
  CHECK(unsupportedWith(set(config, item(cfg::kTlvItemBind, {0, 3, 0, 0}), out), out, cfg::kTlvItemBind));
  CHECK(ok(set(config, item(bind_raw, {0, Binds::kSlotConsole, 0, 0}), out)));
  CHECK(binds.spec(0).set && binds.spec(0).source.kind == Binds::kSlotConsole && binds.spec(0).source.id == 0 &&
        binds.spec(0).source.stream == &console);
  CHECK(malformed(set(config, item(bind_raw, {0, 1, 0}), out)));                         // 3 bytes
  CHECK(malformed(set(config, item(bind_raw, {0, 0, 0, 1, 1, 0, 0}), out)));             // the form before (mode, n)
  CHECK(malformed(set(config, item(cfg::kTlvItemBind, {0, 1, 0, 0, 0}), out)));          // longer, not critical
  CHECK(malformed(set(config, item(bind_raw, {0, 1, 2, 0}), out)));                      // slot 2: not there
  CHECK(unsupportedWith(set(config, item(bind_raw, {1, 1, 0, 0}), out), out, bind_raw));  // port 1: not a serial port
  Result r = set(config, item(bind_raw, {0, Binds::kFixtureUart, 9, 0}), out);            // fn 9: none
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnknownFunction);
  CHECK(unsupportedWith(set(config, item(bind_raw, {0, Binds::kFixtureUart, 1, 0}), out), out, bind_raw));   // a wire
  // a slot without a console cannot be carried; and the bound slot cannot lose its console
  CHECK(malformed(set(config, slotItem(0, 0), out)));
  CHECK(binds.spec(0).set && binds.spec(0).source.id == 0);

  // ---- a set is all or nothing, one key once; the hash follows the settings (§2) ----
  {
    uint32_t before = 0;
    const Bytes items_before = getItems(config, &before);
    const Bytes rename = item(slot_raw, slotValue(0, 0, 0, reg::target_console::kMechanismSdi, "renamed"));
    CHECK(malformed(set(config, cat(rename, item(bind_raw, {0, 1, 0})), out)));   // a bad item after a good one
    uint32_t h = 0;
    CHECK(getItems(config, &h) == items_before && h == before);
    CHECK(malformed(set(config, cat(rename, rename), out)));                      // the same key twice
    CHECK(ok(set(config, rename, out)) && out.size() == 4);
    uint32_t after = 0;
    getItems(config, &after);
    CHECK(after != before && hashIn(out) == after);
    CHECK(ok(set(config, rename, out)) && hashIn(out) == after);   // the same settings: the same hash
  }

  // ---- unset (§2): the key's length is the tag's ----
  {
    out.assign(64, 0);
    const uint8_t wrong[] = {1, 2, cfg::kTlvItemBind, 0, 0};   // a bind key of 2 bytes
    CHECK(malformed(config.handle(cfg::kOpUnset, wrong, sizeof wrong, out.data(), out.size())));
    const uint8_t undeclared[] = {1, 1, 0x30, 0};
    CHECK(unsupportedWith([&] { Result u = config.handle(cfg::kOpUnset, undeclared, sizeof undeclared, out.data(), out.size());
                                out.resize(u.length); return u; }(), out, 0x30));
    out.assign(64, 0);
    const uint8_t nothing[] = {1, 1, cfg::kTlvItemBind, 1};   // no bind on port 1: nothing to do
    CHECK(ok(config.handle(cfg::kOpUnset, nothing, sizeof nothing, out.data(), out.size())));
  }

  // ---- state (§3.3): more storage_state storage_hash unreadable_reason n_slots slot_state... n_binds bind_state...;
  // slot_state = slot state connection last_try_at_ns, bind_state = port flow ----
  {
    Bytes st = stateOf(config);
    CHECK(st.size() == 8 + 12 + 1 + 2 && st[0] == 0 && st[7] == 1);
    if (st.size() == 23) {
      CHECK(st[8] == 0 && st[9] == cfg::kSlotStateAbsent && st[10] == 0 && st[11] == 0);
      CHECK(std::all_of(st.begin() + 12, st.begin() + 20, [](uint8_t b) { return b == 0xff; }));   // never tried
      CHECK(st[20] == 1 && st[21] == 0 && st[22] <= cfg::kBindFlowHeld);
    }
    // the slot's pair connected (a stand-in for an attach, by anyone): state 0 with its connection number
    port.connected = true;
    port.number = ResourceNumbers::take(ResourceNumbers::kConnection);
    port.has_tid = true;
    port.tid = 0x04030201u;
    st = stateOf(config);
    CHECK(st.size() == 23 && st[9] == cfg::kSlotStateConnected && uint16_t(st[10] | st[11] << 8) == port.number);
    ResourceNumbers::close(port.number);
    port.connected = port.has_tid = false;
    port.tid = 0;
    st = stateOf(config);
    CHECK(st.size() == 23 && st[9] == cfg::kSlotStateAbsent);
  }

  // ---- through the endpoint: a non-critical item longer than its form is malformed, nothing applied, no tail ----
  {
    uint16_t corr = 1;
    const uint32_t session = 0x51;
    Bytes a = exchange(ep, link, corr++, 0, reg::core::kOpOpen, cat(u32(3000), {0}), session);
    CHECK(answeredOk(a));
    const Bytes items_before = getItems(config);
    a = exchange(ep, link, corr++, config_fn, cfg::kOpSet,
                 cat(item(cfg::kTlvItemSlot, slotValue(0, 0, 0, 0, "x", 0, {0})), item(cfg::kTlvItemBind, {0, 1, 0, 0})),
                 session);
    CHECK(a.size() >= kResultHeader && a[3] == kResolutionRejected && a[4] == kRejectMalformed);
    CHECK(getItems(config) == items_before);
    a = exchange(ep, link, corr++, config_fn, cfg::kOpSet, item(cfg::kTlvItemBind, {0, 1, 0, 0}), session);
    CHECK(answeredOk(a) && a.size() == kResultHeader + 4);   // hash(u32) and nothing after it
    CHECK(answeredOk(exchange(ep, link, corr++, 0, reg::core::kOpEnd, {}, session)));
  }

  {   // storage_hash (§3.3): the hash when the saved settings were made current, equal to get's while unchanged
    out.assign(64, 0);
    CHECK(ok(config.handle(cfg::kOpSave, nullptr, 0, out.data(), out.size())));
    uint32_t h = 0;
    getItems(config, &h);
    Bytes st = stateOf(config);
    CHECK(st.size() >= 8 && st[1] == cfg::kStorageStateApplied && hashIn(Bytes(st.begin() + 2, st.begin() + 6)) == h);
    CHECK(ok(set(config, item(slot_raw, slotValue(0, 0, 0, reg::target_console::kMechanismSdi, "changed")), out)));
    st = stateOf(config);
    uint32_t h2 = 0;
    getItems(config, &h2);
    CHECK(st.size() >= 8 && hashIn(Bytes(st.begin() + 2, st.begin() + 6)) == h && h2 != h);
    CHECK(ok(set(config, item(slot_raw, slotValue(0, 0, 0, reg::target_console::kMechanismSdi, "renamed")), out)));
  }

  {   // a saved bind whose port is not a serial port of this firmware (probe.config §2): unreadable reason 2, not 3.
      // The settings above (a slot, a bind on port 0) saved on `ep` (port 0 a UART bridge), read on a firmware with the
      // same interfaces whose port 0 is TCP.
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
    const Bytes st = stateOf(other);
    CHECK(st.size() == 9 && st[1] == cfg::kStorageStateUnreadable && st[6] == cfg::kStorageUnreadableInterface &&
          st[7] == 0 && st[8] == 0);   // nothing applied: no slot, no bind
    // the same settings on the same firmware: applied, storage_hash = get's hash
    static NullStream s3b;
    static uint8_t rx3b[512], tx3b[512];
    static Endpoint ep3b(s3b, rx3b, sizeof rx3b, tx3b, sizeof tx3b, {512, 1024, 2}, Endpoint::kUartBridge);
    static Binds binds3b;
    static ProbeConfig same(ep3b, binds3b);
    ep3b.add(wire);
    ep3b.add(console);
    ep3b.add(same);
    same.addPlace(wire, console);
    same.load();
    same.applySaved();
    uint32_t h = 0;
    const Bytes items = getItems(same, &h);
    const Bytes st2 = stateOf(same);
    CHECK(items == getItems(config) && st2.size() >= 8 && st2[1] == cfg::kStorageStateApplied &&
          hashIn(Bytes(st2.begin() + 2, st2.begin() + 6)) == h);
    CHECK(binds3b.spec(0).set && binds3b.spec(0).source.kind == Binds::kSlotConsole);
  }

  {   // label (probe.config §1): text 1-32 bytes (its only rule), else malformed; a channel not below channels or one the
      // probe uses itself (not in the pin table) unsupported with the item's tag
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
    CHECK(ok(set(pinned, label(2, {0xC3, 0xA9}), out)));
    CHECK(ok(set(pinned, label(3, Bytes(32, 'a')), out)));            // label_max_bytes
    CHECK(ok(set(pinned, label(4, {'a', 0x0A}), out)));               // the length is the only rule
    CHECK(malformed(set(pinned, label(1, {}), out)));                 // no text
    CHECK(malformed(set(pinned, label(1, Bytes(33, 'a')), out)));
    CHECK(unsupportedWith(set(pinned, label(8, {'x'}), out), out, label_raw));   // beyond the channels

    // idle (probe.config §1): channel(u16) mode(u8) drive(u8), 4 bytes; drive read only for an output (mode 3 / 4)
    const uint8_t idle_raw = cfg::kTlvItemIdle | kTagCritical;
    auto idle = [&](uint16_t channel, uint8_t mode, uint8_t drive, bool critical = true) {
      return item(uint8_t(cfg::kTlvItemIdle | (critical ? kTagCritical : 0)), {uint8_t(channel), uint8_t(channel >> 8), mode, drive});
    };
    CHECK(ok(set(pinned, idle(1, cfg::kIdleModeHiZ, 0xff), out)));
    CHECK(ok(set(pinned, idle(1, cfg::kIdleModeHiZ, 7), out)));          // an input: drive not read
    CHECK(ok(set(pinned, idle(1, cfg::kIdleModePullUp, 0), out)));
    CHECK(ok(set(pinned, idle(2, cfg::kIdleModeOutputLow, 0xff), out)));   // an output at the default
    CHECK(itemValue(getItems(pinned), cfg::kTlvItemIdle, {2, 0}) == Bytes({2, 0, cfg::kIdleModeOutputLow, 0xff}));
    CHECK(pins.idle(2) == PinTable::kIdleOutputLow && pins.idleDrive(2) == PinTable::kDriveDefault);
    // no drive_levels on this probe (no oep.fixture.gpio): an output idle with any drive but 0xFF is unsupported
    CHECK(unsupportedWith(set(pinned, idle(2, cfg::kIdleModeOutputHigh, 0), out), out, idle_raw));
    CHECK(unsupportedWith(set(pinned, idle(2, cfg::kIdleModeOutputHigh, 1, false), out), out, cfg::kTlvItemIdle));
    CHECK(unsupportedWith(set(pinned, idle(1, 5, 0xff), out), out, idle_raw));   // mode 5
    // any other length: malformed, critical or not (the 6-byte form before too)
    CHECK(malformed(set(pinned, item(idle_raw, {1, 0, 0}), out)));
    CHECK(malformed(set(pinned, item(idle_raw, {1, 0, 0, 0xff, 0}), out)));
    CHECK(malformed(set(pinned, item(cfg::kTlvItemIdle, {1, 0, 0, 0xff, 0}), out)));
    CHECK(malformed(set(pinned, item(idle_raw, {1, 0, 0, 2, 0, 0}), out)));
    CHECK(itemValue(getItems(pinned), cfg::kTlvItemIdle, {1, 0}) == Bytes({1, 0, cfg::kIdleModePullUp, 0}));

    // disable (§1): with an idle on the same channel malformed; disabled, an idle naming it is not taken either
    const Bytes disable3 = item(cfg::kTlvItemDisable | kTagCritical, {3, 0});
    CHECK(malformed(set(pinned, cat(disable3, idle(3, cfg::kIdleModeHiZ, 0xff)), out)));
    CHECK(ok(set(pinned, disable3, out)) && pins.disabled(3));
    CHECK(malformed(set(pinned, idle(3, cfg::kIdleModeHiZ, 0xff), out)));
    CHECK(malformed(set(pinned, item(cfg::kTlvItemDisable, {3, 0, 0}), out)));   // 3 bytes
    CHECK(unsupportedWith(set(pinned, item(cfg::kTlvItemDisable, {9, 0}), out), out, cfg::kTlvItemDisable));

    // slots_max 0 (no wire place): describe's items leave slot out, and a slot item is unsupported with its tag
    uint8_t d[64];
    const size_t dn = pinned.describe(d, sizeof d);
    bool listed = false, slots_max0 = false, bind_modes = false;
    for (size_t at = 0; at + kTlvHeader <= dn; at += kTlvHeader + (d[at + 1] | d[at + 2] << 8)) {
      const size_t len = d[at + 1] | d[at + 2] << 8;
      if (d[at] == cfg::kTlvDescribeItems)
        for (size_t k = 0; k < len; ++k) listed |= d[at + kTlvHeader + k] == cfg::kTlvItemSlot;
      if (d[at] == cfg::kTlvDescribeSlotsMax) slots_max0 = len == 1 && d[at + kTlvHeader] == 0;
      bind_modes |= d[at] == 0x43;   // bind_modes: gone
    }
    CHECK(dn > 0 && !listed && slots_max0 && !bind_modes);
    CHECK(unsupportedWith(set(pinned, slotItem(0, 0), out), out, slot_raw));
    CHECK(unsupportedWith(set(pinned, item(slot_raw, {0}), out), out, slot_raw));   // its form is not looked at
    // a channel without pulls (setNoPull): an idle with mode 1 / 2 unsupported with the item's tag, Hi-Z taken
    pins.setNoPull(1ull << 6);
    CHECK(unsupportedWith(set(pinned, idle(6, cfg::kIdleModePullUp, 0xff), out), out, idle_raw));
    CHECK(unsupportedWith(set(pinned, idle(6, cfg::kIdleModePullDown, 0xff), out), out, idle_raw));
    CHECK(ok(set(pinned, idle(6, cfg::kIdleModeHiZ, 0xff), out)) && ok(set(pinned, idle(5, cfg::kIdleModePullUp, 0xff), out)));
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
    // disable (§1): a plan naming a disabled channel is unavailable cause 5 with the channel
    const Bytes disable6 = item(cfg::kTlvItemDisable | kTagCritical, {6, 0});
    CHECK(unavailableWith(set(cfga, cat(disable6, item(cfg::kTlvItemPlan | kTagCritical, {1, 0, reg::fixture_gpio::kRoleLine, 6, 0})),
                              out),
                          out, reg::core::kUnavailableCauseHeldBySettings, 6));
    // an output idle on a probe with a gpio fixture: drive 0xFF taken; a level only where drive_levels are declared
#if defined(OEP_HOST_FAKE_DRIVE)
    // four levels: 3 the last, 4 past them
    CHECK(ok(set(cfga, item(cfg::kTlvItemIdle | kTagCritical, {7, 0, cfg::kIdleModeOutputHigh, 3}), out)));
    CHECK(pins.idleDrive(7) == 3);
    CHECK(unsupportedWith(set(cfga, item(cfg::kTlvItemIdle | kTagCritical, {7, 0, cfg::kIdleModeOutputHigh, 4}), out), out,
                          cfg::kTlvItemIdle | kTagCritical));
    CHECK(ok(set(cfga, item(cfg::kTlvItemIdle | kTagCritical, {7, 0, cfg::kIdleModeHiZ, 4}), out)));   // input: not read
#else
    CHECK(unsupportedWith(set(cfga, item(cfg::kTlvItemIdle | kTagCritical, {7, 0, cfg::kIdleModeOutputHigh, 0}), out), out,
                          cfg::kTlvItemIdle | kTagCritical));
#endif
    CHECK(ok(set(cfga, item(cfg::kTlvItemIdle | kTagCritical, {7, 0, cfg::kIdleModeOutputHigh, 0xff}), out)));
    out.assign(64, 0);
    const uint8_t unset_idle[] = {1, 2, cfg::kTlvItemIdle, 7, 0};
    CHECK(ok(cfga.handle(cfg::kOpUnset, unset_idle, sizeof unset_idle, out.data(), out.size())));
    const Bytes plan_item = item(cfg::kTlvItemPlan | kTagCritical, {1, 0, reg::fixture_gpio::kRoleLine, 5, 0});
    CHECK(ok(set(cfga, plan_item, out)));
    out.assign(64, 0);
    CHECK(ok(cfga.handle(cfg::kOpSave, nullptr, 0, out.data(), out.size())));
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
    CHECK(cfgc.savedDisabled() == 0);
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
    const Bytes st = stateOf(cfgb);
    CHECK(st.size() >= 2 && st[1] == cfg::kStorageStateApplied);
    // a plan item naming oep.probe.plan's fn (no plan role): refused unsupported, nothing saved changes
    CHECK(unsupportedWith(set(cfgb, item(cfg::kTlvItemPlan | kTagCritical, {4, 0, 1, 6, 0}), out), out,
                          cfg::kTlvItemPlan | kTagCritical));
    // a plan item of another length: malformed
    CHECK(malformed(set(cfgb, item(cfg::kTlvItemPlan, {2, 0, 1, 6, 0, 0}), out)));
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
    auto tried = [&](ProbeConfig &c) {   // slot 0 absent and its last_try_at_ns not all ones
      const Bytes st = stateOf(c);
      if (st.size() < 8 + 12 || st[7] != 1 || st[9] != cfg::kSlotStateAbsent) return false;
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
    CHECK(ok(set(cfg6, slotItem(cfg::kSlotAttachAtBoot, 0, 100), out)));
    CHECK(tried(cfg6));
  }

  // ---- the wifi item (item 0x08, probe.config §1.4): index ssid_len ssid pass_len passphrase, key index ----
  {
    struct FakeWifi final : WifiControl {
      std::vector<WifiEntry> got;
      int applied = 0;
      Status st;
      void apply(const WifiEntry *e, size_t n) override { got.assign(e, e + n); ++applied; }
      Status status() const override { return st; }
    };
    static NullStream s7, s8;
    static uint8_t rx7[512], tx7[512], rx8[512], tx8[512];
    static Endpoint ep7(s7, rx7, sizeof rx7, tx7, sizeof tx7, {512, 1024, 2}, Endpoint::kUartBridge);
    static Endpoint ep8(s8, rx8, sizeof rx8, tx8, sizeof tx8, {512, 1024, 2}, Endpoint::kUartBridge);
    static Binds binds7, binds8;
    static ProbeConfig plain(ep7, binds7), cfg7(ep8, binds8);
    static FakeWifi fake;
    ep7.add(plain);
    ep8.add(cfg7);
    cfg7.setWifi(&fake);
    const uint8_t wraw = kWifiItemTag;
    auto wv = [](uint8_t index, const char *ssid, const char *pass) {
      Bytes v = {index, uint8_t(strlen(ssid))};
      v.insert(v.end(), ssid, ssid + strlen(ssid));
      v.push_back(uint8_t(strlen(pass)));
      v.insert(v.end(), pass, pass + strlen(pass));
      return v;
    };
    const char *secret = "s3cret-pass", *secret2 = "another-pass";
    auto leaks = [&](const Bytes &b) {
      for (const char *p : {secret, secret2})
        if (std::search(b.begin(), b.end(), p, p + strlen(p)) != b.end()) return true;
      return false;
    };
    // not declared without something that joins networks: unsupported with the tag as received
    CHECK(unsupportedWith(set(plain, item(wraw, wv(0, "lab", secret)), out), out, wraw));
    {
      Bytes d(256);
      const size_t n = plain.describe(d.data(), d.size());
      bool found = false;
      describeTlv(Bytes(d.begin(), d.begin() + n), 0, kWifiDescribeMax, &found);
      CHECK(!found);
    }
    // declared: items lists 0x08, wifi_max 4
    {
      Bytes d(256);
      const size_t n = cfg7.describe(d.data(), d.size());
      const Bytes tl(d.begin(), d.begin() + n);
      bool found = false;
      const Bytes items = describeTlv(tl, 0, cfg::kTlvDescribeItems);
      CHECK(std::find(items.begin(), items.end(), kWifiItemTag) != items.end());
      const Bytes max = describeTlv(tl, 0, kWifiDescribeMax, &found);
      CHECK(found && max.size() == 1 && max[0] == ProbeConfig::kMaxWifi);
    }
    // the list goes over in index order, whatever order the set had
    CHECK(ok(set(cfg7, cat(item(wraw, wv(1, "home", secret2)), item(wraw | kTagCritical, wv(0, "lab", secret))), out)));
    CHECK(fake.applied == 1 && fake.got.size() == 2 && fake.got[0].index == 0 && fake.got[1].index == 1 &&
          strcmp(fake.got[0].ssid, "lab") == 0 && strcmp(fake.got[0].pass, secret) == 0 &&
          strcmp(fake.got[1].pass, secret2) == 0);
    // get: no passphrase (pass_len 0xFF, nothing after it)
    uint32_t h1 = 0;
    Bytes items = getItems(cfg7, &h1);
    CHECK(!leaks(items));
    {
      bool found = false;
      const Bytes v = itemValue(items, kWifiItemTag, {0}, &found);
      CHECK(found && v.size() == 2 + 3 + 1 && v[1] == 3 && v[5] == kWifiPassHidden);
    }
    // the same set again: nothing changes, nothing applied again, the same hash
    CHECK(ok(set(cfg7, item(wraw, wv(0, "lab", secret)), out)));
    CHECK(fake.applied == 1 && hashIn(out) == h1);
    // get's form sent back (pass_len 0xFF) keeps the passphrase: the round trip changes nothing
    CHECK(ok(set(cfg7, items, out)));
    CHECK(fake.applied == 1 && hashIn(out) == h1);
    // ... and a new SSID with the passphrase kept
    {
      Bytes v = {0, 4, 'l', 'a', 'b', '2', kWifiPassHidden};
      CHECK(ok(set(cfg7, item(wraw, v), out)));
      CHECK(fake.applied == 2 && strcmp(fake.got[0].ssid, "lab2") == 0 && strcmp(fake.got[0].pass, secret) == 0);
      CHECK(hashIn(out) != h1);
      v[0] = 2;   // no entry 2: nothing to keep
      CHECK(malformed(set(cfg7, item(wraw, v), out)));
    }
    // a changed passphrase changes the hash, though get shows the same bytes
    CHECK(ok(set(cfg7, item(wraw, wv(0, "lab", secret)), out)));
    const uint32_t h2 = hashIn(out);
    const Bytes shown2 = getItems(cfg7);
    CHECK(ok(set(cfg7, item(wraw, wv(0, "lab", "s3cret-pass2")), out)));
    CHECK(hashIn(out) != h2 && getItems(cfg7) == shown2);
    CHECK(ok(set(cfg7, item(wraw, wv(0, "lab", secret)), out)));
    // the forms: an open network; 8-63 printable; 64 hex; anything else malformed; an SSID with 0x00 unsupported
    CHECK(ok(set(cfg7, item(wraw, wv(2, "open", "")), out)));
    CHECK(malformed(set(cfg7, item(wraw, wv(2, "x", "1234567")), out)));                 // 7 bytes
    CHECK(malformed(set(cfg7, item(wraw, wv(2, "x", std::string(64, 'g').c_str())), out)));   // 64, not hex
    CHECK(ok(set(cfg7, item(wraw, wv(2, "x", std::string(64, 'a').c_str())), out)));
    CHECK(malformed(set(cfg7, item(wraw, wv(2, "x", "pass\tword")), out)));             // a control byte
    // index at wifi_max: unsupported with the tag as received (probe.config §1.4), critical or not
    CHECK(unsupportedWith(set(cfg7, item(wraw, wv(uint8_t(ProbeConfig::kMaxWifi), "x", "")), out), out, wraw));
    CHECK(unsupportedWith(set(cfg7, item(wraw | kTagCritical, wv(uint8_t(ProbeConfig::kMaxWifi), "x", "")), out), out,
                          uint8_t(wraw | kTagCritical)));
    CHECK(malformed(set(cfg7, item(wraw, wv(2, std::string(33, 'a').c_str(), "")), out)));      // ssid 33 bytes
    CHECK(malformed(set(cfg7, item(wraw, wv(2, "", "")), out)));                          // ssid empty
    {
      Bytes v = wv(2, "x", "");
      v.push_back(0);   // a byte past the passphrase
      CHECK(malformed(set(cfg7, item(wraw, v), out)));
      v = {2, 3, 'a', 0, 'b', 0};
      CHECK(unsupportedWith(set(cfg7, item(wraw, v), out), out, wraw));
    }
    CHECK(malformed(set(cfg7, cat(item(wraw, wv(3, "a", "")), item(wraw, wv(3, "b", ""))), out)));   // one key twice
    // the state: TLV 0x01 wifi state entry reason rssi ipv4; rssi and ipv4 0 unless connected, whatever the radio says
    fake.st.state = WifiControl::kStateWaiting;
    fake.st.entry = WifiControl::kNoEntry;
    fake.st.reason = WifiControl::kReasonAuth;
    fake.st.rssi = -70;
    memset(fake.st.ipv4, 7, 4);
    {
      const Bytes w = describeTlv(stateOf(cfg7), 9, kWifiStateTlv);
      CHECK(w == Bytes({3, 0xFF, 2, 0, 0, 0, 0, 0}));
    }
    fake.st.reason = WifiControl::kReasonNone;
    fake.st.state = WifiControl::kStateConnected;
    fake.st.entry = 1;
    fake.st.rssi = -61;
    const uint8_t ip[4] = {192, 168, 1, 23};
    memcpy(fake.st.ipv4, ip, 4);
    {
      const Bytes st = stateOf(cfg7);
      CHECK(st.size() == 9 + kTlvHeader + 8);
      const Bytes w = describeTlv(st, 9, kWifiStateTlv);
      CHECK(w.size() == 8 && w[0] == WifiControl::kStateConnected && w[1] == 1 && int8_t(w[3]) == -61 && w[4] == 192 &&
            w[7] == 23);
    }
    // unset by index; saved and read back with the passphrases (never shown)
    const uint8_t unset2[] = {1, 1, kWifiItemTag, 2};
    out.assign(64, 0);
    CHECK(ok(cfg7.handle(cfg::kOpUnset, unset2, sizeof unset2, out.data(), out.size())));
    CHECK(fake.got.size() == 2 && fake.got[1].index == 1);
    out.assign(64, 0);
    CHECK(ok(cfg7.handle(cfg::kOpSave, nullptr, 0, out.data(), out.size())));
    static NullStream s9;
    static uint8_t rx9[512], tx9[512];
    static Endpoint ep9(s9, rx9, sizeof rx9, tx9, sizeof tx9, {512, 1024, 2}, Endpoint::kUartBridge);
    static Binds binds9;
    static ProbeConfig cfg9(ep9, binds9);
    static FakeWifi fake9;
    ep9.add(cfg9);
    cfg9.setWifi(&fake9);
    cfg9.load();
    cfg9.applySaved();
    CHECK(fake9.applied == 1 && fake9.got.size() == 2 && strcmp(fake9.got[0].pass, secret) == 0 &&
          strcmp(fake9.got[1].pass, secret2) == 0);
    {
      const Bytes st = stateOf(cfg9);
      CHECK(st.size() >= 2 && st[1] == cfg::kStorageStateApplied);
      uint32_t h = 0;
      const Bytes got = getItems(cfg9, &h);
      CHECK(!leaks(got) && !leaks(st) && hashIn(u32(h)) == uint32_t(st[2] | st[3] << 8 | st[4] << 16 | uint32_t(st[5]) << 24));
    }
    // every item unset: the list empty (Wi-Fi off)
    const uint8_t unset01[] = {2, 1, kWifiItemTag, 0, 1, kWifiItemTag, 1};
    out.assign(64, 0);
    CHECK(ok(cfg9.handle(cfg::kOpUnset, unset01, sizeof unset01, out.data(), out.size())));
    CHECK(fake9.applied == 2 && fake9.got.empty());

    // probe.config §1.4: a probe with the wifi item answers max_frame 112 or more on every transport - one set of the
    // longest wifi item (32-byte ssid, 64 hex digits) is 10 + 3 + 3 + 32 + 64 = 112 bytes. Below that the item is not
    // declared; at 112 it is.
    static_assert(kWifiMinMaxFrame == 10 + 3 + 3 + reg::kLimitWifiSsidMaxBytes + reg::kLimitWifiPskHexDigits, "§1.4");
    static NullStream s10, s11;
    static uint8_t rx10[512], tx10[512], rx11[512], tx11[512];
    static Endpoint ep10(s10, rx10, sizeof rx10, tx10, sizeof tx10, {kWifiMinMaxFrame - 1, 1024, 2}, Endpoint::kUartBridge);
    static Endpoint ep11(s11, rx11, sizeof rx11, tx11, sizeof tx11, {kWifiMinMaxFrame, 1024, 2}, Endpoint::kTcp);
    static Binds binds10, binds11;
    static ProbeConfig cfg10(ep10, binds10), cfg11(ep11, binds11);
    static FakeWifi fake10, fake11;
    ep10.add(cfg10);
    ep11.add(cfg11);
    CHECK(!cfg10.setWifi(&fake10) && cfg11.setWifi(&fake11));
    {
      Bytes d(256);
      bool found = true;
      describeTlv(Bytes(d.begin(), d.begin() + cfg10.describe(d.data(), d.size())), 0, kWifiDescribeMax, &found);
      CHECK(!found);
      describeTlv(Bytes(d.begin(), d.begin() + cfg11.describe(d.data(), d.size())), 0, kWifiDescribeMax, &found);
      CHECK(found);
    }
    CHECK(unsupportedWith(set(cfg10, item(wraw, wv(0, "lab", secret)), out), out, wraw));
  }

  printf("config: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
