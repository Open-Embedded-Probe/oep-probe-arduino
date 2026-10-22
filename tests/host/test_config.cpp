// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the settings (oep-spec oep-core §13) beyond the shared vectors (test_vectors):
// - fn 0's describe lists the core's items (setting, the idle's choices) and the netif, an interface's describe its own,
//   and fn 0 names every channel (channel_name);
// - get shows each value's origin: a fixed label refuses set and unset, a default one comes back after an unset;
// - a save is read back by name: a firmware whose fns moved applies the same values under its own numbers; a category
//   it does not know leaves the rest applied (state 3, reason 2); a broken container applies nothing (state 2, reason 1);
// - the Wi-Fi values go to the station only when they change; the passphrase is a secret: get shows no data, 0 bytes
//   keep it;
// - an item of the sketch's own under its interface (addSetting refuses one outside core §13.1);
// - a set past what the probe holds is refused cause 2 with nothing changed.
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "OepEndpoint.h"
#include "OepFrame.h"
#include "OepPinTable.h"
#include "OepSettings.h"

uint32_t g_millis = 1000;
void (*g_on_wait)() = nullptr;

using namespace oep;
using namespace setting_id;
using Bytes = std::vector<uint8_t>;
namespace rc = reg::core;

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
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override { tx.insert(tx.end(), b, b + n); return n; }
  int availableForWrite() override { return 1 << 20; }
};

// The station the Wi-Fi values go to: what it was handed, how often.
struct FakeStation final : WifiControl {
  int applies = 0;
  std::vector<WifiEntry> entries;
  Ipv4Setting ipv4;
  void apply(const WifiEntry *e, size_t n, const Ipv4Setting &ip) override {
    ++applies;
    entries.assign(e, e + n);
    ipv4 = ip;
  }
  Status status() const override { return {}; }
};

// An interface of the sketch's own, with items (core §13.1).
struct Board final : Interface {
  const char *name_;
  explicit Board(const char *n = "io.example.board") : name_(n) {}
  const char *name() const override { return name_; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }
  bool offers(uint8_t op) const override { return op == 0x10; }
  Result handle(uint8_t, const uint8_t *, size_t, uint8_t *, size_t) override { return completed(); }
};

// One boot of a probe: an endpoint with pins 0-15, the settings, optionally the station; `boards` interfaces added first.
struct Probe {
  MemStream s;
  uint16_t corr = 1;
  uint8_t rx[1100], tx[1100];
  Endpoint ep{s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0};
  PinTable pins{0xFFFFull};
  Binds binds;
  Settings settings{ep, binds};
  FakeStation station;
  Board board, other{"io.example.other"};
  explicit Probe(bool wifi = true, bool other_first = false) {
    if (other_first) ep.add(other);
    ep.add(board);
    settings.setPins(&pins);
    if (wifi) CHECK(settings.setWifi(&station));
  }
  Result op(uint8_t code, const Bytes &p, Bytes &out) {
    out.assign(1024, 0);
    const Result r = settings.handle(code, p.data(), p.size(), out.data(), out.size());
    out.resize(r.length);
    return r;
  }
  // fn's describe TLVs (every page)
  std::vector<std::pair<uint8_t, Bytes>> describe(uint16_t fn) {
    std::vector<std::pair<uint8_t, Bytes>> all;
    for (uint16_t first = 0;;) {
      const Bytes m = {kRoleRequest, uint8_t(corr), uint8_t(corr >> 8), 0, 0, kOpDescribe, 0, 0, 0, 0,
                       uint8_t(fn), uint8_t(fn >> 8), uint8_t(first), uint8_t(first >> 8)};
      ++corr;
      s.rx = {uint8_t(m.size()), uint8_t(m.size() >> 8)};   // vendor bulk: length(u16) message (transports §1)
      s.rx.insert(s.rx.end(), m.begin(), m.end());
      s.at = 0;
      s.tx.clear();
      ep.poll();
      if (s.tx.size() < 2 + kResultHeader + 1 || s.tx[2 + 3] != kResolutionCompleted) return all;
      const uint8_t *out = s.tx.data() + 2 + kResultHeader;
      const size_t length = s.tx.size() - 2 - kResultHeader;
      size_t n = 0;
      for (size_t at = 1; at + kTlvHeader <= length; at += kTlvHeader + getU16(out + at + 1), ++n)
        all.emplace_back(out[at], Bytes(out + at + kTlvHeader, out + at + kTlvHeader + getU16(out + at + 1)));
      if (!out[0]) return all;
      first = static_cast<uint16_t>(first + n);
    }
  }
  // The item names fn's describe lists, in order (the k-th is number k).
  std::vector<std::string> items(uint16_t fn) {
    std::vector<std::string> names;
    for (const auto &t : describe(fn))
      if (t.first == reg::kDescribeSetting) names.emplace_back(t.second.begin() + 6, t.second.end());
    return names;
  }
  uint8_t number(uint16_t fn, const char *name) {   // looked up by name, as a host does
    const std::vector<std::string> names = items(fn);
    for (size_t k = 0; k < names.size(); ++k) if (names[k] == name) return static_cast<uint8_t>(k + 1);
    return 0;
  }
};

static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
static Bytes value(uint16_t fn, uint8_t number, uint16_t key, const Bytes &data) {
  Bytes b = {uint8_t(fn), uint8_t(fn >> 8), number, uint8_t(key), uint8_t(key >> 8), uint8_t(data.size()), uint8_t(data.size() >> 8)};
  b.insert(b.end(), data.begin(), data.end());
  return b;
}
static Bytes set(std::initializer_list<Bytes> values) {
  Bytes p = {uint8_t(values.size())};
  for (const Bytes &v : values) p.insert(p.end(), v.begin(), v.end());
  return p;
}
static Bytes text(const char *t) { return Bytes(t, t + strlen(t)); }
// get's values, every page from first 0.
struct Value { uint16_t fn; uint8_t number; uint16_t key; uint8_t origin; Bytes data; };
static std::vector<Value> values(Probe &p) {
  std::vector<Value> all;
  Bytes out;
  for (uint16_t first = 0;;) {
    const Result r = p.op(rc::kOpGet, {uint8_t(first), uint8_t(first >> 8)}, out);
    if (!ok(r) || out.size() < 6) return all;
    size_t at = 6;
    for (uint8_t k = 0; k < out[5]; ++k) {
      const size_t len = getU16(&out[at + 6]);
      all.push_back({getU16(&out[at]), out[at + 2], getU16(&out[at + 3]), out[at + 5],
                     Bytes(out.begin() + at + 8, out.begin() + at + 8 + len)});
      at += 8 + len;
    }
    if (!out[0]) return all;
    first = static_cast<uint16_t>(first + out[5]);
  }
}
static bool has(const std::vector<Value> &all, uint16_t fn, uint8_t number, uint16_t key, const Bytes &data,
                uint8_t origin = rc::kSettingOriginUser) {
  for (const Value &v : all)
    if (v.fn == fn && v.number == number && v.key == key && v.data == data && v.origin == origin) return true;
  return false;
}

static void testDescribe() {
  fakeConfigStore(nullptr, 0);
  Probe p;
  CHECK(p.settings.addSetting(p.board, "ntp-server", rc::kSettingFormText, 0, 32));
  CHECK(p.items(0) == (std::vector<std::string>{rc::kSettingChannelLabel, rc::kSettingChannelIdle, rc::kSettingChannelDisable,
                                                 rc::kSettingWifiSsid, rc::kSettingWifiPassphrase, rc::kSettingIpv4Mode,
                                                 rc::kSettingIpv4Address, rc::kSettingIpv4Prefix, rc::kSettingIpv4Gateway,
                                                 rc::kSettingIpv4Dns}));
  CHECK(p.items(1) == std::vector<std::string>{"ntp-server"});
  bool netif = false, pull_up = false;
  for (const auto &t : p.describe(0)) {
    netif |= t.first == rc::kTlvDescribeNetif && t.second == text(rc::kNameNetifWifi);
    pull_up |= t.first == reg::kDescribeSettingChoice && t.second[0] == p.number(0, rc::kSettingChannelIdle) &&
               t.second[1] == rc::kChannelIdlePullUp && Bytes(t.second.begin() + 2, t.second.end()) == text("pull-up");
  }
  CHECK(netif && pull_up);
  Probe q(false);   // no station: no Wi-Fi items, no netif, no network op
  CHECK(q.items(0).size() == 3 && !q.settings.offers(rc::kOpNetwork) && p.settings.offers(rc::kOpNetwork));
}

// fixed and default values (core §13.4)
static void testOrigins() {
  fakeConfigStore(nullptr, 0);
  Probe p(false);
  const uint8_t label = p.number(0, rc::kSettingChannelLabel);
  CHECK(p.settings.fixedLabel(0, "LED"));
  CHECK(p.settings.firmwareValue(nullptr, rc::kSettingChannelLabel, 1, text("RX").data(), 2, false));
  std::vector<Value> got = values(p);
  CHECK(has(got, 0, label, 0, text("LED"), rc::kSettingOriginFixed) && has(got, 0, label, 1, text("RX"), rc::kSettingOriginDefault));
  Bytes out;
  Result r = p.op(rc::kOpSet, set({value(0, label, 0, text("X"))}), out);
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported);
  r = p.op(rc::kOpUnset, {1, 0, 0, label, 0, 0}, out);
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported);
  CHECK(ok(p.op(rc::kOpSet, set({value(0, label, 1, text("UART RX"))}), out)));
  CHECK(has(values(p), 0, label, 1, text("UART RX")));
  CHECK(ok(p.op(rc::kOpUnset, {1, 0, 0, label, 1, 0}, out)));
  CHECK(has(values(p), 0, label, 1, text("RX"), rc::kSettingOriginDefault));
}

// A save is read back by name (core §13.7): a firmware whose fns moved applies the same values under its own numbers.
static void testSavedByName() {
  fakeConfigStore(nullptr, 0);
  Bytes out;
  {
    Probe a(false);
    CHECK(a.settings.addSetting(a.board, "mode", rc::kSettingFormU32));
    const uint8_t label = a.number(0, rc::kSettingChannelLabel), idle = a.number(0, rc::kSettingChannelIdle);
    CHECK(ok(a.op(rc::kOpSet, set({value(0, label, 5, text("DUT RX")), value(0, idle, 6, {rc::kChannelIdlePullUp}),
                                   value(1, 1, 0, {7, 0, 0, 0})}), out)));
    CHECK(ok(a.op(rc::kOpSave, {}, out)));
  }
  Probe b(true, true);   // the station too, and another interface first: the board is fn 2 now
  CHECK(b.settings.addSetting(b.board, "mode", rc::kSettingFormU32));
  b.settings.load();
  b.settings.applySaved();
  const std::vector<Value> got = values(b);
  CHECK(has(got, 0, b.number(0, rc::kSettingChannelLabel), 5, text("DUT RX")));
  CHECK(has(got, 0, b.number(0, rc::kSettingChannelIdle), 6, {rc::kChannelIdlePullUp}));
  CHECK(has(got, 2, 1, 0, {7, 0, 0, 0}));
  CHECK(b.pins.idle(6) == PinTable::kIdlePullUp);
  CHECK(ok(b.op(rc::kOpState, {}, out)) && out[0] == rc::kStorageStateApplied && out[5] == 0);
  Probe c(false);   // a firmware without the board's item: the rest applied, state 3, reason 2
  c.settings.load();
  c.settings.applySaved();
  CHECK(has(values(c), 0, c.number(0, rc::kSettingChannelLabel), 5, text("DUT RX")) && c.pins.idle(6) == PinTable::kIdlePullUp);
  CHECK(ok(c.op(rc::kOpState, {}, out)) && out[0] == rc::kStorageStatePartial && out[5] == rc::kStorageReasonItem);
  // erase: everything unset, then saved
  const std::vector<Value> now = values(c);
  Bytes un = {uint8_t(now.size())};
  for (const Value &v : now) un.insert(un.end(), {uint8_t(v.fn), uint8_t(v.fn >> 8), v.number, uint8_t(v.key), uint8_t(v.key >> 8)});
  CHECK(ok(c.op(rc::kOpUnset, un, out)) && ok(c.op(rc::kOpSave, {}, out)));
  Probe d(false);
  d.settings.load();
  d.settings.applySaved();
  CHECK(values(d).empty() && ok(d.op(rc::kOpState, {}, out)) && out[0] == rc::kStorageStateApplied);
}

// The disabled channels are read before the entries exist (load: a sketch parks its pins first).
static void testSavedDisabled() {
  fakeConfigStore(nullptr, 0);
  Bytes out;
  {
    Probe a(false);
    CHECK(ok(a.op(rc::kOpSet, set({value(0, a.number(0, rc::kSettingChannelDisable), 9, {1})}), out)));
    CHECK(ok(a.op(rc::kOpSave, {}, out)));
  }
  Probe b(false);
  b.settings.load();
  CHECK(b.settings.savedDisabled() == (1ull << 9));
}

// A container whose CRC does not match applies nothing: state 2, reason 1.
static void testBrokenContainer() {
  const uint8_t junk[] = {1, 0, 0, 0, 1, 5, 'l', 'a', 'b', 'e', 'l', 0, 0, 0, 0, 0};
  fakeConfigStore(junk, sizeof junk);
  Probe p(false);
  p.settings.load();
  p.settings.applySaved();
  Bytes out;
  CHECK(values(p).empty());
  CHECK(ok(p.op(rc::kOpState, {}, out)) && out[0] == rc::kStorageStateUnreadable && out[5] == rc::kStorageReasonForm);
}

// Wi-Fi (transports §7): handed to the station only when it changes; the passphrase a secret.
static void testWifi() {
  fakeConfigStore(nullptr, 0);
  Probe p;
  p.settings.applySaved();   // boot: the station gets the (empty) list once
  const int boot = p.station.applies;
  Bytes out;
  const uint8_t ssid = p.number(0, rc::kSettingWifiSsid), pass = p.number(0, rc::kSettingWifiPassphrase);
  CHECK(ok(p.op(rc::kOpSet, set({value(0, ssid, 0, text("lab")), value(0, pass, 0, text("password1"))}), out)));
  CHECK(p.station.applies == boot + 1 && p.station.entries.size() == 1 && p.station.entries[0].pass_length == 9);
  CHECK(ok(p.op(rc::kOpSet, set({value(0, p.number(0, rc::kSettingChannelLabel), 3, text("x"))}), out)));
  CHECK(p.station.applies == boot + 1);
  const std::vector<Value> got = values(p);
  CHECK(has(got, 0, pass, 0, {}));   // no data (core §13.5)
  CHECK(ok(p.op(rc::kOpSet, set({value(0, pass, 0, {})}), out)));   // sent back: kept
  CHECK(p.station.applies == boot + 1 && memcmp(p.station.entries[0].pass, "password1", 9) == 0);
  Result r = p.op(rc::kOpSet, set({value(0, pass, 1, {})}), out);   // 0 bytes with nothing there: malformed
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
  r = p.op(rc::kOpSet, set({value(0, pass, 1, text("short"))}), out);   // under 8: malformed
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
  CHECK(ok(p.op(rc::kOpSet, set({value(0, p.number(0, rc::kSettingIpv4Mode), 1, {rc::kIpv4ModeStatic}),
                                 value(0, p.number(0, rc::kSettingIpv4Address), 1, {192, 168, 1, 50}),
                                 value(0, p.number(0, rc::kSettingIpv4Prefix), 1, {24, 0, 0, 0})}), out)));
  CHECK(p.station.applies == boot + 2 && p.station.ipv4.mode == rc::kIpv4ModeStatic && p.station.ipv4.prefix == 24);
  CHECK(ok(p.op(rc::kOpUnset, {1, 0, 0, ssid, 0, 0}, out)) && p.station.applies == boot + 3 && p.station.entries.empty());
}

// The sketch's own items (core §13.1): by form; refused outside the rules.
static void testOwnSettings() {
  fakeConfigStore(nullptr, 0);
  Probe p(false);
  CHECK(!p.settings.addSetting(p.board, "Bad", rc::kSettingFormText, 0, 32));            // not a-z 0-9 -
  CHECK(!p.settings.addSetting(p.board, "x", rc::kSettingFormEnum));                     // a form the sketch cannot give
  CHECK(!p.settings.addSetting(p.board, "x", rc::kSettingFormText, 0, 0));               // text needs a max_len
  CHECK(!p.settings.addSetting(p.board, "x", rc::kSettingFormU32, 0, 4));                // max_len only for bytes / text
  CHECK(!p.settings.addSetting(p.board, "x.{}", rc::kSettingFormBool));                  // a key place without keys
  CHECK(p.settings.addSetting(p.board, "flag.{}", rc::kSettingFormBool, 2));
  CHECK(p.settings.addSetting(p.board, "token", rc::kSettingFormText, 0, 16, rc::kSettingFlagSecret));
  CHECK(!p.settings.addSetting(p.board, "token", rc::kSettingFormBool));                 // a name twice
  Bytes out;
  CHECK(ok(p.op(rc::kOpSet, set({value(1, 1, 1, {1})}), out)));
  Result r = p.op(rc::kOpSet, set({value(1, 1, 0, {2})}), out);   // bool 2: malformed
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
  r = p.op(rc::kOpSet, set({value(1, 1, 0, {1}), value(1, 1, 2, {1})}), out);   // key 2 of 2: unsupported, item 1
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported &&
        out == Bytes({kTagValue, rc::kTlvUnsupportedPayloadItem, 1, 0, 1}));
  CHECK(ok(p.op(rc::kOpSet, set({value(1, 2, 0, text("s3cret"))}), out)));
  size_t len = 0;
  const uint8_t *v = p.settings.setting(p.board, "token", 0, len);
  CHECK(v && len == 6 && memcmp(v, "s3cret", 6) == 0);
  r = p.op(rc::kOpSet, set({value(9, 1, 0, {1})}), out);   // a fn the probe does not have
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnknownFunction);
}

// More than the probe holds (kMaxItems): unavailable cause 2, nothing changed.
static void testCapacity() {
  fakeConfigStore(nullptr, 0);
  Probe p(false);
  CHECK(p.settings.addSetting(p.board, "blob.{}", rc::kSettingFormBytes, 64, 64));
  Bytes out;
  const size_t fit = Settings::kMaxItems / (kTlvHeader + 2 + 64);   // whole values the store holds
  uint32_t generation = 0;
  for (uint16_t k = 0; k < fit; ++k) {
    CHECK(ok(p.op(rc::kOpSet, set({value(1, 1, k, Bytes(64, uint8_t(k)))}), out)));
    generation = getU32(out.data());
  }
  const Result r = p.op(rc::kOpSet, set({value(1, 1, uint16_t(fit), Bytes(64, 0xEE))}), out);
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnavailable && out.size() >= 4 &&
        out[3] == rc::kUnavailableCauseLimit);
  CHECK(values(p).size() == fit);
  CHECK(ok(p.op(rc::kOpGet, {0, 0}, out)) && getU32(out.data() + 1) == generation);   // nothing changed
}

// fn 0 names every channel (core §7.5: channel_name)
static void testChannelNames() {
  Probe p(false);
  static uint8_t probe[64];
  TlvWriter w(probe, sizeof probe);
  w.u16(kCoreChannels, 3);
  p.ep.setProbeDescription(probe, w.length());
  std::vector<std::string> names;
  for (const auto &t : p.describe(0))
    if (t.first == rc::kTlvDescribeChannelName) names.emplace_back(t.second.begin() + 2, t.second.end());
  CHECK(names == (std::vector<std::string>{"GPIO0", "GPIO1", "GPIO2"}));
}

int main() {
  testDescribe();
  testOrigins();
  testSavedByName();
  testSavedDisabled();
  testBrokenContainer();
  testWifi();
  testOwnSettings();
  testCapacity();
  testChannelNames();
  printf("settings: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
