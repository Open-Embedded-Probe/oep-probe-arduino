// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the settings (oep-spec oep-core §13) beyond the shared vectors (test_vectors):
// - fn 0's describe lists the core's items (setting, the idle's choices) and the netif, an interface's describe its own,
//   and fn 0 names every channel (channel_name);
// - initial values go in on a boot without a save and are values like any other after it; a save, an empty one too,
//   keeps them out; a probe of initial values only offers no set / unset (core §13.4, §13.5);
// - labels are unique but for ASCII case and kept as written; a save with two such labels keeps the lower channel's (§13.8);
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
struct Value { uint16_t fn; uint8_t number; uint16_t key; Bytes data; };
static std::vector<Value> values(Probe &p) {
  std::vector<Value> all;
  Bytes out;
  for (uint16_t first = 0;;) {
    const Result r = p.op(rc::kOpGet, {uint8_t(first), uint8_t(first >> 8)}, out);
    if (!ok(r) || out.size() < 6) return all;
    size_t at = 6;
    for (uint8_t k = 0; k < out[5]; ++k) {
      const size_t len = getU16(&out[at + 5]);   // fn number key len data (core §13.5)
      all.push_back({getU16(&out[at]), out[at + 2], getU16(&out[at + 3]), Bytes(out.begin() + at + 7, out.begin() + at + 7 + len)});
      at += 7 + len;
    }
    if (!out[0]) return all;
    first = static_cast<uint16_t>(first + out[5]);
  }
}
static bool has(const std::vector<Value> &all, uint16_t fn, uint8_t number, uint16_t key, const Bytes &data) {
  for (const Value &v : all)
    if (v.fn == fn && v.number == number && v.key == key && v.data == data) return true;
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

// Initial values (core §13.4): in on a boot without a save, then like any value (unset takes it away); a save - an empty
// one too - keeps them out of the next boot; clashing ones are the firmware's mistake and stay out; a probe of initial
// values only offers get alone (§13.5).
static void testInitialValues() {
  fakeConfigStore(nullptr, 0);
  Bytes out;
  {
    Probe p(false);
    const uint8_t label = p.number(0, rc::kSettingChannelLabel);
    CHECK(p.settings.initialLabel(3, "V003.swio") && !p.settings.initialLabel(99, "x"));   // 99: no channel of this probe
    p.settings.load();
    p.settings.applySaved();
    CHECK(has(values(p), 0, label, 3, text("V003.swio")));
    CHECK(ok(p.op(rc::kOpSet, set({value(0, label, 3, text("dut.swio"))}), out)) && has(values(p), 0, label, 3, text("dut.swio")));
    CHECK(ok(p.op(rc::kOpUnset, {1, 0, 0, label, 3, 0}, out)) && values(p).empty());   // gone: no initial value comes back
    CHECK(ok(p.op(rc::kOpSave, {}, out)));                                               // an empty save
  }
  {
    Probe q(false);
    CHECK(q.settings.initialLabel(3, "V003.swio"));
    q.settings.load();
    q.settings.applySaved();
    CHECK(values(q).empty() && ok(q.op(rc::kOpState, {}, out)) && out[0] == rc::kStorageStateApplied);
  }
  fakeConfigStore(nullptr, 0);
  Probe r(false);
  r.settings.setChangeable(false);
  const uint8_t label = r.number(0, rc::kSettingChannelLabel);
  CHECK(r.settings.initialLabel(3, "V003.swio") && r.settings.initialLabel(4, "v003.SWIO"));   // each a value; together a clash
  r.settings.load();
  r.settings.applySaved();
  CHECK(values(r).empty());
  CHECK(r.settings.offers(rc::kOpGet) && !r.settings.offers(rc::kOpSet) && !r.settings.offers(rc::kOpUnset) &&
        !r.settings.offers(rc::kOpSave) && !r.settings.offers(rc::kOpState));
  const Result x = r.op(rc::kOpSet, set({value(0, label, 3, text("x"))}), out);
  CHECK(x.resolution == kResolutionRejected && x.detail == kRejectUnknownOperation);
}

// Labels are unique but for ASCII case and kept as written (core §13.8); one set may move a label; a save with two such
// labels (an older firmware's) keeps the lower channel's: state 3, reason 2 (§13.7).
static void testLabelsUnique() {
  fakeConfigStore(nullptr, 0);
  Bytes out;
  Probe p(false);
  const uint8_t label = p.number(0, rc::kSettingChannelLabel);
  CHECK(ok(p.op(rc::kOpSet, set({value(0, label, 0, text("V003.nrst"))}), out)));
  Result r = p.op(rc::kOpSet, set({value(0, label, 4, text("v003.NRST"))}), out);
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnavailable && out.size() >= 8 &&
        out[3] == rc::kUnavailableCauseHeldBySettings && getU16(&out[7]) == 4);
  CHECK(ok(p.op(rc::kOpSet, set({value(0, label, 0, text("LED")), value(0, label, 4, text("V003.nrst"))}), out)));
  CHECK(has(values(p), 0, label, 4, text("V003.nrst")) && has(values(p), 0, label, 0, text("LED")));
  Bytes blob = {1, 0, 0, 0, 1};   // the save's form (OepSettings.cpp): version, the core, one item with two values
  const char *n = rc::kSettingChannelLabel;
  blob.push_back(uint8_t(strlen(n)));
  blob.insert(blob.end(), n, n + strlen(n));
  blob.insert(blob.end(), {2, 0});
  for (const char *kv : {"GPIO7\x05PORT1", "GPIO2\x05port1"}) {
    const char *sep = strchr(kv, 5);
    blob.push_back(uint8_t(sep - kv));
    blob.insert(blob.end(), kv, sep);
    blob.push_back(uint8_t(strlen(sep + 1)));
    blob.insert(blob.end(), sep + 1, sep + 1 + strlen(sep + 1));
  }
  const uint32_t crc = crc32Ieee(blob.data(), blob.size());
  blob.insert(blob.end(), {uint8_t(crc), uint8_t(crc >> 8), uint8_t(crc >> 16), uint8_t(crc >> 24)});
  fakeConfigStore(blob.data(), blob.size());
  Probe q(false);
  q.settings.load();
  q.settings.applySaved();
  const std::vector<Value> got = values(q);
  CHECK(got.size() == 1 && has(got, 0, label, 2, text("port1")));
  CHECK(ok(q.op(rc::kOpState, {}, out)) && out[0] == rc::kStorageStatePartial && out[5] == rc::kStorageReasonItem);
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
  testInitialValues();
  testLabelsUnique();
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
