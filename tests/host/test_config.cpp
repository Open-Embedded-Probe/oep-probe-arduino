// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: oep.probe.config's named items (oep-if-probe-config §1-§4) beyond the shared vectors (test_vectors):
// - describe lists the settings in this probe's kind order - the k-th is item number k - with the storage and the netif;
// - a save is read back by name: a firmware whose list moved (the pass-through gone, an own setting added) applies the
//   same items under its own numbers; a name it does not know leaves the rest applied (state 3, reason 2); a broken
//   container applies nothing (state 2, reason 1);
// - wifi / ipv4 go to the station only when they change; the passphrase is kept with pass_len 0xFF and never shown;
// - the sketch's own settings by form and flags (addSetting refuses a declaration outside §1.6 / §4);
// - a set past what the probe holds is refused cause 2 with nothing changed.
#include <stdio.h>
#include <string.h>

#include <memory>
#include <string>
#include <vector>

#include "OepConfig.h"
#include "OepEndpoint.h"
#include "OepFrame.h"
#include "OepPinTable.h"

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

// The station the wifi / ipv4 items go to: what it was handed, how often.
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

// One boot of a probe: an endpoint with pins 0-15, the config, optionally the station and own settings.
struct Probe {
  NullStream s;
  uint8_t rx[1100], tx[1100];
  Endpoint ep{s, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 4}, Endpoint::kVendorBulk, 0};
  PinTable pins{0xFFFFull};
  Binds binds;
  ProbeConfig config{ep, binds};
  FakeStation station;
  explicit Probe(bool wifi = true) {
    config.setPins(&pins);
    if (wifi) CHECK(config.setWifi(&station));
  }
  Result op(uint8_t code, const Bytes &p, Bytes &out) {
    out.assign(1024, 0);
    const Result r = config.handle(code, p.data(), p.size(), out.data(), out.size());
    out.resize(r.length);
    return r;
  }
  // The settings describe lists, in order (the k-th is number k).
  std::vector<std::string> settings() {
    uint8_t d[1024];
    const size_t n = config.describe(d, sizeof d);
    std::vector<std::string> names;
    for (size_t at = 0; at + kTlvHeader <= n; at += kTlvHeader + getU16(d + at + 1))
      if (d[at] == cfg::kTlvDescribeSetting) names.emplace_back(reinterpret_cast<const char *>(d + at + kTlvHeader + 5),
                                                                getU16(d + at + 1) - 5);
    return names;
  }
  uint8_t number(const char *name) {   // looked up by name, as a host does (core §7.7)
    const std::vector<std::string> names = settings();
    for (size_t k = 0; k < names.size(); ++k) if (names[k] == name) return static_cast<uint8_t>(k + 1);
    return 0;
  }
};

static bool ok(const Result &r) { return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess; }
// One item: number len(u16) key(u16) data (§1).
static Bytes item(uint8_t number, uint16_t key, const Bytes &data) {
  Bytes b = {number, uint8_t(data.size() + 2), uint8_t((data.size() + 2) >> 8), uint8_t(key), uint8_t(key >> 8)};
  b.insert(b.end(), data.begin(), data.end());
  return b;
}
static Bytes set(std::initializer_list<Bytes> items) {
  Bytes p = {uint8_t(items.size())};
  for (const Bytes &i : items) p.insert(p.end(), i.begin(), i.end());
  return p;
}
static Bytes text(const char *t) { return Bytes(t, t + strlen(t)); }
// get's items (number key data), every page from first 0.
struct Item { uint8_t number; uint16_t key; Bytes data; };
static std::vector<Item> items(Probe &p) {
  std::vector<Item> all;
  Bytes out;
  for (uint16_t first = 0;;) {
    const Result r = p.op(cfg::kOpGet, {uint8_t(first), uint8_t(first >> 8)}, out);
    if (!ok(r) || out.size() < 6) return all;
    size_t at = 6;
    for (uint8_t k = 0; k < out[5]; ++k) {
      const size_t len = getU16(&out[at + 1]);
      all.push_back({out[at], getU16(&out[at + 3]), Bytes(out.begin() + at + 5, out.begin() + at + 3 + len)});
      at += kTlvHeader + len;
    }
    if (!out[0]) return all;
    first = static_cast<uint16_t>(first + out[5]);
  }
}
static bool has(const std::vector<Item> &all, uint8_t number, uint16_t key, const Bytes &data) {
  for (const Item &i : all) if (i.number == number && i.key == key && i.data == data) return true;
  return false;
}

// describe (§4): storage, one setting per kind in this probe's order (form 0 for the standard names), the netif.
static void testDescribe() {
  fakeConfigStore(nullptr, 0);
  Probe p;
  CHECK(p.config.addSetting("com.example.ntp-server", cfg::kSettingFormText, 1, 32));
  CHECK(p.settings() == (std::vector<std::string>{"label", "idle", "disable", "wifi", "ipv4", "com.example.ntp-server"}));
  CHECK(p.config.numberOf(config_kind::kWifi) == p.number("wifi") && p.number("wifi") == 4);
  uint8_t d[512];
  const size_t n = p.config.describe(d, sizeof d);
  bool storage = false, netif = false;
  for (size_t at = 0; at + kTlvHeader <= n; at += kTlvHeader + getU16(d + at + 1)) {
    storage |= d[at] == cfg::kTlvDescribeStorage && getU32(d + at + kTlvHeader) == ProbeConfig::kMaxItems;
    netif |= d[at] == cfg::kTlvDescribeNetif && getU16(d + at + 1) == strlen(cfg::kNameNetifWifi) &&
             memcmp(d + at + kTlvHeader, cfg::kNameNetifWifi, strlen(cfg::kNameNetifWifi)) == 0;
    if (d[at] == cfg::kTlvDescribeSetting) {
      const bool own = memchr(d + at + kTlvHeader + 5, '.', getU16(d + at + 1) - 5) != nullptr;
      CHECK((d[at + kTlvHeader] == cfg::kSettingFormNamed) != own);   // form 0 only for the standard names
    }
  }
  CHECK(storage && netif);
  Probe q(false);   // no station: no wifi, no ipv4, no netif, no network op
  CHECK(q.settings() == (std::vector<std::string>{"label", "idle", "disable"}));
  CHECK(!q.config.offers(cfg::kOpNetwork) && p.config.offers(cfg::kOpNetwork));
}

// A save is read back by name (§1): a firmware whose list moved applies the same items under its own numbers.
static void testSavedByName() {
  fakeConfigStore(nullptr, 0);
  Bytes out;
  {
    Probe a(false);   // label 1, idle 2, disable 3
    CHECK(ok(a.op(cfg::kOpSet, set({item(a.number("label"), 5, text("DUT RX")), item(a.number("idle"), 6, {1})}), out)));
    CHECK(ok(a.op(cfg::kOpSave, {}, out)));
  }
  Probe b;   // the station and an own setting too: the same names, other places after them
  CHECK(b.config.addSetting("io.example.mode", cfg::kSettingFormU32));
  b.config.load();
  b.config.applySaved();
  const std::vector<Item> got = items(b);
  CHECK(has(got, b.number("label"), 5, text("DUT RX")) && has(got, b.number("idle"), 6, {1}));
  CHECK(b.pins.idle(6) == PinTable::kIdlePullUp);
  CHECK(ok(b.op(cfg::kOpState, {}, out)) && out[0] == cfg::kStorageStateApplied && out[5] == 0);
  // b saves its own setting with the rest; a firmware without that name applies the rest: state 3, reason 2
  CHECK(ok(b.op(cfg::kOpSet, set({item(b.number("io.example.mode"), 0, {7, 0, 0, 0})}), out)));
  CHECK(ok(b.op(cfg::kOpSave, {}, out)));
  Probe c(false);
  c.config.load();
  c.config.applySaved();
  CHECK(has(items(c), c.number("label"), 5, text("DUT RX")) && c.pins.idle(6) == PinTable::kIdlePullUp);
  CHECK(ok(c.op(cfg::kOpState, {}, out)) && out[0] == cfg::kStorageStatePartial && out[5] == cfg::kStorageUnreadableItem);
}

// A container whose CRC does not match applies nothing: state 2, reason 1 (§3).
static void testBrokenContainer() {
  const uint8_t junk[] = {5, 'l', 'a', 'b', 'e', 'l', 1, 0, 3, 0, 5, 0, 'x', 0, 0, 0, 0};
  fakeConfigStore(junk, sizeof junk);
  Probe p(false);
  p.config.load();
  p.config.applySaved();
  Bytes out;
  CHECK(items(p).empty());
  CHECK(ok(p.op(cfg::kOpState, {}, out)) && out[0] == cfg::kStorageStateUnreadable && out[5] == cfg::kStorageUnreadableForm);
}

// wifi / ipv4 (§1.3-§1.5): handed to the station only when they change; pass_len 0xFF keeps the passphrase; get never
// shows it; a label change hands nothing.
static void testWifi() {
  fakeConfigStore(nullptr, 0);
  Probe p;
  p.config.applySaved();   // boot: the station gets the (empty) list once
  const int boot = p.station.applies;
  Bytes out;
  const Bytes lab = {3, 'l', 'a', 'b', 9, 'p', 'a', 's', 's', 'w', 'o', 'r', 'd', '1'};
  CHECK(ok(p.op(cfg::kOpSet, set({item(p.number("wifi"), 0, lab)}), out)));
  CHECK(p.station.applies == boot + 1 && p.station.entries.size() == 1 && p.station.entries[0].pass_length == 9);
  CHECK(ok(p.op(cfg::kOpSet, set({item(p.number("label"), 3, text("x"))}), out)) && p.station.applies == boot + 1);
  const std::vector<Item> got = items(p);
  CHECK(has(got, p.number("wifi"), 0, {3, 'l', 'a', 'b', 0xFF}));   // pass_len 0xFF alone
  CHECK(ok(p.op(cfg::kOpSet, set({item(p.number("wifi"), 0, {3, 'l', 'a', 'b', 0xFF})}), out)));   // sent back: kept
  CHECK(p.station.applies == boot + 1 && p.station.entries[0].pass_length == 9 &&
        memcmp(p.station.entries[0].pass, "password1", 9) == 0);
  const Bytes fixed = {cfg::kIpv4ModeStatic, 192, 168, 1, 50, 24, 192, 168, 1, 1, 0, 0, 0, 0};
  CHECK(ok(p.op(cfg::kOpSet, set({item(p.number("ipv4"), 1, fixed)}), out)));
  CHECK(p.station.applies == boot + 2 && p.station.ipv4.mode == cfg::kIpv4ModeStatic && p.station.ipv4.prefix == 24);
  CHECK(ok(p.op(cfg::kOpUnset, {1, p.number("wifi"), 0, 0}, out)) && p.station.applies == boot + 3 && p.station.entries.empty());
}

// The sketch's own settings (§1.6, §1.7, §4): by form; a secret's value write-only and kept by 0 bytes; a key past keys
// unsupported; declarations outside the rules refused.
static void testOwnSettings() {
  fakeConfigStore(nullptr, 0);
  Probe p(false);
  CHECK(!p.config.addSetting("ntp-server", cfg::kSettingFormText, 1, 32));                  // no dot: a standard name's form
  CHECK(!p.config.addSetting("com.example.x", cfg::kSettingFormNamed));                     // form 0: the standard names' only
  CHECK(!p.config.addSetting("com.example.x", cfg::kSettingFormText, 1, 0));                // text needs a max_len
  CHECK(!p.config.addSetting("com.example.x", cfg::kSettingFormU32, 1, 4));                 // max_len only for bytes / text
  CHECK(!p.config.addSetting("com.example.x", cfg::kSettingFormBool, 1, 0, 0x80));          // a flag not defined
  CHECK(p.config.addSetting("com.example.flag", cfg::kSettingFormBool, 2));
  CHECK(p.config.addSetting("com.example.token", cfg::kSettingFormText, 1, 16, cfg::kSettingFlagSecret));
  CHECK(!p.config.addSetting("com.example.flag", cfg::kSettingFormBool));                   // a name twice
  Bytes out;
  const uint8_t flag = p.number("com.example.flag"), token = p.number("com.example.token");
  CHECK(ok(p.op(cfg::kOpSet, set({item(flag, 1, {1})}), out)));
  Result r = p.op(cfg::kOpSet, set({item(flag, 0, {2})}), out);   // bool 2: malformed
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
  r = p.op(cfg::kOpSet, set({item(flag, 2, {1})}), out);           // key 2 of keys 2: unsupported, the number
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnsupported && out == Bytes({flag}));
  r = p.op(cfg::kOpSet, set({item(token, 0, {})}), out);           // a secret's 0 bytes with nothing there: malformed
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectMalformed);
  CHECK(ok(p.op(cfg::kOpSet, set({item(token, 0, text("s3cret"))}), out)));
  CHECK(has(items(p), token, 0, {}));                             // get shows the key alone
  CHECK(ok(p.op(cfg::kOpSet, set({item(token, 0, {})}), out)));   // 0 bytes: kept
  size_t len = 0;
  const uint8_t *v = p.config.setting("com.example.token", 0, len);
  CHECK(v && len == 6 && memcmp(v, "s3cret", 6) == 0);
  v = p.config.setting("com.example.flag", 1, len);
  CHECK(v && len == 1 && v[0] == 1);
}

// More than the probe holds (kMaxItems): unavailable cause 2, nothing changed (§2).
static void testCapacity() {
  fakeConfigStore(nullptr, 0);
  Probe p(false);
  CHECK(p.config.addSetting("com.example.blob", cfg::kSettingFormBytes, 64, 64));
  Bytes out;
  const uint8_t blob = p.number("com.example.blob");
  const size_t fit = ProbeConfig::kMaxItems / (kTlvHeader + 2 + 64);   // whole items the store holds
  uint32_t generation = 0;
  for (uint16_t k = 0; k < fit; ++k) {
    CHECK(ok(p.op(cfg::kOpSet, set({item(blob, k, Bytes(64, uint8_t(k)))}), out)));
    generation = getU32(out.data());
  }
  const Result r = p.op(cfg::kOpSet, set({item(blob, uint16_t(fit), Bytes(64, 0xEE))}), out);
  CHECK(r.resolution == kResolutionRejected && r.detail == kRejectUnavailable && out.size() >= 4 &&
        out[3] == reg::core::kUnavailableCauseLimit);
  CHECK(items(p).size() == fit);
  CHECK(ok(p.op(cfg::kOpGet, {0, 0}, out)) && getU32(out.data() + 1) == generation);   // nothing changed
}

int main() {
  testDescribe();
  testSavedByName();
  testBrokenContainer();
  testWifi();
  testOwnSettings();
  testCapacity();
  printf("config: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
