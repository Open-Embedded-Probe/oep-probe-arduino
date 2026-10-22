// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepSettings.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_CH32RV) || defined(OEP_HOST_FAKE_CONFIG)

#if defined(ARDUINO_ARCH_ESP32)
#include <Preferences.h>
#elif defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
#endif
#include <initializer_list>
#include <string.h>

#include "OepEndpoint.h"

namespace oep {

namespace {

using namespace setting_id;
namespace rc = reg::core;

// The save (core §13.7: by name, any form the probe likes): version(u8) then groups - category name_len(u8) name
// instance(u16) (the core: no name, instance 0), count(u8) items - an item name_len(u8) name count(u16) values - a value
// key_len(u8) key value_len(u8) value (references as names) - then CRC-32 of all of it (u32). A blob whose CRC does not
// match, or that does not end on a group, cannot be confirmed whole (storage_state 2, reason 1).
constexpr uint8_t kBlobVersion = 1;
constexpr size_t kMaxBlob = OEP_SMALL_RAM ? 512 : 2048;
#if defined(ARDUINO_ARCH_ESP32)
// NVS writes a key whole (old or new after a power loss); "oepcfg" was the settings' (dropped at the first save).
constexpr const char *kNvsNamespace = "oepset", *kNvsKey = "v1";
bool storeRead(uint8_t *blob, size_t capacity, size_t &length) {
  Preferences p;
  if (!p.begin(kNvsNamespace, true)) return false;
  length = p.getBytesLength(kNvsKey);
  const bool ok = length > 0 && length <= capacity && p.getBytes(kNvsKey, blob, length) == length;
  p.end();
  return ok;
}
bool storeWrite(const uint8_t *blob, size_t length) {
  Preferences old;
  if (old.begin("oepcfg", false)) { old.clear(); old.end(); }
  Preferences p;
  if (!p.begin(kNvsNamespace, false)) return false;
  const size_t written = p.putBytes(kNvsKey, blob, length);
  p.end();
  return written == length;
}
#elif defined(OEP_HOST_FAKE_CONFIG)
uint8_t gFakeBlob[kMaxBlob];
size_t gFakeLength = 0;
bool storeRead(uint8_t *blob, size_t capacity, size_t &length) {
  length = gFakeLength;
  if (!length || length > capacity) return false;
  memcpy(blob, gFakeBlob, length);
  return true;
}
bool storeWrite(const uint8_t *blob, size_t length) {
  if (length > sizeof gFakeBlob) return false;
  if (length) memcpy(gFakeBlob, blob, length);
  gFakeLength = length;
  return true;
}
}  // namespace
void fakeConfigStore(const uint8_t *blob, size_t length) { storeWrite(blob, length); }
namespace {
#elif defined(ARDUINO_ARCH_CH32RV)
// ArduinoCore-CH32RV has no store yet: nothing saved, nothing read - no save, no state, and none of their code linked
constexpr bool kNoStore = true;
bool storeRead(uint8_t *, size_t, size_t &length) { length = 0; return false; }
bool storeWrite(const uint8_t *, size_t) { return false; }
#else
// The flash's last sector (arduino-pico EEPROM): magic, length, the blob. A sector rewrite is not atomic under a power
// loss; the CRC makes a torn one read as unreadable (reason 1) rather than as settings (core §13.7).
constexpr uint32_t kEepromMagic = 0x3153454f;   // "OES1"
constexpr size_t kEepromHeader = 6;             // magic(u32) length(u16)
bool storeRead(uint8_t *blob, size_t capacity, size_t &length) {
  EEPROM.begin(kEepromHeader + kMaxBlob);
  const uint8_t *e = EEPROM.getConstDataPtr();
  length = getU16(e + 4);
  if (getU32(e) != kEepromMagic || length == 0 || length > capacity) return false;
  memcpy(blob, e + kEepromHeader, length);
  return true;
}
bool storeWrite(const uint8_t *blob, size_t length) {
  EEPROM.begin(kEepromHeader + kMaxBlob);
  uint8_t *e = EEPROM.getDataPtr();
  putU32(e, kEepromMagic);
  putU16(e + 4, static_cast<uint16_t>(length));
  memcpy(e + kEepromHeader, blob, length);
  return EEPROM.commit();
}
#endif

#if !defined(ARDUINO_ARCH_CH32RV)
constexpr bool kNoStore = false;
#endif

bool nextItem(const uint8_t *store, size_t length, size_t &at, uint8_t &tag, const uint8_t *&value, size_t &vlen) {
  size_t next = 0;
  if (!tlvAt(store, length, at, tag, value, vlen, next)) return false;
  at = next;
  return true;
}

// rejected unsupported about the request's index-th value: the fixed-part marker, then item (core §4.3, §13.5)
Result unsupportedItem(uint8_t *out, size_t capacity, uint8_t index) {
  if (capacity < 5) return unsupportedValue(out, capacity);
  out[0] = kTagValue;
  putTlvHeader(out + 1, rc::kTlvUnsupportedPayloadItem, 1);
  out[4] = index;
  return {kResolutionRejected, kRejectUnsupported, 5};
}

struct Choice { uint8_t value; const char *name; };
// The choices' names: the registry's enum keys with '-' for '_' (core §13.2)
constexpr Choice kIdleChoices[] = {{rc::kChannelIdleHiZ, "hi-z"}, {rc::kChannelIdlePullUp, "pull-up"},
                                   {rc::kChannelIdlePullDown, "pull-down"}, {rc::kChannelIdleLow, "low"},
                                   {rc::kChannelIdleHigh, "high"}};
constexpr Choice kModeChoices[] = {{rc::kIpv4ModeDhcp, "dhcp"}, {rc::kIpv4ModeStatic, "static"}};
constexpr Choice kRestChoices[] = {{reg::wire_rvswd::kRestAuto, "auto"}, {reg::wire_rvswd::kRestHigh, "high"},
                                   {reg::wire_rvswd::kRestLow, "low"}};
// A route's short name by kind (core §13.3)
constexpr Choice kRouteNames[] = {{rc::kRouteKindBridge, "bridge"}, {rc::kRouteKindCdc, "cdc"}, {rc::kRouteKindUsj, "usj"},
                                  {rc::kRouteKindBulk, "bulk"}, {rc::kRouteKindHid, "hid"}, {rc::kRouteKindTcp, "tcp"}};

// A UART format's name: data bits, parity (n e o), stop bits ("8n1"; fixture §2.1). false: not a defined format.
bool formatName(uint8_t format, char *out) {
  if (!FixtureUart::formatDefined(format)) return false;
  out[0] = (format & 0x03) ? '7' : '8';
  out[1] = "neo"[(format >> 2) & 0x03];
  out[2] = (format & 0x10) ? '2' : '1';
  out[3] = 0;
  return true;
}

bool sameText(const uint8_t *a, size_t n, const char *b) { return strlen(b) == n && memcmp(a, b, n) == 0; }

uint8_t gBlob[kMaxBlob];   // the save as written or read (save, readBlob: never at once)

struct PassContext { Settings *settings; uint8_t port; };
PassContext gPassContext[Settings::kMaxPorts];
// The holds' and rests' contexts: the settings and the wire whose items they read
struct WireContext { Settings *settings; const Interface *wire; };
WireContext gHoldContext[Settings::kMaxPlaces + 2], gRestContext[Settings::kMaxPlaces];
size_t gRestCount = 0;

}  // namespace

uint32_t crc32Ieee(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

uint8_t Settings::scratch_[Settings::kMaxItems];

Settings::Settings(Endpoint &endpoint, Binds &binds) : endpoint_(endpoint), binds_(binds) {
  for (size_t p = 0; p < kMaxPorts; ++p) pass_uart_[p] = pass_console_[p] = -1;
  endpoint_.setSettings(this);
}

// ---- the entries ------------------------------------------------------------------------------------------------------

bool Settings::addEntry(uint8_t id, const Interface *owner, void *context, uint8_t own) {
  if (entry_count_ == kMaxEntries) return false;
  entries_[entry_count_++] = {owner, context, id, own};
  return true;
}

uint8_t Settings::entryOf(uint8_t id, const void *owner) const {
  for (uint8_t e = 0; e < entry_count_; ++e)
    if (entries_[e].id == id && entries_[e].owner == owner) return e;
  return kMaxEntries;
}

Settings::Spec Settings::spec(uint8_t entry) const {
  namespace k = reg::core;
  const Entry &e = entries_[entry];
  switch (e.id) {
    case kLabel: return {k::kSettingChannelLabel, k::kSettingFormText, k::kSettingKeyKindChannel, 32, 0, 0};
    case kIdle: return {k::kSettingChannelIdle, k::kSettingFormEnum, k::kSettingKeyKindChannel, 0, 0, 0};
    case kDisable: return {k::kSettingChannelDisable, k::kSettingFormBool, k::kSettingKeyKindChannel, 0, 0, 0};
    case kSsid: return {k::kSettingWifiSsid, k::kSettingFormText, k::kSettingKeyKindIndex, 32, 0, kMaxWifi};
    case kPassphrase:
      return {k::kSettingWifiPassphrase, k::kSettingFormText, k::kSettingKeyKindIndex, 64, k::kSettingFlagSecret, kMaxWifi};
    case kIpMode: return {k::kSettingIpv4Mode, k::kSettingFormEnum, k::kSettingKeyKindNetif, 0, 0, 0};
    case kIpAddress: return {k::kSettingIpv4Address, k::kSettingFormIpv4, k::kSettingKeyKindNetif, 0, 0, 0};
    case kIpPrefix: return {k::kSettingIpv4Prefix, k::kSettingFormU32, k::kSettingKeyKindNetif, 0, 0, 0};
    case kIpGateway: return {k::kSettingIpv4Gateway, k::kSettingFormIpv4, k::kSettingKeyKindNetif, 0, 0, 0};
    case kIpDns: return {k::kSettingIpv4Dns, k::kSettingFormIpv4, k::kSettingKeyKindNetif, 0, 0, 0};
    case kRest: return {reg::wire_rvswd::kSettingRest, k::kSettingFormEnum, k::kSettingKeyKindChannel, 0, 0, 0};
    case kHold: return {reg::wire_rvswd::kSettingHold, k::kSettingFormBool, k::kSettingKeyKindChannel, 0, 0, 0};
    case kPassRx:
      return {reg::fixture_uart::kSettingPassthroughRx, k::kSettingFormChannel, k::kSettingKeyKindTransport, 0, 0, 0};
    case kPassBaud:
      return {reg::fixture_uart::kSettingPassthroughBaud, k::kSettingFormU32, k::kSettingKeyKindTransport, 0, 0, 0};
    case kPassFormat:
      return {reg::fixture_uart::kSettingPassthroughFormat, k::kSettingFormEnum, k::kSettingKeyKindTransport, 0, 0, 0};
    case kConWire:
      return {reg::target_console::kSettingPassthroughWire, k::kSettingFormFn, k::kSettingKeyKindTransport, 0, 0, 0};
    case kConSwdio:
      return {reg::target_console::kSettingPassthroughSwdio, k::kSettingFormChannel, k::kSettingKeyKindTransport, 0, 0, 0};
    case kConSwclk:
      return {reg::target_console::kSettingPassthroughSwclk, k::kSettingFormChannel, k::kSettingKeyKindTransport, 0, 0, 0};
    case kConSpeed:
      return {reg::target_console::kSettingPassthroughMaxSpeed, k::kSettingFormU32, k::kSettingKeyKindTransport, 0, 0, 0};
    case kConMechanism:
      return {reg::target_console::kSettingPassthroughMechanism, k::kSettingFormText, k::kSettingKeyKindTransport, 32, 0, 0};
    default: {
      const Own &o = own_[e.own];
      return {o.name, o.form, o.keys ? k::kSettingKeyKindIndex : k::kSettingKeyKindNone, o.max_len, o.flags, o.keys};
    }
  }
}

uint16_t Settings::ownerFn(uint8_t entry) const {
  const Interface *o = entries_[entry].owner;
  return o ? endpoint_.fnOf(*o) : 0;
}

uint16_t Settings::fnOfEntry(uint8_t entry) const { return entry < entry_count_ ? ownerFn(entry) : 0xFFFF; }

uint8_t Settings::numberOfEntry(uint8_t entry) const {
  uint8_t n = 0;
  for (uint8_t e = 0; e <= entry && e < entry_count_; ++e) n += entries_[e].owner == entries_[entry].owner;
  return n;
}

uint8_t Settings::entryAt(uint16_t fn, uint8_t number) const {
  uint8_t n = 0;
  for (uint8_t e = 0; e < entry_count_; ++e)
    if (ownerFn(e) == fn && (fn != 0 || !entries_[e].owner) && ++n == number) return e;
  return kMaxEntries;
}

uint8_t Settings::entryNamed(const Interface *owner, const char *name, size_t length) const {
  for (uint8_t e = 0; e < entry_count_; ++e)
    if (entries_[e].owner == owner && sameText(reinterpret_cast<const uint8_t *>(name), length, spec(e).name)) return e;
  return kMaxEntries;
}

bool Settings::addPlace(WireRvswd &wire, TargetConsoleStream &console) {
  if (place_count_ == kMaxPlaces) return false;
  Place &p = places_[place_count_++];
  p.wire = &wire;
  p.console = &console;
  addHold(wire, wire.port().hold, checkHoldOf, &wire.port());
  if (wire.isRvswd() && entryOf(kRest, &wire) == kMaxEntries && gRestCount < kMaxPlaces) {
    addEntry(kRest, &wire, nullptr);   // its rest can be fixed per port (oep-if-debug §3.1)
    gRestContext[gRestCount] = {this, &wire};
    wire.port().rest_setting = &Settings::restOf;
    wire.port().rest_context = &gRestContext[gRestCount++];
  }
  if (pass_run_ && entryOf(kConWire, &console) == kMaxEntries) {   // the console's pass-through (console §2.1)
    for (uint8_t id = kConWire; id <= kConMechanism; ++id) addEntry(id, &console, &p);
  }
  return true;
}

bool Settings::addHold(const Interface &wire, WireHold &hold, void (*check)(void *), void *port) {
  for (size_t k = 0; k < hold_count_; ++k) if (holds_[k].port == port) return true;   // one wire, several places
  if (hold_count_ == kMaxHolds) return false;
  gHoldContext[hold_count_] = {this, &wire};
  hold.on = &Settings::holdOf;
  hold.context = &gHoldContext[hold_count_];
  holds_[hold_count_++] = {&wire, check, port};
  addEntry(kHold, &wire, nullptr);
  return true;
}

bool Settings::addUart(FixtureUart &uart) {
  if (uart_count_ == kMaxUarts || !pass_run_) return false;
  uarts_[uart_count_++] = &uart;
  for (uint8_t id = kPassRx; id <= kPassFormat; ++id) addEntry(id, &uart, &uart);
  return true;
}

void Settings::setPins(PinTable *pins) {
  pins_ = pins;
  endpoint_.setPins(pins);
  if (pins && entryOf(kLabel) == kMaxEntries)
    for (uint8_t id = kLabel; id <= kDisable; ++id) addEntry(id, nullptr, nullptr);
}

bool Settings::setWifi(WifiControl *wifi) {
  if (wifi && endpoint_.maxFrame() < reg::kLimitWifiMinMaxFrame) return false;
  wifi_ = wifi;
  // the Wi-Fi's part reached only from here: a sketch that hands no WifiControl links none of it
  wifi_hand_ = wifi ? &Settings::handWifi : nullptr;
  wifi_net_ = wifi ? &Settings::network : nullptr;
  if (wifi && entryOf(kSsid) == kMaxEntries)
    for (uint8_t id = kSsid; id <= kIpDns; ++id) addEntry(id, nullptr, nullptr);
  return true;
}

bool Settings::addSetting(const Interface &owner, const char *name, uint8_t form, uint16_t keys, uint8_t max_len,
                          uint8_t flags) {
  namespace k = reg::core;
  const size_t n = strlen(name);
  size_t braces = 0;
  bool ok = n >= 1 && n <= reg::kLimitSettingNameMaxBytes && form >= k::kSettingFormBytes && form <= k::kSettingFormBool;
  for (size_t i = 0; ok && i < n; ++i) {
    const char c = name[i];
    if (c == '{') { ok = i + 1 < n && name[i + 1] == '}'; ++braces; ++i; continue; }
    ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '.';
  }
  const bool sized = form == k::kSettingFormBytes || form == k::kSettingFormText;
  // max_len only for bytes / text, where one value's get fits the smallest max_frame (core §13.2: max_frame - 19)
  if (!ok || braces != (keys ? 1u : 0u) || own_count_ == kMaxOwn ||
      (sized ? max_len == 0 || reg::kLimitSettingGetOverheadBytes + max_len > endpoint_.maxFrame() : max_len != 0) ||
      (flags & ~(k::kSettingFlagSecret | k::kSettingFlagRestart)) || entryNamed(&owner, name, n) != kMaxEntries)
    return false;
  Own &o = own_[own_count_];
  o.name = name;
  o.form = form;
  o.keys = keys;
  o.max_len = max_len;
  o.flags = flags;
  if (!addEntry(kOwn, &owner, nullptr, static_cast<uint8_t>(own_count_))) return false;
  ++own_count_;
  return true;
}

bool Settings::firmwareValue(const Interface *owner, const char *name, uint16_t key, const uint8_t *data, size_t length,
                             bool fixed) {
  const uint8_t e = entryNamed(owner, name, strlen(name));
  if (e == kMaxEntries || length > 64) return false;
  uint8_t v[3 + 64];
  putU16(v, key);
  v[2] = fixed ? reg::core::kSettingOriginFixed : reg::core::kSettingOriginDefault;
  memcpy(v + 3, data, length);
  return insert(firm_, firm_length_, sizeof firm_, e, v, 3 + length);
}

const uint8_t *Settings::firm(uint8_t entry, uint16_t key, uint8_t &origin, size_t &length) const {
  size_t vlen = 0;
  const uint8_t *v = find(firm_, firm_length_, entry, key, vlen);
  if (!v) return nullptr;
  origin = v[2];
  length = vlen - 3;
  return v + 3;
}

const uint8_t *Settings::value(uint8_t entry, uint16_t key, size_t &length) const {
  size_t vlen = 0;
  if (const uint8_t *v = find(items_, items_length_, entry, key, vlen)) { length = vlen - 2; return v + 2; }
  uint8_t origin = 0;
  return firm(entry, key, origin, length);
}

const uint8_t *Settings::setting(const Interface &owner, const char *name, uint16_t key, size_t &length) const {
  const uint8_t e = entryNamed(&owner, name, strlen(name));
  return e == kMaxEntries ? nullptr : value(e, key, length);
}

// ---- the stores --------------------------------------------------------------------------------------------------------

const uint8_t *Settings::find(const uint8_t *store, size_t length, uint8_t entry, uint16_t key, size_t &vlen) {
  size_t at = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  while (nextItem(store, length, at, tag, v, vlen))
    if (tag == entry + 1 && vlen >= 2 && getU16(v) == key) return v;
  return nullptr;
}

bool Settings::insert(uint8_t *store, size_t &length, size_t capacity, uint8_t entry, const uint8_t *v, size_t len) {
  const uint16_t key = getU16(v);
  remove(store, length, entry, key);
  size_t at = 0, place = length, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *w = nullptr;
  while (at < length) {   // after every value of a smaller entry, or of this one with a smaller key
    const size_t here = at;
    if (!nextItem(store, length, at, tag, w, vlen)) break;
    if (tag > entry + 1 || (tag == entry + 1 && getU16(w) > key)) { place = here; break; }
  }
  if (length + kTlvHeader + len > capacity) return false;
  memmove(store + place + kTlvHeader + len, store + place, length - place);
  putTlvHeader(store + place, static_cast<uint8_t>(entry + 1), static_cast<uint16_t>(len));
  memcpy(store + place + kTlvHeader, v, len);
  length += kTlvHeader + len;
  return true;
}

void Settings::remove(uint8_t *store, size_t &length, uint8_t entry, uint16_t key) {
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  while (at < length) {
    const size_t here = at;
    if (!nextItem(store, length, at, tag, v, vlen)) return;
    if (tag == entry + 1 && vlen >= 2 && getU16(v) == key) {
      memmove(store + here, store + at, length - at);
      length -= at - here;
      return;
    }
  }
}

// A value of item `id` (owner: its interface, nullptr the core) and key in a store (user values only): its data.
const uint8_t *Settings::in(const uint8_t *items, size_t length, uint8_t id, const void *owner, uint16_t key,
                            size_t &n) const {
  const uint8_t e = entryOf(id, owner);
  if (e == kMaxEntries) return nullptr;
  size_t vlen = 0;
  const uint8_t *v = find(items, length, e, key, vlen);
  if (v) { n = vlen - 2; return v + 2; }
  uint8_t origin = 0;
  return firm(e, key, origin, n);
}

uint64_t Settings::disabledIn(const uint8_t *items, size_t length) const {
  uint64_t mask = 0;
  const uint8_t e = entryOf(kDisable);
  if (e == kMaxEntries) return 0;
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  while (nextItem(items, length, at, tag, v, vlen))
    if (tag == e + 1 && vlen == 3 && v[2] && getU16(v) < 64) mask |= uint64_t{1} << getU16(v);
  return mask;
}

uint8_t Settings::idlePlainOf(const uint8_t *items, size_t length, uint16_t channel) const {
  size_t n = 0;
  const uint8_t *d = in(items, length, kIdle, nullptr, channel, n);
  return d && n == 1 ? d[0] : PinTable::kIdleUnset;
}

// ---- names (core §13.3) ----------------------------------------------------------------------------------------------

size_t Settings::routeName(uint16_t index, char *out) const {
  uint8_t kind = 0;
  if (!endpoint_.transportKindOf(index, kind)) return 0;
  uint8_t ordinal = 0;
  for (uint16_t i = 0; i < index; ++i) {
    uint8_t k = 0;
    if (endpoint_.transportKindOf(i, k) && k == kind) ++ordinal;
  }
  for (const Choice &c : kRouteNames)
    if (c.value == kind) {
      const size_t n = putText(out, c.name);
      return n + putDecimal(out + n, ordinal);
    }
  return 0;
}

size_t Settings::keyName(uint8_t entry, uint16_t key, char *out) const {
  switch (spec(entry).key_kind) {
    case reg::core::kSettingKeyKindChannel: return platformChannelName(key, out);
    case reg::core::kSettingKeyKindIndex: return putDecimal(out, key);
    case reg::core::kSettingKeyKindTransport: return routeName(key, out);
    case reg::core::kSettingKeyKindNetif: return key == 1 ? putText(out, reg::core::kNameNetifWifi) : 0;
    default: return 0;
  }
}

bool Settings::keyNamed(uint8_t entry, const uint8_t *name, size_t length, uint16_t &key) const {
  char buf[24];
  const uint8_t kind = spec(entry).key_kind;
  if (kind == reg::core::kSettingKeyKindNone) { key = 0; return length == 0; }
  const uint16_t top = kind == reg::core::kSettingKeyKindChannel ? channels_
                       : kind == reg::core::kSettingKeyKindIndex ? spec(entry).keys
                       : kind == reg::core::kSettingKeyKindTransport ? static_cast<uint16_t>(endpoint_.transportCount()) + 1
                                                                     : 2;
  for (uint16_t k = 0; k < top; ++k) {
    const size_t n = keyName(entry, k, buf);
    if (n && n == length && memcmp(buf, name, n) == 0) { key = k; return true; }
  }
  return false;
}

// A value in the save's form: references and choices as names, everything else as on the wire.
size_t Settings::storedValue(uint8_t entry, const uint8_t *d, size_t n, uint8_t *out) const {
  char *s = reinterpret_cast<char *>(out);
  switch (spec(entry).form) {
    case reg::core::kSettingFormChannel: return n == 2 ? platformChannelName(getU16(d), s) : 0;
    case reg::core::kSettingFormFn: {
      const Interface *it = n == 2 ? endpoint_.interfaceAt(getU16(d)) : nullptr;
      if (!it) return 0;
      const size_t m = putText(s, it->name());
      s[m] = '/';
      return m + 1 + putDecimal(s + m + 1, it->instance());
    }
    case reg::core::kSettingFormEnum: {
      if (n != 1) return 0;
      const uint8_t id = entries_[entry].id;
      if (id == kPassFormat) return formatName(d[0], s) ? 3 : 0;
      const Choice *c = id == kIdle ? kIdleChoices : id == kIpMode ? kModeChoices : kRestChoices;
      const size_t count = id == kIdle ? 5 : id == kIpMode ? 2 : 3;
      for (size_t i = 0; i < count; ++i)
        if (c[i].value == d[0]) return putText(s, c[i].name);
      return 0;
    }
    default:
      memcpy(out, d, n);
      return n;
  }
}

bool Settings::wireValue(uint8_t entry, const uint8_t *s, size_t n, uint8_t *out, size_t &len) const {
  char buf[64];
  switch (spec(entry).form) {
    case reg::core::kSettingFormChannel:
      for (uint16_t ch = 0; ch < channels_; ++ch)
        if (platformChannelName(ch, buf) == n && memcmp(buf, s, n) == 0) { putU16(out, ch); len = 2; return true; }
      return false;
    case reg::core::kSettingFormFn:
      for (uint16_t fn = 1; endpoint_.interfaceAt(fn); ++fn) {
        const Interface *it = endpoint_.interfaceAt(fn);
        size_t m = putText(buf, it->name());
        buf[m++] = '/';
        m += putDecimal(buf + m, it->instance());
        if (m == n && memcmp(buf, s, n) == 0) { putU16(out, fn); len = 2; return true; }
      }
      return false;
    case reg::core::kSettingFormEnum:
      for (unsigned v = 0; v < 256; ++v) {
        const uint8_t b = static_cast<uint8_t>(v);
        const size_t m = storedValue(entry, &b, 1, reinterpret_cast<uint8_t *>(buf));
        if (m && m == n && memcmp(buf, s, n) == 0) { out[0] = b; len = 1; return true; }
      }
      return false;
    default:
      memcpy(out, s, n);
      len = n;
      return true;
  }
}

// ---- checks ------------------------------------------------------------------------------------------------------------

bool Settings::keyValid(uint8_t entry, uint16_t key) const {
  const Spec s = spec(entry);
  switch (s.key_kind) {
    case reg::core::kSettingKeyKindNone: return key == 0;
    case reg::core::kSettingKeyKindIndex: return key < s.keys;
    case reg::core::kSettingKeyKindNetif: return key == 1;   // "wifi", the one netif (core §7.7: from 1)
    case reg::core::kSettingKeyKindTransport: return key < kMaxPorts && endpoint_.isSerialPort(key);   // serial ports
    case reg::core::kSettingKeyKindChannel: return pins_ ? pins_->allowed(key) : key < channels_;
    default: return false;
  }
}

namespace {
// transports §7.1: 8-63 bytes of 0x20-0x7E, or 64 hex digits
bool passphraseForm(const uint8_t *d, size_t n) {
  if (n < 8 || n > 64) return false;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t c = d[i];
    const bool hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    if (n == 64 ? !hex : (c < 0x20 || c > 0x7E)) return false;
  }
  return true;
}
// transports §7.2: an address a host may have
bool addressForm(const uint8_t *d) {
  const uint32_t a = (uint32_t{d[0]} << 24) | (uint32_t{d[1]} << 16) | (uint32_t{d[2]} << 8) | d[3];
  return a != 0 && a != 0xFFFFFFFFu && d[0] != 127 && !(d[0] >= 224 && d[0] <= 239);
}
}  // namespace

Result Settings::checkValue(uint8_t entry, uint16_t key, const uint8_t *d, size_t n, uint8_t item, uint8_t *out,
                            size_t capacity) const {
  namespace k = reg::core;
  const Spec s = spec(entry);
  const uint8_t id = entries_[entry].id;
  // the form (malformed, core §13.3); a secret's 0 bytes keep its value (core §13.5)
  if (n == 0 && (s.flags & k::kSettingFlagSecret)) {
    size_t had = 0;
    return find(items_, items_length_, entry, key, had) ? completed() : rejected(kRejectMalformed);
  }
  switch (s.form) {
    case k::kSettingFormBytes: if (n > s.max_len) return rejected(kRejectMalformed); break;
    case k::kSettingFormText: if (n < 1 || n > s.max_len) return rejected(kRejectMalformed); break;   // length only (§13.3)
    case k::kSettingFormU32: case k::kSettingFormIpv4: if (n != 4) return rejected(kRejectMalformed); break;
    case k::kSettingFormBool: if (n != 1 || d[0] > 1) return rejected(kRejectMalformed); break;
    case k::kSettingFormEnum: case k::kSettingFormTransport: if (n != 1) return rejected(kRejectMalformed); break;
    case k::kSettingFormChannel: case k::kSettingFormFn: if (n != 2) return rejected(kRejectMalformed); break;
    default: return rejected(kRejectMalformed);
  }
  // what the definition leaves out (malformed)
  switch (id) {
    case kPassphrase: if (!passphraseForm(d, n)) return rejected(kRejectMalformed); break;
    case kIpAddress: if (!addressForm(d)) return rejected(kRejectMalformed); break;
    case kIpPrefix: if (getU32(d) < 1 || getU32(d) > 32) return rejected(kRejectMalformed); break;
    case kPassBaud: case kConSpeed: if (getU32(d) == 0) return rejected(kRejectMalformed); break;
    default: break;
  }
  // a fixed value cannot change (core §13.4); a key this probe has no value for (unsupported)
  uint8_t origin = 0;
  size_t fl = 0;
  if (!keyValid(entry, key) || (firm(entry, key, origin, fl) && origin == k::kSettingOriginFixed))
    return unsupportedItem(out, capacity, item);
  uint8_t name[8];
  if (s.form == k::kSettingFormEnum && !storedValue(entry, d, 1, name)) return unsupportedItem(out, capacity, item);
  if (s.form == k::kSettingFormChannel && (pins_ ? !pins_->allowed(getU16(d)) : getU16(d) >= channels_))
    return unsupportedItem(out, capacity, item);
  switch (id) {
    case kLabel: case kDisable: return completed();
    case kIdle: {
      const bool output = d[0] == PinTable::kIdleOutputLow || d[0] == PinTable::kIdleOutputHigh;
      const bool pull = d[0] == PinTable::kIdlePullUp || d[0] == PinTable::kIdlePullDown;
      if ((output && !pins_->canOutput(key)) || (pull && !pins_->canPull(key))) return unsupportedItem(out, capacity, item);
      return completed();
    }
    case kPassRx: {
      const FixtureUart *u = static_cast<const FixtureUart *>(entries_[entry].context);
      return u->rxAllowed(getU16(d)) ? completed() : unsupportedItem(out, capacity, item);
    }
    case kPassBaud: return FixtureUart::baudWithinReach(getU32(d)) ? completed() : unsupportedItem(out, capacity, item);
    case kPassFormat: {
      const FixtureUart *u = static_cast<const FixtureUart *>(entries_[entry].context);
      return u->formatDeclared(d[0]) ? completed() : unsupportedItem(out, capacity, item);
    }
    case kConWire: {   // a wire this console reads through (a place's)
      const Place *p = static_cast<const Place *>(entries_[entry].context);
      return endpoint_.fnOf(*p->wire) == getU16(d) ? completed() : unsupportedItem(out, capacity, item);
    }
    case kConMechanism: {
      const Place *p = static_cast<const Place *>(entries_[entry].context);
      for (uint8_t dr = 0; dr < TargetConsoleStream::kDrivers; ++dr)
        if (sameText(d, n, TargetConsoleStream::kDriverNames[dr]) && p->console->numberOf(dr)) return completed();
      return unsupportedItem(out, capacity, item);
    }
    default: return completed();
  }
}

// The whole after the change (core §13.5, §13.8, fixture §2.1, console §2.1): a disable on a channel in use (cause 1), an
// idle and a disable on one channel (cause 5), a pass-through on a disabled pin or one with an output idle (cause 5),
// two pass-throughs on a port (cause 5), a console pass-through's pins its wire does not allow (unsupported).
Result Settings::checkWhole(const uint8_t *items, size_t length, uint8_t *out, size_t capacity) const {
  const uint64_t disabled = disabledIn(items, length);
  auto isDisabled = [&](uint16_t ch) { return ch < 64 && ((disabled >> ch) & 1); };
  auto heldBy = [&](uint16_t ch) {
    const uint8_t idle = idlePlainOf(items, length, ch);
    return isDisabled(ch) || idle == PinTable::kIdleOutputLow || idle == PinTable::kIdleOutputHigh;
  };
  if (pins_) {
    for (uint16_t ch = 0; ch < PinTable::kChannels; ++ch) {
      if (!isDisabled(ch)) continue;
      if (!pins_->disabled(ch) && pins_->owner(ch) && pins_->owner(ch) != 0xff)
        return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, ch);
      if (idlePlainOf(items, length, ch) != PinTable::kIdleUnset)
        return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, ch);
    }
  }
  for (uint8_t port = 0; port < kMaxPorts; ++port) {
    int users = 0;
    size_t n = 0;
    for (size_t k = 0; k < uart_count_; ++k) {
      const uint8_t *rx = in(items, length, kPassRx, uarts_[k], port, n);
      if (!rx) continue;
      ++users;
      if (heldBy(getU16(rx))) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, getU16(rx));
    }
    for (size_t k = 0; k < place_count_; ++k) {
      const TargetConsoleStream *c = places_[k].console;
      const uint8_t *wire = in(items, length, kConWire, c, port, n);
      if (!wire) continue;
      ++users;
      size_t m = 0;
      const uint8_t *io = in(items, length, kConSwdio, c, port, m);
      const uint8_t *ck = in(items, length, kConSwclk, c, port, m);
      const uint16_t swdio = io ? getU16(io) : 0xffff, swclk = ck ? getU16(ck) : 0xffff;
      for (uint16_t ch : {swdio, swclk})
        if (ch != 0xffff && heldBy(ch)) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, ch);
      DebugPort &dp = places_[k].wire->port();
      if (io && (places_[k].wire->isRvswd() ? ck != nullptr : true) && !pairAllowed(dp, swdio, swclk))
        return unsupportedValue(out, capacity);
      const uint8_t *hz = in(items, length, kConSpeed, c, port, m);
      if (hz && dp.dm.phy().minClockHz() && getU32(hz) < dp.dm.phy().minClockHz()) return unsupportedValue(out, capacity);
    }
    if (users > 1) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings);
  }
  return completed();
}

// ---- the ops -----------------------------------------------------------------------------------------------------------

bool Settings::offers(uint8_t op) const {
  namespace k = reg::core;
  if (op == k::kOpGet || op == k::kOpSet || op == k::kOpUnset) return true;
  if (op == k::kOpNetwork) return wifi_ != nullptr;
  return !kNoStore && storage_ && (op == k::kOpSave || op == k::kOpState);
}

Result Settings::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  namespace k = reg::core;
  Tail tail;
  switch (op) {
    case k::kOpGet: return get(payload, length, out, capacity);
    case k::kOpSet: return set(payload, length, out, capacity);
    case k::kOpUnset: return unset(payload, length, out, capacity);
    case k::kOpSave: return kNoStore ? rejected(kRejectUnknownOperation) : save(payload, length, out, capacity);
    case k::kOpNetwork: return wifi_net_ ? (this->*wifi_net_)(payload, length, out, capacity) : rejected(kRejectUnknownOperation);
    case k::kOpState: {   // [TLV] -> storage_state(u8) saved_generation(u32) reason(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 6) return failed();
      out[0] = storage_state_;
      putU32(out + 1, saved_generation_);
      out[5] = reason_;
      return completed(6);
    }
    default: return rejected(kRejectUnknownOperation);
  }
}

Result Settings::commit(const uint8_t *candidate, size_t length, uint8_t *out, size_t capacity) {
  const Result whole = checkWhole(candidate, length, out, capacity);
  if (refused(whole)) return whole;
  if (capacity < 4) return failed();
  const bool same = length == items_length_ && memcmp(candidate, items_, length) == 0;
  if (!same) {   // a change: the generation goes on (none left at 0xFFFFFFFF, until a restart)
    if (generation_ == 0xFFFFFFFFu) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
    ++generation_;
    adopt(candidate, length);
  }
  putU32(out, generation_);
  return completed(4);
}

Result Settings::set(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // count(u8) count x (fn u16, number u8, key u16, len u16, data) [TLV]: the forms, then the outer TLVs (core §2.3)
  if (length < 1) return rejected(kRejectMalformed);
  const uint8_t count = payload[0];
  size_t at = 1;
  for (uint8_t i = 0; i < count; ++i) {
    if (at + 7 > length || at + 7 + getU16(payload + at + 5) > length) return rejected(kRejectMalformed);
    if (payload[at + 2] == 0) return rejected(kRejectMalformed);   // number 0
    for (size_t q = 1, j = 0; j < i; ++j) {   // the same (fn, number, key) twice
      if (memcmp(payload + q, payload + at, 5) == 0) return rejected(kRejectMalformed);
      q += 7 + getU16(payload + q + 5);
    }
    at += 7 + getU16(payload + at + 5);
  }
  Tail tail;
  const Result parsed = tail.parse(payload + at, length - at, out, capacity);
  if (refused(parsed)) return parsed;
  // every value: malformed first, then the first unsupported (core §4.3: checked before anything changes)
  Result first = completed();
  for (size_t p = 1, i = 0; i < count; ++i) {
    const uint16_t fn = getU16(payload + p), key = getU16(payload + p + 3), n = getU16(payload + p + 5);
    if (fn && !endpoint_.interfaceAt(fn)) return rejected(kRejectUnknownFunction);
    const uint8_t e = entryAt(fn, payload[p + 2]);
    const Result r = e == kMaxEntries ? unsupportedItem(out, capacity, static_cast<uint8_t>(i))
                                      : checkValue(e, key, payload + p + 7, n, static_cast<uint8_t>(i), out, capacity);
    if (r.resolution == kResolutionRejected && r.detail == kRejectMalformed) return r;
    if (refused(r) && !refused(first)) first = r;
    p += 7 + n;
  }
  if (refused(first)) {   // write the first one's payload again (a later check wrote over out)
    for (size_t p = 1, i = 0; i < count; ++i) {
      const uint16_t n = getU16(payload + p + 5);
      const uint8_t e = entryAt(getU16(payload + p), payload[p + 2]);
      const Result r = e == kMaxEntries ? unsupportedItem(out, capacity, static_cast<uint8_t>(i))
                                        : checkValue(e, getU16(payload + p + 3), payload + p + 7, n, static_cast<uint8_t>(i), out, capacity);
      if (refused(r)) return r;
      p += 7 + n;
    }
  }
  uint8_t *const candidate = scratch_;   // one work area for set / unset / applySaved (never at once)
  size_t clen = items_length_;
  memcpy(candidate, items_, clen);
  for (size_t p = 1, i = 0; i < count; ++i) {
    const uint8_t e = entryAt(getU16(payload + p), payload[p + 2]);
    const uint16_t n = getU16(payload + p + 5);
    if (n) {   // a secret's 0 bytes keep it (core §13.5): nothing to change
      uint8_t v[2 + 255];
      putU16(v, getU16(payload + p + 3));
      memcpy(v + 2, payload + p + 7, n);
      if (!insert(candidate, clen, kMaxItems, e, v, 2u + n))
        return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);   // more than this probe holds
    }
    p += 7 + n;
  }
  return commit(candidate, clen, out, capacity);
}

Result Settings::unset(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // count(u8) count x (fn u16, number u8, key u16) [TLV]
  if (length < 1 || length < 1u + 5u * payload[0]) return rejected(kRejectMalformed);
  const uint8_t count = payload[0];
  Tail tail;
  const Result parsed = tail.parse(payload + 1 + 5 * count, length - 1 - 5 * count, out, capacity);
  if (refused(parsed)) return parsed;
  for (uint8_t i = 0; i < count; ++i) {
    const uint8_t *r = payload + 1 + 5 * i;
    if (!r[2]) return rejected(kRejectMalformed);
    for (uint8_t j = 0; j < i; ++j)
      if (memcmp(payload + 1 + 5 * j, r, 5) == 0) return rejected(kRejectMalformed);   // twice
  }
  for (uint8_t i = 0; i < count; ++i) {
    const uint8_t *r = payload + 1 + 5 * i;
    if (getU16(r) && !endpoint_.interfaceAt(getU16(r))) return rejected(kRejectUnknownFunction);
    const uint8_t e = entryAt(getU16(r), r[2]);
    uint8_t origin = 0;
    size_t fl = 0;
    if (e == kMaxEntries || (firm(e, getU16(r + 3), origin, fl) && origin == reg::core::kSettingOriginFixed))
      return unsupportedItem(out, capacity, i);
  }
  uint8_t *const candidate = scratch_;
  size_t clen = items_length_;
  memcpy(candidate, items_, clen);
  for (uint8_t i = 0; i < count; ++i) {
    const uint8_t *r = payload + 1 + 5 * i;
    remove(candidate, clen, entryAt(getU16(r), r[2]), getU16(r + 3));
  }
  return commit(candidate, clen, out, capacity);
}

Result Settings::get(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // first(u16) [TLV] -> more(u8) generation(u32) count(u8) values [TLV]: whole values from the first-th, as many as fit;
  // by fn, number, key (core §13.5)
  Tail tail;
  const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
  if (refused(parsed)) return parsed;
  if (capacity < 6) return failed();
  const uint16_t first = getU16(payload);
  size_t index = 0, put = 6;
  uint8_t count = 0;
  bool more = false;
  uint16_t top = 0;
  for (uint8_t e = 0; e < entry_count_; ++e) if (ownerFn(e) > top) top = ownerFn(e);
  for (uint16_t fn = 0; fn <= top && !more; ++fn) {
    for (uint8_t number = 1; !more; ++number) {
      const uint8_t e = entryAt(fn, number);
      if (e == kMaxEntries) break;
      const bool secret = spec(e).flags & reg::core::kSettingFlagSecret;
      // the keys of both stores, in order
      for (uint32_t key = 0; key <= 0xFFFF && !more;) {
        size_t un = 0, fnl = 0;
        uint8_t origin = 0;
        const uint8_t *u = find(items_, items_length_, e, static_cast<uint16_t>(key), un);
        const uint8_t *f = firm(e, static_cast<uint16_t>(key), origin, fnl);
        if (u || f) {
          if (index++ >= first) {
            const uint8_t *d = u ? u + 2 : f;
            const size_t n = secret ? 0 : (u ? un - 2 : fnl);
            if (put + 8 + n > capacity || count == 255) { more = true; break; }
            putU16(out + put, fn);
            out[put + 2] = number;
            putU16(out + put + 3, static_cast<uint16_t>(key));
            out[put + 5] = u ? reg::core::kSettingOriginUser : origin;
            putU16(out + put + 6, static_cast<uint16_t>(n));
            memcpy(out + put + 8, d, n);
            put += 8 + n;
            ++count;
          }
        }
        // the next key either store has for this entry
        uint32_t next = 0x10000;
        for (const uint8_t *store : {static_cast<const uint8_t *>(items_), static_cast<const uint8_t *>(firm_)}) {
          const size_t slen = store == items_ ? items_length_ : firm_length_;
          size_t at = 0, vlen = 0;
          uint8_t tag = 0;
          const uint8_t *v = nullptr;
          while (nextItem(store, slen, at, tag, v, vlen))
            if (tag == e + 1 && getU16(v) > key && getU16(v) < next) next = getU16(v);
        }
        key = next;
      }
    }
  }
  out[0] = more ? 1 : 0;
  putU32(out + 1, generation_);
  out[5] = count;
  return completed(put);
}

// [TLV] -> tcp_port(u16) count(u8) count x (netif u8, state u8, entry u16, reason u8, rssi i8, address(4), prefix,
// gateway(4), dns(4)) [TLV] (transports §7.3): the one netif, "wifi".
Result Settings::network(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
  if (refused(parsed)) return parsed;
  if (capacity < 3 + 19) return failed();
  const WifiControl::Status s = wifi_->status();
  const bool up = s.state == WifiControl::kStateConnected;
  putU16(out, wifi_->tcpPort());
  out[2] = 1;
  uint8_t *r = out + 3;
  r[0] = 1;
  r[1] = s.state;
  putU16(r + 2, s.state == WifiControl::kStateConnecting || up ? s.entry : WifiControl::kNoEntry);
  r[4] = up ? 0 : s.reason;
  r[5] = up ? static_cast<uint8_t>(s.rssi) : 0;
  memset(r + 6, 0, 13);
  if (up) {
    memcpy(r + 6, s.ipv4, 4);
    r[10] = s.prefix;
    memcpy(r + 11, s.gateway, 4);
    memcpy(r + 15, s.dns, 4);
  }
  return completed(3 + 19);
}

void Settings::describe(uint16_t fn, TlvPager &put) const {
  namespace k = reg::core;
  if (fn == 0 && wifi_) put.text(k::kTlvDescribeNetif, k::kNameNetifWifi);
  for (uint8_t number = 1;; ++number) {
    const uint8_t e = entryAt(fn, number);
    if (e == kMaxEntries) break;
    const Spec s = spec(e);
    uint8_t v[6 + 48];
    const size_t n = strlen(s.name);
    v[0] = s.form;
    v[1] = s.key_kind;
    putU16(v + 2, s.keys);
    v[4] = s.max_len;
    v[5] = s.flags;
    memcpy(v + 6, s.name, n);
    put.put(reg::kDescribeSetting, v, 6 + n);
    if (s.form != k::kSettingFormEnum) continue;
    for (unsigned value = 0; value < 256; ++value) {   // its choices (core §13.2)
      uint8_t c[2 + 16];
      const uint8_t b = static_cast<uint8_t>(value);
      if (entries_[e].id == kPassFormat && !static_cast<const FixtureUart *>(entries_[e].context)->formatDeclared(b)) continue;
      if (entries_[e].id == kIdle && pins_ && b != PinTable::kIdleHiZ) {   // only the states some channel can make
        bool any = false;
        for (uint16_t ch = 0; ch < PinTable::kChannels && !any; ++ch)
          any = pins_->allowed(ch) && (b <= PinTable::kIdlePullDown ? pins_->canPull(ch) : pins_->canOutput(ch));
        if (!any) continue;
      }
      const size_t m = storedValue(e, &b, 1, c + 2);
      if (!m) continue;
      c[0] = number;
      c[1] = b;
      put.put(reg::kDescribeSettingChoice, c, 2 + m);
    }
  }
}

// ---- applying ------------------------------------------------------------------------------------------------------------

// The values made current: disables first (core §8: before any idle), then the idles of the free channels (a channel in
// use takes its new idle when it is freed), the Wi-Fi when its values changed, and the pass-throughs from the next poll.
void Settings::adopt(const uint8_t *items, size_t length) {
  if (items != items_) memmove(items_, items, length);
  items_length_ = length;
  if (wifi_hand_) (this->*wifi_hand_)();
  if (!pins_) return;
  const uint64_t disabled = disabledIn(items_, items_length_);
  pins_->setDisabled(disabled);
  endpoint_.setDisabled(disabled);
  for (uint16_t ch = 0; ch < PinTable::kChannels; ++ch) {
    if (!pins_->allowed(ch)) continue;
    const uint8_t mode = idlePlainOf(items_, items_length_, ch);
    if (pins_->idle(ch) != mode) pins_->setIdle(ch, mode);   // applied now to a free channel only (PinTable)
  }
}

// The Wi-Fi entries in key order and the netif's IPv4 to the station, when they differ from what it has (transports §7).
void Settings::handWifi() {
  if (!wifi_) return;
  WifiEntry entries[kMaxWifi];
  size_t count = 0;
  Ipv4Setting ip;
  uint32_t crc = 0xFFFFFFFFu;
  auto mix = [&](const uint8_t *d, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      crc ^= d[i];
      for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  };
  for (uint16_t key = 0; key < kMaxWifi; ++key) {
    size_t sn = 0, pn = 0;
    const uint8_t *ssid = in(items_, items_length_, kSsid, nullptr, key, sn);
    if (!ssid) continue;
    const uint8_t *pass = in(items_, items_length_, kPassphrase, nullptr, key, pn);
    WifiEntry &e = entries[count++];
    e.index = key;
    e.ssid_length = static_cast<uint8_t>(sn);
    memcpy(e.ssid, ssid, sn);
    e.pass_length = pass ? static_cast<uint8_t>(pn) : 0;
    if (pass) memcpy(e.pass, pass, pn);
    const uint8_t k[2] = {static_cast<uint8_t>(key), static_cast<uint8_t>(sn)};
    mix(k, 2);
    mix(ssid, sn);
    mix(&e.pass_length, 1);
    if (pass) mix(pass, pn);
  }
  size_t n = 0;
  if (const uint8_t *d = in(items_, items_length_, kIpMode, nullptr, 1, n)) ip.mode = d[0];
  if (ip.mode == reg::core::kIpv4ModeStatic) {
    if (const uint8_t *d = in(items_, items_length_, kIpAddress, nullptr, 1, n)) memcpy(ip.address, d, 4);
    if (const uint8_t *d = in(items_, items_length_, kIpPrefix, nullptr, 1, n)) ip.prefix = static_cast<uint8_t>(getU32(d));
    if (const uint8_t *d = in(items_, items_length_, kIpGateway, nullptr, 1, n)) memcpy(ip.gateway, d, 4);
    if (const uint8_t *d = in(items_, items_length_, kIpDns, nullptr, 1, n)) memcpy(ip.dns, d, 4);
  }
  mix(&ip.mode, 1);
  mix(ip.address, 4);
  mix(&ip.prefix, 1);
  mix(ip.gateway, 4);
  mix(ip.dns, 4);
  if (wifi_handed_ && crc == wifi_applied_) return;
  wifi_handed_ = true;
  wifi_applied_ = crc;
  wifi_->apply(entries, count, ip);
}

uint8_t Settings::restOf(const void *self, uint16_t swclk) {
  const WireContext &c = *static_cast<const WireContext *>(self);
  size_t n = 0;
  const uint8_t *d = c.settings->in(c.settings->items_, c.settings->items_length_, kRest, c.wire, swclk, n);
  return d && n == 1 ? d[0] : reg::wire_rvswd::kRestAuto;
}

bool Settings::holdOf(const void *self, uint16_t swdio) {
  const WireContext &c = *static_cast<const WireContext *>(self);
  size_t n = 0;
  const uint8_t *d = c.settings->in(c.settings->items_, c.settings->items_length_, kHold, c.wire, swdio, n);
  return !(d && n == 1 && d[0] == 0);
}

// ---- storage and boot (core §13.7) ----------------------------------------------------------------------------------------

Result Settings::save(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // [TLV] -> generation(u32) [TLV]: every user value, written only when it differs from the save
  Tail tail;
  const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
  if (refused(parsed)) return parsed;
  if (capacity < 4) return failed();
  uint8_t *const blob = gBlob;
  const size_t n = saveBlob(blob, kMaxBlob);
  if (!n) return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);   // more than the save holds
  putU32(out, generation_);
  const uint32_t crc = getU32(blob + n - 4);
  const bool same = storage_state_ == reg::core::kStorageStateApplied && saved_size_ == n && saved_crc_ == crc;
  if (!same) {
    if (!storeWrite(blob, n)) return failed(4);   // the state says what is kept
    saved_size_ = n;
    saved_crc_ = crc;
  }
  storage_state_ = reg::core::kStorageStateApplied;
  reason_ = 0;
  saved_generation_ = generation_;
  return completed(4);
}

size_t Settings::saveBlob(uint8_t *blob, size_t capacity) const {
  size_t put = 0;
  if (capacity < 1 + 4) return 0;
  blob[put++] = kBlobVersion;
  // the owners in entry order, each once: the core first
  const Interface *owners[kMaxEntries + 1];
  size_t owner_count = 0;
  owners[owner_count++] = nullptr;
  for (uint8_t e = 0; e < entry_count_; ++e) {
    bool seen = false;
    for (size_t o = 0; o < owner_count; ++o) seen |= owners[o] == entries_[e].owner;
    if (!seen) owners[owner_count++] = entries_[e].owner;
  }
  for (size_t o = 0; o < owner_count; ++o) {
    const Interface *owner = owners[o];
    size_t group = put, items = 0;
    const char *cat = owner ? owner->name() : "";
    const size_t cl = strlen(cat);
    if (put + 1 + cl + 2 + 1 > capacity) return 0;
    blob[put++] = static_cast<uint8_t>(cl);
    memcpy(blob + put, cat, cl);
    put += cl;
    putU16(blob + put, owner ? owner->instance() : 0);
    put += 2;
    const size_t count_at = put++;
    for (uint8_t e = 0; e < entry_count_; ++e) {
      if (entries_[e].owner != owner) continue;
      const char *name = spec(e).name;
      const size_t nl = strlen(name);
      size_t item_at = put, values = 0;
      if (put + 1 + nl + 2 > capacity) return 0;
      blob[put++] = static_cast<uint8_t>(nl);
      memcpy(blob + put, name, nl);
      put += nl;
      const size_t vcount_at = put;
      put += 2;
      size_t at = 0, vlen = 0;
      uint8_t tag = 0;
      const uint8_t *v = nullptr;
      while (nextItem(items_, items_length_, at, tag, v, vlen)) {
        if (tag != e + 1) continue;
        char key[24];
        uint8_t val[64];
        const size_t kl = keyName(e, getU16(v), key);
        const size_t vl = storedValue(e, v + 2, vlen - 2, val);
        if ((spec(e).key_kind != reg::core::kSettingKeyKindNone && !kl) || (!vl && vlen > 2)) continue;
        if (put + 1 + kl + 1 + vl + 4 > capacity) return 0;
        blob[put++] = static_cast<uint8_t>(kl);
        memcpy(blob + put, key, kl);
        put += kl;
        blob[put++] = static_cast<uint8_t>(vl);
        memcpy(blob + put, val, vl);
        put += vl;
        ++values;
      }
      if (!values) { put = item_at; continue; }   // an item without user values is not written
      putU16(blob + vcount_at, static_cast<uint16_t>(values));
      ++items;
    }
    if (!items) { put = group; continue; }
    blob[count_at] = static_cast<uint8_t>(items);
  }
  if (put + 4 > capacity) return 0;
  putU32(blob + put, crc32Ieee(blob, put));
  return put + 4;
}

// all false: only the core's disabled channels (load, before the entries); true: every value it can take into a
// candidate, applied (applySaved).
void Settings::readBlob(bool all) {
  if (kNoStore) return;
  uint8_t *const blob = gBlob;
  size_t length = 0;
  if (!storeRead(blob, kMaxBlob, length)) return;
  bool whole = length >= 5 && getU32(blob + length - 4) == crc32Ieee(blob, length - 4) && blob[0] == kBlobVersion;
  const size_t end = whole ? length - 4 : 0;
  uint8_t *const applied = scratch_;
  size_t alen = 0;
  bool bad = false, capacity_short = false;
  uint8_t sink[16];
  for (int pass = 0; pass < (all ? 2 : 1) && whole; ++pass) {   // pass 0: the disables, pass 1: the rest (core §13.7's order)
    for (size_t at = 1; whole && at < end;) {
      const size_t cl = blob[at];
      if (at + 1 + cl + 3 > end) { whole = false; break; }
      const uint8_t *cat = blob + at + 1;
      const uint16_t instance = getU16(blob + at + 1 + cl);
      const uint8_t items = blob[at + 3 + cl];
      at += 4 + cl;
      // the owner: the core, or an interface of that name and instance
      const Interface *owner = nullptr;
      bool known = cl == 0;
      for (uint16_t fn = 1; cl && endpoint_.interfaceAt(fn); ++fn) {
        const Interface *it = endpoint_.interfaceAt(fn);
        if (sameText(cat, cl, it->name()) && it->instance() == instance) { owner = it; known = true; break; }
      }
      for (uint8_t i = 0; whole && i < items; ++i) {
        if (at + 1 > end || at + 1 + blob[at] + 2 > end) { whole = false; break; }
        const size_t nl = blob[at];
        const uint8_t *name = blob + at + 1;
        const uint16_t values = getU16(blob + at + 1 + nl);
        at += 3 + nl;
        uint8_t e = kMaxEntries;
        if (all && known) e = entryNamed(owner, reinterpret_cast<const char *>(name), nl);
        const bool disable = cl == 0 && sameText(name, nl, reg::core::kSettingChannelDisable);
        for (uint16_t v = 0; whole && v < values; ++v) {
          if (at + 1 > end || at + 1 + blob[at] + 1 > end || at + 2 + blob[at] + blob[at + 1 + blob[at]] > end) { whole = false; break; }
          const size_t kl = blob[at];
          const uint8_t *key = blob + at + 1;
          const size_t vl = blob[at + 1 + kl];
          const uint8_t *val = blob + at + 2 + kl;
          at += 2 + kl + vl;
          if (!all) {   // the core's disabled channels, by name
            char buf[16];
            if (!disable || vl != 1 || !val[0]) continue;
            for (uint16_t ch = 0; ch < channels_ && ch < 64; ++ch)
              if (platformChannelName(ch, buf) == kl && memcmp(buf, key, kl) == 0) saved_disabled_ |= uint64_t{1} << ch;
            continue;
          }
          if ((pass == 0) != disable) continue;
          uint16_t k = 0;
          uint8_t w[2 + 64];
          size_t wl = 0;
          bool ok = e != kMaxEntries && keyNamed(e, key, kl, k) && wireValue(e, val, vl, w + 2, wl) &&
                    !refused(checkValue(e, k, w + 2, wl, 0, sink, sizeof sink));
          if (ok && entries_[e].id == kIdle && ((disabledIn(applied, alen) >> k) & 1)) ok = false;   // disable wins
          if (ok) {
            size_t had = 0;
            putU16(w, k);
            if (find(applied, alen, e, k, had)) ok = false;   // a key saved twice: neither
            else if (!insert(applied, alen, kMaxItems, e, w, 2 + wl)) { ok = false; capacity_short = true; }
          }
          bad |= !ok;
        }
      }
    }
  }
  if (!whole) {
    storage_state_ = reg::core::kStorageStateUnreadable;
    reason_ = reg::core::kStorageReasonForm;
    saved_disabled_ = 0;
    return;
  }
  if (!all) return;
  saved_size_ = length;
  saved_crc_ = getU32(blob + length - 4);
  adopt(applied, alen);
  storage_state_ = bad ? reg::core::kStorageStatePartial : reg::core::kStorageStateApplied;
  reason_ = !bad ? 0 : capacity_short ? reg::core::kStorageReasonCapacity : reg::core::kStorageReasonItem;
  saved_generation_ = bad ? 0 : 1;
}

void Settings::load() {
  saved_disabled_ = 0;
  readBlob(false);
}

// At boot (core §13.7): every saved value checked by itself against this firmware - disables first, then the rest (a
// channel with a disable keeps no idle); one it cannot take (an item or a name it does not know too) is left out and the
// rest applied (storage_state 3, reason 2). A key saved twice applies neither copy.
void Settings::applySaved() {
  storage_state_ = reg::core::kStorageStateNone;
  reason_ = 0;
  saved_generation_ = 0;
  adopt(items_, 0);
  readBlob(true);
}

// ---- the pass-throughs (fixture §2.1, console §2.1) ----------------------------------------------------------------------

void Settings::passIn(void *self, const uint8_t *data, size_t length) {
  const PassContext *c = static_cast<const PassContext *>(self);
  c->settings->binds_.put(c->port, data, length);
}

void Settings::runPassthroughs() {
  for (uint8_t port = 0; port < kMaxPorts; ++port) {
    const bool session = endpoint_.held(port);   // a session's request came on the port: off for the session
    gPassContext[port] = {this, port};
    size_t n = 0;
    // a UART's
    int want_uart = -1;
    uint16_t rx = 0;
    for (size_t k = 0; k < uart_count_ && want_uart < 0; ++k)
      if (const uint8_t *d = in(items_, items_length_, kPassRx, uarts_[k], port, n)) { want_uart = static_cast<int>(k); rx = getU16(d); }
    if (want_uart >= 0) {
      FixtureUart &uart = *uarts_[want_uart];
      const uint8_t *b = in(items_, items_length_, kPassBaud, &uart, port, n);
      const uint8_t *f = in(items_, items_length_, kPassFormat, &uart, port, n);
      const bool on = !session && !uart.planned() && pins_ && pins_->free(rx);
      if (on && pass_uart_[port] < 0 &&
          uart.passStart(rx, b ? getU32(b) : reg::kUartBridgeBootBaud, f ? f[0] : 0, &Settings::passIn, &gPassContext[port]))
        pass_uart_[port] = static_cast<int8_t>(want_uart);
      if (pass_uart_[port] >= 0) {
        if (on) uart.poll();
        else { uart.passStop(); pass_uart_[port] = -1; binds_.drop(port); }
      }
    } else if (pass_uart_[port] >= 0) {
      uarts_[pass_uart_[port]]->passStop();
      pass_uart_[port] = -1;
      binds_.drop(port);
    }
    // a console's: wire, swdio and mechanism given (and swclk on rvswd)
    int want_place = -1;
    for (size_t k = 0; k < place_count_ && want_place < 0; ++k)
      if (in(items_, items_length_, kConWire, places_[k].console, port, n)) want_place = static_cast<int>(k);
    if (want_place < 0) {
      if (pass_console_[port] >= 0) {
        places_[pass_console_[port]].console->passStop();
        residentRelease(places_[pass_console_[port]].wire->port());
        pass_console_[port] = -1;
        binds_.drop(port);
      }
      console_tried_[port] = false;
      continue;
    }
    const Place &pl = places_[want_place];
    DebugPort &dp = pl.wire->port();
    TargetConsoleStream &console = *pl.console;
    size_t m = 0;
    const uint8_t *io = in(items_, items_length_, kConSwdio, &console, port, m);
    const uint8_t *ck = in(items_, items_length_, kConSwclk, &console, port, m);
    const uint8_t *hz = in(items_, items_length_, kConSpeed, &console, port, m);
    const uint8_t *mech = in(items_, items_length_, kConMechanism, &console, port, m);
    uint8_t number = 0;
    for (uint8_t dr = 0; mech && dr < TargetConsoleStream::kDrivers; ++dr)
      if (sameText(mech, m, TargetConsoleStream::kDriverNames[dr])) number = console.numberOf(dr);
    const uint16_t swdio = io ? getU16(io) : 0xffff, swclk = ck ? getU16(ck) : 0xffff;
    const bool complete = io && number && (!pl.wire->isRvswd() || ck);
    const bool pins_free = !pins_ || (pins_->free(swdio) && (swclk == 0xffff || pins_->free(swclk)));
    const bool on = complete && !session && !dp.connected && pins_free && !skip_boot_attach_ && (!attach_gate_ || attach_gate_());
    if (!on) {
      if (pass_console_[port] >= 0) { console.passStop(); residentRelease(dp); pass_console_[port] = -1; binds_.drop(port); }
      console_tried_[port] = false;   // the condition starts again later: one more try (rvswd)
      continue;
    }
    if (pass_console_[port] < 0) {
      // rvswd's wake may reset the target: tried once while the condition holds; swio tries again (console §2.1)
      if (console_tried_[port] && pl.wire->isRvswd()) continue;
      console_tried_[port] = true;
      dp.yield = [](void *c) {
        PassContext *pc = static_cast<PassContext *>(c);
        pc->settings->runPassthroughsYield(pc->port);
      };
      dp.yield_context = &gPassContext[port];
      const uint32_t max_hz = hz ? getU32(hz) : dp.dm.phy().minClockHz();
      if (residentAttach(dp, swdio, swclk, max_hz ? max_hz : 1000000u) &&
          console.passStart(number, &Settings::passIn, &gPassContext[port]))
        pass_console_[port] = static_cast<int8_t>(want_place);
      else
        residentRelease(dp);
    }
    if (pass_console_[port] >= 0) {
      console.poll();
      if (console.passLost()) { residentRelease(dp); pass_console_[port] = -1; }   // line lost: tried again (swio)
    }
  }
}

void Settings::runPassthroughsYield(uint8_t port) {
  if (port >= kMaxPorts || pass_console_[port] < 0) return;
  places_[pass_console_[port]].console->passStop();
  residentRelease(places_[pass_console_[port]].wire->port());
  pass_console_[port] = -1;
  binds_.drop(port);
}

void Settings::poll() {
  if (pass_run_) (this->*pass_run_)();
  for (size_t k = 0; k < hold_count_; ++k) holds_[k].check(holds_[k].port);   // holds whose pins were taken or settings changed
}

}  // namespace oep

#endif
