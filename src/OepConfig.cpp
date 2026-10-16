// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepConfig.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_CONFIG)

#if defined(ARDUINO_ARCH_ESP32)
#include <Preferences.h>
#elif defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
#endif
#include <initializer_list>
#include <string.h>

#include "OepEndpoint.h"

namespace oep {

namespace cfg = reg::probe_config;

namespace {

using namespace config_kind;
// data lengths after the key (§1's table)
constexpr size_t kPassData = 9, kConsolePassData = 13, kIpv4Data = 14;
constexpr uint8_t kPassHidden = 0xFF;   // wifi pass_len: get's "has a passphrase", set's "keep it" (§1.4)

// The saved blob, by name (§1: read back by name): groups of name_len(u8) name count(u16) count x (len(u16) key data)
// in the store's order, then CRC-32 of them (u32). A blob whose CRC does not match, or that does not end on a group, is
// a container whose bounds or integrity cannot be confirmed (probe.config §3: unreadable, reason 1).
constexpr size_t kMaxNames = kOwn - 1 + ProbeConfig::kMaxOwn;
constexpr size_t kMaxBlob = ProbeConfig::kMaxItems + kMaxNames * (1 + 48 + 2) + 4;
#if defined(ARDUINO_ARCH_ESP32)
// NVS writes a key whole (old or new after a power loss, probe.config §3); "items7" / "items6" / "items5" are the forms
// before.
constexpr const char *kNvsNamespace = "oepcfg", *kNvsItems = "items8";
bool storeRead(uint8_t *blob, size_t capacity, size_t &length) {
  Preferences p;
  if (!p.begin(kNvsNamespace, true)) return false;
  length = p.getBytesLength(kNvsItems);
  const bool ok = length > 0 && length <= capacity && p.getBytes(kNvsItems, blob, length) == length;
  p.end();
  return ok;
}
bool storeOld() {
  Preferences p;
  if (!p.begin(kNvsNamespace, true)) return false;
  const bool old = p.isKey("items7") || p.isKey("items6") || p.isKey("items5");
  p.end();
  return old;
}
bool storeWrite(const uint8_t *blob, size_t length) {   // length 0: nothing saved
  Preferences p;
  if (!p.begin(kNvsNamespace, false)) return false;
  if (p.isKey("items7")) p.remove("items7");
  if (p.isKey("items6")) p.remove("items6");
  if (p.isKey("items5")) p.remove("items5");
  const size_t written = length ? p.putBytes(kNvsItems, blob, length) : (p.remove(kNvsItems), 0);
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
  if (length) memcpy(gFakeBlob, blob, length);
  gFakeLength = length;
  return true;
}
bool storeOld() { return false; }
}  // namespace
void fakeConfigStore(const uint8_t *blob, size_t length) { storeWrite(blob, length); }
namespace {
#else
// The flash's last sector (arduino-pico EEPROM): magic, length, the blob. A sector rewrite is not atomic under a power
// loss; the CRC makes a torn one read as unreadable (reason 1) rather than as settings (probe.config §3).
constexpr uint32_t kEepromMagic = 0x4f455038;   // "OEP8"; "OEP7" / "OEP6" / "OEP5" are the forms before
constexpr size_t kEepromHeader = 6;             // magic(u32) length(u16)
bool storeOld() {
  EEPROM.begin(kEepromHeader + kMaxBlob);
  const uint32_t m = getU32(EEPROM.getConstDataPtr());
  return m == 0x4f455037 || m == 0x4f455036 || m == 0x4f455035;
}
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
  putU32(e, length ? kEepromMagic : 0);
  putU16(e + 4, static_cast<uint16_t>(length));
  if (length) memcpy(e + kEepromHeader, blob, length);
  return EEPROM.commit();
}
#endif

bool nextItem(const uint8_t *store, size_t length, size_t &at, uint8_t &tag, const uint8_t *&value, size_t &vlen) {
  size_t next = 0;
  if (!tlvAt(store, length, at, tag, value, vlen, next)) return false;
  at = next;
  return true;
}

// The value the unavailable payload carries for a channel (core §4.3: cause, channel).
Result heldBySettings(uint8_t *out, size_t capacity, uint16_t channel) {
  return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, channel);
}

// One port's context for a pass-through's sink: the config and the port.
struct PassContext { ProbeConfig *config; uint8_t port; };
PassContext gPassContext[ProbeConfig::kMaxPorts];

}  // namespace

uint32_t crc32Ieee(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

uint8_t ProbeConfig::scratch_[ProbeConfig::kMaxItems];

bool ProbeConfig::addPlace(WireRvswd &wire, TargetConsoleStream &console) {
  if (place_count_ == kMaxPlaces) return false;
  places_[place_count_++] = {&wire, &console};
  return true;
}

bool ProbeConfig::addUart(FixtureUart &uart) {
  if (uart_count_ == kMaxUarts) return false;
  uarts_[uart_count_++] = &uart;
  return true;
}

bool ProbeConfig::addSetting(const char *name, uint8_t form, uint16_t keys, uint8_t max_len, uint8_t flags) {
  const size_t n = strlen(name);
  bool dot = false, ok = n >= 1 && n <= 48 && keys >= 1 && form >= cfg::kSettingFormBytes && form <= cfg::kSettingFormBool;
  for (size_t i = 0; ok && i < n; ++i) {
    const char c = name[i];
    dot |= c == '.';
    ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '.';
  }
  const bool sized = form == cfg::kSettingFormBytes || form == cfg::kSettingFormText;
  // §4: max_len only for bytes / text, where one item's set fits the smallest max_frame (10 + 1 + 3 + 2 + max_len)
  if (!ok || !dot || own_count_ == kMaxOwn || (sized ? max_len == 0 || 16u + max_len > endpoint_.maxFrame() : max_len != 0) ||
      (flags & ~(cfg::kSettingFlagSecret | cfg::kSettingFlagRestart)) || kindNamed(reinterpret_cast<const uint8_t *>(name), n))
    return false;
  Own &o = own_[own_count_++];
  memcpy(o.name, name, n);
  o.form = form;
  o.keys = keys;
  o.max_len = max_len;
  o.flags = flags;
  return true;
}

const uint8_t *ProbeConfig::setting(const char *name, uint16_t key, size_t &length) const {
  const uint8_t kind = kindNamed(reinterpret_cast<const uint8_t *>(name), strlen(name));
  size_t vlen = 0;
  const uint8_t *v = kind ? find(items_, items_length_, kind, key, vlen) : nullptr;
  if (!v) return nullptr;
  length = vlen - 2;
  return v + 2;
}

bool ProbeConfig::declares(uint8_t kind) const {
  switch (kind) {
    case kLabel: return true;
    case kIdle: case kDisable: return pins_ != nullptr;
    case kPass: return uart_count_ > 0;
    case kConsolePass: return place_count_ > 0;
    case kWifi: case kIpv4: return wifi_ != nullptr;
    default: return kind >= kOwn && kind < kOwn + own_count_;
  }
}

// The k-th kind declared is item number k (§1, core §7.7).
uint8_t ProbeConfig::numberOf(uint8_t kind) const {
  if (!declares(kind)) return 0;
  uint8_t n = 0;
  for (uint8_t k = kLabel; k <= kind; ++k) n += declares(k);
  return n;
}

uint8_t ProbeConfig::kindOf(uint8_t number) const {
  for (uint8_t k = kLabel, n = 0; k < kOwn + own_count_; ++k)
    if (declares(k) && ++n == number) return k;
  return 0;
}

const char *ProbeConfig::nameOf(uint8_t kind) const {
  switch (kind) {
    case kLabel: return cfg::kNameSettingLabel;
    case kIdle: return cfg::kNameSettingIdle;
    case kDisable: return cfg::kNameSettingDisable;
    case kPass: return cfg::kNameSettingPassthrough;
    case kConsolePass: return cfg::kNameSettingConsolePassthrough;
    case kWifi: return cfg::kNameSettingWifi;
    case kIpv4: return cfg::kNameSettingIpv4;
    default: return kind >= kOwn && kind < kOwn + own_count_ ? own_[kind - kOwn].name : "";
  }
}

uint8_t ProbeConfig::kindNamed(const uint8_t *name, size_t length) const {
  for (uint8_t k = kLabel; k < kOwn + own_count_; ++k) {
    const char *n = nameOf(k);
    if (strlen(n) == length && memcmp(n, name, length) == 0) return k;
  }
  return 0;
}

// The setting's keys (§1): fn 0's channels, the transports, the wifi entries, the netifs + 1 (netif numbers from 1).
uint16_t ProbeConfig::keysOf(uint8_t kind) const {
  switch (kind) {
    case kLabel: case kIdle: case kDisable: return channels_;
    case kPass: case kConsolePass: return static_cast<uint16_t>(endpoint_.transportCount());
    case kWifi: return kMaxWifi;
    case kIpv4: return 2;   // netif 1, "wifi"
    default: return kind >= kOwn && kind < kOwn + own_count_ ? own_[kind - kOwn].keys : 0;
  }
}

bool ProbeConfig::serialPort(uint16_t port) const { return port < kMaxPorts && endpoint_.isSerialPort(port); }

int ProbeConfig::uartIndex(uint16_t fn) const {
  for (size_t k = 0; k < uart_count_; ++k) if (endpoint_.fnOf(*uarts_[k]) == fn) return static_cast<int>(k);
  return -1;
}

int ProbeConfig::placeIndex(uint16_t wire_fn, uint16_t console_fn) const {
  for (size_t k = 0; k < place_count_; ++k)
    if (endpoint_.fnOf(*places_[k].wire) == wire_fn && endpoint_.fnOf(*places_[k].console) == console_fn)
      return static_cast<int>(k);
  return -1;
}

// The value (key, data) of the store's item (kind, key); nullptr: none.
const uint8_t *ProbeConfig::find(const uint8_t *store, size_t length, uint8_t kind, uint16_t key, size_t &vlen) const {
  size_t at = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  while (nextItem(store, length, at, tag, v, vlen))
    if (tag == kind && vlen >= 2 && getU16(v) == key) return v;
  return nullptr;
}

namespace {

// §1.4's form: ssid 1-32 bytes, pass_len 0 / 8-63 printable / 64 hex / 0xFF alone, nothing after.
bool wifiForm(const uint8_t *d, size_t n) {
  if (n < 2 || d[0] < 1 || d[0] > reg::kLimitWifiSsidMaxBytes || n < 2u + d[0]) return false;
  const uint8_t pl = d[1 + d[0]];
  const uint8_t *pw = d + 2 + d[0];
  const size_t rest = n - 2 - d[0];
  if (pl == kPassHidden || pl == 0) return rest == 0;
  if (rest != pl) return false;
  for (size_t i = 0; i < rest; ++i) {
    const uint8_t c = pw[i];
    const bool hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    if (pl == 64 ? !hex : (pl < 8 || pl > 63 || c < 0x20 || c > 0x7E)) return false;
  }
  return true;
}

// §1.5's form: mode 0 all zero; mode 1 an address a host may have, prefix 1-32, the gateway (when set) in its prefix.
bool ipv4Form(const uint8_t *d, size_t n) {
  if (n != kIpv4Data) return false;
  if (d[0] == cfg::kIpv4ModeDhcp) {
    for (size_t i = 1; i < n; ++i) if (d[i]) return false;
    return true;
  }
  if (d[0] != cfg::kIpv4ModeStatic || d[5] < 1 || d[5] > 32) return false;
  const uint32_t addr = (uint32_t{d[1]} << 24) | (uint32_t{d[2]} << 16) | (uint32_t{d[3]} << 8) | d[4];
  const uint32_t gw = (uint32_t{d[6]} << 24) | (uint32_t{d[7]} << 16) | (uint32_t{d[8]} << 8) | d[9];
  const uint32_t mask = d[5] == 32 ? 0xFFFFFFFFu : ~(0xFFFFFFFFu >> d[5]);
  if (addr == 0 || addr == 0xFFFFFFFFu || d[1] == 127 || (d[1] >= 224 && d[1] <= 239)) return false;
  return gw == 0 || (gw & mask) == (addr & mask);
}

}  // namespace

// One item by itself (probe.config §1): its form (another length is malformed), then what this probe takes
// (unsupported with the number as received). v: key(u16) data.
Result ProbeConfig::checkItem(uint8_t kind, uint8_t number, const uint8_t *v, size_t len, uint8_t *out,
                              size_t capacity) const {
  if (len < 2) return rejected(kRejectMalformed);
  const uint16_t key = getU16(v);
  const uint8_t *d = v + 2;
  const size_t n = len - 2;
  size_t had = 0;
  switch (kind) {
    case kLabel: if (n < 1 || n > reg::kLimitLabelMaxBytes) return rejected(kRejectMalformed); break;
    case kIdle: if (n != 1) return rejected(kRejectMalformed); break;
    case kDisable: if (n != 0) return rejected(kRejectMalformed); break;
    case kPass:
      if (n != kPassData || getU32(d + 4) == 0) return rejected(kRejectMalformed);   // baud 0 (fixture §2's rule)
      break;
    case kConsolePass:
      if (n != kConsolePassData || getU32(d + 6) == 0) return rejected(kRejectMalformed);   // max_speed 0 (debug §1)
      break;
    case kWifi:
      if (!wifiForm(d, n)) return rejected(kRejectMalformed);
      // pass_len 0xFF keeps the entry's passphrase: none there is malformed (§1.4)
      if (d[1 + d[0]] == kPassHidden && !find(items_, items_length_, kWifi, key, had)) return rejected(kRejectMalformed);
      break;
    case kIpv4: if (!ipv4Form(d, n)) return rejected(kRejectMalformed); break;
    default: {
      if (!declares(kind)) return unsupportedTag(out, capacity, number);
      const Own &o = own_[kind - kOwn];
      bool ok = false;
      if (n == 0 && (o.flags & cfg::kSettingFlagSecret)) {   // a secret's 0 bytes keep its value (§1.7)
        ok = find(items_, items_length_, kind, key, had) != nullptr;
      } else {
        switch (o.form) {
          case cfg::kSettingFormBytes: ok = n <= o.max_len; break;
          case cfg::kSettingFormText: ok = n >= 1 && n <= o.max_len; break;
          case cfg::kSettingFormU32: case cfg::kSettingFormIpv4: ok = n == 4; break;
          case cfg::kSettingFormBool: ok = n == 1 && d[0] <= 1; break;
          default: break;
        }
      }
      if (!ok) return rejected(kRejectMalformed);
      if (key >= o.keys) return unsupportedTag(out, capacity, number);
      return completed();
    }
  }
  if (!declares(kind) || key >= keysOf(kind)) return unsupportedTag(out, capacity, number);
  switch (kind) {
    case kLabel: case kIdle: case kDisable: {
      if (pins_ ? !pins_->allowed(key) : key >= PinTable::kChannels) return unsupportedTag(out, capacity, number);
      if (kind == kIdle) {
        const uint8_t mode = d[0];
        const bool output = mode == PinTable::kIdleOutputLow || mode == PinTable::kIdleOutputHigh;
        const bool pull = mode == PinTable::kIdlePullUp || mode == PinTable::kIdlePullDown;
        if (mode > PinTable::kIdleOutputHigh || (output && !pins_->canOutput(key)) || (pull && !pins_->canPull(key)))
          return unsupportedTag(out, capacity, number);
      }
      return completed();
    }
    case kPass: {
      const int k = uartIndex(getU16(d));
      const uint16_t rx = getU16(d + 2);
      if (!serialPort(key) || k < 0 || !uarts_[k]->rxAllowed(rx) || !uarts_[k]->baudWithinReach(getU32(d + 4)) ||
          !FixtureUart::formatDefined(d[8]) || !uarts_[k]->formatDeclared(d[8]))
        return unsupportedTag(out, capacity, number);
      return completed();
    }
    case kConsolePass: {
      const int k = placeIndex(getU16(d), getU16(d + 10));
      if (!serialPort(key) || k < 0) return unsupportedTag(out, capacity, number);
      DebugPort &port = places_[k].wire->port();
      const uint32_t max_hz = getU32(d + 6);
      if (!pairAllowed(port, getU16(d + 2), getU16(d + 4)) ||
          (port.dm.phy().minClockHz() && max_hz < port.dm.phy().minClockHz()) ||
          !places_[k].console->offersMechanism(d[12]))
        return unsupportedTag(out, capacity, number);
      return completed();
    }
    case kIpv4: return key == 0 ? unsupportedTag(out, capacity, number) : completed();   // netifs from 1 (core §7.7)
    default: return completed();
  }
}

// The whole after the change (probe.config §1, §2): a disable on a channel in use (cause 1), an idle and a disable on
// one channel (cause 5), a pass-through on a disabled pin or one with an output idle (cause 5), two pass-throughs on a
// port (cause 5); nothing changes when refused.
Result ProbeConfig::checkWhole(const uint8_t *items, size_t length, uint8_t *out, size_t capacity) const {
  const uint64_t disabled = disabledIn(items, length);
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  bool pass_port[kMaxPorts] = {};
  auto idleOutput = [&](uint16_t ch) {
    size_t n = 0;
    const uint8_t *w = find(items, length, kIdle, ch, n);
    return w && (w[2] == PinTable::kIdleOutputLow || w[2] == PinTable::kIdleOutputHigh);
  };
  auto isDisabled = [&](uint16_t ch) { return ch < 64 && ((disabled >> ch) & 1); };
  while (nextItem(items, length, at, tag, v, vlen)) {
    const uint16_t key = getU16(v);
    if (tag == kDisable) {
      const bool now = pins_ && pins_->disabled(key);
      if (!now && pins_ && pins_->owner(key) && pins_->owner(key) != 0xff)
        return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, key);
    }
    if (tag == kIdle && isDisabled(key)) return heldBySettings(out, capacity, key);
    if ((tag == kPass || tag == kConsolePass) && key < kMaxPorts) {
      if (pass_port[key]) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings);
      pass_port[key] = true;
      const uint8_t *d = v + 2;
      const uint16_t pins[2] = {getU16(d + 2), tag == kConsolePass ? getU16(d + 4) : static_cast<uint16_t>(0xffff)};
      for (uint16_t ch : pins)
        if (ch != 0xffff && (isDisabled(ch) || idleOutput(ch))) return heldBySettings(out, capacity, ch);
    }
  }
  return completed();
}

bool ProbeConfig::insert(uint8_t *store, size_t &length, size_t capacity, uint8_t kind, const uint8_t *v, size_t len) {
  const uint16_t key = getU16(v);
  remove(store, length, kind, key);
  // the place: after every item of a smaller kind, or of this kind with a smaller key (get's order, §2)
  size_t at = 0, place = length, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *w = nullptr;
  while (at < length) {
    const size_t here = at;
    if (!nextItem(store, length, at, tag, w, vlen)) break;
    if (tag > kind || (tag == kind && getU16(w) > key)) { place = here; break; }
  }
  if (length + kTlvHeader + len > capacity) return false;
  memmove(store + place + kTlvHeader + len, store + place, length - place);
  putTlvHeader(store + place, kind, static_cast<uint16_t>(len));
  memcpy(store + place + kTlvHeader, v, len);
  length += kTlvHeader + len;
  return true;
}

void ProbeConfig::remove(uint8_t *store, size_t &length, uint8_t kind, uint16_t key) {
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  while (at < length) {
    const size_t here = at;
    if (!nextItem(store, length, at, tag, v, vlen)) return;
    if (tag == kind && vlen >= 2 && getU16(v) == key) {
      memmove(store + here, store + at, length - at);
      length -= at - here;
      return;
    }
  }
}

uint64_t ProbeConfig::disabledIn(const uint8_t *items, size_t length) const {
  uint64_t mask = 0;
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  while (nextItem(items, length, at, tag, v, vlen))
    if (tag == kDisable && vlen == 2 && getU16(v) < 64) mask |= uint64_t{1} << getU16(v);
  return mask;
}

uint64_t ProbeConfig::savedDisabled() const { return disabledIn(saved_, saved_length_); }

// The items made current: disables first (core §8: before any idle), then the idles of the free channels (a channel in
// use takes its new idle when it is freed), the Wi-Fi when its items changed, and the pass-throughs from the next poll.
void ProbeConfig::adopt(const uint8_t *items, size_t length) {
  if (items != items_) memmove(items_, items, length);
  items_length_ = length;
  handWifi();
  if (!pins_) return;
  const uint64_t disabled = disabledIn(items_, items_length_);
  pins_->setDisabled(disabled);
  endpoint_.setDisabled(disabled);
  for (uint16_t ch = 0; ch < PinTable::kChannels; ++ch) {
    if (!pins_->allowed(ch)) continue;
    size_t vlen = 0;
    const uint8_t *v = find(items_, items_length_, kIdle, ch, vlen);
    const uint8_t mode = v ? v[2] : PinTable::kIdleUnset;
    if (pins_->idle(ch) != mode) pins_->setIdle(ch, mode);   // applied now to a free channel only (PinTable)
  }
}

// The wifi entries in key order and the netif's ipv4 to the station, when they differ from what it has (§1.3).
void ProbeConfig::handWifi() {
  if (!wifi_) return;
  WifiEntry entries[kMaxWifi];
  size_t count = 0, at = 0, vlen = 0;
  Ipv4Setting ip;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  uint32_t crc = 0xFFFFFFFFu;
  while (nextItem(items_, items_length_, at, tag, v, vlen)) {
    if (tag != kWifi && tag != kIpv4) continue;
    for (size_t i = 0; i < vlen; ++i) {   // the CRC of what is handed over (kind, value)
      crc ^= i ? v[i] : static_cast<uint8_t>(v[i] ^ tag);
      for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    const uint8_t *d = v + 2;
    if (tag == kWifi && count < kMaxWifi) {
      WifiEntry &e = entries[count++];
      e.index = getU16(v);
      e.ssid_length = d[0];
      memcpy(e.ssid, d + 1, d[0]);
      e.pass_length = d[1 + d[0]];
      memcpy(e.pass, d + 2 + d[0], e.pass_length);
    } else if (tag == kIpv4 && getU16(v) == 1) {
      ip.mode = d[0];
      memcpy(ip.address, d + 1, 4);
      ip.prefix = d[5];
      memcpy(ip.gateway, d + 6, 4);
      memcpy(ip.dns, d + 10, 4);
    }
  }
  if (wifi_handed_ && crc == wifi_applied_) return;
  wifi_handed_ = true;
  wifi_applied_ = crc;
  wifi_->apply(entries, count, ip);
}

Result ProbeConfig::commit(const uint8_t *candidate, size_t length, uint8_t *out, size_t capacity) {
  const Result whole = checkWhole(candidate, length, out, capacity);
  if (refused(whole)) return whole;
  if (capacity < 4) return failed();
  const bool same = length == items_length_ && memcmp(candidate, items_, length) == 0;
  if (!same) {   // a change: the generation goes on (§2: none left at 0xFFFFFFFF, until a restart)
    if (generation_ == 0xFFFFFFFFu) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
    ++generation_;
    adopt(candidate, length);
  }
  putU32(out, generation_);
  return completed(4);
}

Result ProbeConfig::set(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // count(u8) count x item [TLV]: every item's form, then the outer TLVs (core §2.3), then what it takes
  if (length < 1) return rejected(kRejectMalformed);
  const uint8_t count = payload[0];
  size_t at = 1;
  for (uint8_t i = 0; i < count; ++i) {
    if (at + kTlvHeader > length) return rejected(kRejectMalformed);
    const size_t len = getU16(payload + at + 1);
    if (at + kTlvHeader + len > length) return rejected(kRejectMalformed);
    if (payload[at] == 0 || len < 2) return rejected(kRejectMalformed);   // number 0, no key (§1)
    at += kTlvHeader + len;
  }
  Tail tail;
  const Result parsed = tail.parse(payload + at, length - at, out, capacity);
  if (refused(parsed)) return parsed;
  // the items one by one, then a key twice (malformed, §2)
  Result unsupported = completed();
  for (size_t p = 1, i = 0; i < count; ++i) {
    const uint8_t number = payload[p], kind = kindOf(number);
    const size_t len = getU16(payload + p + 1);
    const Result r = kind ? checkItem(kind, number, payload + p + kTlvHeader, len, out, capacity)
                          : unsupportedTag(out, capacity, number);
    if (r.resolution == kResolutionRejected && r.detail == kRejectMalformed) return r;
    if (refused(r) && !refused(unsupported)) unsupported = r;
    for (size_t q = 1, j = 0; j < i; ++j) {
      const size_t qlen = getU16(payload + q + 1);
      if (payload[q] == number && getU16(payload + q + kTlvHeader) == getU16(payload + p + kTlvHeader))
        return rejected(kRejectMalformed);
      q += kTlvHeader + qlen;
    }
    p += kTlvHeader + len;
  }
  if (refused(unsupported)) return unsupported;
  uint8_t *const candidate = scratch_;   // one work area for set / unset / applySaved (never at once)
  size_t clen = items_length_;
  memcpy(candidate, items_, clen);
  for (size_t p = 1, i = 0; i < count; ++i) {
    const uint8_t kind = kindOf(payload[p]);
    const uint8_t *v = payload + p + kTlvHeader;
    size_t len = getU16(payload + p + 1), had = 0;
    uint8_t kept[2 + 1 + 32 + 1 + 64];
    const uint8_t *old = find(items_, items_length_, kind, getU16(v), had);
    if (kind == kWifi && v[2 + 1 + v[2]] == kPassHidden) {   // the entry's passphrase kept (§1.4)
      const uint8_t *opw = old + 2 + 1 + old[2];   // pass_len then the passphrase
      const size_t keep = 1u + opw[0];
      memcpy(kept, v, len - 1);
      memcpy(kept + len - 1, opw, keep);
      v = kept;
      len = len - 1 + keep;
    } else if (kind >= kOwn && len == 2 && (own_[kind - kOwn].flags & cfg::kSettingFlagSecret)) {
      v = old;   // a secret's value kept (§1.7)
      len = had;
    }
    if (!insert(candidate, clen, kMaxItems, kind, v, len))
      return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);   // more than this probe holds (§2)
    p += kTlvHeader + getU16(payload + p + 1);
  }
  return commit(candidate, clen, out, capacity);
}

Result ProbeConfig::unset(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // count(u8) count x (number u8, key u16) [TLV]
  if (length < 1 || length < 1u + 3u * payload[0]) return rejected(kRejectMalformed);
  const uint8_t count = payload[0];
  Tail tail;
  const Result parsed = tail.parse(payload + 1 + 3 * count, length - 1 - 3 * count, out, capacity);
  if (refused(parsed)) return parsed;
  uint8_t undeclared = 0;
  for (uint8_t i = 0; i < count; ++i) {
    const uint8_t number = payload[1 + 3 * i];
    if (!number) return rejected(kRejectMalformed);
    for (uint8_t j = 0; j < i; ++j)
      if (payload[1 + 3 * j] == number && getU16(payload + 2 + 3 * j) == getU16(payload + 2 + 3 * i))
        return rejected(kRejectMalformed);   // a key twice
    if (!kindOf(number) && !undeclared) undeclared = number;
  }
  if (undeclared) return unsupportedTag(out, capacity, undeclared);
  uint8_t *const candidate = scratch_;   // one work area for set / unset / applySaved (never at once)
  size_t clen = items_length_;
  memcpy(candidate, items_, clen);
  for (uint8_t i = 0; i < count; ++i) remove(candidate, clen, kindOf(payload[1 + 3 * i]), getU16(payload + 2 + 3 * i));
  return commit(candidate, clen, out, capacity);
}

// An item as get shows it (number len key data): a wifi passphrase as pass_len 0xFF / 0 alone (§1.4), a secret's key
// alone (§1.7). out: room for the item.
size_t ProbeConfig::shown(uint8_t kind, const uint8_t *v, size_t vlen, uint8_t *out) const {
  size_t len = vlen;
  if (kind == kWifi) len = 2 + 1 + v[2] + 1;
  else if (kind >= kOwn && (own_[kind - kOwn].flags & cfg::kSettingFlagSecret)) len = 2;
  putTlvHeader(out, numberOf(kind), static_cast<uint16_t>(len));
  memcpy(out + kTlvHeader, v, len);
  if (kind == kWifi) out[kTlvHeader + len - 1] = v[2 + 1 + v[2]] ? kPassHidden : 0;
  return kTlvHeader + len;
}

Result ProbeConfig::get(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // first(u16) [TLV] -> more(u8) generation(u32) count(u8) items [TLV]: whole items from the first-th, as many as fit
  Tail tail;
  const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
  if (refused(parsed)) return parsed;
  if (capacity < 6) return failed();
  const uint16_t first = getU16(payload);
  size_t at = 0, index = 0, put = 6, vlen = 0;
  uint8_t tag = 0, count = 0;
  const uint8_t *v = nullptr;
  bool more = false;
  uint8_t item[kTlvHeader + 2 + 255];
  while (at < items_length_) {
    if (!nextItem(items_, items_length_, at, tag, v, vlen)) break;
    if (index++ < first) continue;
    const size_t n = shown(tag, v, vlen, item);
    if (put + n > capacity || count == 255) { more = true; break; }
    memcpy(out + put, item, n);
    put += n;
    ++count;
  }
  out[0] = more ? 1 : 0;
  putU32(out + 1, generation_);
  out[5] = count;
  return completed(put);
}

// [TLV] -> tcp_port(u16) count(u8) count x (netif u8, state u8, entry u16, reason u8, rssi i8, address(4), prefix,
// gateway(4), dns(4)) [TLV] (§2 network): the one netif, "wifi".
Result ProbeConfig::network(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
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

size_t ProbeConfig::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  if (storage_) w.u32(cfg::kTlvDescribeStorage, kMaxItems);   // the items' bytes a save always holds (§4)
  // setting: form keys(u16) max_len flags name, one per kind declared; the k-th is item number k (§4)
  for (uint8_t k = kLabel; k < kOwn + own_count_; ++k) {
    if (!declares(k)) continue;
    const char *n = nameOf(k);
    uint8_t v[5 + 48];
    const Own *o = k >= kOwn ? &own_[k - kOwn] : nullptr;
    v[0] = o ? o->form : cfg::kSettingFormNamed;
    putU16(v + 1, keysOf(k));
    v[3] = o ? o->max_len : 0;
    v[4] = o ? o->flags : 0;
    memcpy(v + 5, n, strlen(n));
    w.put(cfg::kTlvDescribeSetting, v, 5 + strlen(n));
  }
  if (wifi_) w.text(cfg::kTlvDescribeNetif, cfg::kNameNetifWifi);
  return w.ok() ? w.length() : 0;
}

Result ProbeConfig::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case cfg::kOpGet: return get(payload, length, out, capacity);
    case cfg::kOpSet: return set(payload, length, out, capacity);
    case cfg::kOpUnset: return unset(payload, length, out, capacity);
    case cfg::kOpNetwork: return wifi_ ? network(payload, length, out, capacity) : rejected(kRejectUnknownOperation);
    case cfg::kOpState: {   // [TLV] -> storage_state(u8) saved_generation(u32) reason(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 6) return failed();
      out[0] = storage_state_;
      putU32(out + 1, saved_generation_);
      out[5] = reason_;
      return completed(6);
    }
    case cfg::kOpSave: {   // [TLV] -> generation(u32) [TLV]: every item, written only when it differs from the save (§3)
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 4) return failed();
      putU32(out, generation_);
      const bool same = storage_state_ == cfg::kStorageStateApplied && saved_length_ == items_length_ &&
                        memcmp(saved_, items_, items_length_) == 0;
      if (!same) {
        uint8_t blob[kMaxBlob];   // on loop()'s stack, as the save before
        const size_t n = saveBlob(blob, sizeof blob);
        if (!n || !storeWrite(blob, n)) return failed(4);   // the state says what is kept (§3's table)
        memcpy(saved_, items_, items_length_);
        saved_length_ = items_length_;
        saved_unknown_ = false;
      }
      storage_state_ = cfg::kStorageStateApplied;
      reason_ = 0;
      saved_generation_ = generation_;
      return completed(4);
    }
    case cfg::kOpErase: {
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!storeWrite(nullptr, 0)) return failed();
      saved_length_ = 0;
      saved_unknown_ = false;
      storage_state_ = cfg::kStorageStateNone;
      reason_ = 0;
      saved_generation_ = 0;
      return completed();
    }
    default: return rejected(kRejectUnknownOperation);
  }
}

// ---- storage and boot ------------------------------------------------------------------------------------------------

size_t ProbeConfig::saveBlob(uint8_t *blob, size_t capacity) const {
  size_t put = 0, at = 0, vlen = 0;
  uint8_t tag = 0, group = 0;
  size_t count_at = 0;
  const uint8_t *v = nullptr;
  while (nextItem(items_, items_length_, at, tag, v, vlen)) {
    if (tag != group) {   // a new name
      const char *n = nameOf(tag);
      const size_t nl = strlen(n);
      if (put + 1 + nl + 2 > capacity) return 0;
      blob[put++] = static_cast<uint8_t>(nl);
      memcpy(blob + put, n, nl);
      put += nl;
      count_at = put;
      putU16(blob + put, 0);
      put += 2;
      group = tag;
    }
    if (put + 2 + vlen + 4 > capacity) return 0;
    putU16(blob + put, static_cast<uint16_t>(vlen));
    memcpy(blob + put + 2, v, vlen);
    put += 2 + vlen;
    putU16(blob + count_at, static_cast<uint16_t>(getU16(blob + count_at) + 1));
  }
  if (put + 4 > capacity) return 0;
  putU32(blob + put, crc32Ieee(blob, put));
  return put + 4;
}

void ProbeConfig::load() {
  uint8_t blob[kMaxBlob];
  size_t length = 0;
  saved_length_ = 0;
  saved_unknown_ = false;
  loaded_ = false;
  if (!storeRead(blob, sizeof blob, length)) {
    if (storeOld()) { storage_state_ = cfg::kStorageStateUnreadable; reason_ = cfg::kStorageUnreadableForm; }
    return;
  }
  bool whole = length >= 4 && getU32(blob + length - 4) == crc32Ieee(blob, length - 4);
  // the groups: name_len name count(u16) count x (len(u16) value); a name this firmware does not know is left out
  // (§3: an invalid item), the rest kept as a store
  const size_t end = whole ? length - 4 : 0;
  for (size_t at = 0; whole && at < end;) {
    const size_t nl = blob[at];
    if (at + 1 + nl + 2 > end) { whole = false; break; }
    const uint8_t kind = kindNamed(blob + at + 1, nl);
    const uint16_t count = getU16(blob + at + 1 + nl);
    at += 1 + nl + 2;
    saved_unknown_ |= !kind && count;
    for (uint16_t i = 0; whole && i < count; ++i) {
      if (at + 2 > end || at + 2 + getU16(blob + at) > end) { whole = false; break; }
      const size_t vlen = getU16(blob + at);
      if (kind && (vlen < 2 || saved_length_ + kTlvHeader + vlen > kMaxItems)) saved_unknown_ = true;
      else if (kind) {
        putTlvHeader(saved_ + saved_length_, kind, static_cast<uint16_t>(vlen));
        memcpy(saved_ + saved_length_ + kTlvHeader, blob + at + 2, vlen);
        saved_length_ += kTlvHeader + vlen;
      }
      at += 2 + vlen;
    }
  }
  if (!whole) {
    saved_length_ = 0;
    saved_unknown_ = false;
    storage_state_ = cfg::kStorageStateUnreadable;
    reason_ = cfg::kStorageUnreadableForm;
    return;
  }
  loaded_ = true;
}

// At boot (probe.config §3): every saved item checked by itself against this firmware - disables first, then idles (a
// channel with a disable keeps no idle), labels and the rest; one it cannot take (a name it does not know too) is left
// out and the rest applied (storage_state 3, reason 2). A key saved twice applies neither copy.
void ProbeConfig::applySaved() {
  if (!loaded_) { adopt(items_, 0); return; }
  uint8_t *const applied = scratch_;
  size_t alen = 0;
  bool bad = saved_unknown_;
  uint8_t scratch[16];
  const uint64_t disabled = disabledIn(saved_, saved_length_);
  uint8_t order[kOwn + kMaxOwn] = {kDisable, kIdle, kLabel, kPass, kConsolePass, kWifi, kIpv4};
  size_t kinds = kOwn - 1;
  for (uint8_t k = kOwn; k < kOwn + own_count_; ++k) order[kinds++] = k;
  for (size_t o = 0; o < kinds; ++o) {
    const uint8_t kind = order[o];
    size_t at = 0, vlen = 0;
    uint8_t tag = 0;
    const uint8_t *v = nullptr;
    while (nextItem(saved_, saved_length_, at, tag, v, vlen)) {
      if (tag != kind) continue;
      size_t a = 0, n = 0;
      uint8_t t = 0;
      const uint8_t *w = nullptr;
      int seen = 0;
      while (nextItem(saved_, saved_length_, a, t, w, n)) seen += t == tag && getU16(w) == getU16(v);
      // the wifi passphrase kept as it is (the store's own form): pass_len 0xFF never saved
      bool ok = seen == 1 && !refused(checkItem(tag, numberOf(tag), v, vlen, scratch, sizeof scratch));
      if (ok && tag == kIdle && getU16(v) < 64 && ((disabled >> getU16(v)) & 1)) ok = false;   // disable wins
      if (ok && !insert(applied, alen, kMaxItems, tag, v, vlen)) ok = false;
      bad |= !ok;
    }
  }
  adopt(applied, alen);
  storage_state_ = bad ? cfg::kStorageStatePartial : cfg::kStorageStateApplied;
  reason_ = bad ? cfg::kStorageUnreadableItem : 0;
  saved_generation_ = bad ? 0 : 1;
}

// ---- the pass-throughs (probe.config §1.1, §1.2) --------------------------------------------------------------------

void ProbeConfig::passIn(void *self, const uint8_t *data, size_t length) {
  const PassContext *c = static_cast<const PassContext *>(self);
  c->config->binds_.put(c->port, data, length);
}

void ProbeConfig::runPassthroughs() {
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  bool wanted_uart[kMaxPorts] = {}, wanted_console[kMaxPorts] = {};
  while (nextItem(items_, items_length_, at, tag, v, vlen)) {
    if (tag != kPass && tag != kConsolePass) continue;
    const uint16_t key = getU16(v);
    if (key >= kMaxPorts) continue;
    const uint8_t port = static_cast<uint8_t>(key);
    const uint8_t *d = v + 2;
    const bool session = endpoint_.held(port);   // a session's request came on the port: off for the session
    gPassContext[port] = {this, port};
    if (tag == kPass) {
      const int k = uartIndex(getU16(d));
      if (k < 0) continue;
      FixtureUart &uart = *uarts_[k];
      const uint16_t rx = getU16(d + 2);
      const bool on = !session && !uart.planned() && pins_ && pins_->free(rx);
      wanted_uart[port] = on;
      if (on && pass_uart_[port] < 0 && uart.passStart(rx, getU32(d + 4), d[8], &ProbeConfig::passIn, &gPassContext[port]))
        pass_uart_[port] = static_cast<int8_t>(k);
      if (pass_uart_[port] >= 0) {
        if (on) uart.poll();
        else { uart.passStop(); pass_uart_[port] = -1; binds_.drop(port); }
      }
    } else {
      const int k = placeIndex(getU16(d), getU16(d + 10));
      if (k < 0) continue;
      DebugPort &dp = places_[k].wire->port();
      TargetConsoleStream &console = *places_[k].console;
      const uint16_t swdio = getU16(d + 2), swclk = getU16(d + 4);
      const bool pins_free = !pins_ || (pins_->free(swdio) && (swclk == 0xffff || pins_->free(swclk)));
      const bool on = !session && !dp.connected && pins_free && !skip_boot_attach_ && (!attach_gate_ || attach_gate_());
      wanted_console[port] = on;
      if (!on) {
        if (pass_console_[port] >= 0) { console.passStop(); residentRelease(dp); pass_console_[port] = -1; binds_.drop(port); }
        console_tried_[port] = false;   // the condition starts again later: one more try (rvswd)
        continue;
      }
      if (pass_console_[port] < 0) {
        // rvswd's wake may reset the target: tried once while the condition holds; swio tries again (config §1.2)
        if (console_tried_[port] && places_[k].wire->isRvswd()) continue;
        console_tried_[port] = true;
        dp.yield = [](void *c) {
          PassContext *pc = static_cast<PassContext *>(c);
          pc->config->runPassthroughsYield(pc->port);
        };
        dp.yield_context = &gPassContext[port];
        if (residentAttach(dp, swdio, swclk, getU32(d + 6)) &&
            console.passStart(d[12], &ProbeConfig::passIn, &gPassContext[port]))
          pass_console_[port] = static_cast<int8_t>(k);
        else
          residentRelease(dp);
      }
      if (pass_console_[port] >= 0) {
        console.poll();
        if (console.passLost()) { residentRelease(dp); pass_console_[port] = -1; }   // line lost: tried again (swio)
      }
    }
  }
  for (uint8_t port = 0; port < kMaxPorts; ++port) {   // items removed since
    if (!wanted_uart[port] && pass_uart_[port] >= 0) { uarts_[pass_uart_[port]]->passStop(); pass_uart_[port] = -1; binds_.drop(port); }
    if (!wanted_console[port] && pass_console_[port] >= 0) {
      places_[pass_console_[port]].console->passStop();
      residentRelease(places_[pass_console_[port]].wire->port());
      pass_console_[port] = -1;
      binds_.drop(port);
    }
  }
}

void ProbeConfig::runPassthroughsYield(uint8_t port) {
  if (port >= kMaxPorts || pass_console_[port] < 0) return;
  places_[pass_console_[port]].console->passStop();
  residentRelease(places_[pass_console_[port]].wire->port());
  pass_console_[port] = -1;
  binds_.drop(port);
}

void ProbeConfig::poll() { runPassthroughs(); }

}  // namespace oep

#endif

namespace oep {
namespace {

// A label's text equal to `slot` "." `line` (slot nullptr: `line` alone), ASCII case ignored (host guide §5.3).
bool labelNames(const uint8_t *text, size_t n, const char *slot, const char *line) {
  auto lower = [](uint8_t c) { return c >= 'A' && c <= 'Z' ? static_cast<uint8_t>(c - 'A' + 'a') : c; };
  size_t at = 0;
  auto part = [&](const char *p) {
    for (; *p; ++p, ++at)
      if (at >= n || lower(text[at]) != lower(static_cast<uint8_t>(*p))) return false;
    return true;
  };
  if (slot && !(part(slot) && part("."))) return false;
  return part(line) && at == n;
}

size_t labelsNaming(const uint8_t *items, size_t length, const char *slot, const char *line, uint16_t &channel,
                    uint8_t tag_wanted = config_kind::kLabel) {
  size_t at = 0, found = 0;
  while (at < length) {
    uint8_t tag = 0;
    const uint8_t *v = nullptr;
    size_t vlen = 0, next = 0;
    if (!tlvAt(items, length, at, tag, v, vlen, next)) break;
    at = next;
    if (tag != tag_wanted || vlen < 2 || !labelNames(v + 2, vlen - 2, slot, line)) continue;
    channel = getU16(v);
    ++found;
  }
  return found;
}

}  // namespace

uint16_t findLine(const uint8_t *items, size_t length, const char *slot, const char *line, const uint8_t *firmware,
                  size_t firmware_length) {
  uint16_t channel = 0xffff;
  if (slot) {
    const size_t n = labelsNaming(items, length, slot, line, channel);
    if (n) return n == 1 ? channel : 0xffff;
  }
  const size_t n = labelsNaming(items, length, nullptr, line, channel);
  if (n) return n == 1 ? channel : 0xffff;
  return labelsNaming(firmware, firmware_length, nullptr, line, channel, reg::core::kTlvDescribeLabel) == 1 ? channel : 0xffff;
}

}  // namespace oep
