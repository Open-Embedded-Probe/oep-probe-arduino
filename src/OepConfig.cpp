// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepConfig.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_CONFIG)

#if defined(ARDUINO_ARCH_ESP32)
#include <Preferences.h>
#elif defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
#endif
#include <ctype.h>
#include <stdio.h>

#include "OepEndpoint.h"

namespace oep {

namespace cfg = reg::probe_config;

namespace {

// "items6": the interfaces the items name, then the items (the forms of oep-spec 0f455a0: idle 4 bytes, a slot ending
// at its name, a bind of one stream - a blob of the forms before ("items5") reads as unreadable reason 1)
constexpr const char *kNvsNamespace = "oepcfg", *kNvsItems = "items6", *kNvsOldItems = "items5";
// slot(u8) wire_fn(u16) swdio(u16) swclk(u16) attach(u8) retry_ms(u32) max_speed_hz(u32) idle_clock(u8) mechanism(u8)
// name_len(u8): the slot item's fixed head (probe.config §1.1), the name after it
constexpr size_t kSlotFixed = 19;
enum : size_t { kSlotAttach = 7, kSlotRetryMs = 8, kSlotMaxSpeed = 12, kSlotIdleClock = 16, kSlotMechanism = 17,
                kSlotNameLength = 18 };
// bind: port(u8) kind(u8) id(u16) (probe.config §1.2)
constexpr size_t kBindLength = 4;

// Where the saved blob lives: ESP32 NVS (Preferences), RP2040 / RP2350 the arduino-pico EEPROM (the flash's last
// sector: magic, length, the blob). The blob: the interfaces the items name, then the items. read: false = nothing.
// A write replaces the whole blob (NVS writes a key whole; the EEPROM's commit rewrites its sector).
constexpr size_t kMaxBlob = ProbeConfig::kMaxItems + ProbeConfig::kMaxIds;
#if defined(ARDUINO_ARCH_ESP32)
bool storeRead(uint8_t *blob, size_t capacity, size_t &length) {
  Preferences p;
  if (!p.begin(kNvsNamespace, true)) return false;
  length = p.getBytesLength(kNvsItems);
  const bool ok = length > 0 && length <= capacity && p.getBytes(kNvsItems, blob, length) == length;
  p.end();
  return ok;
}
bool storeOld() {   // a blob of the form before (unreadable reason 1 until the host saves or erases)
  Preferences p;
  if (!p.begin(kNvsNamespace, true)) return false;
  const bool old = p.isKey(kNvsOldItems);
  p.end();
  return old;
}
bool storeWrite(const uint8_t *blob, size_t length) {   // length 0: nothing saved
  Preferences p;
  if (!p.begin(kNvsNamespace, false)) return false;
  if (p.isKey(kNvsOldItems)) p.remove(kNvsOldItems);
  const size_t written = length ? p.putBytes(kNvsItems, blob, length) : (p.remove(kNvsItems), 0);
  p.end();
  return written == length;
}
#elif defined(OEP_HOST_FAKE_CONFIG)
// Host tests (tests/host): the blob in RAM.
uint8_t gFakeBlob[kMaxBlob];
size_t gFakeLength = 0;
bool storeRead(uint8_t *blob, size_t capacity, size_t &length) {
  length = gFakeLength;
  if (!length || length > capacity) return false;
  memcpy(blob, gFakeBlob, length);
  return true;
}
bool storeWrite(const uint8_t *blob, size_t length) {
  memcpy(gFakeBlob, blob, length);
  gFakeLength = length;
  return true;
}
bool storeOld() { return false; }
#else
constexpr uint32_t kEepromMagic = 0x4f455036;   // "OEP6" (0f455a0's forms; "OEP5" is the form before)
constexpr uint32_t kEepromOldMagic = 0x4f455035;
constexpr size_t kEepromHeader = 6;             // magic(u32) length(u16)
bool storeOld() {
  EEPROM.begin(kEepromHeader + kMaxBlob);
  return getU32(EEPROM.getConstDataPtr()) == kEepromOldMagic;
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
bool nameChar(char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_'; }

// The next item of a store: false at the end (a store holds only whole items it wrote itself).
bool nextItem(const uint8_t *store, size_t length, size_t &at, uint8_t &tag, const uint8_t *&value, size_t &vlen) {
  size_t next = 0;
  if (!tlvAt(store, length, at, tag, value, vlen, next)) return false;
  at = next;
  return true;
}

}  // namespace

uint32_t crc32Ieee(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}
uint32_t ProbeConfig::crc32Of(const uint8_t *data, size_t length) { return crc32Ieee(data, length); }

bool ProbeConfig::addPlace(WireRvswd &wire, TargetConsoleStream &console) {
  const uint16_t fn = endpoint_.fnOf(wire);
  if (place_count_ >= kMaxPlaces || !fn || &wire.port() != &console.port()) return false;
  places_[place_count_++] = {fn, &wire.port(), &console};
  return true;
}

bool ProbeConfig::addUart(FixtureUart &uart) {
  const uint16_t fn = endpoint_.fnOf(uart);
  if (uart_count_ >= kMaxUarts || !fn) return false;
  uarts_[uart_count_++] = {fn, &uart};
  return true;
}

// The output strengths of the probe's oep.fixture.gpio (fixture §1.1, describe drive_levels): the chip's levels when
// a gpio fixture is there to declare them, else none (an output idle's drive then takes only 0xFF, probe.config §1).
DriveLevels ProbeConfig::driveLevels() const {
  const DriveLevels levels = platformDriveLevels();
  for (uint16_t f = 1; levels.count && f <= 255; ++f) {
    const Interface *it = endpoint_.interfaceAt(f);
    if (!it) break;
    if (strcmp(it->name(), reg::fixture_gpio::kName) == 0) return levels;
  }
  return {nullptr, 0, 0};
}

// ---- the item store ------------------------------------------------------------------------------------------------

// The items describe declares: label, idle and disable only on a probe with a pin table (their channel is one of it),
// slot only on one with slots (slots_max above 0, probe.config §4: 0 does not handle slots).
bool ProbeConfig::declares(uint8_t tag) const {
  if (tag == cfg::kTlvItemSlot && !place_count_) return false;
  if (tag == kWifiItemTag && !wifi_) return false;   // the wifi item only with something that joins networks
  return keyLength(tag) &&
         (pins_ || (tag != cfg::kTlvItemLabel && tag != cfg::kTlvItemIdle && tag != cfg::kTlvItemDisable));
}

size_t ProbeConfig::keyLength(uint8_t tag) {
  switch (tag) {
    case cfg::kTlvItemPlan: return 5;    // fn role channel (an unset's key is the fn alone: 2)
    case cfg::kTlvItemLabel: return 2;
    case cfg::kTlvItemIdle: return 2;
    case cfg::kTlvItemSlot: return 1;
    case cfg::kTlvItemBind: return 1;
    case cfg::kTlvItemUart: return 2;
    case cfg::kTlvItemDisable: return 2;
    case kWifiItemTag: return 1;         // index
    default: return 0;
  }
}

// The key as a number, for get's order (plan: fn, then role, then channel).
uint64_t ProbeConfig::keyValue(uint8_t tag, const uint8_t *v, size_t length) {
  switch (tag) {
    case cfg::kTlvItemPlan:
      return length >= 5 ? (static_cast<uint64_t>(getU16(v)) << 24) | (static_cast<uint64_t>(v[2]) << 16) | getU16(v + 3) : 0;
    case cfg::kTlvItemLabel:
    case cfg::kTlvItemIdle:
    case cfg::kTlvItemUart:
    case cfg::kTlvItemDisable:
      return length >= 2 ? getU16(v) : 0;
    default:
      return length >= 1 ? v[0] : 0;
  }
}

bool ProbeConfig::itemBefore(uint8_t tag_a, const uint8_t *a, size_t alen, uint8_t tag_b, const uint8_t *b, size_t blen) {
  if (tag_a != tag_b) return tag_a < tag_b;
  return keyValue(tag_a, a, alen) < keyValue(tag_b, b, blen);
}

// Insert one item at its place in get's order. false: no room.
bool ProbeConfig::insertItem(uint8_t *store, size_t &length, size_t capacity, uint8_t tag, const uint8_t *value, size_t vlen) {
  const size_t size = tlvSize(vlen);
  if (length + size > capacity || vlen > 0xFFFF) return false;
  size_t at = 0, insert_at = length;
  uint8_t t = 0;
  const uint8_t *v = nullptr;
  size_t l = 0;
  while (nextItem(store, length, at, t, v, l)) {
    if (itemBefore(tag, value, vlen, t, v, l)) { insert_at = static_cast<size_t>(v - store) - kTlvHeader; break; }
  }
  memmove(store + insert_at + size, store + insert_at, length - insert_at);
  putTlvHeader(store + insert_at, tag, static_cast<uint16_t>(vlen));   // tag len(u16) value (core §2.2)
  memcpy(store + insert_at + kTlvHeader, value, vlen);
  length += size;
  return true;
}

// Whether the value has item `tag`'s one length (probe.config §1: plan 5, idle 4, bind 4, uart 7, disable 2, a slot
// its head and name_len bytes of name, a label its channel and 1 to label_max_bytes of text). Any other: malformed.
bool ProbeConfig::formLength(uint8_t tag, const uint8_t *v, size_t len) {
  switch (tag) {
    case cfg::kTlvItemPlan: return len == 5;
    case cfg::kTlvItemLabel: return len >= 3 && len - 2 <= reg::kLimitLabelMaxBytes;
    case cfg::kTlvItemIdle: return len == 4;
    case cfg::kTlvItemSlot: return len >= kSlotFixed && len == kSlotFixed + v[kSlotNameLength];
    case cfg::kTlvItemBind: return len == kBindLength;
    case cfg::kTlvItemUart: return len == 7;
    case cfg::kTlvItemDisable: return len == 2;
    case kWifiItemTag: {   // index(u8) ssid_len(u8) ssid pass_len(u8) passphrase (pass_len 0xFF: none follows)
      if (len < 3 || v[1] < 1 || v[1] > 32 || len < 3u + v[1]) return false;
      const uint8_t pass = v[2 + v[1]];
      return len == 3u + v[1] + (pass == kWifiPassHidden ? 0 : pass);
    }
    default: return false;
  }
}

// An item as get shows it (the wifi item's passphrase never leaves the probe).
size_t ProbeConfig::shown(uint8_t tag, const uint8_t *v, size_t vlen, uint8_t *out) {
  if (tag != kWifiItemTag || vlen < 3 || vlen < 3u + v[1]) {
    if (out) { putTlvHeader(out, tag, static_cast<uint16_t>(vlen)); memcpy(out + kTlvHeader, v, vlen); }
    return tlvSize(vlen);
  }
  const size_t head = 3u + v[1];
  if (out) {
    putTlvHeader(out, tag, static_cast<uint16_t>(head));
    memcpy(out + kTlvHeader, v, head - 1);
    out[kTlvHeader + head - 1] = v[head - 1] ? kWifiPassHidden : 0;
  }
  return tlvSize(head);
}

namespace {
uint32_t crc32Step(uint32_t crc, const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return crc;
}
}  // namespace

uint32_t ProbeConfig::hash() const {
  uint32_t crc = 0xFFFFFFFFu;
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  bool wifi = false;
  while (nextItem(items_, items_length_, at, tag, v, vlen)) {
    uint8_t one[kTlvHeader + 40];
    if (tag == kWifiItemTag) {
      wifi = true;
      crc = crc32Step(crc, one, shown(tag, v, vlen, one));   // 3 + 3 + 32 at most
    } else {
      crc = crc32Step(crc, v - kTlvHeader, tlvSize(vlen));
    }
  }
  if (wifi) {   // the passphrases: a token that changes with them, not their bytes (get answers without a lock)
    uint8_t token[4];
    putU32(token, wifi_token_);
    crc = crc32Step(crc, token, sizeof token);
  }
  return ~crc;
}

// Remove every item of `tag` whose key begins with `key` (key_length bytes: a plan's fn alone takes the fn's whole plan).
void ProbeConfig::removeItems(uint8_t *store, size_t &length, uint8_t tag, const uint8_t *key, size_t key_length) {
  size_t at = 0;
  while (at < length) {
    uint8_t t = 0;
    const uint8_t *v = nullptr;
    size_t l = 0, start = at;
    if (!nextItem(store, length, at, t, v, l)) break;
    if (t == tag && l >= key_length && memcmp(v, key, key_length) == 0) {
      memmove(store + start, store + at, length - at);
      length -= at - start;
      at = start;
    }
  }
}

// ---- the items -----------------------------------------------------------------------------------------------------

// One item on its own (probe.config §1): its length and the values this probe has. `raw`: the item's tag as received
// (critical bit included); what this probe cannot take is refused unsupported with it (probe.config §1, core §4.3).
Result ProbeConfig::checkItem(uint8_t raw, const uint8_t *v, size_t len, uint8_t *out, size_t capacity) const {
  const uint8_t tag = raw & ~kTagCritical;
  // an item this probe does not declare (describe items): unsupported with its tag as received; its form is not read
  if (!keyLength(tag) || !declares(tag)) return unsupportedTag(out, capacity, raw);
  if (!formLength(tag, v, len)) return rejected(kRejectMalformed);
  switch (tag) {
    case cfg::kTlvItemPlan: {
      const uint16_t fn = getU16(v);
      if (fn == 0) return rejected(kRejectMalformed);
      if (!endpoint_.interfaceAt(fn)) return rejected(kRejectUnknownFunction);
      // a fn with no plan role (oep.probe.plan itself, a wire): as plan_apply, unsupported (oep-if-plan §2.5)
      if (!endpoint_.interfaceAt(fn)->planRoles()) return unsupportedTag(out, capacity, raw);
      return completed();
    }
    case cfg::kTlvItemLabel:
      // the channel: below channels and not one the probe uses itself, as idle and disable (probe.config §1)
      if (!pins_->allowed(getU16(v))) return unsupportedTag(out, capacity, raw);
      return completed();
    case cfg::kTlvItemIdle: {
      // channel(u16) mode(u8) drive(u8) (probe.config §1)
      const uint16_t channel = getU16(v);
      const uint8_t mode = v[2], drive = v[3];
      const bool output = mode == PinTable::kIdleOutputLow || mode == PinTable::kIdleOutputHigh;
      if (mode > PinTable::kIdleOutputHigh) return unsupportedTag(out, capacity, raw);
      if (channel >= PinTable::kChannels || !pins_->allowed(channel)) return unsupportedTag(out, capacity, raw);
      // output low / high: only on a channel this probe can drive; pull-up / pull-down: only where that pull is
      if (output && !pins_->canOutput(channel)) return unsupportedTag(out, capacity, raw);
      if ((mode == PinTable::kIdlePullUp || mode == PinTable::kIdlePullDown) && !pins_->canPull(channel))
        return unsupportedTag(out, capacity, raw);
      // the drive, read for an output only: a level past drive_levels, or any level but the default on a probe that
      // declares none (0xFF always taken)
      uint8_t level = 0;
      if (output && drive != PinTable::kDriveDefault && !PinTable::driveLevelOf(driveLevels(), drive, level))
        return unsupportedTag(out, capacity, raw);
      return completed();
    }
    case cfg::kTlvItemSlot: {
      const uint8_t n = v[0], attach = v[kSlotAttach], idle = v[kSlotIdleClock], mechanism = v[kSlotMechanism],
                    name_length = v[kSlotNameLength];
      const uint16_t wire_fn = getU16(v + 1), swdio = getU16(v + 3), swclk = getU16(v + 5);
      const uint32_t max_hz = getU32(v + kSlotMaxSpeed);
      if (n >= kMaxSlots) return rejected(kRejectMalformed);   // slots_max
      if (name_length < 1 || name_length > kMaxName) return rejected(kRejectMalformed);
      for (uint8_t k = 0; k < name_length; ++k) if (!nameChar(static_cast<char>(v[kSlotFixed + k]))) return rejected(kRejectMalformed);
      if (!endpoint_.interfaceAt(wire_fn)) return rejected(kRejectUnknownFunction);
      if (attach > cfg::kSlotAttachAtBoot) return unsupportedTag(out, capacity, raw);
      int place = -1;
      for (size_t k = 0; k < place_count_; ++k) if (places_[k].wire_fn == wire_fn) place = static_cast<int>(k);
      if (place < 0) return unsupportedTag(out, capacity, raw);   // not a wire slots ride on (rvswd / swio) here
      if (!pairAllowed(*places_[place].port, swdio, swclk)) return unsupportedTag(out, capacity, raw);   // not a pair that wire allows
      DmiPhy &phy = places_[place].port->dm.phy();
      if (idle > reg::wire_rvswd::kIdleClockLow || (idle && !phy.canIdleClockLow())) return unsupportedTag(out, capacity, raw);
      if (!phy.keepsMaxHz(max_hz) || (max_hz && phy.minClockHz() && max_hz < phy.minClockHz())) return unsupportedTag(out, capacity, raw);
      if (mechanism != reg::target_console::kMechanismNone && mechanism > reg::target_console::kMechanismDmseq)
        return unsupportedTag(out, capacity, raw);
      return completed();
    }
    case cfg::kTlvItemBind: {
      // port(u8) kind(u8) id(u16) (probe.config §1.2); a slot that is not there is the whole's (derive)
      const uint8_t port = v[0], kind = v[1];
      const uint16_t id = getU16(v + 2);
      if (kind == Binds::kFixtureUart && !endpoint_.interfaceAt(id)) return rejected(kRejectUnknownFunction);
      if (kind != Binds::kSlotConsole && kind != Binds::kFixtureUart) return unsupportedTag(out, capacity, raw);
      if (kind == Binds::kFixtureUart) {
        bool is_uart = false;
        for (size_t i = 0; i < uart_count_; ++i) is_uart |= uarts_[i].fn == id;
        if (!is_uart) return unsupportedTag(out, capacity, raw);
      }
      if (port >= Binds::kMaxPorts || !endpoint_.isSerialPort(port)) return unsupportedTag(out, capacity, raw);
      return completed();
    }
    case cfg::kTlvItemUart: {
      const uint16_t fn = getU16(v);
      if (!endpoint_.interfaceAt(fn)) return rejected(kRejectUnknownFunction);
      bool is_uart = false;
      for (size_t i = 0; i < uart_count_; ++i) is_uart |= uarts_[i].fn == fn;
      // a baud out of reach, an unused value or reserved bit of format: unsupported (probe.config §1)
      if (!is_uart || !FixtureUart::baudWithinReach(getU32(v + 2)) || !FixtureUart::formatDefined(v[6]))
        return unsupportedTag(out, capacity, raw);
      return completed();
    }
    case cfg::kTlvItemDisable:
      if (!pins_->allowed(getU16(v))) return unsupportedTag(out, capacity, raw);   // not declared: as idle
      return completed();
    case kWifiItemTag: {
      // index below wifi_max (else unsupported with the tag as received); the passphrase none, 8-63 printable ASCII
      // or 64 hex digits (else malformed); an SSID with a 0x00 byte this probe cannot join (unsupported)
      const uint8_t index = v[0], ssid_length = v[1], pass_length = v[2 + ssid_length];
      const uint8_t *pass = v + 3 + ssid_length;
      if (index >= kMaxWifi) return unsupportedTag(out, capacity, raw);
      if (pass_length == 64) {
        for (uint8_t k = 0; k < 64; ++k)
          if (!isxdigit(pass[k])) return rejected(kRejectMalformed);
      } else if (pass_length != 0 && pass_length != kWifiPassHidden) {
        if (pass_length < 8 || pass_length > 63) return rejected(kRejectMalformed);
        for (uint8_t k = 0; k < pass_length; ++k)
          if (pass[k] < 0x20 || pass[k] > 0x7E) return rejected(kRejectMalformed);
      }
      if (memchr(v + 2, 0, ssid_length)) return unsupportedTag(out, capacity, raw);
      return completed();
    }
    default:
      return unsupportedTag(out, capacity, raw);
  }
}

// Everything the items say, and the rules between them (probe.config §1: no two slots on one wire and pair, names
// unique, at most max_connections at-boot slots a wire, binds naming slots that exist and have a console).
Result ProbeConfig::derive(const uint8_t *items, size_t length, Derived &d, uint8_t *out, size_t capacity) const {
  d.role_count = 0;
  d.disabled = 0;
  memset(d.idle, PinTable::kIdleUnset, sizeof d.idle);
  memset(d.idle_drive, PinTable::kDriveDefault, sizeof d.idle_drive);
  for (Slot &s : d.slots) s = Slot{};
  for (Binds::Spec &b : d.binds) b = Binds::Spec{};
  for (UartItem &u : d.uarts) u = UartItem{};
  for (Label &l : d.labels) l = Label{};
  d.wifi_count = 0;
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  size_t labels = 0;
  while (nextItem(items, length, at, tag, v, vlen)) {
    switch (tag) {
      case cfg::kTlvItemPlan:
        if (d.role_count >= Endpoint::kMaxRoles) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
        d.roles[d.role_count++] = {getU16(v), v[2], getU16(v + 3)};
        break;
      case cfg::kTlvItemLabel:
        if (labels < kMaxLabels) d.labels[labels++] = {true, getU16(v)};
        break;
      case cfg::kTlvItemIdle: {
        d.idle[getU16(v)] = v[2];
        const bool output = v[2] == PinTable::kIdleOutputLow || v[2] == PinTable::kIdleOutputHigh;
        d.idle_drive[getU16(v)] = output ? v[3] : PinTable::kDriveDefault;   // inputs: drive not read
        break;
      }
      case cfg::kTlvItemSlot: {
        Slot &s = d.slots[v[0]];
        const uint16_t wire_fn = getU16(v + 1);
        for (size_t k = 0; k < place_count_; ++k) if (places_[k].wire_fn == wire_fn) s.place = static_cast<uint8_t>(k);
        s.set = true;
        s.swdio = getU16(v + 3);
        s.swclk = getU16(v + 5);
        s.attach = v[kSlotAttach];
        s.retry_ms = getU32(v + kSlotRetryMs);
        s.max_hz = getU32(v + kSlotMaxSpeed);
        s.idle_low = v[kSlotIdleClock] != 0;
        s.mechanism = v[kSlotMechanism];
        s.name_length = v[kSlotNameLength];
        memcpy(s.name, v + kSlotFixed, s.name_length);
        s.name[s.name_length] = 0;
        break;
      }
      case cfg::kTlvItemBind: {
        Binds::Spec &b = d.binds[v[0]];
        b.set = true;
        b.source.kind = v[1];
        b.source.id = getU16(v + 2);
        break;
      }
      case cfg::kTlvItemUart:
        for (size_t i = 0; i < uart_count_; ++i)
          if (uarts_[i].fn == getU16(v)) d.uarts[i] = {true, getU32(v + 2), v[6]};
        break;
      case cfg::kTlvItemDisable:
        if (getU16(v) < PinTable::kChannels) d.disabled |= uint64_t{1} << getU16(v);
        break;
      case kWifiItemTag: {   // in index order (the store's); a store never holds pass_len 0xFF (set resolves it)
        if (d.wifi_count >= kMaxWifi) return rejected(kRejectMalformed);
        WifiEntry &e = d.wifi[d.wifi_count++];
        e = WifiEntry{};
        e.index = v[0];
        e.ssid_length = v[1];
        memcpy(e.ssid, v + 2, e.ssid_length);
        e.pass_length = v[2 + e.ssid_length];
        memcpy(e.pass, v + 3 + e.ssid_length, e.pass_length);
        break;
      }
      default:
        break;
    }
  }
  // idle and disable for one channel contradict (probe.config §1)
  for (uint8_t c = 0; c < PinTable::kChannels; ++c)
    if (((d.disabled >> c) & 1) && d.idle[c] != PinTable::kIdleUnset) return rejected(kRejectMalformed);
  // the whole: no two slots on one place with the same pair, names unique, at most one at-boot slot a wire
  // (max_connections 1), every bind's stream there (a slot with a console)
  for (size_t a = 0; a < kMaxSlots; ++a) {
    if (!d.slots[a].set) continue;
    uint8_t at_boot = 0;
    for (size_t b = 0; b < kMaxSlots; ++b) {
      if (!d.slots[b].set) continue;
      if (b != a && strcmp(d.slots[a].name, d.slots[b].name) == 0) return rejected(kRejectMalformed);
      if (d.slots[b].place != d.slots[a].place) continue;
      if (b > a && d.slots[b].swdio == d.slots[a].swdio && d.slots[b].swclk == d.slots[a].swclk) return rejected(kRejectMalformed);
      at_boot += d.slots[b].attach == cfg::kSlotAttachAtBoot;
    }
    if (at_boot > 1) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
  }
  for (uint8_t port = 0; port < Binds::kMaxPorts; ++port) {
    Binds::Source &src = d.binds[port].source;
    if (!d.binds[port].set) continue;
    if (src.kind == Binds::kSlotConsole &&
        (src.id >= kMaxSlots || !d.slots[src.id].set || d.slots[src.id].mechanism == reg::target_console::kMechanismNone))
      return rejected(kRejectMalformed);   // a slot that is not there, or has no console: the settings disagree
    if (!sourceFor(d.slots, src.kind, src.id, src)) return rejected(kRejectMalformed);
  }
  return completed();
}

bool ProbeConfig::sourceFor(const Slot *slots, uint8_t kind, uint16_t id, Binds::Source &out) const {
  out.kind = kind;
  out.id = id;
  if (kind == Binds::kSlotConsole) {
    if (id >= kMaxSlots || !slots[id].set) return false;
    out.stream = places_[slots[id].place].console;
    return true;
  }
  if (kind == Binds::kFixtureUart) {
    for (size_t i = 0; i < uart_count_; ++i) if (uarts_[i].fn == id) { out.stream = uarts_[i].uart; return true; }
  }
  return false;
}

// A candidate store checked as a whole, its plans put in through the endpoint (all or nothing; a refusal changes
// nothing), then made current. Disable and idle go before every other item (probe.config §2): a disabled channel is
// never touched, the idle states are set before the plans; then the slots (a replaced or removed one lets go of its
// connection and console, probe.config §1.1), the binds and the UARTs' items.
Result ProbeConfig::commit(uint8_t *candidate, size_t length, const uint16_t *plan_fns, size_t plan_fn_count, uint8_t *out,
                           size_t capacity, uint8_t plan_raw) {
  static Derived d;   // large: not on the stack
  const Result whole = derive(candidate, length, d, out, capacity);
  if (refused(whole)) return whole;
  // disable (probe.config §1): a channel in use now - a plan this change keeps, a pin something holds (a plan, a
  // connection), a slot this change keeps - cannot be disabled (cause 1); an item naming a disabled channel (a plan, a
  // slot's pins) is cause 5 with the channel
  if (pins_ && d.disabled) {
    RoleAssignment now[Endpoint::kMaxRoles];
    const size_t nnow = endpoint_.plan(now, Endpoint::kMaxRoles);
    auto listed = [&](uint16_t fn) {
      for (size_t k = 0; k < plan_fn_count; ++k) if (plan_fns[k] == fn) return true;
      return false;
    };
    const uint64_t newly = d.disabled & ~pins_->disabledMask();
    for (uint16_t c = 0; c < PinTable::kChannels; ++c) {
      if (!((newly >> c) & 1)) continue;
      bool replaced = false;   // in the old plan of a fn this change replaces: its holder goes
      for (size_t r = 0; r < nnow; ++r) {
        if (now[r].channel != c) continue;
        if (listed(now[r].function)) { replaced = true; continue; }
        return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, c);
      }
      if (!replaced && pins_->owner(c)) return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, c);
      for (uint8_t i = 0; i < kMaxSlots; ++i)
        if (slots_[i].set && d.slots[i].set && memcmp(&slots_[i], &d.slots[i], sizeof(Slot)) == 0 &&
            (slots_[i].swdio == c || slots_[i].swclk == c))
          return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, c);
    }
    auto off = [&](uint16_t c) { return c < PinTable::kChannels && ((d.disabled >> c) & 1); };
    for (size_t r = 0; r < d.role_count; ++r)
      if (off(d.roles[r].channel))
        return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, d.roles[r].channel);
    for (uint8_t i = 0; i < kMaxSlots; ++i) {
      if (!d.slots[i].set) continue;
      if (off(d.slots[i].swdio)) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, d.slots[i].swdio);
      if (off(d.slots[i].swclk)) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, d.slots[i].swclk);
    }
  }
  // idle before the plans: a free channel takes its new idle state now - an output idle drives before a gpio plan takes
  // the channel, which keeps that level until its first set (fixture §1) - and a channel the replaced plans release
  // goes to the new idle. A disabled channel (the saved disables are in the PinTable before applySaved; one disabled by
  // this change) only keeps the mode, never touched; one enabled again gets it from setDisabled below. A refused plan
  // puts the old back. An output idle's drive goes with its level (probe.config §1): a change of either applies both.
  static uint8_t was_idle[PinTable::kChannels], was_drive[PinTable::kChannels];
  memcpy(was_idle, idle_, sizeof was_idle);
  memcpy(was_drive, idle_drive_, sizeof was_drive);
  auto applyIdles = [&](const uint8_t *modes, const uint8_t *drives) {
    if (!pins_) return;
    for (uint16_t c = 0; c < PinTable::kChannels; ++c)
      if (modes[c] != idle_[c] || drives[c] != idle_drive_[c]) {
        idle_[c] = modes[c];
        idle_drive_[c] = drives[c];
        pins_->setIdle(c, idle_[c], !((d.disabled >> c) & 1), idle_drive_[c]);
      }
  };
  applyIdles(d.idle, d.idle_drive);
  if (plan_fn_count) {   // the plans of the fns touched, as the candidate has them
    RoleAssignment roles[Endpoint::kMaxRoles];
    size_t n = 0;
    for (size_t r = 0; r < d.role_count; ++r)
      for (size_t k = 0; k < plan_fn_count; ++k)
        if (d.roles[r].function == plan_fns[k]) roles[n++] = d.roles[r];
    const uint8_t reason = endpoint_.replacePlan(roles, n, plan_fns, plan_fn_count);
    if (reason) applyIdles(was_idle, was_drive);
    if (reason == kRejectUnavailable) return endpoint_.planUnavailable(out, capacity);   // what it met (core §4.3)
    if (reason == kRejectUnsupported) return unsupportedTag(out, capacity, plan_raw);   // the plan item, as received
    if (reason) return rejected(reason);
  }
  // accepted: make it current
  memcpy(items_, candidate, length);
  items_length_ = length;
  setDisabled(d.disabled);   // a channel enabled again goes to its (new) idle state here
  for (uint8_t i = 0; i < kMaxSlots; ++i) {
    const Slot &was = slots_[i], &now = d.slots[i];
    const bool changed = was.set != now.set || (now.set && memcmp(&was, &now, sizeof(Slot)) != 0);
    if (!changed) continue;
    if (was.set) dropSlot(i, reg::common::kMarkDetailClosedSlotChanged);   // replaced or removed: its shares go
    slots_[i] = now;
    runs_[i] = SlotRun{};
    // an at-boot slot set: attach now - but the saved ones not in a boot told to skip them (skipBootAttach)
    runs_[i].due = now.set && now.attach == cfg::kSlotAttachAtBoot && !(applying_saved_ && skip_boot_attach_);
  }
  for (uint8_t port = 0; port < Binds::kMaxPorts; ++port) {
    const Binds::Spec &was = binds_.spec(port), &now = d.binds[port];
    const bool same = was.set == now.set &&
                      (!now.set || (was.source.kind == now.source.kind && was.source.id == now.source.id &&
                                    was.source.stream == now.source.stream));
    if (!same) binds_.set(port, now);
  }
  for (size_t i = 0; i < uart_count_; ++i) {
    if (d.uarts[i].set) uarts_[i].uart->setItem(d.uarts[i].baud, d.uarts[i].format);
    else uarts_[i].uart->clearItem();
  }
  // the networks: handed over when the list changed (the passphrases' token renewed when one of them did)
  bool wifi_changed = d.wifi_count != wifi_count_, pass_changed = false;
  for (size_t i = 0; i < d.wifi_count && i < wifi_count_; ++i) {
    wifi_changed |= d.wifi[i] != wifi_entries_[i];
    pass_changed |= d.wifi[i].index != wifi_entries_[i].index || d.wifi[i].pass_length != wifi_entries_[i].pass_length ||
                    memcmp(d.wifi[i].pass, wifi_entries_[i].pass, d.wifi[i].pass_length) != 0;
  }
  if (wifi_changed) {
    if (pass_changed || d.wifi_count != wifi_count_) wifi_token_ += platformRandom32() | 1;   // always another value
    for (size_t i = 0; i < d.wifi_count; ++i) wifi_entries_[i] = d.wifi[i];
    wifi_count_ = d.wifi_count;
    if (wifi_) wifi_->apply(wifi_entries_, wifi_count_);
  }
  poll();
  return completed();
}

// set: the items given replace the keys they carry (a fn's plan items replace that fn's whole plan); the rest stay.
// Every item is checked before anything changes; the first refusal met is the answer (core §4.3: any one reason).
Result ProbeConfig::set(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  static uint8_t candidate[kMaxItems];
  memcpy(candidate, items_, items_length_);
  size_t clen = items_length_;
  uint16_t plan_fns[16];
  size_t plan_fn_count = 0;
  uint8_t plan_raw = cfg::kTlvItemPlan;   // a plan item's tag as received, for a plan the interface cannot take
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  for (size_t next = 0; at < length; at = next)   // whole TLVs first (core §2.3)
    if (!tlvAt(payload, length, at, tag, v, vlen, next)) return rejected(kRejectMalformed);
  at = 0;
  while (at < length) {
    size_t next = 0;
    tlvAt(payload, length, at, tag, v, vlen, next);
    const uint8_t raw = tag;
    tag &= ~kTagCritical;
    const Result r = checkItem(raw, v, vlen, out, capacity);
    if (refused(r)) return r;
    const size_t klen = keyLength(tag);
    size_t at2 = 0, vlen2 = 0;
    uint8_t tag2 = 0;
    const uint8_t *v2 = nullptr;
    while (at2 < at) {   // the items before this one: the same key twice is malformed
      size_t next2 = 0;
      tlvAt(payload, length, at2, tag2, v2, vlen2, next2);
      at2 = next2;
      if ((tag2 & ~kTagCritical) == tag && memcmp(v, v2, klen) == 0) return rejected(kRejectMalformed);
    }
    if (tag == cfg::kTlvItemPlan) {
      plan_raw = raw;
      bool listed = false;
      for (size_t k = 0; k < plan_fn_count; ++k) listed |= plan_fns[k] == getU16(v);
      if (!listed) {
        if (plan_fn_count >= 16) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
        plan_fns[plan_fn_count++] = getU16(v);
      }
    }
    at = next;
  }
  // the keys replaced come out (a plan's fn: its whole plan), the items go in
  for (size_t k = 0; k < plan_fn_count; ++k) {
    uint8_t fn[2];
    putU16(fn, plan_fns[k]);
    removeItems(candidate, clen, cfg::kTlvItemPlan, fn, 2);
  }
  at = 0;
  while (nextItem(payload, length, at, tag, v, vlen)) {
    tag &= ~kTagCritical;
    uint8_t kept[3 + 32 + 64];
    if (tag == kWifiItemTag && v[2 + v[1]] == kWifiPassHidden) {
      // pass_len 0xFF: the passphrase this entry has now (get's form sent back); no entry with this index: malformed
      size_t at2 = 0, vlen2 = 0;
      uint8_t tag2 = 0;
      const uint8_t *v2 = nullptr, *now = nullptr;
      size_t now_length = 0;
      while (nextItem(items_, items_length_, at2, tag2, v2, vlen2))
        if (tag2 == kWifiItemTag && v2[0] == v[0]) { now = v2; now_length = vlen2; }
      if (!now) return rejected(kRejectMalformed);
      const size_t head = 2u + v[1], pass = now_length - (2u + now[1]);   // pass_len and the passphrase
      memcpy(kept, v, head);
      memcpy(kept + head, now + 2 + now[1], pass);
      v = kept;
      vlen = head + pass;
    }
    if (tag != cfg::kTlvItemPlan) removeItems(candidate, clen, tag, v, keyLength(tag));
    if (!insertItem(candidate, clen, sizeof candidate, tag, v, vlen)) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
  }
  return commit(candidate, clen, plan_fns, plan_fn_count, out, capacity, plan_raw);
}

// unset: n(u8), n x (len(u8), tag(u8), key) - len is the key's length, which is the tag's (probe.config §2: plan
// fn(u16), slot, bind and wifi u8, the others u16; another len is malformed). A key that is not there does nothing;
// the rest is as set.
Result ProbeConfig::unset(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 1) return rejected(kRejectMalformed);
  const uint8_t n = payload[0];
  size_t at = 1;
  int undeclared = -1;   // the first key whose tag this probe does not declare
  for (uint8_t i = 0; i < n; ++i) {   // the shape of every key first
    if (at + 1 >= length || at + 2u + payload[at] > length || payload[at] < 1) return rejected(kRejectMalformed);
    // the tag is the item's tag itself (no critical bit here): one this probe does not declare (describe items) is
    // unsupported with the tag as received (§2, core §4.3)
    const uint8_t tag = payload[at + 1];
    const size_t klen = tag == cfg::kTlvItemPlan ? 2 : keyLength(tag);
    if (!klen || !declares(tag)) { if (undeclared < 0) undeclared = tag; }
    else if (payload[at] != klen) return rejected(kRejectMalformed);
    at += 2u + payload[at];
  }
  Tail tail;
  const Result parsed = tail.parse(payload + at, length - at, out, capacity);
  if (refused(parsed)) return parsed;
  if (undeclared >= 0) return unsupportedTag(out, capacity, static_cast<uint8_t>(undeclared));
  static uint8_t candidate[kMaxItems];
  memcpy(candidate, items_, items_length_);
  size_t clen = items_length_;
  uint16_t plan_fns[16];
  size_t plan_fn_count = 0;
  at = 1;
  for (uint8_t i = 0; i < n; ++i) {
    const uint8_t tag = payload[at + 1];
    const uint8_t *key = payload + at + 2;
    if (tag == cfg::kTlvItemPlan) {
      bool listed = false;
      for (size_t k = 0; k < plan_fn_count; ++k) listed |= plan_fns[k] == getU16(key);
      if (!listed && plan_fn_count < 16) plan_fns[plan_fn_count++] = getU16(key);
      removeItems(candidate, clen, tag, key, 2);
    } else {
      removeItems(candidate, clen, tag, key, keyLength(tag));
    }
    at += 2u + payload[at];
  }
  const Result r = commit(candidate, clen, plan_fns, plan_fn_count, out, capacity);
  if (refused(r)) return r;
  if (capacity < 4) return failed();
  putU32(out, hash());
  return completed(4);
}

// ---- the slots -----------------------------------------------------------------------------------------------------

bool ProbeConfig::bound(uint8_t slot) const {
  for (uint8_t port = 0; port < Binds::kMaxPorts; ++port) {
    const Binds::Spec &b = binds_.spec(port);
    if (b.set && b.source.kind == Binds::kSlotConsole && b.source.id == slot) return true;
  }
  return false;
}

// The slot's share of its connection and of the console its bind opened goes (probe.config §1.1: replaced or removed,
// detail 3; no longer in any bind, detail 1). Nothing else of the target is touched (oep-if-debug §2).
void ProbeConfig::dropSlot(uint8_t i, uint8_t detail) {
  const Slot &s = slots_[i];
  if (!s.set) return;
  const Place &p = places_[s.place];
  if (!p.port->connected || !onPair(s)) return;
  p.console->bindClose(detail);
  releaseConnection(*p.port, DebugPort::kUserSlot, false);
}

// One slot (probe.config §3): a bound slot rides any connection on its pair (whoever attached it), with its console
// open; an at-boot slot attaches by itself (without stopping the hart) at boot, when set, and every retry_ms while the
// target is not there, and - connected - reads DMSTATUS every retry_ms to see that it still answers (§3.1: the line lost
// closes the connection and the retries start). Which target is there the host checks with connections' tid.
void ProbeConfig::runSlot(uint8_t i) {
  const Slot &s = slots_[i];
  SlotRun &r = runs_[i];
  const Place &p = places_[s.place];
  DebugPort &port = *p.port;
  const bool at_boot = s.attach == cfg::kSlotAttachAtBoot;
  // The attach and the liveness check wait while the wire is paused (DmiPhy::wirePaused: a logic sampler's window on
  // the classic ESP32) and run at the first poll after: a request for the wire inside a trigger search gets a turn at
  // once, and the check's frames, every retry_ms, held the sampler up - the bench's forced captures slipped in 4 to 17 %
  // at every rate from 400 kHz down to 100 kHz, about once a second (retry 1 s), where immediate windows never did.
  const bool paused = port.dm.phy().wirePaused();
  if (!port.connected && at_boot) {
    const bool retry = s.retry_ms && r.tried && static_cast<uint32_t>(millis() - r.last_try_ms) >= s.retry_ms;
    // held back while the gate is closed (setAttachGate) or the wire paused
    if ((r.due || retry) && (!attach_gate_ || attach_gate_()) && !paused) {
      r.due = false;
      r.tried = true;
      r.last_try_ms = millis();
      r.last_try_ns = nowNs();
      uint32_t status = 0;
      // the link to the slot's pair first (host-chosen pins; a fixed pair is always there), unless its pins are held
      if (usePair(port, s.swdio, s.swclk)) attachRunning(port, DebugPort::kUserSlot, status, s.max_hz, s.idle_low);
      r.last_check_ms = millis();
    }
  }
  if (!port.connected || !onPair(s)) return;   // no link, or the link is on another pair (another slot, the host)
  if (at_boot && s.retry_ms && !paused && static_cast<uint32_t>(millis() - r.last_check_ms) >= s.retry_ms) {   // liveness
    r.last_check_ms = millis();
    if (!checkConnection(port)) { r.last_try_ms = millis(); return; }   // gone: the retries begin after retry_ms
  }
  if (at_boot || bound(i)) {
    port.users |= DebugPort::kUserSlot;
    if (bound(i) && s.mechanism != reg::target_console::kMechanismNone && !p.console->isOpen()) p.console->bindOpen(s.mechanism);
    if (!bound(i) && (p.console->users() & TargetConsoleStream::kUserSlot)) p.console->bindClose();
  } else if (port.users & DebugPort::kUserSlot) {
    p.console->bindClose();
    releaseConnection(port, DebugPort::kUserSlot, false);
  }
}

void ProbeConfig::poll() {
  for (size_t k = 0; k < place_count_; ++k) {   // the slot on the pair the link is on now: the connections entry's slot
    DebugPort &port = *places_[k].port;
    port.slot = 0xff;
    for (uint8_t i = 0; i < kMaxSlots; ++i) if (slots_[i].set && slots_[i].place == k && onPair(slots_[i])) port.slot = i;
    if (port.slot == 0xff && (port.users & DebugPort::kUserSlot)) {
      places_[k].console->bindClose();
      releaseConnection(port, DebugPort::kUserSlot, false);
    }
  }
  for (uint8_t i = 0; i < kMaxSlots; ++i) if (slots_[i].set) runSlot(i);
}

// ---- storage -------------------------------------------------------------------------------------------------------

// The interfaces `items` name (plan fns, slot wire_fns, the fixture UARTs binds carry and uart items set), each once:
// count(u8), then fn(u16) instance(u16) revision(u8) name_len(u8) name. 0: no room.
size_t ProbeConfig::identities(const uint8_t *items, size_t length, uint8_t *out, size_t capacity) const {
  uint16_t fns[16];
  size_t n = 0;
  auto note = [&](uint16_t fn) {
    for (size_t k = 0; k < n; ++k) if (fns[k] == fn) return;
    if (n < 16) fns[n++] = fn;
  };
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  while (nextItem(items, length, at, tag, v, vlen)) {
    if ((tag == cfg::kTlvItemPlan || tag == cfg::kTlvItemUart) && vlen >= 2) note(getU16(v));
    if (tag == cfg::kTlvItemSlot && vlen >= 3) note(getU16(v + 1));
    if (tag == cfg::kTlvItemBind && vlen == kBindLength && v[1] == Binds::kFixtureUart) note(getU16(v + 2));
  }
  if (capacity < 1) return 0;
  size_t used = 1;
  out[0] = static_cast<uint8_t>(n);
  for (size_t k = 0; k < n; ++k) {
    const Interface *it = endpoint_.interfaceAt(fns[k]);
    const char *name = it ? it->name() : "";
    const size_t name_len = strlen(name);
    if (used + 6 + name_len > capacity) return 0;
    putU16(out + used, fns[k]);
    putU16(out + used + 2, endpoint_.instanceOf(fns[k]));
    out[used + 4] = it ? it->revision() : 0;
    out[used + 5] = static_cast<uint8_t>(name_len);
    memcpy(out + used + 6, name, name_len);
    used += 6 + name_len;
  }
  return used;
}

void ProbeConfig::load() {
  uint8_t blob[kMaxBlob];
  size_t length = 0;
  saved_length_ = ids_length_ = 0;
  saved_hash_ = 0;
  if (!storeRead(blob, sizeof blob, length)) {
    if (storeOld()) {   // saved by a firmware of the form before (59dd028): not read, the host sets it again
      storage_state_ = cfg::kStorageStateUnreadable;
      unreadable_ = cfg::kStorageUnreadableForm;
    }
    return;
  }
  // the identities, then the items (whole TLVs: anything else is another form)
  size_t at = 1;
  for (uint8_t k = 0; blob[0] && k < blob[0] && at + 6 <= length; ++k) at += 6u + blob[at + 5];
  bool whole = at <= length && at <= sizeof ids_ && length - at <= sizeof saved_;
  for (size_t p = at; whole && p < length;) {
    uint8_t tag = 0;
    const uint8_t *v = nullptr;
    size_t vlen = 0, next = 0;
    whole = tlvAt(blob, length, p, tag, v, vlen, next);
    p = next;
  }
  if (!whole) {
    storage_state_ = cfg::kStorageStateUnreadable;
    unreadable_ = cfg::kStorageUnreadableForm;
    return;
  }
  memcpy(ids_, blob, at);
  ids_length_ = at;
  memcpy(saved_, blob + at, length - at);
  saved_length_ = length - at;
  storage_state_ = cfg::kStorageStateApplied;   // until applySaved says otherwise
}

uint64_t ProbeConfig::disabledIn(const uint8_t *items, size_t length) {
  uint64_t mask = 0;
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  while (nextItem(items, length, at, tag, v, vlen))
    if (tag == cfg::kTlvItemDisable && vlen >= 2 && getU16(v) < PinTable::kChannels) mask |= uint64_t{1} << getU16(v);
  return mask;
}

void ProbeConfig::setDisabled(uint64_t mask) {
  if (!pins_) return;
  pins_->setDisabled(mask);   // a channel enabled again goes to its idle state there
  endpoint_.setDisabled(mask);
}

void ProbeConfig::applySaved() {
  applySavedItems();
  setDisabled(disabledIn(items_, items_length_));   // not applied: the channels load() kept aside are free again
}

void ProbeConfig::applySavedItems() {
  if (!saved_length_) return;
  // Each interface the items name, found again by (name, instance, revision): the fn it has in this firmware.
  uint16_t from[16], to[16];
  size_t n = 0;
  for (size_t at = 1, k = 0; k < ids_[0] && n < 16; ++k) {
    const uint16_t fn = getU16(ids_ + at), instance = getU16(ids_ + at + 2);
    const uint8_t revision = ids_[at + 4], name_len = ids_[at + 5];
    const char *name = reinterpret_cast<const char *>(ids_ + at + 6);
    at += 6u + name_len;
    uint16_t now = 0;
    for (uint16_t f = 1; f <= 255 && !now; ++f) {
      const Interface *it = endpoint_.interfaceAt(f);
      if (!it) break;
      if (endpoint_.instanceOf(f) == instance && it->revision() == revision && strlen(it->name()) == name_len &&
          memcmp(it->name(), name, name_len) == 0) now = f;
    }
    if (!now) {   // gone, or another revision: nothing is applied (a jig half set up is worse than none)
      storage_state_ = cfg::kStorageStateUnreadable;
      unreadable_ = cfg::kStorageUnreadableInterface;
      return;
    }
    from[n] = fn;
    to[n++] = now;
  }
  auto map = [&](uint16_t fn) { for (size_t k = 0; k < n; ++k) if (from[k] == fn) return to[k]; return fn; };
  static uint8_t items[kMaxItems];
  memcpy(items, saved_, saved_length_);
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *cv = nullptr;
  while (nextItem(items, saved_length_, at, tag, cv, vlen)) {   // renumber in place
    uint8_t *v = items + (cv - items);
    if ((tag == cfg::kTlvItemPlan || tag == cfg::kTlvItemUart) && vlen >= 2) putU16(v, map(getU16(v)));
    if (tag == cfg::kTlvItemSlot && vlen >= 3) putU16(v + 1, map(getU16(v + 1)));
    if (tag == cfg::kTlvItemBind && vlen == kBindLength && v[1] == Binds::kFixtureUart) putU16(v + 2, map(getU16(v + 2)));
  }
  // a bind whose port is not a serial port of this firmware: unreadable reason 2, as an interface gone (probe.config §2;
  // not a refusal of applying, reason 3)
  at = 0;
  while (nextItem(items, saved_length_, at, tag, cv, vlen))
    if (tag == cfg::kTlvItemBind && vlen >= 1 && !endpoint_.isSerialPort(cv[0])) {
      storage_state_ = cfg::kStorageStateUnreadable;
      unreadable_ = cfg::kStorageUnreadableInterface;
      return;
    }
  // the renumbered items may be out of get's order (fns moved): sorted into a fresh store, then set as a whole
  uint8_t scratch[64];
  static uint8_t candidate[kMaxItems];
  size_t clen = 0;
  uint16_t plan_fns[16];
  size_t plan_fn_count = 0;
  at = 0;
  while (nextItem(items, saved_length_, at, tag, cv, vlen)) {
    const Result r = checkItem(tag, cv, vlen, scratch, sizeof scratch);
    if (refused(r) || !insertItem(candidate, clen, sizeof candidate, tag, cv, vlen)) {
      storage_state_ = cfg::kStorageStateUnreadable;
      unreadable_ = r.resolution == kResolutionRejected && r.detail == kRejectUnknownFunction ? cfg::kStorageUnreadableInterface
                                                                                              : cfg::kStorageUnreadableRefused;
      return;
    }
    if (tag == cfg::kTlvItemPlan) {
      bool listed = false;
      for (size_t k = 0; k < plan_fn_count; ++k) listed |= plan_fns[k] == getU16(cv);
      if (!listed && plan_fn_count < 16) plan_fns[plan_fn_count++] = getU16(cv);
    }
  }
  applying_saved_ = true;
  const Result r = commit(candidate, clen, plan_fns, plan_fn_count, scratch, sizeof scratch);
  applying_saved_ = false;
  if (r.resolution != kResolutionCompleted || r.detail != kOutcomeSuccess) {
    storage_state_ = cfg::kStorageStateUnreadable;
    unreadable_ = cfg::kStorageUnreadableRefused;
    return;
  }
  saved_hash_ = hash();   // the saved items as they read in this boot (probe.config §3.3)
}

// ---- describe and the operations -------------------------------------------------------------------------------------

size_t ProbeConfig::describe(uint8_t *out, size_t capacity) {   // declarations only (core §7.3); the state is op 0x06
  TlvWriter w(out, capacity);
  if (storage_) w.u32(cfg::kTlvDescribeStorage, kMaxItems);   // exactly when save / erase are offered (ops)
  static const uint8_t kItems[] = {cfg::kTlvItemPlan, cfg::kTlvItemLabel, cfg::kTlvItemSlot, cfg::kTlvItemBind,
                                   cfg::kTlvItemUart, cfg::kTlvItemIdle, cfg::kTlvItemDisable, kWifiItemTag};
  uint8_t items[sizeof kItems];
  size_t n = 0;
  for (uint8_t tag : kItems) if (declares(tag)) items[n++] = tag;   // label, idle and disable need the pins, slot slots
  w.put(cfg::kTlvDescribeItems, items, n);
  w.u8(cfg::kTlvDescribeSlotsMax, place_count_ ? static_cast<uint8_t>(kMaxSlots) : 0);
  if (wifi_) w.u8(kWifiDescribeMax, static_cast<uint8_t>(kMaxWifi));   // wifi_max (probe.config §4)
  return w.ok() ? w.length() : 0;
}

// slot_state (probe.config §3.3): slot(u8) state(u8: 0 its connection is there, 1 not) connection(u16, 0 none)
// last_try_at_ns(u64: the last automatic attach, all ones when the probe has not tried one)
constexpr size_t kSlotStateLength = 12, kBindStateLength = 2;
size_t ProbeConfig::slotState(uint8_t i, uint8_t *out) const {
  const Slot &s = slots_[i];
  const SlotRun &r = runs_[i];
  const DebugPort &port = *places_[s.place].port;
  const bool connected = port.connected && onPair(s);
  out[0] = i;
  out[1] = connected ? cfg::kSlotStateConnected : cfg::kSlotStateAbsent;
  putU16(out + 2, connected ? port.number : 0);
  putU64(out + 4, r.tried ? r.last_try_ns : kNeverNs);
  return kSlotStateLength;
}

// state(first_slot u8, first_bind u8) -> more(u8) storage_state(u8) storage_hash(u32) unreadable_reason(u8)
//   n_slots(u8) n_slots x slot_state n_binds(u8) n_binds x bind_state [TLV] (no element lengths, core §2.3)
Result ProbeConfig::state(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
  if (refused(parsed)) return parsed;
  constexpr size_t kWifiTlv = kTlvHeader + 8;   // state entry reason rssi ipv4
  if (capacity < 9 + (wifi_ ? kWifiTlv : 0)) return failed();
  const size_t room = capacity - (wifi_ ? kWifiTlv : 0);   // the wifi TLV after the lists, on every page
  out[1] = storage_state_;
  putU32(out + 2, storage_state_ == cfg::kStorageStateApplied ? saved_hash_ : 0);
  out[6] = storage_state_ == cfg::kStorageStateUnreadable ? unreadable_ : 0;
  bool more = false;
  size_t used = 8;
  uint8_t n = 0, index = 0;
  for (uint8_t i = 0; i < kMaxSlots; ++i) {
    if (!slots_[i].set) continue;
    if (index++ < payload[0]) continue;
    if (used + kSlotStateLength + 1 > room) { more = true; break; }   // a slot_state, and n_binds after it
    used += slotState(i, out + used);
    ++n;
  }
  out[7] = n;
  const size_t binds_at = used++;
  n = index = 0;
  for (uint8_t port = 0; port < Binds::kMaxPorts && !more; ++port) {
    const Binds::Spec &b = binds_.spec(port);
    if (!b.set) continue;
    if (index++ < payload[1]) continue;
    if (used + kBindStateLength > room) { more = true; break; }
    out[used] = port;   // port flow
    out[used + 1] = endpoint_.held(port) ? cfg::kBindFlowHeld : binds_.streaming(port) ? cfg::kBindFlowStreaming : cfg::kBindFlowIdle;
    used += kBindStateLength;
    ++n;
  }
  out[binds_at] = n;
  out[0] = more ? 1 : 0;
  if (wifi_) {   // state TLV 0x01 wifi: state(u8) entry(u8, 0xFF none) reason(u8) rssi(i8) ipv4(4); rssi, ipv4 0 unless state 2
    const WifiControl::Status st = wifi_->status();
    const bool up = st.state == WifiControl::kStateConnected;
    putTlvHeader(out + used, kWifiStateTlv, 8);
    out[used + kTlvHeader] = st.state;
    out[used + kTlvHeader + 1] = st.entry;
    out[used + kTlvHeader + 2] = st.reason;
    out[used + kTlvHeader + 3] = up ? static_cast<uint8_t>(st.rssi) : 0;
    if (up) memcpy(out + used + kTlvHeader + 4, st.ipv4, 4);
    else memset(out + used + kTlvHeader + 4, 0, 4);
    used += kWifiTlv;
  }
  return completed(used);
}

Result ProbeConfig::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (op) {
    case cfg::kOpGet: {   // first(u16) -> more(u8) hash(u32) items from the first-th on (by tag, then key)
      // first(u16) and nothing after it: the answer is a list of items itself
      if (length != 2) return rejected(kRejectMalformed);
      if (capacity < 5) return failed();
      const size_t room = capacity;
      size_t at = 0, index = 0, put = 5;
      const uint16_t first = getU16(payload);
      bool more = false;
      while (at < items_length_) {
        uint8_t tag = 0;
        const uint8_t *v = nullptr;
        size_t vlen = 0, next = 0;
        if (!tlvAt(items_, items_length_, at, tag, v, vlen, next)) break;
        const size_t item = shown(tag, v, vlen, nullptr);   // a wifi item without its passphrase
        if (index++ >= first) {
          if (put + item > room) { more = true; break; }
          shown(tag, v, vlen, out + put);
          put += item;
        }
        at = next;
      }
      out[0] = more ? 1 : 0;
      putU32(out + 1, hash());
      return completed(put);
    }
    case cfg::kOpSet: {
      const Result r = set(payload, length, out, capacity);
      if (r.resolution != kResolutionCompleted || r.detail != kOutcomeSuccess || capacity < 4) return r;
      putU32(out, hash());
      return completed(4);
    }
    case cfg::kOpUnset: return unset(payload, length, out, capacity);
    case cfg::kOpState: return state(payload, length, out, capacity);
    case cfg::kOpSave: {
      Tail tail;
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t h = hash();
      uint8_t blob[kMaxBlob];
      const size_t ids = identities(items_, items_length_, blob, sizeof ids_);
      if (!ids) return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
      if (!(saved_length_ == items_length_ && memcmp(saved_, items_, items_length_) == 0 && ids_length_ == ids &&
            memcmp(ids_, blob, ids) == 0)) {
        memcpy(blob + ids, items_, items_length_);   // the same is not written again; the whole blob is replaced
        if (!storeWrite(blob, items_length_ ? ids + items_length_ : 0)) return failed();
        memcpy(saved_, items_, items_length_);
        saved_length_ = items_length_;
        memcpy(ids_, blob, ids);
        ids_length_ = ids;
        unreadable_ = 0;
        storage_state_ = items_length_ ? cfg::kStorageStateApplied : cfg::kStorageStateNone;
      }
      saved_hash_ = items_length_ ? h : 0;
      if (capacity < 4) return failed();
      putU32(out, h);
      return completed(4);
    }
    case cfg::kOpErase: {
      Tail tail;
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!storeWrite(nullptr, 0)) return failed();
      saved_length_ = ids_length_ = 0;
      unreadable_ = 0;
      saved_hash_ = 0;
      storage_state_ = cfg::kStorageStateNone;
      return completed();
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif

namespace oep {
namespace {

// A label's text equal to `slot` "." `line` (slot nullptr: `line` alone), ASCII case ignored (probe.config §1.3).
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

// How many labels (TLVs `tag`: channel(u16) text) name the line, the channel of the last one in `channel`.
size_t labelsNaming(const uint8_t *items, size_t length, const char *slot, const char *line, uint16_t &channel,
                    uint8_t tag_wanted = reg::probe_config::kTlvItemLabel) {
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
  if (slot) {   // (a)
    const size_t n = labelsNaming(items, length, slot, line, channel);
    if (n) return n == 1 ? channel : 0xffff;   // "S.N": one channel, or none when two or more
    size_t slots = 0, at = 0;
    while (at < length) {
      uint8_t tag = 0;
      const uint8_t *v = nullptr;
      size_t vlen = 0, next = 0;
      if (!tlvAt(items, length, at, tag, v, vlen, next)) break;
      at = next;
      slots += tag == reg::probe_config::kTlvItemSlot;
    }
    if (slots > 1) return 0xffff;              // "N" alone names a line only with at most one slot
  }
  const size_t n = labelsNaming(items, length, nullptr, line, channel);   // (b) a settings label "N"
  if (n) return n == 1 ? channel : 0xffff;
  // (c) a firmware label "N" (fn 0 describe's label 0x46)
  return labelsNaming(firmware, firmware_length, nullptr, line, channel, reg::core::kTlvDescribeLabel) == 1 ? channel : 0xffff;
}

}  // namespace oep
