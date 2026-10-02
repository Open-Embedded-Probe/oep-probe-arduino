// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepConfig.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_CONFIG)

#if defined(ARDUINO_ARCH_ESP32)
#include <Preferences.h>
#elif defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
#endif
#include <stdio.h>

#include "OepEndpoint.h"

namespace oep {

namespace cfg = reg::probe_config;

namespace {

// Where a bind's k-th stream (kind, id) starts, or 0 when the list is cut short or an element's len is under 3
// (probe.config §1.2: n × (len, kind, id); what follows the 3 bytes is for later fields, skipped).
size_t bindStreamAt(const uint8_t *v, size_t len, uint8_t k) {
  size_t at = 4;
  for (uint8_t i = 0;; ++i) {
    if (at >= len || v[at] < 3 || at + 1u + v[at] > len) return 0;
    if (i == k) return at + 1;
    at += 1u + v[at];
  }
}
// "items4": the interfaces the items name, then the items (2026-10-01: retry_ms / max_speed_hz u32 in the slot, the
// uart item, the long TLV form - a blob of 0.0.21 ("items3") is not read)
constexpr const char *kNvsNamespace = "oepcfg", *kNvsItems = "items4";
constexpr size_t kSlotFixed = 19;   // slot wire_fn swdio swclk attach retry_ms max_speed_hz idle_clock mechanism name_len

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
bool storeWrite(const uint8_t *blob, size_t length) {   // length 0: nothing saved
  Preferences p;
  if (!p.begin(kNvsNamespace, false)) return false;
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
#else
constexpr uint32_t kEepromMagic = 0x4f455034;   // "OEP4"
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

// The mark in front of a mixed line (probe.config §1.2): a slot's name; a fixture UART's RX channel label when the
// settings give one, else name#instance.
size_t ProbeConfig::nameOf(void *self, uint8_t kind, uint16_t id, char *out, size_t room) {
  const ProbeConfig &c = *static_cast<ProbeConfig *>(self);
  if (kind == Binds::kSlotConsole && id < kMaxSlots && c.slots_[id].set) {
    const size_t n = c.slots_[id].name_length < room ? c.slots_[id].name_length : room;
    memcpy(out, c.slots_[id].name, n);
    return n;
  }
  if (kind == Binds::kFixtureUart) {
    RoleAssignment roles[Endpoint::kMaxRoles];
    const size_t n = c.endpoint_.plan(roles, Endpoint::kMaxRoles);
    for (size_t i = 0; i < n; ++i) {
      if (roles[i].function != id || roles[i].role != reg::fixture_uart::kRoleRx) continue;
      size_t at = 0, vlen = 0;
      uint8_t tag = 0;
      const uint8_t *v = nullptr;
      while (nextItem(c.items_, c.items_length_, at, tag, v, vlen)) {
        if (tag != cfg::kTlvItemLabel || vlen < 2 || getU16(v) != roles[i].channel) continue;
        size_t k = 0;
        for (size_t j = 2; j < vlen && k < room; ++j) {
          const uint8_t ch = v[j];
          out[k++] = (ch == ']' || ch < 0x20) ? '_' : static_cast<char>(ch);
        }
        return k;
      }
    }
    const Interface *it = c.endpoint_.interfaceAt(id);
    const int n2 = snprintf(out, room, "%s#%u", it ? it->name() : "?", it ? static_cast<unsigned>(it->instance()) : 0u);
    return n2 < 0 ? 0 : (static_cast<size_t>(n2) < room ? static_cast<size_t>(n2) : room);
  }
  const int n = snprintf(out, room, "?");
  return n < 0 ? 0 : static_cast<size_t>(n);
}

// The output strengths of the probe's oep.fixture.gpio (fixture §1.1, describe drive_levels): the chip's levels when
// a gpio fixture is there to declare them, else none (an idle's drive is then kept but not applied, probe.config §1).
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

// The items describe declares: idle and disable only on a probe with a pin table.
bool ProbeConfig::declares(uint8_t tag) const {
  return keyLength(tag) && (pins_ || (tag != cfg::kTlvItemIdle && tag != cfg::kTlvItemDisable));
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
    default: return 0;
  }
}

// The key as a number, for the canonical order (plan: fn, then role, then channel).
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

// Insert one item at its canonical place. false: no room.
bool ProbeConfig::insertItem(uint8_t *store, size_t &length, size_t capacity, uint8_t tag, const uint8_t *value, size_t vlen) {
  const size_t size = tlvSize(vlen);
  if (length + size > capacity) return false;
  size_t at = 0, insert_at = length;
  uint8_t t = 0;
  const uint8_t *v = nullptr;
  size_t l = 0;
  while (nextItem(store, length, at, t, v, l)) {
    if (itemBefore(tag, value, vlen, t, v, l)) { insert_at = static_cast<size_t>(v - store) - (l < 255 ? 2 : 4); break; }
  }
  memmove(store + insert_at + size, store + insert_at, length - insert_at);
  size_t hlen = 0;
  store[insert_at] = tag;
  if (vlen < 255) { store[insert_at + 1] = static_cast<uint8_t>(vlen); hlen = 2; }
  else { store[insert_at + 1] = reg::kTlvLenLong; putU16(store + insert_at + 2, static_cast<uint16_t>(vlen)); hlen = 4; }
  memcpy(store + insert_at + hlen, value, vlen);
  length += size;
  return true;
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

// One item on its own (probe.config §1, the table in §2): its shape and the values this probe has.
// `raw`: the item's tag as received (critical bit included); what it cannot take is refused unsupported with it
// (probe.config §1, core §4.3: a value inside a TLV names that TLV's tag, 0x00 is for the fixed part only).
Result ProbeConfig::checkItem(uint8_t raw, const uint8_t *v, size_t len, uint8_t *out, size_t capacity) const {
  switch (raw & ~kTagCritical) {
    case cfg::kTlvItemPlan: {
      if (len < 5) return rejected(kRejectMalformed);
      const uint16_t fn = getU16(v);
      if (fn == 0) return rejected(kRejectMalformed);
      if (!endpoint_.interfaceAt(fn)) return rejected(kRejectUnknownFunction);
      return completed();
    }
    case cfg::kTlvItemLabel:
      if (len < 3) return rejected(kRejectMalformed);   // a channel and at least one byte of text
      return completed();
    case cfg::kTlvItemIdle: {
      if (len < 3) return rejected(kRejectMalformed);
      // the strength (probe.config §1, fixture §1.1): drive_kind(u8) drive_value(u16) after mode, mode 3 / 4 only
      const bool output = v[2] == PinTable::kIdleOutputLow || v[2] == PinTable::kIdleOutputHigh;
      if (len == 4 || len == 5) return rejected(kRejectMalformed);
      if (len >= 6 && !output) return rejected(kRejectMalformed);
      // an idle mode of 5 or more, an undefined drive_kind (2+): unsupported with the item's tag (probe.config §2's table)
      if (v[2] > PinTable::kIdleOutputHigh) return unsupportedTag(out, capacity, raw);
      if (len >= 6 && v[3] > reg::fixture_gpio::kDriveKindMaxMa) return unsupportedTag(out, capacity, raw);
      if (!pins_) return unsupportedTag(out, capacity, raw);
      if (getU16(v) >= PinTable::kChannels || !pins_->allowed(getU16(v))) return unsupportedTag(out, capacity, raw);
      // output low / high (probe.config §1): only on a channel this probe can drive
      if (output && !pins_->canOutput(getU16(v))) return unsupportedTag(out, capacity, raw);
      uint8_t level = 0;   // a level number this probe does not have (drive_levels declared); without them it is kept
      const DriveLevels levels = driveLevels();
      if (len >= 6 && levels.count && !PinTable::driveLevelOf(levels, v[3], getU16(v + 4), level))
        return unsupportedTag(out, capacity, raw);
      return completed();
    }
    case cfg::kTlvItemSlot: {
      if (len < kSlotFixed) return rejected(kRejectMalformed);
      const uint8_t n = v[0], attach = v[7], idle = v[16], mechanism = v[17], name_length = v[18];
      const uint16_t wire_fn = getU16(v + 1), swdio = getU16(v + 3), swclk = getU16(v + 5);
      const uint32_t retry_ms = getU32(v + 8), max_hz = getU32(v + 12);
      if (len < kSlotFixed + name_length + 1u) return rejected(kRejectMalformed);   // up to lock_len
      if (n >= kMaxSlots || !place_count_) return rejected(kRejectMalformed);   // slots_max
      // an attach policy or idle_clock a later revision may define (2+): unsupported with the item's tag (C-02), once
      // the form is known to be right; the rules that read them apply only to the defined values
      const bool undefined = attach > cfg::kSlotAttachAtBoot || idle > reg::wire_rvswd::kIdleClockLow;
      if (attach <= cfg::kSlotAttachAtBoot && retry_ms && attach != cfg::kSlotAttachAtBoot) return rejected(kRejectMalformed);
      if (name_length < 1 || name_length > kMaxName) return rejected(kRejectMalformed);
      for (uint8_t k = 0; k < name_length; ++k) if (!nameChar(static_cast<char>(v[kSlotFixed + k]))) return rejected(kRejectMalformed);
      const uint8_t lock_len = v[kSlotFixed + name_length];
      const uint8_t *lock = v + kSlotFixed + name_length + 1;
      if (len < kSlotFixed + name_length + 1u + lock_len) return rejected(kRejectMalformed);
      if (lock_len && (lock_len < 3 || lock_len % 2 == 0 || lock[0] == 0)) return rejected(kRejectMalformed);
      // n = the scheme's length (the registry's target_id_len); a scheme not in the table has no length to check
      // against and is refused unsupported below (probe.config §1.1: "whether defined or not", C-02)
      const bool scheme_known =
          lock_len && (lock[0] == reg::common::kTargetIdSchemeWchDmi7f || lock[0] == reg::common::kTargetIdSchemeTargetsel);
      const uint8_t scheme_len = lock_len && lock[0] == reg::common::kTargetIdSchemeTargetsel ? reg::common::kTargetIdLenTargetsel
                                                                                              : reg::common::kTargetIdLenWchDmi7f;
      if (scheme_known && (lock_len - 1) / 2 != scheme_len) return rejected(kRejectMalformed);
      // boot_reset after the lock (optional, 0 when absent): 0 / 1, and 1 only on an at-boot slot (§1.1)
      const size_t after_lock = kSlotFixed + name_length + 1u + lock_len;
      // (boot_reset 2+ is malformed as probe.config §1.1 states it now)
      if (len > after_lock && (v[after_lock] > cfg::kSlotBootResetRetryWithReset ||
                               (v[after_lock] == cfg::kSlotBootResetRetryWithReset && attach == cfg::kSlotAttachHost)))
        return rejected(kRejectMalformed);
      if (undefined) return unsupportedTag(out, capacity, raw);
      // a scheme these wires do not read, defined (targetsel) or not
      if (lock_len && lock[0] != reg::common::kTargetIdSchemeWchDmi7f) return unsupportedTag(out, capacity, raw);
      if (!endpoint_.interfaceAt(wire_fn)) return rejected(kRejectUnknownFunction);
      int place = -1;
      for (size_t k = 0; k < place_count_; ++k) if (places_[k].wire_fn == wire_fn) place = static_cast<int>(k);
      if (place < 0) return unsupportedTag(out, capacity, raw);                                  // a wire without a target id scheme (swd), or not a wire
      if (!pairAllowed(*places_[place].port, swdio, swclk)) return unsupportedTag(out, capacity, raw);   // not a pair that wire allows
      DmiPhy &phy = places_[place].port->dm.phy();
      if (idle && !phy.canIdleClockLow()) return unsupportedTag(out, capacity, raw);   // idle_clock low: rvswd's only
      if (!phy.keepsMaxHz(max_hz) || (max_hz && phy.minClockHz() && max_hz < phy.minClockHz())) return unsupportedTag(out, capacity, raw);
      if (mechanism != reg::target_console::kMechanismNone && mechanism > reg::target_console::kMechanismDmseq)
        return unsupportedTag(out, capacity, raw);
      return completed();
    }
    case cfg::kTlvItemBind: {
      if (len < 4 || v[3] < 1) return rejected(kRejectMalformed);
      if (v[3] > Binds::kMaxStreams) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
      for (uint8_t k = 0; k < v[3]; ++k)   // the form first: every element there (malformed before unsupported)
        if (!bindStreamAt(v, len, k)) return rejected(kRejectMalformed);
      for (uint8_t k = 0; k < v[3]; ++k) {
        const size_t at = bindStreamAt(v, len, k);
        // a stream kind a later revision may define: unsupported with the item's tag (C-02)
        if (v[at] != Binds::kSlotConsole && v[at] != Binds::kFixtureUart) return unsupportedTag(out, capacity, raw);
        if (v[at] == Binds::kFixtureUart) {
          const uint16_t fn = getU16(v + at + 1);
          if (!endpoint_.interfaceAt(fn)) return rejected(kRejectUnknownFunction);
          bool is_uart = false;
          for (size_t i = 0; i < uart_count_; ++i) is_uart |= uarts_[i].fn == fn;
          if (!is_uart) return unsupportedTag(out, capacity, raw);
        }
      }
      if (v[1] > Binds::kMixed) return unsupportedTag(out, capacity, raw);
      if (v[1] == Binds::kManual && v[2] >= v[3]) return rejected(kRejectMalformed);
      if (v[0] >= Binds::kMaxPorts || !endpoint_.isSerialPort(v[0])) return unsupportedTag(out, capacity, raw);
      return completed();
    }
    case cfg::kTlvItemUart: {
      if (len < 7) return rejected(kRejectMalformed);
      const uint16_t fn = getU16(v);
      if (!endpoint_.interfaceAt(fn)) return rejected(kRejectUnknownFunction);
      bool is_uart = false;
      for (size_t i = 0; i < uart_count_; ++i) is_uart |= uarts_[i].fn == fn;
      // an unused value or reserved bit of format is unsupported, as an unrealisable baud (probe.config §2's table)
      if (!is_uart || !FixtureUart::baudWithinReach(getU32(v + 2)) || !FixtureUart::formatDefined(v[6]))
        return unsupportedTag(out, capacity, raw);
      return completed();
    }
    case cfg::kTlvItemDisable:
      if (len < 2) return rejected(kRejectMalformed);
      if (!pins_ || !pins_->allowed(getU16(v))) return unsupportedTag(out, capacity, raw);   // not declared: as idle
      return completed();
    default:
      return unsupportedTag(out, capacity, raw);   // an item this probe does not take (describe items)
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
        uint8_t level = PinTable::kDriveDefault;   // applied only where gpio declares drive_levels (the default otherwise)
        const DriveLevels levels = driveLevels();
        if (vlen >= 6 && levels.count && !PinTable::driveLevelOf(levels, v[3], getU16(v + 4), level))
          level = PinTable::kDriveDefault;
        d.idle_drive[getU16(v)] = level;
        break;
      }
      case cfg::kTlvItemSlot: {
        Slot &s = d.slots[v[0]];
        const uint16_t wire_fn = getU16(v + 1);
        for (size_t k = 0; k < place_count_; ++k) if (places_[k].wire_fn == wire_fn) s.place = static_cast<uint8_t>(k);
        s.set = true;
        s.swdio = getU16(v + 3);
        s.swclk = getU16(v + 5);
        s.attach = v[7];
        s.retry_ms = getU32(v + 8);
        s.max_hz = getU32(v + 12);
        s.idle_low = v[16] != 0;
        s.mechanism = v[17];
        s.name_length = v[18];
        memcpy(s.name, v + kSlotFixed, s.name_length);
        s.name[s.name_length] = 0;
        const uint8_t lock_len = v[kSlotFixed + s.name_length];
        const uint8_t *lock = v + kSlotFixed + s.name_length + 1;
        s.lock_scheme = lock_len ? lock[0] : 0;
        s.lock_length = static_cast<uint8_t>(lock_len ? (lock_len - 1) / 2 : 0);
        memcpy(s.mask, lock + 1, s.lock_length);
        memcpy(s.value, lock + 1 + s.lock_length, s.lock_length);
        const size_t after_lock = kSlotFixed + s.name_length + 1u + lock_len;
        s.boot_reset = vlen > after_lock && v[after_lock] == cfg::kSlotBootResetRetryWithReset;
        break;
      }
      case cfg::kTlvItemBind: {
        Binds::Spec &b = d.binds[v[0]];
        b.set = true;
        b.mode = v[1];
        b.selected = v[1] == Binds::kManual ? v[2] : 0;
        b.count = v[3];
        for (uint8_t k = 0; k < b.count; ++k) {
          const size_t s = bindStreamAt(v, vlen, k);
          b.sources[k].kind = v[s];
          b.sources[k].id = getU16(v + s + 1);
        }
        break;
      }
      case cfg::kTlvItemUart:
        for (size_t i = 0; i < uart_count_; ++i)
          if (uarts_[i].fn == getU16(v)) d.uarts[i] = {true, getU32(v + 2), v[6]};
        break;
      case cfg::kTlvItemDisable:
        if (getU16(v) < PinTable::kChannels) d.disabled |= uint64_t{1} << getU16(v);
        break;
      default:
        break;
    }
  }
  // idle and disable for one channel contradict (probe.config §1)
  for (uint8_t c = 0; c < PinTable::kChannels; ++c)
    if (((d.disabled >> c) & 1) && d.idle[c] != PinTable::kIdleUnset) return rejected(kRejectMalformed);
  // the whole: no two slots on one place with the same pair, names unique, at most one at-boot slot a wire
  // (max_connections 1), every bind's streams there (a slot with a console)
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
    Binds::Spec &b = d.binds[port];
    for (uint8_t k = 0; b.set && k < b.count; ++k) {
      if (b.sources[k].kind == Binds::kSlotConsole) {
        const uint16_t id = b.sources[k].id;
        if (id >= kMaxSlots || !d.slots[id].set || d.slots[id].mechanism == reg::target_console::kMechanismNone)
          return rejected(kRejectMalformed);   // a slot that is not there, or has no console: the settings disagree
      }
      if (!sourceFor(d.slots, b.sources[k].kind, b.sources[k].id, b.sources[k])) return rejected(kRejectMalformed);
    }
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
// nothing), then made current: idle states, slots (a replaced or removed one lets go of its connection and console,
// probe.config §1.1), binds, the UARTs' items.
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
    bool kept[256] = {};   // fns whose plan the settings put in
    {
      RoleAssignment mine[Endpoint::kMaxRoles];
      const size_t n = endpoint_.plan(mine, Endpoint::kMaxRoles, true);
      for (size_t r = 0; r < n; ++r) kept[mine[r].function & 0xff] = true;
    }
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
        return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, c, now[r].function,
                           kept[now[r].function & 0xff] ? reg::core::kHolderKindSettingsPlan : reg::core::kHolderKindPlan);
      }
      if (!replaced && pins_->owner(c)) return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, c);
      for (uint8_t i = 0; i < kMaxSlots; ++i)
        if (slots_[i].set && d.slots[i].set && memcmp(&slots_[i], &d.slots[i], sizeof(Slot)) == 0 &&
            (slots_[i].swdio == c || slots_[i].swclk == c))
          return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, c, 0xFFFF, reg::core::kHolderKindSlot);
    }
    auto off = [&](uint16_t c) { return c < PinTable::kChannels && ((d.disabled >> c) & 1); };
    for (size_t r = 0; r < d.role_count; ++r)
      if (off(d.roles[r].channel))
        return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, d.roles[r].channel, 0xFFFF,
                                           reg::core::kHolderKindDisabled);
    for (uint8_t i = 0; i < kMaxSlots; ++i) {
      if (!d.slots[i].set) continue;
      if (off(d.slots[i].swdio)) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, d.slots[i].swdio, 0xFFFF,
                                           reg::core::kHolderKindDisabled);
      if (off(d.slots[i].swclk)) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings, d.slots[i].swclk, 0xFFFF,
                                           reg::core::kHolderKindDisabled);
    }
  }
  // idle before the plans (probe.config §2: idle, plan, uart, the at-boot attach): a free channel takes its new idle
  // state now - an output idle drives before a gpio plan takes the channel, which keeps that level until its first set
  // (fixture §1) - and a channel the replaced plans release goes to the new idle. A channel disabled by this change only
  // keeps the mode (never touched); one enabled again gets it from setDisabled below. A refused plan puts the old back.
  // An idle's strength goes with its level (probe.config §1): a change of either applies both.
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
    if (reason == kRejectUnavailable) return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse);
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
    runs_[i].due = now.set && now.attach == cfg::kSlotAttachAtBoot;   // an at-boot slot set: attach now
  }
  for (uint8_t port = 0; port < Binds::kMaxPorts; ++port) {
    const Binds::Spec &was = binds_.spec(port), &now = d.binds[port];
    bool same = was.set == now.set && was.mode == now.mode && was.selected == now.selected && was.count == now.count;
    for (uint8_t k = 0; same && k < now.count; ++k)
      same = was.sources[k].kind == now.sources[k].kind && was.sources[k].id == now.sources[k].id;
    if (!same) binds_.set(port, now);
  }
  for (size_t i = 0; i < uart_count_; ++i) {
    if (d.uarts[i].set) uarts_[i].uart->setItem(d.uarts[i].baud, d.uarts[i].format);
    else uarts_[i].uart->clearItem();
  }
  poll();
  return completed();
}

// set: the items given replace the keys they carry (a fn's plan items replace that fn's whole plan); the rest stay.
Result ProbeConfig::set(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  static uint8_t candidate[kMaxItems];
  memcpy(candidate, items_, items_length_);
  size_t clen = items_length_;
  uint16_t plan_fns[16];
  size_t plan_fn_count = 0;
  uint8_t plan_raw = cfg::kTlvItemPlan;   // a plan item's tag as received, for a plan the interface cannot take
  // first pass: every item's shape and what this probe has; the same key twice is malformed
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  while (at < length) {
    size_t next = 0;
    if (!tlvAt(payload, length, at, tag, v, vlen, next)) return rejected(kRejectMalformed);
    const uint8_t raw = tag;
    tag &= ~kTagCritical;
    if (tag == kTagValue || tag == kTagIgnored || tag == kTagInvalid) return rejected(kRejectMalformed);
    // an item this probe does not declare (describe items): unsupported with its tag as received (§1, core §4.3)
    if (!keyLength(tag) || !declares(tag)) return unsupportedTag(out, capacity, raw);
    const Result r = checkItem(raw, v, vlen, out, capacity);
    if (refused(r)) return r;
    const size_t klen = keyLength(tag);
    size_t at2 = 0, vlen2 = 0;
    uint8_t tag2 = 0;
    const uint8_t *v2 = nullptr;
    while (at2 < at) {   // the items before this one: the same key twice?
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
  // second pass: the keys replaced come out (a plan's fn: its whole plan), the items go in
  for (size_t k = 0; k < plan_fn_count; ++k) {
    uint8_t fn[2];
    putU16(fn, plan_fns[k]);
    removeItems(candidate, clen, cfg::kTlvItemPlan, fn, 2);
  }
  at = 0;
  while (nextItem(payload, length, at, tag, v, vlen)) {
    tag &= ~kTagCritical;
    if (tag != cfg::kTlvItemPlan) removeItems(candidate, clen, tag, v, keyLength(tag));
    if (!insertItem(candidate, clen, sizeof candidate, tag, v, vlen)) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
  }
  return commit(candidate, clen, plan_fns, plan_fn_count, out, capacity, plan_raw);
}

// unset: n(u8), n x (len(u8), tag(u8), key). A key that is not there does nothing; the rest is as set.
Result ProbeConfig::unset(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  if (length < 1) return rejected(kRejectMalformed);
  const uint8_t n = payload[0];
  size_t at = 1;
  for (uint8_t i = 0; i < n; ++i) {   // the shape first
    if (at >= length || at + 1u + payload[at] > length || payload[at] < 1) return rejected(kRejectMalformed);
    // the tag is the item's tag itself (no critical bit here): one this probe does not declare (describe items) is
    // unsupported with the tag as received (§2, core §4.3)
    const uint8_t tag = payload[at + 1];
    const size_t klen = tag == cfg::kTlvItemPlan ? 2 : keyLength(tag);
    if (!klen || !declares(tag)) return unsupportedTag(out, capacity, tag);
    if (payload[at] < 1u + klen) return rejected(kRejectMalformed);
    at += 1u + payload[at];
  }
  Tail tail;
  const Result parsed = tail.parse(payload + at, length - at, out, capacity);
  if (refused(parsed)) return parsed;
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
    at += 1u + payload[at];
  }
  const Result r = commit(candidate, clen, plan_fns, plan_fn_count, out, capacity);
  if (refused(r)) return r;
  if (capacity < 4) return failed();
  putU32(out, hash());
  return tail.finish(completed(4), out, capacity);
}

// ---- the slots -----------------------------------------------------------------------------------------------------

bool ProbeConfig::bound(uint8_t slot) const {
  for (uint8_t port = 0; port < Binds::kMaxPorts; ++port) {
    const Binds::Spec &b = binds_.spec(port);
    for (uint8_t k = 0; b.set && k < b.count; ++k)
      if (b.sources[k].kind == Binds::kSlotConsole && b.sources[k].id == slot) return true;
  }
  return false;
}

int ProbeConfig::lockMatches(const Slot &s, bool has_tid, uint32_t tid) const {
  if (!s.lock_scheme) return 1;
  if (!has_tid) return -1;
  if (s.lock_scheme != reg::common::kTargetIdSchemeWchDmi7f || s.lock_length != 4) return 0;
  uint8_t id[4];
  putU32(id, tid);
  for (uint8_t i = 0; i < 4; ++i) if ((id[i] & s.mask[i]) != s.value[i]) return 0;
  return 1;
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

// One slot (probe.config §3): a bound slot rides any connection on its place when the lock matches, with its console
// open; an at-boot slot attaches by itself (without stopping the hart) at boot, when set, and every retry_ms while the
// target is not there, and - connected - reads DMSTATUS every retry_ms to see that it still answers (§3.1: the line lost
// closes the connection and the retries start).
void ProbeConfig::runSlot(uint8_t i) {
  const Slot &s = slots_[i];
  SlotRun &r = runs_[i];
  const Place &p = places_[s.place];
  DebugPort &port = *p.port;
  const bool at_boot = s.attach == cfg::kSlotAttachAtBoot;
  if (!port.connected && at_boot) {
    const bool retry = s.retry_ms && r.tried && static_cast<uint32_t>(millis() - r.last_try_ms) >= s.retry_ms;
    if (r.due || retry) {
      r.due = false;
      r.tried = true;
      r.last_try_ms = millis();
      r.last_try_ns = nowNs();
      uint32_t status = 0;
      bool no_answer = false;
      // the link to the slot's pair first (host-chosen pins; a fixed pair is always there), unless its pins are held;
      // the wire not answering (status line) may be tried once more with the reset line (§3.1)
      bool up = usePair(port, s.swdio, s.swclk) &&
                attachRunning(port, DebugPort::kUserSlot, status, s.max_hz, s.idle_low, nullptr, &no_answer);
      if (!up && no_answer) up = retryWithReset(i, status);
      if (up) {
        r.mismatch = lockMatches(s, port.has_tid, port.tid) != 1;
        r.mismatch_has_tid = port.has_tid;
        r.mismatch_tid = port.tid;
        if (r.mismatch) releaseConnection(port, DebugPort::kUserSlot, false);   // found, not the chip: let go of it
      }
      r.last_check_ms = millis();
    }
  }
  if (!port.connected || !onPair(s)) return;   // no link, or the link is on another pair (another slot, the host)
  if (at_boot && s.retry_ms && static_cast<uint32_t>(millis() - r.last_check_ms) >= s.retry_ms) {   // liveness
    r.last_check_ms = millis();
    if (!checkConnection(port)) { r.last_try_ms = millis(); return; }   // gone: the retries begin after retry_ms
  }
  if (lockMatches(s, port.has_tid, port.tid) == 1 && (at_boot || bound(i))) {
    port.users |= DebugPort::kUserSlot;
    r.mismatch = false;
    if (bound(i) && s.mechanism != reg::target_console::kMechanismNone && !p.console->isOpen()) p.console->bindOpen(s.mechanism);
    if (!bound(i) && (p.console->users() & TargetConsoleStream::kUserSlot)) p.console->bindClose();
  } else if (port.users & DebugPort::kUserSlot) {
    p.console->bindClose();
    releaseConnection(port, DebugPort::kUserSlot, false);
  }
}

// The retry with reset (§3.1) after an automatic attach of slot i the wire did not answer: a boot_reset slot, no
// session has taken the lock since boot, not done for this slot in this boot, and its nrst line (§1.3) one the wire's
// attach could pull with its reset TLV (role 3 of role_channels, not disabled, nothing holding it). The same attach
// (method 0), the line held slot_retry_reset_hold_ms first. true: attached.
bool ProbeConfig::retryWithReset(uint8_t i, uint32_t &dmstatus) {
  const Slot &s = slots_[i];
  BootReset &b = boot_resets_[i];
  if (!s.boot_reset || b.done || endpoint_.lockEverTaken()) return false;
  DebugPort &port = *places_[s.place].port;
  const uint16_t nrst = lineOf(i, "nrst");
  if (nrst > 63 || !((port.reset_allowed >> nrst) & 1)) return false;   // none, or not a reset channel of this wire
  if (pins_ && (pins_->disabled(nrst) || pins_->owner(nrst))) return false;   // disabled, held by a plan / connection
  b.done = true;
  AttachReset reset{nrst, static_cast<uint16_t>(reg::kSlotRetryResetHoldMs), kNeverNs};
  SlotRun &r = runs_[i];
  r.last_try_ms = millis();
  r.last_try_ns = nowNs();
  const bool up = attachRunning(port, DebugPort::kUserSlot, dmstatus, s.max_hz, s.idle_low, &reset);
  b.at_ns = reset.held_at_ns;
  return up;
}

uint16_t ProbeConfig::lineOf(uint8_t slot, const char *line) const {
  const char *name = slot < kMaxSlots && slots_[slot].set ? slots_[slot].name : nullptr;
  if (slot != 0xff && !name) return 0xffff;
  return findLine(items_, items_length_, name, line);
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
    if (tag == cfg::kTlvItemBind && vlen >= 4)
      for (uint8_t k = 0; k < v[3]; ++k)
        if (const size_t s = bindStreamAt(v, vlen, k))
          if (v[s] == Binds::kFixtureUart) note(getU16(v + s + 1));
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
    putU16(out + used + 2, it ? it->instance() : 0);
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
  if (!storeRead(blob, sizeof blob, length)) return;
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
      if (it->instance() == instance && it->revision() == revision && strlen(it->name()) == name_len &&
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
    if (tag == cfg::kTlvItemBind && vlen >= 4)
      for (uint8_t k = 0; k < v[3]; ++k)
        if (const size_t s = bindStreamAt(v, vlen, k))
          if (v[s] == Binds::kFixtureUart) putU16(v + s + 1, map(getU16(v + s + 1)));
  }
  // the renumbered items may be out of the canonical order (fns moved): sorted into a fresh store, then set as a whole
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
  const Result r = commit(candidate, clen, plan_fns, plan_fn_count, scratch, sizeof scratch);
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
  w.u32(cfg::kTlvDescribeStorage, kMaxItems);
  static const uint8_t kItems[] = {cfg::kTlvItemPlan, cfg::kTlvItemLabel, cfg::kTlvItemSlot, cfg::kTlvItemBind,
                                   cfg::kTlvItemUart, cfg::kTlvItemIdle, cfg::kTlvItemDisable};
  w.put(cfg::kTlvDescribeItems, kItems, pins_ ? sizeof kItems : sizeof kItems - 2);   // idle and disable need the pins
  w.u8(cfg::kTlvDescribeSlotsMax, place_count_ ? static_cast<uint8_t>(kMaxSlots) : 0);
  w.u32(cfg::kTlvDescribeBindModes, (1u << cfg::kBindModeLastReset) | (1u << cfg::kBindModeManual) | (1u << cfg::kBindModeMixed));
  return w.ok() ? w.length() : 0;
}

// slot_state (probe.config §3.3): slot(u8) state(u8) connection(u16) last_try_at_ns(u64) tid_scheme(u8) tid_len(u8) tid
// reset_at_ns(u64: the retry with reset's pull, all ones when not done)
size_t ProbeConfig::slotState(uint8_t i, uint8_t *out) const {
  const Slot &s = slots_[i];
  const SlotRun &r = runs_[i];
  const DebugPort &port = *places_[s.place].port;
  out[0] = i;
  bool has_tid = false;
  uint32_t tid = 0;
  if (port.connected && onPair(s)) {
    has_tid = port.has_tid;
    tid = port.tid;
    const int m = lockMatches(s, has_tid, tid);
    out[1] = m == 1 ? cfg::kSlotStateConnected : (m < 0 ? cfg::kSlotStateNoTargetId : cfg::kSlotStateLockMismatch);
    putU16(out + 2, port.number);
  } else {
    has_tid = r.mismatch && r.mismatch_has_tid;
    tid = r.mismatch_tid;
    out[1] = r.mismatch ? (r.mismatch_has_tid ? cfg::kSlotStateLockMismatch : cfg::kSlotStateNoTargetId) : cfg::kSlotStateAbsent;
    putU16(out + 2, 0);
  }
  putU64(out + 4, r.tried ? r.last_try_ns : kNeverNs);
  out[12] = has_tid ? reg::common::kTargetIdSchemeWchDmi7f : 0;
  out[13] = has_tid ? reg::common::kTargetIdLenWchDmi7f : 0;
  if (has_tid) putU32(out + 14, tid);
  const size_t at = has_tid ? 18 : 14;
  putU64(out + at, boot_resets_[i].at_ns);
  return at + 8;
}

// state(first_slot u8, first_bind u8) -> more(u8) storage_state(u8) storage_hash(u32) unreadable_reason(u8)
//   n_slots(u8) n_slots x (len, slot_state) n_binds(u8) n_binds x (len, bind_state) [TLV]
Result ProbeConfig::state(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
  if (refused(parsed)) return parsed;
  if (capacity < 9) return failed();
  const size_t room = tail.anyIgnored() && capacity > 9 + Tail::kMaxIgnored ? capacity - 2 - Tail::kMaxIgnored : capacity;
  out[1] = storage_state_;
  putU32(out + 2, storage_state_ == cfg::kStorageStateApplied ? saved_hash_ : 0);
  out[6] = storage_state_ == cfg::kStorageStateUnreadable ? unreadable_ : 0;
  bool more = false;
  size_t used = 8;
  uint8_t n = 0, index = 0;
  for (uint8_t i = 0; i < kMaxSlots; ++i) {
    if (!slots_[i].set) continue;
    if (index++ < payload[0]) continue;
    if (used + 1 + 26 + 1 > room) { more = true; break; }   // the longest slot_state: 26
    out[used] = static_cast<uint8_t>(slotState(i, out + used + 1));
    used += 1u + out[used];
    ++n;
  }
  out[7] = n;
  const size_t binds_at = used++;
  n = index = 0;
  for (uint8_t port = 0; port < Binds::kMaxPorts && !more; ++port) {
    const Binds::Spec &b = binds_.spec(port);
    if (!b.set) continue;
    if (index++ < payload[1]) continue;
    if (used + 5 > room) { more = true; break; }
    const uint8_t flow = endpoint_.held(port) ? cfg::kBindFlowHeld : binds_.streaming(port) ? cfg::kBindFlowStreaming : cfg::kBindFlowIdle;
    out[used] = 4;
    out[used + 1] = port;
    out[used + 2] = b.mode;
    out[used + 3] = b.mode == Binds::kMixed ? uint8_t{0xff} : binds_.selected(port);
    out[used + 4] = flow;
    used += 5;
    ++n;
  }
  out[binds_at] = n;
  out[0] = more ? 1 : 0;
  return tail.finish(completed(used), out, capacity);
}

Result ProbeConfig::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (op) {
    case cfg::kOpGet: {   // first(u16) -> more(u8) hash(u32) items from the first-th on (the canonical order)
      // no TLV in the request (core §7.3: the answer is a TLV list itself, ignored never in it): malformed
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
        const size_t item = next - at;
        if (index++ >= first) {
          if (put + item > room) { more = true; break; }
          memcpy(out + put, items_ + at, item);
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
      return tail.finish(completed(4), out, capacity);
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
      return tail.finish(completed(), out, capacity);
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

// How many label items name the line, the channel of the last one in `channel`.
size_t labelsNaming(const uint8_t *items, size_t length, const char *slot, const char *line, uint16_t &channel) {
  size_t at = 0, found = 0;
  while (at < length) {
    uint8_t tag = 0;
    const uint8_t *v = nullptr;
    size_t vlen = 0, next = 0;
    if (!tlvAt(items, length, at, tag, v, vlen, next)) break;
    at = next;
    if (tag != reg::probe_config::kTlvItemLabel || vlen < 2 || !labelNames(v + 2, vlen - 2, slot, line)) continue;
    channel = getU16(v);
    ++found;
  }
  return found;
}

}  // namespace

uint16_t findLine(const uint8_t *items, size_t length, const char *slot, const char *line) {
  uint16_t channel = 0xffff;
  if (slot) {
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
  return labelsNaming(items, length, nullptr, line, channel) == 1 ? channel : 0xffff;
}

}  // namespace oep
