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

enum : uint8_t {
  kLabel = cfg::kTlvItemLabel, kIdle = cfg::kTlvItemIdle, kDisable = cfg::kTlvItemDisable,
  kPass = cfg::kTlvItemPassthrough, kConsolePass = cfg::kTlvItemConsolePassthrough,
};
constexpr size_t kPassLength = 10, kConsolePassLength = 14;

// The saved blob: the items in get's order, then CRC-32 of them (u32): a blob whose CRC does not match, or that does not
// end on an item, is a container whose bounds or integrity cannot be confirmed (probe.config §3: unreadable, reason 1).
constexpr size_t kMaxBlob = ProbeConfig::kMaxItems + 4;
#if defined(ARDUINO_ARCH_ESP32)
// NVS writes a key whole (old or new after a power loss, probe.config §3); "items6" / "items5" are the forms before.
constexpr const char *kNvsNamespace = "oepcfg", *kNvsItems = "items7";
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
  const bool old = p.isKey("items6") || p.isKey("items5");
  p.end();
  return old;
}
bool storeWrite(const uint8_t *blob, size_t length) {   // length 0: nothing saved
  Preferences p;
  if (!p.begin(kNvsNamespace, false)) return false;
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
constexpr uint32_t kEepromMagic = 0x4f455037;   // "OEP7"; "OEP6" / "OEP5" are the forms before
constexpr size_t kEepromHeader = 6;             // magic(u32) length(u16)
bool storeOld() {
  EEPROM.begin(kEepromHeader + kMaxBlob);
  const uint32_t m = getU32(EEPROM.getConstDataPtr());
  return m == 0x4f455036 || m == 0x4f455035;
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

bool ProbeConfig::declares(uint8_t item) const {
  switch (item) {
    case kLabel: return true;
    case kIdle: case kDisable: return pins_ != nullptr;
    case kPass: return uart_count_ > 0;
    case kConsolePass: return place_count_ > 0;
    default: return false;
  }
}

bool ProbeConfig::serialPort(uint8_t port) const { return port < kMaxPorts && endpoint_.isSerialPort(port); }

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

// An item's key: its channel, a pass-through's port. false: not an item of these forms (its length is checked first).
bool ProbeConfig::keyOf(uint8_t item, const uint8_t *v, size_t len, uint16_t &key) {
  if ((item == kPass || item == kConsolePass) && len >= 1) { key = v[0]; return true; }
  if (len >= 2) { key = getU16(v); return true; }
  return false;
}

// One item by itself (probe.config §1): its form (another length is malformed), then what this probe takes (unsupported
// with the item's tag as received).
Result ProbeConfig::checkItem(uint8_t raw, const uint8_t *v, size_t len, uint8_t *out, size_t capacity) const {
  const uint8_t item = raw & ~kTagCritical;
  switch (item) {
    case kLabel:
      if (len < 3 || len > 2u + reg::kLimitLabelMaxBytes) return rejected(kRejectMalformed);
      break;
    case kIdle: if (len != 3) return rejected(kRejectMalformed); break;
    case kDisable: if (len != 2) return rejected(kRejectMalformed); break;
    case kPass:
      if (len != kPassLength) return rejected(kRejectMalformed);
      if (getU32(v + 5) == 0) return rejected(kRejectMalformed);   // baud 0 (fixture §2's rule)
      break;
    case kConsolePass:
      if (len != kConsolePassLength) return rejected(kRejectMalformed);
      if (getU32(v + 7) == 0) return rejected(kRejectMalformed);   // max_speed 0 (debug §1's rule)
      break;
    default: return unsupportedTag(out, capacity, raw);   // an item this probe does not know (§1)
  }
  if (!declares(item)) return unsupportedTag(out, capacity, raw);
  switch (item) {
    case kLabel: case kIdle: case kDisable: {
      const uint16_t ch = getU16(v);
      if (pins_ ? !pins_->allowed(ch) : ch >= PinTable::kChannels) return unsupportedTag(out, capacity, raw);
      if (item == kIdle) {
        const uint8_t mode = v[2];
        const bool output = mode == PinTable::kIdleOutputLow || mode == PinTable::kIdleOutputHigh;
        const bool pull = mode == PinTable::kIdlePullUp || mode == PinTable::kIdlePullDown;
        if (mode > PinTable::kIdleOutputHigh || (output && !pins_->canOutput(ch)) || (pull && !pins_->canPull(ch)))
          return unsupportedTag(out, capacity, raw);
      }
      return completed();
    }
    case kPass: {
      const int k = uartIndex(getU16(v + 1));
      const uint16_t rx = getU16(v + 3);
      if (!serialPort(v[0]) || k < 0 || !uarts_[k]->rxAllowed(rx) || !uarts_[k]->baudWithinReach(getU32(v + 5)) ||
          !FixtureUart::formatDefined(v[9]) || !uarts_[k]->formatDeclared(v[9]))
        return unsupportedTag(out, capacity, raw);
      return completed();
    }
    case kConsolePass: {
      const int k = placeIndex(getU16(v + 1), getU16(v + 11));
      if (!serialPort(v[0]) || k < 0) return unsupportedTag(out, capacity, raw);
      DebugPort &port = places_[k].wire->port();
      const uint32_t max_hz = getU32(v + 7);
      if (!pairAllowed(port, getU16(v + 3), getU16(v + 5)) ||
          (port.dm.phy().minClockHz() && max_hz < port.dm.phy().minClockHz()) ||
          !places_[k].console->offersMechanism(v[13]))
        return unsupportedTag(out, capacity, raw);
      return completed();
    }
    default: return unsupportedTag(out, capacity, raw);
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
    size_t a = 0, n = 0;
    uint8_t t = 0;
    const uint8_t *w = nullptr;
    while (nextItem(items, length, a, t, w, n))
      if (t == kIdle && getU16(w) == ch && (w[2] == PinTable::kIdleOutputLow || w[2] == PinTable::kIdleOutputHigh)) return true;
    return false;
  };
  auto isDisabled = [&](uint16_t ch) { return ch < 64 && ((disabled >> ch) & 1); };
  while (nextItem(items, length, at, tag, v, vlen)) {
    if (tag == kDisable) {
      const uint16_t ch = getU16(v);
      const bool now = pins_ && pins_->disabled(ch);
      if (!now && pins_ && pins_->owner(ch) && pins_->owner(ch) != 0xff)
        return unavailable(out, capacity, reg::core::kUnavailableCausePinInUse, ch);
    }
    if (tag == kIdle && isDisabled(getU16(v))) return heldBySettings(out, capacity, getU16(v));
    if (tag == kPass || tag == kConsolePass) {
      if (pass_port[v[0]]) return unavailable(out, capacity, reg::core::kUnavailableCauseHeldBySettings);
      pass_port[v[0]] = true;
      const uint16_t pins[2] = {getU16(v + 3), tag == kConsolePass ? getU16(v + 5) : static_cast<uint16_t>(0xffff)};
      for (uint16_t ch : pins)
        if (ch != 0xffff && (isDisabled(ch) || idleOutput(ch))) return heldBySettings(out, capacity, ch);
    }
  }
  return completed();
}

bool ProbeConfig::insert(uint8_t *store, size_t &length, size_t capacity, uint8_t item, const uint8_t *v, size_t len) {
  uint16_t key = 0;
  keyOf(item, v, len, key);
  remove(store, length, item, key);
  // the place: after every item of a smaller tag, or of this tag with a smaller key (get's order, §2)
  size_t at = 0, place = length, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *w = nullptr;
  while (at < length) {
    const size_t here = at;
    if (!nextItem(store, length, at, tag, w, vlen)) break;
    uint16_t k = 0;
    keyOf(tag, w, vlen, k);
    if (tag > item || (tag == item && k > key)) { place = here; break; }
  }
  if (length + kTlvHeader + len > capacity) return false;
  memmove(store + place + kTlvHeader + len, store + place, length - place);
  putTlvHeader(store + place, item, static_cast<uint16_t>(len));
  memcpy(store + place + kTlvHeader, v, len);
  length += kTlvHeader + len;
  return true;
}

void ProbeConfig::remove(uint8_t *store, size_t &length, uint8_t item, uint16_t key) {
  size_t at = 0, vlen = 0;
  uint8_t tag = 0;
  const uint8_t *v = nullptr;
  while (at < length) {
    const size_t here = at;
    if (!nextItem(store, length, at, tag, v, vlen)) return;
    uint16_t k = 0;
    if (tag == item && keyOf(tag, v, vlen, k) && k == key) {
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
// use takes its new idle when it is freed), and the pass-throughs from the next poll.
void ProbeConfig::adopt(const uint8_t *items, size_t length) {
  if (items != items_) memmove(items_, items, length);
  items_length_ = length;
  if (!pins_) return;
  const uint64_t disabled = disabledIn(items_, items_length_);
  pins_->setDisabled(disabled);
  endpoint_.setDisabled(disabled);
  for (uint16_t ch = 0; ch < PinTable::kChannels; ++ch) {
    if (!pins_->allowed(ch)) continue;
    uint8_t mode = PinTable::kIdleUnset;
    size_t at = 0, vlen = 0;
    uint8_t tag = 0;
    const uint8_t *v = nullptr;
    while (nextItem(items_, items_length_, at, tag, v, vlen)) if (tag == kIdle && getU16(v) == ch) mode = v[2];
    if (pins_->idle(ch) != mode) pins_->setIdle(ch, mode);   // applied now to a free channel only (PinTable)
  }
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
    if ((payload[at] & ~kTagCritical) == 0) return rejected(kRejectMalformed);   // item 0 (§1)
    at += kTlvHeader + len;
  }
  Tail tail;
  const Result parsed = tail.parse(payload + at, length - at, out, capacity);
  if (refused(parsed)) return parsed;
  // the items one by one, then a key twice (malformed, §2)
  Result unsupported = completed();
  for (size_t p = 1, i = 0; i < count; ++i) {
    const uint8_t raw = payload[p];
    const size_t len = getU16(payload + p + 1);
    const Result r = checkItem(raw, payload + p + kTlvHeader, len, out, capacity);
    if (r.resolution == kResolutionRejected && r.detail == kRejectMalformed) return r;
    if (refused(r) && !refused(unsupported)) unsupported = r;
    for (size_t q = 1, j = 0; j < i; ++j) {
      const size_t qlen = getU16(payload + q + 1);
      uint16_t a = 0, b = 0;
      if ((payload[q] & ~kTagCritical) == (raw & ~kTagCritical) && keyOf(raw & ~kTagCritical, payload + q + kTlvHeader, qlen, a) &&
          keyOf(raw & ~kTagCritical, payload + p + kTlvHeader, len, b) && a == b)
        return rejected(kRejectMalformed);
      q += kTlvHeader + qlen;
    }
    p += kTlvHeader + len;
  }
  if (refused(unsupported)) return unsupported;
  static uint8_t candidate[kMaxItems];
  size_t clen = items_length_;
  memcpy(candidate, items_, clen);
  for (size_t p = 1, i = 0; i < count; ++i) {
    const size_t len = getU16(payload + p + 1);
    if (!insert(candidate, clen, sizeof candidate, payload[p] & ~kTagCritical, payload + p + kTlvHeader, len))
      return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);   // more than this probe holds (§2)
    p += kTlvHeader + len;
  }
  return commit(candidate, clen, out, capacity);
}

Result ProbeConfig::unset(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  // count(u8) count x (tag u8, key u16) [TLV]: a passthrough's key (its port) in the u16 (§1.1)
  if (length < 1 || length < 1u + 3u * payload[0]) return rejected(kRejectMalformed);
  const uint8_t count = payload[0];
  Tail tail;
  const Result parsed = tail.parse(payload + 1 + 3 * count, length - 1 - 3 * count, out, capacity);
  if (refused(parsed)) return parsed;
  uint8_t undeclared = 0;
  for (uint8_t i = 0; i < count; ++i) {
    const uint8_t raw = payload[1 + 3 * i], item = raw & ~kTagCritical;
    if (!item) return rejected(kRejectMalformed);
    for (uint8_t j = 0; j < i; ++j)
      if ((payload[1 + 3 * j] & ~kTagCritical) == item && getU16(payload + 2 + 3 * j) == getU16(payload + 2 + 3 * i))
        return rejected(kRejectMalformed);   // a key twice
    if (!declares(item) && !undeclared) undeclared = raw;
  }
  if (undeclared) return unsupportedTag(out, capacity, undeclared);
  static uint8_t candidate[kMaxItems];
  size_t clen = items_length_;
  memcpy(candidate, items_, clen);
  for (uint8_t i = 0; i < count; ++i) remove(candidate, clen, payload[1 + 3 * i] & ~kTagCritical, getU16(payload + 2 + 3 * i));
  return commit(candidate, clen, out, capacity);
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
  while (at < items_length_) {
    const size_t here = at;
    if (!nextItem(items_, items_length_, at, tag, v, vlen)) break;
    if (index++ < first) continue;
    const size_t n = at - here;
    if (put + n > capacity || count == 255) { more = true; break; }
    memcpy(out + put, items_ + here, n);
    put += n;
    ++count;
  }
  out[0] = more ? 1 : 0;
  putU32(out + 1, generation_);
  out[5] = count;
  return completed(put);
}

size_t ProbeConfig::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  if (storage_) w.u32(cfg::kTlvDescribeStorage, kMaxItems);   // the items' bytes a save always holds (§4)
  uint8_t items[5], n = 0;
  for (uint8_t item : {kLabel, kIdle, kPass, kConsolePass, kDisable}) if (declares(item)) items[n++] = item;
  w.put(cfg::kTlvDescribeItems, items, n);
  return w.ok() ? w.length() : 0;
}

Result ProbeConfig::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case cfg::kOpGet: return get(payload, length, out, capacity);
    case cfg::kOpSet: return set(payload, length, out, capacity);
    case cfg::kOpUnset: return unset(payload, length, out, capacity);
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
        uint8_t blob[kMaxBlob];
        memcpy(blob, items_, items_length_);
        putU32(blob + items_length_, crc32Ieee(items_, items_length_));
        if (!storeWrite(blob, items_length_ + 4)) return failed(4);   // the state says what is kept (§3's table)
        memcpy(saved_, items_, items_length_);
        saved_length_ = items_length_;
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
      storage_state_ = cfg::kStorageStateNone;
      reason_ = 0;
      saved_generation_ = 0;
      return completed();
    }
    default: return rejected(kRejectUnknownOperation);
  }
}

// ---- storage and boot ------------------------------------------------------------------------------------------------

void ProbeConfig::load() {
  uint8_t blob[kMaxBlob];
  size_t length = 0;
  saved_length_ = 0;
  loaded_ = false;
  if (!storeRead(blob, sizeof blob, length)) {
    if (storeOld()) { storage_state_ = cfg::kStorageStateUnreadable; reason_ = cfg::kStorageUnreadableForm; }
    return;
  }
  bool whole = length >= 4 && getU32(blob + length - 4) == crc32Ieee(blob, length - 4);
  for (size_t at = 0, vlen = 0; whole && at < length - 4;) {
    uint8_t tag = 0;
    const uint8_t *v = nullptr;
    whole = nextItem(blob, length - 4, at, tag, v, vlen);
  }
  if (!whole) { storage_state_ = cfg::kStorageStateUnreadable; reason_ = cfg::kStorageUnreadableForm; return; }
  memcpy(saved_, blob, length - 4);
  saved_length_ = length - 4;
  loaded_ = true;
}

// At boot (probe.config §3): every saved item checked by itself against this firmware - disables first, then idles (a
// channel with a disable keeps no idle), labels and pass-throughs; one it cannot take is left out and the rest applied
// (storage_state 3, reason 2). A key saved twice applies neither copy.
void ProbeConfig::applySaved() {
  if (!loaded_) { adopt(items_, 0); return; }
  static uint8_t applied[kMaxItems];
  size_t alen = 0;
  bool bad = false;
  uint8_t scratch[16];
  const uint64_t disabled = disabledIn(saved_, saved_length_);
  for (uint8_t item : {kDisable, kIdle, kLabel, kPass, kConsolePass}) {
    size_t at = 0, vlen = 0;
    uint8_t tag = 0;
    const uint8_t *v = nullptr;
    while (nextItem(saved_, saved_length_, at, tag, v, vlen)) {
      if (tag != item) continue;
      uint16_t key = 0;
      bool twice = false;
      if (keyOf(tag, v, vlen, key)) {
        size_t a = 0, n = 0;
        uint8_t t = 0;
        const uint8_t *w = nullptr;
        int seen = 0;
        while (nextItem(saved_, saved_length_, a, t, w, n)) { uint16_t k = 0; seen += t == tag && keyOf(t, w, n, k) && k == key; }
        twice = seen > 1;
      }
      bool ok = !twice && !refused(checkItem(tag, v, vlen, scratch, sizeof scratch));
      if (ok && tag == kIdle && getU16(v) < 64 && ((disabled >> getU16(v)) & 1)) ok = false;   // disable wins
      if (ok && !insert(applied, alen, sizeof applied, tag, v, vlen)) ok = false;
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
    const uint8_t port = v[0];
    if (port >= kMaxPorts) continue;
    const bool session = endpoint_.held(port);   // a session's request came on the port: off for the session
    gPassContext[port] = {this, port};
    if (tag == kPass) {
      const int k = uartIndex(getU16(v + 1));
      if (k < 0) continue;
      FixtureUart &uart = *uarts_[k];
      const uint16_t rx = getU16(v + 3);
      const bool on = !session && !uart.planned() && pins_ && pins_->free(rx);
      wanted_uart[port] = on;
      if (on && pass_uart_[port] < 0 && uart.passStart(rx, getU32(v + 5), v[9], &ProbeConfig::passIn, &gPassContext[port]))
        pass_uart_[port] = static_cast<int8_t>(k);
      if (pass_uart_[port] >= 0) {
        if (on) uart.poll();
        else { uart.passStop(); pass_uart_[port] = -1; binds_.drop(port); }
      }
    } else {
      const int k = placeIndex(getU16(v + 1), getU16(v + 11));
      if (k < 0) continue;
      DebugPort &dp = places_[k].wire->port();
      TargetConsoleStream &console = *places_[k].console;
      const uint16_t swdio = getU16(v + 3), swclk = getU16(v + 5);
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
        if (residentAttach(dp, swdio, swclk, getU32(v + 7)) &&
            console.passStart(v[13], &ProbeConfig::passIn, &gPassContext[port]))
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
  if (slot) {
    const size_t n = labelsNaming(items, length, slot, line, channel);
    if (n) return n == 1 ? channel : 0xffff;
  }
  const size_t n = labelsNaming(items, length, nullptr, line, channel);
  if (n) return n == 1 ? channel : 0xffff;
  return labelsNaming(firmware, firmware_length, nullptr, line, channel, reg::core::kTlvDescribeLabel) == 1 ? channel : 0xffff;
}

}  // namespace oep
