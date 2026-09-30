// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepConfig.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <Preferences.h>
#include <stdio.h>

#include "OepEndpoint.h"

namespace oep {

namespace cfg = reg::probe_config;

namespace {
constexpr const char *kNvsNamespace = "oepcfg", *kNvsItems = "items2", *kNvsList = "list1";   // items: the 2026-09-30 slot layout
constexpr size_t kSlotFixed = 17;   // slot wire_fn swdio swclk attach retry_s max_speed idle_clock mechanism name_len
bool nameChar(char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_'; }
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

size_t ProbeConfig::nameOf(void *self, uint8_t kind, uint16_t id, char *out, size_t room) {
  const ProbeConfig &c = *static_cast<ProbeConfig *>(self);
  if (kind == Binds::kSlotConsole && id < kMaxSlots && c.slots_[id].set) {
    const size_t n = c.slots_[id].name_length < room ? c.slots_[id].name_length : room;
    memcpy(out, c.slots_[id].name, n);
    return n;
  }
  const int n = snprintf(out, room, "uart%u", static_cast<unsigned>(id));   // no label items here (probe.config §1.2)
  return n < 0 ? 0 : (static_cast<size_t>(n) < room ? static_cast<size_t>(n) : room);
}

// ---- the items -----------------------------------------------------------------------------------------------------

size_t ProbeConfig::canonical(uint8_t *out, size_t capacity) const {
  TlvWriter w(out, capacity);
  RoleAssignment roles[Endpoint::kMaxRoles];
  const size_t n = endpoint_.plan(roles, Endpoint::kMaxRoles, true);   // the settings' plans only
  for (uint32_t key = 0; key <= 0xffffff; ) {                         // ascending (fn, role)
    uint32_t next = 0xffffffff;
    for (size_t i = 0; i < n; ++i) {
      const uint32_t k = uint32_t(roles[i].function) << 8 | roles[i].role;
      if (k == key) {
        uint8_t v[5];
        putU16(v, roles[i].function);
        v[2] = roles[i].role;
        putU16(v + 3, roles[i].channel);
        w.put(cfg::kTlvItemPlan, v, 5);
      } else if (k > key && k < next) {
        next = k;
      }
    }
    if (next == 0xffffffff) break;
    key = next;
  }
  for (uint16_t c = 0; c < PinTable::kChannels; ++c) {
    if (idle_[c] == PinTable::kIdleUnset) continue;
    uint8_t v[3];
    putU16(v, c);
    v[2] = idle_[c];
    w.put(cfg::kTlvItemIdle, v, 3);
  }
  for (uint8_t i = 0; i < kMaxSlots; ++i) {   // ascending slot
    const Slot &s = slots_[i];
    if (!s.set) continue;
    const Place &p = places_[s.place];
    uint8_t v[kSlotFixed + kMaxName + 1 + 2 * kMaxLock];
    v[0] = i;
    putU16(v + 1, p.wire_fn);
    putU16(v + 3, s.swdio);
    putU16(v + 5, s.swclk);
    v[7] = s.attach;
    putU16(v + 8, s.retry_s);
    putU32(v + 10, s.max_hz);
    v[14] = s.idle_low ? reg::wire_rvswd::kIdleClockLow : reg::wire_rvswd::kIdleClockHigh;
    v[15] = s.mechanism;
    v[16] = s.name_length;
    memcpy(v + kSlotFixed, s.name, s.name_length);
    size_t at = kSlotFixed + s.name_length;
    v[at++] = s.lock_scheme;
    if (s.lock_scheme) {
      memcpy(v + at, s.mask, s.lock_length);
      memcpy(v + at + s.lock_length, s.value, s.lock_length);
      at += 2u * s.lock_length;
    }
    w.put(cfg::kTlvItemSlot, v, at);
  }
  for (uint8_t port = 0; port < Binds::kMaxPorts; ++port) {   // ascending port
    const Binds::Spec &b = binds_.spec(port);
    if (!b.set) continue;
    uint8_t v[4 + 3 * Binds::kMaxStreams] = {port, b.mode, b.mode == Binds::kManual ? b.selected : uint8_t{0}, b.count};
    for (uint8_t i = 0; i < b.count; ++i) {
      v[4 + 3 * i] = b.sources[i].kind;
      putU16(v + 5 + 3 * i, b.sources[i].id);
    }
    w.put(cfg::kTlvItemBind, v, 4u + 3u * b.count);
  }
  return w.ok() ? w.length() : 0;
}

uint32_t ProbeConfig::hash() const {
  uint8_t buf[kMaxSaved];
  return crc32Ieee(buf, canonical(buf, sizeof buf));
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

// Checks every item first, then changes: plans (the endpoint's, the fns named, all or nothing), idle states, slots
// and binds. A refusal leaves everything as it was. Keys a set does not carry keep their items (probe.config §2).
Result ProbeConfig::apply(const uint8_t *items, size_t length) {
  RoleAssignment roles[Endpoint::kMaxRoles];
  size_t role_count = 0;
  uint16_t plan_fns[16];
  size_t plan_fn_count = 0;
  Slot slots[kMaxSlots];
  for (size_t i = 0; i < kMaxSlots; ++i) slots[i] = slots_[i];
  bool slot_touched[kMaxSlots] = {};
  struct BindItem { bool touched = false; bool set = false; uint8_t mode = 0, selected = 0, count = 0;
                    uint8_t kinds[Binds::kMaxStreams] = {}; uint16_t ids[Binds::kMaxStreams] = {}; };
  BindItem binds[Binds::kMaxPorts];
  uint8_t idle[PinTable::kChannels];
  memcpy(idle, idle_, sizeof idle);
  uint64_t idle_touched = 0;
  bool has_idle = false;
  for (size_t at = 0; at < length;) {
    if (length - at < 2 || length - at - 2 < items[at + 1]) return rejected(kRejectMalformed);
    const uint8_t tag = items[at] & 0x7f, len = items[at + 1];   // kept and hashed without the critical bit (§2)
    const uint8_t *v = items + at + 2;
    at += 2u + len;
    if (tag == cfg::kTlvItemPlan) {
      if (len != 2 && len != 5) return rejected(kRejectMalformed);
      const uint16_t fn = getU16(v);
      bool listed = false;
      for (size_t k = 0; k < plan_fn_count; ++k) listed |= plan_fns[k] == fn;
      if (!listed) {
        if (plan_fn_count >= 16) return rejected(kRejectMalformed);
        plan_fns[plan_fn_count++] = fn;
      }
      if (len == 2) continue;   // fn alone: that fn's plan goes
      for (size_t r = 0; r < role_count; ++r)
        if (roles[r].function == fn && roles[r].role == v[2]) return rejected(kRejectMalformed);   // a key twice
      if (role_count >= Endpoint::kMaxRoles) return rejected(kRejectUnavailable);   // over plan_roles (core §8)
      roles[role_count++] = {fn, v[2], getU16(v + 3)};
    } else if (tag == cfg::kTlvItemIdle) {
      if (!pins_) return rejected(kRejectUnsupported);
      if (len != 2 && len != 3) return rejected(kRejectMalformed);
      const uint16_t c = getU16(v);
      if (c >= PinTable::kChannels || !pins_->allowed(c) || ((idle_touched >> c) & 1)) return rejected(kRejectMalformed);
      if (len == 3 && v[2] > PinTable::kIdlePullDown) return rejected(kRejectMalformed);
      idle_touched |= uint64_t{1} << c;
      idle[c] = len == 3 ? v[2] : PinTable::kIdleUnset;
      has_idle = true;
    } else if (tag == cfg::kTlvItemSlot) {
      if (len < 1) return rejected(kRejectMalformed);
      const uint8_t n = v[0];
      if (n >= kMaxSlots || slot_touched[n]) return rejected(kRejectMalformed);   // slots_max
      slot_touched[n] = true;
      Slot &s = slots[n];
      s = Slot{};
      if (len == 1) continue;   // slot alone: it goes
      if (len < kSlotFixed + 1 || len < kSlotFixed + v[16] + 1u) return rejected(kRejectMalformed);
      const uint16_t wire_fn = getU16(v + 1), swdio = getU16(v + 3), swclk = getU16(v + 5);
      const uint8_t attach = v[7], idle = v[14], mechanism = v[15], name_length = v[16];
      const uint16_t retry_s = getU16(v + 8);
      const uint32_t max_hz = getU32(v + 10);
      if (attach > cfg::kSlotAttachAtBoot || (retry_s && attach != cfg::kSlotAttachAtBoot)) return rejected(kRejectMalformed);
      if (idle > reg::wire_rvswd::kIdleClockLow) return rejected(kRejectMalformed);
      if (name_length < 1 || name_length > kMaxName) return rejected(kRejectMalformed);
      for (uint8_t k = 0; k < name_length; ++k) if (!nameChar(static_cast<char>(v[kSlotFixed + k]))) return rejected(kRejectMalformed);
      int place = -1;   // the place of this wire, with a pair it allows (fixed, or any the host may choose)
      for (size_t k = 0; k < place_count_; ++k)
        if (places_[k].wire_fn == wire_fn && pairAllowed(*places_[k].port, swdio, swclk)) place = static_cast<int>(k);
      if (place < 0) return rejected(kRejectUnavailable);
      DmiPhy &phy = places_[place].port->dm.phy();
      if (idle && !phy.canIdleClockLow()) return rejected(kRejectMalformed);   // idle_clock low: rvswd's only
      if (!phy.keepsMaxHz(max_hz)) return rejected(kRejectUnsupported);       // a ceiling this line cannot keep
      if (mechanism > reg::target_console::kMechanismDmseq) return rejected(kRejectUnsupported);
      const uint8_t *lock = v + kSlotFixed + name_length;
      const size_t lock_bytes = len - kSlotFixed - name_length;   // scheme [mask value]
      if (lock[0] == 0 ? lock_bytes != 1 : (lock_bytes < 3 || (lock_bytes - 1) % 2 || (lock_bytes - 1) / 2 > kMaxLock))
        return rejected(kRejectMalformed);
      s.set = true;
      s.place = static_cast<uint8_t>(place);
      s.swdio = swdio;
      s.swclk = swclk;
      s.attach = attach;
      s.retry_s = retry_s;
      s.max_hz = max_hz;
      s.idle_low = idle != 0;
      s.mechanism = mechanism;
      s.name_length = name_length;
      memcpy(s.name, v + kSlotFixed, name_length);
      s.name[name_length] = 0;
      s.lock_scheme = lock[0];
      s.lock_length = static_cast<uint8_t>(lock[0] ? (lock_bytes - 1) / 2 : 0);
      memcpy(s.mask, lock + 1, s.lock_length);
      memcpy(s.value, lock + 1 + s.lock_length, s.lock_length);
    } else if (tag == cfg::kTlvItemBind) {
      if (len < 1 || v[0] >= Binds::kMaxPorts || binds[v[0]].touched) return rejected(kRejectMalformed);
      BindItem &b = binds[v[0]];
      b.touched = true;
      if (len == 1) continue;   // port alone: that bind goes
      if (!endpoint_.isSerialPort(v[0])) return rejected(kRejectUnavailable);
      if (len < 4 || v[3] < 1 || v[3] > Binds::kMaxStreams || len != 4u + 3u * v[3]) return rejected(kRejectMalformed);
      if (v[1] > Binds::kMixed) return rejected(kRejectUnsupported);
      if (v[1] == Binds::kManual && v[2] >= v[3]) return rejected(kRejectMalformed);
      b.set = true;
      b.mode = v[1];
      b.selected = v[1] == Binds::kManual ? v[2] : 0;
      b.count = v[3];
      for (uint8_t k = 0; k < b.count; ++k) {
        b.kinds[k] = v[4 + 3 * k];
        b.ids[k] = getU16(v + 5 + 3 * k);
        if (b.kinds[k] != Binds::kSlotConsole && b.kinds[k] != Binds::kFixtureUart) return rejected(kRejectMalformed);
      }
    } else {
      return rejected(kRejectUnsupported);   // label: not taken here; others unknown
    }
  }
  // the whole: one slot per place, at most one at-boot slot per wire (max_connections 1), every bind's streams there
  // the whole: no two slots on one place with the same pair, at most one at-boot slot per wire (max_connections 1),
  // every bind's streams there
  for (size_t a = 0; a < kMaxSlots; ++a) {
    if (!slots[a].set) continue;
    uint8_t at_boot = 0;
    for (size_t b = 0; b < kMaxSlots; ++b) {
      if (!slots[b].set || slots[b].place != slots[a].place) continue;
      if (b > a && slots[b].swdio == slots[a].swdio && slots[b].swclk == slots[a].swclk) return rejected(kRejectUnavailable);
      at_boot += slots[b].attach == cfg::kSlotAttachAtBoot;
    }
    if (at_boot > 1) return rejected(kRejectUnavailable);
  }
  Binds::Spec specs[Binds::kMaxPorts];
  for (uint8_t port = 0; port < Binds::kMaxPorts; ++port) {
    specs[port] = binds_.spec(port);
    const BindItem &b = binds[port];
    if (b.touched) {
      specs[port] = Binds::Spec{};
      if (b.set) {
        specs[port].set = true;
        specs[port].mode = b.mode;
        specs[port].selected = b.selected;
        specs[port].count = b.count;
      }
    }
    Binds::Spec &sp = specs[port];
    for (uint8_t k = 0; sp.set && k < sp.count; ++k) {
      const uint8_t kind = b.touched ? b.kinds[k] : sp.sources[k].kind;
      const uint16_t id = b.touched ? b.ids[k] : sp.sources[k].id;
      if (!sourceFor(slots, kind, id, sp.sources[k])) return rejected(kRejectUnavailable);   // a slot gone, a fn not a UART
    }
  }
  RoleAssignment before[Endpoint::kMaxRoles];
  const size_t had = endpoint_.plan(before, Endpoint::kMaxRoles, true);
  (void)had;
  if (plan_fn_count) {
    const uint8_t reason = endpoint_.replacePlan(roles, role_count, plan_fns, plan_fn_count);
    if (reason) return rejected(reason);
  }
  // accepted: make it current
  if (has_idle) {
    memcpy(idle_, idle, sizeof idle_);
    for (uint16_t c = 0; c < PinTable::kChannels; ++c) if ((idle_touched >> c) & 1) pins_->setIdle(c, idle_[c]);
  }
  for (size_t i = 0; i < kMaxSlots; ++i) {
    if (!slot_touched[i]) continue;
    slots_[i] = slots[i];
    runs_[i] = SlotRun{};
    runs_[i].due = slots[i].set && slots[i].attach == cfg::kSlotAttachAtBoot;   // an at-boot slot set: attach now
  }
  for (uint8_t port = 0; port < Binds::kMaxPorts; ++port) {
    if (binds[port].touched) binds_.set(port, specs[port]);
  }
  poll();
  return completed();
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
  if (s.lock_scheme != reg::wire_rvswd::kTargetIdSchemeWchDmi7f || s.lock_length != 4) return 0;
  uint8_t id[4];
  putU32(id, tid);
  for (uint8_t i = 0; i < 4; ++i) if ((id[i] & s.mask[i]) != s.value[i]) return 0;
  return 1;
}

// One slot (probe.config §3): a bound slot rides any connection on its place when the lock matches, with its console
// open; an at-boot slot attaches by itself (without stopping the hart) at boot, when set, and every retry_s while the
// target is not there. Nothing else uses the connection for the slot.
void ProbeConfig::runSlot(uint8_t i) {
  const Slot &s = slots_[i];
  SlotRun &r = runs_[i];
  const Place &p = places_[s.place];
  DebugPort &port = *p.port;
  const bool at_boot = s.attach == cfg::kSlotAttachAtBoot;
  if (!port.connected && at_boot) {
    const bool retry = s.retry_s && r.tried && static_cast<uint32_t>(millis() - r.last_try_ms) >= 1000u * s.retry_s;
    if (r.due || retry) {
      r.due = false;
      r.tried = true;
      r.last_try_ms = millis();
      uint32_t status = 0;
      // the link to the slot's pair first (host-chosen pins; a fixed pair is always there), unless its pins are held
      if (usePair(port, s.swdio, s.swclk) && attachRunning(port, DebugPort::kUserSlot, status, s.max_hz, s.idle_low)) {
        r.mismatch = lockMatches(s, port.has_tid, port.tid) != 1;
        r.mismatch_has_tid = port.has_tid;
        r.mismatch_tid = port.tid;
        if (r.mismatch) releaseConnection(port, DebugPort::kUserSlot, false);   // found, not the chip: let go of it
      }
    }
  }
  if (!port.connected || !onPair(s)) return;   // no link, or the link is on another pair (another slot, the host)
  if (lockMatches(s, port.has_tid, port.tid) == 1 && (at_boot || bound(i))) {
    port.users |= DebugPort::kUserSlot;
    r.mismatch = false;
    if (bound(i) && !p.console->isOpen()) p.console->bindOpen(s.mechanism);
  } else if (port.users & DebugPort::kUserSlot) {
    releaseConnection(port, DebugPort::kUserSlot, false);
  }
}

void ProbeConfig::poll() {
  for (size_t k = 0; k < place_count_; ++k) {   // the slot on the pair the link is on now: the connections entry's slot
    DebugPort &port = *places_[k].port;
    port.slot = 0xff;
    for (uint8_t i = 0; i < kMaxSlots; ++i) if (slots_[i].set && slots_[i].place == k && onPair(slots_[i])) port.slot = i;
    if (port.slot == 0xff && (port.users & DebugPort::kUserSlot)) releaseConnection(port, DebugPort::kUserSlot, false);
  }
  for (uint8_t i = 0; i < kMaxSlots; ++i) if (slots_[i].set) runSlot(i);
}

// ---- storage -------------------------------------------------------------------------------------------------------

void ProbeConfig::load() {
  Preferences p;
  if (!p.begin(kNvsNamespace, true)) return;
  saved_length_ = p.getBytesLength(kNvsItems);
  if (saved_length_ > 0 && saved_length_ <= sizeof saved_ && p.getBytes(kNvsItems, saved_, saved_length_) == saved_length_) {
    saved_hash_ = crc32Ieee(saved_, saved_length_);
    saved_list_ = p.getUInt(kNvsList, 0);
    storage_state_ = cfg::kStorageStateApplied;   // until applySaved says otherwise
  } else {
    saved_length_ = 0;
  }
  p.end();
}

void ProbeConfig::applySaved() {
  if (!saved_length_) return;
  // Saved for another interface list (another firmware): its fn numbers may name other functions - leave it unapplied.
  if (saved_list_ != endpoint_.listHash()) {
    storage_state_ = cfg::kStorageStateUnreadable;
    return;
  }
  const Result r = apply(saved_, saved_length_);
  if (r.resolution != kResolutionCompleted || r.detail != kOutcomeSuccess) storage_state_ = cfg::kStorageStateUnreadable;
}

// ---- describe and the operations -------------------------------------------------------------------------------------

size_t ProbeConfig::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  uint8_t st[13];
  putU32(st, kMaxSaved);
  st[4] = storage_state_;
  putU32(st + 5, saved_length_ ? saved_hash_ : 0);
  putU32(st + 9, save_ms_ > 200 ? save_ms_ : 200);
  w.put(cfg::kTlvDescribeStorage, st, sizeof st);
  static const uint8_t kItems[] = {cfg::kTlvItemPlan, cfg::kTlvItemSlot, cfg::kTlvItemBind, cfg::kTlvItemIdle};
  w.put(cfg::kTlvDescribeItems, kItems, pins_ ? sizeof kItems : sizeof kItems - 1);
  w.u8(cfg::kTlvDescribeSlotsMax, place_count_ ? static_cast<uint8_t>(kMaxSlots) : 0);
  w.u8(cfg::kTlvDescribeBindModes, 0b111);   // last-reset, manual, mixed
  for (uint8_t i = 0; i < kMaxSlots; ++i) {   // slot_state (probe.config §3.2), lock-free
    const Slot &s = slots_[i];
    if (!s.set) continue;
    const SlotRun &r = runs_[i];
    const DebugPort &port = *places_[s.place].port;
    uint8_t v[14] = {i};
    bool has_tid = false;
    uint32_t tid = 0;
    if (port.connected && onPair(s)) {
      has_tid = port.has_tid;
      tid = port.tid;
      const int m = lockMatches(s, has_tid, tid);
      v[1] = m == 1 ? cfg::kSlotStateConnected : (m < 0 ? cfg::kSlotStateNoTargetId : cfg::kSlotStateLockMismatch);
      putU16(v + 2, port.number);
    } else {
      has_tid = r.mismatch && r.mismatch_has_tid;
      tid = r.mismatch_tid;
      v[1] = r.mismatch ? (r.mismatch_has_tid ? cfg::kSlotStateLockMismatch : cfg::kSlotStateNoTargetId)
                        : cfg::kSlotStateAbsent;
      putU16(v + 2, 0);
    }
    putU32(v + 4, r.tried ? static_cast<uint32_t>(millis() - r.last_try_ms) : 0xffffffffu);
    v[8] = has_tid ? reg::wire_rvswd::kTargetIdSchemeWchDmi7f : 0;
    v[9] = has_tid ? 4 : 0;
    putU32(v + 10, tid);
    w.put(cfg::kTlvDescribeSlotState, v, has_tid ? 14 : 10);
  }
  for (uint8_t port = 0; port < Binds::kMaxPorts; ++port) {   // bind_state
    const Binds::Spec &b = binds_.spec(port);
    if (!b.set) continue;
    const uint8_t flow = endpoint_.held(port) ? cfg::kBindFlowHeld
                         : binds_.streaming(port) ? cfg::kBindFlowStreaming : cfg::kBindFlowIdle;
    const uint8_t v[4] = {port, b.mode, b.mode == Binds::kMixed ? uint8_t{0xff} : binds_.selected(port), flow};
    w.put(cfg::kTlvDescribeBindState, v, sizeof v);
  }
  return w.ok() ? w.length() : 0;
}

Result ProbeConfig::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (op) {
    case cfg::kOpGet: {   // first(u16) -> more(u8) hash(u32) items from the first-th on
      if (length != 2) return rejected(kRejectMalformed);
      uint8_t items[kMaxSaved];
      const size_t n = canonical(items, sizeof items);
      if (capacity < 5) return failed();
      size_t at = 0, index = 0, put = 5;
      const uint16_t first = getU16(payload);
      bool more = false;
      while (at < n) {
        const size_t item = 2u + items[at + 1];
        if (index++ >= first) {
          if (put + item > capacity) { more = true; break; }
          memcpy(out + put, items + at, item);
          put += item;
        }
        at += item;
      }
      out[0] = more ? 1 : 0;
      putU32(out + 1, crc32Ieee(items, n));
      return completed(put);
    }
    case cfg::kOpSet: {
      const Result r = apply(payload, length);
      if (r.resolution != kResolutionCompleted || r.detail != kOutcomeSuccess || capacity < 4) return r;
      putU32(out, hash());
      return completed(4);
    }
    case cfg::kOpSave: {
      if (length) return rejected(kRejectMalformed);
      uint8_t items[kMaxSaved];
      const size_t n = canonical(items, sizeof items);
      const uint32_t h = crc32Ieee(items, n);
      if (!(saved_length_ == n && saved_hash_ == h && saved_list_ == endpoint_.listHash())) {   // the same is not written again
        const uint32_t t0 = millis();
        Preferences p;
        if (!p.begin(kNvsNamespace, false)) return failed();
        const size_t written = n ? p.putBytes(kNvsItems, items, n) : (p.remove(kNvsItems), 0);
        p.putUInt(kNvsList, endpoint_.listHash());
        p.end();
        save_ms_ = millis() - t0;
        if (written != n) return failed();
        memcpy(saved_, items, n);
        saved_length_ = n;
        saved_hash_ = h;
        saved_list_ = endpoint_.listHash();
        storage_state_ = n ? cfg::kStorageStateApplied : cfg::kStorageStateNone;
      }
      if (capacity < 4) return failed();
      putU32(out, h);
      return completed(4);
    }
    case cfg::kOpErase: {
      if (length) return rejected(kRejectMalformed);
      Preferences p;
      if (!p.begin(kNvsNamespace, false)) return failed();
      p.remove(kNvsItems);
      p.end();
      saved_length_ = 0;
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
