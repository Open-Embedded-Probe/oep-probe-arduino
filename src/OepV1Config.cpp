#include "OepV1Config.h"

#if defined(ARDUINO_ARCH_ESP32)

#include <Preferences.h>

#include "OepV1Endpoint.h"

namespace oep {
namespace v1 {

namespace cfg = reg::probe_config;

uint32_t crc32Ieee(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

size_t ProbeConfig::canonical(uint8_t *out, size_t capacity) const {
  TlvWriter w(out, capacity);
  if (boot_mode_ != 0xFF) w.u8(cfg::kTlvItemBootMode, boot_mode_);
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
  for (size_t port = 0; port < port_count_; ++port) {   // ascending port = the canonical order
    const Bind &b = binds_[port];
    if (!b.set) continue;
    uint8_t v[4 + kMaxBindArgs] = {static_cast<uint8_t>(port), b.source, b.attach, b.flags};
    memcpy(v + 4, b.args, b.arg_length);
    w.put(cfg::kTlvItemBind, v, 4u + b.arg_length);
  }
  for (uint32_t fn = 0; fn <= 0xffff; ++fn) {           // ascending wire_fn (few targets: a short scan per fn)
    bool any = false;
    for (const Target &t : targets_) {
      if (!t.set) continue;
      any |= t.wire_fn >= fn;
      if (t.wire_fn != fn) continue;
      uint8_t v[3 + 2 * kMaxIdMask];
      putU16(v, t.wire_fn);
      v[2] = t.scheme;
      memcpy(v + 3, t.mask, t.length);
      memcpy(v + 3 + t.length, t.value, t.length);
      w.put(cfg::kTlvItemTarget, v, 3u + 2u * t.length);
    }
    if (!any) break;
  }
  for (uint16_t c = 0; c < PinTable::kChannels; ++c) {
    if (idle_[c] == PinTable::kIdleUnset) continue;
    uint8_t v[3];
    putU16(v, c);
    v[2] = idle_[c];
    w.put(cfg::kTlvItemIdle, v, 3);
  }
  return w.ok() ? w.length() : 0;
}

uint32_t ProbeConfig::hash() const {
  uint8_t buf[kMaxSaved];
  return crc32Ieee(buf, canonical(buf, sizeof buf));
}

const ProbeConfig::Target *ProbeConfig::target(uint16_t wire_fn) const {
  for (const Target &t : targets_) if (t.set && t.wire_fn == wire_fn) return &t;
  return nullptr;
}

bool ProbeConfig::matches(uint16_t wire_fn, uint8_t scheme, const uint8_t *id, size_t length) const {
  const Target *t = target(wire_fn);
  if (!t || t->scheme != scheme || t->length != length) return false;
  for (size_t i = 0; i < length; ++i) if ((id[i] & t->mask[i]) != t->value[i]) return false;
  return true;
}

// Checks every item first, then changes: plans (the endpoint's, the fns named, all or nothing), then binds (a bind may
// need a plan's pins), then idle states, targets and boot_mode. A refusal leaves everything as it was. Keys a set does
// not carry keep their items (oep-if-probe-config §2).
Result ProbeConfig::apply(const uint8_t *items, size_t length) {
  bool has_mode = false, bind_touched[kMaxPorts] = {};
  uint8_t mode = boot_mode_;
  RoleAssignment roles[Endpoint::kMaxRoles];
  size_t role_count = 0;
  uint16_t plan_fns[16];
  size_t plan_fn_count = 0;
  Bind binds[kMaxPorts];
  for (size_t port = 0; port < kMaxPorts; ++port) binds[port] = binds_[port];
  Target targets[kMaxTargets];
  for (size_t i = 0; i < kMaxTargets; ++i) targets[i] = targets_[i];
  bool target_touched[kMaxTargets] = {};
  uint8_t idle[PinTable::kChannels];
  memcpy(idle, idle_, sizeof idle);
  uint64_t idle_touched = 0;
  bool has_idle = false;
  for (size_t at = 0; at < length;) {
    if (length - at < 2 || length - at - 2 < items[at + 1]) return rejected(kRejectMalformed);
    const uint8_t tag = items[at] & 0x7f, len = items[at + 1];   // kept and hashed without the critical bit (§2)
    const uint8_t *v = items + at + 2;
    at += 2u + len;
    if (tag == cfg::kTlvItemBootMode) {
      if (len > 1 || has_mode) return rejected(kRejectMalformed);
      if (len && v[0] >= mode_count_) return rejected(kRejectUnsupported);
      has_mode = true;
      mode = len ? v[0] : 0xFF;
    } else if (tag == cfg::kTlvItemPlan) {
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
      if (role_count >= Endpoint::kMaxRoles) return rejected(kRejectMalformed);
      roles[role_count++] = {fn, v[2], getU16(v + 3)};
    } else if (tag == cfg::kTlvItemBind) {
      if (len < 1 || v[0] >= port_count_ || bind_touched[v[0]]) return rejected(kRejectMalformed);
      bind_touched[v[0]] = true;
      Bind &b = binds[v[0]];
      b = Bind{};
      if (len == 1) continue;   // port alone: that bind goes
      if (len < 4 || len - 4 > kMaxBindArgs) return rejected(kRejectMalformed);
      if (v[1] > cfg::kBindSourceTargetConsole) return rejected(kRejectUnsupported);
      // fixture.uart: fn u16, baud u32 (0: not until the port's line coding, which then needs flags bit0), format u8
      if (v[1] == cfg::kBindSourceFixtureUart && (len != 4 + kUartArgs || v[2] != cfg::kBindAttachHost))
        return rejected(kRejectMalformed);
      if (v[1] == cfg::kBindSourceFixtureUart && getU32(v + 6) == 0 && !(v[3] & 1)) return rejected(kRejectMalformed);
      // target.console: wire_fn u16, mechanism u8, swdio u16, swclk u16, max_speed u32 (0: no ceiling)
      if (v[1] == cfg::kBindSourceTargetConsole && len != 4 + kConsoleArgs) return rejected(kRejectMalformed);
      if (v[2] > cfg::kBindAttachAtBoot) return rejected(kRejectUnsupported);
      if (v[3] & ~0x01) return rejected(kRejectUnsupported);   // bit0 line coding
      b.set = true;
      b.source = v[1];
      b.attach = v[2];
      b.flags = v[3];
      b.arg_length = static_cast<uint8_t>(len - 4);
      memcpy(b.args, v + 4, b.arg_length);
    } else if (tag == cfg::kTlvItemTarget) {
      if (len < 2 || (len > 2 && (len < 5 || (len - 3) % 2 || (len - 3) / 2 > kMaxIdMask))) return rejected(kRejectMalformed);
      const uint16_t fn = getU16(v);
      int slot = -1;
      for (size_t i = 0; i < kMaxTargets; ++i) if (targets[i].set && targets[i].wire_fn == fn) slot = static_cast<int>(i);
      if (slot >= 0 && target_touched[slot]) return rejected(kRejectMalformed);
      if (len == 2) { if (slot >= 0) { targets[slot] = Target{}; target_touched[slot] = true; } continue; }
      if (slot < 0) for (size_t i = 0; i < kMaxTargets && slot < 0; ++i) if (!targets[i].set) slot = static_cast<int>(i);
      if (slot < 0) return rejected(kRejectUnavailable);
      Target &t = targets[slot];
      target_touched[slot] = true;
      t.set = true;
      t.wire_fn = fn;
      t.scheme = v[2];
      t.length = static_cast<uint8_t>((len - 3) / 2);
      memcpy(t.mask, v + 3, t.length);
      memcpy(t.value, v + 3 + t.length, t.length);
    } else if (tag == cfg::kTlvItemIdle) {
      if (!pins_) return rejected(kRejectUnsupported);
      if (len != 2 && len != 3) return rejected(kRejectMalformed);
      const uint16_t c = getU16(v);
      if (c >= PinTable::kChannels || !pins_->allowed(c) || ((idle_touched >> c) & 1)) return rejected(kRejectMalformed);
      if (len == 3 && v[2] > PinTable::kIdlePullDown) return rejected(kRejectUnsupported);
      idle_touched |= uint64_t{1} << c;
      idle[c] = len == 3 ? v[2] : PinTable::kIdleUnset;
      has_idle = true;
    } else {
      return rejected(kRejectUnsupported);   // label: not in this prototype; others unknown
    }
  }
  // a bind that attaches by itself checks the target it finds against its wire's target item: it must be there
  for (size_t port = 0; port < port_count_; ++port) {
    const Bind &b = binds[port];
    if (!b.set || b.source != cfg::kBindSourceTargetConsole || b.attach == cfg::kBindAttachHost) continue;
    bool found = false;
    for (const Target &t : targets) found |= t.set && t.wire_fn == getU16(b.args);
    if (!found) return rejected(kRejectUnavailable);
  }
  RoleAssignment before[Endpoint::kMaxRoles];
  const size_t had = endpoint_.plan(before, Endpoint::kMaxRoles, true);
  if (plan_fn_count) {
    const uint8_t reason = endpoint_.replacePlan(roles, role_count, plan_fns, plan_fn_count);
    if (reason) return rejected(reason);
  }
  const Bind clear{};
  for (size_t port = 0; port < livePorts(); ++port) {   // only the running mode's ports are wired; others are kept
    if (!bind_touched[port]) continue;
    if (hook_ && !hook_(static_cast<uint8_t>(port), binds[port].set ? binds[port] : clear, hook_context_)) {
      for (size_t undo = 0; undo < port; ++undo)   // put back what was wired before, then the plans
        if (bind_touched[undo] && hook_)
          hook_(static_cast<uint8_t>(undo), binds_[undo].set ? binds_[undo] : clear, hook_context_);
      if (plan_fn_count) endpoint_.replacePlan(before, had, plan_fns, plan_fn_count);
      return failed();
    }
  }
  for (size_t port = 0; port < kMaxPorts; ++port) {
    if (bind_touched[port]) bind_state_[port] = 0;
    binds_[port] = binds[port];
  }
  if (has_idle) {
    memcpy(idle_, idle, sizeof idle_);
    for (uint16_t c = 0; c < PinTable::kChannels; ++c) if ((idle_touched >> c) & 1) pins_->setIdle(c, idle_[c]);
  }
  for (size_t i = 0; i < kMaxTargets; ++i) targets_[i] = targets[i];
  if (has_mode) boot_mode_ = mode;
  return completed();
}

void ProbeConfig::load() {
  Preferences p;
  if (!p.begin("oepcfg", true)) return;
  saved_length_ = p.getBytesLength("items");
  if (saved_length_ > 0 && saved_length_ <= sizeof saved_ && p.getBytes("items", saved_, saved_length_) == saved_length_) {
    saved_hash_ = crc32Ieee(saved_, saved_length_);
    saved_list_ = p.getUInt("list", 0);
    storage_state_ = cfg::kStorageStateApplied;   // until applySaved says otherwise
    // the boot mode is needed before USB starts: take it out now
    for (size_t at = 0; at + 2 <= saved_length_; at += 2u + saved_[at + 1]) {
      if (saved_[at] == cfg::kTlvItemBootMode && saved_[at + 1] == 1) boot_mode_ = saved_[at + 2];
    }
  } else {
    saved_length_ = 0;
  }
  p.end();
}

uint8_t ProbeConfig::bootMode(uint8_t fallback) {
  current_mode_ = boot_mode_ < mode_count_ ? boot_mode_ : fallback;
  return current_mode_;
}

void ProbeConfig::applySaved() {
  if (!saved_length_) return;
  // Saved for another interface list (another firmware): its fn numbers may name other functions - leave it unapplied.
  // Its boot mode was already used, if the device could be built in it (the way back over USB).
  if (saved_list_ != endpoint_.listHash()) {
    storage_state_ = cfg::kStorageStateUnreadable;
    return;
  }
  const Result r = apply(saved_, saved_length_);
  if (r.resolution != kResolutionCompleted || r.detail != kOutcomeSuccess) {
    storage_state_ = cfg::kStorageStateUnreadable;
    boot_mode_ = 0xFF;
  }
}

bool ProbeConfig::forgetBootMode() {
  if (boot_mode_ == 0xFF) return false;
  uint8_t kept[kMaxSaved];
  size_t n = 0;
  for (size_t at = 0; at + 2 <= saved_length_ && at + 2u + saved_[at + 1] <= saved_length_; at += 2u + saved_[at + 1]) {
    if (saved_[at] == cfg::kTlvItemBootMode) continue;
    memcpy(kept + n, saved_ + at, 2u + saved_[at + 1]);
    n += 2u + saved_[at + 1];
  }
  Preferences p;
  if (!p.begin("oepcfg", false)) return false;
  if (n) p.putBytes("items", kept, n);
  else p.remove("items");
  p.end();
  boot_mode_ = 0xFF;
  return true;
}

void ProbeConfig::poll() {
  if (reboot_ && static_cast<int32_t>(millis() - reboot_at_) >= 0) ESP.restart();
}

size_t ProbeConfig::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  for (size_t i = 0; i < mode_count_; ++i) {
    uint8_t v[3 + 32] = {static_cast<uint8_t>(i), modes_[i].functions, modes_[i].ports};
    const size_t n = strnlen(modes_[i].name, 32);
    memcpy(v + 3, modes_[i].name, n);
    w.put(cfg::kTlvDescribeMode, v, 3 + n);
  }
  const uint8_t cm[2] = {current_mode_, boot_mode_ < mode_count_ ? boot_mode_ : current_mode_};
  w.put(cfg::kTlvDescribeCurrentMode, cm, 2);
  for (size_t port = 0; port < livePorts(); ++port) {
    const uint8_t v[2] = {static_cast<uint8_t>(port), port_interfaces_[port]};
    w.put(cfg::kTlvDescribePort, v, 2);
  }
  uint8_t st[17];
  putU32(st, kMaxSaved);
  st[4] = storage_state_;
  putU32(st + 5, saved_length_ ? saved_hash_ : 0);
  putU32(st + 9, save_ms_ > 200 ? save_ms_ : 200);
  putU32(st + 13, 5000);   // reboot to re-enumerated (measured about 1.5 s on the P4 HS)
  w.put(cfg::kTlvDescribeStorage, st, sizeof st);
  static const uint8_t kItems[] = {cfg::kTlvItemBootMode, cfg::kTlvItemPlan, cfg::kTlvItemBind, cfg::kTlvItemTarget,
                                   cfg::kTlvItemIdle};
  w.put(cfg::kTlvDescribeItems, kItems, pins_ ? sizeof kItems : sizeof kItems - 1);
  for (size_t port = 0; port < port_count_; ++port) {
    if (!binds_[port].set) continue;
    const uint8_t v[2] = {static_cast<uint8_t>(port), bind_state_[port]};
    w.put(cfg::kTlvDescribeBindState, v, 2);
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
        if (!p.begin("oepcfg", false)) return failed();
        const size_t written = n ? p.putBytes("items", items, n) : (p.remove("items"), 0);
        p.putUInt("list", endpoint_.listHash());
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
      if (!p.begin("oepcfg", false)) return failed();
      p.remove("items");
      p.end();
      saved_length_ = 0;
      saved_hash_ = 0;
      storage_state_ = cfg::kStorageStateNone;
      return completed();
    }
    case cfg::kOpReboot:
      if (length) return rejected(kRejectMalformed);
      reboot_ = true;
      reboot_at_ = millis() + 100;   // after the answer has left
      return completed();
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace v1
}  // namespace oep

#endif
