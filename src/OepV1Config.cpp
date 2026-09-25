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
  const size_t n = endpoint_.plan(roles, Endpoint::kMaxRoles);
  if (n) {
    uint8_t v[Endpoint::kMaxRoles * 5];
    for (size_t i = 0; i < n; ++i) {
      putU16(v + i * 5, roles[i].function);
      v[i * 5 + 2] = roles[i].role;
      putU16(v + i * 5 + 3, roles[i].channel);
    }
    w.put(cfg::kTlvItemPlan, v, n * 5);
  }
  for (size_t port = 0; port < port_count_; ++port) {   // ascending port = the canonical order
    const Bind &b = binds_[port];
    if (!b.set) continue;
    uint8_t v[4 + kMaxBindArgs] = {static_cast<uint8_t>(port), b.source, b.attach, b.flags};
    memcpy(v + 4, b.args, b.arg_length);
    w.put(cfg::kTlvItemBind, v, 4u + b.arg_length);
  }
  if (target_.set) {
    uint8_t v[6];
    putU16(v, target_.wire_fn);
    putU32(v + 2, target_.chip_id);
    w.put(cfg::kTlvItemTarget, v, 6);
  }
  return w.ok() ? w.length() : 0;
}

uint32_t ProbeConfig::hash() const {
  uint8_t buf[kMaxSaved];
  return crc32Ieee(buf, canonical(buf, sizeof buf));
}

// Checks every item first, then changes: plan (the endpoint's, all or nothing), then binds (a bind may need the plan's
// pins), then boot_mode. A refusal leaves everything as it was.
Result ProbeConfig::apply(const uint8_t *items, size_t length) {
  bool has_mode = false, has_plan = false, has_bind = false, has_target = false;
  Target target;
  uint8_t mode = 0xFF;
  RoleAssignment roles[Endpoint::kMaxRoles];
  size_t role_count = 0;
  Bind binds[kMaxPorts];
  for (size_t at = 0; at < length;) {
    if (length - at < 2 || length - at - 2 < items[at + 1]) return rejected(kRejectMalformed);
    const uint8_t tag = items[at], len = items[at + 1];
    const uint8_t *v = items + at + 2;
    at += 2u + len;
    if (tag == cfg::kTlvItemBootMode) {
      if (len > 1) return rejected(kRejectMalformed);
      if (len && v[0] >= mode_count_) return rejected(kRejectUnsupported);
      has_mode = true;
      mode = len ? v[0] : 0xFF;
    } else if (tag == cfg::kTlvItemPlan) {
      if (len % 5 || len / 5 > Endpoint::kMaxRoles) return rejected(kRejectMalformed);
      has_plan = true;
      role_count = len / 5;
      for (size_t i = 0; i < role_count; ++i) roles[i] = {getU16(v + i * 5), v[i * 5 + 2], getU16(v + i * 5 + 3)};
    } else if (tag == cfg::kTlvItemBind) {
      has_bind = true;
      if (!len) continue;   // clears every bind
      if (len < 4 || len - 4 > kMaxBindArgs) return rejected(kRejectMalformed);
      if (v[0] >= port_count_ || binds[v[0]].set) return rejected(kRejectMalformed);
      if (v[1] > cfg::kBindSourceTargetConsole) return rejected(kRejectUnsupported);
      // fixture.uart: fn u16, baud u32 (0: not until the port's line coding, which then needs flags bit0), format u8
      if (v[1] == cfg::kBindSourceFixtureUart && (len != 11 || v[2] != cfg::kBindAttachHost)) return rejected(kRejectMalformed);
      if (v[1] == cfg::kBindSourceFixtureUart && getU32(v + 6) == 0 && !(v[3] & 1)) return rejected(kRejectMalformed);
      if (v[1] == cfg::kBindSourceTargetConsole && len != 7) return rejected(kRejectMalformed);
      if (v[2] > cfg::kBindAttachAtBoot) return rejected(kRejectUnsupported);
      if (v[3] & ~0x01) return rejected(kRejectUnsupported);   // bit0 line coding; bit1 (TX before open) withdrawn
      Bind &b = binds[v[0]];
      b.set = true;
      b.source = v[1];
      b.attach = v[2];
      b.flags = v[3];
      b.arg_length = static_cast<uint8_t>(len - 4);
      memcpy(b.args, v + 4, b.arg_length);
    } else if (tag == cfg::kTlvItemTarget) {
      if (len != 0 && len != 6) return rejected(kRejectMalformed);
      has_target = true;
      target.set = len == 6;
      if (len) { target.wire_fn = getU16(v); target.chip_id = getU32(v + 2); }
    } else {
      return rejected(kRejectUnsupported);   // label: not in this prototype; others unknown
    }
  }
  // a bind that attaches by itself checks the target it finds against the target item: it must be there
  const bool target_after = has_target ? target.set : target_.set;
  for (size_t port = 0; port < port_count_; ++port) {
    const Bind &b = has_bind ? binds[port] : binds_[port];
    if (b.set && b.source == cfg::kBindSourceTargetConsole && b.attach != cfg::kBindAttachHost && !target_after)
      return rejected(kRejectUnavailable);
  }
  RoleAssignment before[Endpoint::kMaxRoles];
  const size_t had = endpoint_.plan(before, Endpoint::kMaxRoles);
  if (has_plan) {
    const uint8_t reason = endpoint_.replacePlan(roles, role_count);
    if (reason) return rejected(reason);
  }
  if (has_bind) {   // only the ports the running mode has are wired; the others are kept for a mode that has them
    const Bind clear{};
    for (size_t port = 0; port < livePorts(); ++port) {
      if (!binds[port].set && !binds_[port].set) continue;
      if (hook_ && !hook_(static_cast<uint8_t>(port), binds[port].set ? binds[port] : clear, hook_context_)) {
        for (size_t undo = 0; undo < port; ++undo)   // put back what was wired before, then the plan
          if (hook_) hook_(static_cast<uint8_t>(undo), binds_[undo].set ? binds_[undo] : clear, hook_context_);
        if (has_plan) endpoint_.replacePlan(before, had);
        return failed();
      }
    }
    for (size_t port = 0; port < port_count_; ++port) binds_[port] = binds[port];
  }
  if (has_mode) boot_mode_ = mode;
  if (has_target) target_ = target;
  return completed();
}

void ProbeConfig::load() {
  Preferences p;
  if (!p.begin("oepcfg", true)) return;
  saved_length_ = p.getBytesLength("items");
  if (saved_length_ > 0 && saved_length_ <= sizeof saved_ && p.getBytes("items", saved_, saved_length_) == saved_length_) {
    saved_hash_ = crc32Ieee(saved_, saved_length_);
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
      if (!(saved_length_ == n && saved_hash_ == h)) {   // the same content is not written again
        const uint32_t t0 = millis();
        Preferences p;
        if (!p.begin("oepcfg", false)) return failed();
        const size_t written = n ? p.putBytes("items", items, n) : (p.remove("items"), 0);
        p.end();
        save_ms_ = millis() - t0;
        if (written != n) return failed();
        memcpy(saved_, items, n);
        saved_length_ = n;
        saved_hash_ = h;
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
