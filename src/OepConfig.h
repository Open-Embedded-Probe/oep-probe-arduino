// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 oep.probe.config (oep-spec docs/oep-if-probe-config.ja.md, revision 1): the probe's settings, set by the host
// and kept only when the host saves them. No modes, no reboot: everything set takes effect at once.
//
//   0x01 get(first u16) -> more(u8) hash(u32) items      (no lock)
//   0x02 set(items) -> hash(u32)      0x03 save -> hash(u32)      0x04 erase
//   0x05 unset(n u8, n x (len u8, tag u8, key)) -> hash(u32)
//   0x06 state(first_slot u8, first_bind u8) -> more storage_state storage_hash unreadable_reason
//        n_slots x (len, slot_state) n_binds x (len, bind_state)          (no lock)
//
// Items (TLV), each with a key; a set replaces the keys it carries and leaves the others, unset removes keys:
//   0x01 plan  fn(u16) role(u8) channel(u16)            key (fn, role, channel); a set replaces the whole plan of the fn
//   0x02 label channel(u16) text                         key channel (read back with get; oep.core's describe has only
//                                                        the firmware's fixed labels)
//   0x03 idle  channel(u16) mode(u8)                     key channel: 0 Hi-Z, 1 pull-up, 2 pull-down while free
//   0x04 slot  slot(u8) wire_fn(u16) swdio(u16) swclk(u16) attach(u8) retry_ms(u32) max_speed_hz(u32) idle_clock(u8)
//              mechanism(u8: 0xFF none) name_len(u8) name lock_len(u8) [lock_scheme(u8) mask(n) value(n)]   key slot
//   0x05 bind  port(u8) mode(u8) selected(u8) n(u8) n x (len(u8) kind(u8) id(u16))                       key port
//   0x06 uart  fn(u16) baud(u32) format(u8)              key fn: a fixture UART's settings, in force when its plan has pins
//   0x07 disable channel(u16)                            key channel: never used, driven or configured (not on this
//                                                        board): every request naming it is unavailable cause 5, and
//                                                        the pin is never parked; describe still offers it
// The probe keeps every item's bytes as the host sent them (the critical bit cleared, unknown tails kept) in the
// canonical order (tag, then key), which is what get pages and the hash (CRC-32) covers. Saved to NVS on ESP32
// (Preferences "oepcfg" / "items4") or the flash's last sector on RP2040 / RP2350 (EEPROM, "OEP4"), with the identity of
// every interface the items name; a saved copy naming an interface that is gone is not applied (state says why).
//
// The places a slot may name are the sketch's wires with their consoles (addPlace: one connection each); a slot names
// a pair its wire allows (the fixed pair, or any pair when the host chooses the pins), and several slots may share a
// wire on different pairs (one at boot). The streams a bind may carry are those places' consoles and the fixture UARTs
// added (addUart), which also take the uart item.
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"
#include "OepBind.h"
#include "OepConsole.h"
#include "OepFixture.h"
#include "OepTarget.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)

namespace oep {

class Endpoint;

class ProbeConfig final : public Interface {
 public:
  static constexpr size_t kMaxPlaces = 2, kMaxSlots = 4, kMaxUarts = 4, kMaxLock = 8, kMaxName = 32;
  // The items (canonical bytes) this probe holds and saves: describe storage max_bytes. The identity table saved with
  // them has its own room (kMaxIds), so a configuration of kMaxItems bytes always fits (probe.config §2).
  static constexpr size_t kMaxItems = 384, kMaxIds = 320;
  static constexpr size_t kMaxLabels = 8;

  ProbeConfig(Endpoint &endpoint, Binds &binds) : endpoint_(endpoint), binds_(binds) {
    memset(idle_, PinTable::kIdleUnset, sizeof idle_);
    binds_.setNamer(&ProbeConfig::nameOf, this);
  }
  // A place a slot may name: this wire and its console. Add after the endpoint has them.
  bool addPlace(WireRvswd &wire, TargetConsoleStream &console);
  // A fixture UART a bind may carry and the uart item sets (after the endpoint has it).
  bool addUart(FixtureUart &uart);
  // The pins whose idle state the idle item sets and the disable item takes away (without it, both are refused).
  void setPins(PinTable *pins) { pins_ = pins; endpoint_.setPins(pins); }   // the endpoint's plan replacements too

  // Read what was saved, then apply it: both after the sketch's last endpoint.add(). The saved items keep the
  // (name, instance, revision) of every interface they name and are renumbered to where those are now; one gone (or of
  // another revision) leaves them unapplied (storage unreadable, probe.config §2).
  // load() only reads the storage (no interface is needed): a sketch that parks its free pins at start-up loads first
  // and leaves the saved disable items' channels alone (savedDisabled, probe.config §2: they apply before any idle /
  // park). applySaved then sets the PinTable's disabled channels from what was applied (none when it was not, those
  // pins then going to their idle state).
  void load();
  uint64_t savedDisabled() const { return disabledIn(saved_, saved_length_); }
  void applySaved();
  void poll();   // from loop(): the slots' automatic attach, retries, liveness, the bound consoles

  const char *name() const override { return reg::probe_config::kName; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return reg::probe_config::kRevision; }
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::probe_config::kLockFreeOps, op); }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  struct Place { uint16_t wire_fn = 0; DebugPort *port = nullptr; TargetConsoleStream *console = nullptr; };
  struct Uart { uint16_t fn = 0; FixtureUart *uart = nullptr; };
  struct Slot {
    bool set = false;
    uint8_t place = 0, attach = 0, mechanism = 0, name_length = 0, lock_scheme = 0, lock_length = 0;
    uint32_t retry_ms = 0;
    uint16_t swdio = 0, swclk = 0;   // the slot's pair on its place's wire (fixed, or one the host chose)
    uint32_t max_hz = 0;           // the line's settings for the probe's own attach (oep-if-debug §3): the target's
    bool idle_low = false;
    char name[kMaxName + 1] = {};
    uint8_t mask[kMaxLock] = {}, value[kMaxLock] = {};
  };
  struct SlotRun {                 // what the probe keeps about a slot (not the setting)
    bool due = false;              // an automatic attach to make now (boot, the slot set)
    bool tried = false;
    uint64_t last_try_ns = kNeverNs;
    uint32_t last_try_ms = 0, last_check_ms = 0;
    bool mismatch = false, mismatch_has_tid = false;   // the last automatic attach found the lock not matching
    uint32_t mismatch_tid = 0;
  };
  struct Label { bool set = false; uint16_t channel = 0; };
  struct UartItem { bool set = false; uint32_t baud = 0; uint8_t format = 0; };
  // Everything the items say, derived from their canonical bytes and checked as a whole (derive).
  struct Derived {
    RoleAssignment roles[Endpoint::kMaxRoles];
    size_t role_count = 0;
    uint8_t idle[PinTable::kChannels];
    Slot slots[kMaxSlots];
    Binds::Spec binds[Binds::kMaxPorts];
    UartItem uarts[kMaxUarts];
    Label labels[kMaxLabels];
    uint64_t disabled = 0;         // the disable items' channels (under 64; a higher one names no pin here)
  };
  Endpoint &endpoint_;
  Binds &binds_;
  PinTable *pins_ = nullptr;
  Place places_[kMaxPlaces];
  size_t place_count_ = 0;
  Uart uarts_[kMaxUarts];
  size_t uart_count_ = 0;
  Slot slots_[kMaxSlots];          // several per place (each its own pair), at most one at boot per wire
  SlotRun runs_[kMaxSlots];
  bool onPair(const Slot &s) const {   // the place's link is on this slot's pair now
    const DebugPort &p = *places_[s.place].port;
    return p.swdio == s.swdio && p.swclk == s.swclk;
  }
  uint8_t idle_[PinTable::kChannels];             // the idle items (kIdleUnset: none)
  // the items as the host sent them, canonical order (tag, then key): what get pages, the hash covers and save keeps
  uint8_t items_[kMaxItems];
  size_t items_length_ = 0;
  uint8_t storage_state_ = reg::probe_config::kStorageStateNone;
  uint8_t unreadable_ = 0;                        // why the saved items were not applied (probe.config §3.3)
  uint32_t saved_hash_ = 0;                       // of the saved items after their fns were renumbered (0: unreadable)
  uint8_t saved_[kMaxItems];                      // the saved items (canonical, as saved)
  size_t saved_length_ = 0;
  uint8_t ids_[kMaxIds];                          // their interfaces: count, then fn(u16) instance(u16) revision name_len name
  size_t ids_length_ = 0;
  size_t identities(const uint8_t *items, size_t length, uint8_t *out, size_t capacity) const;

  // The item store: one item's key (the bytes after the tag that identify it) and the canonical order.
  static size_t keyLength(uint8_t tag);
  static uint64_t keyValue(uint8_t tag, const uint8_t *value, size_t length);
  static bool itemBefore(uint8_t tag_a, const uint8_t *a, size_t alen, uint8_t tag_b, const uint8_t *b, size_t blen);
  static bool insertItem(uint8_t *store, size_t &length, size_t capacity, uint8_t tag, const uint8_t *value, size_t vlen);
  static void removeItems(uint8_t *store, size_t &length, uint8_t tag, const uint8_t *key, size_t key_length);
  uint32_t hash() const { return crc32Of(items_, items_length_); }
  static uint64_t disabledIn(const uint8_t *items, size_t length);
  void setDisabled(uint64_t mask);   // the PinTable's and the endpoint's
  void applySavedItems();
  static uint32_t crc32Of(const uint8_t *data, size_t length);

  // One item's own checks (probe.config §1 / §2's table) - its shape and what this probe has; nothing of the whole.
  Result checkItem(uint8_t tag, const uint8_t *v, size_t len, uint8_t *out, size_t capacity) const;
  // The whole: every item decoded into `d`, the rules between items checked. completed() or the rejection.
  Result derive(const uint8_t *items, size_t length, Derived &d, uint8_t *out, size_t capacity) const;
  // A candidate store (the current items with a set's / unset's changes) checked, its plans reserved, then made current.
  Result commit(uint8_t *candidate, size_t length, const uint16_t *plan_fns, size_t plan_fn_count, uint8_t *out, size_t capacity);
  Result set(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result unset(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result state(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  size_t slotState(uint8_t i, uint8_t *out) const;
  bool sourceFor(const Slot *slots, uint8_t kind, uint16_t id, Binds::Source &out) const;
  bool bound(uint8_t slot) const;
  // 1 the lock matches (or none), 0 it does not, -1 there is a lock and no target_id to check
  int lockMatches(const Slot &s, bool has_tid, uint32_t tid) const;
  void dropSlot(uint8_t slot, uint8_t detail);   // its share of the connection and the console goes
  void runSlot(uint8_t slot);
  static size_t nameOf(void *self, uint8_t kind, uint16_t id, char *out, size_t room);
};

uint32_t crc32Ieee(const uint8_t *data, size_t length);

}  // namespace oep

#endif
