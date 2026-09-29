// OEP v1 oep.probe.config (oep-spec docs/oep-if-probe-config.ja.md, revision 1): the probe's settings, set by the host
// and kept only when the host saves them. No modes, no reboot: everything set takes effect at once.
//
//   0x01 get(first u16) -> more(u8) hash(u32) items      (no lock)
//   0x02 set(items) -> hash(u32)    0x03 save -> hash(u32)    0x04 erase
//
// Items (TLV, tag u8 len u8 value), each with a key; a set replaces the keys it carries and leaves the others:
//   0x01 plan fn(u16) role(u8) channel(u16)     key fn: the items of a fn are that fn's plan (fn alone clears)
//   0x03 idle channel(u16) mode(u8)             key channel: 0 Hi-Z, 1 pull-up, 2 pull-down while free
//   0x04 slot slot(u8) wire_fn(u16) swdio(u16) swclk(u16) attach(u8) retry_s(u16) mechanism(u8) name_len(u8) name
//        lock_scheme(u8) [mask(n) value(n)]     key slot: a place a target is wired to (slot alone clears)
//   0x05 bind port(u8) mode(u8) selected(u8) n(u8) n x (kind(u8) id(u16))   key port: what a serial port carries
// label (0x02) is not taken (unsupported). No defaults: nothing the host did not set is done. Saved to NVS on ESP32
// (Preferences, namespace "oepcfg") with the identity of the interface list; a saved copy made for another list is
// not applied.
//
// The places a slot may name are the sketch's wires with their consoles (addPlace: one fixed pin pair each, one
// connection each), the streams a bind may carry are those places' consoles and the fixture UARTs added (addUart).
#pragma once

#include <Arduino.h>

#include "OepPinTable.h"
#include "Oep.h"
#include "OepBind.h"
#include "OepConsole.h"
#include "OepFixture.h"
#include "OepTarget.h"

#if defined(ARDUINO_ARCH_ESP32)

namespace oep {

class Endpoint;

class ProbeConfig final : public Interface {
 public:
  static constexpr size_t kMaxPlaces = 2, kMaxUarts = 4, kMaxSaved = 512, kMaxLock = 8, kMaxName = 32;

  ProbeConfig(Endpoint &endpoint, Binds &binds) : endpoint_(endpoint), binds_(binds) {
    memset(idle_, PinTable::kIdleUnset, sizeof idle_);
    binds_.setNamer(&ProbeConfig::nameOf, this);
  }
  // A place a slot may name: this wire (its fixed pair) and its console. Add after the endpoint has them.
  bool addPlace(WireRvswd &wire, TargetConsoleStream &console);
  // A fixture UART a bind may carry (after the endpoint has it).
  bool addUart(FixtureUart &uart);
  // The pins whose idle state the idle item sets (without it, idle items are refused).
  void setPins(PinTable *pins) { pins_ = pins; }

  // Before the interfaces are used: read what was saved, then applySaved() once they are all added.
  void load();
  void applySaved();
  void poll();   // from loop(): the slots' automatic attach, retries, the bound consoles

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
    uint16_t retry_s = 0;
    char name[kMaxName + 1] = {};
    uint8_t mask[kMaxLock] = {}, value[kMaxLock] = {};
  };
  struct SlotRun {                 // what the probe keeps about a slot (not the setting)
    bool due = false;              // an automatic attach to make now (boot, the slot set)
    bool tried = false;
    uint32_t last_try_ms = 0;
    bool mismatch = false, mismatch_has_tid = false;   // the last automatic attach found the lock not matching
    uint32_t mismatch_tid = 0;
  };
  Endpoint &endpoint_;
  Binds &binds_;
  PinTable *pins_ = nullptr;
  Place places_[kMaxPlaces];
  size_t place_count_ = 0;
  Uart uarts_[kMaxUarts];
  size_t uart_count_ = 0;
  Slot slots_[kMaxPlaces];         // one slot per place at most
  SlotRun runs_[kMaxPlaces];
  uint8_t idle_[PinTable::kChannels];             // the idle items (kIdleUnset: none)
  uint8_t storage_state_ = reg::probe_config::kStorageStateNone;
  uint32_t saved_hash_ = 0, save_ms_ = 0, saved_list_ = 0;
  uint8_t saved_[kMaxSaved];
  size_t saved_length_ = 0;

  size_t canonical(uint8_t *out, size_t capacity) const;   // the items in their canonical order (the hash's input)
  uint32_t hash() const;
  Result apply(const uint8_t *items, size_t length);
  bool sourceFor(const Slot *slots, uint8_t kind, uint16_t id, Binds::Source &out) const;
  bool bound(uint8_t slot) const;
  // 1 the lock matches (or none), 0 it does not, -1 there is a lock and no target_id to check
  int lockMatches(const Slot &s, bool has_tid, uint32_t tid) const;
  void runSlot(uint8_t slot);
  static size_t nameOf(void *self, uint8_t kind, uint16_t id, char *out, size_t room);
};

uint32_t crc32Ieee(const uint8_t *data, size_t length);

}  // namespace oep

#endif
