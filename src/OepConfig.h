// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 oep.probe.config (oep-spec interfaces/oep-if-probe-config.ja.md, revision 1): the probe's settings, set by
// the host and kept only when the host saves them. No modes, no reboot: everything set takes effect at once.
//
//   0x01 get(first u16) -> more(u8) hash(u32) items      (no lock)
//   0x02 set(items) -> hash(u32)      0x03 save -> hash(u32)      0x04 erase
//   0x05 unset(n u8, n x (len u8, tag u8, key; len the key's length)) -> hash(u32)
//   0x06 state(first_slot u8, first_bind u8) -> more storage_state storage_hash unreadable_reason
//        n_slots x slot_state n_binds x bind_state                         (no lock)
//
// Items (TLV), each with a key; a set replaces the keys it carries and leaves the others, unset removes keys:
//   0x01 plan  fn(u16) role(u8) channel(u16)            key (fn, role, channel); a set replaces the whole plan of the fn
//   0x02 label channel(u16) text (1-32 bytes)           key channel (read back with get; fn 0's describe has only
//                                                        the firmware's fixed labels)
//   0x03 idle  channel(u16) mode(u8) drive(u8)           key channel: 0 Hi-Z, 1 pull-up, 2 pull-down, 3 output low,
//                                                        4 output high while free; drive (a level of fixture §1.1's
//                                                        drive_levels, 0xFF the default) only for 3 / 4
//   0x04 slot  slot(u8) wire_fn(u16) swdio(u16) swclk(u16) attach(u8) retry_ms(u32) max_speed_hz(u32) idle_clock(u8)
//              mechanism(u8: 0xFF none) name_len(u8) name                                             key slot
//   0x05 bind  port(u8) kind(u8) id(u16): the one stream the serial port carries                    key port
//   0x06 uart  fn(u16) baud(u32) format(u8)              key fn: a fixture UART's settings, in force when its plan has pins
//   0x07 disable channel(u16)                            key channel: never used, driven or configured (not on this
//                                                        board): every request naming it is unavailable cause 5, and
//                                                        the pin is never parked; describe still offers it
//   0x08 wifi  index(u8) ssid_len(u8) ssid pass_len(u8) passphrase   key index (probe.config §1.4). A network the
//              probe joins to serve OEP over TCP: entries tried in index order, every one (a hidden SSID shows in
//              no scan), the first that connects is kept, and the list is tried again after the link goes. index
//              below wifi_max (else unsupported with the tag as received); ssid 1-32 bytes (no 0x00 here:
//              unsupported); passphrase none (pass_len 0, an open network), 8-63 bytes 0x20-0x7E or 64 hex digits,
//              else malformed. The passphrase is write-only: get carries pass_len 0xFF and no passphrase for an entry
//              that has one (0 for none); a set with pass_len 0xFF keeps the passphrase the entry has (malformed when
//              there is no such entry). The hash never covers the passphrase itself (a random token that changes when
//              one does). Up to kMaxWifi entries (describe wifi_max 0x46); a set / unset that changes or removes the
//              entry in use drops the link after its answer, other changes keep it (OepWifi.h); state carries the
//              link (TLV 0x01 wifi): state(u8) entry(u8) reason(u8) rssi(i8) ipv4(4), rssi and ipv4 0 unless state 2.
// Every item has one form (probe.config §1): a value of any other length is malformed, critical or not. The probe
// keeps every item's bytes as the host sent them (the critical bit cleared), ordered by tag, then by key compared as
// numbers - the order get pages them in. The hash is the probe's own: CRC-32 of those bytes (a host never computes it).
// Saved to NVS on ESP32 (Preferences "oepcfg" / "items6") or the flash's last sector on RP2040 / RP2350 (EEPROM, "OEP6"),
// with the identity of every interface the items name; a saved copy naming an interface that is gone is not applied
// (state says why), and one of a form before ("items5" / "OEP5") reads as unreadable reason 1.
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

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_CONFIG)

namespace oep {

class Endpoint;

// The wifi item (probe.config §1.4): the registry's numbers under this file's names.
constexpr uint8_t kWifiItemTag = reg::probe_config::kTlvItemWifi;            // index ssid_len ssid pass_len passphrase
constexpr uint8_t kWifiDescribeMax = reg::probe_config::kTlvDescribeWifiMax; // describe: wifi_max(u8)
constexpr uint8_t kWifiStateTlv = reg::probe_config::kTlvStateAnswerWifi;    // state: state entry reason rssi ipv4
constexpr uint8_t kWifiPassHidden = reg::probe_config::kWifiPassLenHidden;   // get: has a passphrase; set: keep it

// One network the probe may join (the wifi item, decoded).
struct WifiEntry {
  uint8_t index = 0, ssid_length = 0, pass_length = 0;
  char ssid[33] = {};
  char pass[65] = {};
  bool operator==(const WifiEntry &o) const {
    return index == o.index && ssid_length == o.ssid_length && pass_length == o.pass_length &&
           memcmp(ssid, o.ssid, ssid_length) == 0 && memcmp(pass, o.pass, pass_length) == 0;
  }
  bool operator!=(const WifiEntry &o) const { return !(*this == o); }
};

// What joins the networks (OepWifi.h on an ESP32): the settings hand it their list, it reports the link.
class WifiControl {
 public:
  enum : uint8_t {
    kStateOff = reg::probe_config::kWifiStateOff, kStateConnecting = reg::probe_config::kWifiStateConnecting,
    kStateConnected = reg::probe_config::kWifiStateConnected, kStateWaiting = reg::probe_config::kWifiStateWaiting,
  };
  enum : uint8_t {
    kReasonNone = reg::probe_config::kWifiReasonNone, kReasonNotFound = reg::probe_config::kWifiReasonNotFound,
    kReasonAuth = reg::probe_config::kWifiReasonAuth, kReasonNoAddress = reg::probe_config::kWifiReasonNoAddress,
    kReasonOther = reg::probe_config::kWifiReasonOther,
  };
  static constexpr uint8_t kNoEntry = reg::probe_config::kWifiEntryNone;
  struct Status {
    uint8_t state = kStateOff, entry = kNoEntry, reason = kReasonNone;
    int8_t rssi = 0;          // dBm while connected (0: not known)
    uint8_t ipv4[4] = {};
  };
  // The whole list, in index order (count 0: Wi-Fi off). Called when it changes; the change may take effect a moment
  // later (the answer that set it goes out first).
  virtual void apply(const WifiEntry *entries, size_t count) = 0;
  virtual Status status() const = 0;
};

class ProbeConfig final : public Interface {
 public:
  static constexpr size_t kMaxPlaces = 2, kMaxSlots = 4, kMaxUarts = 4, kMaxName = 32;
  // The items' TLV bytes this probe holds and saves: describe storage max_bytes. The identity table saved with
  // them has its own room (kMaxIds), so a configuration of kMaxItems bytes always fits (probe.config §2).
  // (kMaxItems: 384 for the other items, and four wifi entries of the longest form, 102 bytes each with the header)
  static constexpr size_t kMaxItems = 800, kMaxIds = 320;
  static constexpr size_t kMaxWifi = 4;   // wifi entries (describe wifi_max)
  static constexpr size_t kMaxLabels = 8;

  ProbeConfig(Endpoint &endpoint, Binds &binds) : endpoint_(endpoint), binds_(binds) {
    memset(idle_, PinTable::kIdleUnset, sizeof idle_);
    memset(idle_drive_, PinTable::kDriveDefault, sizeof idle_drive_);
  }
  // A place a slot may name: this wire and its console. Add after the endpoint has them.
  bool addPlace(WireRvswd &wire, TargetConsoleStream &console);
  // A fixture UART a bind may carry and the uart item sets (after the endpoint has it).
  bool addUart(FixtureUart &uart);
  // The pins whose idle state the idle item sets and the disable item takes away (without it, both are refused).
  void setPins(PinTable *pins) { pins_ = pins; endpoint_.setPins(pins); }   // the endpoint's plan replacements too
  // The networks the wifi item lists go to `wifi` (before load(); without one the item is not declared).
  void setWifi(WifiControl *wifi) { wifi_ = wifi; }

  // Read what was saved, then apply it: both after the sketch's last endpoint.add(). The saved items keep the
  // (name, instance, revision) of every interface they name and are renumbered to where those are now; one gone (or of
  // another revision) leaves them unapplied (storage unreadable, probe.config §2).
  // load() only reads the storage (no interface is needed): a sketch that parks its free pins at start-up loads first
  // and leaves the saved disable items' channels alone (savedDisabled, probe.config §2: disable and idle apply before
  // every other item). applySaved then puts the idle states before the plans, uarts, slots and binds, and sets the
  // PinTable's disabled channels from what was applied (none when it was not, those pins then going to their idle state).
  void load();
  uint64_t savedDisabled() const { return disabledIn(saved_, saved_length_); }
  void applySaved();
  void poll();   // from loop(): the slots' automatic attach, retries, liveness, the bound consoles
  // The at-boot slots' automatic attach (and its retries) waits while `ready` answers false (nullptr: never waits;
  // probe.config §3.1: the probe attaches them when it can try). A probe whose transport is its own USB device holds its
  // wires' bit-banging back until the host has configured the device (BootGuard::attachReady): a console polling the
  // target from boot delays and batches the USB interrupts while the host enumerates it. Nothing else waits: disables,
  // idles, plans and uarts are applied at once. Untried, a slot's state reads 1 with last_try_at_ns all ones (§3.3).
  void setAttachGate(bool (*ready)()) { attach_gate_ = ready; }
  // Before applySaved: the saved at-boot slots are not attached by the probe in this boot (a boot after repeated crashes,
  // BootGuard::safe). Their state reads 1 (not there) with last_try_at_ns all ones (never tried, §3.3); a set of the slot,
  // or the host's own attach, connects it as usual.
  void skipBootAttach() { skip_boot_attach_ = true; }
  bool bootAttachSkipped() const { return skip_boot_attach_; }
  const char *name() const override { return reg::probe_config::kName; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return reg::probe_config::kRevision; }
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::probe_config::kLockFreeOps, op); }
  // get, set, unset, state always; save and erase (a pair) with storage, the describe's storage tag then present
  // (probe.config §2, core §1.2)
  bool offers(uint8_t op) const override {
    return opIn(op, reg::probe_config::kOpGet, reg::probe_config::kOpState) &&
           (storage_ || (op != reg::probe_config::kOpSave && op != reg::probe_config::kOpErase));
  }
  // A probe that saves nothing (setStorage(false), before the first poll): no save / erase, no storage tag.
  void setStorage(bool on) { storage_ = on; }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  struct Place { uint16_t wire_fn = 0; DebugPort *port = nullptr; TargetConsoleStream *console = nullptr; };
  struct Uart { uint16_t fn = 0; FixtureUart *uart = nullptr; };
  struct Slot {
    bool set = false;
    uint8_t place = 0, attach = 0, mechanism = 0, name_length = 0;
    uint32_t retry_ms = 0;
    uint16_t swdio = 0, swclk = 0;   // the slot's pair on its place's wire (fixed, or one the host chose)
    uint32_t max_hz = 0;           // the line's settings for the probe's own attach (oep-if-debug §3): the target's
    bool idle_low = false;
    char name[kMaxName + 1] = {};
  };
  struct SlotRun {                 // what the probe keeps about a slot (not the setting)
    bool due = false;              // an automatic attach to make now (boot, the slot set)
    bool tried = false;
    uint64_t last_try_ns = kNeverNs;
    uint32_t last_try_ms = 0, last_check_ms = 0;
  };
  struct Label { bool set = false; uint16_t channel = 0; };
  struct UartItem { bool set = false; uint32_t baud = 0; uint8_t format = 0; };
  // Everything the items say, derived from the stored items and checked as a whole (derive).
  struct Derived {
    RoleAssignment roles[Endpoint::kMaxRoles];
    size_t role_count = 0;
    uint8_t idle[PinTable::kChannels];
    uint8_t idle_drive[PinTable::kChannels];   // an output idle's level (kDriveDefault: the default; inputs always)
    Slot slots[kMaxSlots];
    Binds::Spec binds[Binds::kMaxPorts];
    UartItem uarts[kMaxUarts];
    Label labels[kMaxLabels];
    uint64_t disabled = 0;         // the disable items' channels (under 64; a higher one names no pin here)
    WifiEntry wifi[kMaxWifi];      // the wifi items, in index order
    size_t wifi_count = 0;
  };
  WifiControl *wifi_ = nullptr;
  WifiEntry wifi_entries_[kMaxWifi];   // what wifi_ was given
  size_t wifi_count_ = 0;
  uint32_t wifi_token_ = 0;            // stands for the passphrases in the hash; new whenever one changes
  // An item as get shows it: a wifi item without its passphrase (pass_len 0xFF when it has one). Returns its TLV
  // length; out nullptr: only the length.
  static size_t shown(uint8_t tag, const uint8_t *v, size_t vlen, uint8_t *out);
  Endpoint &endpoint_;
  Binds &binds_;
  PinTable *pins_ = nullptr;
  Place places_[kMaxPlaces];
  size_t place_count_ = 0;
  Uart uarts_[kMaxUarts];
  size_t uart_count_ = 0;
  Slot slots_[kMaxSlots];          // several per place (each its own pair), at most one at boot per wire
  SlotRun runs_[kMaxSlots];
  bool (*attach_gate_)() = nullptr;
  bool skip_boot_attach_ = false, applying_saved_ = false;
  bool onPair(const Slot &s) const {   // the place's link is on this slot's pair now
    const DebugPort &p = *places_[s.place].port;
    return p.swdio == s.swdio && p.swclk == s.swclk;
  }
  uint8_t idle_[PinTable::kChannels];             // the idle items (kIdleUnset: none)
  uint8_t idle_drive_[PinTable::kChannels];       // their strengths (kDriveDefault: none)
  // the items as the host sent them, ordered by tag, then key: what get pages, the hash covers and save keeps
  uint8_t items_[kMaxItems];
  size_t items_length_ = 0;
  uint8_t storage_state_ = reg::probe_config::kStorageStateNone;
  uint8_t unreadable_ = 0;                        // why the saved items were not applied (probe.config §3.3)
  uint32_t saved_hash_ = 0;                       // of the saved items after their fns were renumbered (0: unreadable)
  uint8_t saved_[kMaxItems];                      // the saved items (in get's order, as saved)
  size_t saved_length_ = 0;
  uint8_t ids_[kMaxIds];                          // their interfaces: count, then fn(u16) instance(u16) revision name_len name
  size_t ids_length_ = 0;
  size_t identities(const uint8_t *items, size_t length, uint8_t *out, size_t capacity) const;

  // The item store: one item's key (the bytes after the tag that identify it) and the order of get.
  static size_t keyLength(uint8_t tag);
  static bool formLength(uint8_t tag, const uint8_t *v, size_t len);
  bool storage_ = true;
  bool declares(uint8_t tag) const;
  static uint64_t keyValue(uint8_t tag, const uint8_t *value, size_t length);
  static bool itemBefore(uint8_t tag_a, const uint8_t *a, size_t alen, uint8_t tag_b, const uint8_t *b, size_t blen);
  static bool insertItem(uint8_t *store, size_t &length, size_t capacity, uint8_t tag, const uint8_t *value, size_t vlen);
  static void removeItems(uint8_t *store, size_t &length, uint8_t tag, const uint8_t *key, size_t key_length);
  uint32_t hash() const;   // the probe's own choice (probe.config §2): CRC-32 of the items as get shows them, the token
  DriveLevels driveLevels() const;
  static uint64_t disabledIn(const uint8_t *items, size_t length);
  void setDisabled(uint64_t mask);   // the PinTable's and the endpoint's
  void applySavedItems();
  static uint32_t crc32Of(const uint8_t *data, size_t length);

  // One item's own checks (probe.config §1) - its length and what this probe has; nothing of the whole.
  Result checkItem(uint8_t raw, const uint8_t *v, size_t len, uint8_t *out, size_t capacity) const;
  // The whole: every item decoded into `d`, the rules between items checked. completed() or the rejection.
  Result derive(const uint8_t *items, size_t length, Derived &d, uint8_t *out, size_t capacity) const;
  // A candidate store (the current items with a set's / unset's changes) checked, its plans reserved, then made current.
  Result commit(uint8_t *candidate, size_t length, const uint16_t *plan_fns, size_t plan_fn_count, uint8_t *out, size_t capacity,
                uint8_t plan_raw = reg::probe_config::kTlvItemPlan);
  Result set(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result unset(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result state(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  size_t slotState(uint8_t i, uint8_t *out) const;
  bool sourceFor(const Slot *slots, uint8_t kind, uint16_t id, Binds::Source &out) const;
  bool bound(uint8_t slot) const;
  void dropSlot(uint8_t slot, uint8_t detail);   // its share of the connection and the console goes
  void runSlot(uint8_t slot);
};

uint32_t crc32Ieee(const uint8_t *data, size_t length);

}  // namespace oep

#endif

namespace oep {

// The label convention (oep-if-probe-config §1.3: the host's way to find a line; the probe drives none of these lines
// itself) over a store of items (TLVs as probe.config keeps them) and the firmware's labels (`firmware`: fn 0's describe TLVs, its label 0x46): the channel of line `line` (nrst, power_hi,
// power_lo) for the slot named `slot` - (a) the settings label whose text is "slot.line", else, only when the items hold
// at most one slot item, (b) the settings label whose text is "line", else (c) the firmware label "line"; slot nullptr
// (settings without slots): steps (b) and (c). Texts compared ignoring ASCII case; the first step that finds a channel
// ends the search, with none when it finds two or more. 0xFFFF: none.
uint16_t findLine(const uint8_t *items, size_t length, const char *slot, const char *line,
                  const uint8_t *firmware = nullptr, size_t firmware_length = 0);

}  // namespace oep
