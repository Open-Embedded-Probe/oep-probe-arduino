// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 settings (oep-spec oep-core §13): values the probe keeps past a restart, gathered and saved by the core, each
// item declared by its owner - the core (fn 0) or an interface (its fn) - and read and written through fn 0's ops.
//
//   0x20 get(first u16) [TLV] -> more(u8) generation(u32) count(u8) count x value [TLV]            (no lock)
//        value: fn(u16) number(u8) key(u16) len(u16) data (no origin, core §13.4)
//   0x21 set(count u8, count x value) [TLV] -> generation(u32) [TLV]                  (a probe that lets values change)
//   0x22 unset(count u8, count x (fn u16, number u8, key u16)) [TLV] -> generation(u32) [TLV]
//   0x23 save [TLV] -> generation(u32) [TLV]                                         (with storage)
//   0x24 state [TLV] -> storage_state(u8) saved_generation(u32) reason(u8) [TLV]      (no lock; with storage)
//   0x25 network [TLV] -> tcp_port(u16) count(u8) count x 19-byte row [TLV]          (no lock; with Wi-Fi, transports §7.3)
//
// The items this probe knows (an entry each, in the order the sketch's calls register them; an item's number is its
// place among its owner's entries, from 1):
//   fn 0 (setPins)    channel.{}.label (text 1-32), channel.{}.idle (enum), channel.{}.disable (bool)        core §13.8
//   fn 0 (setWifi)    wifi.{}.ssid, wifi.{}.passphrase (secret), ipv4.{}.mode / address / prefix / gateway / dns  transports §7
//   a debug wire      rest.{} (rvswd), hold.{}                                                                debug §2, §3.1
//   a fixture uart    passthrough.{}.rx / baud / format                                                       fixture §2.1
//   a console         passthrough.{}.wire / swdio / swclk / max-speed / mechanism                             console §2.1
//   the sketch's      addSetting(owner, name, ...)                                                            core §13.1
// The values are kept as one store in (entry, key) order: tag = entry + 1, value = key(u16) data (the wire's data). The
// sketch's initial values (core §13.4) go into it before applySaved(), which keeps them only on a boot without a save.
// generation: 1 at boot, +1 for every set / unset that changes the contents (never 0; at 0xFFFFFFFF a change is refused
// cause 2 until a restart). Saved by name (core §13.7): the category (empty for the core, else the interface's name and
// instance), the item's name, the key's name and the value with its references as names - to NVS on an ESP32
// (Preferences "oepset" / "v1"), the flash's last sector on an RP2 (EEPROM, "OES1"); none on a CH32 yet.
#pragma once

#include <Arduino.h>
#include <string.h>

#include "Oep.h"
#include "OepBind.h"
#include "OepConsole.h"
#include "OepFixture.h"
#include "OepPinTable.h"
#include "OepTarget.h"

namespace oep {

// The items this library knows (its own numbers, never on the wire).
namespace setting_id {
enum : uint8_t {
  kLabel = 1, kIdle, kDisable, kSsid, kPassphrase, kIpMode, kIpAddress, kIpPrefix, kIpGateway, kIpDns,
  kRest, kHold, kPassRx, kPassBaud, kPassFormat, kConWire, kConSwdio, kConSwclk, kConSpeed, kConMechanism, kOwn,
};
}


}  // namespace oep

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_CH32RV) || defined(OEP_HOST_FAKE_CONFIG)

namespace oep {

class Endpoint;

// One Wi-Fi entry (transports §7.1) as WifiControl gets it: index is its key.
struct WifiEntry {
  uint16_t index = 0;
  uint8_t ssid_length = 0, pass_length = 0;
  char ssid[33] = {};
  char pass[65] = {};
  bool operator==(const WifiEntry &o) const {
    return index == o.index && ssid_length == o.ssid_length && pass_length == o.pass_length &&
           memcmp(ssid, o.ssid, ssid_length) == 0 && memcmp(pass, o.pass, pass_length) == 0;
  }
  bool operator!=(const WifiEntry &o) const { return !(*this == o); }
};
// The wifi netif's IPv4 (transports §7.2); DHCP without a mode.
struct Ipv4Setting {
  uint8_t mode = reg::core::kIpv4ModeDhcp, address[4] = {}, prefix = 0, gateway[4] = {}, dns[4] = {};
  bool operator==(const Ipv4Setting &o) const {
    return mode == o.mode && prefix == o.prefix && !memcmp(address, o.address, 4) && !memcmp(gateway, o.gateway, 4) &&
           !memcmp(dns, o.dns, 4);
  }
  bool operator!=(const Ipv4Setting &o) const { return !(*this == o); }
};
// What joins the networks (OepWifi.h on an ESP32): the settings hand it their list and the netif's IPv4, it reports the
// link (network's row, transports §7.3).
class WifiControl {
 public:
  enum : uint8_t {
    kStateOff = reg::core::kNetworkStateOff, kStateConnecting = reg::core::kNetworkStateConnecting,
    kStateConnected = reg::core::kNetworkStateUp, kStateWaiting = reg::core::kNetworkStateWaiting,
  };
  enum : uint8_t {
    kReasonNone = reg::core::kNetworkReasonNone, kReasonNotFound = reg::core::kNetworkReasonNotFound,
    kReasonAuth = reg::core::kNetworkReasonAuth, kReasonNoAddress = reg::core::kNetworkReasonNoAddress,
    kReasonOther = reg::core::kNetworkReasonOther,
  };
  static constexpr uint16_t kNoEntry = 0xFFFF;
  struct Status {
    uint8_t state = kStateOff;
    uint16_t entry = kNoEntry;
    uint8_t reason = kReasonNone;
    int8_t rssi = 0;
    uint8_t ipv4[4] = {}, prefix = 0, gateway[4] = {}, dns[4] = {};
  };
  // The networks in entry order and the IPv4; called after the set's answer is made, only when they changed. A change
  // to the entry in use or the IPv4 reconnects; other changes keep the link (transports §7).
  virtual void apply(const WifiEntry *entries, size_t count, const Ipv4Setting &ipv4) = 0;
  virtual Status status() const = 0;
  virtual uint16_t tcpPort() const { return 0; }   // the TCP port listened on now (0: none)
};

class Settings final : public SettingsCore {
 public:
  static constexpr size_t kMaxPlaces = 2, kMaxUarts = 4, kMaxPorts = Binds::kMaxPorts;
  static constexpr size_t kMaxWifi = 4;   // Wi-Fi entries (wifi.{}'s keys)
  static constexpr size_t kMaxOwn = 4;    // the sketch's own items (addSetting)
  static constexpr size_t kMaxEntries = OEP_SMALL_RAM ? 20 : 40;
  // The values' bytes this probe holds (every value with its 3-byte header and key): a change past it is refused cause 2.
  static constexpr size_t kMaxItems = OEP_SMALL_RAM ? 256 : 800;   // a small-RAM build: two of these

  // Attaches itself to the endpoint: fn 0's settings ops and the items in every fn's describe.
  Settings(Endpoint &endpoint, Binds &binds);
  // A wire and its console the console pass-through may name (after the endpoint has them): the console's items, the
  // wire's hold (and rest on an rvswd wire).
  bool addPlace(WireRvswd &wire, TargetConsoleStream &console);
  // A debug wire's hold after a close (oep-if-debug §2): its item hold.{} read through `hold`, and `check` called from
  // poll() to end a hold whose pins were taken or whose settings changed. addPlace does it for its wire; oep.wire.swd
  // (SwdPort) comes here: settings.addHold(swd, swd.hold, checkSwdHold, &swd).
  bool addHold(const Interface &wire, WireHold &hold, void (*check)(void *), void *port);
  // A fixture UART whose pass-through items the settings carry (after the endpoint has it).
  bool addUart(FixtureUart &uart);
  // The pins: fn 0's channel items (label, idle, disable).
  void setPins(PinTable *pins);
  // fn 0's channels (describe channels): the channel keys.
  void setChannels(uint16_t channels) { channels_ = channels; }
  // The Wi-Fi station: netif "wifi", its items and the network op. false (and nothing declared) when a transport's
  // max_frame is below wifi_min_max_frame (transports §7: the longest value must fit every transport).
  bool setWifi(WifiControl *wifi);
  // An item of the sketch's own under `owner` (an interface it added; core §13.1): its name (labels of a-z 0-9 - joined
  // by '.', one "{}" where a key goes), form (bytes, text, u32, ipv4, bool), keys (key_kind index: 1 or more; 0: no
  // key), max_len (bytes / text), flags (secret, restart). Before applySaved(). false: no room or not a valid one.
  bool addSetting(const Interface &owner, const char *name, uint8_t form, uint16_t keys = 0, uint8_t max_len = 0,
                  uint8_t flags = 0);
  // An initial value (core §13.4): put in on a boot without a save (every boot with no storage), checked as a set's
  // values are; after that like any other value. By the item's name and owner (nullptr: the core), after the item's
  // add* / setPins / setWifi and before applySaved(). false: no such item, not a value it takes, or no room.
  bool initialValue(const Interface *owner, const char *name, uint16_t key, const uint8_t *data, size_t length);
  bool initialLabel(uint16_t channel, const char *text) {   // a port's name: "V003.swio" (host guide §5.3)
    return initialValue(nullptr, reg::core::kSettingChannelLabel, channel, reinterpret_cast<const uint8_t *>(text),
                        strlen(text));
  }
  // A probe of initial values only (core §13.5): no set / unset (nor save / state) - its values never change.
  void setChangeable(bool on) { changeable_ = on; }
  // An own item's value now (nullptr: none): its data and length.
  const uint8_t *setting(const Interface &owner, const char *name, uint16_t key, size_t &length) const;
  // load() reads the saved channel items early (savedDisabled: the disabled channels, for a sketch that parks its pins
  // first); applySaved() reads the save again and applies every value it can (core §13.7): after the sketch's last
  // endpoint.add() and every add* here.
  void load();
  uint64_t savedDisabled() const { return saved_disabled_; }
  void applySaved();
  void poll();   // from loop(): the pass-throughs, the holds
  // The pass-throughs (fixture §2.1, console §2.1: a uart's or a console's bytes to a serial port's raw stream, binds):
  // declared and run once this is called - a probe with a serial transport; one without links none of it.
  void usePassthroughs() { pass_run_ = &Settings::runPassthroughs; }
  void setAttachGate(bool (*ready)()) { attach_gate_ = ready; }   // a console pass-through's attach waits on it
  void skipBootAttach() { skip_boot_attach_ = true; }
  bool bootAttachSkipped() const { return skip_boot_attach_; }
  // A probe that saves nothing (before the first poll): no save / state (core §13.7).
  void setStorage(bool on) { storage_ = on; }

  bool offers(uint8_t op) const override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  void describe(uint16_t fn, TlvPager &put) const override;
  uint32_t generation() const { return generation_; }
  // The value now of an item (nullptr: none): its data and length.
  const uint8_t *value(uint8_t entry, uint16_t key, size_t &length) const;
  // The entry of an item (kMaxEntries: none) by id and owner (nullptr: the core).
  uint8_t entryOf(uint8_t id, const void *owner = nullptr) const;
  // A test's starting point: the generation (0xFFFFFFFF refuses every change).
  void testGeneration(uint32_t g) { generation_ = g; }
  // (fn, number) of an entry this boot: a test's way to address an item.
  uint16_t fnOfEntry(uint8_t entry) const;
  uint8_t numberOfEntry(uint8_t entry) const;

 private:
  struct Entry {
    const Interface *owner;   // nullptr: the core
    void *context;            // the uart, the place, the hold site
    uint8_t id, own;          // own: own_[own] for kOwn
  };
  struct Place { WireRvswd *wire = nullptr; TargetConsoleStream *console = nullptr; };
  struct Own { const char *name = nullptr; uint8_t form = 0, max_len = 0, flags = 0; uint16_t keys = 0; };
  struct HoldSite { const Interface *wire = nullptr; void (*check)(void *) = nullptr; void *port = nullptr; };
  struct Spec { const char *name; uint8_t form, key_kind, max_len, flags; uint16_t keys; };
  static constexpr size_t kMaxHolds = kMaxPlaces + 2;

  Endpoint &endpoint_;
  Binds &binds_;
  PinTable *pins_ = nullptr;
  uint16_t channels_ = PinTable::kChannels;
  Entry entries_[kMaxEntries];
  uint8_t entry_count_ = 0;
  Place places_[kMaxPlaces];
  size_t place_count_ = 0;
  HoldSite holds_[kMaxHolds];
  size_t hold_count_ = 0;
  FixtureUart *uarts_[kMaxUarts] = {};
  size_t uart_count_ = 0;
  Own own_[kMaxOwn];
  size_t own_count_ = 0;
  WifiControl *wifi_ = nullptr;
  void (Settings::*wifi_hand_)() = nullptr;   // handWifi, once setWifi had one
  Result (Settings::*wifi_net_)(const uint8_t *, size_t, uint8_t *, size_t) = nullptr;   // network, likewise
  void (Settings::*pass_run_)() = nullptr;   // runPassthroughs, once usePassthroughs was called
  uint32_t wifi_applied_ = 0;    // CRC-32 of the Wi-Fi values last handed to wifi_
  bool wifi_handed_ = false;
  bool storage_ = true, skip_boot_attach_ = false, changeable_ = true;
  bool (*attach_gate_)() = nullptr;
  uint8_t items_[kMaxItems];     // the user values, in (entry, key) order
  size_t items_length_ = 0;
  static uint8_t scratch_[kMaxItems];   // a change's candidate store
  uint32_t generation_ = 1;
  uint8_t storage_state_ = reg::core::kStorageStateNone, reason_ = 0;
  uint32_t saved_generation_ = 0;
  uint32_t saved_crc_ = 0;       // the save as it is (its blob's CRC and length): a save of the same is not written
  size_t saved_size_ = 0;
  uint64_t saved_disabled_ = 0;
  int8_t pass_uart_[kMaxPorts];     // the uarts_ index passing to the port now, -1: none
  int8_t pass_console_[kMaxPorts];  // the places_ index whose console passes to the port now, -1: none
  bool console_tried_[kMaxPorts] = {};

  bool addEntry(uint8_t id, const Interface *owner, void *context, uint8_t own = 0);
  Spec spec(uint8_t entry) const;
  uint16_t ownerFn(uint8_t entry) const;
  uint8_t entryAt(uint16_t fn, uint8_t number) const;   // kMaxEntries: none
  uint8_t entryNamed(const Interface *owner, const char *name, size_t length) const;
  bool keyValid(uint8_t entry, uint16_t key) const;
  // the store helpers (tag = entry + 1, value = key(u16) data)
  static const uint8_t *find(const uint8_t *store, size_t length, uint8_t entry, uint16_t key, size_t &vlen);
  static bool insert(uint8_t *store, size_t &length, size_t capacity, uint8_t entry, const uint8_t *v, size_t len);
  static void remove(uint8_t *store, size_t &length, uint8_t entry, uint16_t key);
  // one value by itself: its form (malformed), then what this probe takes (unsupported)
  Result checkValue(uint8_t entry, uint16_t key, const uint8_t *d, size_t n, uint8_t item, uint8_t *out, size_t capacity) const;
  Result checkWhole(const uint8_t *items, size_t length, uint8_t *out, size_t capacity) const;
  uint8_t idlePlainOf(const uint8_t *items, size_t length, uint16_t channel) const;   // PinTable::kIdleUnset: none
  uint64_t disabledIn(const uint8_t *items, size_t length) const;
  uint16_t labelClash(const uint8_t *items, size_t length) const;   // 0xffff: the labels are unique (core §13.8)
  const uint8_t *in(const uint8_t *items, size_t length, uint8_t id, const void *owner, uint16_t key, size_t &n) const;
  // The place of console `owner` (the front, common §2) on the wire of fn `fn`; -1: none of its places
  int placeOf(const Interface *owner, uint16_t fn) const;
  // places_[k] is the first of its console's places (the pass-through items are the console's, once)
  bool firstOf(size_t k) const;
  void adopt(const uint8_t *items, size_t length);
  void handWifi();
  Result commit(const uint8_t *candidate, size_t length, uint8_t *out, size_t capacity);
  Result get(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result set(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result unset(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result save(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result network(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  // names: a key's and a value's (core §13.3), both ways
  size_t keyName(uint8_t entry, uint16_t key, char *out) const;
  bool keyNamed(uint8_t entry, const uint8_t *name, size_t length, uint16_t &key) const;
  size_t routeName(uint16_t index, char *out) const;
  size_t storedValue(uint8_t entry, const uint8_t *d, size_t n, uint8_t *out) const;      // the save's form
  bool wireValue(uint8_t entry, const uint8_t *s, size_t n, uint8_t *out, size_t &len) const;   // back to the wire's
  size_t saveBlob(uint8_t *blob, size_t capacity) const;
  void readBlob(bool all);
  void runPassthroughs();
  void runPassthroughsYield(uint8_t port);
  static void passIn(void *self, const uint8_t *data, size_t length);
  static uint8_t restOf(const void *self, uint16_t swclk);   // rest.{} of that channel (auto when none)
  static bool holdOf(const void *self, uint16_t swdio);      // hold.{} of that channel (on when none)
};

uint32_t crc32Ieee(const uint8_t *data, size_t length);
#if defined(OEP_HOST_FAKE_CONFIG)
void fakeConfigStore(const uint8_t *blob, size_t length);   // a host test's saved blob (the save, then its CRC-32)
#endif

}  // namespace oep

#endif
