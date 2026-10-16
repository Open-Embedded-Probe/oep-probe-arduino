// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 oep.probe.config (oep-spec interfaces/oep-if-probe-config.ja.md, revision 1): the probe's settings - a
// channel's disable, idle and label, a serial port's pass-through, the Wi-Fi networks and the IPv4 of the network, and
// the sketch's own settings - set by the host, kept until a restart, and saved only when the host saves them.
//
//   0x10 get(first u16) -> more(u8) generation(u32) count(u8) items [TLV]                       (no lock)
//   0x11 set(count u8, items) [TLV] -> generation(u32) [TLV]
//   0x12 save [TLV] -> generation(u32) [TLV]      0x13 erase [TLV]                (with storage)
//   0x14 unset(count u8, count x (number u8, key u16)) [TLV] -> generation(u32) [TLV]
//   0x15 state [TLV] -> storage_state(u8) saved_generation(u32) reason(u8) [TLV]   (no lock; with storage)
//   0x16 network [TLV] -> tcp_port(u16) count(u8) count x 19-byte row [TLV]        (no lock; with Wi-Fi)
//
// Items are named (§1): an item is number(u8) len(u16) key(u16) data on the wire, the number being the item's place in
// describe's setting list (from 1, for this boot). This probe holds them by kind - its own fixed order: label, idle,
// disable, passthrough, console-passthrough, wifi, ipv4, then the sketch's own settings (addSetting) - which is also
// the list's order, so get's (number, key) order is the store's (kind, key) order. The store is kind(u8) len(u16)
// key(u16) data, the wire's form with the kind for the number:
//   label   channel  text (1-32 bytes)                         idle  channel  mode(u8)        disable  channel
//   passthrough          port  fn(u16) rx(u16) baud(u32) format(u8)                                     (§1.1)
//   console-passthrough  port  wire_fn(u16) swdio(u16) swclk(u16) max_speed(u32) console_fn(u16) mechanism(u8) (§1.2)
//   wifi  entry  ssid_len ssid pass_len passphrase (the store keeps the passphrase; get shows pass_len only, §1.4)
//   ipv4  netif  mode address(4) prefix gateway(4) dns(4)                                                (§1.5)
// generation: 1 at boot, +1 for every set / unset that changes the contents (never 0; at 0xFFFFFFFF a change is
// refused cause 2 until a restart). Saved by name (§1: a firmware whose list moved reads them back by name) to NVS on
// an ESP32 (Preferences "oepcfg" / "items8"), the flash's last sector on an RP2 (EEPROM, "OEP8"); a save of the forms
// before reads as unreadable (reason 1).
#pragma once

#include <Arduino.h>
#include <string.h>

#include "OepPinTable.h"
#include "Oep.h"
#include "OepBind.h"
#include "OepConsole.h"
#include "OepFixture.h"
#include "OepTarget.h"

namespace oep {
// The kinds the store holds (the probe's own numbers, never on the wire; see above). findLine reads labels by kLabel.
namespace config_kind {
enum : uint8_t { kLabel = 1, kIdle, kDisable, kPass, kConsolePass, kWifi, kIpv4, kOwn };
}

}  // namespace oep

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_CONFIG)

namespace oep {

class Endpoint;

// One wifi item (probe.config §1.4) as WifiControl gets it: index is its key.
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
// The wifi netif's ipv4 item (probe.config §1.5); mode DHCP (all zero) without one.
struct Ipv4Setting {
  uint8_t mode = reg::probe_config::kIpv4ModeDhcp, address[4] = {}, prefix = 0, gateway[4] = {}, dns[4] = {};
  bool operator==(const Ipv4Setting &o) const {
    return mode == o.mode && prefix == o.prefix && !memcmp(address, o.address, 4) && !memcmp(gateway, o.gateway, 4) &&
           !memcmp(dns, o.dns, 4);
  }
  bool operator!=(const Ipv4Setting &o) const { return !(*this == o); }
};
// What joins the networks (OepWifi.h on an ESP32): the settings hand it their list and the netif's IPv4, it reports the
// link (the network op's row, probe.config §2).
class WifiControl {
 public:
  enum : uint8_t {
    kStateOff = reg::probe_config::kNetworkStateOff, kStateConnecting = reg::probe_config::kNetworkStateConnecting,
    kStateConnected = reg::probe_config::kNetworkStateUp, kStateWaiting = reg::probe_config::kNetworkStateWaiting,
  };
  enum : uint8_t {
    kReasonNone = reg::probe_config::kNetworkReasonNone, kReasonNotFound = reg::probe_config::kNetworkReasonNotFound,
    kReasonAuth = reg::probe_config::kNetworkReasonAuth, kReasonNoAddress = reg::probe_config::kNetworkReasonNoAddress,
    kReasonOther = reg::probe_config::kNetworkReasonOther,
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
  // to the entry in use or the IPv4 reconnects; other changes keep the link (§1.3).
  virtual void apply(const WifiEntry *entries, size_t count, const Ipv4Setting &ipv4) = 0;
  virtual Status status() const = 0;
  virtual uint16_t tcpPort() const { return 0; }   // the TCP port listened on now (0: none)
};

class ProbeConfig final : public Interface {
 public:
  static constexpr size_t kMaxPlaces = 2, kMaxUarts = 4, kMaxPorts = Binds::kMaxPorts;
  static constexpr size_t kMaxWifi = 4;   // wifi entries (the wifi setting's keys)
  static constexpr size_t kMaxOwn = 4;    // the sketch's own settings (addSetting)
  // The items' bytes this probe holds and saves (describe storage max_bytes): every item with its 3-byte header, a wifi
  // item with its passphrase (§1.4).
  static constexpr size_t kMaxItems = 800;

  ProbeConfig(Endpoint &endpoint, Binds &binds) : endpoint_(endpoint), binds_(binds) {
    for (size_t p = 0; p < kMaxPorts; ++p) pass_uart_[p] = pass_console_[p] = -1;
  }
  // A wire and its console a console-passthrough item may name (after the endpoint has them).
  bool addPlace(WireRvswd &wire, TargetConsoleStream &console);
  // A fixture UART a passthrough item may name (after the endpoint has it).
  bool addUart(FixtureUart &uart);
  // The pins the idle and disable items work on.
  void setPins(PinTable *pins) { pins_ = pins; endpoint_.setPins(pins); }
  // fn 0's channels (describe channels): the channel items' keys.
  void setChannels(uint16_t channels) { channels_ = channels; }
  // The Wi-Fi station: netif "wifi", the wifi and ipv4 items and the network op. false (and nothing declared) when a
  // transport's max_frame is below wifi_min_max_frame (§1.4: the longest wifi set must fit every transport).
  bool setWifi(WifiControl *wifi) {
    if (wifi && endpoint_.maxFrame() < reg::kLimitWifiMinMaxFrame) return false;
    wifi_ = wifi;
    return true;
  }
  // A setting of the sketch's own (§1.6): a reverse-DNS name (1-48 bytes with a dot), form 1-5 (bytes, text, u32, ipv4,
  // bool), keys (1 or more), max_len (bytes / text), flags (§1.7: secret, restart). Before load(). false: no room or not
  // a valid declaration.
  bool addSetting(const char *name, uint8_t form, uint16_t keys = 1, uint8_t max_len = 0, uint8_t flags = 0);
  // An own setting's value now (nullptr: not set): its data and length.
  const uint8_t *setting(const char *name, uint16_t key, size_t &length) const;
  // load() reads what was saved; applySaved() applies it item by item (probe.config §3): both after the sketch's last
  // endpoint.add(). savedDisabled: the saved disable items' channels, for a sketch that parks its pins first.
  void load();
  uint64_t savedDisabled() const;
  void applySaved();
  void poll();   // from loop(): the pass-throughs
  void setAttachGate(bool (*ready)()) { attach_gate_ = ready; }   // a console pass-through's attach waits on it
  void skipBootAttach() { skip_boot_attach_ = true; }
  bool bootAttachSkipped() const { return skip_boot_attach_; }
  // A probe that saves nothing (before the first poll): no save / erase / state, no storage tag (probe.config §4).
  void setStorage(bool on) { storage_ = on; }

  const char *name() const override { return reg::probe_config::kName; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return reg::probe_config::kRevision; }
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::probe_config::kLockFreeOps, op); }
  bool offers(uint8_t op) const override {
    using namespace reg::probe_config;
    if (op == kOpGet || op == kOpSet || op == kOpUnset) return true;
    if (op == kOpNetwork) return wifi_ != nullptr;
    return storage_ && (op == kOpSave || op == kOpErase || op == kOpState);
  }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  uint32_t generation() const { return generation_; }
  // The store (kind len key data, see above): findLine reads its labels.
  const uint8_t *items() const { return items_; }
  size_t itemsLength() const { return items_length_; }
  // An item's number on the wire this boot (0: not in the list), by the store's kind (config_kind).
  uint8_t numberOf(uint8_t kind) const;
  // A test's starting point: the generation (probe.config §2: 0xFFFFFFFF refuses every change).
  void testGeneration(uint32_t g) { generation_ = g; }

 private:
  struct Place { WireRvswd *wire = nullptr; TargetConsoleStream *console = nullptr; };
  struct Own { char name[49] = {}; uint8_t form = 0, max_len = 0, flags = 0; uint16_t keys = 1; };
  Endpoint &endpoint_;
  Binds &binds_;
  PinTable *pins_ = nullptr;
  uint16_t channels_ = PinTable::kChannels;
  Place places_[kMaxPlaces];
  size_t place_count_ = 0;
  FixtureUart *uarts_[kMaxUarts] = {};
  size_t uart_count_ = 0;
  WifiControl *wifi_ = nullptr;
  uint32_t wifi_applied_ = 0;    // CRC-32 of the wifi / ipv4 items last handed to wifi_ (0xFFFFFFFF: none yet)
  bool wifi_handed_ = false;
  Own own_[kMaxOwn];
  size_t own_count_ = 0;
  bool storage_ = true, skip_boot_attach_ = false;
  bool (*attach_gate_)() = nullptr;
  uint8_t items_[kMaxItems];     // the store, in (kind, key) order
  size_t items_length_ = 0;
  uint32_t generation_ = 1;
  uint8_t storage_state_ = reg::probe_config::kStorageStateNone, reason_ = 0;
  uint32_t saved_generation_ = 0;
  uint8_t saved_[kMaxItems];     // what is saved now, as a store (the items whose names this firmware knows)
  static uint8_t scratch_[kMaxItems];   // a change's candidate store / the saved items being applied
  size_t saved_length_ = 0;
  bool saved_unknown_ = false;   // the save held names this firmware does not know (§3: invalid items)
  bool loaded_ = false;          // a saved blob read whole (load)
  int8_t pass_uart_[kMaxPorts];     // the uarts_ index passing to the port now, -1: none
  int8_t pass_console_[kMaxPorts];  // the places_ index whose console passes to the port now, -1: none
  bool console_tried_[kMaxPorts] = {};

  bool declares(uint8_t kind) const;
  uint8_t kindOf(uint8_t number) const;   // 0: no item of that number
  uint8_t kindNamed(const uint8_t *name, size_t length) const;   // 0: a name this firmware does not know
  const char *nameOf(uint8_t kind) const;
  uint16_t keysOf(uint8_t kind) const;
  bool serialPort(uint16_t port) const;
  int uartIndex(uint16_t fn) const;
  int placeIndex(uint16_t wire_fn, uint16_t console_fn) const;
  const uint8_t *find(const uint8_t *store, size_t length, uint8_t kind, uint16_t key, size_t &vlen) const;
  Result checkItem(uint8_t kind, uint8_t number, const uint8_t *v, size_t len, uint8_t *out, size_t capacity) const;
  Result checkWhole(const uint8_t *items, size_t length, uint8_t *out, size_t capacity) const;
  static bool insert(uint8_t *store, size_t &length, size_t capacity, uint8_t kind, const uint8_t *v, size_t len);
  static void remove(uint8_t *store, size_t &length, uint8_t kind, uint16_t key);
  void adopt(const uint8_t *items, size_t length);   // make them current: the pins, the pass-throughs, the Wi-Fi
  void handWifi();
  uint64_t disabledIn(const uint8_t *items, size_t length) const;
  size_t shown(uint8_t kind, const uint8_t *v, size_t vlen, uint8_t *out) const;   // an item as get shows it
  Result get(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result set(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result unset(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result network(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result commit(const uint8_t *candidate, size_t length, uint8_t *out, size_t capacity);
  size_t saveBlob(uint8_t *blob, size_t capacity) const;   // the store by name, then CRC-32
  void runPassthroughs();
  void runPassthroughsYield(uint8_t port);   // a session's attach takes the resident connection
  static void passIn(void *self, const uint8_t *data, size_t length);
  uint8_t passing_port_ = 0xff;   // the port passIn writes for (set around a UART's poll)
};

uint32_t crc32Ieee(const uint8_t *data, size_t length);
#if defined(OEP_HOST_FAKE_CONFIG)
void fakeConfigStore(const uint8_t *blob, size_t length);   // a host test's saved blob (items, then their CRC-32)
#endif

}  // namespace oep

#endif

namespace oep {

// The label convention (host guide §5.3: the host's way to find a line; the probe drives none of these lines itself)
// over a store of items (ProbeConfig::items: kind 1 the labels, channel key then text) and the firmware's labels: the
// channel of line `line` of port `slot` - the settings label "slot.line", else (at most one port named) "line", else
// the firmware label "line" (ASCII case ignored; two found in one step: none). A port is the S of "S.N" labels; with
// more than one named, "line" and the firmware's labels name nothing. 0xFFFF: none.
uint16_t findLine(const uint8_t *items, size_t length, const char *slot, const char *line,
                  const uint8_t *firmware = nullptr, size_t firmware_length = 0);

}  // namespace oep
