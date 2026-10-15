// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP v1 oep.probe.config (oep-spec interfaces/oep-if-probe-config.ja.md, revision 1): the probe's settings - a
// channel's disable, idle and label, and a serial port's pass-through - set by the host, kept until a restart, and
// saved only when the host saves them.
//
//   0x10 get(first u16) -> more(u8) generation(u32) count(u8) items [TLV]                       (no lock)
//   0x11 set(count u8, items) [TLV] -> generation(u32) [TLV]
//   0x12 save [TLV] -> generation(u32) [TLV]      0x13 erase [TLV]                (with storage)
//   0x14 unset(count u8, count x (tag u8, key u16)) [TLV] -> generation(u32) [TLV]
//   0x15 state [TLV] -> storage_state(u8) saved_generation(u32) reason(u8) [TLV]   (no lock; with storage)
//
// Items: tag(u8) len(u16) value; the tag's low 7 bits are the item (bit 7 ignored, the host sends 0):
//   0x02 label   channel(u16) text (1-32 bytes)                                             key channel
//   0x03 idle    channel(u16) mode(u8: 0 Hi-Z, 1 pull-up, 2 pull-down, 3 output low, 4 output high)  key channel
//   0x05 passthrough   port(u8) fn(u16) rx(u16) baud(u32) format(u8)                         key port (§1.1)
//   0x06 console_passthrough port(u8) wire_fn(u16) swdio(u16) swclk(u16) max_speed(u32) console_fn(u16)
//                            mechanism(u8)                                                    key port (§1.2)
//   0x07 disable channel(u16)                                                                key channel
// generation: 1 at boot, +1 for every set / unset that changes the contents (never 0; at 0xFFFFFFFF a change is
// refused cause 2 until a restart). Saved to NVS on an ESP32 (Preferences "oepcfg" / "items7"), the flash's last
// sector on an RP2 (EEPROM, "OEP7"); a save of the forms before reads as unreadable (reason 1).
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

// Wi-Fi's networks: no longer oep.probe.config's (oep-spec cf69012: a transport's setting is defined elsewhere); kept
// for the sketches that hand a WifiControl over, which gets nothing from these settings now.
struct WifiEntry {
  uint8_t index = 0, ssid_length = 0, pass_length = 0;
  char ssid[33] = {};
  char pass[65] = {};
};
class WifiControl {
 public:
  struct Status { uint8_t state = 0, entry = 0xff, reason = 0; int8_t rssi = 0; uint8_t ipv4[4] = {}; };
  virtual void apply(const WifiEntry *entries, size_t count) = 0;
  virtual Status status() const = 0;
};

class ProbeConfig final : public Interface {
 public:
  static constexpr size_t kMaxPlaces = 2, kMaxUarts = 4, kMaxPorts = Binds::kMaxPorts;
  // The items' bytes this probe holds and saves (describe storage max_bytes): every item with its 3-byte header.
  static constexpr size_t kMaxItems = 512;

  ProbeConfig(Endpoint &endpoint, Binds &binds) : endpoint_(endpoint), binds_(binds) {
    for (size_t p = 0; p < kMaxPorts; ++p) pass_uart_[p] = pass_console_[p] = -1;
  }
  // A wire and its console a console_passthrough item may name (after the endpoint has them).
  bool addPlace(WireRvswd &wire, TargetConsoleStream &console);
  // A fixture UART a passthrough item may name (after the endpoint has it).
  bool addUart(FixtureUart &uart);
  // The pins the idle and disable items work on.
  void setPins(PinTable *pins) { pins_ = pins; endpoint_.setPins(pins); }
  bool setWifi(WifiControl *) { return false; }
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
    return storage_ && (op == kOpSave || op == kOpErase || op == kOpState);
  }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  uint32_t generation() const { return generation_; }
  const uint8_t *items() const { return items_; }
  size_t itemsLength() const { return items_length_; }
  // A test's starting point: the generation (probe.config §2: 0xFFFFFFFF refuses every change).
  void testGeneration(uint32_t g) { generation_ = g; }

 private:
  struct Place { WireRvswd *wire = nullptr; TargetConsoleStream *console = nullptr; };
  Endpoint &endpoint_;
  Binds &binds_;
  PinTable *pins_ = nullptr;
  Place places_[kMaxPlaces];
  size_t place_count_ = 0;
  FixtureUart *uarts_[kMaxUarts] = {};
  size_t uart_count_ = 0;
  bool storage_ = true, skip_boot_attach_ = false;
  bool (*attach_gate_)() = nullptr;
  uint8_t items_[kMaxItems];     // the items in get's order (item, then key), tag len value (bit 7 cleared)
  size_t items_length_ = 0;
  uint32_t generation_ = 1;
  uint8_t storage_state_ = reg::probe_config::kStorageStateNone, reason_ = 0;
  uint32_t saved_generation_ = 0;
  uint8_t saved_[kMaxItems];     // what is saved now (the same form)
  size_t saved_length_ = 0;
  bool loaded_ = false;          // a saved blob read whole (load)
  int8_t pass_uart_[kMaxPorts];     // the uarts_ index passing to the port now, -1: none
  int8_t pass_console_[kMaxPorts];  // the places_ index whose console passes to the port now, -1: none
  bool console_tried_[kMaxPorts] = {};

  bool declares(uint8_t item) const;
  bool serialPort(uint8_t port) const;
  int uartIndex(uint16_t fn) const;
  int placeIndex(uint16_t wire_fn, uint16_t console_fn) const;
  static bool keyOf(uint8_t item, const uint8_t *v, size_t len, uint16_t &key);
  Result checkItem(uint8_t raw, const uint8_t *v, size_t len, uint8_t *out, size_t capacity) const;
  Result checkWhole(const uint8_t *items, size_t length, uint8_t *out, size_t capacity) const;
  static bool insert(uint8_t *store, size_t &length, size_t capacity, uint8_t item, const uint8_t *v, size_t len);
  static void remove(uint8_t *store, size_t &length, uint8_t item, uint16_t key);
  void adopt(const uint8_t *items, size_t length);   // make them current: the pins, the pass-throughs
  uint64_t disabledIn(const uint8_t *items, size_t length) const;
  Result get(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result set(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result unset(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result commit(const uint8_t *candidate, size_t length, uint8_t *out, size_t capacity);
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
// over a store of items and the firmware's labels: the channel of line `line` - the settings label "slot.line", else
// "line", else the firmware label "line" (ASCII case ignored; two found in one step: none). 0xFFFF: none.
uint16_t findLine(const uint8_t *items, size_t length, const char *slot, const char *line,
                  const uint8_t *firmware = nullptr, size_t firmware_length = 0);

}  // namespace oep
