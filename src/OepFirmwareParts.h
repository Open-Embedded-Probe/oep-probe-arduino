// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The distributed firmware's parts a sketch names from any of its files: the feature bits, FirmwareConfig and
// oep::firmware's add / onSetup / onLoop. Starting the firmware (begin / loop) is OepFirmware.h's, included from the
// sketch's .ino alone.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "Oep.h"   // interfaceName(), Interface
#include "OepEndpoint.h"   // onSetup's Endpoint and PinTable: complete for the sketch's own files
#include "OepPinTable.h"

namespace oep {

class Endpoint;
class Interface;
class PinTable;

// The firmware's features, one bit each: begin<F>() adds the ones F has, in the firmware's own order; a feature F
// leaves out links none of its code. Which a chip has is oep::kAvailable, the distributed firmware's set oep::kDefaults
// (both from OepFirmware.h, for the chip the build is for).
// Transports (at least one), and the ESP32-P4's DFU (outside OEP)
inline constexpr uint64_t kBulk = 1ull << 0;      // USB vendor bulk
inline constexpr uint64_t kHid = 1ull << 1;       // USB HID
inline constexpr uint64_t kCdc = 1ull << 2;       // USB CDC
inline constexpr uint64_t kUsj = 1ull << 3;       // the chip's own USB serial (ESP32-P4's USB-Serial/JTAG)
inline constexpr uint64_t kBridge = 1ull << 4;    // a UART bridge
inline constexpr uint64_t kWifi = 1ull << 5;      // TCP over Wi-Fi (the settings' networks)
inline constexpr uint64_t kDfu = 1ull << 6;       // USB DFU (not OEP: the firmware's own update)
inline constexpr uint64_t kPortSpeed = 1ull << 7; // fn 0's port_speed on the UART bridge (needs kBridge)
// Interfaces
inline constexpr uint64_t kRvswd = 1ull << 8;           // oep.wire.rvswd and oep.target.riscv-dm
inline constexpr uint64_t kConsole = 1ull << 9;         // the RVSWD wire's target console (needs kRvswd)
inline constexpr uint64_t kSwio = 1ull << 10;           // oep.wire.swio
inline constexpr uint64_t kSwioConsole = 1ull << 11;    // the SWIO wire's target console (needs kSwio)
inline constexpr uint64_t kSwd = 1ull << 12;            // oep.wire.swd and oep.target.arm-adi
inline constexpr uint64_t kGpio = 1ull << 13;           // oep.fixture.gpio
inline constexpr uint64_t kUart = 1ull << 14;           // oep.fixture.uart
inline constexpr uint64_t kI2cTarget = 1ull << 15;      // oep.fixture.i2c-target
inline constexpr uint64_t kSpiTarget = 1ull << 16;      // oep.fixture.spi-target
inline constexpr uint64_t kI2cController = 1ull << 17;  // oep.fixture.i2c-controller
inline constexpr uint64_t kSpiController = 1ull << 18;  // oep.fixture.spi-controller
inline constexpr uint64_t kCapture = 1ull << 19;        // oep.fixture.logic by DMA or the sampler (ESP32, ESP32-P4)
inline constexpr uint64_t kLogic = 1ull << 20;          // oep.fixture.logic by PIO / the CH32's capture
inline constexpr uint64_t kAnalog = 1ull << 21;         // oep.fixture.analog
inline constexpr uint64_t kGroup = 1ull << 22;          // oep.fixture.capture-group
inline constexpr uint64_t kPwm = 1ull << 23;            // oep.fixture.pwm
inline constexpr uint64_t kCounter = 1ull << 24;        // oep.fixture.counter
inline constexpr uint64_t kDac = 1ull << 25;            // oep.fixture.dac
inline constexpr uint64_t kMemo = 1ull << 26;           // oep.probe.memo
inline constexpr uint64_t kLink = 1ull << 27;           // oep.probe.link
inline constexpr uint64_t kSettings = 1ull << 28;       // the settings (core §13)
inline constexpr uint64_t kNone = 0;
inline constexpr uint64_t kTransports = kBulk | kHid | kCdc | kUsj | kBridge | kWifi;

// Values the firmware takes at run time; nullptr: the distributed firmware's
struct FirmwareConfig {
  const char *model = nullptr;     // fn 0's describe model (core §7.5)
  const char *product = nullptr;   // the USB product name (iProduct), for people
};

// The distributed firmware: the sketch's own parts registered before begin(), begun and run by OepFirmware.h
class Firmware {
 public:
  using SetupFn = void (*)(Endpoint &endpoint, PinTable &pins);
  using LoopFn = void (*)();

  // An interface of the sketch's own: added after the firmware's, before the saved settings are applied, in the order
  // registered (endpoint.add)
  // false: a name the endpoint refuses (core §13 rule 1: at most 48 bytes of a reverse domain name)
  bool add(Interface &interface) {
    if (!interfaceName(interface.name())) return false;
    append(interfaces_, new Node{&interface, nullptr, nullptr, nullptr});
    return true;
  }
  // Called in begin(), before the sketch's interfaces are added: take pins (owners::kSketch and up), set things up
  void onSetup(SetupFn fn) { append(setups_, new Node{nullptr, fn, nullptr, nullptr}); }
  // Called every loop(), after the firmware's own work; it must not wait
  void onLoop(LoopFn fn) { append(loops_, new Node{nullptr, nullptr, fn, nullptr}); }

  // OepFirmware.h (the sketch's .ino): the distributed firmware's set, or the features F has
  void begin(const FirmwareConfig &config = FirmwareConfig());
  template <uint64_t F>
  void begin(const FirmwareConfig &config = FirmwareConfig());
  void loop();

  // The firmware's own: the sketch's parts at their place
  void runSetup(Endpoint &endpoint, PinTable &pins);
  void runLoop() {
    for (Node *n = loops_; n; n = n->next) n->loop();
  }

 private:
  struct Node {
    Interface *interface;
    SetupFn setup;
    LoopFn loop;
    Node *next;
  };
  static void append(Node *&head, Node *node) {
    Node **at = &head;
    while (*at) at = &(*at)->next;
    *at = node;
  }
  Node *interfaces_ = nullptr, *setups_ = nullptr, *loops_ = nullptr;
};

inline Firmware firmware;

}  // namespace oep
