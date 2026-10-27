// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// WCH CH32 on ArduinoCore-CH32RV (profiles ch32x035, ch32v307, ch32h417; the core's 0.4.0, LTO by default).
//
// Transports, each a choice (below): the USB vendor bulk interface (class 0xFF, 'O', 'E': transports §3) and the USB HID
// one at full speed, length-prefixed frames; a USB CDC port and a UART bridge (COBS) too if chosen. The USB device is the project's VID:PID 1209:4F45 (registry usb; PID-USE.md), serial = the unit id (the chip's 64-bit
// unique id in hex); iProduct "OEP probe (CH32X035)" is a name for people. The core runs the USB stack from loop() and
// delay(): no other context, so the classes are read and written from loop() as Streams.
//
// Interfaces (revision 1), as the RP2's (Rp2.h): fn 0 (the core); oep.wire.rvswd + oep.target.riscv-dm +
// oep.target.console (WCH CH32, 2 wires); oep.wire.swd + oep.target.arm-adi (ARM); oep.fixture.gpio / uart;
// oep.fixture.i2c-target / spi-target (the core's Wire and SPI slaves) / i2c-controller / spi-controller (GPIOs);
// the settings (core §13) (no store yet: nothing saved, setStorage(false)); oep.probe.link; oep.probe.plan and
// oep.probe.restart (the endpoint's own). Every channel (platformPad: port x 24 + bit) may be SWDIO / SWCLK of either
// wire, the reset line of attach's reset TLV, a gpio or a UART pin (any enabled USART's route; another pair is
// refused at the UART's begin).
//
// A CH32 but the H41x builds with OEP_SMALL_RAM (Oep.h): the endpoint's tables cut, a 1 KiB console ring. Everything is
// in by default but on the CH32X035 (62 KiB flash, 20 KiB RAM; kRoomy below): the V307 97.7 KB / 24.8 KB, the H417
// 106 KB (0.2.2).
//
// Each interface and transport is a choice (OepFeatures.h, OepChoice.h): oepWithX() is weak here, its default adding X
// with oepAddX() (or nothing, for CDC and the UART bridge); a sketch's own oepWithX() replaces it, an empty one leaving X out.
// Each X's objects live in its oepAddX() and what X leaves to the others (its loop work, its part of the settings)
// goes through the hooks below, so a left-out X links none of its code. The CH32 has: Rvswd (oep.wire.rvswd +
// oep.target.riscv-dm), Console (oep.target.console on the RVSWD wire), Swio (oep.wire.swio, CH32V00x; SwioConsole on
// it), Swd (oep.wire.swd + oep.target.arm-adi), Gpio,
// Uart, Config (the settings (core §13)), Link (oep.probe.link); Bulk (the USB vendor bulk transport), Hid (the USB HID transport: usage page 0xFF4F, usage 0x45,
// transports §1, §3), Cdc (a USB CDC port, COBS: a serial transport), Bridge (a UART bridge on Serial1's default pins,
// 115200: a serial transport).
#pragma once
#include <CH32RV.h>
#include <CH32RVUsbDevice.h>

#include <OepBind.h>
#include <OepCh32Dm.h>
#include <OepChoice.h>
#include <OepSettings.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepP4I2cTarget.h>
#include <OepPinTable.h>
#include <OepPlatform.h>
#include <OepRvswdPhy.h>
#include <OepCh32SpiTarget.h>   // empty on the X035 (its flash)
#include <OepI2cController.h>
#include <OepSpiController.h>
#include <OepSwd.h>
#include <OepSwioPhy.h>
#include <OepTarget.h>

#include "OepFeatures.h"

// The chip the build is for (its board: the profiles ch32x035 / ch32v307 / ch32h417), a name for people and describe's
// model
#if defined(ARDUINO_CH32X035)
static constexpr const char *kProduct = "OEP probe (CH32X035)", *kModel = "ch32x035";
#elif defined(ARDUINO_CH32V307)
static constexpr const char *kProduct = "OEP probe (CH32V307)", *kModel = "ch32v307";
#elif defined(ARDUINO_CH32H417)
static constexpr const char *kProduct = "OEP probe (CH32H417)", *kModel = "ch32h417";
#else
static constexpr const char *kProduct = "OEP probe (CH32)", *kModel = "ch32";
#endif

// The vendor class as the endpoint's Stream (it is a Print with available / read / peek, not a Stream)
class VendorStream final : public Stream {
 public:
  explicit VendorStream(CH32RVUsbDeviceVendor &vendor) : vendor_(vendor) {}
  int available() override { return vendor_.available(); }
  int read() override { return vendor_.read(); }
  int peek() override { return vendor_.peek(); }
  size_t write(uint8_t c) override { return vendor_.write(c); }
  size_t write(const uint8_t *data, size_t length) override { return vendor_.write(data, length); }
  void flush() override { vendor_.flush(); }

 private:
  CH32RVUsbDeviceVendor &vendor_;
};

// The HID transport (transports §1): a report carries count(u16) and that many bytes of the length-prefixed stream, the
// rest padding; the reports are the core's vendor HID ones (report ID 6, 63 bytes at full speed: 61 of the stream).
class HidStream final : public Stream {
 public:
  static constexpr size_t kReport = 63, kPayload = kReport - 2;
  explicit HidStream(CH32RVUsbDeviceHidVendor &h) : h_(h) {
    h_.onOutputReport([this](const CH32RVUsbDeviceHidReport &r) { take(r.data, r.length); });
  }
  int available() override { return static_cast<int>((head_ + sizeof rx_ - tail_) % sizeof rx_); }
  int read() override {
    if (head_ == tail_) return -1;
    const uint8_t c = rx_[tail_];
    tail_ = (tail_ + 1) % sizeof rx_;
    return c;
  }
  int peek() override { return head_ == tail_ ? -1 : rx_[tail_]; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    for (size_t i = 0; i < n; ++i) {
      tx_[2 + fill_++] = b[i];
      if (fill_ == kPayload) flush();
    }
    return n;
  }
  void flush() override {
    if (!fill_) return;
    tx_[0] = static_cast<uint8_t>(fill_);
    tx_[1] = 0;
    memset(tx_ + 2 + fill_, 0, kPayload - fill_);   // the padding 0 (transports §1 rule 3)
    // a report still in flight: wait for it (dropping one breaks the stream) - but once one could not go (nobody reads
    // HID), later ones do not wait again until one goes
    stuck_ = !h_.sendInput(tx_, kReport, stuck_ ? 0 : 200);
    fill_ = 0;
  }

 private:
  void take(const uint8_t *d, size_t len) {   // one output report: its count bytes into the ring (rule 6: a bad count drops it)
    if (!d || len < 2) return;
    const size_t n = d[0] | (d[1] << 8);
    if (n > len - 2) return;
    for (size_t i = 0; i < n; ++i) {
      const size_t next = (head_ + 1) % sizeof rx_;
      if (next == tail_) return;
      rx_[head_] = d[2 + i];
      head_ = next;
    }
  }
  CH32RVUsbDeviceHidVendor &h_;
  uint8_t rx_[640];
  volatile size_t head_ = 0, tail_ = 0;
  uint8_t tx_[kReport];
  size_t fill_ = 0;
  bool stuck_ = false;
};

static uint8_t txBuffer[512];
static oep::Endpoint endpoint(txBuffer, sizeof txBuffer, {512, 1024, 2});
static uint8_t probeTlv[160];
static char serial[17];

// The channels (platformPad: port x 24 + bit): the pads PA, PB and PC have (CH32RV_GPIO_PORT_WIDTH a port; the
// CH32X035's 24: PA0-PA23, PB0-PB23, PC0-PC15), less the chip's own: the X035's USB pads (PC16 / PC17) and SWD (PC18 /
// PC19) are past channel 63; a V307's USBFS (PA11 / PA12, the core's controller there) and SWD (PA13 / PA14); an H417's
// SWD (PB8 / PB9; its USBSS has pads of its own). Which of these a package brings out is the board's; a pad it does not
// is left alone by the host.
static constexpr uint64_t kPortPads = (1ull << (CH32RV_GPIO_PORT_WIDTH < 24 ? CH32RV_GPIO_PORT_WIDTH : 24)) - 1;
static constexpr uint64_t kPads = kPortPads | (kPortPads << 24) | (kPortPads << 48);
#if defined(ARDUINO_CH32V307)
static constexpr uint64_t kChipOwn = 0xfull << 11;   // PA11-PA14
#elif defined(ARDUINO_CH32H417)
static constexpr uint64_t kChipOwn = 0x3ull << (24 + 8);   // PB8 / PB9
#else
static constexpr uint64_t kChipOwn = 0;
#endif
static constexpr uint64_t kChannels = kPads & ~kChipOwn;
static constexpr uint16_t kUnset = 0xfffe;   // no pair chosen yet
static oep::PinTable pins(kChannels);

// What a choice leaves to run later, each added only from its own oepAddX(): what waits for the USB device, its loop work
static oep::Hooks<void (*)(), 1> afterUsb;
static oep::Hooks<void (*)(), 5> loopWork;

// What a choice leaves for another to find, each set only from its own oepAddX(); the code using both is the other's (the
// settings', the SWIO wire's), so neither names the other's code: a serial transport (the settings' pass-throughs), the
// consoles (and the SWIO wire), SWD's port and the uart, for the settings; the riscv-dm target for the SWIO wire
static bool serialPorts = false;
static oep::TargetConsoleStream *consoleIn = nullptr, *swioConsoleIn = nullptr;
// oep.target.console: the probe's one (common §2), each wire's console a place in it
static oep::TargetConsole *consoles = nullptr;
static void addConsolePlace(oep::TargetConsoleStream &place) {
  if (consoles) { consoles->add(place); return; }
  static oep::Slot<oep::TargetConsole> one;
  consoles = &one.make(place);
  endpoint.add(*consoles);
}
static oep::WireRvswd *swioWireIn = nullptr;
static oep::TargetRiscvDm *riscvDmIn = nullptr;   // the RVSWD wire's, the SWIO wire's connections too
static oep::SwdPort *swdIn = nullptr;
static oep::WireSwd *swdWireIn = nullptr;   // its interface: the owner of its hold item (core §13.1)
static oep::FixtureUart *uartIn = nullptr;

void oepAddBulk() {
  static oep::Slot<CH32RVUsbDeviceVendor> usbVendor;
  static oep::Slot<VendorStream> stream;
  static uint8_t rx[520];
  usbVendor.make(USBDevice, 64).setInterfaceProtocol(oep::reg::kUsbVendorBulkSubclass, oep::reg::kUsbVendorBulkProtocol);
  endpoint.addTransport(stream.make(*usbVendor), rx, sizeof rx, oep::Endpoint::kVendorBulk, 0);   // the first interface
}

void oepAddHid() {
  static oep::Slot<CH32RVUsbDeviceHidVendor> usbHid;
  static oep::Slot<HidStream> stream;
  static uint8_t rx[520];
  stream.make(usbHid.make(USBDevice, HidStream::kReport));
  usbHid->setUsage(oep::reg::kUsbHidUsagePage, oep::reg::kUsbHidUsage);   // 0xFF4F / 0x45 (registry usb)
  endpoint.addTransport(*stream, rx, sizeof rx, oep::Endpoint::kHid, 0xff, true);   // the device's one HID interface (the core numbers it)
}

// The ports are the firmware's own, not the core's SerialUSB / Serial1 / Serial: naming one of those links it (and its
// registration with the device) whether or not it is chosen
static CH32RVUsbDeviceCdcSerial &cdc() {
  static CH32RVUsbDeviceCdcSerial port(USBDevice);
  return port;
}

void oepAddCdc() {
  cdc().begin(115200);   // before USBDevice.begin: its interface is laid out with the vendor one
  afterUsb.add([] {   // its interface number, once laid out
    static uint8_t rx[oep::cobsFrameMax(512) + 8];   // the encoded candidate (transports §1)
    endpoint.addTransport(cdc(), rx, sizeof rx, oep::Endpoint::kUsbCdc, cdc().interfaceNumber(), true);
  });
  serialPorts = true;
}

void oepAddBridge() {
  static oep::Slot<oep::OepUart> port;
  static uint8_t rx[oep::cobsFrameMax(512) + 8];
  port.make(1).begin(115200);   // USART2 (Serial1's), its default route (transports §1: a UART bridge at its boot speed)
  endpoint.addTransport(*port, rx, sizeof rx, oep::Endpoint::kUartBridge);
  serialPorts = true;
}

// The RVSWD wire's parts, the console's too (its choice is made in the wire's: no wire, no console)
struct RvswdParts {
  oep::RvswdPhy phy;
  oep::Ch32Dm dm{phy};
  oep::DebugPort port{dm, kUnset, kUnset};
  oep::WireRvswd wire{port, 0};
  RvswdParts() {
    port.pin_choice = kChannels;
    port.pins = &pins;
    port.reset_allowed = kChannels;
  }
};
static oep::Slot<RvswdParts> rvswd;

static oep::Slot<oep::TargetRiscvDm> riscvDm;

void oepAddRvswd() {
  RvswdParts &r = rvswd.make();
  endpoint.add(r.wire);
  riscvDmIn = &riscvDm.make(r.port, 0);
  endpoint.add(*riscvDmIn);
  oepWithConsole();
}

void oepAddConsole() {
  static oep::Slot<oep::DmConsole> driver;
  static oep::Slot<oep::TargetConsoleStream> console;
  consoleIn = &console.make(rvswd->port, driver.make(rvswd->dm, rvswd->phy), 0);
  addConsolePlace(*consoleIn);
  loopWork.add([] { console->poll(); });
}

// The one-wire link (CH32V00x): its own wire fn, its connections served by the RVSWD wire's riscv-dm target (or one of
// its own without that wire), its console a place of the one oep.target.console (its choice made in the wire's)
struct SwioParts {
  oep::SwioPhy phy;
  oep::Ch32Dm dm{phy};
  oep::DebugPort port{dm, kUnset, 0xffff};   // one wire: swclk stays 0xffff
  oep::WireRvswd wire{port, 0, "oep.wire.swio"};
  SwioParts() {
    port.pin_choice = kChannels;
    port.pins = &pins;
    port.pin_owner = 0xf2;
    port.reset_allowed = kChannels;
  }
};
static oep::Slot<SwioParts> swio;

void oepAddSwio() {
  SwioParts &w = swio.make();
  endpoint.add(w.wire);
  swioWireIn = &w.wire;
  if (riscvDmIn) {
    riscvDmIn->addPort(w.port);
  } else {
    riscvDmIn = &riscvDm.make(w.port, 0);
    endpoint.add(*riscvDmIn);
  }
  oepWithSwioConsole();
}

void oepAddSwioConsole() {
  static oep::Slot<oep::DmConsole> driver;
  static oep::Slot<oep::TargetConsoleStream> console;
  swioConsoleIn = &console.make(swio->port, driver.make(swio->dm, swio->phy), 0);   // the one-wire link's place
  addConsolePlace(*swioConsoleIn);
  loopWork.add([] { console->poll(); });
}

void oepAddSwd() {
  static oep::SwdPort swd{kUnset, kUnset};
  static oep::Slot<oep::WireSwd> wireSwd;
  static oep::Slot<oep::TargetArmAdi> adi;
  swd.pin_choice = kChannels;
  swd.pins = &pins;
  endpoint.add(wireSwd.make(swd, 0));
  endpoint.add(adi.make(swd, 0));
  swdIn = &swd;
  swdWireIn = &*wireSwd;
}

void oepAddGpio() {
  static oep::Slot<oep::FixtureGpio> gpio;
  endpoint.add(gpio.make(pins, 0, 1));
}

void oepAddUart() {
  static oep::Slot<oep::OepUart> port;
  static oep::Slot<oep::FixtureUart> uart;
  endpoint.add(uart.make(pins, port.make(0), 0, 2));   // USART1 (Serial's)
  uart->setRoleChannels(kChannels, kChannels);   // any enabled USART's route (its begin refuses another pair)
  uartIn = &*uart;
  loopWork.add([] { uart->poll(); });
}

// oep.fixture.i2c-target on the core's Wire slave (PinTable owner 3)
void oepAddI2c() {
  static oep::Slot<oep::P4I2cTarget> i2c;
  endpoint.add(i2c.make(pins));
}

// oep.fixture.spi-target on the core's SPI slave (CH32RVSPISlave; PinTable owner 6)
void oepAddSpi() {
#if !defined(ARDUINO_CH32X035)
  static oep::Slot<oep::Ch32SpiTarget> spi;
  endpoint.add(spi.make(pins));
  loopWork.add([] { spi->service(); });
#endif
}

void oepAddSettings() {
  static oep::Slot<oep::Binds> binds;
  static oep::Slot<oep::Settings> config;
  oep::Settings &c = config.make(endpoint, binds.make());
  c.setStorage(false);   // no store on this core yet (core §13.7: no save / erase / state)
  endpoint.setRawPorts(&*binds);
  if (serialPorts) c.usePassthroughs();   // a serial port's raw stream carries them (fixture §2.1)
  if (consoleIn) c.addPlace(rvswd->wire, *consoleIn);
  if (swioConsoleIn) c.addPlace(*swioWireIn, *swioConsoleIn);   // a slot on the one wire (CH32V00x): the second place
  if (swdIn) c.addHold(*swdWireIn, swdIn->hold, oep::checkSwdHold, swdIn);
  if (uartIn) c.addUart(*uartIn);
  c.setPins(&pins);
  loopWork.add([] { config->poll(); });
  c.applySaved();
}

void oepAddLink() {
  static oep::Slot<oep::Link> link;
  endpoint.add(link.make(endpoint));   // oep.probe.link (oep-if-link)
}

// oep.fixture.i2c-controller / spi-controller: the lines on GPIOs (PinTable owners 8 / 9)
void oepAddI2cController() {
  static oep::Slot<oep::I2cController> i2c;
  endpoint.add(i2c.make(pins));
}

void oepAddSpiController() {
  static oep::Slot<oep::SpiController> spi;
  endpoint.add(spi.make(pins));
}

// This build's choices (OepFeatures.h: a sketch's own replaces one). The CH32X035's 62 KiB take the WCH set (RVSWD,
// console, gpio, link, HID: 62.4 KB); SWD, uart and the settings are left out by default.
#if defined(ARDUINO_CH32X035)
static constexpr bool kRoomy = false;
#else
static constexpr bool kRoomy = true;
#endif
__attribute__((weak)) void oepWithBulk() { oepAddBulk(); }
__attribute__((weak)) void oepWithHid() { oepAddHid(); }
__attribute__((weak)) void oepWithCdc() {}
__attribute__((weak)) void oepWithBridge() {}
__attribute__((weak)) void oepWithRvswd() { oepAddRvswd(); }
__attribute__((weak)) void oepWithConsole() { oepAddConsole(); }
__attribute__((weak)) void oepWithSwd() {
  if (kRoomy) oepAddSwd();
}
__attribute__((weak)) void oepWithGpio() { oepAddGpio(); }
__attribute__((weak)) void oepWithUart() {
  if (kRoomy) oepAddUart();
}
__attribute__((weak)) void oepWithI2c() {
  if (kRoomy) oepAddI2c();
}
__attribute__((weak)) void oepWithSpi() {
  if (kRoomy) oepAddSpi();
}
__attribute__((weak)) void oepWithSwio() {
  if (kRoomy) oepAddSwio();
}
__attribute__((weak)) void oepWithSwioConsole() { oepAddSwioConsole(); }
__attribute__((weak)) void oepWithSettings() {
  if (kRoomy) oepAddSettings();
}
__attribute__((weak)) void oepWithLink() { oepAddLink(); }
__attribute__((weak)) void oepWithI2cController() {
  if (kRoomy) oepAddI2cController();
}
__attribute__((weak)) void oepWithSpiController() {
  if (kRoomy) oepAddSpiController();
}

// oep.probe.restart (oep-if-restart): off the bus first (kRestartDetachMs, Oep.h), then CH32RV.restart(). restart_max_ms:
// an estimate (the boot, the host's enumeration), to be measured on the bench.
static constexpr uint32_t kRestartMaxMs = 2000;
static void restartProbe() {
  USBDevice.end();
  delay(oep::kRestartDetachMs);
  oep::platformRestart();
}

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, kModel, id, oep::platformUnitId(id, sizeof id), 64);
  oep::describeChip(w);
  return w.ok() ? w.length() : 0;
}

void setup() {
  oep::platformUnitId(reinterpret_cast<uint8_t *>(serial), sizeof serial);
  CH32RVUsbDeviceConfig usb;
  usb.vid = oep::reg::kUsbProjectVid;   // the project's VID:PID (registry usb, PID-USE.md)
  usb.pid = oep::reg::kUsbProjectPid;
  usb.manufacturer = "Open Embedded Probe";
  usb.product = kProduct;
  usb.serialNumber = serial;   // the unit id, lowercase (transports §3)
  // every channel a floating input until the host takes one (oep-core §8): before the transports, a UART bridge's pins
  // among them
  oep::platformParkMask(kChannels);
  oepWithBulk();   // the transports before USBDevice.begin (their interfaces laid out in this order)
  oepWithHid();
  oepWithCdc();
  oepWithBridge();
  USBDevice.begin(usb);
  afterUsb.run();
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setRestart(restartProbe, kRestartMaxMs);
  // The fns in this order, whichever are in (a build's set numbers them; a host finds each by name); the settings last,
  // after the ones they hold
  oepWithRvswd();   // and the console's choice
  oepWithSwd();
  oepWithGpio();
  oepWithUart();
  oepWithI2c();
  oepWithSwio();   // and its console's choice
  oepWithLink();
  oepWithI2cController();
  oepWithSpiController();
  oepWithSpi();
  oepWithSettings();
}

void loop() {
  deliverEvents();   // the HID output reports' callback runs only here (the core's 0.3.0)
  endpoint.poll();
  loopWork.run();
}
