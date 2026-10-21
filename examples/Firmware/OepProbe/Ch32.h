// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// WCH CH32 on ArduinoCore-CH32RV (profile ch32x035; the core's 0.2.2, LTO by default).
//
// Transport: the USB vendor bulk interface (class 0xFF, 'O', 'E': transports §3) at full speed, length-prefixed frames;
// the USB device is the project's VID:PID 1209:4F45 (registry usb; PID-USE.md), serial = the unit id (the chip's 64-bit
// unique id in hex); iProduct "OEP probe (CH32X035)" is a name for people. The core runs the USB stack from loop() and
// delay(): no other context, so the vendor class is read and written from loop() as a Stream.
//
// Interfaces (revision 1), as the RP2's (Rp2.h): fn 0 (the core); oep.wire.rvswd + oep.target.riscv-dm +
// oep.target.console (WCH CH32, 2 wires); oep.wire.swd + oep.target.arm-adi (ARM); oep.fixture.gpio / uart;
// oep.probe.config (no store yet: nothing saved, setStorage(false)); oep.probe.link; oep.probe.plan and
// oep.probe.restart (the endpoint's own). Every channel (platformPad: port x 24 + bit) may be SWDIO / SWCLK of either
// wire, the reset line of attach's reset TLV, a gpio or a UART pin (any enabled USART's route; another pair is
// refused at the UART's begin).
//
// The CH32X035 (62 KiB flash, 20 KiB RAM) builds with OEP_SMALL_RAM (Oep.h): the endpoint's tables cut, no serial
// transport, a 1 KiB console ring.
//
// Each interface is a build choice (the sketch's build_opt.h: -DOEP_CH32_SWD=0 ...; 1 = in, the default), so a chip
// takes the set that fits it (a serial transport brings the library's serial part in by itself):
//   OEP_CH32_RVSWD    oep.wire.rvswd + oep.target.riscv-dm
//   OEP_CH32_CONSOLE  oep.target.console on the RVSWD wire (needs OEP_CH32_RVSWD)
//   OEP_CH32_SWD      oep.wire.swd + oep.target.arm-adi
//   OEP_CH32_GPIO     oep.fixture.gpio
//   OEP_CH32_UART     oep.fixture.uart
//   OEP_CH32_CONFIG   oep.probe.config (no store yet)
//   OEP_CH32_LINK     oep.probe.link
//   OEP_CH32_HID      the USB HID transport too (usage page 0xFF4F, usage 0x45: transports §1, §3)
//   OEP_CH32_CDC      a USB CDC port too (a serial transport, COBS); 0 by default
//   OEP_CH32_BRIDGE   a UART bridge too (Serial1 on its default pins, 115200: a serial transport); 0 by default
#pragma once
#ifndef OEP_CH32_BRIDGE
#define OEP_CH32_BRIDGE 0
#endif
#ifndef OEP_CH32_HID
#define OEP_CH32_HID 1
#endif
#ifndef OEP_CH32_RVSWD
#define OEP_CH32_RVSWD 1
#endif
#ifndef OEP_CH32_CONSOLE
#define OEP_CH32_CONSOLE OEP_CH32_RVSWD
#endif
#ifndef OEP_CH32_SWD
#define OEP_CH32_SWD 1
#endif
#ifndef OEP_CH32_GPIO
#define OEP_CH32_GPIO 1
#endif
#ifndef OEP_CH32_UART
#define OEP_CH32_UART 1
#endif
#ifndef OEP_CH32_CONFIG
#define OEP_CH32_CONFIG 1
#endif
#ifndef OEP_CH32_LINK
#define OEP_CH32_LINK 1
#endif
#ifndef OEP_CH32_CDC
#define OEP_CH32_CDC 0
#endif
#if OEP_CH32_CONSOLE && !OEP_CH32_RVSWD
#error "OEP_CH32_CONSOLE needs OEP_CH32_RVSWD"
#endif
#include <CH32RV.h>
#include <CH32RVUsbDevice.h>

#include <OepBind.h>
#include <OepCh32Dm.h>
#include <OepConfig.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepPinTable.h>
#include <OepPlatform.h>
#include <OepRvswdPhy.h>
#include <OepSwd.h>
#include <OepTarget.h>

static constexpr const char *kProduct = "OEP probe (CH32X035)", *kModel = "ch32x035";

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

static CH32RVUsbDeviceVendor usbVendor(USBDevice, 64);

#if OEP_CH32_HID
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
static CH32RVUsbDeviceHidVendor usbHid(USBDevice, HidStream::kReport);
static HidStream hidStream(usbHid);
static uint8_t hidRx[520];
#endif
static VendorStream vendorStream(usbVendor);

static uint8_t rxBuffer[520];
static uint8_t txBuffer[512];
static oep::Endpoint endpoint(vendorStream, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {512, 1024, 2},
                              oep::Endpoint::kVendorBulk, 0);
#if OEP_CH32_CDC
static uint8_t cdcRx[oep::cobsFrameMax(512) + 8];   // the encoded candidate (transports §1)
#endif
#if OEP_CH32_BRIDGE
static uint8_t bridgeRx[oep::cobsFrameMax(512) + 8];
#endif
#if OEP_CH32_LINK
static oep::Link oepLink(endpoint);   // oep.probe.link (oep-if-link)
#endif
static uint8_t probeTlv[160];
static char serial[17];

// The channels (platformPad: port x 24 + bit): PA0-PA23, PB0-PB23, PC0-PC15. The CH32X035's USB pads (PC16 / PC17) and
// its own SWD (PC18 / PC19) are past channel 63. Which of these a package brings out is the board's; a pad it does not
// is left alone by the host.
static constexpr uint64_t kChannels = ~0ull;
static constexpr uint16_t kUnset = 0xfffe;   // no pair chosen yet
static oep::PinTable pins(kChannels);

#if OEP_CH32_RVSWD
static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort rvswd{dm, kUnset, kUnset};
static oep::WireRvswd wireRvswd(rvswd, 0);
static oep::TargetRiscvDm riscvDm(rvswd, 0);
#endif
#if OEP_CH32_CONSOLE
static oep::DmConsole consoleDriver(dm, phy);
static oep::TargetConsoleStream console(rvswd, consoleDriver, 0);
#endif
#if OEP_CH32_SWD
static oep::SwdPort swd{kUnset, kUnset};
static oep::WireSwd wireSwd(swd, 0);
static oep::TargetArmAdi adi(swd, 0);
#endif
#if OEP_CH32_GPIO
static oep::FixtureGpio gpio(pins, 0, 1);
#endif
#if OEP_CH32_UART
static oep::FixtureUart uart(pins, Serial, 0, 2);
#endif
#if OEP_CH32_CONFIG
static oep::Binds binds;
static oep::ProbeConfig config(endpoint, binds);
#endif

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
  usbVendor.setInterfaceProtocol(oep::reg::kUsbVendorBulkSubclass, oep::reg::kUsbVendorBulkProtocol);
#if OEP_CH32_HID
  usbHid.setUsage(oep::reg::kUsbHidUsagePage, oep::reg::kUsbHidUsage);   // 0xFF4F / 0x45 (registry usb)
#endif
#if OEP_CH32_CDC
  SerialUSB.begin(115200);   // before USBDevice.begin: its interface is laid out with the vendor one
#endif
  USBDevice.begin(usb);
#if OEP_CH32_HID
  endpoint.addTransport(hidStream, hidRx, sizeof hidRx, oep::Endpoint::kHid, 0xff, true);   // the device's one HID interface (the core numbers it)
#endif
#if OEP_CH32_BRIDGE
  Serial1.begin(115200);   // its default route (transports §1: a UART bridge at its boot speed)
  endpoint.addTransport(Serial1, bridgeRx, sizeof bridgeRx, oep::Endpoint::kUartBridge);
#endif
#if OEP_CH32_CDC
  endpoint.addTransport(SerialUSB, cdcRx, sizeof cdcRx, oep::Endpoint::kUsbCdc, SerialUSB.interfaceNumber(), true);
#endif
  oep::platformParkMask(kChannels);   // every channel a floating input until the host takes one (oep-core §8)
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setRestart(restartProbe, kRestartMaxMs);
  // The fns in this order, whichever are in (a build's set numbers them; a host finds each by name)
#if OEP_CH32_RVSWD
  rvswd.pin_choice = kChannels;
  rvswd.pins = &pins;
  rvswd.reset_allowed = kChannels;
  endpoint.add(wireRvswd);
  endpoint.add(riscvDm);
#endif
#if OEP_CH32_CONSOLE
  endpoint.add(console);
#endif
#if OEP_CH32_SWD
  swd.pin_choice = kChannels;
  swd.pins = &pins;
  endpoint.add(wireSwd);
  endpoint.add(adi);
#endif
#if OEP_CH32_GPIO
  endpoint.add(gpio);
#endif
#if OEP_CH32_UART
  endpoint.add(uart);
  uart.setRoleChannels(kChannels, kChannels);   // any enabled USART's route (its begin refuses another pair)
#endif
#if OEP_CH32_CONFIG
  config.setStorage(false);   // no store on this core yet (probe.config §4: no save / erase / state)
  endpoint.setRawPorts(&binds);
  endpoint.add(config);
#if OEP_CH32_CDC || OEP_CH32_BRIDGE
  config.usePassthroughs();   // a serial port's raw stream carries them (probe.config §1.1)
#endif
#if OEP_CH32_CONSOLE
  config.addPlace(wireRvswd, console);
#endif
#if OEP_CH32_SWD
  config.addHold(swd.hold, oep::checkSwdHold, &swd);
#endif
#if OEP_CH32_UART
  config.addUart(uart);
#endif
  config.setPins(&pins);
#endif
#if OEP_CH32_LINK
  endpoint.add(oepLink);
#endif
#if OEP_CH32_CONFIG
  config.applySaved();
#endif
}

void loop() {
  endpoint.poll();
#if OEP_CH32_CONSOLE
  console.poll();
#endif
#if OEP_CH32_UART
  uart.poll();
#endif
#if OEP_CH32_CONFIG
  config.poll();
#endif
}
