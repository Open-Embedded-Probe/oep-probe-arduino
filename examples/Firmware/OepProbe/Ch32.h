// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// WCH CH32 on ArduinoCore-CH32RV (profile ch32x035; the core's 0.2.1).
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
#pragma once
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

// The vendor class as the endpoint's Stream: it has no peek in the core's 0.2.1, so one byte is held back for it.
class VendorStream final : public Stream {
 public:
  explicit VendorStream(CH32RVUsbDeviceVendor &vendor) : vendor_(vendor) {}
  int available() override { return vendor_.available() + (held_ >= 0 ? 1 : 0); }
  int read() override {
    if (held_ >= 0) { const int c = held_; held_ = -1; return c; }
    return vendor_.read();
  }
  int peek() override {
    if (held_ < 0) held_ = vendor_.read();
    return held_;
  }
  size_t write(uint8_t c) override { return vendor_.write(c); }
  size_t write(const uint8_t *data, size_t length) override { return vendor_.write(data, length); }
  void flush() override { vendor_.flush(); }

 private:
  CH32RVUsbDeviceVendor &vendor_;
  int held_ = -1;
};

static CH32RVUsbDeviceVendor usbVendor(USBDevice, 64);
static VendorStream vendorStream(usbVendor);

static uint8_t rxBuffer[520];
static uint8_t txBuffer[512];
static oep::Endpoint endpoint(vendorStream, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {512, 1024, 2},
                              oep::Endpoint::kVendorBulk, 0);
static oep::Link oepLink(endpoint);   // oep.probe.link (oep-if-link)
static uint8_t probeTlv[160];
static char serial[17];

// The channels (platformPad: port x 24 + bit): PA0-PA23, PB0-PB23, PC0-PC15. The CH32X035's USB pads (PC16 / PC17) and
// its own SWD (PC18 / PC19) are past channel 63. Which of these a package brings out is the board's; a pad it does not
// is left alone by the host.
static constexpr uint64_t kChannels = ~0ull;
static constexpr uint16_t kUnset = 0xfffe;   // no pair chosen yet
static oep::PinTable pins(kChannels);

static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort rvswd{dm, kUnset, kUnset};
static oep::WireRvswd wireRvswd(rvswd, 0);
static oep::TargetRiscvDm riscvDm(rvswd, 0);
static oep::DmConsole consoleDriver(dm, phy);
static oep::TargetConsoleStream console(rvswd, consoleDriver, 0);

static oep::SwdPort swd{kUnset, kUnset};
static oep::WireSwd wireSwd(swd, 0);
static oep::TargetArmAdi adi(swd, 0);

static oep::FixtureGpio gpio(pins, 0, 1);
static oep::FixtureUart uart(pins, Serial, 0, 2);
static oep::Binds binds;
static oep::ProbeConfig config(endpoint, binds);

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
  USBDevice.begin(usb);
  config.setStorage(false);   // no store on this core yet (probe.config §4: no save / erase / state)
  oep::platformParkMask(kChannels);   // every channel a floating input until the host takes one (oep-core §8)
  rvswd.pin_choice = kChannels;
  rvswd.pins = &pins;
  rvswd.reset_allowed = kChannels;
  swd.pin_choice = kChannels;
  swd.pins = &pins;
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setRestart(restartProbe, kRestartMaxMs);
  endpoint.add(wireRvswd);
  endpoint.add(riscvDm);
  endpoint.add(console);
  endpoint.add(wireSwd);
  endpoint.add(adi);
  endpoint.add(gpio);
  endpoint.add(uart);
  uart.setRoleChannels(kChannels, kChannels);   // any enabled USART's route (its begin refuses another pair)
  endpoint.setRawPorts(&binds);
  endpoint.add(config);   // last of these: the fns before it keep their numbers
  config.addPlace(wireRvswd, console);
  config.addHold(swd.hold, oep::checkSwdHold, &swd);
  config.addUart(uart);
  config.setPins(&pins);
  endpoint.add(oepLink);
  config.applySaved();
}

void loop() {
  endpoint.poll();
  console.poll();
  uart.poll();
  config.poll();
}
