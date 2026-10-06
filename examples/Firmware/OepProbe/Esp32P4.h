// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// ESP32-P4 (profile esp32p4). One endpoint, four transports (the describe lists them in this order):
//
//   0 vendor bulk      HS port (direct build: build_opt.h, compile with --clean)            OEP only
//   1 USB-Serial/JTAG  the P4's USJ (HWCDC Serial)                                           serial port: OEP + raw
//   2 HID              HS port, vendor HID reports                                           OEP only
//   3 USB CDC          HS port                                                               serial port: OEP + raw
//
//   (and a DFU interface on the HS port: the probe's own firmware update, outside OEP, part of the same USB device)
//
// Updating the firmware over the HS port alone: `dfu-util -D OepProbe-esp32p4-<version>.bin` (the release's app image)
// writes the other app partition, checks it and restarts into it; settings (NVS) stay. The new firmware counts as good
// once the HS port has enumerated; until then the bootloader goes back to the one before at the next reset. The DFU
// interface takes no endpoint (EP0 only). A first flash of an empty chip, or a recovery, is esptool on USB-Serial/JTAG
// with the merged image.
//
// A serial port always takes OEP frames (0x00 <COBS> 0x00); its other bytes are what its bind carries (oep.probe.config:
// a slot's console, a fixture UART). The HS device is the project's VID:PID 1209:4F45 (registry usb; PID-USE.md), serial =
// the unit id (the MAC, lowercase hex); its iProduct "OEP probe (ESP32-P4)" is a name for people. USB-Serial/JTAG keeps the
// chip's fixed ID: a host reaches it by the user choosing its port. describe discoverable is 1 once the HS port has
// enumerated (a board with only USB-Serial/JTAG wired never gets there and says 0).
// How a host tells the ports apart inside a device known to be OEP (transports §3, registry usb): the vendor bulk interface is class 0xFF, subclass 0x4F
// ('O'), protocol 0x45 ('E'); the HID's report descriptor says usage page 0xFF4F, usage 0x45. EspUsbDevice writes 0 / 0
// and 0xFF00 / 1 itself, so the two functions below patch their descriptors.
//
// Interfaces (revision 1): fn 0 (the core); oep.wire.rvswd + oep.target.riscv-dm + oep.target.console; oep.wire.swio (WCH
// CH32V00x, one wire: its connections go through the same oep.target.riscv-dm, its console is oep.target.console instance 1,
// and a slot may name it: the second place); oep.fixture.gpio /
// uart (x2) / capture (PARLIO: up to 16 channels, 2 ch 160 Msps / 8 ch 40 Msps / 16 ch 20 Msps); the ESP-IDF SPI / I2C
// devices oep.fixture.spi-target / i2c-target; oep.probe.config (saved in NVS); oep.fixture.analog
// (ADC1 on GPIO16-23, up to 4 channels, 46 kHz in all) and oep.fixture.capture-group (the analog with the logic);
// oep.probe.link; oep.probe.plan and oep.probe.restart (the endpoint's own, listed last). Every GPIO but the
// USB-Serial/JTAG pair (24, 25) may be the RVSWD pair, the SWIO pin, the reset line, or any fixture's pin - the host chooses.
#pragma once
#include <esp_mac.h>
#include <soc/usb_serial_jtag_reg.h>
#include <EspUsbDevice.h>
#include <device/usbd.h>   // tud_disconnect (EspUsbDevice's TinyUSB)

#include <OepAnalog.h>
#include <OepBind.h>
#include <OepCapture.h>
#include <OepCaptureGroup.h>
#include <OepCh32Dm.h>
#include <OepConfig.h>
#include <OepConsole.h>
#include <OepDirectBulkStream.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepP4I2cTarget.h>
#include <OepP4SpiTarget.h>
#include <OepPinTable.h>
#include <OepRvswdPhy.h>
#include <OepSwioPhy.h>
#include <OepTarget.h>
#include "UsbStreams.h"

static constexpr uint16_t kUnset = 0xfffe;                        // no pair chosen yet

// The vendor bulk function with OEP's subclass / protocol in its interface descriptor (transports §3).
class OepVendor final : public EspUsbDeviceVendor {
 public:
  using EspUsbDeviceVendor::EspUsbDeviceVendor;
  uint16_t configurationDescriptor(uint8_t *dst, uint8_t interfaceNumber, uint8_t endpointNumber, uint16_t endpointSize) override {
    const uint16_t n = EspUsbDeviceVendor::configurationDescriptor(dst, interfaceNumber, endpointNumber, endpointSize);
    if (n >= 9) { dst[6] = oep::reg::kUsbVendorBulkSubclass; dst[7] = oep::reg::kUsbVendorBulkProtocol; }   // bInterfaceSubClass, bInterfaceProtocol
    return n;
  }
};
// The vendor HID with OEP's usage page / usage at the top of its report descriptor (transports §3).
class OepHid final : public EspUsbDeviceHidVendor {
 public:
  using EspUsbDeviceHidVendor::EspUsbDeviceHidVendor;
  const uint8_t *hidReportDescriptor() const override {
    const uint8_t *base = EspUsbDeviceHidVendor::hidReportDescriptor();
    const uint16_t n = EspUsbDeviceHidVendor::hidReportDescriptorLength();
    if (!base || n < 5 || n > sizeof patched_) return base;
    memcpy(patched_, base, n);
    patched_[1] = oep::reg::kUsbHidUsagePage & 0xff;   // 0x06 page(u16): the library's 0xFF00 becomes 0xFF4F
    patched_[2] = oep::reg::kUsbHidUsagePage >> 8;
    patched_[4] = oep::reg::kUsbHidUsage;              // 0x09 usage: 0x45
    return patched_;
  }

 private:
  mutable uint8_t patched_[64];
};

// The HS device: functions are created before it starts, interface numbers follow the class order (HID 0, vendor 1,
// CDC 2-3, DFU 4).
static EspUsbDevice usbDevice;
static OepVendor vendor(usbDevice);
static oep::DirectBulkStream bulk(vendor);
static OepHid hid(usbDevice, 511);
static HidStream hidStream(hid);
static EspUsbDeviceCdcSerial cdc(usbDevice, "OEP");   // a name to show; every CDC of the probe speaks OEP
static CdcStream cdcStream(cdc);
static EspUsbDeviceDfu dfu(usbDevice, EspUsbDeviceDfuMode::Download, "OEP probe firmware");

// The new image after a DFU update is on trial (the bootloader's rollback, CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE):
// Arduino would confirm it at boot; this confirms it in loop() once the HS port has enumerated - a firmware that got
// that far can take the next DFU update, so it is never left without a way back.
extern "C" bool verifyRollbackLater() { return true; }

static uint8_t rxVendor[1024], rxUsj[1100], rxHid[1024], rxCdc[1100];   // serial ports: cobsFrameMax(1024)
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(bulk, rxVendor, sizeof rxVendor, txBuffer, sizeof txBuffer, {1024, 4096, 8},
                              oep::Endpoint::kVendorBulk, 1);
static oep::Link oepLink(endpoint);   // oep.probe.link (oep-if-link)

// Reserved: GPIO24/25 (USB-Serial/JTAG). Every other GPIO is a channel the host may give to anything.
static constexpr uint64_t kReserved = (1ull << 24) | (1ull << 25);
static constexpr uint64_t kChannels = ((1ull << 55) - 1) & ~kReserved;
static oep::PinTable pins(kChannels);

static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort rvswd{dm, kUnset, kUnset};
static oep::WireRvswd wire(rvswd, 0);
static oep::TargetRiscvDm riscvDm(rvswd, 0);
static oep::DmConsole consoleDriver(dm, phy);   // the DM console framings (SDI / DMDATA / dmseq)
static oep::TargetConsoleStream console(rvswd, consoleDriver, 0);
// The one-wire link (CH32V00x): its own wire fn, its connections served by riscvDm above (addPort in setup).
static oep::SwioPhy swioPhy;
static oep::Ch32Dm swioDm(swioPhy);
static oep::DebugPort swio{swioDm, kUnset, 0xffff};   // one wire: swclk stays 0xffff
static oep::WireRvswd swioWire(swio, 0, "oep.wire.swio");
static oep::DmConsole swioConsoleDriver(swioDm, swioPhy);
static oep::TargetConsoleStream swioConsole(swio, swioConsoleDriver, 1);   // the one-wire link's own console

static oep::FixtureGpio gpio(pins, 0, 1);
// PinTable owners: gpio 1, uart1 2, uart2 5 (the I2C device is 3, the SPI device 6, the RVSWD wire 0xf0, the SWIO wire
// 0xf2, the analog 7)
static oep::FixtureUart uart1(pins, Serial1, 0, 2), uart2(pins, Serial2, 1, 5);
static oep::LogicCapture capture(endpoint, pins, 0);   // PARLIO RX; its lines are never driven
static oep::P4I2cTarget i2c(pins);
static oep::P4SpiTarget spi(pins);
static oep::Binds binds;
static oep::ProbeConfig config(endpoint, binds);
// ADC1 (GPIO16-23) in continuous mode, and a group that starts it with the PARLIO capture
static constexpr uint64_t kAdc1 = 0xffull << 16;
static oep::AnalogCapture analog(endpoint, kAdc1, 0);
static oep::CaptureGroup group(endpoint, 0);
static uint8_t probeTlv[160];
static char serial_[20];

// oep.probe.restart (oep-if-restart): the HS device detaches first so the host records an unplug rather than a device that went
// silent, waits oep::kRestartDetachMs for the host to see it (20 ms, as EspUsbDevice's own restarts wait, left the WeAct
// P4 failing its device descriptor request after the restart until a replug, bench 0.0.29-dev+3c0cd99), then
// esp_restart - within restart_after_answer_ms of the answer (Oep.h). restart_max_ms (its describe): the chip is in
// setup() after about 0.5 s (the ROM, the bootloader checking the app image of about 0.6 MB with rollback on, the
// PSRAM); then the host enumerates the HS device again - a composite of HID, vendor bulk, CDC and DFU, for which an OS
// binds four drivers (Windows about 1 s or more) - and the transport opens again before it confirms (USB-Serial/JTAG,
// whose own device may stay on the bus, is back no later). 3000 ms is about twice the slow end (an estimate from the
// boot path, to be measured on the bench).
static constexpr uint32_t kRestartMaxMs = 3000;
static void restartProbe() {
  tud_disconnect();
  delay(oep::kRestartDetachMs);
  esp_restart();
}

// A DFU update's restart into the new image, done here and not by EspUsbDevice (restartWhenComplete off): its restart
// detached and reset 20 ms later, and the P4 came back failing its device descriptor request until a replug (bench, as
// oep.probe.restart's above). The host's last GETSTATUS is answered first (kDfuStatusMs, as EspUsbDevice waits), then
// the device goes off the bus for kDfuDetachMs before esp_restart (no answer of OEP's is waiting: no time limit).
static constexpr uint32_t kDfuStatusMs = 500, kDfuDetachMs = 100;
static volatile bool dfuDone = false;
static volatile uint32_t dfuDoneMs = 0;
static void restartAfterDfu() {
  if (!dfuDone || millis() - dfuDoneMs < kDfuStatusMs) return;
  tud_disconnect();
  delay(kDfuDetachMs);
  esp_restart();
}

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, "esp32p4", id, oep::platformUnitId(id, sizeof id), 55, kReserved);
  oep::describeChip(w);   // the MCU and its revision (a capture records what it was taken on)
  return w.ok() ? w.length() : 0;
}

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §3)
  // USB-Serial/JTAG: opening and closing the port must not reset the probe (probe guide §7)
  REG_SET_BIT(USB_SERIAL_JTAG_CHIP_RST_REG, USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS);
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.setTxTimeoutMs(0);   // a port nobody reads never stops loop()
  Serial.begin(115200);

  oep::platformUnitId(reinterpret_cast<uint8_t *>(serial_), sizeof serial_);   // the USB serial is the unit id (transports §3)
  EspUsbDeviceConfig usb;
  usb.vid = oep::reg::kUsbProjectVid;   // the project's VID:PID (registry usb, PID-USE.md)
  usb.pid = oep::reg::kUsbProjectPid;
  usb.manufacturer = "Open Embedded Probe";
  usb.product = "OEP probe (ESP32-P4)";   // a name for people; no host identifies the probe by it
  usb.serialNumber = serial_;
  usb.controller = EspUsbController::HighSpeed;
  dfu.restartWhenComplete(false);   // restartAfterDfu, from loop()
  dfu.onComplete([]() {             // the usbd task: the image verified, the host's last GETSTATUS still to answer
    dfuDoneMs = millis();
    dfuDone = true;
    return true;
  });
  usbDevice.begin(usb);
  bulk.begin();
  endpoint.setFlushAfterBurst(true);
  endpoint.addTransport(Serial, rxUsj, sizeof rxUsj, oep::Endpoint::kUsbSerialJtag);
  endpoint.addTransport(hidStream, rxHid, sizeof rxHid, oep::Endpoint::kHid, 0, true);
  endpoint.addTransport(cdcStream, rxCdc, sizeof rxCdc, oep::Endpoint::kUsbCdc, 2, true);
  endpoint.setRawPorts(&binds);
  // describe discoverable: set in loop() once the HS port has enumerated with the project's VID:PID (transports §3, core §7.5)
  endpoint.setRestart(restartProbe, kRestartMaxMs);

  rvswd.pin_choice = kChannels;
  rvswd.pins = &pins;
  rvswd.reset_allowed = kChannels;   // attach's reset TLV: the channel the host names (no default), nobody holding it
  swio.pin_choice = kChannels;
  swio.pins = &pins;
  swio.pin_owner = 0xf2;
  swio.reset_allowed = kChannels;
  riscvDm.addPort(swio);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  endpoint.add(console);
  endpoint.add(gpio);
  endpoint.add(uart1);
  endpoint.add(uart2);
  endpoint.add(capture);
  endpoint.add(i2c);
  endpoint.add(spi);
  endpoint.add(config);   // last: the fns before it keep their numbers
  config.addPlace(wire, console);
  config.addUart(uart1);
  config.addUart(uart2);
  config.setPins(&pins);   // the idle item sets these pins' free state
  analog.setPins(&pins, 7);   // its pads go analog: claimed against wires and settings
  endpoint.add(analog);   // after config: the fns before it keep their numbers
  endpoint.add(group);
  group.addTrack(capture, capture);
  group.addTrack(analog, analog, 1400000);   // its first value comes a conversion frame after the start
  endpoint.add(swioWire);   // after the group: the fns before it keep their numbers
  endpoint.add(swioConsole);
  config.addPlace(swioWire, swioConsole);   // a slot on the one wire (CH32V00x): the second place
  endpoint.add(oepLink);   // oep.probe.link (the link test), last: the fns before it keep their numbers
  // Last, once every interface is added: the saved settings name fns, and are kept only for the same interface list
  // (applied before the analog and the group were added, they never matched it: unreadable after every reboot, 0.0.11-0.0.16).
  config.load();
  // Every channel Hi-Z, no pull, before the first answer (core §8: the P4 boots some pins with a pull), but the saved
  // disable items' channels, which are never touched (probe.config §2: applied before any idle / park; applySaved
  // gives them back if the settings are not applied).
  pins.setDisabled(config.savedDisabled());
  oep::platformParkMask(kChannels & ~pins.disabledMask());
  // In the order of probe.config §2: every idle (outputs driven) first, then the plans, the uarts, and the at-boot
  // slots' attach last (on its poll), so a target powered through an output idle is up before it.
  config.applySaved();
}

void loop() {
  static bool confirmed = false;
  if (!confirmed && usbDevice.ready()) {   // the HS port enumerated (configured by a host)
    confirmed = true;
    EspUsbDeviceFirmwareUpdate::markValid();   // this firmware is good (see verifyRollbackLater)
    endpoint.setDiscoverable(true);            // the probe enumerates with the project's VID:PID (core §7.5)
  }
  restartAfterDfu();
  endpoint.poll();
  console.poll();
  swioConsole.poll();
  config.poll();
  uart1.poll();
  uart2.poll();
  capture.poll();
  i2c.service();
  spi.service();
  analog.poll();
  group.poll();
}
