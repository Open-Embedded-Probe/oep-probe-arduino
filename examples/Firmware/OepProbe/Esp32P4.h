// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// ESP32-P4 (profile esp32p4). One endpoint, four transports (the describe lists them in this order):
//
//   0 vendor bulk      HS port (direct build: build_opt.h, compile with --clean)            OEP only
//   1 USB-Serial/JTAG  the P4's USJ (HWCDC Serial)                                           serial port: OEP + raw
//   2 HID              HS port, vendor HID reports                                           OEP only
//   3 USB CDC          HS port                                                               serial port: OEP + raw
//
// A serial port always takes OEP frames (0x00 <COBS> 0x00); its other bytes are what its bind carries (oep.probe.config:
// a slot's console, a fixture UART). The HS device is VID:PID 303a:0002 until the OEP PID is granted (PID-USE.md),
// iProduct "OEP probe (ESP32-P4)", serial MAC + "-hs" (one usbipd bind lasts across reflashes).
//
// Interfaces (revision 1): oep.core; oep.wire.rvswd + oep.target.riscv-dm + oep.target.console; oep.fixture.gpio /
// uart (x2) / capture (PARLIO: up to 16 channels, 2 ch 160 Msps / 8 ch 40 Msps / 16 ch 20 Msps); the ESP-IDF SPI / I2C
// devices io.github.ch32-riscv-ug.esp32.spi-target / i2c-target; oep.probe.config (saved in NVS); oep.fixture.analog
// (ADC1 on GPIO16-23, up to 4 channels, 46 kHz in all) and oep.fixture.capture-group (the analog with the logic). Every GPIO but the
// USB-Serial/JTAG pair (24, 25) may be the RVSWD pair, the reset line, or any fixture's pin - the host chooses.
#pragma once
#include <esp_mac.h>
#include <soc/usb_serial_jtag_reg.h>
#include <EspUsbDevice.h>

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
#include <OepTarget.h>
#include "UsbStreams.h"

static constexpr uint16_t kUsbVid = 0x303a, kUsbPid = 0x0002;   // until the OEP PID (pid.codes) is granted
static constexpr uint16_t kUnset = 0xfffe;                        // no pair chosen yet

// The HS device: functions are created before it starts, interface numbers follow the class order (HID 0, vendor 1,
// CDC 2-3).
static EspUsbDevice usbDevice;
static EspUsbDeviceVendor vendor(usbDevice);
static oep::DirectBulkStream bulk(vendor);
static EspUsbDeviceHidVendor hid(usbDevice, 511);
static HidStream hidStream(hid);
static EspUsbDeviceCdcSerial cdc(usbDevice, "OEP");   // a name to show; every CDC of the probe speaks OEP
static CdcStream cdcStream(cdc);

static uint8_t rxVendor[1024], rxUsj[1100], rxHid[1024], rxCdc[1100];   // serial ports: cobsFrameMax(1024)
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(bulk, rxVendor, sizeof rxVendor, txBuffer, sizeof txBuffer, {1024, 4096, 8},
                              oep::Endpoint::kVendorBulk, 1);

// Reserved: GPIO24/25 (USB-Serial/JTAG). Every other GPIO is a channel the host may give to anything.
static constexpr uint64_t kReserved = (1ull << 24) | (1ull << 25);
static constexpr uint64_t kChannels = ((1ull << 55) - 1) & ~kReserved;
static oep::PinTable pins(kChannels);

static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort rvswd{dm, kUnset, kUnset};
static oep::WireRvswd wire(rvswd, 1);
static oep::TargetRiscvDm riscvDm(rvswd, 1);
static oep::DmConsole consoleDriver(dm, phy);   // the DM console framings (SDI / DMDATA / dmseq)
static oep::TargetConsoleStream console(rvswd, consoleDriver, 1);

static oep::FixtureGpio gpio(pins, 1, 1);
// PinTable owners: gpio 1, uart1 2, uart2 5 (the I2C device is 3, the SPI device 6, the RVSWD wire 0xf0)
static oep::FixtureUart uart1(pins, Serial1, 1, 2), uart2(pins, Serial2, 2, 5);
static oep::LogicCapture capture(endpoint, kReserved, 4);   // PARLIO RX; its lines are never driven
static oep::P4I2cTarget i2c(pins);
static oep::P4SpiTarget spi(pins);
static oep::Binds binds;
static oep::ProbeConfig config(endpoint, binds);
// ADC1 (GPIO16-23) in continuous mode, and a group that starts it with the PARLIO capture
static constexpr uint64_t kAdc1 = 0xffull << 16;
static oep::AnalogCapture analog(endpoint, kAdc1, 1);
static oep::CaptureGroup group(endpoint, 1);
static uint8_t probeTlv[160];
static char serial_[20];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];
  oep::describeCore(w, "esp32-p4", id, oep::platformUnitId(id, sizeof id), 55, kReserved);
  oep::describeChip(w);   // the MCU and its revision (a capture records what it was taken on)
  return w.ok() ? w.length() : 0;
}

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §2.5)
  // USB-Serial/JTAG: opening and closing the port must not reset the probe (probe guide §3.7)
  REG_SET_BIT(USB_SERIAL_JTAG_CHIP_RST_REG, USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS);
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.setTxTimeoutMs(0);   // a port nobody reads never stops loop()
  Serial.begin(115200);

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BASE);
  snprintf(serial_, sizeof serial_, "%02x%02x%02x%02x%02x%02x-hs", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  EspUsbDeviceConfig usb;
  usb.vid = kUsbVid;
  usb.pid = kUsbPid;
  usb.manufacturer = "Open Embedded Probe";
  usb.product = "OEP probe (ESP32-P4)";   // iProduct "OEP...": how discovery knows the probe (core §3.3)
  usb.serialNumber = serial_;
  usb.controller = EspUsbController::HighSpeed;
  usbDevice.begin(usb);
  bulk.begin();
  endpoint.setFlushAfterBurst(true);
  endpoint.addTransport(Serial, rxUsj, sizeof rxUsj, oep::Endpoint::kUsbSerialJtag);
  endpoint.addTransport(hidStream, rxHid, sizeof rxHid, oep::Endpoint::kHid, 0, true);
  endpoint.addTransport(cdcStream, rxCdc, sizeof rxCdc, oep::Endpoint::kUsbCdc, 2, true);
  endpoint.setRawPorts(&binds);
  endpoint.setOepPid(true);   // describe oep_pid: listed by discovery (until the PID: by the iProduct starting "OEP")

  rvswd.pin_choice = kChannels;
  rvswd.pins = &pins;
  rvswd.reset_allowed = kChannels;   // attach_under_reset: the channel the host names (no default), nobody holding it
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(esp_random());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  endpoint.add(console);
  // The channels are left as the P4 boots them (inputs, nothing driven) until the host takes one.
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
  config.load();
  config.applySaved();
  endpoint.add(analog);   // after config: the fns before it keep their numbers
  endpoint.add(group);
  group.addTrack(capture, capture);
  group.addTrack(analog, analog, 1400000);   // its first value comes a conversion frame after the start
}

void loop() {
  endpoint.poll();
  console.poll();
  config.poll();
  uart1.poll();
  uart2.poll();
  capture.poll();
  i2c.service();
  spi.service();
  analog.poll();
  group.poll();
}
