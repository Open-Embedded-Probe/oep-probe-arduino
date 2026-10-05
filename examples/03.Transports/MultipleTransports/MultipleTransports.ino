// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// One probe, four ways in: an ESP32-P4 answering the same endpoint on four USB transports at once (oep-spec
// docs/oep-core.ja.md §3, §7.5; probe-development-guide §8). A host uses whichever it can open:
//
//   0 vendor bulk      the HS port, the fastest (libusb)                     OEP only        (the endpoint's own)
//   1 USB-Serial/JTAG  the P4's built-in port (the upload port)               a serial port   addTransport(Serial)
//   2 HID              the HS port, vendor HID reports (no driver anywhere)   OEP only        addTransport(hid)
//   3 USB CDC          the HS port, a serial port                             a serial port   addTransport(cdc)
//
// What each transport is goes into oep.core's describe (transport: index, kind, USB interface), so a host that found
// the probe one way knows the others. The two serial ports carry OEP frames as 0x00 <COBS> 0x00; what else they carry
// outside the frames is their bind (06.Settings/ProbeConfig). Vendor bulk and HID carry length-prefixed messages.
//
// The HS vendor bulk interface is written directly (build_opt.h: CFG_TUD_VENDOR_TXRX_BUFFERED=0 and friends; compile
// with --clean after changing it): oep::DirectBulkStream hands whole frames to the USB stack. The HS device says
// iProduct "OEP ..." (free text; starting "OEP" is a host's temporary clue until the project's VID:PID, host guide §4)
// and a serial number of the chip's MAC (the unit_id, how a host finds a probe named by it, core §3.3).
//
// One interface, oep.fixture.gpio, so there is something to use; add yours the same way.
#include <esp_mac.h>
#include <soc/usb_serial_jtag_reg.h>
#include <EspUsbDevice.h>

#include <OepDirectBulkStream.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepPinTable.h>
#include <OepPlatform.h>
#include "UsbStreams.h"

// The HS device's functions are created before it starts; interface numbers follow the class order: HID 0, vendor 1,
// CDC 2-3.
static EspUsbDevice usbDevice;
static EspUsbDeviceVendor vendor(usbDevice);
static oep::DirectBulkStream bulk(vendor);
static EspUsbDeviceHidVendor hid(usbDevice, 511);
static HidStream hidStream(hid);
static EspUsbDeviceCdcSerial cdc(usbDevice, "OEP");
static CdcStream cdcStream(cdc);

// Every transport has its own receive buffer; the result buffer is shared (one request is answered at a time).
static uint8_t rxVendor[1024], rxUsj[1100], rxHid[1024], rxCdc[1100];   // a serial port holds the encoded frame
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(bulk, rxVendor, sizeof rxVendor, txBuffer, sizeof txBuffer, {1024, 4096, 8},
                              oep::Endpoint::kVendorBulk, 1);   // transport 0: vendor bulk, USB interface 1

static constexpr uint64_t kChannels = ((1ull << 55) - 1) & ~((1ull << 24) | (1ull << 25));   // not the USJ pair
static oep::PinTable pins(kChannels);
static oep::FixtureGpio gpio(pins, 0);
static uint8_t probeTlv[64];
static char serial_[20];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, "multiple-transports", id, oep::platformUnitId(id, sizeof id), 55, ~kChannels & ((1ull << 55) - 1));
  return w.ok() ? w.length() : 0;
}

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §3)
  REG_SET_BIT(USB_SERIAL_JTAG_CHIP_RST_REG, USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS);   // opening the port must not reset
  Serial.setTxTimeoutMs(0);   // a port nobody reads never stops loop()
  Serial.begin(115200);
  oep::platformParkMask(kChannels);   // every channel Hi-Z, no pull, before the first answer (core §8)

  oep::platformUnitId(reinterpret_cast<uint8_t *>(serial_), sizeof serial_);   // the USB serial is the unit id (core §3.3)
  EspUsbDeviceConfig usb;
  usb.vid = 0x303a;   // the board's default: a temporary USB ID, not for distribution (PID-USE.md)
  usb.pid = 0x0002;
  usb.manufacturer = "Open Embedded Probe";
  usb.product = "OEP multiple transports";   // free text; starting "OEP" is a host's temporary clue
  usb.serialNumber = serial_;
  usb.controller = EspUsbController::HighSpeed;
  usbDevice.begin(usb);
  bulk.begin();

  // transports 1-3 (0 is the endpoint's own): stream, receive buffer, kind, USB interface, and whether to flush after a
  // burst of results (streams that hold bytes until a packet fills: HID, CDC)
  endpoint.addTransport(Serial, rxUsj, sizeof rxUsj, oep::Endpoint::kUsbSerialJtag);
  endpoint.addTransport(hidStream, rxHid, sizeof rxHid, oep::Endpoint::kHid, 0, true);
  endpoint.addTransport(cdcStream, rxCdc, sizeof rxCdc, oep::Endpoint::kUsbCdc, 2, true);
  endpoint.setFlushAfterBurst(true);
  // describe discoverable stays 0: it is 1 only on the project's own USB VID:PID, none listed yet (core §3.3 / §7.5)

  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.add(gpio);
}

void loop() {
  endpoint.poll();   // every transport, in turn
}
