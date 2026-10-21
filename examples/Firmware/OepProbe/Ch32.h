// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// WCH CH32 on ArduinoCore-CH32RV (profile ch32x035; the core's 0.2.1). A first step: fn 0 (the core) and the endpoint's
// own interfaces (oep.probe.plan, oep.probe.restart) on the USB vendor bulk interface (class 0xFF, 'O', 'E': transports
// §3) at full speed - the size it takes on the CH32X035 (62 KiB flash, 20 KiB RAM) before the wires and fixtures come.
//
// The USB device is the project's VID:PID 1209:4F45 (registry usb; PID-USE.md), serial = the unit id (the chip's 64-bit
// unique id in hex); iProduct "OEP probe (CH32X035)" is a name for people. The core calls the USB stack from loop() and
// delay(): no other context, so the vendor class is read and written from loop() as a Stream.
#pragma once
#include <CH32RV.h>
#include <CH32RVUsbDevice.h>

#include <OepEndpoint.h>
#include <OepPlatform.h>

static constexpr const char *kProduct = "OEP probe (CH32X035)", *kModel = "ch32x035";

// The vendor class as the endpoint's Stream: it has no peek, so one byte is held back for it.
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
static uint8_t probeTlv[160];
static char serial[17];

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
  oep::describeCore(w, kModel, id, oep::platformUnitId(id, sizeof id), 0);
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
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setRestart(restartProbe, kRestartMaxMs);
}

void loop() { endpoint.poll(); }
