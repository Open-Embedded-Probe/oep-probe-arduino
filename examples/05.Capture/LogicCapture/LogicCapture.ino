// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A logic analyzer: oep.fixture.logic alone on an ESP32-P4, at the P4's full speed (oep-spec docs/oep-if-capture.ja.md).
// Up to 16 channels on any pins the host plans, one-shot / repeat / streaming: 2 ch at 160 Msps, 8 ch at 40 Msps, 16 ch
// at 20 Msps.
//
// How it goes that fast:
//   - OEP runs on the HS OTG vendor bulk interface (EspUsbDevice) in a direct build (build_opt.h:
//     CFG_TUD_VENDOR_TXRX_BUFFERED=0; compile with --clean): results go through oep::DirectBulkStream, and a streaming
//     capture's pushes go zero-copy from the PARLIO DMA buffers through the same transport (logic-capture §7.9).
//   - 16 KiB frames (confirm's max_frame): one bulk transfer per push.
//
// With nothing wired, io.github.open-embedded-probe.test-signal (this example's own interface, TestSignal.h) puts an
// LEDC square on a pin so the capture has something to see. The USB device is the project's VID:PID 1209:4F45 (registry usb;
// PID-USE.md), serial = the unit id (the board MAC, lowercase hex); its iProduct "OEP capture (ESP32-P4)" is a name for people.
// USB-Serial/JTAG stays for uploads and a status line. Host: host/stream_test.py (streams at a rate, checks every edge).
#include <esp_mac.h>
#include <EspUsbDevice.h>
#include <OepDirectBulkStream.h>
#include <OepPlatform.h>
#include <OepCapture.h>
#include <OepEndpoint.h>
#include "TestSignal.h"

static EspUsbDevice usbDevice;
static EspUsbDeviceVendor vendor(usbDevice);
static oep::DirectBulkStream bulk(vendor);
static uint8_t rxBuffer[16384];  // a whole max_frame request (link_sink sends full frames)
static uint8_t txBuffer[16384];
static oep::Endpoint endpoint(bulk, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {16384, 16384, 8},
                                  oep::Endpoint::kVendorBulk, 0);
// board pull-ups (7, 8), the old 8-wire link (24, 25), straps (35), UART0 (37, 38), the LED (51)
static constexpr uint64_t kReserved = (1ull << 7) | (1ull << 8) | (1ull << 24) | (1ull << 25) | (1ull << 35) |
                                      (1ull << 37) | (1ull << 38) | (1ull << 51);
static oep::PinTable pins(((1ull << 55) - 1) & ~kReserved);   // the channels a plan may give the capture
static oep::LogicCapture capture(endpoint, pins, 0);
static TestSignal signal_;
static uint8_t probeTlv[160];
static char serial_[20];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, "logic-capture", id, oep::platformUnitId(id, sizeof id), 55, kReserved);
  oep::describeChip(w);   // the MCU and its revision (a capture records what it was taken on)
  w.label(51, "LED");
  return w.ok() ? w.length() : 0;
}

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);
  Serial.setTxTimeoutMs(0);   // nobody may be reading USB-Serial/JTAG: a write there must never stall loop()
  Serial.begin(115200);
  oep::platformParkMask(((1ull << 55) - 1) & ~kReserved);   // every channel Hi-Z, no pull, before the first answer (core §8)
  oep::platformUnitId(reinterpret_cast<uint8_t *>(serial_), sizeof serial_);   // the USB serial is the unit id (core §3.3)
  EspUsbDeviceConfig config;
  config.vid = oep::reg::kUsbProjectVid;   // the project's VID:PID (registry usb, PID-USE.md)
  config.pid = oep::reg::kUsbProjectPid;
  config.manufacturer = "Open Embedded Probe";
  config.product = "OEP capture (ESP32-P4)";   // a name for people; no host identifies the probe by it
  config.serialNumber = serial_;
  config.controller = EspUsbController::HighSpeed;
  const bool ok = usbDevice.begin(config);
  const bool direct = bulk.begin();   // needs the direct build; results and pushes then go through writeDirect()
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setPushQueue(4096);
  if (direct) endpoint.setDirect(&bulk);
  endpoint.setFlushAfterBurst(true);
  // describe discoverable 1: OEP's one transport here is the HS device, so a host that asks reached it through the
  // project's VID:PID (core §3.3 / §7.5)
  endpoint.setDiscoverable(true);
  endpoint.add(capture);
  endpoint.add(signal_);
  Serial.printf("# OEP P4 capture probe usb_begin=%d direct=%d serial=%s\n", ok ? 1 : 0, direct ? 1 : 0, serial_);
}

void loop() {
  capture.poll();
  endpoint.poll();
}
