// OEP v1 logic capture probe on an ESP32-P4 with nothing wired (oep-spec docs/oep-if-capture.ja.md): OEP on the HS
// OTG vendor bulk interface (EspUsbDevice), direct build (build_opt.h: CFG_TUD_VENDOR_TXRX_BUFFERED=0; compile with
// --clean): results through oep::DirectBulkStream, capture streaming zero-copy through the same transport (2 ch at
// 160 Msps, logic-capture §7.9). oep.test.signal puts an LEDC square on a pin so the capture has something to see.
// USB identity 303a:4021, serial = the board MAC + "-hs", so one usbipd bind lasts across reflashes. USB-Serial/JTAG
// stays for uploads and a status line. Host: host/stream_test.py.
#include <esp_mac.h>
#include <EspUsbDevice.h>
#include <OepDirectBulkStream.h>
#include <OepPlatform.h>
#include <OepV1Capture.h>
#include <OepV1Endpoint.h>
#include "TestSignal.h"

static EspUsbDevice usbDevice;
static EspUsbDeviceVendor vendor(usbDevice);
static oep::DirectBulkStream bulk(vendor);
static uint8_t rxBuffer[16384];  // a whole max_frame request (link_sink sends full frames)
static uint8_t txBuffer[16384];
static oep::v1::Endpoint endpoint(bulk, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {16384, 16384, 16});
// board pull-ups (7, 8), the old 8-wire link (24, 25), straps (35), UART0 (37, 38), the LED (51)
static constexpr uint64_t kReserved = (1ull << 7) | (1ull << 8) | (1ull << 24) | (1ull << 25) | (1ull << 35) |
                                      (1ull << 37) | (1ull << 38) | (1ull << 51);
static oep::v1::LogicCapture capture(endpoint, kReserved, 0);
static TestSignal signal_;
static uint8_t probeTlv[160];
static char serial_[20];

static size_t describeProbe() {
  oep::v1::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];
  oep::v1::describeCore(w, "esp32-p4", id, oep::platformUnitId(id, sizeof id), 55, kReserved);
  w.label(51, "LED");
  return w.ok() ? w.length() : 0;
}

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);
  Serial.setTxTimeoutMs(0);   // nobody may be reading USB-Serial/JTAG: a write there must never stall loop()
  Serial.begin(115200);
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BASE);   // the same MAC board-identify names the board by
  snprintf(serial_, sizeof serial_, "%02x%02x%02x%02x%02x%02x-hs", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  EspUsbDeviceConfig config;
  config.vid = 0x303a;
  config.pid = 0x4021;
  config.manufacturer = "ch32-riscv-ug";
  config.product = "OEP probe (P4 HS)";
  config.serialNumber = serial_;
  config.controller = EspUsbController::HighSpeed;
  const bool ok = usbDevice.begin(config);
  const bool direct = bulk.begin();   // needs the direct build; results and pushes then go through writeDirect()
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(esp_random());
  endpoint.setPushQueue(4096);
  if (direct) endpoint.setDirect(&bulk);
  endpoint.setFlushAfterBurst(true);
  endpoint.add(capture);
  endpoint.add(signal_);
  Serial.printf("# OEP P4 capture probe usb_begin=%d direct=%d serial=%s\n", ok ? 1 : 0, direct ? 1 : 0, serial_);
}

void loop() {
  capture.poll();
  endpoint.poll();
}
