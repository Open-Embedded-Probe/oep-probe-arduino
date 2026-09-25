// Prototypes P1-P5 of oep-spec probe-cdc-and-persistence §7 on a P4 HS probe (no target needed; see README.ja.md):
//   P1/P2 one OEP endpoint - one session and lock - over vendor bulk (direct build), a vendor HID and a CDC port
//   P3    oep.fixture.uart forwarded to a data CDC port ("UART bridge"), the port's line coding reruns the UART
//   P4    oep.probe.config: boot modes that change the USB configuration, the plan and a bind kept in NVS
//         mode 0 "debug": vendor bulk + HID + CDC (OEP) + data port 0 + Mass Storage + DFU    mode 1 "logic": vendor + DFU
//         modes 2-5: vendor + HID / + CDC / + 2 or 3 data ports (P7: throughput per USB configuration)
//         With nothing saved this sketch starts in mode 0 (its own choice, not an OEP default).
//   P5    updating the probe itself (outside OEP): DFU download (dfu-util -D, EP0 only) in both modes, a Mass Storage
//         firmware disk (copy a .bin) in mode 0; both write the spare OTA partition and restart into it.
// Direct build (build_opt.h): compile with --clean after changing it.
// oep.test.bridge (scratch): 0x02 loopback(on u8, tx_pad u8) - UART1's RX from its TX pad inside the GPIO matrix;
// 0x03 status -> baud u32, gaps u32, line coding baud u32; 0x04 build -> text (this build's date and time).
#include <esp_mac.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/uart_periph.h>
#include <EspUsbDevice.h>
#include <OepDirectBulkStream.h>
#include <OepV1Config.h>
#include <OepV1Endpoint.h>
#include <OepV1Fixture.h>
#include "UsbStreams.h"

static EspUsbDevice usbDevice;
static EspUsbDeviceVendor vendor(usbDevice);
static oep::DirectBulkStream bulk(vendor);
static uint8_t rxA[16384], rxB[16384], rxC[16384], tx[16384];
static oep::v1::Endpoint endpoint(bulk, rxA, sizeof rxA, tx, sizeof tx, {16384, 16384, 16});

// Boot modes: what the HS device offers (P4, and P7's throughput per USB configuration). DFU (EP0 only) is in every
// mode. Data port 0 is the bridge; further data ports exist only to take their share of the device.
struct Build { bool hid, cdc; uint8_t data; bool msc; };
static const Build kBuilds[] = {
    {true, true, 1, true},     // 0 debug: vendor + HID + CDC (OEP) + data port 0 + Mass Storage
    {false, false, 0, false},  // 1 logic: vendor only
    {true, false, 0, false},   // 2 vendor + HID
    {true, true, 0, false},    // 3 vendor + HID + CDC (OEP)
    {true, true, 2, false},    // 4 + 2 data ports
    {true, true, 3, false},    // 5 + 3 data ports
};
static const oep::v1::ProbeConfig::Mode kModes[] = {
    {0x1f, 1, "debug"}, {0x11, 0, "logic"}, {0x15, 0, "vendor+hid"}, {0x17, 0, "vendor+hid+cdc"},
    {0x17, 2, "vendor+hid+cdc+2"}, {0x17, 3, "vendor+hid+cdc+3"},
};
static const uint8_t kPortInterfaces[] = {4, 6, 8};   // HID 0, vendor 1, CDC OEP 2-3, data port n = 4 + 2n
static oep::v1::ProbeConfig config(endpoint, kModes, 6, kPortInterfaces);

static const uint8_t kFixturePins[] = {0, 1, 2, 3, 4, 5, 6, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
static oep::PinTable pins(kFixturePins, sizeof kFixturePins);
static oep::v1::FixtureUart uart(pins, Serial1, 0, 2);

static EspUsbDeviceDfu *dfu = nullptr;
static EspUsbDeviceMsc *msc = nullptr;
static EspUsbDeviceMscFirmwareDisk *disk = nullptr;
static uint8_t diskStorage[16 * 1024];
static EspUsbDeviceHidVendor *hid = nullptr;
static EspUsbDeviceCdcSerial *cdc = nullptr, *bridgePort = nullptr;
static HidStream *hidStream = nullptr;
static CdcStream *cdcStream = nullptr;
static PortStream *portStream = nullptr;
static uint8_t mode = 0;
static bool followCoding = false;   // the bind's flag bit0: the port's line coding reruns the UART
static volatile bool codingPending = false;
static volatile uint32_t codingBaud = 0;
static volatile uint8_t codingData = 8, codingParity = 0, codingStop = 1;
static int loopPad = -1;
static uint16_t uartFn = 0;
static uint8_t probeTlv[64];
static char serial_[20];

static void applyLoopback() {
  if (loopPad < 0) return;
  gpio_input_enable(static_cast<gpio_num_t>(loopPad));
  esp_rom_gpio_connect_in_signal(loopPad, UART_PERIPH_SIGNAL(1, SOC_UART_RX_PIN_IDX), false);
}

// bind: port 0 <- fixture.uart (args fn u16 = this uart's fn), flags bit0 line coding. Only in a mode with the port.
static bool bindHook(uint8_t port, const oep::v1::ProbeConfig::Bind &b, void *) {
  if (port != 0 || !portStream) return false;
  if (b.source == 0) { uart.setPort(nullptr); followCoding = false; return true; }
  if (oep::v1::getU16(b.args) != uartFn) return false;
  uart.setPort(portStream);
  followCoding = b.flags & 1;
  return true;
}

class BridgeTest final : public oep::v1::Interface {
 public:
  const char *name() const override { return "oep.test.bridge"; }
  uint16_t instance() const override { return 0; }
  oep::Result handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) override {
    if (op == 0x02 && n == 2) { loopPad = p[0] ? p[1] : -1; applyLoopback(); return oep::completed(); }
    if (op == 0x03 && n == 0 && capacity >= 12) {
      oep::v1::putU32(out, uart.baud());
      oep::v1::putU32(out + 4, uart.portGaps());
      oep::v1::putU32(out + 8, codingBaud);
      return oep::completed(12);
    }
    if (op == 0x04 && n == 0) {
      static const char kBuild[] = __DATE__ " " __TIME__;
      const size_t len = sizeof kBuild - 1 < capacity ? sizeof kBuild - 1 : capacity;
      memcpy(out, kBuild, len);
      return oep::completed(len);
    }
    return oep::rejected(oep::kRejectMalformed);
  }
};
static BridgeTest bridgeTest;

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);
  Serial.setTxTimeoutMs(0);
  Serial.begin(115200);
  config.load();
  mode = config.bootMode(0);
  const Build &b = kBuilds[mode < 6 ? mode : 0];
  // functions are created before the device starts; interface numbers follow this order
  if (b.hid) { hid = new EspUsbDeviceHidVendor(usbDevice, 511); hidStream = new HidStream(*hid); }
  if (b.cdc) { cdc = new EspUsbDeviceCdcSerial(usbDevice, "OEP control (CDC)"); cdcStream = new CdcStream(*cdc); }
  for (uint8_t i = 0; i < b.data; ++i) {
    auto *port = new EspUsbDeviceCdcSerial(usbDevice, i == 0 ? "UART bridge" : "Data port");
    if (i == 0) { bridgePort = port; portStream = new PortStream(*port); }
  }
  if (b.msc) {
    msc = new EspUsbDeviceMsc(usbDevice);
    disk = new EspUsbDeviceMscFirmwareDisk(diskStorage, sizeof diskStorage);
    if (disk->begin("OEPPROBE")) {
      disk->addTextFile("README.TXT", "Copy a probe firmware .bin here: it is written to the spare OTA partition,\r\n"
                                      "checked, and the probe restarts into it.\r\n");
      disk->attach(*msc);
    }
  }
  dfu = new EspUsbDeviceDfu(usbDevice, EspUsbDeviceDfuMode::Download, "Probe firmware");
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BASE);
  snprintf(serial_, sizeof serial_, "%02x%02x%02x%02x%02x%02x-hs", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  EspUsbDeviceConfig usb;
  usb.vid = 0x303a;
  usb.pid = 0x4021;
  usb.manufacturer = "ch32-riscv-ug";
  usb.product = "OEP probe (P4 HS)";
  usb.serialNumber = serial_;
  usb.controller = EspUsbController::HighSpeed;
  const bool ok = usbDevice.begin(usb);
  if (!ok) {
    // A saved mode the device cannot be built in (P7: three CDC ports and more did not fit) would leave the probe
    // unreachable over USB for good: forget the saved boot mode and start again in this sketch's own mode.
    Serial.printf("# P4 mode=%u usb_begin failed: %s\n", mode, usbDevice.lastErrorName());
    if (config.forgetBootMode()) { delay(100); ESP.restart(); }
  }
  const bool direct = bulk.begin();
  endpoint.setFlushAfterBurst(true);
  if (hidStream) endpoint.addTransport(*hidStream, rxC, sizeof rxC, oep::v1::Endpoint::Framing::kLengthPrefixed, true);
  if (cdcStream) endpoint.addTransport(*cdcStream, rxB, sizeof rxB, oep::v1::Endpoint::Framing::kLengthPrefixed, true);
  if (bridgePort) {
    bridgePort->onLineCoding([](const EspUsbDeviceCdcLineCoding &c) {   // usbd task: applied in loop()
      codingBaud = c.baud;
      codingData = c.dataBits;
      codingParity = c.parity == 1 ? 2 : c.parity == 2 ? 1 : 0;   // CDC: 1 odd, 2 even; fixture.uart: 1 even, 2 odd
      codingStop = c.stopBits == 2 ? 2 : 1;
      codingPending = true;
    });
  }
  endpoint.add(config);
  endpoint.add(uart);
  uartFn = 2;   // list order: config 1, uart 2
  endpoint.add(bridgeTest);
  config.setBindHook(bindHook, nullptr);
  config.applySaved();
  endpoint.setBootId(esp_random());
  Serial.printf("# P4 mode=%u usb=%d direct=%d\n", mode, ok ? 1 : 0, direct ? 1 : 0);
}

void loop() {
  static bool valid = false;   // the new image booted and USB is up: keep it (else the bootloader rolls back)
  if (!valid && usbDevice.ready()) { valid = true; EspUsbDeviceFirmwareUpdate::markValid(); }
  if (codingPending) {
    codingPending = false;
    if (followCoding && uart.setLineCoding(codingBaud, codingData, codingParity, codingStop)) applyLoopback();
  }
  uart.poll();
  endpoint.poll();
  config.poll();
}
