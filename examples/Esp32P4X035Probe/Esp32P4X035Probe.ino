// OEP v1 probe on ESP32-P4 for the CH32X035 fixture (oep-spec docs/oep-core.ja.md, docs/probe-development-guide.ja.md
// §3.8). One endpoint, four transports (the describe lists them in this order):
//
//   0 vendor bulk      HS port (direct build: build_opt.h, compile with --clean)            OEP only
//   1 USB-Serial/JTAG  the P4's USJ (HWCDC Serial)                                           serial port: OEP + raw
//   2 HID              HS port, vendor HID reports                                           OEP only
//   3 USB CDC          HS port, "OEP console"                                                serial port: OEP + raw
//
// A serial port always takes OEP frames (0x00 <COBS> 0x00); its other bytes are what its bind carries (oep.probe.config:
// a slot's console, a fixture UART). Limits from E155 (USB-Serial/JTAG): 1 KiB frames, 4 KiB window. The HS device is
// VID:PID 303a:0002 (probe guide §3.8: the OEP PID is not taken yet; kUsbVid / kUsbPid) with serial MAC + "-hs".
//
// oep.core, oep.wire.rvswd, oep.target.riscv-dm, oep.target.console, the fixtures oep.fixture.gpio / uart (x2) /
// capture (PARLIO), all revision 1, the ESP-IDF I2C / SPI targets under io.github.ch32-riscv-ug.esp32.* (v0 payloads,
// revision 0), and oep.probe.config (one slot: this fixture's RVSWD pair; binds on ports 1 and 3; saved in NVS).
#include <esp_mac.h>
#include <soc/usb_serial_jtag_reg.h>
#include <EspUsbDevice.h>
#include <OepCh32Dm.h>
#include <OepDirectBulkStream.h>
#include <OepDmConsole.h>
#include <OepP4I2cTarget.h>
#include <OepP4SpiTarget.h>
#include <OepPinTable.h>
#include <OepRvswdPhy.h>
#include <OepV1Bind.h>
#include <OepV1Capture.h>
#include <OepV1Config.h>
#include <OepV1Console.h>
#include <OepV1Endpoint.h>
#include <OepV1Fixture.h>
#include <OepV1Target.h>
#include "UsbStreams.h"

static constexpr uint16_t kUsbVid = 0x303a, kUsbPid = 0x0002;   // until the OEP PID (pid.codes) is taken

// The HS device: functions are created before it starts, interface numbers follow the class order (HID 0, vendor 1,
// CDC 2-3).
static EspUsbDevice usbDevice;
static EspUsbDeviceVendor vendor(usbDevice);
static oep::DirectBulkStream bulk(vendor);
static EspUsbDeviceHidVendor hid(usbDevice, 511);
static HidStream hidStream(hid);
static EspUsbDeviceCdcSerial cdc(usbDevice, "OEP console");   // a name to show; every CDC of the probe speaks OEP
static CdcStream cdcStream(cdc);

static uint8_t rxVendor[1024], rxUsj[1100], rxHid[1024], rxCdc[1100];   // serial ports: cobsFrameMax(1024)
static uint8_t txBuffer[1024];
static oep::v1::Endpoint endpoint(bulk, rxVendor, sizeof rxVendor, txBuffer, sizeof txBuffer, {1024, 4096, 8},
                                  oep::v1::Endpoint::kVendorBulk, 1);

// Fixture wiring: P4 GPIO2 -> X035 PC18 (SWDIO), GPIO54 -> PC19 (SWCLK). The flash layout is the host's business.
static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy);
static oep::v1::DebugPort port{dm, 2, 54};
static oep::v1::WireRvswd wire(port, 1);
static oep::v1::TargetRiscvDm riscvDm(port, 1);
static oep::DmConsole consoleDriver(dm, phy);   // the DM console framings (SDI / DMDATA / dmseq)
static oep::v1::TargetConsoleStream console(port, consoleDriver, 1);

// Reserved: GPIO2/54 (RVSWD), GPIO24/25 (USB-Serial/JTAG). Every other GPIO is a fixture channel.
static constexpr uint64_t kReserved = (1ull << 2) | (1ull << 24) | (1ull << 25) | (1ull << 54);
static constexpr uint64_t kFixtures = ((1ull << 55) - 1) & ~kReserved;
static oep::PinTable pins(kFixtures);
static oep::v1::FixtureGpio gpio(pins, 2);
static oep::v1::FixtureUart uart1(pins, Serial1, 3, 2), uart2(pins, Serial2, 4, 5);   // uart2: X035 USART2 tests (GPIO48/49)
static oep::v1::LogicCapture capture(endpoint, kReserved, 5);   // PARLIO RX; its lines are never driven
// ESP-IDF I2C / SPI slave tools under the project's own names (capability-name-hierarchy.ja.md, decision 4).
static oep::P4I2cTarget i2c(pins);
static oep::P4SpiTarget spi(pins);
static oep::v1::Binds binds;
static oep::v1::ProbeConfig config(endpoint, binds);
static uint8_t probeTlv[160];
static char serial_[20];

static size_t describeProbe() {
  oep::v1::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];
  oep::v1::describeCore(w, "esp32-p4-devkit", id, oep::platformUnitId(id, sizeof id), 55, kReserved);
  w.text(oep::v1::kCoreProfile, "io.github.ch32-riscv-ug.p4-x035");
  w.label(2, "SWDIO");
  w.label(54, "SWCLK");
  w.label(51, "LED");
  return w.ok() ? w.length() : 0;
}

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §2.5)
  // This jig's wiring (fixed): channel 6 drives the DUT USART4 RX (PB1), which must not float while no UART holds it (a floating
  // RX line fed the DUT's command parser noise, 2026-09-22). Idle = pull-up; every other free pin stays Hi-Z.
  pins.setIdle(6, oep::PinTable::kIdlePullUp);
  // USB-Serial/JTAG: opening and closing the port must not reset the probe (probe guide §3.7)
  REG_SET_BIT(USB_SERIAL_JTAG_CHIP_RST_REG, USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS);
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.setTxTimeoutMs(0);   // a port nobody reads never stops loop()
  Serial.begin(115200);
  phy.begin(2, 54);

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BASE);
  snprintf(serial_, sizeof serial_, "%02x%02x%02x%02x%02x%02x-hs", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  EspUsbDeviceConfig usb;
  usb.vid = kUsbVid;
  usb.pid = kUsbPid;
  usb.manufacturer = "ch32-riscv-ug";
  usb.product = "OEP probe (P4 HS)";   // iProduct "OEP...": how discovery knows the probe (core §3.3, probe guide §3.8)
  usb.serialNumber = serial_;
  usb.controller = EspUsbController::HighSpeed;
  usbDevice.begin(usb);
  bulk.begin();
  endpoint.setFlushAfterBurst(true);
  endpoint.addTransport(Serial, rxUsj, sizeof rxUsj, oep::v1::Endpoint::kUsbSerialJtag);
  endpoint.addTransport(hidStream, rxHid, sizeof rxHid, oep::v1::Endpoint::kHid, 0, true);
  endpoint.addTransport(cdcStream, rxCdc, sizeof rxCdc, oep::v1::Endpoint::kUsbCdc, 2, true);
  endpoint.setRawPorts(&binds);
  // describe oep_pid: listed by discovery (until the OEP PID is taken: by the iProduct starting "OEP")
  endpoint.setOepPid(true);

  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(esp_random());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  // attach-under-reset: no NRST is wired on this jig, so no default; the host may name any fixture channel.
  port.reset_allowed = kFixtures;
  endpoint.add(console);
  // The fixture pins are left as the P4 boots them (inputs, nothing driven); the RP2 sketches park theirs because
  // the RP2 pad comes up with a pull-down.
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
}
