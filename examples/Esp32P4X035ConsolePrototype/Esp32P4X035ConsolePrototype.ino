// Prototype P6 (oep-spec probe-cdc-and-persistence §7) on the CH32X035 fixture: Esp32P4X035Probe (OEP on
// USB-Serial/JTAG) plus the P4's HS port as a USB device with one data CDC port, "Target console", that a bind
// (oep.probe.config, source 2 = oep.target.console) follows both ways:
//   attach 0 (host): when a host attaches, the probe opens the console on that connection and counts the bind as a user
//   attach 1 (on open): when the port is opened (DTR), a non-halting attach, the chip_id (DM 0x7f) checked against
//                       the target item, then the console
//   attach 2 (at boot): the same once at boot
// A host's detach then drops only the host's use (the link stays for the console); detach with force closes it.
// oep.test.console-bind (scratch): 0x01 status -> state u8 (0 idle, 1 console on, 2 chip mismatch, 3 attach failed, 4 chip_id unknown),
//   users u8, connected u8, console_open u8, chip_seen u32, port_gaps u32, dtr u8
#include <OepCh32Dm.h>
#include <EspUsbDevice.h>
#include <esp_mac.h>
#include <OepV1Config.h>
#include <OepFixtureServices.h>
#include <OepP4I2cTarget.h>
#include <OepP4SpiTarget.h>
#include <OepRvswdPhy.h>
#include <OepDmConsole.h>
#include <OepV1Capture.h>
#include <OepV1Console.h>
#include <OepV1Endpoint.h>
#include <OepV1Fixture.h>
#include <OepV1Target.h>

static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::v1::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8});

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
// ESP-IDF I2C / SPI slave tools under the project's own names (capability-name-hierarchy.ja.md, decision 4):
// both implementations so far are the ESP-IDF slave drivers, whose quirks stay out of any oep. name.
static oep::P4I2cTarget i2c(pins);
static oep::P4SpiTarget spi(pins);
static const uint8_t kI2cRoles[] = {1, 2};                    // SDA, SCL
static const uint8_t kSpiRoles[] = {1, 2, 3, 4};              // SCK, MOSI, MISO, CS
// lock-free: i2c status (5) and read_hw (0x10), spi status (4)
static oep::v1::V0Fixture i2cV1(i2c, "io.github.ch32-riscv-ug.esp32.i2c-target", 6, pins, kI2cRoles, 2,
                                (1u << 5) | (1u << 16), oep::v1::kImplementationPeripheral, sizeof oep::v1::kImplementationPeripheral);
static oep::v1::V0Fixture spiV1(spi, "io.github.ch32-riscv-ug.esp32.spi-target", 7, pins, kSpiRoles, 4, 1u << 4,
                                oep::v1::kImplementationPeripheral, sizeof oep::v1::kImplementationPeripheral);
static uint8_t probeTlv[160];

// The HS port: one data CDC port (port 0) for the console bind.
static EspUsbDevice usbDevice;
static EspUsbDeviceCdcSerial consolePort(usbDevice, "Target console");
class PortStream final : public Stream {   // never waits; availableForWrite() says whether the port is open
 public:
  explicit PortStream(EspUsbDeviceCdcSerial &p) : p_(p) {}
  int available() override { return p_.available(); }
  int read() override { return p_.read(); }
  size_t readBytes(char *b, size_t n) override { return p_.read(reinterpret_cast<uint8_t *>(b), n); }
  int peek() override { return -1; }
  size_t write(uint8_t c) override { return p_.write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override { return p_.write(b, n); }
  void flush() override { p_.flush(); }
  int availableForWrite() override { return p_.connected() ? 4096 : 0; }
 private:
  EspUsbDeviceCdcSerial &p_;
};
static PortStream portStream(consolePort);
static const oep::v1::ProbeConfig::Mode kModes[] = {{0x00, 1, "console"}};   // OEP stays on USB-Serial/JTAG
static const uint8_t kPortInterfaces[] = {0};
static oep::v1::ProbeConfig config(endpoint, kModes, 1, kPortInterfaces);
static constexpr uint16_t kWireFn = 1;   // oep.wire.rvswd is the first interface added

// The bind engine (scratch): what source 2 asks for, run from loop().
static struct {
  bool active = false;
  uint8_t mechanism = 2, attach = 0, state = 0;
  bool booted = false, last_dtr = false;
  uint32_t chip_seen = 0;
} bindState;

static bool bindHook(uint8_t port_no, const oep::v1::ProbeConfig::Bind &b, void *) {
  if (port_no != 0) return false;
  if (b.source == 0) {
    console.setPort(nullptr);
    if (bindState.active) oep::v1::releaseConnection(port, oep::v1::DebugPort::kUserBind, false);
    bindState.active = false;
    bindState.state = 0;
    return true;
  }
  if (b.source != 2 || oep::v1::getU16(b.args) != kWireFn || b.args[2] > 2) return false;
  bindState.active = true;
  bindState.mechanism = b.args[2];
  bindState.attach = b.attach;
  bindState.booted = false;
  bindState.state = 0;
  console.setPort(&portStream);
  return true;
}

static void openConsoleForBind() {
  if (console.bindOpen(bindState.mechanism)) {
    port.users |= oep::v1::DebugPort::kUserBind;
    bindState.state = 1;
  }
}

static void attachForBind() {
  uint32_t status = 0, chip = 0;
  if (!oep::v1::attachRunning(port, oep::v1::DebugPort::kUserBind, status)) { bindState.state = 3; return; }
  dm.readDmi(0x7f, chip);
  bindState.chip_seen = chip;
  const auto &t = config.target();
  // v1 wire §5.10: bits [7:4] are the silicon revision and are not compared (ch32rv matches AttachChip's chip_id the
  // same way); 0 or all ones means this part does not say (0x7f is only confirmed on L103 / V203 / V003 / X035)
  if (chip == 0 || chip == 0xffffffffu) {
    oep::v1::releaseConnection(port, oep::v1::DebugPort::kUserBind, false);
    bindState.state = 4;
    return;
  }
  if (t.set && ((chip ^ t.chip_id) & ~0xf0u)) {   // not the target this bind was set for: leave it alone
    oep::v1::releaseConnection(port, oep::v1::DebugPort::kUserBind, false);
    bindState.state = 2;
    return;
  }
  openConsoleForBind();
}

static void bindPoll() {
  if (!bindState.active) return;
  const bool dtr = consolePort.connected();
  const bool rose = dtr && !bindState.last_dtr;
  bindState.last_dtr = dtr;
  if (port.connected && !console.isOpen()) openConsoleForBind();   // a connection is there (the host's or ours)
  else if (!port.connected && bindState.state == 1) bindState.state = 0;   // lost: wait for the next signal
  if (port.connected) return;
  if (bindState.attach == 1 && rose) attachForBind();
  if (bindState.attach == 2 && !bindState.booted) { bindState.booted = true; attachForBind(); }
}

class ConsoleBindTest final : public oep::v1::Interface {
 public:
  const char *name() const override { return "oep.test.console-bind"; }
  uint16_t instance() const override { return 0; }
  bool lockFree(uint8_t) const override { return true; }
  oep::Result handle(uint8_t op, const uint8_t *, size_t n, uint8_t *out, size_t capacity) override {
    if (op != 0x01 || n || capacity < 13) return oep::rejected(oep::kRejectMalformed);
    out[0] = bindState.state;
    out[1] = port.users;
    out[2] = port.connected;
    out[3] = console.isOpen();
    oep::v1::putU32(out + 4, bindState.chip_seen);
    oep::v1::putU32(out + 8, console.portGaps());
    out[12] = consolePort.connected();
    return oep::completed(13);
  }
};
static ConsoleBindTest bindTest;
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
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  phy.begin(2, 54);
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
  endpoint.add(i2cV1);
  endpoint.add(spiV1);
  endpoint.add(config);
  endpoint.add(bindTest);
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BASE);
  snprintf(serial_, sizeof serial_, "%02x%02x%02x%02x%02x%02x-hs", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  config.load();
  config.bootMode(0);
  EspUsbDeviceConfig usb;
  usb.vid = 0x303a;
  usb.pid = 0x4021;
  usb.manufacturer = "ch32-riscv-ug";
  usb.product = "OEP probe (P4 X035 fixture, HS)";
  usb.serialNumber = serial_;
  usb.controller = EspUsbController::HighSpeed;
  usbDevice.begin(usb);
  config.setBindHook(bindHook, nullptr);
  config.applySaved();
}

void loop() {
  endpoint.poll();
  bindPoll();
  console.poll();
  config.poll();
  uart1.poll();
  uart2.poll();
  capture.poll();
  i2c.service();
  spi.service();
}
