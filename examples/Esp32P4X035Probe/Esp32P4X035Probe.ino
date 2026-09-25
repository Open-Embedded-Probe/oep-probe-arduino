// OEP v1 probe on ESP32-P4 for the CH32X035 fixture (oep-spec docs/v1-core-wire-delta.ja.md).
// Transport: USB-Serial/JTAG (HWCDC). Limits from E155: 1 KiB frames, 4 KiB window.
//
// oep.core (the probe described in its describe), oep.wire.rvswd (scan / attach / detach / attach_under_reset),
// oep.target.riscv-dm (DMI step lists, block read/write, run until halt, halt / resume, reset, step),
// oep.target.console (a position-addressed stream of the target's DM console), the fixtures
// oep.fixture.gpio / uart (x2) / capture (PARLIO), all revision 1, and the ESP-IDF I2C / SPI targets under
// io.github.ch32-riscv-ug.esp32.* (v0 payloads, revision 0).
#include <OepCh32Dm.h>
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
}

void loop() {
  endpoint.poll();
  console.poll();
  uart1.poll();
  uart2.poll();
  capture.poll();
  i2c.service();
  spi.service();
}
