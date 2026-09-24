// OEP v1 draft probe on the classic ESP32 for the UIAPduino CH32V003 jig (E132 wiring).
// Transport: UART0 through the board's USB-UART bridge at 115200. Target link: GPIO16 -> PD1/SWIO
// (single wire, SwioPhy).
//
// v1 draft: oep.core (the probe in its describe), oep.wire.swio, oep.target.riscv-dm, oep.target.console,
// and the fixtures oep.fixture.gpio / uart / capture (v0 services under v1 names). GPIO23 -> PD7/NRST is a gpio
// channel labelled NRST and the default reset line for oep.wire.swio attach-under-reset (the host may name another);
// a plain pin reset (UIAPduino bootloader, PINRSTF) pulses it open drain through fixture.gpio.
// The ESP-IDF I2C / SPI targets are offered under io.github.ch32-riscv-ug.esp32.*.
#include <OepCh32Dm.h>
#include <OepFixtureCapture.h>
#include <OepFixtureServices.h>
#include <OepP4I2cTarget.h>
#include <OepP4SpiTarget.h>
#include <OepSwioPhy.h>
#include <OepDmConsole.h>
#include <OepV1Console.h>
#include <OepV1Endpoint.h>
#include <OepV1Fixture.h>
#include <OepV1Target.h>

// UART0 runs through the board's USB-UART bridge (no flow control) and usbip: long bursts of pipelined
// responses lost bytes (2026-09-22, 2026-09-24). One 512-byte frame in flight keeps the outstanding data small.
static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::v1::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {512, 512, 1},
                                  oep::v1::Endpoint::Framing::kCobsCrc);   // UART: COBS + CRC-16
// Reserved: GPIO16 (SWIO), GPIO1/3 (UART0 transport), GPIO6-11 (SPI flash), GPIO0/2/12/15 (boot straps; 2 is the LED).
static constexpr uint64_t kReserved = (1ull << 16) | (1ull << 1) | (1ull << 3) | (0x3full << 6) |
                                      (1ull << 0) | (1ull << 2) | (1ull << 12) | (1ull << 15);
// Bonded GPIOs on the jig (E132): 4 5 13 14 17 18 19 21 22 23 25 26 27 32 33 34 35 36 39. 34-39 are input only.
static constexpr uint64_t kBonded = (1ull << 4) | (1ull << 5) | (1ull << 13) | (1ull << 14) | (1ull << 17) | (1ull << 18) |
                                    (1ull << 19) | (1ull << 21) | (1ull << 22) | (1ull << 23) | (1ull << 25) | (1ull << 26) |
                                    (1ull << 27) | (1ull << 32) | (1ull << 33) | (1ull << 34) | (1ull << 35) | (1ull << 36) |
                                    (1ull << 39);
static constexpr uint64_t kFixtures = kBonded & ~kReserved;

// CH32V003 (UIAPduino). The flash layout and the RAM loader are the host's business.
static oep::SwioPhy phy;
static oep::Ch32Dm dm(phy);
static oep::v1::DebugPort port{dm, oep::SwioPhy::kPin, 0xffff};
static oep::v1::WireRvswd wire(port, 1, "oep.wire.swio");
static oep::v1::TargetRiscvDm riscvDm(port, 1);
static oep::DmConsole consoleDriver(dm, phy);
static oep::v1::TargetConsoleStream console(port, consoleDriver, 1);
static oep::PinTable pins(kFixtures);
static oep::FixtureGpio gpio(pins);
static oep::FixtureUart uart(pins, Serial2, 2);       // DUT console: V003 PD5/TX -> GPIO22, PD6/RX <- GPIO21 (E132)
static oep::FixtureCapture capture(pins);             // GPIO sampler on core 0, 0.4..2 MHz, 1 byte/sample
static const uint8_t kCaptureExtra[] = {oep::v1::kTagImplementation, 1, 1};   // software sampler
static oep::v1::V0Fixture gpioV1(gpio, "oep.fixture.gpio", 2, pins, oep::v1::kGpioRoles, 1, oep::v1::kGpioLockFree);
static oep::v1::V0Fixture uartV1(uart, "oep.fixture.uart", 3, pins, oep::v1::kUartRoles, 2, 0,
                                 oep::v1::kImplementationPeripheral, sizeof oep::v1::kImplementationPeripheral);
static oep::v1::V0Fixture captureV1(capture, "oep.fixture.capture", 4, pins, oep::v1::kCaptureRoles, 8,
                                    oep::v1::kCaptureLockFree, kCaptureExtra, sizeof kCaptureExtra);
// ESP-IDF I2C / SPI slave tools under the project's own names (capability-name-hierarchy.ja.md, decision 4):
// both implementations so far are the ESP-IDF slave drivers, whose quirks stay out of any oep. name.
static oep::P4I2cTarget i2c(pins);
static oep::P4SpiTarget spi(pins);
static const uint8_t kI2cRoles[] = {1, 2};                    // SDA, SCL
static const uint8_t kSpiRoles[] = {1, 2, 3, 4};              // SCK, MOSI, MISO, CS
// lock-free: i2c status (5) and read_hw (0x10), spi status (4)
static oep::v1::V0Fixture i2cV1(i2c, "io.github.ch32-riscv-ug.esp32.i2c-target", 5, pins, kI2cRoles, 2,
                                (1u << 5) | (1u << 16), oep::v1::kImplementationPeripheral,
                                sizeof oep::v1::kImplementationPeripheral);
static oep::v1::V0Fixture spiV1(spi, "io.github.ch32-riscv-ug.esp32.spi-target", 6, pins, kSpiRoles, 4, 1u << 4,
                                oep::v1::kImplementationPeripheral, sizeof oep::v1::kImplementationPeripheral);
static uint8_t probeTlv[200];

static size_t describeProbe() {
  oep::v1::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];
  oep::v1::describeCore(w, "esp32-d0wd", id, oep::platformUnitId(id, sizeof id), 40, kReserved);
  w.text(oep::v1::kCoreProfile, "io.github.ch32-riscv-ug.esp32-v003");
  w.label(16, "SWIO");
  w.label(23, "NRST");
  w.label(22, "DUT TX");
  w.label(21, "DUT RX");
  w.u32(oep::v1::kCoreUartRates, 115200);
  return w.ok() ? w.length() : 0;
}

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  phy.begin(oep::SwioPhy::kPin);
  // E132: every UIAP pin is wired here, including the software-USB pair (PD3/PD4); a permanent
  // ESP32 pull on either USB line breaks enumeration. Idle must be genuinely high impedance.
  oep::platformParkMask(kFixtures);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(esp_random());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  port.reset_default = 23;   // attach-under-reset through the V003's NRST unless the host names another channel
  port.reset_allowed = kFixtures;
  console.setMaxRead(480);   // 512-byte frames
  endpoint.add(console);
  endpoint.add(gpioV1);
  endpoint.add(uartV1);
  endpoint.add(captureV1);
  endpoint.add(i2cV1);
  endpoint.add(spiV1);
}

void loop() {
  endpoint.poll();
  console.poll();
  i2c.service();
  spi.service();
}
