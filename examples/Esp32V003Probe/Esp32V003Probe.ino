// OEP v1 draft probe on the classic ESP32 for the UIAPduino CH32V003 jig (E132 wiring).
// Transport: UART0 through the board's USB-UART bridge at 115200. Target link: GPIO16 -> PD1/SWIO
// (single wire, SwioPhy).
//
// v1 draft: oep.core (the probe in its describe), oep.wire.swio, oep.target.riscv-dm, oep.target.console,
// and the fixtures oep.fixture.gpio / uart / capture (v0 services under v1 names). GPIO23 -> PD7/NRST is a
// gpio channel labelled NRST, not a reset capability (oep-spec capability-name-hierarchy.ja.md, decision 2):
// pulse it open-drain through fixture.gpio when a pin reset is needed (UIAPduino bootloader, PINRSTF).
// The ESP-IDF I2C / SPI targets come back later under io.github.ch32-riscv-ug.esp32.*.
#include <OepCh32Dm.h>
#include <OepFixtureCapture.h>
#include <OepFixtureServices.h>
#include <OepSwioPhy.h>
#include <OepTargetServices.h>
#include <OepV1Console.h>
#include <OepV1Endpoint.h>
#include <OepV1Fixture.h>
#include <OepV1Target.h>

// UART0 runs through the board's USB-UART bridge (no flow control) and usbip: long bursts of pipelined
// responses lost bytes (2026-09-22, 2026-09-24). One 512-byte frame in flight keeps the outstanding data small.
static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::v1::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {512, 512, 1});
// Reserved: GPIO16 (SWIO), GPIO1/3 (UART0 transport), GPIO6-11 (SPI flash), GPIO0/2/12/15 (boot straps; 2 is the LED).
static constexpr uint64_t kReserved = (1ull << 16) | (1ull << 1) | (1ull << 3) | (0x3full << 6) |
                                      (1ull << 0) | (1ull << 2) | (1ull << 12) | (1ull << 15);
// Bonded GPIOs on the jig (E132): 4 5 13 14 17 18 19 21 22 23 25 26 27 32 33 34 35 36 39. 34-39 are input only.
static constexpr uint64_t kBonded = (1ull << 4) | (1ull << 5) | (1ull << 13) | (1ull << 14) | (1ull << 17) | (1ull << 18) |
                                    (1ull << 19) | (1ull << 21) | (1ull << 22) | (1ull << 23) | (1ull << 25) | (1ull << 26) |
                                    (1ull << 27) | (1ull << 32) | (1ull << 33) | (1ull << 34) | (1ull << 35) | (1ull << 36) |
                                    (1ull << 39);
static uint8_t fixturePins[40];
static size_t fixturePinCount = 0;

// CH32V003F4U6: 16 KiB, 64-byte pages, QingKe V2.
static oep::SwioPhy phy;
static oep::Ch32Dm dm(phy, {0x08000000u, 16384u, 64u, 64u}, oep::DmProfile::kQingKeV2);
static oep::v1::DebugPort port{dm, oep::SwioPhy::kPin, 0xffff};
static oep::v1::WireRvswd wire(port, 1, "oep.wire.swio");
static oep::v1::TargetRiscvDm riscvDm(port, 1);
static oep::TargetConsole consoleDriver(dm, phy);
static oep::v1::TargetConsoleStream console(port, consoleDriver, 1);
static oep::PinTable *pins = nullptr;
static oep::FixtureGpio *gpio = nullptr;
static oep::FixtureUart *uart = nullptr;       // DUT console: V003 PD5/TX -> GPIO22, PD6/RX <- GPIO21 (E132)
static oep::FixtureCapture *capture = nullptr;  // GPIO sampler on core 0, 0.4..2 MHz, 1 byte/sample
static oep::v1::V0Fixture *gpioV1 = nullptr, *uartV1 = nullptr, *captureV1 = nullptr;
static const uint8_t kGpioRoles[] = {1};
static const uint8_t kUartRoles[] = {1, 2};
static const uint8_t kCaptureRoles[] = {0, 1, 2, 3, 4, 5, 6, 7};
static const uint8_t kUartExtra[] = {oep::v1::kTagImplementation, 1, 2};
static const uint8_t kCaptureExtra[] = {oep::v1::kTagImplementation, 1, 1};   // software sampler
static uint8_t probeTlv[200];

static size_t describeProbe() {
  oep::v1::TlvWriter w(probeTlv, sizeof probeTlv);
  w.text(oep::v1::kCoreFirmware, "3.1.0-v1draft");
  w.text(oep::v1::kCoreModel, "esp32-d0wd");
  const uint64_t mac = ESP.getEfuseMac();
  uint8_t id[6];
  for (int i = 0; i < 6; ++i) id[i] = static_cast<uint8_t>(mac >> (8 * i));
  w.put(oep::v1::kCoreUnitId, id, sizeof id);
  w.u16(oep::v1::kCoreChannels, 40);
  uint8_t bitmap[2 + 5] = {0, 0};
  for (int i = 0; i < 5; ++i) bitmap[2 + i] = static_cast<uint8_t>(kReserved >> (8 * i));
  w.put(oep::v1::kCoreReserved, bitmap, sizeof bitmap);
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
  for (uint8_t pin = 0; pin < 40; ++pin) if ((kBonded >> pin) & 1 && !((kReserved >> pin) & 1)) fixturePins[fixturePinCount++] = pin;
  // E132: every UIAP pin is wired here, including the software-USB pair (PD3/PD4); a permanent
  // ESP32 pull on either USB line breaks enumeration. Idle must be genuinely high impedance.
  for (size_t i = 0; i < fixturePinCount; ++i) pinMode(fixturePins[i], INPUT);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(esp_random());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  console.setMaxRead(480);   // 512-byte frames
  endpoint.add(console);
  pins = new oep::PinTable(fixturePins, fixturePinCount);
  gpio = new oep::FixtureGpio(*pins);
  uart = new oep::FixtureUart(*pins, Serial2, 2);
  capture = new oep::FixtureCapture(*pins);
  gpioV1 = new oep::v1::V0Fixture(*gpio, "oep.fixture.gpio", 2, *pins, kGpioRoles, 1, 1u << 2);
  uartV1 = new oep::v1::V0Fixture(*uart, "oep.fixture.uart", 3, *pins, kUartRoles, 2, 0, kUartExtra, sizeof kUartExtra);
  captureV1 = new oep::v1::V0Fixture(*capture, "oep.fixture.capture", 4, *pins, kCaptureRoles, 8, (1u << 3) | (1u << 4),
                                     kCaptureExtra, sizeof kCaptureExtra);
  endpoint.add(*gpioV1);
  endpoint.add(*uartV1);
  endpoint.add(*captureV1);
}

void loop() {
  endpoint.poll();
  console.poll();
}
