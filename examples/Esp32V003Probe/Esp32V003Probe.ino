// OEP v0 development probe on the classic ESP32 for the UIAPduino CH32V003 jig (E132 wiring).
// Transport: UART0 through the board's USB-UART bridge at 115200. Target link: GPIO16 -> PD1/SWIO
// (single wire, SwioPhy). GPIO23 -> RST is wired but deliberately never driven.
#include <OepCh32Dm.h>
#include <OepEndpoint.h>
#include <OepFixtureCapture.h>
#include <OepFixtureServices.h>
#include <OepP4I2cTarget.h>
#include <OepP4SpiTarget.h>
#include <OepProbeIdentity.h>
#include <OepSwioPhy.h>
#include <OepTargetServices.h>

// UART0 runs through a CP2102 (576-byte buffer, no flow control) and usbip: a 16 KiB burst of
// pipelined responses lost bytes three times on 2026-09-22 (framing lost / bad result header).
// One 512-byte frame in flight keeps the outstanding data below the bridge's buffer.
static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer,
                              {512, 512, 1});
// 'E32V' classic ESP32 probe for V003; firmware 3.0.0. Reserved: GPIO16 (SWIO), GPIO23 (RST, never driven),
// GPIO1/3 (UART0 transport), GPIO6-11 (SPI flash), GPIO0/2/12/15 (boot straps; 2 is the LED).
static constexpr uint64_t kReserved = (1ull << 16) | (1ull << 23) | (1ull << 1) | (1ull << 3) | (0x3full << 6) |
                                      (1ull << 0) | (1ull << 2) | (1ull << 12) | (1ull << 15);
static constexpr uint64_t kAllPins = (1ull << 40) - 1;
// Bonded GPIOs on the jig (E132): 4 5 13 14 17 18 19 21 22 25 26 27 32 33 34 35 36 39. 34-39 are input only.
static constexpr uint64_t kBonded = (1ull << 4) | (1ull << 5) | (1ull << 13) | (1ull << 14) | (1ull << 17) | (1ull << 18) |
                                    (1ull << 19) | (1ull << 21) | (1ull << 22) | (1ull << 25) | (1ull << 26) | (1ull << 27) |
                                    (1ull << 32) | (1ull << 33) | (1ull << 34) | (1ull << 35) | (1ull << 36) | (1ull << 39);
static oep::ProbeIdentity identity({0x45333256u, 0x00030000u, kReserved, kBonded & kAllPins & ~kReserved});
static uint8_t fixturePins[40];
static size_t fixturePinCount = 0;

// CH32V003F4U6: 16 KiB, 64-byte pages, QingKe V2 (RAM loader flash path).
static oep::SwioPhy phy;
static oep::Ch32Dm dm(phy, {0x08000000u, 16384u, 64u, 64u}, oep::DmProfile::kQingKeV2);
static oep::TargetControl targetControl(dm, phy, 23);   // GPIO23 -> PD7/NRST: reset mode 3 (pin reset, sets PINRSTF)
static oep::TargetMemory targetMemory(dm);
static oep::TargetFlash targetFlash(dm);
// Console with no console wiring: the target writes into the debug module's data
// registers and the probe collects them from loop() (ArduinoCore-CH32's SerialSDI).
static oep::TargetConsole targetConsole(dm, phy);
static oep::PinTable *pinTable = nullptr;
static oep::FixtureGpio *fixtureGpio = nullptr;
static oep::FixtureUart *fixtureUart = nullptr;   // DUT console: V003 PD5/TX -> GPIO22, PD6/RX <- GPIO21 (E132)
static oep::P4I2cTarget *i2cTarget = nullptr;     // ESP-IDF slave tools; the "P4" name is historical
static oep::P4SpiTarget *spiTarget = nullptr;
static oep::FixtureCapture *fixtureCapture = nullptr;   // GPIO sampler on core 0, 0.4..2 MHz, 1 byte/sample

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  phy.begin(oep::SwioPhy::kPin);
  for (uint8_t pin = 0; pin < 40; ++pin) if ((kBonded >> pin) & 1 && !((kReserved >> pin) & 1)) fixturePins[fixturePinCount++] = pin;
  // E132: every UIAP pin is wired here, including the software-USB pair (PD3/PD4); a permanent
  // ESP32 pull on either USB line breaks enumeration. Idle must be genuinely high impedance.
  for (size_t i = 0; i < fixturePinCount; ++i) pinMode(fixturePins[i], INPUT);
  pinMode(23, INPUT);   // RST: wired, never driven
  pinTable = new oep::PinTable(fixturePins, fixturePinCount);
  fixtureGpio = new oep::FixtureGpio(*pinTable);
  fixtureUart = new oep::FixtureUart(*pinTable, Serial2, 2);
  i2cTarget = new oep::P4I2cTarget(*pinTable);
  spiTarget = new oep::P4SpiTarget(*pinTable);
  fixtureCapture = new oep::FixtureCapture(*pinTable);
  endpoint.addService(identity);
  endpoint.addService(targetControl);
  endpoint.addService(targetMemory);
  endpoint.addService(targetFlash);
  endpoint.addService(targetConsole);
  endpoint.addService(*fixtureGpio);
  endpoint.addService(*fixtureUart);
  endpoint.addService(*i2cTarget);
  endpoint.addService(*spiTarget);
  endpoint.addService(*fixtureCapture);
}

void loop() {
  endpoint.poll();
  targetConsole.poll();
  i2cTarget->service();
  spiTarget->service();
  if (endpoint.idleFor(1500)) endpoint.abandonAll();
}
