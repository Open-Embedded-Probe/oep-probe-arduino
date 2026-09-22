// OEP v0 development probe on ESP32-P4 for the CH32X035 fixture.
// Transport: USB-Serial/JTAG (HWCDC). Limits from E155: 1 KiB frames, 4 KiB window.
#include <OepCh32Dm.h>
#include <OepEndpoint.h>
#include <OepFixtureServices.h>
#include <OepP4I2cTarget.h>
#include <OepProbeIdentity.h>
#include <OepRvswdPhy.h>
#include <OepTargetServices.h>

static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer,
                              {1024, 4096, 8});
// 'P4DV' generic P4 development probe; firmware 3.0.0. Reserved: GPIO2/54 (RVSWD), GPIO24/25 (USB-Serial/JTAG).
static constexpr uint64_t kReserved = (1ull << 2) | (1ull << 24) | (1ull << 25) | (1ull << 54);
static constexpr uint64_t kAllPins = (1ull << 55) - 1;
static oep::ProbeIdentity identity({0x50344456u, 0x00030000u, kReserved, kAllPins & ~kReserved});
// Fixture channels = every P4 GPIO that is not reserved.
static uint8_t fixturePins[55];
static size_t fixturePinCount = 0;

// Fixture wiring: P4 GPIO2 -> X035 PC18 (SWDIO), GPIO54 -> PC19 (SWCLK). CH32X035F8U6: 62 KiB, 256-byte pages.
static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy, {0x08000000u, 63488u, 256u, 256u});
static oep::TargetControl targetControl(dm);
static oep::TargetMemory targetMemory(dm);
static oep::TargetFlash targetFlash(dm);
static oep::PinTable *pinTable = nullptr;
static oep::FixtureGpio *fixtureGpio = nullptr;
static oep::FixtureUart *fixtureUart = nullptr;
static oep::P4I2cTarget *p4I2cTarget = nullptr;  // vendor tool (owner 0x0100)

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  phy.begin(2, 54);
  for (uint8_t pin = 0; pin < 55; ++pin) if (!((kReserved >> pin) & 1)) fixturePins[fixturePinCount++] = pin;
  pinTable = new oep::PinTable(fixturePins, fixturePinCount);
  fixtureGpio = new oep::FixtureGpio(*pinTable);
  fixtureUart = new oep::FixtureUart(*pinTable, Serial1);
  p4I2cTarget = new oep::P4I2cTarget(*pinTable);
  endpoint.addService(identity);
  endpoint.addService(targetControl);
  endpoint.addService(targetMemory);
  endpoint.addService(targetFlash);
  endpoint.addService(*fixtureGpio);
  endpoint.addService(*fixtureUart);
  endpoint.addService(*p4I2cTarget);
}

void loop() {
  endpoint.poll();
  p4I2cTarget->service();
  if (endpoint.idleFor(1500)) endpoint.abandonAll();
}
