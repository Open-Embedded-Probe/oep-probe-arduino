// OEP v0 development probe on a SparkFun Pro Micro RP2350 for the CH32L103 jig.
// Transport: USB CDC (Serial). Target link: RVSWD on two GPIOs through the SIO
// backend in OepRvswdPhy.cpp (same frame as the ESP32-P4 probe).
//
// The jig's wiring is not published yet: the RVSWD pair below is a build-time
// default, overridden with -DOEP_RVSWD_SWDIO=<n> -DOEP_RVSWD_SWCLK=<n> until the
// pin scan names them.
#include <OepCh32Dm.h>
#include <OepEndpoint.h>
#include <OepFixtureServices.h>
#include <OepProbeIdentity.h>
#include <OepRvswdPhy.h>
#include <OepTargetServices.h>

// Candidate, not yet confirmed. The pin survey (2026-09-23) found exactly one pin with a
// real external pull-up (GP24) and two with a real pull-down (GP23, GP29) - the signature a
// CH32's SWDIO and SWCLK leave - while every other pin follows whatever pull we apply. No
// debug module answered on any pair, but the CH32L103's WCH-LinkE was unplugged at the time,
// so that board had no power. Override with -DOEP_RVSWD_SWDIO= / -DOEP_RVSWD_SWCLK=.
#ifndef OEP_RVSWD_SWDIO
#define OEP_RVSWD_SWDIO 24
#endif
#ifndef OEP_RVSWD_SWCLK
#define OEP_RVSWD_SWCLK 23
#endif

static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer,
                              {1024, 4096, 8});
// 'R235' RP2350 development probe; firmware 3.0.0.
static constexpr uint8_t kSwdio = OEP_RVSWD_SWDIO, kSwclk = OEP_RVSWD_SWCLK;
// GP19 is the PSRAM chip select on this board; leave it alone.
static constexpr uint64_t kReserved = (uint64_t{1} << kSwdio) | (uint64_t{1} << kSwclk) | (uint64_t{1} << 19);
// GPIO 0..29 exist on the RP2350A; the Pro Micro does not bring all of them out, and an
// unbonded channel only toggles a register bit. probe.identity reports what is offered.
static constexpr uint64_t kAllPins = (uint64_t{1} << 30) - 1;
static oep::ProbeIdentity identity({0x52323335u, 0x00030000u, kReserved, kAllPins & ~kReserved});
static uint8_t fixturePins[30];
static size_t fixturePinCount = 0;

// CH32L103C8T6: 64 KiB flash from 0x08000000, 256-byte pages (probe-rs ch32-l1-usr),
// QingKe V4 - the same DM profile as the CH32X035.
static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy, {0x08000000u, 65536u, 256u, 256u});
static oep::TargetControl targetControl(dm, phy);
static oep::TargetMemory targetMemory(dm);
static oep::TargetFlash targetFlash(dm);
static oep::PinTable *pinTable = nullptr;
static oep::FixtureGpio *fixtureGpio = nullptr;
static oep::FixtureUart *fixtureUart = nullptr;   // DUT console once the jig's UART pins are known

void setup() {
  Serial.begin(115200);
  phy.begin(kSwdio, kSwclk);
  for (uint8_t pin = 0; pin < 30; ++pin) if (!((kReserved >> pin) & 1)) fixturePins[fixturePinCount++] = pin;
  pinTable = new oep::PinTable(fixturePins, fixturePinCount);
  fixtureGpio = new oep::FixtureGpio(*pinTable);
  // UART0 reaches GP0/1, GP12/13, GP16/17 and GP28/29 on this part; a plan that asks for
  // anything else is refused by setRX/setTX rather than silently mis-wired.
  fixtureUart = new oep::FixtureUart(*pinTable, Serial1, 2);
  endpoint.addService(identity);
  endpoint.addService(targetControl);
  endpoint.addService(targetMemory);
  endpoint.addService(targetFlash);
  endpoint.addService(*fixtureGpio);
  endpoint.addService(*fixtureUart);
}

void loop() {
  endpoint.poll();
  if (endpoint.idleFor(1500)) endpoint.abandonAll();
}
