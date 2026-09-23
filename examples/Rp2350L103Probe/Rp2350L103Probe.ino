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

// Measured 2026-09-23 once the CH32L103 had power: the pair sweep found its debug module on
// SWDIO=GP0, SWCLK=GP1 (DMSTATUS 0x00000c82) and on no other ordered pair. These are also
// Serial1's default pins, so fixture.uart has to take a different pair from the plan.
// Override with -DOEP_RVSWD_SWDIO= / -DOEP_RVSWD_SWCLK=.
#ifndef OEP_RVSWD_SWDIO
#define OEP_RVSWD_SWDIO 0
#endif
#ifndef OEP_RVSWD_SWCLK
#define OEP_RVSWD_SWCLK 1
#endif
// The jig is flying leads, and it shows: at 100 and 200 ns the link passes attach()'s read
// and write checks yet memory reads come back as the previous operation's leftover, while
// 500 ns is stable (2026-09-23). Hold the floor there until the CH32L103 is properly wired.
#ifndef OEP_RVSWD_MIN_HALF_NS
#define OEP_RVSWD_MIN_HALF_NS 500
#endif

static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer,
                              {1024, 4096, 8});
// 'R235' RP2350 development probe; firmware 3.0.0.
static constexpr uint8_t kSwdio = OEP_RVSWD_SWDIO, kSwclk = OEP_RVSWD_SWCLK;
// GP2 is the target's NRST: measured 2026-09-23 by pulling each spare channel down on its
// own and watching which one took the debug module away. It idles high on the CH32's own
// pull-up, which is why the RP2 pad's default pull-down used to hold the part in reset.
static constexpr uint8_t kNrst = 2;
// GP19 is the PSRAM chip select on this board; leave it alone.
static constexpr uint64_t kReserved = (uint64_t{1} << kSwdio) | (uint64_t{1} << kSwclk) |
                                      (uint64_t{1} << kNrst) | (uint64_t{1} << 19);
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
static oep::TargetControl targetControl(dm, phy, kNrst);   // reset mode 3 = pin reset (sets PINRSTF)
static oep::TargetMemory targetMemory(dm);
static oep::TargetFlash targetFlash(dm);
// Console with no console wiring: the target writes into the debug module's data
// registers and the probe collects them from loop() (ArduinoCore-CH32's SerialSDI).
static oep::TargetConsole targetConsole(dm, phy);
static oep::PinTable *pinTable = nullptr;
static oep::FixtureGpio *fixtureGpio = nullptr;
static oep::FixtureUart *fixtureUart = nullptr;   // DUT console once the jig's UART pins are known

void setup() {
  Serial.begin(115200);
  // Reserved does not mean untouched: the RP2 pad's default pull-down on this line holds
  // the target in reset. Release it and never drive it high - the target pulls it up.
  pinMode(kNrst, INPUT);
  phy.begin(kSwdio, kSwclk);
  phy.setMinHalfNs(OEP_RVSWD_MIN_HALF_NS);
  phy.setIdleClockLow(true);   // a CH32L103 resets its debug link when the bus rests high
  for (uint8_t pin = 0; pin < 30; ++pin) if (!((kReserved >> pin) & 1)) fixturePins[fixturePinCount++] = pin;
  pinTable = new oep::PinTable(fixturePins, fixturePinCount);
  // Hi-Z everything the probe does not own. RP2 pads boot with a pull-down, and this jig
  // is only half wired: on the CH32L103 that pull-down held a line the target cares about
  // and the hart would not halt, though its debug module answered normally (2026-09-23).
  oep::platformParkPins(fixturePins, fixturePinCount);
  fixtureGpio = new oep::FixtureGpio(*pinTable);
  // UART0 reaches GP0/1, GP12/13, GP16/17 and GP28/29 on this part; a plan that asks for
  // anything else is refused by setRX/setTX rather than silently mis-wired.
  fixtureUart = new oep::FixtureUart(*pinTable, Serial1, 2);
  endpoint.addService(identity);
  endpoint.addService(targetControl);
  endpoint.addService(targetMemory);
  endpoint.addService(targetFlash);
  endpoint.addService(targetConsole);
  endpoint.addService(*fixtureGpio);
  endpoint.addService(*fixtureUart);
}

void loop() {
  endpoint.poll();
  targetConsole.poll();
  if (endpoint.idleFor(1500)) endpoint.abandonAll();
}
