// OEP v1 draft probe on a SparkFun Pro Micro RP2350 for the CH32L103 jig (oep-spec docs/v1-core-wire-delta.ja.md).
// Transport: USB CDC (Serial, length-prefixed frames). Target link: RVSWD on two GPIOs through the SIO
// backend in OepRvswdPhy.cpp (same frame as the ESP32-P4 probe).
//
// v1 draft: oep.core (the probe in its describe), oep.wire.rvswd, oep.target.riscv-dm, oep.target.console,
// oep.fixture.gpio / uart. GP2 is the target's NRST: a gpio channel labelled NRST and the default reset line for
// oep.wire.rvswd attach-under-reset (the host may name another channel) - open drain only, never driven high.
#include <OepCh32Dm.h>
#include <OepFixtureServices.h>
#include <OepRvswdPhy.h>
#include <OepTargetServices.h>
#include <OepV1Console.h>
#include <OepV1Endpoint.h>
#include <OepV1Fixture.h>
#include <OepV1Target.h>
#include <pico/unique_id.h>

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
static oep::v1::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8});
static constexpr uint8_t kSwdio = OEP_RVSWD_SWDIO, kSwclk = OEP_RVSWD_SWCLK;
// GP2 is the target's NRST: measured 2026-09-23 by pulling each spare channel down on its
// own and watching which one took the debug module away. It idles high on the CH32's own
// pull-up, which is why the RP2 pad's default pull-down used to hold the part in reset.
static constexpr uint8_t kNrst = 2;
// GP19 is the PSRAM chip select on this board; leave it alone.
static constexpr uint64_t kReserved = (uint64_t{1} << kSwdio) | (uint64_t{1} << kSwclk) | (uint64_t{1} << 19);
static uint8_t fixturePins[30];
static size_t fixturePinCount = 0;

// CH32L103C8T6: 64 KiB flash from 0x08000000, 256-byte pages, QingKe V4 - the same DM profile as the CH32X035.
static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy, {0x08000000u, 65536u, 256u, 256u});
static oep::v1::DebugPort port{dm, kSwdio, kSwclk};
static oep::v1::WireRvswd wire(port, 1);
static oep::v1::TargetRiscvDm riscvDm(port, 1);
static oep::TargetConsole consoleDriver(dm, phy);
static oep::v1::TargetConsoleStream console(port, consoleDriver, 1);
static oep::PinTable *pins = nullptr;
static oep::FixtureGpio *gpio = nullptr;
static oep::FixtureUart *uart = nullptr;   // UART0 reaches GP0/1, GP12/13, GP16/17 and GP28/29 on this part
static oep::v1::V0Fixture *gpioV1 = nullptr, *uartV1 = nullptr;
static const uint8_t kGpioRoles[] = {1};
static const uint8_t kUartRoles[] = {1, 2};
static const uint8_t kUartExtra[] = {oep::v1::kTagImplementation, 1, 2};
static uint8_t probeTlv[200];

static size_t describeProbe() {
  oep::v1::TlvWriter w(probeTlv, sizeof probeTlv);
  w.text(oep::v1::kCoreFirmware, "3.1.0-v1draft");
  w.text(oep::v1::kCoreModel, "sparkfun-promicro-rp2350");
  pico_unique_board_id_t id;   // the flash's unique id: the probe says who it is on any transport
  pico_get_unique_board_id(&id);
  w.put(oep::v1::kCoreUnitId, id.id, sizeof id.id);
  w.u16(oep::v1::kCoreChannels, 30);
  uint8_t bitmap[2 + 4] = {0, 0};
  for (int i = 0; i < 4; ++i) bitmap[2 + i] = static_cast<uint8_t>(kReserved >> (8 * i));
  w.put(oep::v1::kCoreReserved, bitmap, sizeof bitmap);
  w.text(oep::v1::kCoreProfile, "io.github.ch32-riscv-ug.rp2350-l103");
  w.label(kSwdio, "SWDIO");
  w.label(kSwclk, "SWCLK");
  w.label(kNrst, "NRST");
  return w.ok() ? w.length() : 0;
}

void setup() {
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
  Serial.begin(115200);
  // The RP2 pad's default pull-down on NRST holds the target in reset. Release it and never drive it high -
  // the target pulls it up.
  pinMode(kNrst, INPUT);
  phy.begin(kSwdio, kSwclk);
  phy.setMinHalfNs(OEP_RVSWD_MIN_HALF_NS);
  // A CH32L103 resets its debug link when the bus rests high (2026-09-23). Where this setting belongs - the
  // jig's profile here, or the host at attach - waits for a measurement (a LinkE -> L103 capture or an OEP
  // observation, decided 2026-09-24).
  phy.setIdleClockLow(true);
  for (uint8_t pin = 0; pin < 30; ++pin) if (!((kReserved >> pin) & 1)) fixturePins[fixturePinCount++] = pin;
  pins = new oep::PinTable(fixturePins, fixturePinCount);
  // Hi-Z everything the probe does not own. RP2 pads boot with a pull-down, and this jig is only half wired:
  // on the CH32L103 that pull-down held a line the target cares about and the hart would not halt, though its
  // debug module answered normally (2026-09-23).
  oep::platformParkPins(fixturePins, fixturePinCount);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(rp2040.hwrand32());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  port.reset_default = kNrst;   // attach-under-reset through the L103's NRST unless the host names another channel
  for (size_t i = 0; i < fixturePinCount; ++i) port.reset_allowed |= uint64_t{1} << fixturePins[i];
  endpoint.add(console);
  gpio = new oep::FixtureGpio(*pins);
  uart = new oep::FixtureUart(*pins, Serial1, 2);
  gpioV1 = new oep::v1::V0Fixture(*gpio, "oep.fixture.gpio", 2, *pins, kGpioRoles, 1, 1u << 2);
  uartV1 = new oep::v1::V0Fixture(*uart, "oep.fixture.uart", 3, *pins, kUartRoles, 2, 0, kUartExtra, sizeof kUartExtra);
  endpoint.add(*gpioV1);
  endpoint.add(*uartV1);
}

void loop() {
  endpoint.poll();
  console.poll();
}
