// OEP v1 probe on a SparkFun Pro Micro RP2350 for the CH32L103 jig (oep-spec docs/v1-core-wire-delta.ja.md).
// Transport: USB CDC (Serial, length-prefixed frames). Target link: RVSWD on two GPIOs through the SIO
// backend in OepRvswdPhy.cpp (same frame as the ESP32-P4 probe).
//
// oep.core (the probe in its describe), oep.wire.rvswd, oep.target.riscv-dm, oep.target.console,
// oep.fixture.gpio / uart (all revision 1). GP2 is the target's NRST: a gpio channel labelled NRST and the default reset line for
// oep.wire.rvswd attach-under-reset (the host may name another channel) - open drain only, never driven high.
#include <OepCh32Dm.h>
#include <OepFixtureServices.h>
#include <OepRvswdPhy.h>
#include <OepDmConsole.h>
#include <OepV1Console.h>
#include <OepV1Endpoint.h>
#include <OepV1Fixture.h>
#include <OepV1Target.h>

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
// The speed search takes the fastest period that passes, and on the CH32L103 that is marginal right after a reset,
// where flashing happens on its slow default clock: with no floor it settled at 680 kHz SWCLK, saw parity retries,
// and one sketch in 28 failed its flash verify even after rewrites; with 500 ns, 42 of 42 passed (2026-09-25,
// after the RP2 timing fix and the reset rework). Not the wiring - it is the same as the other targets.
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
static constexpr uint64_t kFixtures = ((1ull << 30) - 1) & ~kReserved;

// CH32L103C8T6. The flash layout is the host's business.
static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy);
static oep::v1::DebugPort port{dm, kSwdio, kSwclk};
static oep::v1::WireRvswd wire(port, 1);
static oep::v1::TargetRiscvDm riscvDm(port, 1);
static oep::DmConsole consoleDriver(dm, phy);
static oep::v1::TargetConsoleStream console(port, consoleDriver, 1);
static oep::PinTable pins(kFixtures);
static oep::v1::FixtureGpio gpio(pins, 2);
static oep::v1::FixtureUart uart(pins, Serial1, 3, 2);   // UART0 reaches GP0/1, GP12/13, GP16/17 and GP28/29 on this part
static uint8_t probeTlv[200];

static size_t describeProbe() {
  oep::v1::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];   // the flash's unique id: the probe says who it is on any transport
  oep::v1::describeCore(w, "sparkfun-promicro-rp2350", id, oep::platformUnitId(id, sizeof id), 30, kReserved);
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
  // Hi-Z everything the probe does not own. RP2 pads boot with a pull-down, and this jig is only half wired:
  // on the CH32L103 that pull-down held a line the target cares about and the hart would not halt, though its
  // debug module answered normally (2026-09-23).
  oep::platformParkMask(kFixtures);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(rp2040.hwrand32());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  port.reset_default = kNrst;   // attach-under-reset through the L103's NRST unless the host names another channel
  port.reset_allowed = kFixtures;
  endpoint.add(console);
  endpoint.add(gpio);
  endpoint.add(uart);
}

void loop() {
  endpoint.poll();
  console.poll();
  uart.poll();
}
