// OEP v1 probe on a Waveshare RP2040-Zero (oep-spec docs/v1-core-wire-delta.ja.md): the ordinary-ARM counterpart
// of the CH32 probes. Its GP0/GP1 reach the Pro Micro RP2350's SWD port (SWCLK = GP0, SWDIO = GP1, DPIDR 0x4c013477,
// measured 2026-09-23).
//
// Transport: USB CDC (Serial, length-prefixed frames).
// oep.core (the probe in its describe), oep.wire.swd, oep.target.arm-adi, oep.fixture.gpio / uart (revision 1).
#include <OepFixtureServices.h>
#include <OepV1Endpoint.h>
#include <OepV1Fixture.h>
#include <OepV1Swd.h>

static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::v1::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8});
static constexpr uint8_t kSwclk = 0, kSwdio = 1;
// GP16 drives the on-board WS2812; GP0/GP1 are the SWD pair. GP0..GP15 and GP26..GP29 reach the castellated edge.
static constexpr uint64_t kReserved = (uint64_t{1} << kSwclk) | (uint64_t{1} << kSwdio) | (uint64_t{1} << 16);
static constexpr uint64_t kBonded = 0xffffu | (0xfull << 26);
static constexpr uint64_t kFixtures = kBonded & ~kReserved;

static oep::v1::SwdPort port{kSwdio, kSwclk};
static oep::v1::WireSwd wire(port, 1);
static oep::v1::TargetArmAdi adi(port, 1);
static oep::PinTable pins(kFixtures);
static oep::v1::FixtureGpio gpio(pins, 2);
static oep::v1::FixtureUart uart(pins, Serial1, 3, 2);
static uint8_t probeTlv[200];

static size_t describeProbe() {
  oep::v1::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];
  oep::v1::describeCore(w, "waveshare-rp2040-zero", id, oep::platformUnitId(id, sizeof id), 30, kReserved);
  w.text(oep::v1::kCoreProfile, "io.github.ch32-riscv-ug.rp2040zero-rp2350-swd");
  w.label(kSwclk, "SWCLK");
  w.label(kSwdio, "SWDIO");
  return w.ok() ? w.length() : 0;
}

void setup() {
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
  Serial.begin(115200);
  // Hi-Z everything the probe does not own (RP2 pads boot with a pull-down).
  oep::platformParkMask(kFixtures);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(rp2040.hwrand32());
  endpoint.add(wire);
  endpoint.add(adi);
  endpoint.add(gpio);
  endpoint.add(uart);
}

void loop() {
  endpoint.poll();
  uart.poll();
}
