// OEP v0 probe on a Waveshare RP2040-Zero, for now as the peer of the Pro Micro RP2350
// probe: with fixture.gpio on both boards the host can drive one side and sample the
// other, which is how the bench finds what is actually wired (the same method as the
// ESP32-P4 pin map). No target services yet - nothing CH32 is wired to this board.
//
// Transport: USB CDC (Serial).
#include <OepEndpoint.h>
#include <OepFixtureServices.h>
#include <OepProbeIdentity.h>

static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer,
                              {1024, 4096, 8});
// 'R204' RP2040-Zero probe; firmware 3.0.0. GP16 drives the on-board WS2812.
static constexpr uint64_t kReserved = uint64_t{1} << 16;
static constexpr uint64_t kBonded =
    0xffffu | (0xfull << 26);   // GP0..GP15 and GP26..GP29 reach the castellated edge
static oep::ProbeIdentity identity({0x52323034u, 0x00030000u, kReserved, kBonded & ~kReserved});
static uint8_t fixturePins[32];
static size_t fixturePinCount = 0;
static oep::PinTable *pinTable = nullptr;
static oep::FixtureGpio *fixtureGpio = nullptr;
static oep::FixtureUart *fixtureUart = nullptr;

void setup() {
  Serial.begin(115200);
  for (uint8_t pin = 0; pin < 30; ++pin)
    if (((kBonded >> pin) & 1) && !((kReserved >> pin) & 1)) fixturePins[fixturePinCount++] = pin;
  pinTable = new oep::PinTable(fixturePins, fixturePinCount);
  // Hi-Z everything the probe does not own. RP2 pads boot with a pull-down, and this jig
  // is only half wired: on the CH32L103 that pull-down held a line the target cares about
  // and the hart would not halt, though its debug module answered normally (2026-09-23).
  oep::platformParkPins(fixturePins, fixturePinCount);
  fixtureGpio = new oep::FixtureGpio(*pinTable);
  fixtureUart = new oep::FixtureUart(*pinTable, Serial1, 2);
  endpoint.addService(identity);
  endpoint.addService(*fixtureGpio);
  endpoint.addService(*fixtureUart);
}

void loop() {
  endpoint.poll();
  if (endpoint.idleFor(1500)) endpoint.abandonAll();
}
