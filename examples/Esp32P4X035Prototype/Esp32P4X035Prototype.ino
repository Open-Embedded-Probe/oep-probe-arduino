#include <OepPrototype.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <Esp32FixtureGpio.h>
#include <Esp32FixtureIdfI2c.h>
#include <Esp32P4ProbeCapabilities.h>
#include <Esp32FixtureUart.h>
#include <StaticProbeInfo.h>
#include <X035RvswdTargetControl.h>

// ESP32-P4 GPIO calls are slow enough to satisfy the fixture wiring without
// an additional microsecond delay. The backend remains configurable for
// longer/noisier wiring.
// The X035 fixture has passed full-image hash verification with no extra
// post-frame guard.  Other wiring must retain the backend default (20 us)
// until it has its own validation evidence.
oep::prototype::X035RvswdTargetControl target(2, 54, 0, 0);
// Read-only discovery surface. GPIO2 and GPIO54 are the RVSWD transport;
// every other P4 GPIO is kept as an input and may be sampled by the host.
const uint8_t fixturePins[] = {
    0, 1,    3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
    19, 20, 21, 22, 23,                 28, 29, 30, 31, 32, 33, 34,
    35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
    51, 52, 53};
oep::prototype::Esp32FixtureGpio fixtureGpio(
    fixturePins, sizeof(fixturePins));
// X035 USART4 reset-default route: PB0 (DUT TX) -> GPIO12 (probe RX),
// PB1 (DUT RX) <- GPIO6 (probe TX). USART4 is selected for fixture tests
// because both board header signals are mapped; USART2 is retained for
// diagnosing its independent long-TX issue.
oep::prototype::Esp32FixtureUart fixtureUart(Serial1, 12, 6);
// X035 route-2 pair: PC16(SCL)->GPIO52, PC17(SDA)->GPIO50. The software
// target is retained as a 10 kHz fault-isolation backend. This image bypasses
// Arduino Wire and uses the ESP-IDF I2C1 slave driver directly.
oep::prototype::Esp32FixtureIdfI2c fixtureI2c(
    I2C_NUM_1, 0x42, 50, 52, 10000);
// `P4DV` identifies the generic P4 development-probe hardware/firmware, not
// the attached DUT. GPIO2/54 are used by this firmware's target transport;
// GPIO24--27 are reserved by USB. ProbeCapabilities is the authoritative,
// paged declaration used by new hosts.
oep::prototype::StaticProbeInfo probeInfo({
    0x50344456u, 2u,
    (uint64_t{1} << 2) | (uint64_t{1} << 24) | (uint64_t{1} << 25) |
        (uint64_t{1} << 26) | (uint64_t{1} << 27) | (uint64_t{1} << 54),
    0x003ffffffffffffbull});
oep::prototype::Esp32P4ProbeCapabilities probeCapabilities;
oep::prototype::Endpoint endpoint(
    Serial, &target, &target, &target, &fixtureGpio, &fixtureUart, &fixtureI2c,
    &probeInfo, &probeCapabilities);

void setup() {
  Serial.begin(115200);
  target.begin();
  fixtureI2c.begin();
}

void loop() {
  endpoint.poll();
  // A killed host process cannot send normalize-user.  Do not leave the DUT
  // halted indefinitely: after a deliberately conservative idle interval,
  // reset/release the RVSWD session.  Normal full-image requests arrive much
  // more frequently than this interval.
  if (target.sessionActive() && endpoint.idleFor(1500)) {
    target.normalizeUser();
  }
}
