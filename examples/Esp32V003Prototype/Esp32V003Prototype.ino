#include <OepPrototype.h>
#include <Esp32FixtureGpio.h>
#include <V003SwioTargetControl.h>

oep::prototype::V003SwioTargetControl target(16);
const uint8_t fixturePins[] = {25, 26, 5, 19, 18, 17, 33, 27,
                               4, 14, 13, 32, 22, 21};
oep::prototype::Esp32FixtureGpio fixtureGpio(
    fixturePins, sizeof(fixturePins));
oep::prototype::Endpoint endpoint(
    Serial, &target, &target, &target, &fixtureGpio);

void setup() {
  // Prototype transport only. Do not print an ASCII banner on this stream.
  Serial.begin(115200);
  target.begin();
}

void loop() {
  endpoint.poll();
}
