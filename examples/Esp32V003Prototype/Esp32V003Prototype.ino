#include <OepPrototype.h>
#include <V003SwioTargetControl.h>

oep::prototype::V003SwioTargetControl target(16);
oep::prototype::Endpoint endpoint(Serial, &target, &target);

void setup() {
  // Prototype transport only. Do not print an ASCII banner on this stream.
  Serial.begin(115200);
  target.begin();
}

void loop() {
  endpoint.poll();
}
