#include <OepPrototype.h>

oep::prototype::Endpoint endpoint(Serial);

void setup() {
  // Prototype transport only. Do not print an ASCII banner on this stream.
  Serial.begin(115200);
}

void loop() {
  endpoint.poll();
}
