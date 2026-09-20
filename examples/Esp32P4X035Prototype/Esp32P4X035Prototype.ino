#include <OepPrototype.h>
#include <X035RvswdTargetControl.h>

oep::prototype::X035RvswdTargetControl target(2, 54);
oep::prototype::Endpoint endpoint(Serial, &target, &target, &target);

void setup() {
  Serial.begin(115200);
  target.begin();
}

void loop() { endpoint.poll(); }
