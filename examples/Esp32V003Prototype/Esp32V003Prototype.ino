#include <OepPrototype.h>

class PendingV003Backend : public oep::prototype::TargetControlBackend {
 public:
  oep::prototype::BackendResult getStatus(
      oep::prototype::TargetStatus&) override {
    return oep::prototype::BackendResult::Unavailable;
  }
  oep::prototype::BackendResult normalizeUser() override {
    return oep::prototype::BackendResult::Unavailable;
  }
  oep::prototype::BackendResult enterProductBootloader() override {
    return oep::prototype::BackendResult::Unavailable;
  }
};

PendingV003Backend target;
oep::prototype::Endpoint endpoint(Serial, &target);

void setup() {
  // Prototype transport only. Do not print an ASCII banner on this stream.
  Serial.begin(115200);
}

void loop() {
  endpoint.poll();
}
