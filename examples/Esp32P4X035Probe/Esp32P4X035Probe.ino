// OEP v0 development probe on ESP32-P4 for the CH32X035 fixture.
// Transport: USB-Serial/JTAG (HWCDC). Limits from E155: 1 KiB frames, 4 KiB window.
#include <OepEndpoint.h>
#include <OepProbeIdentity.h>

static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer,
                              {1024, 4096, 8});
// 'P4DV' generic P4 development probe; firmware 3.0.0; GPIO2/54 RVSWD and GPIO24-27 USB reserved.
static oep::ProbeIdentity identity({0x50344456u, 0x00030000u,
                                    (1ull << 2) | (1ull << 24) | (1ull << 25) | (1ull << 26) | (1ull << 27) | (1ull << 54),
                                    0x003ffffffffffffbull});

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  endpoint.addService(identity);
}

void loop() {
  endpoint.poll();
  if (endpoint.idleFor(1500)) endpoint.abandonAll();
}
