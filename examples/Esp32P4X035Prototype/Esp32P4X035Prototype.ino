#include <OepPrototype.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <Esp32FixtureGpio.h>
#include <Esp32FixtureUart.h>
#include <X035RvswdTargetControl.h>

// ESP32-P4 GPIO calls are slow enough to satisfy the fixture wiring without
// an additional microsecond delay. The backend remains configurable for
// longer/noisier wiring.
oep::prototype::X035RvswdTargetControl target(2, 54, 0);
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
oep::prototype::Endpoint endpoint(
    Serial, &target, &target, &target, &fixtureGpio, &fixtureUart);

// Temporary I2C peer for X035 HIL. PC16(SCL)->GPIO52 and
// PC17(SDA)->GPIO50 are the X035 route-2 pair. Keep this separate from the
// OEP protocol until request/result buffering and error reporting are defined.
constexpr uint8_t kI2cPeerAddress = 0x42;
void onI2cRequest() { Wire.write(0xA5); Wire.write(0x5A); }
void onI2cReceive(int) { while (Wire.available()) (void)Wire.read(); }

void setup() {
  Serial.begin(115200);
  target.begin();
  Wire.begin(kI2cPeerAddress, 50, 52, 100000);
  gpio_set_pull_mode(GPIO_NUM_50, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(GPIO_NUM_52, GPIO_PULLUP_ONLY);
  Wire.onRequest(onI2cRequest);
  Wire.onReceive(onI2cReceive);
}

void loop() { endpoint.poll(); }
