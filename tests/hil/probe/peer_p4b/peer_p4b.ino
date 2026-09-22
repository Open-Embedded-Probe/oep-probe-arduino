// Peer for the fixture-service HIL over the 8-wire P4 link (E154: 33,32,26..31 pin to pin).
// UART echo: peer RX = GPIO26 (probe TX), peer TX = GPIO27 (probe RX), 115200.
// GPIO mirror: peer copies the level of GPIO28 (probe drives) onto GPIO29 (probe samples).
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 26, 27);
  pinMode(28, INPUT);
  pinMode(29, OUTPUT);
}

void loop() {
  digitalWrite(29, digitalRead(28));
  while (Serial1.available()) Serial1.write(Serial1.read());
  if (Serial.available() && Serial.read() == '?') Serial.println("# HIL peer_p4b uart26/27 mirror28->29");
}
