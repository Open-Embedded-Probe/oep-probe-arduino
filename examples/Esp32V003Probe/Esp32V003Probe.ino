// OEP v1 probe on the classic ESP32 for the UIAPduino CH32V003 jig (E132 wiring).
// Transport: UART0 through the board's USB-UART bridge at 115200, fixed (probe guide §3.5) - the probe's one transport,
// serial port 0: OEP frames (0x00 <COBS> 0x00) and the raw bytes of its bind on one line (oep-core §3.4). Target link:
// GPIO16 -> PD1/SWIO (single wire, SwioPhy). The bridge's auto-reset circuit resets the ESP32 when the port is opened
// with DTR / RTS in the wrong order: a host opens it with both on (host guide §1).
//
// oep.core (the probe in its describe), oep.wire.swio, oep.target.riscv-dm, oep.target.console, and the fixtures
// oep.fixture.gpio / uart, all revision 1 (no oep.fixture.capture: the v0 GPIO sampler has no revision-1 shape yet).
// GPIO23 -> PD7/NRST is a gpio channel labelled NRST and the default reset line for oep.wire.swio attach-under-reset
// (the host may name another); a plain pin reset (UIAPduino bootloader, PINRSTF) pulses it open drain through
// fixture.gpio. The ESP-IDF I2C / SPI targets are offered under io.github.ch32-riscv-ug.esp32.* (v0 payloads).
#include <OepCh32Dm.h>
#include <OepPinTable.h>
#include <OepP4I2cTarget.h>
#include <OepP4SpiTarget.h>
#include <OepSwioPhy.h>
#include <OepDmConsole.h>
#include <OepBind.h>
#include <OepConfig.h>
#include <OepConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepSampler.h>
#include <OepTarget.h>

// UART0 runs through the board's USB-UART bridge (no flow control) and usbip: long bursts of pipelined
// responses lost bytes (2026-09-22, 2026-09-24). One 512-byte frame in flight keeps the outstanding data small.
static uint8_t rxBuffer[1024];   // the encoded candidate: cobsFrameMax(512)
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {512, 512, 1},
                                  oep::Endpoint::kUartBridge);
// Reserved: GPIO16 (SWIO), GPIO1/3 (UART0 transport), GPIO6-11 (SPI flash), GPIO0/2/12/15 (boot straps; 2 is the LED).
static constexpr uint64_t kReserved = (1ull << 16) | (1ull << 1) | (1ull << 3) | (0x3full << 6) |
                                      (1ull << 0) | (1ull << 2) | (1ull << 12) | (1ull << 15);
// Bonded GPIOs on the jig (E132): 4 5 13 14 17 18 19 21 22 23 25 26 27 32 33 34 35 36 39. 34-39 are input only.
static constexpr uint64_t kBonded = (1ull << 4) | (1ull << 5) | (1ull << 13) | (1ull << 14) | (1ull << 17) | (1ull << 18) |
                                    (1ull << 19) | (1ull << 21) | (1ull << 22) | (1ull << 23) | (1ull << 25) | (1ull << 26) |
                                    (1ull << 27) | (1ull << 32) | (1ull << 33) | (1ull << 34) | (1ull << 35) | (1ull << 36) |
                                    (1ull << 39);
static constexpr uint64_t kFixtures = kBonded & ~kReserved;

// CH32V003 (UIAPduino). The flash layout and the RAM loader are the host's business.
static oep::SwioPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort port{dm, oep::SwioPhy::kPin, 0xffff};
static oep::WireRvswd wire(port, 1, "oep.wire.swio");
static oep::TargetRiscvDm riscvDm(port, 1);
static oep::DmConsole consoleDriver(dm, phy);
static oep::TargetConsoleStream console(port, consoleDriver, 1);
static oep::PinTable pins(kFixtures);
static oep::FixtureGpio gpio(pins, 2);
static oep::FixtureUart uart(pins, Serial2, 3, 2);   // DUT console: V003 PD5/TX -> GPIO22, PD6/RX <- GPIO21 (E132)
// ESP-IDF I2C / SPI slave tools under the project's own names (capability-name-hierarchy.ja.md, decision 4):
// both implementations so far are the ESP-IDF slave drivers, whose quirks stay out of any oep. name.
// fixture.capture revision 1: the core-0 GPIO sampler, one-shot, one byte per sample (up to 8 lines, 0.4..2 MHz)
static oep::SamplerCapture capture(endpoint, kReserved);
static oep::P4I2cTarget i2c(pins);
static oep::P4SpiTarget spi(pins);
static oep::Binds binds;
static oep::ProbeConfig config(endpoint, binds);   // one slot (this jig's SWIO), port 0's bind, saved in NVS
static uint8_t probeTlv[200];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];
  oep::describeCore(w, "esp32-d0wd", id, oep::platformUnitId(id, sizeof id), 40, kReserved);
  w.text(oep::kCoreProfile, "io.github.ch32-riscv-ug.esp32-v003");
  w.label(16, "SWIO");
  w.label(23, "NRST");
  w.label(22, "DUT TX");
  w.label(21, "DUT RX");
  return w.ok() ? w.length() : 0;
}

void setup() {
  // This jig's wiring (fixed): channel 21 drives the DUT PD6 / RX, which must not float while no UART holds it (a floating
  // RX line fed the DUT's command parser noise, 2026-09-22). Idle = pull-up; every other free pin stays Hi-Z.
  pins.setIdle(21, oep::PinTable::kIdlePullUp);
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  phy.begin(oep::SwioPhy::kPin);
  // E132: every UIAP pin is wired here, including the software-USB pair (PD3/PD4); a permanent
  // ESP32 pull on either USB line breaks enumeration. Idle must be genuinely high impedance.
  oep::platformParkMask(kFixtures);
  endpoint.setRawPorts(&binds);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(esp_random());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  port.reset_default = 23;   // attach-under-reset through the V003's NRST unless the host names another channel
  port.reset_allowed = kFixtures;
  console.setMaxRead(480);   // 512-byte frames
  endpoint.add(console);
  endpoint.add(gpio);
  endpoint.add(uart);
  endpoint.add(i2c);
  endpoint.add(spi);
  endpoint.add(capture);
  endpoint.add(config);   // last: the fns before it keep their numbers
  config.addPlace(wire, console);
  config.addUart(uart);
  config.setPins(&pins);
  config.load();
  config.applySaved();
}

void loop() {
  endpoint.poll();
  console.poll();
  config.poll();
  uart.poll();
  capture.poll();
  i2c.service();
  spi.service();
}
