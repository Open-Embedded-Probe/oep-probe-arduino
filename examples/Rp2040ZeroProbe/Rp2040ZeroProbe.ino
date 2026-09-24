// OEP v1 draft probe on a Waveshare RP2040-Zero (oep-spec docs/v1-core-wire-delta.ja.md): the ordinary-ARM counterpart
// of the CH32 probes. Its GP0/GP1 reach the Pro Micro RP2350's SWD port (SWCLK = GP0, SWDIO = GP1, DPIDR 0x4c013477,
// measured 2026-09-23).
//
// Transport: USB CDC (Serial, length-prefixed frames).
// v1 draft: oep.core (the probe in its describe), oep.wire.swd, oep.target.arm-adi, oep.fixture.gpio / uart.
#include <OepFixtureServices.h>
#include <OepV1Endpoint.h>
#include <OepV1Fixture.h>
#include <OepV1Swd.h>
#include <pico/unique_id.h>

static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::v1::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8});
static constexpr uint8_t kSwclk = 0, kSwdio = 1;
// GP16 drives the on-board WS2812; GP0/GP1 are the SWD pair. GP0..GP15 and GP26..GP29 reach the castellated edge.
static constexpr uint64_t kReserved = (uint64_t{1} << kSwclk) | (uint64_t{1} << kSwdio) | (uint64_t{1} << 16);
static constexpr uint64_t kBonded = 0xffffu | (0xfull << 26);
static uint8_t fixturePins[32];
static size_t fixturePinCount = 0;

static oep::v1::SwdPort port{kSwdio, kSwclk};
static oep::v1::WireSwd wire(port, 1);
static oep::v1::TargetArmAdi adi(port, 1);
static oep::PinTable *pins = nullptr;
static oep::FixtureGpio *gpio = nullptr;
static oep::FixtureUart *uart = nullptr;
static oep::v1::V0Fixture *gpioV1 = nullptr, *uartV1 = nullptr;
static const uint8_t kGpioRoles[] = {1};
static const uint8_t kUartRoles[] = {1, 2};
static const uint8_t kUartExtra[] = {oep::v1::kTagImplementation, 1, 2};
static uint8_t probeTlv[200];

static size_t describeProbe() {
  oep::v1::TlvWriter w(probeTlv, sizeof probeTlv);
  w.text(oep::v1::kCoreFirmware, "3.1.0-v1draft");
  w.text(oep::v1::kCoreModel, "waveshare-rp2040-zero");
  pico_unique_board_id_t id;
  pico_get_unique_board_id(&id);
  w.put(oep::v1::kCoreUnitId, id.id, sizeof id.id);
  w.u16(oep::v1::kCoreChannels, 30);
  uint8_t bitmap[2 + 4] = {0, 0};
  for (int i = 0; i < 4; ++i) bitmap[2 + i] = static_cast<uint8_t>(kReserved >> (8 * i));
  w.put(oep::v1::kCoreReserved, bitmap, sizeof bitmap);
  w.text(oep::v1::kCoreProfile, "io.github.ch32-riscv-ug.rp2040zero-rp2350-swd");
  w.label(kSwclk, "SWCLK");
  w.label(kSwdio, "SWDIO");
  return w.ok() ? w.length() : 0;
}

void setup() {
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
  Serial.begin(115200);
  for (uint8_t pin = 0; pin < 30; ++pin)
    if (((kBonded >> pin) & 1) && !((kReserved >> pin) & 1)) fixturePins[fixturePinCount++] = pin;
  pins = new oep::PinTable(fixturePins, fixturePinCount);
  // Hi-Z everything the probe does not own (RP2 pads boot with a pull-down).
  oep::platformParkPins(fixturePins, fixturePinCount);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(rp2040.hwrand32());
  endpoint.add(wire);
  endpoint.add(adi);
  gpio = new oep::FixtureGpio(*pins);
  uart = new oep::FixtureUart(*pins, Serial1, 2);
  gpioV1 = new oep::v1::V0Fixture(*gpio, "oep.fixture.gpio", 2, *pins, kGpioRoles, 1, 1u << 2);
  uartV1 = new oep::v1::V0Fixture(*uart, "oep.fixture.uart", 3, *pins, kUartRoles, 2, 0, kUartExtra, sizeof kUartExtra);
  endpoint.add(*gpioV1);
  endpoint.add(*uartV1);
}

void loop() { endpoint.poll(); }
