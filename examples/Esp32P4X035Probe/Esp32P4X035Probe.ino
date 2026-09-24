// OEP v1 draft probe on ESP32-P4 for the CH32X035 fixture (oep-spec docs/v1-core-wire-delta.ja.md).
// Transport: USB-Serial/JTAG (HWCDC). Limits from E155: 1 KiB frames, 4 KiB window.
//
// v1 draft: oep.core (the probe described in its describe), oep.wire.rvswd (scan / attach / detach),
// oep.target.riscv-dm (DMI step lists, block read/write, run until halt, halt / resume, ndmreset),
// oep.target.console (a position-addressed stream of the target's DM console), and the fixtures
// oep.fixture.gpio / uart (x2) / capture - the v0 services under v1 names, their operations unchanged.
#include <OepCh32Dm.h>
#include <OepFixtureCapture.h>
#include <OepFixtureServices.h>
#include <OepRvswdPhy.h>
#include <OepTargetServices.h>
#include <OepV1Console.h>
#include <OepV1Endpoint.h>
#include <OepV1Fixture.h>
#include <OepV1Target.h>

static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::v1::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8});

// Fixture wiring: P4 GPIO2 -> X035 PC18 (SWDIO), GPIO54 -> PC19 (SWCLK). CH32X035F8U6: 62 KiB, 256-byte pages.
static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy, {0x08000000u, 63488u, 256u, 256u});
static oep::v1::DebugPort port{dm, 2, 54};
static oep::v1::WireRvswd wire(port, 1);
static oep::v1::TargetRiscvDm riscvDm(port, 1);
static oep::TargetConsole consoleDriver(dm, phy);   // the DM console framings (SDI / DMDATA / dmseq)
static oep::v1::TargetConsoleStream console(port, consoleDriver, 1);

// Reserved: GPIO2/54 (RVSWD), GPIO24/25 (USB-Serial/JTAG). Every other GPIO is a fixture channel.
static constexpr uint64_t kReserved = (1ull << 2) | (1ull << 24) | (1ull << 25) | (1ull << 54);
static uint8_t fixturePins[55];
static size_t fixturePinCount = 0;
static oep::PinTable *pins = nullptr;
static oep::FixtureGpio *gpio = nullptr;
static oep::FixtureUart *uart1 = nullptr, *uart2 = nullptr;   // uart2: X035 USART2 tests (GPIO48/49)
static oep::FixtureCapture *capture = nullptr;
static oep::v1::V0Fixture *gpioV1 = nullptr, *uart1V1 = nullptr, *uart2V1 = nullptr, *captureV1 = nullptr;
static const uint8_t kGpioRoles[] = {1};                      // line
static const uint8_t kUartRoles[] = {1, 2};                   // RX, TX
static const uint8_t kCaptureRoles[] = {0, 1, 2, 3, 4, 5, 6, 7};   // line k
static const uint8_t kUartExtra[] = {oep::v1::kTagImplementation, 1, 2};   // peripheral
static const uint8_t kCaptureExtra[] = {oep::v1::kTagImplementation, 1, 3};   // peripheral + DMA
static uint8_t probeTlv[160];

static size_t describeProbe() {
  oep::v1::TlvWriter w(probeTlv, sizeof probeTlv);
  w.text(oep::v1::kCoreFirmware, "3.1.0-v1draft");
  w.text(oep::v1::kCoreModel, "esp32-p4-devkit");
  const uint64_t mac = ESP.getEfuseMac();   // the unit id: the probe says who it is on any transport
  uint8_t id[6];
  for (int i = 0; i < 6; ++i) id[i] = static_cast<uint8_t>(mac >> (8 * i));
  w.put(oep::v1::kCoreUnitId, id, sizeof id);
  w.u16(oep::v1::kCoreChannels, 55);
  uint8_t bitmap[2 + 7] = {0, 0};
  for (int i = 0; i < 7; ++i) bitmap[2 + i] = static_cast<uint8_t>(kReserved >> (8 * i));
  w.put(oep::v1::kCoreReserved, bitmap, sizeof bitmap);
  w.text(oep::v1::kCoreProfile, "io.github.ch32-riscv-ug.p4-x035");
  w.label(2, "SWDIO");
  w.label(54, "SWCLK");
  w.label(51, "LED");
  return w.ok() ? w.length() : 0;
}

void setup() {
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);
  phy.begin(2, 54);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(esp_random());
  endpoint.add(wire);
  endpoint.add(riscvDm);
  endpoint.add(console);
  for (uint8_t pin = 0; pin < 55; ++pin) if (!((kReserved >> pin) & 1)) fixturePins[fixturePinCount++] = pin;
  pins = new oep::PinTable(fixturePins, fixturePinCount);
  gpio = new oep::FixtureGpio(*pins);
  uart1 = new oep::FixtureUart(*pins, Serial1, 2);
  uart2 = new oep::FixtureUart(*pins, Serial2, 5);
  capture = new oep::FixtureCapture(*pins);
  // lock-free: gpio read_bank (op 2); capture status (3) and read (4). UART reads consume, so they need the lock.
  gpioV1 = new oep::v1::V0Fixture(*gpio, "oep.fixture.gpio", 2, *pins, kGpioRoles, 1, 1u << 2);
  uart1V1 = new oep::v1::V0Fixture(*uart1, "oep.fixture.uart", 3, *pins, kUartRoles, 2, 0, kUartExtra, sizeof kUartExtra);
  uart2V1 = new oep::v1::V0Fixture(*uart2, "oep.fixture.uart", 4, *pins, kUartRoles, 2, 0, kUartExtra, sizeof kUartExtra);
  captureV1 = new oep::v1::V0Fixture(*capture, "oep.fixture.capture", 5, *pins, kCaptureRoles, 8, (1u << 3) | (1u << 4),
                                     kCaptureExtra, sizeof kCaptureExtra);
  endpoint.add(*gpioV1);
  endpoint.add(*uart1V1);
  endpoint.add(*uart2V1);
  endpoint.add(*captureV1);
}

void loop() {
  endpoint.poll();
  console.poll();
}
