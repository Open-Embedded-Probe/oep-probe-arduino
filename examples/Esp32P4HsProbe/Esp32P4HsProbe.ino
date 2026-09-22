// OEP v0 development probe on ESP32-P4 over the HS OTG vendor bulk interface (EspUsbDevice).
// No target attached on the USB bench: identity, fixture.gpio and fixture.capture only.
// Limits from E160: 1 KiB frames, 16 KiB window (the host coalesces frames into one URB), 16 in flight.
// USB identity is kept identical to the bench's bound device (E104) so the Windows usbipd
// binding survives a reflash: changing VID/PID/serial needs an administrator to bind again.
#include <EspUsbDevice.h>
#include <OepBulkStream.h>
#include <OepEndpoint.h>
#include <OepFixtureCapture.h>
#include <OepFixtureServices.h>
#include <OepProbeIdentity.h>

static EspUsbDevice usbDevice;
static EspUsbDeviceVendor vendor(usbDevice);
static oep::BulkStream bulk(vendor);
static uint8_t rxBuffer[1024];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(bulk, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 16384, 16});
// 'P4HS' HS bench probe; firmware 3.0.0. Reserved: GPIO37/38 (UART0 console).
static constexpr uint64_t kReserved = (1ull << 37) | (1ull << 38);
static constexpr uint64_t kAllPins = (1ull << 55) - 1;
static oep::ProbeIdentity identity({0x50344853u, 0x00030000u, kReserved, kAllPins & ~kReserved});
static uint8_t fixturePins[55];
static size_t fixturePinCount = 0;
static oep::PinTable *pinTable = nullptr;
static oep::FixtureGpio *fixtureGpio = nullptr;
static oep::FixtureCapture *fixtureCapture = nullptr;

void setup() {
  Serial.begin(115200);
  delay(300);
  EspUsbDeviceConfig config;
  config.vid = 0x303a;
  config.pid = 0x4021;
  config.manufacturer = "wch-protocols";
  config.product = "OEP v0 probe (P4 HS bulk)";
  config.serialNumber = "e104-p4-windows-v1";
  config.controller = EspUsbController::HighSpeed;
  config.webusbEnabled = true;
  const bool ok = usbDevice.begin(config);
  for (uint8_t pin = 0; pin < 55; ++pin) if (!((kReserved >> pin) & 1)) fixturePins[fixturePinCount++] = pin;
  pinTable = new oep::PinTable(fixturePins, fixturePinCount);
  fixtureGpio = new oep::FixtureGpio(*pinTable);
  fixtureCapture = new oep::FixtureCapture(*pinTable);
  endpoint.addService(identity);
  endpoint.addService(*fixtureGpio);
  endpoint.addService(*fixtureCapture);
  endpoint.setFlushAfterBurst(true);
  Serial.printf("# OEP P4 HS probe usb_begin=%d transport=vendor-bulk frame=1024 window=16384 inflight=16\n", ok ? 1 : 0);
}

void loop() {
  endpoint.poll();
  if (endpoint.idleFor(1500)) endpoint.abandonAll();
}
