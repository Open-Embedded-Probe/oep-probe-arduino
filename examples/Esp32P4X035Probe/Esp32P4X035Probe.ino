// OEP v1 draft probe on ESP32-P4 for the CH32X035 fixture (oep-spec docs/v1-core-wire-delta.ja.md).
// Transport: USB-Serial/JTAG (HWCDC). Limits from E155: 1 KiB frames, 4 KiB window.
//
// v1 draft: oep.core (the probe described in its describe), oep.wire.rvswd (scan / attach / detach),
// oep.target.riscv-dm (DMI step lists, block read/write, run until halt, halt / resume, ndmreset),
// oep.target.console (a position-addressed stream of the target's DM console). The v0 fixtures come back
// as v1 interfaces next (decided 2026-09-24: v0 may go away until v1 exists).
#include <OepCh32Dm.h>
#include <OepRvswdPhy.h>
#include <OepTargetServices.h>
#include <OepV1Console.h>
#include <OepV1Endpoint.h>
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

// Reserved: GPIO2/54 (RVSWD), GPIO24/25 (USB-Serial/JTAG).
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
  const uint64_t reserved = (1ull << 2) | (1ull << 24) | (1ull << 25) | (1ull << 54);
  uint8_t bitmap[2 + 7] = {0, 0};
  for (int i = 0; i < 7; ++i) bitmap[2 + i] = static_cast<uint8_t>(reserved >> (8 * i));
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
}

void loop() {
  endpoint.poll();
  console.poll();
}
