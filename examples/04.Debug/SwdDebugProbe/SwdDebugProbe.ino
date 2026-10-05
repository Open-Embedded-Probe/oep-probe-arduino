// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A debugger for ARM Cortex-M chips on SWD, on any RP2040 / RP2350 board (oep-spec docs/oep-if-debug.ja.md §5-§6):
//
//   oep.wire.swd        scan / attach / detach: wakes the port (JTAG-to-SWD, then dormant-to-SWD for an SWD v2 port
//                       such as the RP2350's), multidrop TARGETSEL when the host gives one
//   oep.target.arm-adi  DP / AP transfer lists and block transfers over the connection: the host builds MEM-AP access,
//                       halting through the DHCSR, flashing - the probe only moves the words
//
// Wire kSwclk / kSwdio to the target's SWCLK / SWDIO and GND. Firmware/OepProbe has the same wire with host-chosen pins.
//
// From Python (oep-client-python):
//   from oep_client import arm, link
//   hst = link.open_host("<port>"); hst.open(3000)
//   wire = arm.SwdWire(hst)
//   conn, dpidr, dormant = wire.attach()
//   print(hex(dpidr))
#include <OepEndpoint.h>
#include <OepPlatform.h>
#include <OepSwd.h>

static constexpr uint8_t kSwclk = 2, kSwdio = 3;   // GP2 -> SWCLK, GP3 -> SWDIO
static constexpr uint64_t kChannels = (1ull << kSwdio) | (1ull << kSwclk);
static constexpr uint16_t kChannelCount = (kSwdio > kSwclk ? kSwdio : kSwclk) + 1;

static uint8_t rxBuffer[1100];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8},
                              oep::Endpoint::kUsbCdc, 0);   // CDC communication interface 0 (core §7.5)

static oep::SwdPort port{kSwdio, kSwclk};
static oep::WireSwd wire(port, 0);
static oep::TargetArmAdi adi(port, 0);
static uint8_t probeTlv[64];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  // the channels up to the wire's pair: the pair is the wire's, the ones below it are not offered (reserved)
  oep::describeCore(w, "swd-debug-probe", id, oep::platformUnitId(id, sizeof id), kChannelCount,
                    ((1ull << kChannelCount) - 1) & ~kChannels);
  return w.ok() ? w.length() : 0;
}

void setup() {
  // Every channel not reserved - the pair - free before the first answer and until a host attaches (oep-core §8: Hi-Z,
  // no pull - the RP2's pads come out of reset pulled down), the same as after a detach.
  oep::platformParkMask(kChannels);
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
  Serial.begin(115200);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.add(wire);
  endpoint.add(adi);
}

void loop() {
  endpoint.poll();
}
