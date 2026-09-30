// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A debugger for WCH CH32 chips on their 2-wire RVSWD link (CH32V2 / V3 / X0 / L1 and the like): the smallest probe a
// flash tool or a debugger can use (oep-spec docs/oep-if-debug.ja.md).
//
//   oep.wire.rvswd       scan the pair, attach (a connection), detach
//   oep.target.riscv-dm  the RISC-V debug module over that connection: DMI steps, halt / resume / step, reset, block
//                        read / write through the bus (what flashing needs), run a loader until it halts
//   oep.target.console   the target's console through the debug module (dmseq / DMDATA / SDI): no UART, no pin
//
// The pair is fixed here (kSwdio / kSwclk): wire them to the target's SWDIO / SWCLK, and GND. Firmware/OepProbe lets
// the host choose the pins instead. The probe knows nothing of the chip: the flash layout and the loader are the host's
// (ch32rv, ArduinoCore-CH32), and so is how the line should rest and how fast it may go (the attach's idle_clock and
// max_speed: a CH32L103 wants SWCLK low and at most 1 MHz right after a reset).
//
// From Python (oep-client-python):
//   from oep_client import link, target
//   hst = link.open_host("<port>"); hst.open(3000)
//   wire = target.Wire(hst)
//   conn, dmstatus = wire.attach(halt=True)          # add idle_clock="low", max_speed=1_000_000 for a CH32L103
//   dm = target.RiscvDm(hst, conn)
//   print(dm.read_block(0x08000000, 4).hex())        # the first 4 words of flash
//   dm.resume(); wire.detach(conn)
#include <OepCh32Dm.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepPlatform.h>
#include <OepRvswdPhy.h>
#include <OepTarget.h>

#if defined(ARDUINO_ARCH_RP2040)
static constexpr uint8_t kTransport = oep::Endpoint::kUsbCdc;   // USB CDC
static constexpr uint8_t kSwdio = 2, kSwclk = 3;                  // GP2 -> SWDIO, GP3 -> SWCLK
#else
#include <soc/usb_serial_jtag_reg.h>
// an ESP32 with dedicated GPIO (the P4; an S3, C3 or C6 builds too): its USB-Serial/JTAG is the Serial port
static constexpr uint8_t kTransport = oep::Endpoint::kUsbSerialJtag;
static constexpr uint8_t kSwdio = 2, kSwclk = 3;   // GPIO2 -> SWDIO, GPIO3 -> SWCLK
#endif

static uint8_t rxBuffer[1100];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8}, kTransport);

// The PHY drives the two wires (the RP2's SIO, the P4's dedicated GPIO), Ch32Dm speaks DMI over it, and one DebugPort is
// the wire's one connection that the three interfaces share.
static oep::RvswdPhy phy;
static oep::Ch32Dm dm(phy);
static oep::DebugPort port{dm, kSwdio, kSwclk};
static oep::WireRvswd wire(port, 1);
static oep::TargetRiscvDm riscvDm(port, 1);
static oep::DmConsole consoleDriver(dm, phy);
static oep::TargetConsoleStream console(port, consoleDriver, 1);
static uint8_t probeTlv[64];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[8];
  // the pair is the probe's own: reserved, so no plan takes it
  oep::describeCore(w, "rvswd-debug-probe", id, oep::platformUnitId(id, sizeof id), 64,
                    (1ull << kSwdio) | (1ull << kSwclk));
  return w.ok() ? w.length() : 0;
}

void setup() {
#if defined(ARDUINO_ARCH_RP2040)
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
#else
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §2.5)
#if defined(USB_SERIAL_JTAG_CHIP_RST_REG)   // the P4: opening the port must not reset the chip (an S3 has no such bit)
  REG_SET_BIT(USB_SERIAL_JTAG_CHIP_RST_REG, USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS);
#endif
  Serial.setTxTimeoutMs(0);
#endif
  Serial.begin(115200);
  phy.begin(kSwdio, kSwclk);   // the wires rest released (Hi-Z) until a host attaches
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setBootId(oep::platformRandom32());
  endpoint.add(wire);       // fn 1
  endpoint.add(riscvDm);    // fn 2
  endpoint.add(console);    // fn 3
}

void loop() {
  endpoint.poll();
  console.poll();   // collects the target's console words while a connection is up and the hart runs
}
