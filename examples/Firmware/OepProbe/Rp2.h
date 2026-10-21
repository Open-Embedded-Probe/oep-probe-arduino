// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// RP2040 / RP2350 (built for the Raspberry Pi Pico / Pico 2; profiles rp2040 / rp2350).
//
// Transport (the Cdc choice): USB CDC (Serial), a serial port: COBS frames (oep-transports §1). The USB device is the project's VID:PID
// 1209:4F45 (registry usb; PID-USE.md), serial = the unit id; its iProduct "OEP probe (RP2040)" /
// "(RP2350)" is a name for people.
//
// Interfaces (revision 1): fn 0 (the core); oep.wire.rvswd + oep.target.riscv-dm + oep.target.console (WCH CH32, 2 wires);
// oep.wire.swd + oep.target.arm-adi (ARM); oep.fixture.gpio / uart; oep.fixture.analog (GP26-28, 500 kS/s in all);
// oep.probe.link; oep.probe.plan and oep.probe.restart (the endpoint's own, listed last). Each is a choice (OepFeatures.h:
// a weak oepWithX() here adds it, a sketch's own replaces it; OepChoice.h): Rvswd (and Console in it), Swd, Gpio, Uart,
// Config, Analog, Link, I2c (oep.fixture.i2c-target), Swio (oep.wire.swio, CH32V00x; SwioConsole in it), and Cdc (the transport), all in by default.
// Every channel below may be SWDIO / SWCLK of either
// wire, the reset line of attach's reset TLV, a gpio or a UART pin (UART0: GP0/1, GP12/13, GP16/17, GP28/29); a live
// debug connection holds its pair, a plan holds its pins (oep-core §8.1). What a target needs of its line (idle_clock,
// max_speed) comes from the host (oep-if-debug §3). oep.probe.config keeps a jig's settings in flash (slots on any pair,
// a bind of the CDC port - the target's console on the same line as OEP -, labels, idle states).
//
// The bench's trace (off by default; never in a release): build with -DOEP_DEBUG_LOG=1 -DOEP_DEBUG_LOG_TX=<GP> (e.g.
// arduino-cli compile --profile rp2350 --build-property "compiler.cpp.extra_flags=-DOEP_DEBUG_LOG=1 -DOEP_DEBUG_LOG_TX=4")
// and a line per request and riscv-dm run's steps go out of Serial2 (UART1: TX on GP4, GP8, GP20 or GP24) at 921600
// 8N1. That pin is then the probe's own (reserved, never offered).
#pragma once
#include <USB.h>

#include <OepAnalog.h>
#include <OepBind.h>
#include <OepBootGuard.h>
#include <OepCh32Dm.h>
#include <OepChoice.h>
#include <OepConfig.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepP4I2cTarget.h>
#include <OepLog.h>
#include <OepPinTable.h>
#include <OepRvswdPhy.h>
#include <OepSwd.h>
#include <OepSwioPhy.h>
#include <OepTarget.h>

#include "OepFeatures.h"

#if defined(ARDUINO_ARCH_RP2350) || defined(PICO_RP2350)
static constexpr const char *kProduct = "OEP probe (RP2350)", *kModel = "rp2350";
#else
static constexpr const char *kProduct = "OEP probe (RP2040)", *kModel = "rp2040";
#endif

// The pins a Pico / Pico 2 brings out: GP0-GP22, GP26-GP28 (GP23-GP25 and GP29 are the board's own there). Other boards
// with the same chip run this too (a SparkFun Pro Micro RP2350: its GP19 is the PSRAM's chip select); their own parts on
// these pins (an LED, a PSRAM chip select) are for the host to leave alone. (A Pro Micro profile that also gave
// GP23-GP25 / GP29 was dropped: one build a chip.)
static constexpr uint64_t kBoardChannels = ((1ull << 23) - 1) | (0x7ull << 26);
#if OEP_DEBUG_LOG
#ifndef OEP_DEBUG_LOG_TX
#error "OEP_DEBUG_LOG: name the trace's TX pin with OEP_DEBUG_LOG_TX (Serial2 = UART1: GP4, GP8, GP20 or GP24)"
#endif
static constexpr uint64_t kChannels = kBoardChannels & ~(1ull << OEP_DEBUG_LOG_TX);   // the trace's pin is the probe's
#else
static constexpr uint64_t kChannels = kBoardChannels;
#endif
static constexpr uint64_t kReserved = ((1ull << 30) - 1) & ~kChannels;
static constexpr uint16_t kUnset = 0xfffe;   // no pair chosen yet

static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(txBuffer, sizeof txBuffer, {1024, 4096, 8});
static oep::PinTable pins(kChannels);
static uint8_t probeTlv[256];   // with the firmware text's note of the boot before (BootGuard::lastBoot)

// What a choice leaves to run later, each added only from its own oepAddX(): its loop work; the settings' fn at its place,
// what comes last
static oep::Hooks<void (*)(), 5> loopWork;
static oep::Hooks<void (*)(), 1> settingsFn;
static oep::Hooks<void (*)(), 1> lastly;

// What a choice leaves for another to find, each set only from its own oepAddX(); the code using both is the other's (the
// settings', the SWIO wire's), so neither names the other's code: the consoles (and the SWIO wire), SWD's port and the
// uart, for the settings; the riscv-dm target for the SWIO wire
static oep::TargetConsoleStream *consoleIn = nullptr, *swioConsoleIn = nullptr;
static oep::WireRvswd *swioWireIn = nullptr;
static oep::TargetRiscvDm *riscvDmIn = nullptr;   // the RVSWD wire's, the SWIO wire's connections too
static oep::SwdPort *swdIn = nullptr;
static oep::FixtureUart *uartIn = nullptr;

// The RVSWD wire's parts, the console's too (its choice is made in the wire's: no wire, no console)
struct RvswdParts {
  oep::RvswdPhy phy;
  oep::Ch32Dm dm{phy};
  oep::DebugPort port{dm, kUnset, kUnset};
  oep::WireRvswd wire{port, 0};
  oep::TargetRiscvDm riscvDm{port, 0};
  RvswdParts() {
    port.pin_choice = kChannels;
    port.pins = &pins;
    port.reset_allowed = kChannels;   // attach's reset TLV: the channel the host names (no default), nobody holding it
  }
};
static oep::Slot<RvswdParts> rvswd;

// The USB CDC port (Serial), the RP2's transport
void oepAddCdc() {
  static uint8_t rx[1100];   // the encoded candidate: cobsFrameMax(1024)
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
  Serial.begin(115200);
  endpoint.addTransport(Serial, rx, sizeof rx, oep::Endpoint::kUsbCdc, 0);
}

void oepAddRvswd() {
  RvswdParts &r = rvswd.make();
  endpoint.add(r.wire);
  riscvDmIn = &r.riscvDm;
  endpoint.add(r.riscvDm);
  oepWithConsole();
}

void oepAddConsole() {
  static oep::Slot<oep::DmConsole> driver;
  static oep::Slot<oep::TargetConsoleStream> console;
  endpoint.add(console.make(rvswd->port, driver.make(rvswd->dm, rvswd->phy), 0));
  consoleIn = &*console;
  loopWork.add([] { console->poll(); });
}

void oepAddSwd() {
  static oep::SwdPort swd{kUnset, kUnset};
  static oep::Slot<oep::WireSwd> wireSwd;
  static oep::Slot<oep::TargetArmAdi> adi;
  swd.pin_choice = kChannels;
  swd.pins = &pins;
  endpoint.add(wireSwd.make(swd, 0));
  endpoint.add(adi.make(swd, 0));
  swdIn = &swd;
}

void oepAddGpio() {
  static oep::Slot<oep::FixtureGpio> gpio;
  endpoint.add(gpio.make(pins, 0, 1));
}

void oepAddUart() {
  static oep::Slot<oep::FixtureUart> uart;
  endpoint.add(uart.make(pins, Serial1, 0, 2));
  uart->setRoleChannels(oep::platformUartRxMask(0), oep::platformUartTxMask(0));   // Serial1 = UART0: the pins it reaches
  uartIn = &*uart;
  loopWork.add([] { uart->poll(); });
}

// The at-boot slots' attach waits for the host to have configured the device (or BootGuard::kAttachGraceMs without one):
// a console from the saved settings polls its target over RVSWD from boot, interrupts off for each frame, and the USB
// interrupts it delayed and batched while the host enumerated the probe met the panic above.
static bool autoAttachReady() { return oep::BootGuard::attachReady(tud_mounted()); }

// oep.probe.config, keeping a jig's settings in flash: read first (its disable items' channels are never parked:
// probe.config §2, disable before anything else; applySaved gives them back if the settings are not applied), its fn
// after the uart's
void oepAddConfig() {
  static oep::Slot<oep::Binds> binds;
  static oep::Slot<oep::ProbeConfig> config;
  oep::ProbeConfig &c = config.make(endpoint, binds.make());
  if (oep::BootGuard::safe()) {   // after BootGuard::kSafeAfter fast crash-boots: the saved at-boot slots wait for the host
    c.skipBootAttach();
    OEP_LOGF("safe boot after %u fast crash-boots: at-boot slots not attached", unsigned(oep::BootGuard::crashes()));
  }
  c.setAttachGate(autoAttachReady);
  c.load();
  pins.setDisabled(c.savedDisabled());
  settingsFn.add([] {
    endpoint.setRawPorts(&*binds);
    endpoint.add(*config);   // the fns before it keep their numbers
    config->usePassthroughs();   // a serial port's raw stream carries them (probe.config §1.1)
    if (consoleIn) config->addPlace(rvswd->wire, *consoleIn);
    if (swdIn) config->addHold(swdIn->hold, oep::checkSwdHold, swdIn);   // oep.wire.swd's hold after a close (oep-if-debug §2): wire-hold
    if (uartIn) config->addUart(*uartIn);
    config->setPins(&pins);   // the idle item sets these pins' free state
    loopWork.add([] { config->poll(); });
  });
  // Last, once every interface is added: the saved settings name fns, and are kept only for the same interface list
  // (applied before the analog and the group were added, they never matched it: unreadable after every reboot,
  // 0.0.11-0.0.16). probe.config §2: disable and idle (outputs driven) before every other item; the at-boot slots' attach
  // comes on config.poll(), so a target powered through an output idle is up before it.
  lastly.add([] {
    if (swioConsoleIn) config->addPlace(*swioWireIn, *swioConsoleIn);   // a slot on the one wire: the second place
    config->applySaved();
  });
}

// the ADC on GP26-28 (a Pico's GP29 reads VSYS), channels in turn, copied by DMA
static constexpr uint64_t kAdc = 0x7ull << 26;
void oepAddAnalog() {
  static oep::Slot<oep::AnalogCapture> analog;
  analog.make(endpoint, kAdc, 0).setPins(&pins, 7);   // PinTable owners: gpio 1, uart 2, the analog 7 (its pads go analog)
  endpoint.add(*analog);
  loopWork.add([] { analog->poll(); });
}

// oep.fixture.i2c-target on arduino-pico's Wire slave (PinTable owner 3)
void oepAddI2c() {
  static oep::Slot<oep::P4I2cTarget> i2c;
  endpoint.add(i2c.make(pins));
}

// The one-wire link (CH32V00x) on the SIO (OepSwioPhy.cpp): its own wire fn, its connections served by the RVSWD wire's
// riscv-dm target (or one of its own without that wire), its console instance 1 (its choice made in the wire's)
struct SwioParts {
  oep::SwioPhy phy;
  oep::Ch32Dm dm{phy};
  oep::DebugPort port{dm, kUnset, 0xffff};   // one wire: swclk stays 0xffff
  oep::WireRvswd wire{port, 0, "oep.wire.swio"};
  SwioParts() {
    port.pin_choice = kChannels;
    port.pins = &pins;
    port.pin_owner = 0xf2;
    port.reset_allowed = kChannels;
  }
};
static oep::Slot<SwioParts> swio;

void oepAddSwio() {
  static oep::Slot<oep::TargetRiscvDm> riscvDm;
  SwioParts &w = swio.make();
  endpoint.add(w.wire);
  swioWireIn = &w.wire;
  if (riscvDmIn) {
    riscvDmIn->addPort(w.port);
  } else {
    riscvDmIn = &riscvDm.make(w.port, 0);
    endpoint.add(*riscvDmIn);
  }
  oepWithSwioConsole();
}

void oepAddSwioConsole() {
  static oep::Slot<oep::DmConsole> driver;
  static oep::Slot<oep::TargetConsoleStream> console;
  swioConsoleIn = &console.make(swio->port, driver.make(swio->dm, swio->phy), 1);   // the one-wire link's own console
  endpoint.add(*swioConsoleIn);
  loopWork.add([] { console->poll(); });
}

void oepAddLink() {
  static oep::Slot<oep::Link> link;
  endpoint.add(link.make(endpoint));   // oep.probe.link (oep-if-link)
}

// This build's choices (OepFeatures.h: a sketch's own replaces one)
__attribute__((weak)) void oepWithCdc() { oepAddCdc(); }
__attribute__((weak)) void oepWithRvswd() { oepAddRvswd(); }
__attribute__((weak)) void oepWithConsole() { oepAddConsole(); }
__attribute__((weak)) void oepWithSwd() { oepAddSwd(); }
__attribute__((weak)) void oepWithGpio() { oepAddGpio(); }
__attribute__((weak)) void oepWithUart() { oepAddUart(); }
__attribute__((weak)) void oepWithConfig() { oepAddConfig(); }
__attribute__((weak)) void oepWithAnalog() { oepAddAnalog(); }
__attribute__((weak)) void oepWithLink() { oepAddLink(); }
__attribute__((weak)) void oepWithI2c() { oepAddI2c(); }
__attribute__((weak)) void oepWithSwio() { oepAddSwio(); }
__attribute__((weak)) void oepWithSwioConsole() { oepAddSwioConsole(); }

// oep.probe.restart (oep-if-restart): oep::platformRestart - the USB device off the bus (tud_disconnect), then
// rp2040.reboot(), a watchdog reset, oep::kRestartDetachMs later. restart_max_ms (its describe): the chip is running
// setup() within about 0.1 s (boot ROM, flash, the settings read), then the host enumerates the device again and its
// OS gives the CDC port back - 0.3 s to about 1 s (Windows the slowest) - before it can confirm. 2000 ms is about twice the slow end (an estimate from the boot path,
// to be measured on the bench).
static constexpr uint32_t kRestartMaxMs = 2000;

// Never left halted (OepBootGuard.h): a panic - pico-sdk's panic() ends in _exit, the core's weak one a breakpoint loop - and a
// HardFault - crt0's weak isr_hardfault, a breakpoint - take the USB device off the bus and reset the chip. Seen on the
// bench (0.0.29-dev+3c0cd99, Pro Micro RP2350, SWD): TinyUSB 0.18's "Can't continue xfer on inactive ep" panic in the
// USB interrupt while the host enumerated the probe after a restart; the breakpoint escalated to a HardFault, whose own
// breakpoint stopped the core with the pull-up on, and the host failed the device descriptor request until a replug.
extern "C" void _exit(int) { oep::BootGuard::crashed(); }
extern "C" void isr_hardfault() { oep::BootGuard::crashed(); }

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];   // the flash's unique id: the probe says who it is on any transport
  oep::describeCore(w, kModel, id, oep::platformUnitId(id, sizeof id), 30,
                    oep::BootGuard::lastBoot());   // the firmware text: the version (and what ended the boot before)
  oep::describeChip(w);   // the MCU and its revision (a capture records what it was taken on)
  return w.ok() ? w.length() : 0;
}

void setup() {
  oep::BootGuard::begin();   // the watchdog, and the count of fast crash-boots
  USB.disconnect();
  USB.setManufacturer("Open Embedded Probe");
  USB.setVIDPID(oep::reg::kUsbProjectVid, oep::reg::kUsbProjectPid);   // the project's VID:PID (registry usb, PID-USE.md)
  USB.setProduct(kProduct);   // a name for people; no host identifies the probe by it
  static uint8_t serial[17];
  oep::platformUnitId(serial, sizeof serial);
  USB.setSerialNumber(reinterpret_cast<const char *>(serial));   // the unit id, lowercase (transports §3)
  USB.connect();
  oepWithCdc();
#if OEP_DEBUG_LOG
  Serial2.setTX(OEP_DEBUG_LOG_TX);
  Serial2.begin(921600);
  oep::setLog(&Serial2);
  OEP_LOGF("oep trace on (%s)", kProduct);
#endif
  oepWithConfig();   // first: the saved settings read before the channels are parked
  oep::platformParkMask(kChannels & ~pins.disabledMask());   // Hi-Z every channel (RP2 pads boot with a pull-down) until the host takes one
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setRestart(oep::platformRestart, kRestartMaxMs);
  // The fns in this order, whichever are in (a build's set numbers them; a host finds each by name)
  oepWithRvswd();   // and the console's choice
  oepWithSwd();
  oepWithGpio();
  oepWithUart();
  settingsFn.run();
  oepWithAnalog();   // after the settings: the fns before it keep their numbers
  oepWithLink();
  oepWithI2c();   // after the ones before: they keep their numbers (the saved settings name fns)
  oepWithSwio();   // and its console's choice
  lastly.run();
}

void loop() {
  oep::BootGuard::poll();
  endpoint.poll();
  loopWork.run();
}
