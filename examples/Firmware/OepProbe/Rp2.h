// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// RP2040 / RP2350 (built for the Raspberry Pi Pico / Pico 2; profiles rp2040 / rp2350).
//
// Transports, each a choice: USB CDC (Cdc; Serial, transport 0), a serial port: COBS frames (oep-transports §1); USB
// vendor bulk (Bulk) and HID (Hid), length-prefixed frames (OepRp2Usb.h: this library's class drivers on arduino-pico's
// stack); a UART bridge (Bridge, off by default: Serial2 on GP8 / GP9). The USB device is the project's VID:PID 1209:4F45 (registry usb; PID-USE.md), serial = the unit id; its
// iProduct "OEP probe (RP2040)" / "(RP2350)" is a name for people.
//
// Interfaces (revision 1): fn 0 (the core); oep.wire.rvswd + oep.target.riscv-dm + oep.target.console (WCH CH32, 2 wires);
// oep.wire.swd + oep.target.arm-adi (ARM); oep.fixture.gpio / uart; oep.fixture.analog (GP26-28, 500 kS/s in all);
// oep.probe.link; oep.probe.plan and oep.probe.restart (the endpoint's own, listed last). Each is a choice (OepFeatures.h:
// a weak oepWithX() here adds it, a sketch's own replaces it; OepChoice.h): Rvswd (and Console in it), Swd, Gpio, Uart,
// Config, Analog, Link, I2c (oep.fixture.i2c-target), Logic (oep.fixture.logic), Group (oep.fixture.capture-group:
// the logic with the analog), Swio (oep.wire.swio, CH32V00x; SwioConsole in it), and Cdc (the transport), all in by default.
// Every channel below may be SWDIO / SWCLK of either
// wire, the reset line of attach's reset TLV, a gpio or a UART pin (UART0: GP0/1, GP12/13, GP16/17, GP28/29); a live
// debug connection holds its pair, a plan holds its pins (oep-core §8.1). What a target needs of its line (idle_clock,
// max_speed) comes from the host (oep-if-debug §3). the settings (core §13) keeps a jig's settings in flash (slots on any pair,
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
#include <OepCaptureGroup.h>
#include <OepCh32Dm.h>
#include <OepChoice.h>
#include <OepSettings.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepP4I2cTarget.h>
#include <OepLog.h>
#include <OepPinTable.h>
#include <OepRp2Logic.h>
#include <OepRp2Usb.h>
#include <OepRvswdPhy.h>
#include <OepCounter.h>
#include <OepI2cController.h>
#include <OepPwm.h>
#include <OepRp2SpiTarget.h>
#include <OepSpiController.h>
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
static oep::Hooks<void (*)(), 7> loopWork;
static oep::Hooks<void (*)(), 3> afterCdc;   // the other transports, after the CDC port (transport 0: the binds' serial port 0)
static oep::Hooks<void (*)(), 1> settingsFn;
static oep::Hooks<void (*)(), 1> lastly;

// What a choice leaves for another to find, each set only from its own oepAddX(); the code using both is the other's (the
// settings', the SWIO wire's), so neither names the other's code: the consoles (and the SWIO wire), SWD's port and the
// uart, for the settings; the riscv-dm target for the SWIO wire
static oep::TargetConsoleStream *consoleIn = nullptr, *swioConsoleIn = nullptr;
// oep.target.console: the probe's one (common §2), each wire's console a place in it
static oep::TargetConsole *consoles = nullptr;
static void addConsolePlace(oep::TargetConsoleStream &place) {
  if (consoles) { consoles->add(place); return; }
  static oep::Slot<oep::TargetConsole> one;
  consoles = &one.make(place);
  endpoint.add(*consoles);
}
static oep::WireRvswd *swioWireIn = nullptr;
static oep::TargetRiscvDm *riscvDmIn = nullptr;   // the RVSWD wire's, the SWIO wire's connections too
static oep::Rp2LogicCapture *logicIn = nullptr;   // the logic and the analog, for the group
static oep::AnalogCapture *analogIn = nullptr;
static oep::SwdPort *swdIn = nullptr;
static oep::WireSwd *swdWireIn = nullptr;   // its interface: the owner of its hold item (core §13.1)
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

// The USB vendor bulk and HID interfaces (OepRp2Usb.h), in the descriptor before USB.connect(), their transports after
// the CDC port's
void oepAddBulk() {
  static oep::rp2usb::VendorStream vendor;
  vendor.begin();
  afterCdc.add([] {
    static uint8_t rx[1026];   // one frame of max_frame
    endpoint.addTransport(vendor, rx, sizeof rx, oep::Endpoint::kVendorBulk, vendor.interfaceNumber(), true);
  });
}

void oepAddHid() {
  static oep::rp2usb::HidStream hid;
  hid.begin();
  afterCdc.add([] {
    static uint8_t rx[1026];
    endpoint.addTransport(hid, rx, sizeof rx, oep::Endpoint::kHid, hid.interfaceNumber(), true);
  });
}

// A UART bridge (transports §1: a serial transport, COBS): Serial2 = UART1 on its default pins, GP8 (TX) / GP9 (RX),
// 115200 - those two then the probe's own (never offered, never parked)
static uint64_t ownPins = 0;
void oepAddBridge() {
  static uint8_t rx[1100];   // the encoded candidate: cobsFrameMax(1024)
  ownPins |= (1ull << PIN_SERIAL2_TX) | (1ull << PIN_SERIAL2_RX);
  pins.forbid(ownPins);
  Serial2.setTX(PIN_SERIAL2_TX);
  Serial2.setRX(PIN_SERIAL2_RX);
  Serial2.begin(115200);
  afterCdc.add([] { endpoint.addTransport(Serial2, rx, sizeof rx, oep::Endpoint::kUartBridge); });
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
  consoleIn = &console.make(rvswd->port, driver.make(rvswd->dm, rvswd->phy), 0);
  addConsolePlace(*consoleIn);
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
  swdWireIn = &*wireSwd;
}

void oepAddGpio() {
  static oep::Slot<oep::FixtureGpio> gpio;
  endpoint.add(gpio.make(pins, 0, oep::owners::kGpio));
}

void oepAddUart() {
  static oep::Slot<oep::FixtureUart> uart;
  endpoint.add(uart.make(pins, Serial1, 0, oep::owners::kUart));
  uart->setRoleChannels(oep::platformUartRxMask(0), oep::platformUartTxMask(0));   // Serial1 = UART0: the pins it reaches
  uartIn = &*uart;
  loopWork.add([] { uart->poll(); });
}

// The at-boot slots' attach waits for the host to have configured the device (or BootGuard::kAttachGraceMs without one):
// a console from the saved settings polls its target over RVSWD from boot, interrupts off for each frame, and the USB
// interrupts it delayed and batched while the host enumerated the probe met the panic above.
static bool autoAttachReady() { return oep::BootGuard::attachReady(tud_mounted()); }

// the settings (core §13), keeping a jig's settings in flash: read first (its disable items' channels are never parked:
// core §13.7, disable before anything else; applySaved gives them back if the settings are not applied), its fn
// after the uart's
void oepAddSettings() {
  static oep::Slot<oep::Binds> binds;
  static oep::Slot<oep::Settings> config;
  oep::Settings &c = config.make(endpoint, binds.make());
  if (oep::BootGuard::safe()) {   // after BootGuard::kSafeAfter fast crash-boots: the saved at-boot slots wait for the host
    c.skipBootAttach();
    OEP_LOGF("safe boot after %u fast crash-boots: at-boot slots not attached", unsigned(oep::BootGuard::crashes()));
  }
  c.setAttachGate(autoAttachReady);
  c.load();
  pins.setDisabled(c.savedDisabled());
  settingsFn.add([] {
    endpoint.setRawPorts(&*binds);
    config->usePassthroughs();   // a serial port's raw stream carries them (fixture §2.1)
    if (consoleIn) config->addPlace(rvswd->wire, *consoleIn);
    if (swdIn) config->addHold(*swdWireIn, swdIn->hold, oep::checkSwdHold, swdIn);   // oep.wire.swd's hold after a close (oep-if-debug §2): hold.{}
    if (uartIn) config->addUart(*uartIn);
    config->setPins(&pins);   // the idle item sets these pins' free state
    loopWork.add([] { config->poll(); });
  });
  // Last, once every interface is added: the saved settings name fns, and are kept only for the same interface list
  // (applied before the analog and the group were added, they never matched it: unreadable after every reboot,
  // 0.0.11-0.0.16). core §13.7: disable and idle (outputs driven) before every other item; the at-boot slots' attach
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
  analog.make(endpoint, kAdc, 0).setPins(&pins, oep::owners::kAnalog);   // its pads go analog
  endpoint.add(*analog);
  analogIn = &*analog;
  loopWork.add([] { analog->poll(); });
}

// oep.fixture.i2c-target on arduino-pico's Wire slave (owners::kI2cTarget)
void oepAddI2c() {
  static oep::Slot<oep::P4I2cTarget> i2c;
  endpoint.add(i2c.make(pins));
}

// The one-wire link (CH32V00x) on the SIO (OepSwioPhy.cpp): its own wire fn, its connections served by the RVSWD wire's
// riscv-dm target (or one of its own without that wire), its console a place of the one oep.target.console (its choice made in the wire's)
struct SwioParts {
  oep::SwioPhy phy;
  oep::Ch32Dm dm{phy};
  oep::DebugPort port{dm, kUnset, 0xffff};   // one wire: swclk stays 0xffff
  oep::WireRvswd wire{port, 0, "oep.wire.swio"};
  SwioParts() {
    port.pin_choice = kChannels;
    port.pins = &pins;
    port.pin_owner = oep::owners::kSecondWire;
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
  swioConsoleIn = &console.make(swio->port, driver.make(swio->dm, swio->phy), 0);   // the one-wire link's place
  addConsolePlace(*swioConsoleIn);
  loopWork.add([] { console->poll(); });
}

// oep.fixture.logic, one-shot, paced by a PIO state machine (OepRp2Logic.h): up to 8 lines on any channels
void oepAddLogic() {
  static oep::Slot<oep::Rp2LogicCapture> logic;
  logicIn = &logic.make(endpoint, pins, 0);
  endpoint.add(*logicIn);
  loopWork.add([] { logic->poll(); });
}

// oep.fixture.capture-group: the logic (its trigger the group's) with the analog (the tracks of those in)
void oepAddGroup() {
  static oep::Slot<oep::CaptureGroup> group;
  endpoint.add(group.make(endpoint, 0));
  if (logicIn) group->addTrack(*logicIn, *logicIn);
  if (analogIn) group->addTrack(*analogIn, *analogIn);
  loopWork.add([] { group->poll(); });
}

void oepAddLink() {
  static oep::Slot<oep::Link> link;
  endpoint.add(link.make(endpoint));   // oep.probe.link (oep-if-link)
}

// oep.fixture.i2c-controller / spi-controller: the lines on GPIOs (owners::kI2cController / kSpiController)
void oepAddI2cController() {
  static oep::Slot<oep::I2cController> i2c;
  endpoint.add(i2c.make(pins));
}

void oepAddSpiController() {
  static oep::Slot<oep::SpiController> spi;
  endpoint.add(spi.make(pins));
}

// oep.fixture.spi-target on a PIO state machine (Rp2SpiTarget; owners::kSpiTarget) - built, not yet run on hardware
void oepAddSpi() {
  static oep::Slot<oep::Rp2SpiTarget> spi;
  endpoint.add(spi.make(pins));
}

// oep.fixture.pwm (owners::kPwm) / counter (it only listens: no owner)
void oepAddPwm() {
  static oep::Slot<oep::FixturePwm> pwm;
  endpoint.add(pwm.make(pins));
}

void oepAddCounter() {
  static oep::Slot<oep::FixtureCounter> counter;
  endpoint.add(counter.make(pins));
}

// The sketch's own parts (OepFeatures.h): none here; a sketch's oepUserSetup / oepUserLoop replace these
__attribute__((weak)) void oepUserSetup(oep::Endpoint &, oep::PinTable &) {}
__attribute__((weak)) void oepUserLoop() {}

// This build's choices (OepFeatures.h: a sketch's own replaces one)
__attribute__((weak)) void oepWithCdc() { oepAddCdc(); }
__attribute__((weak)) void oepWithBulk() { oepAddBulk(); }
__attribute__((weak)) void oepWithHid() { oepAddHid(); }
__attribute__((weak)) void oepWithBridge() {}
__attribute__((weak)) void oepWithRvswd() { oepAddRvswd(); }
__attribute__((weak)) void oepWithConsole() { oepAddConsole(); }
__attribute__((weak)) void oepWithSwd() { oepAddSwd(); }
__attribute__((weak)) void oepWithGpio() { oepAddGpio(); }
__attribute__((weak)) void oepWithUart() { oepAddUart(); }
__attribute__((weak)) void oepWithSettings() { oepAddSettings(); }
__attribute__((weak)) void oepWithAnalog() { oepAddAnalog(); }
__attribute__((weak)) void oepWithLink() { oepAddLink(); }
__attribute__((weak)) void oepWithI2c() { oepAddI2c(); }
__attribute__((weak)) void oepWithSwio() { oepAddSwio(); }
__attribute__((weak)) void oepWithLogic() { oepAddLogic(); }
__attribute__((weak)) void oepWithGroup() { oepAddGroup(); }
__attribute__((weak)) void oepWithSwioConsole() { oepAddSwioConsole(); }
__attribute__((weak)) void oepWithI2cController() { oepAddI2cController(); }
__attribute__((weak)) void oepWithSpiController() { oepAddSpiController(); }
__attribute__((weak)) void oepWithSpi() { oepAddSpi(); }
__attribute__((weak)) void oepWithPwm() { oepAddPwm(); }
__attribute__((weak)) void oepWithCounter() { oepAddCounter(); }

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
  oepWithBulk();   // their interfaces in the descriptor USB.connect() builds
  oepWithHid();
  oepWithBridge();
  USB.connect();
  oepWithCdc();
  afterCdc.run();
#if OEP_DEBUG_LOG
  Serial2.setTX(OEP_DEBUG_LOG_TX);
  Serial2.begin(921600);
  oep::setLog(&Serial2);
  OEP_LOGF("oep trace on (%s)", kProduct);
#endif
  oepWithSettings();   // first: the saved settings read before the channels are parked
  oep::platformParkMask(kChannels & ~pins.disabledMask() & ~ownPins);   // Hi-Z every channel (RP2 pads boot with a pull-down) until the host takes one
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
  oepWithLogic();
  oepWithGroup();
  oepWithI2cController();
  oepWithSpiController();
  oepWithPwm();
  oepWithCounter();
  oepWithSpi();
  oepUserSetup(endpoint, pins);   // the sketch's own, after the build's (and before the saved settings)
  lastly.run();
}

void loop() {
  oep::BootGuard::poll();
  endpoint.poll();
  loopWork.run();
  oepUserLoop();
}
