// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// WCH CH32 on ArduinoCore-CH32RV (profiles ch32x035, ch32v307, ch32h417; the core's 0.7.1, LTO by default).
//
// Transports, each a choice (below): the USB vendor bulk interface (class 0xFF, 'O', 'E': transports §3) and the USB HID
// one at full speed, length-prefixed frames; a USB CDC port and a UART bridge (COBS) too if chosen. The USB device is the project's VID:PID 1209:4F45 (registry usb; PID-USE.md), serial = the unit id (the chip's 64-bit
// unique id in hex); iProduct "OEP probe (CH32X035)" is a name for people. The core runs the USB stack from loop() and
// delay(): no other context, so the classes are read and written from loop() as Streams.
//
// Interfaces (revision 1), as the RP2's (Rp2.h): fn 0 (the core); oep.wire.rvswd + oep.target.riscv-dm +
// oep.target.console (WCH CH32, 2 wires); oep.wire.swd + oep.target.arm-adi (ARM); oep.fixture.gpio / uart;
// oep.fixture.i2c-target / spi-target (the core's Wire and SPI slaves) / i2c-controller / spi-controller (GPIOs);
// oep.fixture.analog (the core's CH32RVAdcStream: ADC1, the pins in turn); oep.fixture.logic (CH32RVLogicCapture: one port);
// the settings (core §13) (no store yet: nothing saved, setStorage(false)); oep.probe.link; oep.probe.plan and
// oep.probe.restart (the endpoint's own). Every channel (platformPad: port x 24 + bit) may be SWDIO / SWCLK of either
// wire, the reset line of attach's reset TLV, a gpio or a UART pin (any enabled USART's route; another pair is
// refused at the UART's begin).
//
// A CH32 but the H41x is a small-RAM family (oep::kSmallRam, Oep.h): the endpoint's tables cut, a 1 KiB console ring. Everything is
// in by default but on the CH32X035 (62 KiB flash, 20 KiB RAM; kRoomy below): the V307 97.7 KB / 24.8 KB, the H417
// 106 KB (0.2.2).
//
// Each interface and transport is a feature (OepFirmware.h, OepChoice.h): oep::kX, begin<F>() adding it with oepAddX();
// oep::kAvailable / kDefaults below (CDC and the UART bridge not in by default).
// Each X's objects live in its oepAddX() and what X leaves to the others (its loop work, its part of the settings)
// goes through the hooks below, so a left-out X links none of its code. The CH32 has: Rvswd (oep.wire.rvswd +
// oep.target.riscv-dm), Console (oep.target.console on the RVSWD wire), Swio (oep.wire.swio, CH32V00x; SwioConsole on
// it), Swd (oep.wire.swd + oep.target.arm-adi), Gpio,
// Uart, Config (the settings (core §13)), Link (oep.probe.link); Bulk (the USB vendor bulk transport), Hid (the USB HID transport: usage page 0xFF4F, usage 0x45,
// transports §1, §3), Cdc (a USB CDC port, COBS: a serial transport), Bridge (a UART bridge on Serial1's default pins,
// 115200: a serial transport).
#pragma once
#include <CH32RV.h>
#include <CH32RVUsbDevice.h>

#include <OepBind.h>
#include <OepCh32Dm.h>
#include <OepChoice.h>
#include <OepSettings.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepP4I2cTarget.h>
#include <OepPinTable.h>
#include <OepPlatform.h>
#include <OepRvswdPhy.h>
#include <OepAnalog.h>   // its CH32 build not on the X035 (its flash)
#include <OepCh32Logic.h>   // empty on the X035 (its flash)
#include <OepCh32DirectBulkStream.h>
#include <OepH41xLogic.h>   // the H41x's UHSIF only
#include <OepCh32SpiTarget.h>   // empty on the X035 (its flash)
#include <OepCounter.h>
#include <OepDac.h>
#include <OepI2cController.h>
#include <OepSpiController.h>
#include <OepSwd.h>
#include <OepSwioPhy.h>
#include <OepTarget.h>

// The chip the build is for (its board: the profiles ch32x035 / ch32v307 / ch32h417), a name for people and describe's
// model
#if defined(ARDUINO_CH32X035)
static constexpr const char *kProduct = "OEP probe (CH32X035)", *kModel = "ch32x035";
#elif defined(ARDUINO_CH32V307)
static constexpr const char *kProduct = "OEP probe (CH32V307)", *kModel = "ch32v307";
#elif defined(ARDUINO_CH32H417)
static constexpr const char *kProduct = "OEP probe (CH32H417)", *kModel = "ch32h417";
#else
static constexpr const char *kProduct = "OEP probe (CH32)", *kModel = "ch32";
#endif
// the names in use: these, or the sketch's FirmwareConfig
static const char *modelName = kModel, *productName = kProduct;

// The vendor class as the endpoint's Stream (it is a Print with available / read / peek, not a Stream)
class VendorStream final : public Stream {
 public:
  explicit VendorStream(CH32RVUsbDeviceVendor &vendor) : vendor_(vendor) {}
  int available() override { return vendor_.available(); }
  int read() override { return vendor_.read(); }
  int peek() override { return vendor_.peek(); }
  size_t write(uint8_t c) override { return vendor_.write(c); }
  size_t write(const uint8_t *data, size_t length) override { return vendor_.write(data, length); }
  void flush() override { vendor_.flush(); }

 private:
  CH32RVUsbDeviceVendor &vendor_;
};

// The HID transport (transports §1): a report carries count(u16) and that many bytes of the length-prefixed stream, the
// rest padding; the reports are the core's vendor HID ones (report ID 6, 63 bytes at full speed: 61 of the stream).
class HidStream final : public Stream {
 public:
  static constexpr size_t kReport = 63, kPayload = kReport - 2;
  explicit HidStream(CH32RVUsbDeviceHidVendor &h) : h_(h) {
    h_.onOutputReport([this](const CH32RVUsbDeviceHidReport &r) { take(r.data, r.length); });
  }
  int available() override { return static_cast<int>((head_ + sizeof rx_ - tail_) % sizeof rx_); }
  int read() override {
    if (head_ == tail_) return -1;
    const uint8_t c = rx_[tail_];
    tail_ = (tail_ + 1) % sizeof rx_;
    return c;
  }
  int peek() override { return head_ == tail_ ? -1 : rx_[tail_]; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    for (size_t i = 0; i < n; ++i) {
      tx_[2 + fill_++] = b[i];
      if (fill_ == kPayload) flush();
    }
    return n;
  }
  void flush() override {
    if (!fill_) return;
    tx_[0] = static_cast<uint8_t>(fill_);
    tx_[1] = 0;
    memset(tx_ + 2 + fill_, 0, kPayload - fill_);   // the padding 0 (transports §1 rule 3)
    // a report still in flight: wait for it (dropping one breaks the stream) - but once one could not go (nobody reads
    // HID), later ones do not wait again until one goes
    stuck_ = !h_.sendInput(tx_, kReport, stuck_ ? 0 : 200);
    fill_ = 0;
  }

 private:
  void take(const uint8_t *d, size_t len) {   // one output report: its count bytes into the ring (transports §1)
    if (!d || len < 2) return;
    const size_t n = d[0] | (d[1] << 8);
    if (!guard_.accept(n > len - 2, millis())) return;   // a count over what it carries, and up to the pause after
    for (size_t i = 0; i < n; ++i) {
      const size_t next = (head_ + 1) % sizeof rx_;
      if (next == tail_) return;
      rx_[head_] = d[2 + i];
      head_ = next;
    }
  }
  CH32RVUsbDeviceHidVendor &h_;
  oep::HidCountGuard guard_;
  uint8_t rx_[640];
  volatile size_t head_ = 0, tail_ = 0;
  uint8_t tx_[kReport];
  size_t fill_ = 0;
  bool stuck_ = false;
};

static uint8_t txBuffer[512];
#if defined(OEP_LOGIC_UHSIF)
// The H41x: frames of a UHSIF buffer (16 KiB) and its 64 bytes on the vendor bulk port (its rx holds one whole), so a
// logic stream goes in pushes of that size (OepH41xLogic.h); the other ports' own max_frame stays theirs
static constexpr uint16_t kMaxFrame = 16384 + 64;
static oep::Endpoint endpoint(txBuffer, sizeof txBuffer, {kMaxFrame, 4 * kMaxFrame, 8});
#else
static constexpr uint16_t kMaxFrame = 512;
static oep::Endpoint endpoint(txBuffer, sizeof txBuffer, {512, 1024, 2});
#endif
static uint8_t probeTlv[160];
static char serial[17];

// The channels (platformPad: port x 24 + bit): every port's pads (CH32RV_GPIO_PORT_WIDTH a port, the last port the
// core's pin map has) up to oep::kChannelCount (OepChannelMask.h: the X035 stops at 64 - PA0-PA23, PB0-PB23, PC0-PC15 -
// its USB pads PC16 / PC17 and SWD PC18 / PC19 past it; a V307 PA0-PE15, an H41x PA0-PF15), less the chip's own: a V307's
// USBFS (PA11 / PA12, the core's controller there) and SWD (PA13 / PA14); an H417's SWD (PB8 / PB9; its USBSS has pads
// of its own). Which of these a package brings out is the board's; a pad it does not is left alone by the host.
static constexpr int kPorts = (NUM_DIGITAL_PINS - 1) / 32 - 1;   // pin = ((port + 2) << 5) | bit
static constexpr int kPortWidth = CH32RV_GPIO_PORT_WIDTH < 24 ? CH32RV_GPIO_PORT_WIDTH : 24;
static constexpr oep::ChannelMask padsOf(int ports) {
  oep::ChannelMask m = {};
  for (int p = 0; p < ports; ++p)
    for (int b = 0; b < kPortWidth && p * 24 + b < oep::kChannelCount; ++b) m = m | oep::channelBit(static_cast<uint16_t>(p * 24 + b));
  return m;
}
#if defined(ARDUINO_CH32V307)
static constexpr oep::ChannelMask kChipOwn = 0xfull << 11;   // PA11-PA14
#elif defined(ARDUINO_CH32H417)
static constexpr oep::ChannelMask kChipOwn = 0x3ull << (24 + 8);   // PB8 / PB9
#else
static constexpr oep::ChannelMask kChipOwn = 0;
#endif
static constexpr oep::ChannelMask kChannels = padsOf(kPorts) & ~kChipOwn;
#if defined(ARDUINO_CH32H417)
static_assert(oep::kChannelCount >= 5 * 24 + 16, "the H41x's channels reach PF15");
#elif defined(ARDUINO_CH32V307)
static_assert(oep::kChannelCount >= 4 * 24 + 16, "the V307's channels reach PE15");
#endif
static constexpr uint16_t kUnset = 0xfffe;   // no pair chosen yet
static oep::PinTable pins(kChannels);

// What a choice leaves to run later, each added only from its own oepAddX(): what waits for the USB device, its loop work
static oep::Hooks<void (*)(), 1> afterUsb;
static oep::Hooks<void (*)(), 8> loopWork;

// What a choice leaves for another to find, each set only from its own oepAddX(); the code using both is the other's (the
// settings', the SWIO wire's), so neither names the other's code: a serial transport (the settings' pass-throughs), the
// consoles (and the SWIO wire), SWD's port and the uart, for the settings; the riscv-dm target for the SWIO wire
static bool serialPorts = false;
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
static oep::SwdPort *swdIn = nullptr;
static oep::WireSwd *swdWireIn = nullptr;   // its interface: the owner of its hold item (core §13.1)
static oep::FixtureUart *uartIn = nullptr;

void oepAddBulk() {
  static oep::Slot<CH32RVUsbDeviceVendor> usbVendor;
  usbVendor.make(USBDevice, 64).setInterfaceProtocol(oep::reg::kUsbVendorBulkSubclass, oep::reg::kUsbVendorBulkProtocol);
#if defined(OEP_LOGIC_UHSIF)
  // the zero-copy port (borrowed IN transfers, the USBSS DMA reading the buffers itself): results and a capture's runs
  // (OepCh32DirectBulkStream.h); set up before the device starts
  static oep::Slot<oep::Ch32DirectBulkStream> stream;
  static uint8_t rx[kMaxFrame + 2];
  const bool direct = stream.make(*usbVendor).begin();
  // flushed after each poll that wrote: a result goes out on flush (one buffer, one transfer)
  endpoint.addTransport(*stream, rx, sizeof rx, oep::Endpoint::kVendorBulk, 0, true);   // the first interface
  if (direct) endpoint.setDirect(&*stream);
#else
  static oep::Slot<VendorStream> stream;
  static uint8_t rx[520];
  endpoint.addTransport(stream.make(*usbVendor), rx, sizeof rx, oep::Endpoint::kVendorBulk, 0);   // the first interface
#endif
}

void oepAddHid() {
  static oep::Slot<CH32RVUsbDeviceHidVendor> usbHid;
  static oep::Slot<HidStream> stream;
  static uint8_t rx[520];
  stream.make(usbHid.make(USBDevice, HidStream::kReport));
  usbHid->setUsage(oep::reg::kUsbHidUsagePage, oep::reg::kUsbHidUsage);   // 0xFF4F / 0x45 (registry usb)
  endpoint.addTransport(*stream, rx, sizeof rx, oep::Endpoint::kHid, 0xff, true);   // the device's one HID interface (the core numbers it)
}

// The ports are the firmware's own, not the core's SerialUSB / Serial1 / Serial: naming one of those links it (and its
// registration with the device) whether or not it is chosen
static CH32RVUsbDeviceCdcSerial &cdc() {
  static CH32RVUsbDeviceCdcSerial port(USBDevice);
  return port;
}

void oepAddCdc() {
  cdc().begin(115200);   // before USBDevice.begin: its interface is laid out with the vendor one
  afterUsb.add([] {   // its interface number, once laid out
    static uint8_t rx[oep::cobsFrameMax(512) + 8];   // the encoded candidate (transports §1)
    endpoint.addTransport(cdc(), rx, sizeof rx, oep::Endpoint::kUsbCdc, cdc().interfaceNumber(), true);
  });
  serialPorts = true;
}

void oepAddBridge() {
  static oep::Slot<oep::OepUart> port;
  static uint8_t rx[oep::cobsFrameMax(512) + 8];
  port.make(1).begin(115200);   // USART2 (Serial1's), its default route (transports §1: a UART bridge at its boot speed)
  endpoint.addTransport(*port, rx, sizeof rx, oep::Endpoint::kUartBridge);
  serialPorts = true;
}

// The RVSWD wire's parts, the console's too (its choice is made in the wire's: no wire, no console)
struct RvswdParts {
  oep::RvswdPhy phy;
  oep::Ch32Dm dm{phy};
  oep::DebugPort port{dm, kUnset, kUnset};
  oep::WireRvswd wire{port, 0};
  RvswdParts() {
    port.pin_choice = kChannels;
    port.pins = &pins;
    port.reset_allowed = kChannels;
  }
};
static oep::Slot<RvswdParts> rvswd;

static oep::Slot<oep::TargetRiscvDm> riscvDm;

void oepAddRvswd() {
  RvswdParts &r = rvswd.make();
  endpoint.add(r.wire);
  riscvDmIn = &riscvDm.make(r.port, 0);
  endpoint.add(*riscvDmIn);
}

void oepAddConsole() {
  static oep::Slot<oep::DmConsole> driver;
  static oep::Slot<oep::TargetConsoleStream> console;
  consoleIn = &console.make(rvswd->port, driver.make(rvswd->dm, rvswd->phy), 0);
  addConsolePlace(*consoleIn);
  loopWork.add([] { console->poll(); });
}

// The one-wire link (CH32V00x): its own wire fn, its connections served by the RVSWD wire's riscv-dm target (or one of
// its own without that wire), its console a place of the one oep.target.console (its choice made in the wire's)
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
  SwioParts &w = swio.make();
  endpoint.add(w.wire);
  swioWireIn = &w.wire;
  if (riscvDmIn) {
    riscvDmIn->addPort(w.port);
  } else {
    riscvDmIn = &riscvDm.make(w.port, 0);
    endpoint.add(*riscvDmIn);
  }
}

void oepAddSwioConsole() {
  static oep::Slot<oep::DmConsole> driver;
  static oep::Slot<oep::TargetConsoleStream> console;
  swioConsoleIn = &console.make(swio->port, driver.make(swio->dm, swio->phy), 0);   // the one-wire link's place
  addConsolePlace(*swioConsoleIn);
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
  static oep::Slot<oep::OepUart> port;
  static oep::Slot<oep::FixtureUart> uart;
  endpoint.add(uart.make(pins, port.make(0), 0, oep::owners::kUart));   // USART1 (Serial's)
  uart->setRoleChannels(kChannels, kChannels);   // any enabled USART's route (its begin refuses another pair)
  uartIn = &*uart;
  loopWork.add([] { uart->poll(); });
}

// oep.fixture.i2c-target on the core's Wire slave (owners::kI2cTarget)
void oepAddI2c() {
  static oep::Slot<oep::P4I2cTarget> i2c;
  endpoint.add(i2c.make(pins));
}

// oep.fixture.analog on the core's CH32RVAdcStream: ADC1's inputs among the channels (owners::kAnalog). Its rates
// measured now, before a host: the first ADC input's pad goes analog for it, then floats again.
void oepAddAnalog() {
#if defined(OEP_ANALOG_CH32)
  oep::ChannelMask adc = {};
  for (uint16_t c = 0; c < oep::kChannelCount; ++c)
    if (oep::hasChannel(kChannels, c) && digitalPinHasADC(oep::platformPad(c))) adc = adc | oep::channelBit(c);
  static oep::Slot<oep::AnalogCapture> analog;
  analog.make(endpoint, adc, 0).setPins(&pins, oep::owners::kAnalog);
  analog->measureRates();
  endpoint.add(*analog);
  loopWork.add([] { analog->poll(); });
#endif
}

// oep.fixture.logic on the core's CH32RVLogicCapture: one port's pins 0-15, up to 8 channels (it only listens)
void oepAddLogic() {
#if defined(OEP_LOGIC_CH32)
  static oep::Slot<oep::Ch32LogicCapture> logic;
  endpoint.add(logic.make(endpoint, pins, 0));
  loopWork.add([] { logic->poll(); });
#endif
#if defined(OEP_LOGIC_UHSIF)
  // the H41x's UHSIF: 16 data lines streamed at up to 200 MS/s over the vendor port's zero-copy path (instance 1)
  static oep::Slot<oep::H41xLogicCapture> fast;
  endpoint.add(fast.make(endpoint, pins, 1));
  loopWork.add([] { fast->poll(); });
#endif
}

// oep.fixture.counter (it only listens: no owner)
void oepAddCounter() {
#if !defined(ARDUINO_CH32X035)
  static oep::Slot<oep::FixtureCounter> counter;
  endpoint.add(counter.make(pins));
#endif
}

// oep.fixture.dac on the DAC pads the core names (CH32RV_DAC1_PIN / 2: PA4 / PA5 on a V307 / H417; owners::kDac)
void oepAddDac() {
#if defined(CH32RV_DAC1_PIN) && !defined(ARDUINO_CH32X035)
  oep::ChannelMask dac = {};
  for (uint16_t c = 0; c < oep::kChannelCount; ++c) {
    const int pad = oep::platformPad(c);
    if (pad == CH32RV_DAC1_PIN) dac = dac | oep::channelBit(c);
#if defined(CH32RV_DAC2_PIN)
    if (pad == CH32RV_DAC2_PIN) dac = dac | oep::channelBit(c);
#endif
  }
  static oep::Slot<oep::FixtureDac> d;
  endpoint.add(d.make(pins, dac));
#endif
}

// oep.fixture.spi-target on the core's SPI slave (CH32RVSPISlave; owners::kSpiTarget)
void oepAddSpi() {
#if !defined(ARDUINO_CH32X035)
  static oep::Slot<oep::Ch32SpiTarget> spi;
  endpoint.add(spi.make(pins));
  loopWork.add([] { spi->service(); });
#endif
}

void oepAddSettings() {
  static oep::Slot<oep::Binds> binds;
  static oep::Slot<oep::Settings> config;
  oep::Settings &c = config.make(endpoint, binds.make());
  c.setStorage(false);   // no store on this core yet (core §13.7: no save / erase / state)
  endpoint.setRawPorts(&*binds);
  if (serialPorts) c.usePassthroughs();   // a serial port's raw stream carries them (fixture §2.1)
  if (consoleIn) c.addPlace(rvswd->wire, *consoleIn);
  if (swioConsoleIn) c.addPlace(*swioWireIn, *swioConsoleIn);   // a slot on the one wire (CH32V00x): the second place
  if (swdIn) c.addHold(*swdWireIn, swdIn->hold, oep::checkSwdHold, swdIn);
  if (uartIn) c.addUart(*uartIn);
  c.setPins(&pins);
  loopWork.add([] { config->poll(); });
  c.applySaved();
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

// What this chip has (oep::kAvailable) and the distributed firmware's set (oep::kDefaults). The CH32X035's 62 KiB take
// the WCH set (RVSWD, console, gpio, link, vendor bulk, HID); the larger chips take SWD, SWIO, uart, the I2C / SPI
// targets and controllers, analog, logic, counter, dac and the settings too.
#if defined(ARDUINO_CH32X035)
static constexpr bool kRoomy = false;
#else
static constexpr bool kRoomy = true;
#endif
namespace oep {
inline constexpr uint64_t kAvailable = kBulk | kHid | kCdc | kBridge | kRvswd | kConsole | kSwio | kSwioConsole | kSwd |
                                       kGpio | kUart | kI2cTarget | kAnalog | kLogic | kCounter | kDac | kSpiTarget |
                                       kSettings | kLink | kI2cController | kSpiController;
inline constexpr uint64_t kDefaults = kBulk | kHid | kRvswd | kConsole | kGpio | kLink |
                                      (kRoomy ? kSwd | kUart | kI2cTarget | kSpiTarget | kAnalog | kLogic | kCounter | kDac | kSwio |
                                                     kSwioConsole | kSettings | kI2cController | kSpiController
                                               : 0);
}  // namespace oep

// oep.probe.restart (oep-if-restart): off the bus first (kRestartDetachMs, Oep.h), then CH32RV.restart(). restart_max_ms:
// an estimate (the boot, the host's enumeration), to be measured on the bench.
static constexpr uint32_t kRestartMaxMs = 2000;
static void restartProbe() {
  USBDevice.end();
  delay(oep::kRestartDetachMs);
  oep::platformRestart();
}

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, modelName, id, oep::platformUnitId(id, sizeof id), static_cast<uint16_t>(oep::topChannel(kChannels) + 1));
  oep::describeChip(w);
  return w.ok() ? w.length() : 0;
}

template <uint64_t F>
void firmwareSetup(const oep::FirmwareConfig &cfg) {
  if (cfg.model) modelName = cfg.model;
  if (cfg.product) productName = cfg.product;
  oep::platformUnitId(reinterpret_cast<uint8_t *>(serial), sizeof serial);
  CH32RVUsbDeviceConfig usb;
  usb.vid = oep::reg::kUsbProjectVid;   // the project's VID:PID (registry usb, PID-USE.md)
  usb.pid = oep::reg::kUsbProjectPid;
  usb.manufacturer = "Open Embedded Probe";
  usb.product = productName;
  usb.serialNumber = serial;   // the unit id, lowercase (transports §3)
  // every channel a floating input until the host takes one (oep-core §8): before the transports, a UART bridge's pins
  // among them
  oep::platformParkMask(kChannels);
  if constexpr ((F & oep::kBulk) != 0) oepAddBulk();   // the transports before USBDevice.begin (their interfaces laid out in this order)
  if constexpr ((F & oep::kHid) != 0) oepAddHid();
  if constexpr ((F & oep::kCdc) != 0) oepAddCdc();
  if constexpr ((F & oep::kBridge) != 0) oepAddBridge();
  USBDevice.begin(usb);
  afterUsb.run();
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.setRestart(restartProbe, kRestartMaxMs);
  // The fns in this order, whichever are in (a build's set numbers them; a host finds each by name); the settings last,
  // after the ones they hold
  if constexpr ((F & oep::kRvswd) != 0) {
    oepAddRvswd();
    if constexpr ((F & oep::kConsole) != 0) oepAddConsole();
  }   // and the console's choice
  if constexpr ((F & oep::kSwd) != 0) oepAddSwd();
  if constexpr ((F & oep::kGpio) != 0) oepAddGpio();
  if constexpr ((F & oep::kUart) != 0) oepAddUart();
  if constexpr ((F & oep::kI2cTarget) != 0) oepAddI2c();
  if constexpr ((F & oep::kSwio) != 0) {
    oepAddSwio();
    if constexpr ((F & oep::kSwioConsole) != 0) oepAddSwioConsole();
  }   // and its console's choice
  if constexpr ((F & oep::kLink) != 0) oepAddLink();
  if constexpr ((F & oep::kI2cController) != 0) oepAddI2cController();
  if constexpr ((F & oep::kSpiController) != 0) oepAddSpiController();
  if constexpr ((F & oep::kSpiTarget) != 0) oepAddSpi();
  if constexpr ((F & oep::kAnalog) != 0) oepAddAnalog();
  if constexpr ((F & oep::kLogic) != 0) oepAddLogic();
  if constexpr ((F & oep::kCounter) != 0) oepAddCounter();
  if constexpr ((F & oep::kDac) != 0) oepAddDac();
  oep::firmware.runSetup(endpoint, pins);   // the sketch's own, after the build's (and before the saved settings)
  if constexpr ((F & oep::kSettings) != 0) oepAddSettings();
}

inline void firmwareLoop() {
  deliverEvents();   // the HID output reports' callback runs only here (the core's 0.3.0)
  endpoint.poll();
  loopWork.run();
  oep::firmware.runLoop();
}
