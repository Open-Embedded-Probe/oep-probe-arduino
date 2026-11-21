// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The ESP32 family but the classic and the P4 (their own files: Esp32.h, Esp32P4.h): the ESP32-S3 (profile esp32s3),
// and the chips after it, the parts each takes chosen from what ESP-IDF declares the SoC has (soc/soc_caps.h: SOC_*),
// never from its name - a new target builds with what it declares. One endpoint, its transports each a choice (the
// describe lists those in, in this order):
//
//   vendor bulk      the USB-OTG port (Bulk)                                                  OEP only
//   HID              the USB-OTG port, vendor HID reports (Hid)                               OEP only
//   USB CDC          the USB-OTG port (Cdc)                                                   serial port: OEP + raw
//   USB-Serial/JTAG  the chip's USJ (Usj; HWCDC Serial)                                       serial port: OEP + raw
//   UART bridge      UART0 on its pads (U0TXD / U0RXD), a board's USB-UART chip (Bridge)      serial port: OEP + raw
//
// The USB-OTG port (SOC_USB_OTG_SUPPORTED, a TinyUSB build: USBMode=default) is the project's VID:PID 1209:4F45 (registry
// usb; PID-USE.md), high speed where the SoC has a UTMI PHY (SOC_USB_UTMI_PHY_NUM), else full speed; serial = the unit
// id (the MAC, lowercase hex); its iProduct "OEP probe (ESP32-S3)" (the target's name, CONFIG_IDF_TARGET) is a name for
// people. The vendor bulk interface is class 0xFF, subclass 0x4F, protocol 0x45; the HID's report descriptor says usage
// page 0xFF4F, usage 0x45 (patched over EspUsbDevice's own, as the P4's). A HWCDC build (USBMode=hwcdc) or a SoC with no
// OTG takes the USB-Serial/JTAG (SOC_USB_SERIAL_JTAG_SUPPORTED) instead, which keeps the chip's fixed ID. A first flash,
// an update or a recovery goes through the UART bridge's pads (or the USJ) with esptool.
//
// Interfaces (revision 1), each where the SoC has what it needs: fn 0 (the core); oep.wire.rvswd + oep.target.riscv-dm +
// oep.target.console (SOC_DEDICATED_GPIO_SUPPORTED); oep.fixture.gpio / uart (one per UART beside UART0: SOC_UART_NUM);
// oep.fixture.i2c-target (SOC_I2C_SUPPORT_SLAVE); oep.fixture.spi-target; the settings (core §13) (saved in NVS);
// oep.fixture.analog (ADC1, SOC_ADC_DMA_SUPPORTED); oep.probe.link; oep.wire.swd + oep.target.arm-adi;
// oep.fixture.i2c-controller / spi-controller (GPIOs); oep.fixture.pwm (SOC_LEDC_SUPPORTED) / counter
// (SOC_PCNT_SUPPORTED); oep.wire.swio and its console place (SOC_DEDICATED_GPIO_SUPPORTED); oep.probe.memo;
// oep.probe.plan and oep.probe.restart (the endpoint's own, listed last).
//
// Channels, found at start-up: the SoC's valid GPIOs (SOC_GPIO_VALID_GPIO_MASK) but those ESP-IDF reserved as it started
// (the flash's and the PSRAM's pads, which change with the module: platformUnusablePins), the USB pads while a USB
// transport has them (USBPHY_DM_NUM / USBPHY_DP_NUM) and the UART bridge's. Each may be the RVSWD pair, the reset line
// or any fixture's pin - the host chooses.
// Each is a feature (OepFirmware.h: oep::kX, begin<F>() adding it; oep::kAvailable / kDefaults below; OepChoice.h).
#pragma once
#include <esp_mac.h>
#include <soc/soc_caps.h>
#include <soc/uart_pins.h>

#include <OepBind.h>
#include <OepBootGuard.h>
#include <OepChoice.h>
#include <OepSettings.h>
#include <OepMemo.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepP4SpiTarget.h>
#include <OepI2cController.h>
#include <OepPinTable.h>
#include <OepSpiController.h>
#include <OepSwd.h>
#include <OepTarget.h>

// What the SoC and the build offer (each a declaration, never a chip's name)
#if defined(SOC_USB_OTG_SUPPORTED) && SOC_USB_OTG_SUPPORTED && defined(ARDUINO_USB_MODE) && !ARDUINO_USB_MODE
#define OEP_SOC_OTG 1   // a TinyUSB build on a SoC with USB-OTG: EspUsbDevice
#include <soc/usb_pins.h>
#include <EspUsbDevice.h>
#include <OepDirectBulkStream.h>
#include "UsbStreams.h"
#elif defined(SOC_USB_SERIAL_JTAG_SUPPORTED) && SOC_USB_SERIAL_JTAG_SUPPORTED
#define OEP_SOC_USJ 1   // the chip's USB-Serial/JTAG as the Serial port
#include <soc/usb_pins.h>
#endif
#if defined(SOC_DEDICATED_GPIO_SUPPORTED) && SOC_DEDICATED_GPIO_SUPPORTED
#define OEP_SOC_RVSWD 1
#include <OepCh32Dm.h>
#include <OepConsole.h>
#include <OepDmConsole.h>
#include <OepRvswdPhy.h>
#include <OepSwioPhy.h>
#endif
#if defined(SOC_I2C_SUPPORT_SLAVE) && SOC_I2C_SUPPORT_SLAVE
#define OEP_SOC_I2C_TARGET 1   // I2C0's slave mode
#include <OepP4I2cTarget.h>
#endif
#if defined(SOC_ADC_DMA_SUPPORTED) && SOC_ADC_DMA_SUPPORTED
#define OEP_SOC_ANALOG 1   // ADC1 in continuous mode (DMA)
#include <OepAnalog.h>
#endif
#if defined(SOC_LEDC_SUPPORTED) && SOC_LEDC_SUPPORTED
#define OEP_SOC_PWM 1
#include <OepPwm.h>
#endif
#if defined(SOC_PCNT_SUPPORTED) && SOC_PCNT_SUPPORTED
#define OEP_SOC_COUNTER 1
#include <OepCounter.h>
#endif

static constexpr uint16_t kUnset = 0xfffe;                        // no pair chosen yet

// The probe's names from the target ESP-IDF builds for (CONFIG_IDF_TARGET, "esp32s3"): model "esp32s3", the USB product
// "OEP probe (ESP32-S3)"
static constexpr char kProbeModel[] = CONFIG_IDF_TARGET;
static char usbProduct[40];
static const char *productOf(const char *target) {
  char *o = usbProduct;
  o += snprintf(o, sizeof usbProduct, "OEP probe (");
  for (const char *t = target; *t && o < usbProduct + sizeof usbProduct - 3; ++t) {
    *o++ = static_cast<char>(toupper(static_cast<unsigned char>(*t)));
    if (t - target == 4 && t[1]) *o++ = '-';   // "esp32" then "-": ESP32-S3, ESP32-C6
  }
  *o++ = ')';
  *o = 0;
  return usbProduct;
}
static const char *modelName = kProbeModel, *productName = nullptr;

#if defined(OEP_SOC_OTG)
// The OTG functions with what a host tells them apart by (transports §3), each keeping the interface number the device
// gave it as it built the descriptors (as the P4's, Esp32P4.h).
class OepVendor final : public EspUsbDeviceVendor {
 public:
  using EspUsbDeviceVendor::EspUsbDeviceVendor;
  uint16_t configurationDescriptor(uint8_t *dst, uint8_t interfaceNumber, uint8_t endpointNumber, uint16_t endpointSize) override {
    number = interfaceNumber;
    const uint16_t n = EspUsbDeviceVendor::configurationDescriptor(dst, interfaceNumber, endpointNumber, endpointSize);
    if (dst && n >= 9) { dst[6] = oep::reg::kUsbVendorBulkSubclass; dst[7] = oep::reg::kUsbVendorBulkProtocol; }   // bInterfaceSubClass, bInterfaceProtocol
    return n;
  }
  uint8_t number = 0xff;
};
class OepHid final : public EspUsbDeviceHidVendor {
 public:
  using EspUsbDeviceHidVendor::EspUsbDeviceHidVendor;
  uint16_t configurationDescriptor(uint8_t *dst, uint8_t interfaceNumber, uint8_t endpointNumber, uint16_t endpointSize) override {
    number = interfaceNumber;
    return EspUsbDeviceHidVendor::configurationDescriptor(dst, interfaceNumber, endpointNumber, endpointSize);
  }
  const uint8_t *hidReportDescriptor() const override {
    const uint8_t *base = EspUsbDeviceHidVendor::hidReportDescriptor();
    const uint16_t n = EspUsbDeviceHidVendor::hidReportDescriptorLength();
    if (!base || n < 5 || n > sizeof patched_) return base;
    memcpy(patched_, base, n);
    patched_[1] = oep::reg::kUsbHidUsagePage & 0xff;   // 0x06 page(u16): the library's 0xFF00 becomes 0xFF4F
    patched_[2] = oep::reg::kUsbHidUsagePage >> 8;
    patched_[4] = oep::reg::kUsbHidUsage;              // 0x09 usage: 0x45
    return patched_;
  }
  uint8_t number = 0xff;

 private:
  mutable uint8_t patched_[64];
};
class OepCdc final : public EspUsbDeviceCdcSerial {
 public:
  using EspUsbDeviceCdcSerial::EspUsbDeviceCdcSerial;
  uint16_t configurationDescriptor(uint8_t *dst, uint8_t interfaceNumber, uint8_t endpointNumber, uint16_t endpointSize) override {
    number = interfaceNumber;
    return EspUsbDeviceCdcSerial::configurationDescriptor(dst, interfaceNumber, endpointNumber, endpointSize);
  }
  uint8_t number = 0xff;
};
static EspUsbDevice usbDevice;
#endif

// max_frame 4096 on the vendor port (a serial port's the endpoint's default). The frame buffers in internal RAM, unless
// the SoC's USB-OTG reads PSRAM by DMA (SOC_USB_OTG_PSRAM_DMA_CAPABLE): the S3's does not.
static constexpr size_t kMaxFrame = 4096;
static constexpr size_t kAnswerBytes = 4096;
static constexpr size_t kRxSerial = oep::cobsFrameMax(oep::Endpoint::kDefaultSerialFrame), kRxFrame = kMaxFrame + 2;
static uint8_t txBoot[64];
static uint8_t *frameBuffer(size_t n) {
#if defined(SOC_USB_OTG_PSRAM_DMA_CAPABLE) && SOC_USB_OTG_PSRAM_DMA_CAPABLE
  if (void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)) return static_cast<uint8_t *>(p);
#endif
  return static_cast<uint8_t *>(heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}
static oep::Endpoint endpoint(txBoot, sizeof txBoot, {kMaxFrame, 4 * kMaxFrame, 8});

// The channels: the SoC's valid GPIOs, narrowed at start-up (firmwareSetup) by what the chip and the transports take
static constexpr uint64_t kSocGpios = SOC_GPIO_VALID_GPIO_MASK;
static constexpr int kBridgeTx = U0TXD_GPIO_NUM, kBridgeRx = U0RXD_GPIO_NUM;
static uint64_t channels = kSocGpios;
static uint64_t ownPins = 0;   // the transports' pads: never offered, never parked
static oep::PinTable pins(kSocGpios);
static uint8_t probeTlv[256];
static char serial_[20];

static oep::Hooks<void (*)(), 5> afterUsb;
static oep::Hooks<void (*)(), 12> loopWork;
static oep::Hooks<void (*)(), 1> settingsFn;
static oep::Hooks<void (*)(), 1> lastly;

static oep::SwdPort *swdIn = nullptr;
static oep::WireSwd *swdWireIn = nullptr;
static oep::FixtureUart *uart1In = nullptr, *uart2In = nullptr;

#if defined(OEP_SOC_OTG)
static oep::Slot<OepVendor> vendor;
static oep::Slot<oep::DirectBulkStream> bulk;
void oepAddBulk() {
  bulk.make(vendor.make(usbDevice));
  static bool direct = false;
  direct = bulk->begin();   // before the device starts: its callbacks are registered while it is stopped
  afterUsb.add([] {
    endpoint.addTransport(*bulk, frameBuffer(kRxFrame), kRxFrame, oep::Endpoint::kVendorBulk, vendor->number, true);
    if (direct) endpoint.setDirect(&*bulk);
  });
}

// usb.transports (transports §3.1): the OTG device's sets, the first the default; a set the device refuses to start
// falls back to the default (the controller's IN endpoints bound them)
struct UsbSet { const char *name; bool hid; uint8_t cdc; };
static constexpr UsbSet kUsbSets[] = {{"bulk-hid-cdc", true, 1}, {"bulk-hid", true, 0}, {"bulk-cdc2", false, 2}};
static constexpr uint8_t kUsbSetCount = sizeof kUsbSets / sizeof kUsbSets[0], kMaxCdc = 2;
static const char *const kUsbSetNames[kUsbSetCount] = {kUsbSets[0].name, kUsbSets[1].name, kUsbSets[2].name};
static uint8_t usbSet = 0;

static oep::Slot<OepHid> hid;
static oep::Slot<HidStream> hidStream;
static bool hidMade = false, hidChosen = false;
static void makeHid() {
  if (!hidChosen || !kUsbSets[usbSet].hid || hidMade) return;
  hidStream.make(hid.make(usbDevice, HidStream::kReport));
  hidMade = true;
}
void oepAddHid() {
  hidChosen = true;
  makeHid();
  afterUsb.add([] {
    if (!hidMade) return;
    endpoint.addTransport(*hidStream, frameBuffer(kRxFrame), kRxFrame, oep::Endpoint::kHid, hid->number, true);
    endpoint.useHidInput(*hidStream, hidStream->in());
  });
}

static oep::Slot<OepCdc> cdcs[kMaxCdc];
static oep::Slot<CdcStream> cdcStreams[kMaxCdc];
static const char *const kCdcNames[kMaxCdc] = {"OEP", "OEP 2"};
static uint8_t cdcMade = 0;
static bool cdcChosen = false;
static void makeCdcs() {
  for (; cdcChosen && cdcMade < kUsbSets[usbSet].cdc; ++cdcMade) cdcStreams[cdcMade].make(cdcs[cdcMade].make(usbDevice, kCdcNames[cdcMade]));
}
void oepAddCdc() {
  cdcChosen = true;
  makeCdcs();
  afterUsb.add([] {
    for (uint8_t i = 0; i < cdcMade; ++i)
      endpoint.addTransport(*cdcStreams[i], frameBuffer(kRxSerial), kRxSerial, oep::Endpoint::kUsbCdc, cdcs[i]->number, true);
  });
}
static void fallBackUsbSet() {
  while (cdcMade) { --cdcMade; cdcs[cdcMade].destroy(); cdcStreams[cdcMade].destroy(); }
  if (hidMade) { hid.destroy(); hidStream.destroy(); hidMade = false; }
  usbSet = 0;
  makeHid();
  makeCdcs();
}
#endif

#if defined(OEP_SOC_USJ)
void oepAddUsj() {
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.setTxTimeoutMs(0);   // a port nobody reads never stops loop()
  Serial.begin(115200);
  afterUsb.add([] { endpoint.addTransport(Serial, frameBuffer(kRxSerial), kRxSerial, oep::Endpoint::kUsbSerialJtag); });
}
#endif

// The UART bridge (transports §1: a serial transport, COBS): UART0 on its own pads, 115200 - a board's USB-UART chip, the
// boot ROM's download and log. In by default: a host reaches OEP there too, and esptool updates the firmware there.
void oepAddBridge() {
  static oep::Slot<HardwareSerial> port;
  ownPins |= (uint64_t{1} << kBridgeTx) | (uint64_t{1} << kBridgeRx);
  port.make(0).begin(115200, SERIAL_8N1, kBridgeRx, kBridgeTx);
  afterUsb.add([] { endpoint.addTransport(*port, frameBuffer(kRxSerial), kRxSerial, oep::Endpoint::kUartBridge); });
}

#if defined(OEP_SOC_RVSWD)
static oep::TargetConsoleStream *consoleIn = nullptr;
static oep::TargetConsole *consoles = nullptr;
static void addConsolePlace(oep::TargetConsoleStream &place) {
  if (consoles) { consoles->add(place); return; }
  static oep::Slot<oep::TargetConsole> one;
  consoles = &one.make(place);
  endpoint.add(*consoles);
}
struct RvswdParts {
  oep::RvswdPhy phy;
  oep::Ch32Dm dm{phy};
  oep::DebugPort port{dm, kUnset, kUnset};
  oep::WireRvswd wire{port, 0};
  RvswdParts() {
    port.pin_choice = channels;
    port.pins = &pins;
    port.reset_allowed = channels;
  }
};
static oep::Slot<RvswdParts> rvswd;
static oep::Slot<oep::TargetRiscvDm> riscvDm;
static oep::TargetRiscvDm *riscvDmIn = nullptr;

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

// The one-wire link (CH32V00x) on the same dedicated GPIO: its own wire fn, its connections served by the RVSWD wire's
// riscv-dm target (or one of its own without that wire), its console a second place of the one oep.target.console
static oep::TargetConsoleStream *swioConsoleIn = nullptr;
struct SwioParts {
  oep::SwioPhy phy;
  oep::Ch32Dm dm{phy};
  oep::DebugPort port{dm, kUnset, 0xffff};   // one wire: swclk stays 0xffff
  oep::WireRvswd wire{port, 0, "oep.wire.swio"};
  SwioParts() {
    port.pin_choice = channels;
    port.pins = &pins;
    port.pin_owner = oep::owners::kSecondWire;
    port.reset_allowed = channels;
  }
};
static oep::Slot<SwioParts> swio;

void oepAddSwio() {
  SwioParts &w = swio.make();
  endpoint.add(w.wire);
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
  swioConsoleIn = &console.make(swio->port, driver.make(swio->dm, swio->phy), 0);
  addConsolePlace(*swioConsoleIn);
  loopWork.add([] { console->poll(); });
}
#endif

void oepAddGpio() {
  static oep::Slot<oep::FixtureGpio> gpio;
  endpoint.add(gpio.make(pins, 0, oep::owners::kGpio));
}

// The fixture UARTs: one per UART beside UART0 (the bridge's), at most two; their interrupts on the core loop() does not
// run on where there are two (as the P4's, Esp32P4.h)
static constexpr int kFixtureUarts = SOC_UART_NUM - 1 < 2 ? SOC_UART_NUM - 1 : 2;
void oepAddUart() {
  static oep::Slot<oep::FixtureUart> uart1, uart2;
  uart1In = &uart1.make(pins, Serial1, 0, oep::owners::kUart);
#if SOC_CPU_CORES_NUM > 1
  uart1->setInterruptCore(CONFIG_ARDUINO_RUNNING_CORE == 0 ? 1 : 0);
#endif
  endpoint.add(*uart1);
  if constexpr (kFixtureUarts > 1) {
#if SOC_UART_NUM > 2
    uart2In = &uart2.make(pins, Serial2, 1, oep::owners::kUart2);
#if SOC_CPU_CORES_NUM > 1
    uart2->setInterruptCore(CONFIG_ARDUINO_RUNNING_CORE == 0 ? 1 : 0);
#endif
    endpoint.add(*uart2);
#endif
  }
  loopWork.add([] {
    uart1->poll();
    if (uart2In) uart2In->poll();
  });
}

#if defined(OEP_SOC_I2C_TARGET)
void oepAddI2c() {
  static oep::Slot<oep::P4I2cTarget> i2c;
  endpoint.add(i2c.make(pins));
  loopWork.add([] { i2c->service(); });
}
#endif

#if defined(OEP_SOC_ANALOG)
static oep::AnalogCapture *analogIn = nullptr;
// ADC1's pads, as the driver maps its channels
static uint64_t adc1Pins() {
  uint64_t m = 0;
  for (int ch = 0; ch < SOC_ADC_CHANNEL_NUM(0); ++ch) {
    int io = -1;
    if (adc_continuous_channel_to_io(ADC_UNIT_1, adc_channel_t(ch), &io) == ESP_OK && io >= 0) m |= uint64_t{1} << io;
  }
  return m;
}
void oepAddAnalog() {
  static oep::Slot<oep::AnalogCapture> analog;
  analogIn = &analog.make(endpoint, adc1Pins() & channels, 0);
  analog->setPins(&pins, oep::owners::kAnalog);   // its pads go analog: claimed against wires and settings
  endpoint.add(*analog);
  loopWork.add([] { analog->poll(); });
}
#endif

void oepAddSpi() {
  static oep::Slot<oep::P4SpiTarget> spi;
  endpoint.add(spi.make(pins));
  loopWork.add([] { spi->service(); });
}

static bool usbReady() {
#if defined(OEP_SOC_OTG)
  return usbDevice.ready();
#else
  return false;   // no USB device of its own: BootGuard's grace time decides
#endif
}
static bool autoAttachReady() { return oep::BootGuard::attachReady(usbReady()); }

void oepAddSettings() {
  static oep::Slot<oep::Binds> binds;
  static oep::Slot<oep::Settings> config;
  oep::Settings &c = config.make(endpoint, binds.make());
  endpoint.setRawPorts(&*binds);
  c.load();
  pins.setDisabled(c.savedDisabled());
  settingsFn.add([] {
    config->usePassthroughs();
#if defined(OEP_SOC_RVSWD)
    if (consoleIn) config->addPlace(rvswd->wire, *consoleIn);
#endif
    if (uart1In) config->addUart(*uart1In);
    if (uart2In) config->addUart(*uart2In);
    config->setPins(&pins);
#if defined(OEP_SOC_OTG)
    config->setUsbTransports(kUsbSetNames, kUsbSetCount);
#endif
    loopWork.add([] { config->poll(); });
  });
  lastly.add([] {
    if (swdIn) config->addHold(*swdWireIn, swdIn->hold, oep::checkSwdHold, swdIn);
#if defined(OEP_SOC_RVSWD)
    if (swioConsoleIn) config->addPlace(swio->wire, *swioConsoleIn);   // a slot on the one wire: the second place
#endif
    if (oep::BootGuard::safe()) config->skipBootAttach();
    config->setAttachGate(autoAttachReady);
    config->applySaved();
  });
}

void oepAddSwd() {
  static oep::SwdPort swd{kUnset, kUnset};
  static oep::Slot<oep::WireSwd> wireSwd;
  static oep::Slot<oep::TargetArmAdi> adi;
  swd.pin_choice = channels & ~oep::platformInputOnlyPins();
  swd.pins = &pins;
  endpoint.add(wireSwd.make(swd, 0));
  endpoint.add(adi.make(swd, 0));
  swdIn = &swd;
  swdWireIn = &*wireSwd;
}

void oepAddI2cController() {
  static oep::Slot<oep::I2cController> i2c;
  endpoint.add(i2c.make(pins));
}

void oepAddSpiController() {
  static oep::Slot<oep::SpiController> spi;
  endpoint.add(spi.make(pins));
}

#if defined(OEP_SOC_PWM)
void oepAddPwm() {
  static oep::Slot<oep::FixturePwm> pwm;
  endpoint.add(pwm.make(pins));
}
#endif

#if defined(OEP_SOC_COUNTER)
void oepAddCounter() {
  static oep::Slot<oep::FixtureCounter> counter;
  endpoint.add(counter.make(pins));
}
#endif

void oepAddMemo() {
  static oep::Memo memo;
  if (memo.begin()) endpoint.add(memo);
}

void oepAddLink() {
  static oep::Slot<oep::Link> link;
  endpoint.add(link.make(endpoint));
}

// What this SoC has (oep::kAvailable: from its declarations above) and the distributed firmware's set (all of them)
namespace oep {
inline constexpr uint64_t kSocUsb =
#if defined(OEP_SOC_OTG)
    kBulk | kHid | kCdc;
#elif defined(OEP_SOC_USJ)
    kUsj;
#else
    0;
#endif
inline constexpr uint64_t kSocRvswd =
#if defined(OEP_SOC_RVSWD)
    kRvswd | kConsole | kSwio | kSwioConsole;
#else
    0;
#endif
inline constexpr uint64_t kSocI2cTarget =
#if defined(OEP_SOC_I2C_TARGET)
    kI2cTarget;
#else
    0;
#endif
inline constexpr uint64_t kSocAnalog =
#if defined(OEP_SOC_ANALOG)
    kAnalog;
#else
    0;
#endif
inline constexpr uint64_t kSocPwm =
#if defined(OEP_SOC_PWM)
    kPwm;
#else
    0;
#endif
inline constexpr uint64_t kSocCounter =
#if defined(OEP_SOC_COUNTER)
    kCounter;
#else
    0;
#endif
inline constexpr uint64_t kAvailable = kSocUsb | kBridge | kSocRvswd | kGpio | kUart | kSocI2cTarget | kSpiTarget |
                                       kSettings | kSocAnalog | kSwd | kI2cController | kSpiController | kSocPwm |
                                       kSocCounter | kMemo | kLink;
inline constexpr uint64_t kDefaults = kAvailable;
}  // namespace oep

// oep.probe.restart: off the bus first (the host records an unplug), then esp_restart. restart_max_ms: an estimate (the
// boot, the USB device enumerated again), to be measured on the bench.
static constexpr uint32_t kRestartMaxMs = 3000;
static void restartProbe() {
  oep::BootGuard::planned();
#if defined(OEP_SOC_OTG)
  usbDevice.end();
#endif
  delay(oep::kRestartDetachMs);
  esp_restart();
}

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, modelName, id, oep::platformUnitId(id, sizeof id), SOC_GPIO_PIN_COUNT, oep::BootGuard::lastBoot());
  oep::describeChip(w);
  return w.ok() ? w.length() : 0;
}

template <uint64_t F>
void firmwareSetup(const oep::FirmwareConfig &cfg) {
  if (cfg.model) modelName = cfg.model;
  productName = cfg.product ? cfg.product : productOf(CONFIG_IDF_TARGET);
  oep::BootGuard::begin();
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §3)
  oep::platformUnitId(reinterpret_cast<uint8_t *>(serial_), sizeof serial_);
  // The USB pads while a USB transport has them
#if defined(OEP_SOC_OTG) || defined(OEP_SOC_USJ)
  ownPins |= (uint64_t{1} << USBPHY_DM_NUM) | (uint64_t{1} << USBPHY_DP_NUM);
#endif
#if defined(OEP_SOC_OTG)
  EspUsbDeviceConfig usb;
  usb.vid = oep::reg::kUsbProjectVid;
  usb.pid = oep::reg::kUsbProjectPid;
  usb.manufacturer = "Open Embedded Probe";
  usb.product = productName;
  usb.serialNumber = serial_;
#if defined(SOC_USB_UTMI_PHY_NUM) && SOC_USB_UTMI_PHY_NUM > 0
  usb.controller = EspUsbController::HighSpeed;
#else
  usb.controller = EspUsbController::FullSpeed;
#endif
  const int saved = oep::Settings::savedUsbTransports(kUsbSetNames, kUsbSetCount);
  usbSet = saved < 0 ? 0 : static_cast<uint8_t>(saved);
  if constexpr ((F & oep::kBulk) != 0) oepAddBulk();
  if constexpr ((F & oep::kHid) != 0) oepAddHid();
  if constexpr ((F & oep::kCdc) != 0) oepAddCdc();
#endif
#if defined(OEP_SOC_USJ)
  if constexpr ((F & oep::kUsj) != 0) oepAddUsj();
#endif
  if constexpr ((F & oep::kBridge) != 0) oepAddBridge();
  // The channels: what the chip and the transports leave (before any part takes its pin choice from them)
  channels = kSocGpios & ~oep::platformUnusablePins() & ~ownPins;
  pins.forbid(~channels);
#if defined(OEP_SOC_OTG)
  if (!usbDevice.begin(usb) && usbSet != 0) {
    fallBackUsbSet();
    usbDevice.begin(usb);
  }
#endif
  endpoint.setAnswerBuffer(frameBuffer(kAnswerBytes), kAnswerBytes);
  afterUsb.run();
  endpoint.setRestart(restartProbe, kRestartMaxMs);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  if constexpr ((F & oep::kSettings) != 0) oepAddSettings();
#if defined(OEP_SOC_RVSWD)
  if constexpr ((F & oep::kRvswd) != 0) {
    oepAddRvswd();
    if constexpr ((F & oep::kConsole) != 0) oepAddConsole();
  }
#endif
  if constexpr ((F & oep::kGpio) != 0) oepAddGpio();
  if constexpr ((F & oep::kUart) != 0) oepAddUart();
#if defined(OEP_SOC_I2C_TARGET)
  if constexpr ((F & oep::kI2cTarget) != 0) oepAddI2c();
#endif
  if constexpr ((F & oep::kSpiTarget) != 0) oepAddSpi();
  settingsFn.run();
#if defined(OEP_SOC_ANALOG)
  if constexpr ((F & oep::kAnalog) != 0) oepAddAnalog();   // after the settings: the fns before it keep their numbers
#endif
  if constexpr ((F & oep::kLink) != 0) oepAddLink();
  if constexpr ((F & oep::kSwd) != 0) oepAddSwd();
  if constexpr ((F & oep::kI2cController) != 0) oepAddI2cController();
  if constexpr ((F & oep::kSpiController) != 0) oepAddSpiController();
#if defined(OEP_SOC_PWM)
  if constexpr ((F & oep::kPwm) != 0) oepAddPwm();
#endif
#if defined(OEP_SOC_COUNTER)
  if constexpr ((F & oep::kCounter) != 0) oepAddCounter();
#endif
#if defined(OEP_SOC_RVSWD)
  if constexpr ((F & oep::kSwio) != 0) {
    oepAddSwio();
    if constexpr ((F & oep::kSwioConsole) != 0) oepAddSwioConsole();
  }
#endif
  if constexpr ((F & oep::kMemo) != 0) oepAddMemo();
  oep::platformParkMask(channels & ~pins.disabledMask());
  oep::firmware.runSetup(endpoint, pins);
  lastly.run();
}

inline void firmwareLoop() {
  oep::BootGuard::poll();
  endpoint.poll();
  loopWork.run();
  oep::firmware.runLoop();
}
