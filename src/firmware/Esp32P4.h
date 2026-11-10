// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// ESP32-P4 / P4X (profiles esp32p4 / esp32p4x). One endpoint, up to five transports, each a choice (the describe lists
// those in, in this order):
//
//   vendor bulk      HS port (Bulk; the direct build: build_opt.h, compile with --clean)     OEP only
//   USB-Serial/JTAG  the P4's USJ (Usj; HWCDC Serial)                                         serial port: OEP + raw
//   HID              HS port, vendor HID reports (Hid)                                        OEP only
//   USB CDC          HS port (Cdc)                                                            serial port: OEP + raw
//   UART bridge      UART3 on GPIO37 / 38, UART0's pads (Bridge)                              serial port: OEP + raw
//
//   (and a DFU interface on the HS port (Dfu): the probe's own firmware update, outside OEP, part of the same USB device)
//
// Updating the firmware over the HS port alone: `dfu-util -D OepProbe-<model>-<version>.bin` (the release's app image;
// model = esp32p4 or esp32p4x). The custom app descriptor is checked against the actual chip before accepting DFU.
// writes the other app partition, checks it and restarts into it; settings (NVS) stay. The new firmware is confirmed as
// it starts (see verifyRollbackLater): no trial, and no later reset takes the probe back to the one before. The DFU
// interface takes no endpoint (EP0 only). A first flash of an empty chip, or a recovery, is esptool on USB-Serial/JTAG
// with the merged image.
//
// A serial port always takes OEP frames (0x00 <COBS> 0x00); its other bytes are what its bind carries (the settings (core §13):
// a slot's console, a fixture UART). The HS device is the project's VID:PID 1209:4F45 (registry usb; PID-USE.md), serial =
// the unit id (the MAC, lowercase hex); its iProduct "OEP probe (ESP32-P4)" / "OEP probe (ESP32-P4X)" is a name for people. USB-Serial/JTAG keeps the
// chip's fixed ID: a host reaches it by the user choosing its port.
// How a host tells the ports apart inside a device known to be OEP (transports §3, registry usb): the vendor bulk interface is class 0xFF, subclass 0x4F
// ('O'), protocol 0x45 ('E'); the HID's report descriptor says usage page 0xFF4F, usage 0x45. EspUsbDevice writes 0 / 0
// and 0xFF00 / 1 itself, so the two functions below patch their descriptors.
//
// Interfaces (revision 1): fn 0 (the core); oep.wire.rvswd + oep.target.riscv-dm + oep.target.console; oep.wire.swio (WCH
// CH32V00x, one wire: its connections go through the same oep.target.riscv-dm, its console a place of the one oep.target.console,
// and a slot may name it: the second place); oep.fixture.gpio /
// uart (x2) / capture (PARLIO: up to 16 channels, 2 ch 160 Msps / 8 ch 40 Msps / 16 ch 20 Msps); the ESP-IDF SPI / I2C
// devices oep.fixture.spi-target / i2c-target; the settings (core §13) (saved in NVS); oep.fixture.analog
// (ADC1 on GPIO16-23, up to 4 channels, 46 kHz in all) and oep.fixture.capture-group (the analog with the logic);
// oep.probe.link; oep.fixture.i2c-controller / spi-controller (GPIOs, OepI2cController.h / OepSpiController.h);
// oep.probe.plan and oep.probe.restart (the endpoint's own, listed last). Every GPIO but the
// USB-Serial/JTAG pair (24, 25) and the UART bridge's (37, 38) may be the RVSWD pair, the SWIO pin, the reset line, or any
// fixture's pin - the host chooses.
// Each is a feature (OepFirmware.h: oep::kX, begin<F>() adding it; oep::kAvailable / kDefaults below; OepChoice.h): Rvswd (and
// Console in it), Gpio, Uart (both), Capture, I2c, Spi, Config, Analog, Group, Swio (and SwioConsole in it), Link, Swd
// (oep.wire.swd + oep.target.arm-adi), I2cController, SpiController; all in by default.
#pragma once
#include <esp_mac.h>
#include <esp_app_desc.h>
#include <esp_app_format.h>
#include <esp_ota_ops.h>
#include <soc/usb_serial_jtag_reg.h>
#include <EspUsbDevice.h>

#include <OepAnalog.h>
#include <OepBind.h>
#include <OepBootGuard.h>
#include <OepCapture.h>
#include <OepCaptureGroup.h>
#include <OepCh32Dm.h>
#include <OepChoice.h>
#include <OepSettings.h>
#include <OepMemo.h>
#include <OepConsole.h>
#include <OepDirectBulkStream.h>
#include <OepDmConsole.h>
#include <OepEndpoint.h>
#include <OepFixture.h>
#include <OepP4I2cTarget.h>
#include <OepP4SpiTarget.h>
#include <OepCounter.h>
#include <OepI2cController.h>
#include <OepPwm.h>
#include <OepPinTable.h>
#include <OepRvswdPhy.h>
#include <OepSpiController.h>
#include <OepSwd.h>
#include <OepSwioPhy.h>
#include <OepTarget.h>
#include "UsbStreams.h"
#include "P4FirmwareIdentity.h"

static constexpr uint16_t kUnset = 0xfffe;                        // no pair chosen yet

// The SDK selected by ChipVariant: pre-v3 silicon and v3+ silicon need different firmware.
// describeChip additionally reports the actual silicon revision read from eFuse.
#if CONFIG_ESP32P4_REV_MIN_FULL >= 300
static constexpr char kProbeModel[] = "esp32p4x";
static constexpr char kUsbProduct[] = "OEP probe (ESP32-P4X)";
static constexpr uint16_t kFirmwareVariant = P4FirmwareIdentity::kP4X;
#else
static constexpr char kProbeModel[] = "esp32p4";
static constexpr char kUsbProduct[] = "OEP probe (ESP32-P4)";
static constexpr uint16_t kFirmwareVariant = P4FirmwareIdentity::kP4;
#endif

// the names in use: these, or the sketch's FirmwareConfig
static const char *modelName = kProbeModel, *productName = kUsbProduct;

// Arduino's elf2image recipe does not put the SDK's revision bounds in the image header.
// Keep our descriptor in the final image even when compiler constant folding removes references to it.
static const P4FirmwareIdentity kFirmwareIdentity
    __attribute__((section(".rodata_custom_desc"), used, retain, aligned(4))) = {
      P4FirmwareIdentity::kMagic, P4FirmwareIdentity::kVersion, kFirmwareVariant,
      CONFIG_ESP32P4_REV_MIN_FULL, CONFIG_ESP32P4_REV_MAX_FULL, 0
    };

// The HS functions with what a host tells them apart by (transports §3), each keeping the interface number the device
// gave it as it built the descriptors (in usbDevice.begin: the class order, HID, vendor, CDC, DFU, of those in). The
// device asks each for its descriptor twice, first with no buffer to measure it.
// The vendor bulk function with OEP's subclass / protocol in its interface descriptor.
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
// The vendor HID with OEP's usage page / usage at the top of its report descriptor.
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
// The CDC port (its communication interface's number: the describe's, core §7.5)
class OepCdc final : public EspUsbDeviceCdcSerial {
 public:
  using EspUsbDeviceCdcSerial::EspUsbDeviceCdcSerial;
  uint16_t configurationDescriptor(uint8_t *dst, uint8_t interfaceNumber, uint8_t endpointNumber, uint16_t endpointSize) override {
    number = interfaceNumber;
    return EspUsbDeviceCdcSerial::configurationDescriptor(dst, interfaceNumber, endpointNumber, endpointSize);
  }
  uint8_t number = 0xff;
};

// The HS device: its functions are made (by their choices) before it starts.
static EspUsbDevice usbDevice;

// max_frame 27136 (53 x 512) on the vendor port (each route its own, core §7.1; a serial port's at most 2048): a
// streaming capture's zero-copy stage is one frame, one 27 KiB USB transfer - what the HS port needs for 40 MB/s and
// more (45.6 MB/s measured with 27,136-byte transfers; at 1024 a stream topped out near 7 MB/s, at 8192 near 27). The
// frame-sized buffers come from PSRAM in setup() (up only by then; every P4 has it): internal RAM is what the DMA ring
// and the stages need. An answer is at most kAnswerBytes (the vendor stream's result buffers). The endpoint starts on
// a small static answer buffer, swapped before its first poll.
static constexpr size_t kMaxFrame = 27136;   // 53 x 512: a stage is one 27 KiB transfer (45 MB/s on the HS port)
static constexpr size_t kAnswerBytes = 16384;   // an answer's most (DirectBulkStream's result buffers, internal RAM)
static constexpr size_t kRxSerial = oep::cobsFrameMax(oep::Endpoint::kDefaultSerialFrame), kRxFrame = kMaxFrame + 2;
static uint8_t txBoot[64];
static uint8_t *frameBuffer(size_t n) {
  void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return static_cast<uint8_t *>(p ? p : heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}
static oep::Endpoint endpoint(txBoot, sizeof txBoot, {kMaxFrame, 4 * kMaxFrame, 8});

// Reserved: GPIO24/25 (USB-Serial/JTAG). Every other GPIO is a channel the host may give to anything.
static constexpr uint64_t kReserved = (1ull << 24) | (1ull << 25);
static constexpr uint64_t kChannels = ((1ull << 55) - 1) & ~kReserved;
static oep::PinTable pins(kChannels);
static uint8_t probeTlv[256];   // with the firmware text's note of the boot before (BootGuard::lastBoot)
static char serial_[20];
// Who holds a channel in pins: oep::owners (OepPinTable.h), the second uart kUart2, the SWIO wire beside RVSWD kSecondWire

// What a choice leaves to run later, each added only from its own oepAddX(): what waits for the HS device (the
// transports, in the describe's order), its loop work; the settings' fn at its place, what comes last
static oep::Hooks<void (*)(), 5> afterUsb;
static oep::Hooks<void (*)(), 12> loopWork;
static oep::Hooks<void (*)(), 1> settingsFn;
static oep::Hooks<void (*)(), 1> lastly;

// What a choice leaves for another to find, each set only from its own oepAddX(); the code using both is the other's (the
// settings', the group's, the SWIO wire's), so neither names the other's code: the consoles (and the SWIO wire) and the
// uarts for the settings, the logic and the analog for the group, the riscv-dm target for the SWIO wire
static oep::SwdPort *swdIn = nullptr;
static oep::WireSwd *swdWireIn = nullptr;   // its interface: the owner of its hold item (core §13.1)
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
static oep::FixtureUart *uart1In = nullptr, *uart2In = nullptr;
static oep::LogicCapture *captureIn = nullptr;
static oep::AnalogCapture *analogIn = nullptr;
static oep::TargetRiscvDm *riscvDmIn = nullptr;

// The vendor bulk port, with its zero-copy path (borrowed IN transfers): a capture's pushes go out as whole stages through
// writeDirect while its subscriber came in on the vendor port (without it every push went through the loop, one max_frame
// at a time: 5-7 MB/s on the P4X, 2026-10-20). The first transport (the one a DirectTransport belongs to).
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

void oepAddUsj() {   // USB-Serial/JTAG: opening and closing the port must not reset the probe (probe guide §7)
#if defined(USB_DEVICE_CHIP_RST_REG)
  // P4 revision >= 3 headers rename this register; the reset-disable bit is unchanged.
  REG_SET_BIT(USB_DEVICE_CHIP_RST_REG, USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS);
#else
  REG_SET_BIT(USB_SERIAL_JTAG_CHIP_RST_REG, USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS);
#endif
  Serial.setRxBufferSize(8192);
  Serial.setTxBufferSize(8192);
  Serial.setTxTimeoutMs(0);   // a port nobody reads never stops loop()
  Serial.begin(115200);
  afterUsb.add([] { endpoint.addTransport(Serial, frameBuffer(kRxSerial), kRxSerial, oep::Endpoint::kUsbSerialJtag); });
}

// usb.transports (transports §3.1): the sets of the HS device's OEP transports this probe can start - the vendor bulk
// port always (the first transport), the HID or not, 0-3 CDC ports - each one the HS controller's 7 IN endpoints and
// FIFO hold (begin() checked them on the bench: HID + vendor + 3 CDC does not fit). Named by transports §3.1's rule;
// the first is the default (no value, or one this firmware does not list). Chosen at boot from the save
// (savedUsbTransports), before the device starts; a set the device refuses to start falls back to the default.
struct UsbSet { const char *name; bool hid; uint8_t cdc; };
static constexpr UsbSet kUsbSets[] = {
    {"bulk-hid-cdc", true, 1}, {"bulk-hid", true, 0}, {"bulk-hid-cdc2", true, 2}, {"bulk-cdc3", false, 3}};
static constexpr uint8_t kUsbSetCount = sizeof kUsbSets / sizeof kUsbSets[0], kMaxCdc = 3;
static const char *const kUsbSetNames[kUsbSetCount] = {kUsbSets[0].name, kUsbSets[1].name, kUsbSets[2].name, kUsbSets[3].name};
static uint8_t usbSet = 0;   // the set this boot runs

static oep::Slot<OepHid> hid;
static oep::Slot<HidStream> hidStream;
static bool hidMade = false, hidChosen = false;   // chosen: the build has the HID (kHid)
static void makeHid() {
  if (!hidChosen || !kUsbSets[usbSet].hid || hidMade) return;
  hidStream.make(hid.make(usbDevice, HidStream::kReport));
  hidMade = true;
}
void oepAddHid() {
  hidChosen = true;
  makeHid();
  afterUsb.add([] {
    if (hidMade) endpoint.addTransport(*hidStream, frameBuffer(kRxFrame), kRxFrame, oep::Endpoint::kHid, hid->number, true);
  });
}

// The CDC ports of the set, each a name to show (every CDC of the probe speaks OEP; a composite that repeats a class
// needs a name to tell them apart)
static oep::Slot<OepCdc> cdcs[kMaxCdc];
static oep::Slot<CdcStream> cdcStreams[kMaxCdc];
static const char *const kCdcNames[kMaxCdc] = {"OEP", "OEP 2", "OEP 3"};
static uint8_t cdcMade = 0;
static bool cdcChosen = false;   // the build has the CDC ports (kCdc)
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
// The device refused the set (begin() false): its HID and CDC ports undone, the default's made in their place
static void fallBackUsbSet() {
  while (cdcMade) { --cdcMade; cdcs[cdcMade].destroy(); cdcStreams[cdcMade].destroy(); }
  if (hidMade) { hid.destroy(); hidStream.destroy(); hidMade = false; }
  usbSet = 0;
  makeHid();
  makeCdcs();
}

// A UART bridge (transports §1: a serial transport, COBS): UART3 on UART0's pads, GPIO37 (TX) / GPIO38 (RX), 115200 -
// those two the probe's own (never offered, never parked). In by default: they are the chip's default UART (the boot
// ROM's download and log), so a USB-UART adapter there reaches OEP too, and no fixture takes them.
static constexpr int kBridgeTx = 37, kBridgeRx = 38;
static uint64_t ownPins = 0;
void oepAddBridge() {
  static oep::Slot<HardwareSerial> port;
  ownPins |= (1ull << kBridgeTx) | (1ull << kBridgeRx);
  pins.forbid(ownPins);
  port.make(3).begin(115200, SERIAL_8N1, kBridgeRx, kBridgeTx);
  afterUsb.add([] { endpoint.addTransport(*port, frameBuffer(kRxSerial), kRxSerial, oep::Endpoint::kUartBridge); });
}

// Dfu. A DFU update's image is taken as working: it is confirmed as soon as it starts. The core's sdkconfig has the
// bootloader's rollback (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE), under which a new image stays pending until confirmed
// and the next boot of any kind goes back to the one before. The sketch does not define verifyRollbackLater, so the
// core's (false) stands and initArduino confirms the image (esp_ota_mark_app_valid_cancel_rollback) before setup() runs
// - the earliest point an app can. A panic, a watchdog or a host's re-enumeration after that never goes back; a crash
// loop the saved settings set off is BootGuard's safe boot. What the bootloader still does: an image failing its own
// check, or a reset before the core confirmed it, boots the one before (BootGuard::lastBoot says "update to <slot> did
// not reach setup").

// A DFU update's restart into the new image, done here and not by EspUsbDevice (restartWhenComplete off): its restart
// detached and reset 20 ms later, and the P4 came back failing its device descriptor request until a replug (bench, as
// oep.probe.restart's above). The host's last GETSTATUS is answered first (kDfuStatusMs, as EspUsbDevice waits), then
// the device goes off the bus for kDfuDetachMs before esp_restart (no answer of OEP's is waiting: no time limit).
static constexpr uint32_t kDfuStatusMs = 500, kDfuDetachMs = 100;
static volatile bool dfuDone = false;
static volatile uint32_t dfuDoneMs = 0;
static bool compatibleDfuImage() {
  // EspUsbDevice has verified the image and selected its partition before calling onComplete.
  // Returning false makes it restore the running partition and report DFU_STATUS_ERR_FIRMWARE.
  const esp_partition_t *candidate = esp_ota_get_boot_partition();
  if (!candidate || candidate == esp_ota_get_running_partition()) return false;
  P4FirmwareIdentity identity = {};
  constexpr size_t offset = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t);
  if (esp_partition_read(candidate, offset, &identity, sizeof identity) != ESP_OK) return false;
  return identity.compatible(kFirmwareVariant, ESP.getChipRevision());
}
static void restartAfterDfu() {
  if (!dfuDone || millis() - dfuDoneMs < kDfuStatusMs) return;
  oep::BootGuard::planned();
  usbDevice.end();   // off the bus: the host sees an unplug
  delay(kDfuDetachMs);
  esp_restart();
}

void oepAddDfu() {
  static oep::Slot<EspUsbDeviceDfu> dfu;
  dfu.make(usbDevice, EspUsbDeviceDfuMode::Download, "OEP probe firmware");
  EspUsbDfuConfig c = dfu->configuration();
  c.transferBytes = 4096;   // the block dfu-util sends (4 KiB: a flash sector a block)
  dfu->configure(c);
  dfu->restartWhenComplete(false);   // restartAfterDfu, from loop()
  dfu->onComplete([]() {   // the USB worker: the image verified, the host's last GETSTATUS still to answer
    if (!compatibleDfuImage()) return false;
    dfuDoneMs = millis();
    dfuDone = true;
    return true;
  });
  loopWork.add(restartAfterDfu);
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
    port.reset_allowed = kChannels;   // attach's reset TLV: the channel the host names (no default), nobody holding it
  }
};
static oep::Slot<RvswdParts> rvswd;
static oep::Slot<oep::TargetRiscvDm> riscvDm;   // the RVSWD wire's, the SWIO wire's connections too

void oepAddRvswd() {
  RvswdParts &r = rvswd.make();
  endpoint.add(r.wire);
  riscvDmIn = &riscvDm.make(r.port, 0);
  endpoint.add(*riscvDmIn);
}

void oepAddConsole() {
  static oep::Slot<oep::DmConsole> driver;   // the DM console framings (SDI / DMDATA / dmseq)
  static oep::Slot<oep::TargetConsoleStream> console;
  consoleIn = &console.make(rvswd->port, driver.make(rvswd->dm, rvswd->phy), 0);
  addConsolePlace(*consoleIn);
  loopWork.add([] { console->poll(); });
}

void oepAddGpio() {
  static oep::Slot<oep::FixtureGpio> gpio;
  endpoint.add(gpio.make(pins, 0, oep::owners::kGpio));
}

// The fixture UARTs' interrupts run on core 0 (FixtureUart::setInterruptCore): loop() runs on core 1, and its SWIO /
// RVSWD frames turn that core's interrupts off one frame at a time - SWIO up to SwioPhy::kIrqOffMaxUs, RVSWD about 55 us
// at attach's period and up to about 1.1 ms at the slowest max_speed. With the interrupt on core 1 and arduino-esp32's
// threshold of 120 bytes the RX FIFO had 8 bytes (40 us at 2000000) left before it overflowed (bench, both P4 jigs: a
// DUT's burst at 2000000 came back with about 125 bytes wrong, intermittent; 1500000 passed - the overflow is the
// reading from the code, not yet seen on the jigs). Core 0 has no interrupt-off section of the probe's
// (the capture's harvest and the SPI target's CS gate run there), and the threshold is oep::kUartRxFifoFull (96 bytes
// left, 480 us at 2000000). A flash write (settings, a DFU update) holds every non-IRAM interrupt off on both cores:
// what the FIFO cannot hold then is marked lost.
static constexpr int kUartCore = 0;
static_assert(kUartCore != CONFIG_ARDUINO_RUNNING_CORE, "the fixture UARTs' interrupts must not share loop()'s core");
void oepAddUart() {
  static oep::Slot<oep::FixtureUart> uart1, uart2;
  uart1In = &uart1.make(pins, Serial1, 0, oep::owners::kUart);
  uart2In = &uart2.make(pins, Serial2, 1, oep::owners::kUart2);
  uart1->setInterruptCore(kUartCore);   // before any plan begins them (applySaved)
  uart2->setInterruptCore(kUartCore);
  endpoint.add(*uart1);
  endpoint.add(*uart2);
  loopWork.add([] {
    uart1->poll();
    uart2->poll();
  });
}

void oepAddCapture() {
  static oep::Slot<oep::LogicCapture> capture;
  captureIn = &capture.make(endpoint, pins, 0);   // PARLIO RX; its lines are never driven
  endpoint.add(*capture);
  loopWork.add([] { capture->poll(); });
}

void oepAddI2c() {
  static oep::Slot<oep::P4I2cTarget> i2c;
  endpoint.add(i2c.make(pins));
  loopWork.add([] { i2c->service(); });
}

void oepAddSpi() {
  static oep::Slot<oep::P4SpiTarget> spi;
  endpoint.add(spi.make(pins));
  loopWork.add([] { spi->service(); });
}

// The at-boot slots' attach waits for the host to have configured the HS device (or BootGuard::kAttachGraceMs without
// one): their frames run on loop()'s core with its interrupts masked, as the RP2's did while its USB stack crashed.
static bool autoAttachReady() { return oep::BootGuard::attachReady(usbDevice.ready()); }

// the settings (core §13), saved in NVS: read first (its disable items' channels are never parked: core §13.7, disable
// before anything else; applySaved gives them back if the settings are not applied), its fn after the SPI device's
void oepAddSettings() {
  static oep::Slot<oep::Binds> binds;
  static oep::Slot<oep::Settings> config;
  oep::Settings &c = config.make(endpoint, binds.make());
  endpoint.setRawPorts(&*binds);
  c.load();
  pins.setDisabled(c.savedDisabled());
  settingsFn.add([] {
    config->usePassthroughs();   // a serial port's raw stream carries them (fixture §2.1)
    if (consoleIn) config->addPlace(rvswd->wire, *consoleIn);
    if (uart1In) config->addUart(*uart1In);
    if (uart2In) config->addUart(*uart2In);
    config->setPins(&pins);   // the idle item sets these pins' free state
    config->setUsbTransports(kUsbSetNames, kUsbSetCount);   // read at the next boot (setup, savedUsbTransports)
    loopWork.add([] { config->poll(); });
  });
  // Last, once every interface is added: the saved settings name fns, and are kept only for the same interface list
  // (applied before the analog and the group were added, they never matched it: unreadable after every reboot,
  // 0.0.11-0.0.16). core §13.7: disable and idle (outputs driven) before every other item; the at-boot slots' attach
  // comes on config.poll(), so a target powered through an output idle is up before it.
  lastly.add([] {
    // oep.wire.swd's hold after a close (oep-if-debug §2): its fn after the settings'
    if (swdIn) config->addHold(*swdWireIn, swdIn->hold, oep::checkSwdHold, swdIn);
    if (swioConsoleIn) config->addPlace(*swioWireIn, *swioConsoleIn);   // a slot on the one wire (CH32V00x): the second place
    if (oep::BootGuard::safe()) config->skipBootAttach();   // after BootGuard::kSafeAfter fast crash-boots
    config->setAttachGate(autoAttachReady);
    config->applySaved();
  });
}

// ADC1 (GPIO16-23) in continuous mode
static constexpr uint64_t kAdc1 = 0xffull << 16;
void oepAddAnalog() {
  static oep::Slot<oep::AnalogCapture> analog;
  analogIn = &analog.make(endpoint, kAdc1, 0);
  analog->setPins(&pins, oep::owners::kAnalog);   // its pads go analog: claimed against wires and settings
  endpoint.add(*analog);   // after the settings: the fns before it keep their numbers
  loopWork.add([] { analog->poll(); });
}

// oep.fixture.capture-group: the analog with the PARLIO capture (the tracks of those in)
void oepAddGroup() {
  static oep::Slot<oep::CaptureGroup> group;
  endpoint.add(group.make(endpoint, 0));
  if (captureIn) group->addTrack(*captureIn, *captureIn);
  if (analogIn) group->addTrack(*analogIn, *analogIn);
  loopWork.add([] { group->poll(); });
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
  endpoint.add(w.wire);   // after the group: the fns before it keep their numbers
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

// oep.wire.swd + oep.target.arm-adi on the GPIO registers (OepEspBitBang.h), any pair the host chooses (owners::kSwdWire)
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

// oep.fixture.i2c-controller / spi-controller: the lines on GPIOs (owners::kI2cController / kSpiController)
void oepAddI2cController() {
  static oep::Slot<oep::I2cController> i2c;
  endpoint.add(i2c.make(pins));
}

void oepAddSpiController() {
  static oep::Slot<oep::SpiController> spi;
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

void oepAddMemo() {
  static oep::Memo memo;
  if (memo.begin()) endpoint.add(memo);   // oep.probe.memo (oep-if-memo), on the LittleFS; left out when it does not mount
}

void oepAddLink() {
  static oep::Slot<oep::Link> link;
  endpoint.add(link.make(endpoint));   // oep.probe.link (oep-if-link)
}

// What this chip has (oep::kAvailable) and the distributed firmware's set (oep::kDefaults: all of them), the bits of
// OepFirmware.h
namespace oep {
inline constexpr uint64_t kAvailable = kBulk | kUsj | kHid | kCdc | kBridge | kDfu | kRvswd | kConsole | kGpio | kUart |
                                       kCapture | kI2cTarget | kSpiTarget | kSettings | kAnalog | kGroup | kSwio |
                                       kSwioConsole | kSwd | kI2cController | kSpiController | kPwm | kCounter | kMemo |
                                       kLink;
inline constexpr uint64_t kDefaults = kAvailable;
}  // namespace oep

// oep.probe.restart (oep-if-restart): the HS device detaches first so the host records an unplug rather than a device that went
// silent, waits oep::kRestartDetachMs for the host to see it (20 ms, as EspUsbDevice's own restarts wait, left the WeAct
// P4 failing its device descriptor request after the restart until a replug, bench 0.0.29-dev+3c0cd99), then
// esp_restart - within about 100 ms of the answer (Oep.h). restart_max_ms (its describe): the chip is in
// setup() after about 0.5 s (the ROM, the bootloader checking the app image of about 0.6 MB with rollback on, the
// PSRAM); then the host enumerates the HS device again - a composite of HID, vendor bulk, CDC and DFU, for which an OS
// binds four drivers (Windows about 1 s or more) - and the transport opens again before it confirms (USB-Serial/JTAG,
// whose own device may stay on the bus, is back no later). 3000 ms is about twice the slow end (an estimate from the
// boot path, to be measured on the bench).
static constexpr uint32_t kRestartMaxMs = 3000;
static void restartProbe() {
  oep::BootGuard::planned();
  usbDevice.end();   // off the bus: the host sees an unplug
  delay(oep::kRestartDetachMs);
  esp_restart();
}

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, modelName, id, oep::platformUnitId(id, sizeof id), 55,
                    oep::BootGuard::lastBoot());   // the firmware text: the version (and what ended the boot before)
  oep::describeChip(w);   // the MCU and its revision (a capture records what it was taken on)
  return w.ok() ? w.length() : 0;
}

template <uint64_t F>
void firmwareSetup(const oep::FirmwareConfig &cfg) {
  if (cfg.model) modelName = cfg.model;
  if (cfg.product) productName = cfg.product;
  oep::BootGuard::begin();   // loop() under the task watchdog, and the count of fast crash-boots
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §3)
  oep::platformUnitId(reinterpret_cast<uint8_t *>(serial_), sizeof serial_);   // the USB serial is the unit id (transports §3)
  EspUsbDeviceConfig usb;
  usb.vid = oep::reg::kUsbProjectVid;   // the project's VID:PID (registry usb, PID-USE.md)
  usb.pid = oep::reg::kUsbProjectPid;
  usb.manufacturer = "Open Embedded Probe";
  usb.product = productName;   // a name for people; no host identifies the probe by it
  usb.serialNumber = serial_;
  usb.controller = EspUsbController::HighSpeed;
  const int saved = oep::Settings::savedUsbTransports(kUsbSetNames, kUsbSetCount);
  usbSet = saved < 0 ? 0 : static_cast<uint8_t>(saved);
  // The transports in the describe's order, their functions made before the device starts
  if constexpr ((F & oep::kBulk) != 0) oepAddBulk();
  if constexpr ((F & oep::kUsj) != 0) oepAddUsj();
  if constexpr ((F & oep::kHid) != 0) oepAddHid();
  if constexpr ((F & oep::kCdc) != 0) oepAddCdc();
  if constexpr ((F & oep::kBridge) != 0) oepAddBridge();
  if constexpr ((F & oep::kDfu) != 0) oepAddDfu();
  if (!usbDevice.begin(usb) && usbSet != 0) {   // a set the device cannot start: the default this boot (value kept)
    fallBackUsbSet();
    usbDevice.begin(usb);
  }
  endpoint.setAnswerBuffer(frameBuffer(kAnswerBytes), kAnswerBytes);
  afterUsb.run();
  endpoint.setRestart(restartProbe, kRestartMaxMs);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  if constexpr ((F & oep::kSettings) != 0) oepAddSettings();   // the saved settings read first (their fn below at its place)
  // The fns in this order, whichever are in (a build's set numbers them; a host finds each by name)
  if constexpr ((F & oep::kRvswd) != 0) {
    oepAddRvswd();
    if constexpr ((F & oep::kConsole) != 0) oepAddConsole();
  }   // and the console's choice
  if constexpr ((F & oep::kGpio) != 0) oepAddGpio();
  if constexpr ((F & oep::kUart) != 0) oepAddUart();
  if constexpr ((F & oep::kCapture) != 0) oepAddCapture();
  if constexpr ((F & oep::kI2cTarget) != 0) oepAddI2c();
  if constexpr ((F & oep::kSpiTarget) != 0) oepAddSpi();
  settingsFn.run();
  if constexpr ((F & oep::kAnalog) != 0) oepAddAnalog();
  if constexpr ((F & oep::kGroup) != 0) oepAddGroup();
  if constexpr ((F & oep::kSwio) != 0) {
    oepAddSwio();
    if constexpr ((F & oep::kSwioConsole) != 0) oepAddSwioConsole();
  }   // and its console's choice
  if constexpr ((F & oep::kLink) != 0) oepAddLink();
  if constexpr ((F & oep::kSwd) != 0) oepAddSwd();   // after the ones before: they keep their numbers (the saved settings name fns)
  if constexpr ((F & oep::kI2cController) != 0) oepAddI2cController();
  if constexpr ((F & oep::kSpiController) != 0) oepAddSpiController();
  if constexpr ((F & oep::kPwm) != 0) oepAddPwm();
  if constexpr ((F & oep::kCounter) != 0) oepAddCounter();
  if constexpr ((F & oep::kMemo) != 0) oepAddMemo();   // last: it has no settings, and the fns before it keep their numbers
  // Every channel Hi-Z, no pull, before the first answer (core §8: the P4 boots some pins with a pull), but the saved
  // disable items' channels, which are never touched (core §13.7: disable before anything else; applySaved
  // gives them back if the settings are not applied).
  oep::platformParkMask(kChannels & ~pins.disabledMask() & ~ownPins);
  oep::firmware.runSetup(endpoint, pins);   // the sketch's own, after the build's (and before the saved settings)
  lastly.run();
}

inline void firmwareLoop() {
  oep::BootGuard::poll();
  endpoint.poll();
  loopWork.run();
  oep::firmware.runLoop();
}
