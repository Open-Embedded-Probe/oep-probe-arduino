// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepRp2Usb.h"

#if defined(ARDUINO_ARCH_RP2040) && !defined(USE_TINYUSB) && !defined(NO_USB)

#include <CoreMutex.h>
#include <USB.h>
#include <device/usbd.h>
#include <device/usbd_pvt.h>
#include <initializer_list>
#include <string.h>

#include "Oep.h"

namespace oep {
namespace rp2usb {
namespace {

UsbStream *gVendor = nullptr, *gHid = nullptr;   // the ones begun

// The HID report descriptor: OEP's usage page / usage, a 64-byte input and a 64-byte output report, no report ID
const uint8_t kHidReport[] = {
    0x06, reg::kUsbHidUsagePage & 0xff, reg::kUsbHidUsagePage >> 8,   // Usage Page (0xFF4F)
    0x09, reg::kUsbHidUsage,                                         // Usage (0x45)
    0xA1, 0x01,                                                      // Collection (Application)
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x40,            // 0..255, 8 bits, 64 of them
    0x09, 0x01, 0x81, 0x02,                                          // Input (Data, Var, Abs)
    0x09, 0x02, 0x91, 0x02,                                          // Output (Data, Var, Abs)
    0xC0,                                                            // End Collection
};
constexpr uint8_t kHidDescriptorType = 0x21, kHidReportType = 0x22;
constexpr uint8_t kHidSetIdle = 0x0A, kHidSetProtocol = 0x0B;

void drvInit() {}
bool drvDeinit() { return true; }
void drvReset(uint8_t) {
  if (gVendor) gVendor->reset();
  if (gHid) gHid->reset();
}
uint16_t drvOpen(uint8_t rhport, tusb_desc_interface_t const *desc, uint16_t max_len) {
  const uint8_t *d = reinterpret_cast<const uint8_t *>(desc);
  if (gVendor) {
    if (const uint16_t n = gVendor->open(rhport, d, max_len)) return n;
  }
  if (gHid) {
    if (const uint16_t n = gHid->open(rhport, d, max_len)) return n;
  }
  return 0;
}
bool drvControl(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
  if (gHid && request->bmRequestType_bit.recipient == TUSB_REQ_RCPT_INTERFACE &&
      tu_u16_low(request->wIndex) == gHid->interfaceNumber())
    return gHid->control(rhport, stage, request);
  return false;
}
bool drvXfer(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t bytes) {
  (void)result;
  for (UsbStream *s : {gVendor, gHid})
    if (s && s->owns(ep_addr)) {
      s->transferred(rhport, ep_addr, tu_edpt_dir(ep_addr) == TUSB_DIR_IN, bytes);
      return true;
    }
  return false;
}

const usbd_class_driver_t kDriver = {
    "OEP", drvInit, drvDeinit, drvReset, drvOpen, drvControl, drvXfer, nullptr,
};

}  // namespace

// ---- the stream side (loop()) ---------------------------------------------------------------------------------------

int UsbStream::available() {
  CoreMutex m(&USB.mutex, false);
  if (!m) return 0;
  tud_task();
  armOut(0);
  return static_cast<int>(rx_.used());
}

int UsbStream::read() {
  CoreMutex m(&USB.mutex, false);
  if (!m) return -1;
  tud_task();
  if (!rx_.used()) return -1;
  const int c = rx_.take();
  armOut(0);
  return c;
}

int UsbStream::peek() {
  CoreMutex m(&USB.mutex, false);
  if (!m || !rx_.used()) return -1;
  return rx_.b[rx_.tail % sizeof rx_.b];
}

int UsbStream::availableForWrite() {
  CoreMutex m(&USB.mutex, false);
  return m && opened_ ? static_cast<int>(tx_.room()) : 0;
}

// Into the ring, sent as it fills; a ring that stays full (nobody reads the interface) gives up after 200 ms
size_t UsbStream::write(const uint8_t *data, size_t length) {
  size_t done = 0;
  for (uint32_t since = millis(); done < length && millis() - since < 200;) {
    {
      CoreMutex m(&USB.mutex, false);
      if (!m) continue;
      if (!opened_) return done;
      while (done < length && tx_.room()) tx_.put(data[done++]);
      kick(0);
      tud_task();
    }
    if (done < length) delayMicroseconds(50);
  }
  return done;
}

void UsbStream::flush() {
  CoreMutex m(&USB.mutex, false);
  if (!m) return;
  kick(0);
  tud_task();
}

uint8_t UsbStream::interfaceNumber() const { return local_ == 0xff ? 0xff : USB.findInterfaceID(local_); }

// ---- the class driver's side (the stack's task, under USB.mutex) ----------------------------------------------------

void UsbStream::reset() {
  opened_ = in_busy_ = out_armed_ = zlp_due_ = false;
  rx_.head = rx_.tail = tx_.head = tx_.tail = 0;
}

void UsbStream::kick(uint8_t rhport) {
  if (!opened_ || in_busy_ || !tud_ready()) return;
  size_t n = fillIn();
  if (!n && !zlp_due_) return;
  if (!n) zlp_due_ = false;
  if (!usbd_edpt_claim(rhport, ep_in_)) return;
  in_busy_ = usbd_edpt_xfer(rhport, ep_in_, in_, static_cast<uint16_t>(n));
  if (!in_busy_) usbd_edpt_release(rhport, ep_in_);
}

void UsbStream::armOut(uint8_t rhport) {
  if (!opened_ || out_armed_ || rx_.room() < kPacket || !tud_ready()) return;
  if (!usbd_edpt_claim(rhport, ep_out_)) return;
  out_armed_ = usbd_edpt_xfer(rhport, ep_out_, out_, kPacket);
  if (!out_armed_) usbd_edpt_release(rhport, ep_out_);
}

void UsbStream::transferred(uint8_t rhport, uint8_t ep, bool in, uint32_t bytes) {
  (void)ep;
  if (in) {
    in_busy_ = false;
    kick(rhport);
  } else {
    out_armed_ = false;
    takeOut(bytes);
    armOut(rhport);
  }
}

void UsbStream::fillDescriptor(int itf, uint8_t *dst, int len, void *self) {
  UsbStream *s = static_cast<UsbStream *>(self);
  memcpy(dst, s->descriptor_, len);
  dst[2] = static_cast<uint8_t>(itf);   // bInterfaceNumber
}

void UsbStream::registerWith(uint8_t interface_len, uint8_t *descriptor, int ordering) {
  (void)descriptor;
  descriptor_len_ = interface_len;
  local_ = USB.registerInterface(1, fillDescriptor, this, descriptor_len_, ordering, 0);
}

// ---- vendor bulk ----------------------------------------------------------------------------------------------------

void VendorStream::begin() {
  if (local_ != 0xff) return;
  ep_out_ = USB.registerEndpointOut();
  ep_in_ = USB.registerEndpointIn();
  const uint8_t d[] = {
      9, TUSB_DESC_INTERFACE, 0, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, reg::kUsbVendorBulkSubclass,
      reg::kUsbVendorBulkProtocol, 0,
      7, TUSB_DESC_ENDPOINT, ep_out_, TUSB_XFER_BULK, kPacket, 0, 0,
      7, TUSB_DESC_ENDPOINT, ep_in_, TUSB_XFER_BULK, kPacket, 0, 0,
  };
  memcpy(descriptor_, d, sizeof d);
  gVendor = this;
  registerWith(sizeof d, descriptor_, 20);
}

uint16_t VendorStream::open(uint8_t rhport, const uint8_t *desc, uint16_t max_len) {
  const auto *itf = reinterpret_cast<const tusb_desc_interface_t *>(desc);
  if (itf->bInterfaceClass != TUSB_CLASS_VENDOR_SPECIFIC || itf->bInterfaceSubClass != reg::kUsbVendorBulkSubclass ||
      itf->bInterfaceProtocol != reg::kUsbVendorBulkProtocol)
    return 0;
  const uint16_t len = 9 + 7 + 7;
  if (max_len < len) return 0;
  uint8_t out = 0, in = 0;
  if (!usbd_open_edpt_pair(rhport, desc + 9, 2, TUSB_XFER_BULK, &out, &in)) return 0;
  ep_out_ = out;
  ep_in_ = in;
  reset();
  opened_ = true;
  armOut(rhport);
  return len;
}

// As much of the ring as one transfer takes; one that ends on a whole packet is closed by a zero-length one after it
// when nothing follows (the host's read of a larger size ends there)
size_t VendorStream::fillIn() {
  size_t n = 0;
  while (n < sizeof in_ && tx_.used()) in_[n++] = tx_.take();
  if (n) zlp_due_ = n % kPacket == 0;
  return n;
}

void VendorStream::takeOut(uint32_t bytes) {
  for (uint32_t i = 0; i < bytes && rx_.room(); ++i) rx_.put(out_[i]);
}

// ---- HID ------------------------------------------------------------------------------------------------------------

void HidStream::begin() {
  if (local_ != 0xff) return;
  ep_out_ = USB.registerEndpointOut();
  ep_in_ = USB.registerEndpointIn();
  const uint8_t d[] = {
      9, TUSB_DESC_INTERFACE, 0, 0, 2, TUSB_CLASS_HID, 0, 0, 0,
      9, kHidDescriptorType, 0x11, 0x01, 0, 1, kHidReportType, sizeof kHidReport, 0,
      7, TUSB_DESC_ENDPOINT, ep_in_, TUSB_XFER_INTERRUPT, kPacket, 0, 1,
      7, TUSB_DESC_ENDPOINT, ep_out_, TUSB_XFER_INTERRUPT, kPacket, 0, 1,
  };
  memcpy(descriptor_, d, sizeof d);
  gHid = this;
  registerWith(sizeof d, descriptor_, 30);
}

uint16_t HidStream::open(uint8_t rhport, const uint8_t *desc, uint16_t max_len) {
  const auto *itf = reinterpret_cast<const tusb_desc_interface_t *>(desc);
  if (itf->bInterfaceClass != TUSB_CLASS_HID || itf->bInterfaceNumber != interfaceNumber()) return 0;
  const uint16_t len = 9 + 9 + 7 + 7;
  if (max_len < len) return 0;
  uint8_t out = 0, in = 0;
  if (!usbd_open_edpt_pair(rhport, desc + 18, 2, TUSB_XFER_INTERRUPT, &out, &in)) return 0;
  ep_out_ = out;
  ep_in_ = in;
  reset();
  opened_ = true;
  armOut(rhport);
  return len;
}

bool HidStream::control(uint8_t rhport, uint8_t stage, const void *request) {
  const auto *r = static_cast<const tusb_control_request_t *>(request);
  if (stage != CONTROL_STAGE_SETUP) return true;
  if (r->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD && r->bRequest == TUSB_REQ_GET_DESCRIPTOR) {
    const uint8_t type = tu_u16_high(r->wValue);
    if (type == kHidReportType) return tud_control_xfer(rhport, r, const_cast<uint8_t *>(kHidReport), sizeof kHidReport);
    if (type == kHidDescriptorType) return tud_control_xfer(rhport, r, descriptor_ + 9, 9);
    return false;
  }
  if (r->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS && (r->bRequest == kHidSetIdle || r->bRequest == kHidSetProtocol))
    return tud_control_status(rhport, r);
  return false;
}

// One report: count (u16 LE), up to 62 bytes of the stream, the rest 0
size_t HidStream::fillIn() {
  size_t n = 0;
  while (n < kPacket - 2 && tx_.used()) in_[2 + n++] = tx_.take();
  if (!n) return 0;
  in_[0] = static_cast<uint8_t>(n);
  in_[1] = 0;
  memset(in_ + 2 + n, 0, kPacket - 2 - n);
  return kPacket;
}

// One report: its count's bytes (a count over what it carries: the report dropped whole, transports §1)
void HidStream::takeOut(uint32_t bytes) {
  if (bytes < 2) return;
  const size_t n = out_[0] | (out_[1] << 8);
  if (n > bytes - 2) return;
  for (size_t i = 0; i < n && rx_.room(); ++i) rx_.put(out_[2 + i]);
}

}  // namespace rp2usb
}  // namespace oep

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
  *driver_count = 1;
  return &oep::rp2usb::kDriver;
}

#endif
