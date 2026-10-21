// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The RP2's USB vendor bulk and HID transports (oep-transports §1, §3) on arduino-pico's own USB stack (usbstack=picosdk:
// the pico-sdk's TinyUSB, with CDC as Serial): neither class is in that stack's build (CFG_TUD_VENDOR 0, and its HID's
// report descriptor is the Keyboard / Mouse libraries' composite), so both are this library's TinyUSB class drivers
// (usbd_app_driver_get_cb, offered before the stack's own: arduino-pico defines none unless built for picotool upload),
// their interfaces in arduino-pico's descriptor (USB.registerInterface, before USB.connect()).
//   Vendor bulk: class 0xFF, subclass 'O', protocol 'E'; two 64-byte bulk endpoints; the length-prefixed stream as it
//   is, a burst that ends on a whole packet closed by a zero-length one (the host's read ends there).
//   HID: usage page 0xFF4F, usage 0x45, no report ID; 64-byte input and output reports on interrupt endpoints, each
//   count (u16 LE) and that many bytes of the stream, the rest 0.
// The stack's task runs from loop() (every call here) and from arduino-pico's USB interrupt, both under USB.mutex: the
// buffers here are touched only under it. Linux / macOS reach the vendor interface through libusb; Windows would want
// MS OS 2.0 descriptors for WinUSB, which arduino-pico's descriptor does not carry (HID needs none).
#pragma once

#if defined(ARDUINO_ARCH_RP2040) && !defined(USE_TINYUSB) && !defined(NO_USB)

#include <Arduino.h>

namespace oep {
namespace rp2usb {

// A byte ring (N a power of two)
template <size_t N>
struct Ring {
  uint8_t b[N];
  size_t head = 0, tail = 0;
  size_t used() const { return head - tail; }
  size_t room() const { return N - used(); }
  void put(uint8_t c) { b[head++ % N] = c; }
  uint8_t take() { return b[tail++ % N]; }
};

class UsbStream : public Stream {
 public:
  int available() override;
  int read() override;
  int peek() override;
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *data, size_t length) override;
  void flush() override;
  int availableForWrite() override;
  uint8_t interfaceNumber() const;   // after USB.connect()

  // the class driver's side (under USB.mutex, in the stack's task)
  virtual uint16_t open(uint8_t rhport, const uint8_t *desc, uint16_t max_len) = 0;
  virtual bool control(uint8_t rhport, uint8_t stage, const void *request) { (void)rhport; (void)stage; (void)request; return false; }
  void transferred(uint8_t rhport, uint8_t ep, bool in, uint32_t bytes);
  void reset();
  bool owns(uint8_t ep) const { return opened_ && (ep == ep_in_ || ep == ep_out_); }
  uint8_t localId() const { return local_; }

 protected:
  static constexpr size_t kPacket = 64;
  virtual size_t fillIn() = 0;              // the next IN transfer into in_ (0: nothing)
  virtual void takeOut(uint32_t bytes) = 0;   // an OUT transfer's bytes in out_
  void kick(uint8_t rhport);                 // an IN transfer when none is under way
  void armOut(uint8_t rhport);               // the next OUT transfer, when the ring has room for it
  void registerWith(uint8_t interface_len, uint8_t *descriptor, int ordering);
  Ring<2048> rx_, tx_;
  uint8_t ep_in_ = 0, ep_out_ = 0, local_ = 0xff;
  bool opened_ = false, in_busy_ = false, out_armed_ = false, zlp_due_ = false;
  alignas(4) uint8_t in_[512];
  alignas(4) uint8_t out_[kPacket];
  uint8_t descriptor_[32];
  uint8_t descriptor_len_ = 0;
  static void fillDescriptor(int itf, uint8_t *dst, int len, void *self);
};

// The vendor bulk interface (call begin() before USB.connect())
class VendorStream final : public UsbStream {
 public:
  void begin();
  uint16_t open(uint8_t rhport, const uint8_t *desc, uint16_t max_len) override;

 protected:
  size_t fillIn() override;
  void takeOut(uint32_t bytes) override;
  size_t last_in_ = 0;
};

// The HID interface (call begin() before USB.connect())
class HidStream final : public UsbStream {
 public:
  void begin();
  uint16_t open(uint8_t rhport, const uint8_t *desc, uint16_t max_len) override;
  bool control(uint8_t rhport, uint8_t stage, const void *request) override;

 protected:
  size_t fillIn() override;
  void takeOut(uint32_t bytes) override;
};

}  // namespace rp2usb
}  // namespace oep

#endif
