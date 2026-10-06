// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Streams over the P4 HS port's EspUsbDevice functions: the vendor HID (count(u16) report framing, report ID first on
// output) and a CDC port that never waits long (a serial port: frames and raw bytes, oep-core §3.4).
#pragma once
#include <EspUsbDevice.h>
#include <class/cdc/cdc_device.h>   // EspUsbDevice's TinyUSB: the CDC FIFO's free room

class CdcStream final : public Stream {
 public:
  explicit CdcStream(EspUsbDeviceCdcSerial &p) : p_(p) {}
  int available() override { return p_.available(); }
  int read() override { return p_.read(); }
  size_t readBytes(char *b, size_t n) override { return p_.read(reinterpret_cast<uint8_t *>(b), n); }
  int peek() override { return -1; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  // A frame waits a little for room while the port is open; nothing waits while it is closed (nobody reads it).
  size_t write(const uint8_t *b, size_t n) override {
    if (!p_.connected()) return 0;
    size_t done = 0;
    for (uint32_t t = millis(); done < n && millis() - t < 200;) { const size_t w = p_.write(b + done, n - done); done += w; if (!w) delay(0); }
    return done;
  }
  void flush() override { p_.flush(); }
  // The FIFO's real free room: the endpoint writes raw bytes only into it, so they never wait (a port that is open but
  // not read used to take 200 ms per 64-byte chunk, and the loop stopped for seconds).
  int availableForWrite() override { return p_.connected() ? static_cast<int>(tud_cdc_n_write_available(p_.port())) : 0; }
 private:
  EspUsbDeviceCdcSerial &p_;
};

class HidStream final : public Stream {
 public:
  static constexpr size_t kReport = 511, kPayload = kReport - 2;
  explicit HidStream(EspUsbDeviceHidVendor &h) : h_(h) {
    h_.onOutputReport([this](const EspUsbDeviceHidReport &r) {
      // The host puts the report ID (6) first, as HID wants when the descriptor declares one. EspUsbDevice strips it
      // only when several HID classes are merged; with this single HID it arrives as data[0] of a kReport + 1 report.
      const uint8_t *d = r.data;
      size_t len = r.length;
      if (d && r.reportId == 0 && len == kReport + 1 && d[0] == ESP_USB_DEVICE_HID_REPORT_ID_VENDOR) { ++d; --len; }
      if (len < 2 || !d) return;
      size_t n = d[0] | (d[1] << 8);
      if (n > len - 2) return;   // a count over what the report carries: the report is discarded whole (core §3.1)
      for (size_t i = 0; i < n; ++i) { const size_t next = (head_ + 1) % sizeof rx_; if (next == tail_) return; rx_[head_] = d[2 + i]; head_ = next; }
    });
  }
  int available() override { return (head_ + sizeof rx_ - tail_) % sizeof rx_; }
  int read() override { if (head_ == tail_) return -1; const uint8_t c = rx_[tail_]; tail_ = (tail_ + 1) % sizeof rx_; return c; }
  size_t readBytes(char *b, size_t n) override {   // in at most two copies
    size_t done = 0;
    while (done < n && head_ != tail_) {
      const size_t t = tail_, h = head_;
      const size_t run = h > t ? h - t : sizeof rx_ - t;
      const size_t k = run < n - done ? run : n - done;
      memcpy(b + done, rx_ + t, k);
      tail_ = (t + k) % sizeof rx_;
      done += k;
    }
    return done;
  }
  int peek() override { return head_ == tail_ ? -1 : rx_[tail_]; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    for (size_t i = 0; i < n; ++i) { tx_[2 + fill_++] = b[i]; if (fill_ == kPayload) flush(); }
    return n;
  }
  void flush() override {
    if (!fill_) return;
    tx_[0] = fill_ & 0xff; tx_[1] = fill_ >> 8;
    memset(tx_ + 2 + fill_, 0, kPayload - fill_);
    // a report still in flight makes sendInput return false: wait for it (dropping the report broke the framing) - but
    // once a report could not go (nobody reads HID), later ones do not wait again until one goes
    bool sent = h_.sendInput(tx_, kReport, stuck_ ? 0 : 200);
    for (uint32_t t = millis(); !sent && !stuck_ && millis() - t < 500;) {
      delayMicroseconds(50);
      sent = h_.sendInput(tx_, kReport, 200);
    }
    stuck_ = !sent;
    fill_ = 0;
  }
  int availableForWrite() override { return 4096; }
 private:
  EspUsbDeviceHidVendor &h_;
  uint8_t rx_[8192];
  volatile size_t head_ = 0, tail_ = 0;
  uint8_t tx_[kReport];
  size_t fill_ = 0;
  bool stuck_ = false;   // the last report could not go
};
