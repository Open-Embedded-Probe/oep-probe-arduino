// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Streams over the P4 HS port's EspUsbDevice functions: the vendor HID (count(u16) report framing, report ID first on
// output) and a CDC port that never waits long (a serial port: frames and raw bytes, oep-transports §4).
#pragma once
#include <OepHid.h>
#include <EspUsbDevice.h>
#include <OepFrame.h>

class CdcStream final : public Stream {
 public:
  explicit CdcStream(EspUsbDeviceCdcSerial &p) : p_(p) { p_.setTxTimeoutMs(0); }   // write() waits here, not in the port
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
  // The ring's real free room: the endpoint writes raw bytes only into it, so they never wait (a port that is open but
  // not read used to take 200 ms per 64-byte chunk, and the loop stopped for seconds).
  int availableForWrite() override { return p_.connected() ? static_cast<int>(p_.writeAvailable()) : 0; }
 private:
  EspUsbDeviceCdcSerial &p_;
};

class HidStream final : public Stream {
 public:
  static constexpr size_t kReport = 511, kPayload = kReport - oep::kHidHead;
  explicit HidStream(EspUsbDeviceHidVendor &h) : h_(h) {
    h_.onOutputReport([this](const EspUsbDeviceHidReport &r) {
      // The host puts the report ID (6) first, as HID wants when the descriptor declares one. EspUsbDevice strips it
      // only when several HID classes are merged; with this single HID it arrives as data[0] of a kReport + 1 report.
      const uint8_t *d = r.data;
      size_t len = r.length;
      if (r.reportId != 0 && r.reportId != ESP_USB_DEVICE_HID_REPORT_ID_VENDOR) return;   // another ID: dropped (rule 5)
      if (d && r.reportId == 0 && len == kReport + 1) {
        if (d[0] != ESP_USB_DEVICE_HID_REPORT_ID_VENDOR) return;   // another ID: dropped, not a bad count (rule 5)
        ++d;
        --len;
      }
      in_.take(d, len);   // count, first and the stream's bytes (transports §1, rules 6 and 7)
    });
  }
  oep::HidIn &in() { return in_; }   // for Endpoint::useHidInput
  int available() override { return static_cast<int>(in_.available()); }
  int read() override { return in_.read(); }
  size_t readBytes(char *b, size_t n) override { return in_.read(reinterpret_cast<uint8_t *>(b), n); }
  int peek() override { return in_.peek(); }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    for (size_t i = 0; i < n; ++i) { payload_[fill_++] = b[i]; if (fill_ == kPayload) flush(); }
    return n;
  }
  void flush() override {
    if (!fill_) return;
    oep::hidPackReport(tx_, kPayload, payload_, fill_, out_);   // first: where its first frame starts
    // a report still in flight makes sendInput return false: wait for it - but once a report could not go (nobody
    // reads HID), later ones do not wait again until one goes. A report dropped here costs the host the frames it
    // cut, not the stream: the next report's first shows where to read on (transports §1)
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
  oep::HidIn in_{rx_, sizeof rx_};
  oep::HidFrameStarts out_;   // the frames written: each report's first
  uint8_t payload_[kPayload];
  uint8_t tx_[kReport];
  size_t fill_ = 0;
  bool stuck_ = false;   // the last report could not go
};
