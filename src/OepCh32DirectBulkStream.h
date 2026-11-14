// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Stream + zero-copy transport over an ArduinoCore-CH32RV vendor bulk interface (CH32RVUsbDeviceVendor): its borrowed
// IN transfers (writeDirect) and its packets received in callback mode, as OepDirectBulkStream.h on the ESP32-P4. The
// library takes one direct transfer at a time and every IN goes through it: results (the endpoint's writes, framed into
// one of two buffers and sent on flush) go before queued data - but never inside a frame: after a buffer queued open
// (queueDataOpen, its last frame's rest in the next one) they wait for one that ends on a frame - and the next transfer
// is armed from the completion callback. A result ends the host's transfer (a zero-length packet after a whole number of packets); an interface's data
// does not. The CH32 stack runs from loop() and delay() (ch32rv_usb_task), so everything here runs in that one context:
// queueData from loop(), the completions from the stack's deliverEvents. On the USBSS (H41x) the DMA reads the buffer
// itself when it starts 4-byte aligned (a transfer that does not goes a packet at a time through a bounce buffer).
#pragma once

#include <Arduino.h>

#if defined(ARDUINO_ARCH_CH32RV) || defined(OEP_HOST_FAKE_CH32_USB)
#include <CH32RVUsbDevice.h>

#include "Oep.h"

#define OEP_HAS_CH32_DIRECT_BULK_STREAM 1

namespace oep {

class Ch32DirectBulkStream final : public Stream, public DirectTransport {
 public:
  // kResultBytes: one buffer holds the largest frame whole (16 KiB + its length); kRxBytes: two whole max_frame requests
  static constexpr size_t kResultBytes = 16384 + 128, kRxBytes = 32768, kQueue = 32;
  static constexpr uint32_t kWaitMs = 2000;   // a result waits this long for a buffer the host has taken; then dropped whole

  explicit Ch32DirectBulkStream(CH32RVUsbDeviceVendor &vendor) : vendor_(vendor) {}

  // Before the device's begin(): the receive mode and the callbacks. false: borrowed transfers unsupported.
  bool begin() {
    if (!CH32RVUsbDeviceVendor::directWriteSupported()) return false;
    CH32RVUsbBulkStreamConfig c = vendor_.configuration();
    c.rxMode = CH32RVUsbBulkRxMode::Callback;
    if (!vendor_.configure(c)) return false;
    vendor_.onRxData([this](const uint8_t *data, size_t length) { received(data, length); });
    vendor_.onDirectTxComplete([this](const CH32RVUsbTransferResult &) { completed(); });
    return true;
  }

  // Stream (the endpoint's side)
  int available() override { return static_cast<int>((rx_head_ - rx_tail_) & (kRxBytes - 1)); }
  int read() override {
    if (rx_head_ == rx_tail_) return -1;
    const uint8_t c = rx_[rx_tail_];
    rx_tail_ = (rx_tail_ + 1) & (kRxBytes - 1);
    return c;
  }
  size_t readBytes(char *out, size_t length) {   // (not virtual in the core's Stream: the endpoint reads byte by byte there)
    size_t done = 0;
    while (done < length && rx_head_ != rx_tail_) {
      const size_t tail = rx_tail_, head = rx_head_;
      const size_t run = head > tail ? head - tail : kRxBytes - tail;
      const size_t n = run < length - done ? run : length - done;
      memcpy(out + done, rx_ + tail, n);
      rx_tail_ = (tail + n) & (kRxBytes - 1);
      done += n;
    }
    return done;
  }
  int peek() override { return rx_head_ == rx_tail_ ? -1 : rx_[rx_tail_]; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  // Frame by frame (length(u16) then the message): a frame goes into one buffer whole, or is dropped whole when no
  // buffer frees up within kWaitMs - never half a frame. Every byte is accepted.
  size_t write(const uint8_t *data, size_t size) override {
    size_t at = 0;
    while (at < size) {
      if (frame_left_ == 0 && drop_left_ == 0) {
        prefix_[prefix_have_++] = data[at++];
        if (prefix_have_ < 2) continue;
        prefix_have_ = 0;
        const size_t body = prefix_[0] | (prefix_[1] << 8);
        if (!room(2 + body)) { drop_left_ = body; ++dropped_; continue; }
        memcpy(result_[current_] + length_[current_], prefix_, 2);
        length_[current_] += 2;
        frame_left_ = body;
        continue;
      }
      size_t n = size - at;
      const size_t left = drop_left_ ? drop_left_ : frame_left_;
      if (n > left) n = left;
      if (drop_left_) {
        drop_left_ -= n;
      } else {
        memcpy(result_[current_] + length_[current_], data + at, n);
        length_[current_] += n;
        frame_left_ -= n;
      }
      at += n;
    }
    return size;
  }
  uint32_t dropped() const { return dropped_; }
  void flush() override {
    const int i = current_;
    const size_t n = length_[i];
    if (n == 0) return;
    busy_[i] = true;
    current_ ^= 1;
    queue(true, {result_[i], n, resultSent, this, true, false});
    kick();
  }
  int availableForWrite() override {
    return busy_[current_] ? 0 : static_cast<int>(kResultBytes - 64 - length_[current_]);
  }

  // DirectTransport (an interface's data, from loop())
  bool queueData(const uint8_t *buffer, size_t length, Done done, void *context) override {
    if (!queue(false, {buffer, length, done, context, false, false})) return false;
    kick();
    return true;
  }
  bool queueDataOpen(const uint8_t *buffer, size_t length, Done done, void *context) override {
    if (!queue(false, {buffer, length, done, context, false, true})) return false;
    kick();
    return true;
  }
  bool resultsWaiting() const override { return results_head_ != results_tail_; }
  size_t queued() const override { return (data_head_ - data_tail_) + (in_flight_ ? 1 : 0); }
  bool mounted() const { return vendor_.mounted(); }
  uint32_t refused() const { return refused_; }

 private:
  // ends: the host's transfer; open: the buffer ends inside a frame (results wait until a buffer that does not)
  struct Entry { const uint8_t *buffer; size_t length; Done done; void *context; bool ends, open; };
  CH32RVUsbDeviceVendor &vendor_;
  uint8_t result_[2][kResultBytes] __attribute__((aligned(4)));
  size_t length_[2] = {};
  bool busy_[2] = {};
  int current_ = 0;
  uint8_t rx_[kRxBytes];
  size_t rx_head_ = 0, rx_tail_ = 0;
  Entry results_[kQueue], data_[kQueue];
  size_t results_head_ = 0, results_tail_ = 0, data_head_ = 0, data_tail_ = 0;
  Entry sending_ = {};
  bool in_flight_ = false, kicking_ = false, again_ = false, stalled_ = false;
  bool open_ = false;   // the data handed on last ended inside a frame
  uint32_t refused_ = 0, dropped_ = 0;
  size_t frame_left_ = 0, drop_left_ = 0;
  uint8_t prefix_[2] = {};
  uint8_t prefix_have_ = 0;

  static void resultSent(void *context, const uint8_t *buffer) {
    auto *self = static_cast<Ch32DirectBulkStream *>(context);
    for (int i = 0; i < 2; ++i)
      if (buffer == self->result_[i]) { self->length_[i] = 0; self->busy_[i] = false; }
    self->stalled_ = false;
  }
  // The stack runs meanwhile (delay(0) runs ch32rv_usb_task), so a completion can free the buffer.
  bool waitWritable() {
    for (uint32_t start = millis(); busy_[current_];) {
      if (!vendor_.mounted() || stalled_ || millis() - start > kWaitMs) { stalled_ = true; return false; }
      delay(0);
    }
    return true;
  }
  bool room(size_t bytes) {
    if (bytes > kResultBytes - 64) return false;
    if (!waitWritable()) return false;
    if (length_[current_] + bytes > kResultBytes - 64) {
      flush();
      if (!waitWritable()) return false;
    }
    return true;
  }
  bool queue(bool result, const Entry &e) {
    Entry *q = result ? results_ : data_;
    size_t &head = result ? results_head_ : data_head_, &tail = result ? results_tail_ : data_tail_;
    if (head - tail >= kQueue) return false;
    q[head % kQueue] = e;
    ++head;
    return true;
  }
  // The next transfer to the library when none is out: results first. A completion during a writeDirect (it runs no
  // stack, but a nested kick is cheap to guard) asks the running kick to look again.
  void kick() {
    if (kicking_) { again_ = true; return; }
    kicking_ = true;
    do {
      again_ = false;
      while (!in_flight_) {
        const bool result = results_head_ != results_tail_ && !open_;
        if (!result && data_head_ == data_tail_) break;
        const Entry e = result ? results_[results_tail_ % kQueue] : data_[data_tail_ % kQueue];
        sending_ = e;
        in_flight_ = true;
        const CH32RVUsbTransferSubmission sub = vendor_.writeDirect(
            e.buffer, e.length,
            e.ends ? CH32RVUsbTransferTermination::EndWithShortPacket : CH32RVUsbTransferTermination::Continue);
        if (!sub && sub.error == CH32RVUsbTransferError::Busy) { in_flight_ = false; break; }   // goes on the completion
        if (result) ++results_tail_; else ++data_tail_;
        if (sub) { if (!result) open_ = e.open; break; }
        in_flight_ = false;   // not taken: not mounted, or a contract mistake; the buffer is given back unsent
        ++refused_;
        if (e.done) e.done(e.context, e.buffer);
      }
    } while (again_);
    kicking_ = false;
  }
  void completed() {   // the stack: the buffer out is back (sent, or cancelled by a reset or the device's end)
    const Entry e = sending_;
    in_flight_ = false;
    if (e.done) e.done(e.context, e.buffer);
    kick();
  }
  void received(const uint8_t *data, size_t length) {   // a full ring drops (requests are small)
    for (size_t i = 0; i < length; ++i) {
      const size_t next = (rx_head_ + 1) & (kRxBytes - 1);
      if (next == rx_tail_) return;
      rx_[rx_head_] = data[i];
      rx_head_ = next;
    }
  }
};

}  // namespace oep

#endif
