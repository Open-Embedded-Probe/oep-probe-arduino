// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Stream + zero-copy transport over an EspUsbDevice (3.x) vendor bulk interface: its borrowed IN transfers
// (writeDirect) and its packets received in callback mode. One IN transfer is in flight at a time; the next is armed
// from the completion callback, so a stream of full buffers runs gapless (wch-protocols E110 / E116 / E117). Results
// are written into one of two 64-aligned buffers and sent on flush(); they go before queued data. A result ends the
// host's transfer (a zero-length packet after a whole number of packets); an interface's data does not.
#pragma once

#include <Arduino.h>

#if __has_include(<EspUsbDevice.h>)
#include <EspUsbDevice.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>

#include "Oep.h"

#define OEP_HAS_DIRECT_BULK_STREAM 1

namespace oep {

class DirectBulkStream final : public Stream, public DirectTransport {
 public:
  // kRxBytes: two whole max_frame requests; the USB worker can land a frame faster than loop() reads it byte by byte
  // kResultBytes: one buffer holds the largest frame whole (16 KiB + its length)
  static constexpr size_t kResultBytes = 16384 + 128, kRxBytes = 32768, kQueue = 32;
  // A result waits this long for a buffer the host has taken (IN); after it, the result is dropped whole.
  static constexpr uint32_t kWaitMs = 2000;

  explicit DirectBulkStream(EspUsbDeviceVendor &vendor) : vendor_(vendor) {}

  // Before the device's begin() (its callbacks are registered while it is stopped): the vendor's receive mode, the
  // buffers and the callbacks. false: borrowed transfers unsupported, or no memory.
  bool begin() {
    if (!EspUsbDeviceVendor::directWriteSupported()) return false;
    EspUsbVendorConfig c = vendor_.configuration();
    c.rxMode = EspUsbBulkRxMode::Callback;   // each packet straight into rx_ (callback mode has no ring of its own)
    c.rxCapacity = 0;
    if (vendor_.configure(c) != ESP_OK) return false;
    for (int i = 0; i < 2; ++i) {
      result_[i] = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, kResultBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
      if (!result_[i]) return false;
    }
    return vendor_.onRxData([this](const uint8_t *data, size_t length) { received(data, length); }) == ESP_OK &&
           vendor_.onDirectTxComplete([this](const EspUsbTransferResult &) { completed(); }) == ESP_OK;
  }

  // Stream (the endpoint's side)
  int available() override { return static_cast<int>((rx_head_ - rx_tail_) & (kRxBytes - 1)); }
  int read() override {
    if (rx_head_ == rx_tail_) return -1;
    const uint8_t c = rx_[rx_tail_];
    rx_tail_ = (rx_tail_ + 1) & (kRxBytes - 1);
    return c;
  }
  // What is there, in at most two copies (the ring's end, then its start): the endpoint reads whole chunks this way
  size_t readBytes(char *out, size_t length) override {
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
  // The endpoint writes length(u16) then the message: this stream follows those frame boundaries so a frame goes into
  // one buffer whole, or is dropped whole (dropped()) when no buffer frees up within kWaitMs - never half a frame,
  // which broke the host's framing and cost it a timeout (2026-09-30, the X035 jig over usbip: 200 ms without the
  // host taking IN dropped the rest of a result). Every byte is accepted (the caller never sees a short write).
  size_t write(const uint8_t *data, size_t size) override {
    size_t at = 0;
    while (at < size) {
      if (frame_left_ == 0 && drop_left_ == 0) {          // a frame starts: its length first
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
      const size_t n = min(size - at, drop_left_ ? drop_left_ : frame_left_);
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
  uint32_t dropped() const { return dropped_; }   // results dropped whole: no buffer freed up within kWaitMs
  void flush() override {
    const int i = current_;
    const size_t n = length_[i];
    if (n == 0) return;
    busy_[i] = true;
    current_ ^= 1;
    queue(true, {result_[i], n, resultSent, this, true});
    kick();
  }
  int availableForWrite() override {
    return busy_[current_] ? 0 : static_cast<int>(kResultBytes - 64 - length_[current_]);
  }

  // DirectTransport (an interface's data, from any task)
  bool queueData(const uint8_t *buffer, size_t length, Done done, void *context) override {
    if (!queue(false, {buffer, length, done, context, false})) return false;
    kick();
    return true;
  }
  size_t queued() const override { return (data_head_ - data_tail_) + (in_flight_ ? 1 : 0); }
  bool mounted() const { return vendor_.mounted(); }
  uint32_t refused() const { return refused_; }

 private:
  struct Entry { const uint8_t *buffer; size_t length; Done done; void *context; bool ends; };   // ends: the host's transfer
  EspUsbDeviceVendor &vendor_;
  uint8_t *result_[2] = {};
  volatile size_t length_[2] = {};
  volatile bool busy_[2] = {};
  volatile int current_ = 0;
  uint8_t rx_[kRxBytes];
  volatile size_t rx_head_ = 0, rx_tail_ = 0;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  Entry results_[kQueue], data_[kQueue];
  volatile size_t results_head_ = 0, results_tail_ = 0, data_head_ = 0, data_tail_ = 0;
  volatile bool in_flight_ = false;
  Entry sending_ = {};
  volatile uint32_t refused_ = 0;

  static void resultSent(void *context, const uint8_t *buffer) {
    auto *self = static_cast<DirectBulkStream *>(context);
    for (int i = 0; i < 2; ++i)
      if (buffer == self->result_[i]) { self->length_[i] = 0; self->busy_[i] = false; }
    self->stalled_ = false;   // the host takes IN again
  }
  // Once a frame was dropped (the host stopped taking IN), later ones are dropped at once until a transfer completes:
  // a host that went away costs one wait, not kWaitMs for every result (the loop would crawl).
  bool waitWritable() {
    for (uint32_t start = millis(); busy_[current_];) {
      if (!vendor_.mounted() || stalled_ || millis() - start > kWaitMs) { stalled_ = true; return false; }
      delayMicroseconds(20);
    }
    return true;
  }
  // Room for a whole frame of `bytes` in the current buffer: the current one is sent first if the frame does not fit,
  // and a buffer still in flight is waited for (kWaitMs). false: no room (the frame is dropped whole).
  bool room(size_t bytes) {
    if (bytes > kResultBytes - 64) return false;
    if (!waitWritable()) return false;
    if (length_[current_] + bytes > kResultBytes - 64) {
      flush();
      if (!waitWritable()) return false;
    }
    return true;
  }
  size_t frame_left_ = 0, drop_left_ = 0;   // bytes of the frame being written, or being dropped
  uint8_t prefix_[2] = {};
  uint8_t prefix_have_ = 0;
  uint32_t dropped_ = 0;
  volatile bool stalled_ = false;   // a frame was dropped and no transfer completed since
  bool queue(bool result, const Entry &e) {
    portENTER_CRITICAL(&mux_);
    Entry *q = result ? results_ : data_;
    volatile size_t &head = result ? results_head_ : data_head_, &tail = result ? results_tail_ : data_tail_;
    const bool ok = head - tail < kQueue;
    if (ok) {
      const size_t at = head;
      q[at % kQueue] = e;
      head = at + 1;
    }
    portEXIT_CRITICAL(&mux_);
    return ok;
  }
  // Arm the next transfer if none is in flight: results first.
  void kick() {
    for (;;) {
      Entry e;
      portENTER_CRITICAL(&mux_);
      if (in_flight_ || (results_head_ == results_tail_ && data_head_ == data_tail_)) { portEXIT_CRITICAL(&mux_); return; }
      if (results_head_ != results_tail_) {
        const size_t at = results_tail_;
        e = results_[at % kQueue];
        results_tail_ = at + 1;
      } else {
        const size_t at = data_tail_;
        e = data_[at % kQueue];
        data_tail_ = at + 1;
      }
      in_flight_ = true;
      sending_ = e;
      portEXIT_CRITICAL(&mux_);
      if (vendor_.writeDirect(e.buffer, e.length, e.ends ? EspUsbTransferTermination::EndWithShortPacket
                                                         : EspUsbTransferTermination::Continue))
        return;
      refused_ = refused_ + 1;   // not mounted, or a contract mistake: the buffer is given back unsent
      portENTER_CRITICAL(&mux_);
      in_flight_ = false;
      portEXIT_CRITICAL(&mux_);
      if (e.done) e.done(e.context, e.buffer);
    }
  }
  void completed() {   // the USB worker: the buffer is back (sent, or cancelled by a reset or the device's end)
    portENTER_CRITICAL(&mux_);
    const Entry e = sending_;
    in_flight_ = false;
    portEXIT_CRITICAL(&mux_);
    if (e.done) e.done(e.context, e.buffer);
    kick();
  }
  void received(const uint8_t *data, size_t length) {   // the USB worker; a full ring drops (requests are small)
    for (size_t i = 0; i < length; ++i) {
      const size_t next = (rx_head_ + 1) & (kRxBytes - 1);
      if (next == rx_tail_) return;
      rx_[rx_head_] = data[i];
      rx_head_ = next;
    }
  }
};

}  // namespace oep
#else
#define OEP_HAS_DIRECT_BULK_STREAM 0
#endif
