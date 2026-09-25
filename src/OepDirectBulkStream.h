// Stream + zero-copy transport over an EspUsbDevice vendor bulk interface in a direct build
// (-DCFG_TUD_VENDOR_TXRX_BUFFERED=0 in the sketch's build_opt.h, compiled with --clean). One IN transfer is in flight
// at a time (writeDirect); the next is armed from the completion callback, so a stream of full buffers runs gapless
// (wch-protocols E110 / E116 / E117). Results are written into one of two 64-aligned buffers and sent on flush();
// they go before queued data. The direct build sends no zero-length packet, so a result whose length is a whole number
// of 512-byte packets is sent as two transfers (the last byte alone) to end the host's transfer.
#pragma once

#include <Arduino.h>

#if __has_include(<EspUsbDevice.h>)
#include <EspUsbDevice.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>

#include "OepV1.h"

#define OEP_HAS_DIRECT_BULK_STREAM 1

namespace oep {

class DirectBulkStream final : public Stream, public v1::DirectTransport {
 public:
  // kRxBytes: two whole max_frame requests; the usbd task can land a frame faster than loop() reads it byte by byte
  static constexpr size_t kResultBytes = 16384 + 64, kRxBytes = 32768, kQueue = 32, kPacket = 512;

  explicit DirectBulkStream(EspUsbDeviceVendor &vendor) : vendor_(vendor) {}

  // After the device's begin(): buffers and callbacks. false: not a direct build, or no memory.
  bool begin() {
    if (!EspUsbDeviceVendor::directWriteSupported()) return false;
    for (int i = 0; i < 2; ++i) {
      result_[i] = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, kResultBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
      tail_[i] = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, 64, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
      if (!result_[i] || !tail_[i]) return false;
    }
    vendor_.onRxData([this](const uint8_t *data, size_t length) { received(data, length); });
    vendor_.onTxComplete([this](size_t) { completed(); });
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
  int peek() override { return rx_head_ == rx_tail_ ? -1 : rx_[rx_tail_]; }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *data, size_t size) override {
    size_t done = 0;
    while (done < size) {
      if (!waitWritable()) return done;
      const size_t n = min(size - done, kResultBytes - 64 - length_[current_]);
      memcpy(result_[current_] + length_[current_], data + done, n);
      length_[current_] += n;
      done += n;
      if (length_[current_] >= kResultBytes - 64) flush();
    }
    return done;
  }
  void flush() override {
    const int i = current_;
    const size_t n = length_[i];
    if (n == 0) return;
    busy_[i] = true;
    current_ ^= 1;
    if (n % kPacket == 0) {   // end the host's transfer with a short packet: the last byte goes alone
      tail_[i][0] = result_[i][n - 1];
      queue(true, {result_[i], n - 1, nullptr, nullptr});
      queue(true, {tail_[i], 1, resultSent, this});
    } else {
      queue(true, {result_[i], n, resultSent, this});
    }
    kick();
  }
  int availableForWrite() override {
    return busy_[current_] ? 0 : static_cast<int>(kResultBytes - 64 - length_[current_]);
  }

  // DirectTransport (an interface's data, from any task)
  bool queueData(const uint8_t *buffer, size_t length, Done done, void *context) override {
    if (!queue(false, {buffer, length, done, context})) return false;
    kick();
    return true;
  }
  size_t queued() const override { return (data_head_ - data_tail_) + (in_flight_ ? 1 : 0); }
  bool mounted() const { return vendor_.mounted(); }
  uint32_t refused() const { return refused_; }

 private:
  struct Entry { const uint8_t *buffer; size_t length; Done done; void *context; };
  EspUsbDeviceVendor &vendor_;
  uint8_t *result_[2] = {}, *tail_[2] = {};
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
      if (buffer == self->result_[i] || buffer == self->tail_[i]) { self->length_[i] = 0; self->busy_[i] = false; }
  }
  bool waitWritable() {
    for (uint32_t start = millis(); busy_[current_];) {
      if (!vendor_.mounted() || millis() - start > 200) return false;
      delayMicroseconds(20);
    }
    return true;
  }
  bool queue(bool result, const Entry &e) {
    portENTER_CRITICAL(&mux_);
    Entry *q = result ? results_ : data_;
    volatile size_t &head = result ? results_head_ : data_head_, &tail = result ? results_tail_ : data_tail_;
    const bool ok = head - tail < kQueue;
    if (ok) q[head++ % kQueue] = e;
    portEXIT_CRITICAL(&mux_);
    return ok;
  }
  // Arm the next transfer if none is in flight: results first.
  void kick() {
    for (;;) {
      Entry e;
      portENTER_CRITICAL(&mux_);
      if (in_flight_ || (results_head_ == results_tail_ && data_head_ == data_tail_)) { portEXIT_CRITICAL(&mux_); return; }
      e = results_head_ != results_tail_ ? results_[results_tail_++ % kQueue] : data_[data_tail_++ % kQueue];
      in_flight_ = true;
      sending_ = e;
      portEXIT_CRITICAL(&mux_);
      if (vendor_.writeDirect(e.buffer, e.length)) return;
      ++refused_;   // not mounted, or a contract mistake: the buffer is given back unsent
      portENTER_CRITICAL(&mux_);
      in_flight_ = false;
      portEXIT_CRITICAL(&mux_);
      if (e.done) e.done(e.context, e.buffer);
    }
  }
  void completed() {   // usbd task
    portENTER_CRITICAL(&mux_);
    const Entry e = sending_;
    in_flight_ = false;
    portEXIT_CRITICAL(&mux_);
    if (e.done) e.done(e.context, e.buffer);
    kick();
  }
  void received(const uint8_t *data, size_t length) {   // usbd task; a full ring drops (requests are small)
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
