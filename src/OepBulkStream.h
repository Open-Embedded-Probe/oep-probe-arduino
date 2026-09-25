// Stream adapter over an EspUsbDevice vendor bulk interface (ESP32-P4 HS OTG), so the
// v0 endpoint runs unchanged over USB bulk. E160 (2026-09-22): the buffered vendor
// build arms an IN transfer only on a full packet or a flush, so the endpoint flushes
// at the end of each burst; the host coalesces frames into one bulk write.
#pragma once

#include <Arduino.h>

#if __has_include(<EspUsbDevice.h>)
#include <EspUsbDevice.h>
#define OEP_HAS_BULK_STREAM 1

namespace oep {

class BulkStream final : public Stream {
 public:
  explicit BulkStream(EspUsbDeviceVendor &vendor) : vendor_(vendor) {}
  int available() override { return (peeked_ >= 0 ? 1 : 0) + vendor_.available(); }
  int read() override {
    if (peeked_ >= 0) { const int c = peeked_; peeked_ = -1; return c; }
    return vendor_.read();
  }
  int peek() override {
    if (peeked_ < 0) peeked_ = vendor_.read();
    return peeked_;
  }
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *buffer, size_t size) override {
    size_t done = 0;
    while (done < size) {
      const size_t n = vendor_.write(buffer + done, size - done);
      if (n == 0) {
        if (!vendor_.mounted()) return done;
        vendor_.flush();          // make room: push what is queued
        delayMicroseconds(50);
        continue;
      }
      done += n;
    }
    return done;
  }
  void flush() override { vendor_.flush(); }
  // What can be written now without waiting (the v1 endpoint sizes its pushes from this).
  int availableForWrite() override { return static_cast<int>(vendor_.writeAvailable()); }
  bool mounted() const { return vendor_.mounted(); }

 private:
  EspUsbDeviceVendor &vendor_;
  int peeked_ = -1;
};

}  // namespace oep
#else
#define OEP_HAS_BULK_STREAM 0
#endif
