// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host test fake of ArduinoCore-CH32RV's CH32RVSPISlave (0.4.0): the queue of armed transfers and the finished ones,
// the fill byte, missed(). hostSelect() is one chip select from the master's side: n MOSI bytes in, the MISO bytes out.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum BitOrder { LSBFIRST = 0, MSBFIRST = 1 };
#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

class CH32RVSPISlave {
 public:
  static constexpr uint8_t QUEUE_DEPTH = 4;
  struct Result {
    uint8_t *rx;
    size_t length;
    size_t size;
  };
  static inline bool fake_fail_begin = false;   // the next begin() refuses the pins
  int sck = -1, miso = -1, mosi = -1, cs = -1;
  uint8_t mode = 0;
  BitOrder order = MSBFIRST;
  bool begun = false;

  bool begin(int s, int mi, int mo, int c, uint8_t m = SPI_MODE0, BitOrder o = MSBFIRST) {
    if (fake_fail_begin) { fake_fail_begin = false; return false; }
    sck = s; miso = mi; mosi = mo; cs = c; mode = m; order = o;
    begun = true;
    armed_count_ = done_count_ = 0;
    return true;
  }
  void end() { begun = false; armed_count_ = done_count_ = 0; }
  void setFill(uint8_t f) { fill_ = f; }
  bool queue(const uint8_t *tx, uint8_t *rx, size_t size) {
    if (!begun || armed_count_ + done_count_ >= QUEUE_DEPTH) return false;
    armed_[armed_count_++] = {tx, rx, size};
    return true;
  }
  bool poll(Result &out) {
    if (!done_count_) return false;
    out = done_[0];
    --done_count_;
    memmove(done_, done_ + 1, sizeof done_[0] * done_count_);
    return true;
  }
  uint32_t missed() const { return missed_; }

  // One chip select with n bytes clocked: the armed head's tx then the fill byte go out, up to its size kept
  void hostSelect(const uint8_t *mosi_bytes, size_t n, uint8_t *miso_bytes = nullptr) {
    if (!begun || n == 0) return;   // no clock: nothing used up
    if (!armed_count_) {
      ++missed_;
      if (miso_bytes) memset(miso_bytes, fill_, n);
      return;
    }
    const Slot s = armed_[0];
    --armed_count_;
    memmove(armed_, armed_ + 1, sizeof armed_[0] * armed_count_);
    for (size_t i = 0; i < n; ++i) {
      if (miso_bytes) miso_bytes[i] = s.tx && i < s.size ? s.tx[i] : fill_;
      if (s.rx && i < s.size) s.rx[i] = mosi_bytes[i];
    }
    done_[done_count_++] = {s.rx, n, s.size};
  }

 private:
  struct Slot {
    const uint8_t *tx;
    uint8_t *rx;
    size_t size;
  };
  uint8_t fill_ = 0xff;
  Slot armed_[QUEUE_DEPTH] = {};
  uint8_t armed_count_ = 0;
  Result done_[QUEUE_DEPTH] = {};
  uint8_t done_count_ = 0;
  uint32_t missed_ = 0;
};
