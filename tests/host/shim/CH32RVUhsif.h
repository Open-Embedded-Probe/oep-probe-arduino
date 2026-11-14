// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host test fake of ArduinoCore-CH32RV's CH32RVUhsif, for OEP_HOST_FAKE_UHSIF: the setup calls checked as the library
// checks them (buffers 64-byte multiples, 32-byte aligned), and hostWrite() puts samples in as the two alternating
// threads of CH32RVLogicCapture's machine do - thread 0 the even buffers, thread 1 the odd ones, each round its own ring
// (first + k x stride) - counting bytesWritten per thread (a u32, as the library's).
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <functional>

enum class CH32RVUhsifInput : uint8_t { Ctrl0 = 0, DataCountHit = 8, DmaReady = 9, One = 10 };
struct CH32RVUhsifTransition { uint8_t next; uint8_t alpha; uint32_t beta; };
struct CH32RVUhsifState { CH32RVUhsifInput input[4]; uint16_t goRight; CH32RVUhsifTransition left, right; };
enum class CH32RVUhsifClock : uint8_t { Sysclk, UsbhsPll };

class CH32RVUhsif {
 public:
  static constexpr uint8_t SampleData = 0x04;
  static constexpr uint32_t Push = 1u << 7, CountData = 1u << 16, ResetDataCount = 1u << 17, Interrupt = 1u << 18;
  static constexpr uint32_t thread(uint8_t n) { return (uint32_t)(n & 3u) << 4; }
  static constexpr uint8_t MaxRingBuffers = 16;
  static constexpr uint16_t BufferUnit = 64, MaxBufferBytes = 0xFFC0;

  static uint32_t sourceHz(CH32RVUhsifClock s) { return s == CH32RVUhsifClock::Sysclk ? 400000000u : 480000000u; }
  bool begin(uint8_t divider, CH32RVUhsifClock source = CH32RVUhsifClock::Sysclk) {
    divider_ = divider;
    source_ = source;
    begun_ = true;
    width_ = 16;
    threads_ = 0;
    return true;
  }
  void end() { begun_ = running_ = false; }
  bool setBusWidth(uint8_t bits) { if (!begun_ || (bits != 8 && bits != 16 && bits != 32)) return false; width_ = bits; return true; }
  bool setStates(const CH32RVUhsifState *, uint8_t count, const CH32RVUhsifTransition &) { return begun_ && count; }
  bool setDataCounter(uint32_t limit) { limit_ = limit; return begun_; }
  bool setRing(uint8_t n, void *first, uint16_t bytes, uint8_t count, uint32_t stride) {
    if (!begun_ || n > 3 || !first || (reinterpret_cast<uintptr_t>(first) & 31) || bytes % BufferUnit || !count ||
        count > MaxRingBuffers || (count > 1 && (stride & 31)))
      return false;
    ring_[n] = {static_cast<uint8_t *>(first), bytes, count, stride};
    threads_ |= 1u << n;
    return true;
  }
  uint32_t bytesWritten(uint8_t n) const { return written_[n]; }
  bool start() {
    if (!begun_ || (threads_ & 3) != 3 || limit_ + 2 != ring_[0].bytes / (width_ / 8u)) return false;
    written_[0] = written_[1] = 0;
    at_ = 0;
    buffer_ = 0;
    running_ = true;
    ++starts;
    return true;
  }
  void stop() { running_ = false; begun_ = false; threads_ = 0; }   // the library's reset clears the setup
  bool running() const { return running_; }
  uint32_t clockHz() const { return sourceHz(source_) / (divider_ + 1u); }

  // Host: n samples (16-bit) from next(i), i counted from the stream's start, into the buffers in the machine's order
  void hostWrite(uint64_t n, const std::function<uint16_t(uint64_t)> &next) {
    for (uint64_t i = 0; i < n && running_; ++i) {
      const uint8_t t = buffer_ & 1;
      const Ring &r = ring_[t];
      uint8_t *buf = r.first + ((buffer_ >> 1) % r.count) * r.stride;
      const uint16_t v = next(sample_);
      const uint32_t step = width_ / 8u;   // 8 bits: the sample's low byte (DATA0-7)
      memcpy(buf + at_, &v, step);
      ++sample_;
      at_ += step;
      written_[t] += step;
      if (at_ == r.bytes) { at_ = 0; ++buffer_; }
    }
  }
  void hostRewind() { sample_ = 0; }
  int starts = 0;
  uint8_t divider_ = 0;
  uint8_t width_ = 16;
  CH32RVUhsifClock source_ = CH32RVUhsifClock::Sysclk;

 private:
  struct Ring { uint8_t *first = nullptr; uint16_t bytes = 0; uint8_t count = 0; uint32_t stride = 0; };
  Ring ring_[4];
  bool begun_ = false, running_ = false;
  uint8_t threads_ = 0;
  uint32_t limit_ = 0;
  uint32_t written_[4] = {};
  uint64_t buffer_ = 0, sample_ = 0;
  uint32_t at_ = 0;
};
