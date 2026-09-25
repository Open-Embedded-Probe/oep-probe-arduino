// Length-prefixed frame for reliable byte streams (USB CDC, USB-Serial/JTAG, TCP).
//   len lo | len hi | message
// UART bindings add COBS + CRC in a separate reader; this one trusts the stream.
#pragma once

#include <Arduino.h>

namespace oep {

// What an endpoint accepts per frame and keeps in flight (core confirm reports it).
struct Limits {
  uint16_t max_frame;
  uint32_t window_bytes;   // v1 confirm reports a u32; v0 a u16
  uint8_t max_inflight;
};

class FrameReader {
 public:
  FrameReader(uint8_t *buffer, size_t capacity, uint16_t max_frame)
      : buffer_(buffer), capacity_(capacity), max_frame_(max_frame) {}
  FrameReader() : FrameReader(nullptr, 0, 0) {}
  void reset(uint8_t *buffer, size_t capacity, uint16_t max_frame) { *this = FrameReader(buffer, capacity, max_frame); }
  // Feed one byte. Returns true when a complete message is available.
  bool push(uint8_t byte);
  // Feed a run of bytes: consumes them up to the end of a message (true: the message is available, `data` / `n` point
  // past it) or all of them. One clock read per call and the body copied whole - push() reads the clock per byte,
  // which cost ~1.2 us a byte on the ESP32-P4 (host -> probe capped at 0.8 MB/s).
  bool feed(const uint8_t *&data, size_t &n);
  const uint8_t *message() const { return buffer_; }
  size_t length() const { return length_; }
  void consume() { length_ = 0; have_ = 0; state_ = State::LengthLow; }
  uint32_t dropped() const { return dropped_; }
  uint32_t resyncs() const { return resyncs_; }
  // A frame's bytes arrive back to back (1 KiB is 89 ms at 115200); a longer gap mid-frame means
  // the prefix was a stray byte from another program and the reader starts over on the next byte.
  static constexpr uint32_t kIdleResyncMs = 200;

 private:
  enum class State : uint8_t { LengthLow, LengthHigh, Body, Discard };
  uint8_t *buffer_;
  size_t capacity_;
  uint16_t max_frame_;
  State state_ = State::LengthLow;
  size_t length_ = 0;   // announced message length
  size_t have_ = 0;     // bytes received so far
  uint32_t dropped_ = 0;
  uint32_t resyncs_ = 0;
  uint32_t last_byte_ms_ = 0;
};

// Write one message with its length prefix. Returns bytes of message written (0 on failure).
size_t writeFrame(Stream &stream, const uint8_t *message, size_t length);

// UART binding (oep-spec v0-core-wire-model §2.2, provisional): message + CRC-16 (little endian), COBS-encoded,
// ended by 0x00. A corrupted, truncated or joined frame fails the decode or the CRC and is dropped; the next
// 0x00 is a fresh start, so a stray byte costs one frame, never the link.
// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor ("123456789" -> 0x29B1). Provisional.
uint16_t crc16Ccitt(const uint8_t *data, size_t length, uint16_t crc = 0xFFFF);

class CobsReader {
 public:
  CobsReader(uint8_t *buffer, size_t capacity) : buffer_(buffer), capacity_(capacity) {}
  CobsReader() : CobsReader(nullptr, 0) {}
  void reset(uint8_t *buffer, size_t capacity) { *this = CobsReader(buffer, capacity); }
  bool push(uint8_t byte);   // true when a checked message is available
  const uint8_t *message() const { return buffer_; }
  size_t length() const { return length_; }
  void consume() { length_ = 0; have_ = 0; }
  uint32_t crcErrors() const { return crc_errors_; }
  uint32_t malformed() const { return malformed_; }

 private:
  uint8_t *buffer_;
  size_t capacity_;
  size_t have_ = 0;     // encoded bytes since the last delimiter
  size_t length_ = 0;   // decoded message length once complete
  bool overflow_ = false;
  uint32_t crc_errors_ = 0, malformed_ = 0;
};

size_t writeCobsFrame(Stream &stream, const uint8_t *message, size_t length);

}  // namespace oep
