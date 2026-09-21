// Length-prefixed frame for reliable byte streams (USB CDC, USB-Serial/JTAG, TCP).
//   len lo | len hi | message
// UART bindings add COBS + CRC in a separate reader; this one trusts the stream.
#pragma once

#include <Arduino.h>

namespace oep {

class FrameReader {
 public:
  FrameReader(uint8_t *buffer, size_t capacity, uint16_t max_frame)
      : buffer_(buffer), capacity_(capacity), max_frame_(max_frame) {}
  // Feed one byte. Returns true when a complete message is available.
  bool push(uint8_t byte);
  const uint8_t *message() const { return buffer_; }
  size_t length() const { return length_; }
  void consume() { length_ = 0; have_ = 0; state_ = State::LengthLow; }
  uint32_t dropped() const { return dropped_; }

 private:
  enum class State : uint8_t { LengthLow, LengthHigh, Body, Discard };
  uint8_t *buffer_;
  size_t capacity_;
  uint16_t max_frame_;
  State state_ = State::LengthLow;
  size_t length_ = 0;   // announced message length
  size_t have_ = 0;     // bytes received so far
  uint32_t dropped_ = 0;
};

// Write one message with its length prefix. Returns bytes of message written (0 on failure).
size_t writeFrame(Stream &stream, const uint8_t *message, size_t length);

}  // namespace oep
