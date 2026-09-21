#include "OepFrame.h"

namespace oep {

bool FrameReader::push(uint8_t byte) {
  switch (state_) {
    case State::LengthLow:
      length_ = byte;
      state_ = State::LengthHigh;
      return false;
    case State::LengthHigh:
      length_ |= static_cast<size_t>(byte) << 8;
      have_ = 0;
      if (length_ == 0) {  // reserved keepalive: nothing to deliver
        state_ = State::LengthLow;
        return false;
      }
      if (length_ > max_frame_ || length_ > capacity_) {
        ++dropped_;
        state_ = State::Discard;  // stay in sync by skipping the announced bytes
        return false;
      }
      state_ = State::Body;
      return false;
    case State::Body:
      buffer_[have_++] = byte;
      if (have_ < length_) return false;
      return true;
    case State::Discard:
      if (++have_ >= length_) state_ = State::LengthLow;
      return false;
  }
  return false;
}

size_t writeFrame(Stream &stream, const uint8_t *message, size_t length) {
  if (length == 0 || length > 0xffff) return 0;
  const uint8_t prefix[2] = {static_cast<uint8_t>(length), static_cast<uint8_t>(length >> 8)};
  if (stream.write(prefix, 2) != 2) return 0;
  return stream.write(message, length);
}

}  // namespace oep
