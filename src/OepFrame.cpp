#include "OepFrame.h"
#include <string.h>

namespace oep {

bool FrameReader::feed(const uint8_t *&data, size_t &n) {
  if (n == 0) return false;
  const uint32_t now = millis();
  if (state_ != State::LengthLow && static_cast<uint32_t>(now - last_byte_ms_) > kIdleResyncMs) {
    ++resyncs_;
    state_ = State::LengthLow;
  }
  while (n) {
    if (state_ == State::Body && have_ < length_) {
      const size_t take = length_ - have_ < n ? length_ - have_ : n;
      memcpy(buffer_ + have_, data, take);
      have_ += take;
      data += take;
      n -= take;
      if (have_ == length_) { last_byte_ms_ = now; return true; }
      continue;
    }
    last_byte_ms_ = now;   // push() then sees no idle gap
    const bool done = push(*data++);
    --n;
    if (done) return true;
  }
  last_byte_ms_ = now;
  return false;
}

bool FrameReader::push(uint8_t byte) {
  // Resync (2026-09-22, classic ESP32 UART): a single stray byte was taken as a length and the
  // reader then waited for bytes that never came, or skipped a 64 KiB "frame", until a reset.
  const uint32_t now = millis();
  if (state_ != State::LengthLow && static_cast<uint32_t>(now - last_byte_ms_) > kIdleResyncMs) {
    ++resyncs_;
    state_ = State::LengthLow;
  }
  last_byte_ms_ = now;
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
        // No legal frame is this long, so the two bytes were not a prefix: treat this byte as a
        // new low byte instead of skipping up to 64 KiB (which wedged the probe).
        ++dropped_;
        length_ = byte;
        state_ = State::LengthHigh;
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

uint16_t crc16Ccitt(const uint8_t *data, size_t length, uint16_t crc) {
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

bool CobsReader::push(uint8_t byte) {
  if (byte != 0) {
    if (have_ < capacity_) buffer_[have_++] = byte;
    else overflow_ = true;
    return false;
  }
  // Delimiter: decode in place (the output is never longer than the input).
  const size_t n = have_;
  have_ = 0;
  if (n == 0) return false;                 // back-to-back delimiters: nothing
  if (overflow_) { overflow_ = false; ++malformed_; return false; }
  size_t in = 0, out = 0;
  while (in < n) {
    const uint8_t code = buffer_[in++];
    if (in + code - 1 > n) { ++malformed_; return false; }
    for (uint8_t i = 1; i < code; ++i) buffer_[out++] = buffer_[in++];
    if (code != 0xff && in < n) buffer_[out++] = 0;
  }
  if (out < 3) { ++malformed_; return false; }
  const uint16_t got = static_cast<uint16_t>(buffer_[out - 2] | buffer_[out - 1] << 8);
  if (crc16Ccitt(buffer_, out - 2) != got) { ++crc_errors_; return false; }
  length_ = out - 2;
  return true;
}

size_t writeCobsFrame(Stream &stream, const uint8_t *message, size_t length) {
  if (length == 0) return 0;
  const uint16_t crc = crc16Ccitt(message, length);
  const uint8_t tail[2] = {static_cast<uint8_t>(crc), static_cast<uint8_t>(crc >> 8)};
  const size_t total = length + 2;
  auto at = [&](size_t i) -> uint8_t { return i < length ? message[i] : tail[i - length]; };
  // Standard COBS: each block is its length + 1 followed by up to 254 non-zero bytes; a block shorter than
  // 254 bytes implies the zero that ended it (none after the last block). A full block (code 0xFF) implies
  // nothing, so the next block starts right after it - and if the data ends there, an empty block follows.
  // Encoded into a small buffer and written in pieces: one Stream::write per byte takes the UART driver's lock
  // every time.
  uint8_t out[64];
  size_t n = 0;
  auto put = [&](uint8_t b) {
    out[n++] = b;
    if (n == sizeof out) { stream.write(out, n); n = 0; }
  };
  size_t i = 0;
  for (;;) {
    size_t j = i;
    while (j < total && at(j) != 0 && j - i < 254) ++j;
    const uint8_t code = static_cast<uint8_t>(j - i + 1);
    put(code);
    for (size_t k = i; k < j; ++k) put(at(k));
    if (code == 0xff) { i = j; continue; }
    if (j >= total) break;
    i = j + 1;   // skip the zero this block stood for
  }
  put(0);
  if (n) stream.write(out, n);
  return length;
}

}  // namespace oep
