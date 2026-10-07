// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepFrame.h"
#include <string.h>

namespace oep {

bool FrameReader::feed(const uint8_t *&data, size_t &n) {
  if (n == 0) return false;
  const uint32_t now = millis();
  if (gapRestarts() && static_cast<uint32_t>(now - last_byte_ms_) > kIdleResyncMs) {
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
  if (gapRestarts() && static_cast<uint32_t>(now - last_byte_ms_) > kIdleResyncMs) {
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
        // transports §1: a length over max_frame - that frame and the input up to the next pause of probe_frame_gap_ms are
        // discarded, unanswered; the next frame starts after the pause. (Not skipping the announced length: a stray
        // byte taken as a length wedged the probe for up to 64 KiB, 2026-09-22.)
        ++dropped_;
        overlong_ = true;
        state_ = State::Discard;
        return false;
      }
      state_ = State::Body;
      return false;
    case State::Body:
      buffer_[have_++] = byte;
      if (have_ < length_) return false;
      return true;
    case State::Discard:   // until a pause (the check above)
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

namespace {
// Standard COBS (transports §1): each block is its length + 1 followed by up to 254 non-zero bytes; a block shorter
// than 254 bytes implies the zero that ended it (none after the last block). A full block (code 0xFF) implies nothing;
// when the data ends right after one, no empty block follows. at(i): the i-th byte of the data; put(b): one encoded byte.
template <class At, class Put>
void cobsBlocks(size_t total, At at, Put put) {
  size_t i = 0;
  for (;;) {
    size_t j = i;
    while (j < total && at(j) != 0 && j - i < 254) ++j;
    const uint8_t code = static_cast<uint8_t>(j - i + 1);
    put(code);
    for (size_t k = i; k < j; ++k) put(at(k));
    if (j >= total) break;
    if (code == 0xff) { i = j; continue; }
    i = j + 1;   // skip the zero this block stood for
  }
}
}  // namespace

size_t cobsEncode(const uint8_t *data, size_t length, uint8_t *out, size_t capacity) {
  size_t n = 0;
  bool fits = true;
  cobsBlocks(length, [&](size_t i) { return data[i]; }, [&](uint8_t b) {
    if (n < capacity) out[n++] = b; else fits = false;
  });
  return fits ? n : 0;
}

bool cobsDecode(const uint8_t *enc, size_t n, uint8_t *out, size_t capacity, size_t &length) {
  size_t in = 0, put = 0;
  while (in < n) {
    const uint8_t code = enc[in++];
    if (code == 0 || in + code - 1 > n || put + code - 1u > capacity) return false;
    memcpy(out + put, enc + in, code - 1u);
    put += code - 1u;
    in += code - 1u;
    if (code != 0xff && in < n) {
      if (put >= capacity) return false;
      out[put++] = 0;
    }
  }
  length = put;
  return true;
}

bool SerialReader::close() {   // decodes into dec_; the encoded bytes stay as they were
  size_t out = 0;
  if (!cobsDecode(enc_, have_, dec_, dec_cap_, out)) return false;
  if (out < 3) return false;
  const uint16_t got = static_cast<uint16_t>(dec_[out - 2] | dec_[out - 1] << 8);
  if (crc16Ccitt(dec_, out - 2) != got) { ++crc_errors_; return false; }
  length_ = out - 2;
  return true;
}

void SerialReader::spill(RawSink sink, void *context) {
  static const uint8_t kZero = 0;
  if (!open_) return;
  // A candidate with nothing after its 0x00 is a delimiter (a frame's closing 0x00, 0x00s in a row), not raw bytes: the
  // closing 0x00 of every frame opens one, and spilling it after the 200 ms gap sent a stray 0x00 to the bound console
  // (2026-09-29, the X035 jig: after the first answer, a monitor reopened, a flash).
  if (have_) {
    sink(context, &kZero, 1);
    sink(context, enc_, have_);
  }
  open_ = false;
  have_ = 0;
}

void SerialReader::idle(RawSink sink, void *context) {
  if (open_ && static_cast<uint32_t>(millis() - last_ms_) >= kGapMs) spill(sink, context);
}

bool SerialReader::feed(const uint8_t *&data, size_t &n, RawSink sink, void *context) {
  if (!n) return false;
  idle(sink, context);
  last_ms_ = millis();
  while (n) {
    if (!open_) {   // raw up to the next 0x00, in one piece
      const uint8_t *zero = static_cast<const uint8_t *>(memchr(data, 0, n));
      const size_t run = zero ? static_cast<size_t>(zero - data) : n;
      if (run) sink(context, data, run);
      data += run;
      n -= run;
      if (!zero) return false;
      ++data;
      --n;
      open_ = true;   // the 0x00 starts a candidate
      have_ = 0;
      continue;
    }
    const uint8_t b = *data;
    if (b == 0) {
      if (have_ == 0) { ++data; --n; continue; }   // 0x00 0x00: an empty frame; this 0x00 starts the next candidate
      const bool message = close();
      if (!message) {                              // not a frame: raw, its leading 0x00 too
        ++bad_;
        static const uint8_t kZero = 0;
        sink(context, &kZero, 1);
        sink(context, enc_, have_);
      }
      have_ = 0;                                   // the closing 0x00 starts the next candidate (open_ stays)
      ++data;
      --n;
      if (message) return true;
      continue;
    }
    if (have_ == enc_cap_) {                       // longer than any frame: raw
      spill(sink, context);
      continue;                                    // this byte again, now outside a candidate
    }
    enc_[have_++] = b;
    ++data;
    --n;
  }
  return false;
}

size_t writeCobsFrame(Stream &stream, const uint8_t *message, size_t length) {
  if (length == 0) return 0;
  const uint16_t crc = crc16Ccitt(message, length);
  const uint8_t tail[2] = {static_cast<uint8_t>(crc), static_cast<uint8_t>(crc >> 8)};
  const size_t total = length + 2;
  auto at = [&](size_t i) -> uint8_t { return i < length ? message[i] : tail[i - length]; };
  // Encoded into a small buffer and written in pieces: one Stream::write per byte takes the UART driver's lock every time.
  uint8_t out[64];
  size_t n = 0;
  auto put = [&](uint8_t b) {
    out[n++] = b;
    if (n == sizeof out) { stream.write(out, n); n = 0; }
  };
  put(0);
  cobsBlocks(total, at, put);
  put(0);
  if (n) stream.write(out, n);
  return length;
}

}  // namespace oep
