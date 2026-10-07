// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The two framings of oep-transports §1:
//   FrameReader / writeFrame        len lo | len hi | message, on the message transports (vendor bulk, HID, TCP)
//   SerialReader / writeCobsFrame   0x00 <COBS(message + CRC-16)> 0x00 on every serial port (USB CDC, USB-Serial/JTAG,
//                                   a UART bridge), which also carries raw bytes outside the frames (transports §4)
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
  // TCP (transports §2): a pause inside a frame is normal there and does not restart the read. A length over max_frame is
  // discarded up to the next pause, and overlong() says so: transports §1 asks a TCP probe to close the connection
  // instead, which the endpoint does when the transport is a Connection (a plain Stream cannot be closed).
  void setTcp(bool on) { tcp_ = on; }
  // A length over max_frame was read since the last call (the reader is discarding).
  bool overlong() { const bool was = overlong_; overlong_ = false; return was; }

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
  bool tcp_ = false, overlong_ = false;
  bool gapRestarts() const { return state_ == State::Discard || (state_ != State::LengthLow && !tcp_); }
};

// Write one message with its length prefix. Returns bytes of message written (0 on failure).
size_t writeFrame(Stream &stream, const uint8_t *message, size_t length);

// Serial-port framing (oep-transports §1, §4): message + CRC-16 (little endian), COBS-encoded, sent as 0x00 <COBS> 0x00.
// A serial port carries raw bytes (a target's console) on the same line: a candidate runs from a 0x00 to the next
// 0x00; one that decodes with a matching CRC is a message, any other (with its leading 0x00) is raw, and the closing
// 0x00 starts the next candidate. Bytes outside a candidate are raw at once, and a candidate that stops for 200 ms is
// raw too. A candidate that is only its 0x00 (the delimiter after a frame, 0x00s in a row) is nothing, not raw.
// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor ("123456789" -> 0x29B1).
uint16_t crc16Ccitt(const uint8_t *data, size_t length, uint16_t crc = 0xFFFF);

class SerialReader {
 public:
  using RawSink = void (*)(void *context, const uint8_t *data, size_t length);
  // encoded: the candidate's bytes (a max_frame message is COBS(max_frame + 2) = max_frame + 2 + max_frame / 254 + 1
  // bytes; a longer candidate is raw). decoded: where a message is decoded (at least max_frame + 2).
  void reset(uint8_t *encoded, size_t encoded_capacity, uint8_t *decoded, size_t decoded_capacity) {
    *this = SerialReader();
    enc_ = encoded;
    enc_cap_ = encoded_capacity;
    dec_ = decoded;
    dec_cap_ = decoded_capacity;
  }
  // Consume bytes until a message is complete (true: message() / length(); data and n point past it) or all are used.
  // Raw bytes go to sink(context, ...) as they are found.
  bool feed(const uint8_t *&data, size_t &n, RawSink sink, void *context);
  // No byte for 200 ms: a candidate still open is raw.
  void idle(RawSink sink, void *context);
  const uint8_t *message() const { return dec_; }
  size_t length() const { return length_; }
  uint32_t crcErrors() const { return crc_errors_; }
  // Candidates closed by a 0x00 that were not a frame (no valid COBS, or the CRC did not match): the line's noise. The
  // endpoint's port_speed watches it on a port it sped up (oep-if-link §3).
  uint32_t badCandidates() const { return bad_; }
  static constexpr uint32_t kGapMs = 200;

 private:
  uint8_t *enc_ = nullptr, *dec_ = nullptr;
  size_t enc_cap_ = 0, dec_cap_ = 0;
  bool open_ = false;       // a candidate started (its leading 0x00 is implied, not stored)
  size_t have_ = 0;         // encoded bytes of the candidate
  size_t length_ = 0;
  uint32_t last_ms_ = 0, crc_errors_ = 0, bad_ = 0;
  bool close();             // the candidate ends at a 0x00: true = a message in dec_
  void spill(RawSink sink, void *context);   // the open candidate as raw bytes (0x00 first)
};

// One message as 0x00 <COBS(message + CRC)> 0x00, in one piece where the stream takes it. No empty block after a full
// one (transports §1).
size_t writeCobsFrame(Stream &stream, const uint8_t *message, size_t length);
// The most bytes writeCobsFrame writes for a message of `length`.
constexpr size_t cobsFrameMax(size_t length) { return length + 2 + (length + 2) / 254 + 3; }
// COBS alone, no delimiters (transports §1): the encoding of `data` into `out` (its length; 0 when it does not fit),
// and a decoding (false: a code byte 0, a block past the end, or no room) - what writeCobsFrame and SerialReader use.
size_t cobsEncode(const uint8_t *data, size_t length, uint8_t *out, size_t capacity);
bool cobsDecode(const uint8_t *encoded, size_t n, uint8_t *out, size_t capacity, size_t &length);

}  // namespace oep
