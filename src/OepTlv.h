// TLV helpers for describe / plan payloads: tag(u8) len(u8) value. Tag bit7 = critical.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace oep {

struct TlvReader {
  const uint8_t *data;
  size_t length;
  size_t offset = 0;
  // Advance to the next element; false at the end or on a malformed element.
  bool next(uint8_t &tag, const uint8_t *&value, uint8_t &value_length) {
    if (offset + 2 > length) return false;
    tag = data[offset];
    value_length = data[offset + 1];
    if (offset + 2 + value_length > length) return false;
    value = data + offset + 2;
    offset += 2 + value_length;
    return true;
  }
  bool wellFormed() const {
    size_t o = 0;
    while (o + 2 <= length) { o += 2 + data[o + 1]; }
    return o == length;
  }
};

inline size_t tlvPut(uint8_t *out, size_t capacity, size_t used, uint8_t tag, const uint8_t *value, uint8_t length) {
  if (used + 2 + length > capacity) return 0;
  out[used] = tag;
  out[used + 1] = length;
  if (length) memcpy(out + used + 2, value, length);
  return used + 2 + length;
}

inline size_t tlvPutU16(uint8_t *out, size_t capacity, size_t used, uint8_t tag, uint16_t v) {
  const uint8_t b[2] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8)};
  return tlvPut(out, capacity, used, tag, b, 2);
}

inline size_t tlvPutU32(uint8_t *out, size_t capacity, size_t used, uint8_t tag, uint32_t v) {
  const uint8_t b[4] = {static_cast<uint8_t>(v), static_cast<uint8_t>(v >> 8), static_cast<uint8_t>(v >> 16), static_cast<uint8_t>(v >> 24)};
  return tlvPut(out, capacity, used, tag, b, 4);
}

}  // namespace oep
