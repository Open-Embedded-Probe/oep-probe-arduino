// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepHid.h"

namespace oep {

uint16_t HidFrameStarts::next(const uint8_t *d, size_t n) const {
  size_t at;
  if (have_ == 0) at = 0;
  else if (have_ == 1) {
    if (!n) return kHidNoStart;
    at = 1 + (lo_ | static_cast<size_t>(d[0]) << 8);   // the length's high byte, then the body
  } else at = left_;
  return at < n ? static_cast<uint16_t>(at) : kHidNoStart;
}

void HidFrameStarts::advance(const uint8_t *d, size_t n) {
  for (size_t i = 0; i < n;) {
    if (have_ == 0) {
      lo_ = d[i++];
      have_ = 1;
      got_ = 1;
    } else if (have_ == 1) {
      left_ = lo_ | static_cast<uint32_t>(d[i++]) << 8;
      have_ = left_ ? 2 : 0;
      got_ = left_ ? 2 : 0;
    } else {
      const size_t take = left_ < n - i ? left_ : n - i;
      i += take;
      left_ -= static_cast<uint32_t>(take);
      got_ += static_cast<uint32_t>(take);
      if (!left_) have_ = 0, got_ = 0;
    }
  }
}

size_t hidPackReport(uint8_t *out, size_t room, const uint8_t *data, size_t count, HidFrameStarts &starts) {
  const uint16_t first = starts.next(data, count);
  starts.advance(data, count);
  out[0] = static_cast<uint8_t>(count), out[1] = static_cast<uint8_t>(count >> 8);
  out[2] = static_cast<uint8_t>(first), out[3] = static_cast<uint8_t>(first >> 8);
  if (count) memcpy(out + kHidHead, data, count);
  memset(out + kHidHead + count, 0, room - count);
  return kHidHead + room;
}

void HidIn::take(const uint8_t *d, size_t len) {
  if (!d || len < kHidHead) return;
  if (__atomic_exchange_n(&break_asked_, false, __ATOMIC_ACQUIRE)) broken();   // the reader found it broken
  size_t n = d[0] | static_cast<size_t>(d[1]) << 8;
  const uint16_t first = static_cast<uint16_t>(d[2] | d[3] << 8);
  if (n > len - kHidHead || (first != kHidNoStart && first >= n)) return broken();   // rule 7
  if (!n) return;
  const uint8_t *data = d + kHidHead;
  if (synced_ && starts_.next(data, n) != first) broken();   // rule 6: a report missing before this one
  if (!synced_) {
    if (first == kHidNoStart || marksFull()) return;
    if (owe_mark_) {   // the reader starts over here, before the bytes read from first
      const uint32_t h = __atomic_load_n(&mark_head_, __ATOMIC_RELAXED);
      marks_[h % kMarks] = __atomic_load_n(&produced_, __ATOMIC_RELAXED);
      __atomic_store_n(&mark_head_, h + 1, __ATOMIC_RELEASE);
      owe_mark_ = false;
    }
    data += first;
    n -= first;
    synced_ = true;
    starts_.reset();
  }
  if (room() < n) return broken();   // its bytes cannot be kept: the stream broken
  const uint32_t p = __atomic_load_n(&produced_, __ATOMIC_RELAXED);
  for (size_t i = 0; i < n; ++i) ring_[(p + i) % size_] = data[i];
  starts_.advance(data, n);
  __atomic_store_n(&produced_, p + static_cast<uint32_t>(n), __ATOMIC_RELEASE);
}

size_t HidIn::available() const { return limit() - __atomic_load_n(&consumed_, __ATOMIC_RELAXED); }

size_t HidIn::read(uint8_t *out, size_t n) {
  const uint32_t c = __atomic_load_n(&consumed_, __ATOMIC_RELAXED);
  const size_t have = limit() - c;
  if (n > have) n = have;
  for (size_t i = 0; i < n; ++i) out[i] = ring_[(c + i) % size_];
  __atomic_store_n(&consumed_, c + static_cast<uint32_t>(n), __ATOMIC_RELEASE);
  return n;
}

int HidIn::peek() const { return available() ? ring_[__atomic_load_n(&consumed_, __ATOMIC_RELAXED) % size_] : -1; }

size_t HidIn::room() const {
  return size_ - (__atomic_load_n(&produced_, __ATOMIC_RELAXED) - __atomic_load_n(&consumed_, __ATOMIC_ACQUIRE));
}

bool HidIn::marksFull() const {
  return __atomic_load_n(&mark_head_, __ATOMIC_RELAXED) - __atomic_load_n(&mark_tail_, __ATOMIC_ACQUIRE) >= kMarks;
}

// The stream broken while it was read: the frame begun is dropped by a mark put where the reading goes on (nothing is
// taken in till then, so the mark sits at the head either way); a broken one just opened has nothing to drop.
void HidIn::broken() {
  if (!synced_) return;
  ++losses_;
  synced_ = false;
  owe_mark_ = true;
}

// produced_ first: a mark is never past it
uint32_t HidIn::limit() const {
  const uint32_t p = __atomic_load_n(&produced_, __ATOMIC_ACQUIRE);
  const uint32_t t = __atomic_load_n(&mark_tail_, __ATOMIC_RELAXED);
  if (t == __atomic_load_n(&mark_head_, __ATOMIC_ACQUIRE)) return p;
  const uint32_t m = marks_[t % kMarks], c = __atomic_load_n(&consumed_, __ATOMIC_RELAXED);
  return m - c < p - c ? m : p;   // as distances from the consumer: the totals wrap
}

}  // namespace oep
