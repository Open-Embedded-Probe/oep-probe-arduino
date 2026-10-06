// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Where a fixture UART's receive lost bytes or took a bad one, in the order the bytes came (oep-if-common §1.3: the lost
// mark is placed at or before the first byte after the loss, never after it).
//
// A place is a count of the bytes the UART received since it began (u32, compared by difference): the mark goes into
// the stream when the bytes before it have been put there. UartRxLedger turns the ESP-IDF UART driver's events, in the
// order its interrupt queued them, into those places and into how many bytes the driver's ring may be read up to - a
// byte after a loss is not read before the loss is known, so its mark never lands behind it.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <atomic>

namespace oep {

struct UartLoss {
  uint32_t at;      // the bytes received before the place (from the UART's begin)
  uint8_t detail;   // mark_detail_lost: 1 overflow, 2 framing, 3 parity
};

// The ESP-IDF driver's events, as FixtureUart takes them (driver/uart.h's uart_event_type_t, mapped).
enum class UartEvent : uint8_t { kData, kBufferFull, kFifoOverflow, kBreak, kFrameError, kParityError, kOther };

// One writer (the task that takes the driver's events, in queue order), one reader (FixtureUart::poll). The driver's
// interrupt, per batch: it reads the RX FIFO into its ring (UART_DATA, size), or into its stash when the ring is full
// (UART_BUFFER_FULL, size: kept, put into the ring before anything newer once there is room); a FIFO overflow resets the
// FIFO (UART_FIFO_OVF) after the data before it was read, so the gap lies right after every byte counted so far; a
// framing / parity error or a break (the bad byte stays in the FIFO) is queued after the data read in the same batch,
// so the bad byte is in that batch's last chunk or after it.
class UartRxLedger {
 public:
  static constexpr size_t kLosses = 16;   // a power of two
  static_assert((kLosses & (kLosses - 1)) == 0, "kLosses: a power of two");

  // Neither side running (the UART begins).
  void reset() {
    pushed_ = last_chunk_ = suspect_ = 0;
    held_ = suspected_ = false;
    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
    limit_.store(0, std::memory_order_release);
  }

  // ---- the writer
  void event(UartEvent type, uint32_t size) {
    switch (type) {
      case UartEvent::kData:
      case UartEvent::kBufferFull:   // stashed, not dropped: what is lost after it comes as a FIFO overflow
        last_chunk_ = pushed_;
        pushed_ += size;
        break;
      case UartEvent::kFifoOverflow: add({pushed_, kDetailOverflow}); break;
      case UartEvent::kBreak:
      case UartEvent::kFrameError: add({bad(), kDetailFraming}); break;
      case UartEvent::kParityError: add({bad(), kDetailParity}); break;
      default: break;
    }
    if (suspect_ && --suspect_ == 0) {
      add({pushed_, kDetailOverflow});
      suspected_ = true;
    }
  }
  // Taking an event found the driver's queue full or nearly (queued: this event and those behind it): what the driver
  // posted while it was full was dropped, after these. Once they are counted a loss is placed there (whether a dropped
  // event said one cannot be told; a FIFO overflow is what fills the queue with the reader away).
  void queueFull(uint32_t queued) {
    if (queued > suspect_) suspect_ = queued;
  }
  // received: the bytes the reader took plus those the driver holds, looked at with no event waiting (the look and the
  // driver's interrupt on the same core, the reader's count read first). More than counted: events were dropped - the
  // count is put right (the bytes come after the loss placed for the full queue; without one, at the first byte the
  // reader may not have taken yet).
  void check(uint32_t received) {
    if (static_cast<int32_t>(received - pushed_) > 0) {
      if (!suspected_) add({limit_.load(std::memory_order_relaxed), kDetailOverflow});
      last_chunk_ = pushed_;
      pushed_ = received;
    }
    suspected_ = false;
  }
  // After a batch (no event waiting): the reader may take what was counted - up to a loss not yet handed over.
  void publish() {
    if (held_ && room()) {
      put(hold_);
      held_ = false;
    }
    limit_.store(held_ ? hold_.at : pushed_, std::memory_order_release);
  }

  // ---- the reader: limit() first, then the losses (every loss before the limit is handed over by then)
  uint32_t limit() const { return limit_.load(std::memory_order_acquire); }
  bool next(UartLoss &loss) const {
    const uint32_t head = head_.load(std::memory_order_relaxed);
    if (head == tail_.load(std::memory_order_acquire)) return false;
    loss = ring_[head % kLosses];
    return true;
  }
  void pop() { head_.store(head_.load(std::memory_order_relaxed) + 1, std::memory_order_release); }

 private:
  static constexpr uint8_t kDetailOverflow = 1, kDetailFraming = 2, kDetailParity = 3;   // mark_detail_lost
  uint32_t pushed_ = 0, last_chunk_ = 0;
  uint32_t suspect_ = 0;     // events still to count before the place of a full queue's drop
  bool suspected_ = false;   // that place was marked since the last check
  bool held_ = false;   // a loss the ring had no room for: later ones merge into it (it is at or before them)
  UartLoss hold_{};
  UartLoss ring_[kLosses];
  std::atomic<uint32_t> head_{0}, tail_{0}, limit_{0};

  // The bad byte's batch: its last chunk, unless the reader may already be past it (a chunk published before the
  // error came is a batch of its own, and the bad byte came after it).
  uint32_t bad() const {
    const uint32_t published = limit_.load(std::memory_order_relaxed);
    return static_cast<int32_t>(last_chunk_ - published) < 0 ? published : last_chunk_;
  }
  bool room() const { return tail_.load(std::memory_order_relaxed) - head_.load(std::memory_order_acquire) < kLosses; }
  void put(const UartLoss &loss) {
    const uint32_t tail = tail_.load(std::memory_order_relaxed);
    ring_[tail % kLosses] = loss;
    tail_.store(tail + 1, std::memory_order_release);
  }
  void add(const UartLoss &loss) {
    if (held_ && room()) {
      put(hold_);
      held_ = false;
    }
    if (held_) {   // merged: the held one is at or before this one
      if (static_cast<int32_t>(loss.at - hold_.at) < 0) hold_.at = loss.at;
      return;
    }
    if (room()) {
      put(loss);
    } else {
      held_ = true;
      hold_ = loss;
    }
  }
};

}  // namespace oep
