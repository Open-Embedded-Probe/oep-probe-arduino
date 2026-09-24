#include "OepDmConsole.h"

#include <string.h>


namespace oep {

// ---- target.console ------------------------------------------------------
void DmConsole::push(uint8_t byte) {
  if (!discarding_ && sink_) sink_(sink_ctx_, byte);
}

void DmConsole::poll() {
  // DATA0 and DATA1 are the abstract command's operands too, so leave them alone unless
  // the target is attached and running its own code - and not while the host drives the debug
  // module through raw DMI writes, which halted() does not see.
  if (!enabled_ || dm_.halted() || dm_.hostRaw()) return;
  if (!phy_.attached()) {
    // A reset detaches, and the console has to outlive that: the point of it is to watch
    // a target through its own restarts. Retry at a slow rate so a target that is simply
    // gone does not turn every loop into a full attach.
    if (millis() - last_attach_ms_ < 250) return;
    last_attach_ms_ = millis();
    if (!dm_.attach()) return;
  }
  if (framing_ == 1) pollDmdata();
  else if (framing_ == 2) pollSeq();
  else pollSdi();
}

// SerialSDI: the target waits for DATA0 to read zero, writes DATA1 = bytes 3..6 and
// DATA0 = length | bytes 0..2 << 8, and we zero DATA0 once we have the frame.
void DmConsole::pollSdi() {
  uint32_t data0 = 0;
  if (!phy_.read(0x04, data0)) return;
  const uint8_t length = static_cast<uint8_t>(data0 & 0xff);
  if (length == 0 || length > 7) return;       // 0 = nothing waiting; anything else is not a frame
  uint32_t data1 = 0;
  if (!phy_.read(0x05, data1)) return;
  const uint8_t bytes[7] = {
      static_cast<uint8_t>(data0 >> 8), static_cast<uint8_t>(data0 >> 16), static_cast<uint8_t>(data0 >> 24),
      static_cast<uint8_t>(data1), static_cast<uint8_t>(data1 >> 8),
      static_cast<uint8_t>(data1 >> 16), static_cast<uint8_t>(data1 >> 24)};
  for (uint8_t i = 0; i < length; ++i) push(bytes[i]);
  phy_.write(0x04, 0);                         // taken: this is what the target waits for
}

// SerialDMDATA, minichlink's framing. The status byte is the low byte of DATA0: bit 7 says
// the word is the target's, the low bits are a byte count biased by 4. We answer each of its
// words exactly once, and the answer is also our outgoing frame when there is one - three
// bytes at a time, since only DATA0 carries host payload - or zero when there is not.
void DmConsole::pollDmdata() {
  uint32_t data0 = 0;
  if (!phy_.read(0x04, data0)) return;
  if (data0 & 0x80u) {                         // the target's word
    uint32_t count = data0 & 0x3fu;
    if (count > 4u) {
      count -= 4u;
      if (count > 7u) count = 7u;
      uint32_t data1 = 0;
      if (!phy_.read(0x05, data1)) return;
      const uint8_t bytes[7] = {
          static_cast<uint8_t>(data0 >> 8), static_cast<uint8_t>(data0 >> 16), static_cast<uint8_t>(data0 >> 24),
          static_cast<uint8_t>(data1), static_cast<uint8_t>(data1 >> 8),
          static_cast<uint8_t>(data1 >> 16), static_cast<uint8_t>(data1 >> 24)};
      for (uint32_t i = 0; i < count; ++i) push(bytes[i]);
      saw_empty_ = false;
      sendOrClear();
      return;
    }
    // Count 4 is the target's empty frame: "the mailbox is yours". But its write() leaves
    // that word and then, a moment later, its own frame on top - so an empty frame may be
    // the instant before real bytes land, and answering it then wipes them out. Measured on
    // a CH32X035 at 48 MHz behind the P4's fast PHY: every other frame vanished, "core_ap"
    // and " READY" gone and "i" and "\r\n" arriving (2026-09-23). A slow target hid it. So
    // answer an empty frame only once it has stood still for a whole poll.
    if (!saw_empty_) {
      saw_empty_ = true;
      return;
    }
    saw_empty_ = false;
    sendOrClear();
    return;
  }
  // Bit 7 clear: our own frame not yet collected, or the word we just left. Not ours to
  // touch - the target owns the initiative, and a probe only ever answers the target's
  // words, as minichlink's terminal does. Writing into a clear word races the target's own
  // poll, which leaves its empty frame there at the same moment; on the CH32X035 that ate
  // the host's frames and PING never came back (2026-09-23).
  saw_empty_ = false;
}

// Clearing bit 7 is how the target learns its frame was taken - and our own frame clears
// it too. So when there is something to send, send it here rather than zeroing first: a
// target that keeps printing leaves its empty frame on every poll, and a probe that only
// ever answered with zero would never get a turn (2026-09-23).
void DmConsole::sendOrClear() {
  const uint16_t waiting = pending();
  if (!waiting) {
    phy_.write(0x04, 0);
    return;
  }
  uint8_t p[3] = {0, 0, 0};
  const uint8_t chunk = waiting > 3 ? 3 : static_cast<uint8_t>(waiting);
  for (uint8_t i = 0; i < chunk; ++i) {
    p[i] = tx_[tx_tail_];
    tx_tail_ = static_cast<uint16_t>((tx_tail_ + 1) % kTxCapacity);
  }
  phy_.write(0x04, uint32_t(chunk + 4u) | (uint32_t(p[0]) << 8) |
                       (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 24));
}

// dmseq (framing 2), oep-spec docs/target-console-dmseq.ja.md. SerialDMDATA's carrier with
// 1-bit sequence numbers both ways and a CRC-8 on every word, so a DMI access that goes
// astray - a lost answer, a corrupted word - neither duplicates nor drops a byte.
//
// Test hooks, off unless a build defines them: OEP_CONSOLE_FAULT_PERMILLE drops or corrupts
// that share of answers and corrupts that share of frames read; OEP_CONSOLE_FAULT_SYN drops
// the answers to the first N SYN frames after each resync. Both were how the framing was
// chosen (oep-spec experiments/dm-console-seq); keep them for regressions.
#ifndef OEP_CONSOLE_FAULT_PERMILLE
#define OEP_CONSOLE_FAULT_PERMILLE 0
#endif
#ifndef OEP_CONSOLE_FAULT_SYN
#define OEP_CONSOLE_FAULT_SYN 0
#endif

static uint8_t seqCrc8(const uint8_t *p, size_t n) {   // poly 0x07, init 0xFF
  uint8_t crc = 0xff;
  for (size_t i = 0; i < n; ++i) {
    crc ^= p[i];
    for (int b = 0; b < 8; ++b) crc = static_cast<uint8_t>((crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1);
  }
  return crc;
}

static bool seqFault() {
#if OEP_CONSOLE_FAULT_PERMILLE > 0
  static uint32_t x = 2463534242u;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  return (x % 1000u) < OEP_CONSOLE_FAULT_PERMILLE;
#else
  return false;
#endif
}

void DmConsole::pollSeq() {
  // DATA0 first, DATA1 only when the frame reaches it, and the answer only after both: once
  // answered, the target may post its next frame and overwrite DATA1.
  uint32_t w0 = 0, w1 = 0;
  if (!phy_.read(0x04, w0)) return;
  if (!(w0 & 0x80u)) return;                   // our answer still there, or nothing yet
  const uint8_t n = w0 & 0x07u;
  if (n >= 3 && !phy_.read(0x05, w1)) return;
  if (seqFault()) w0 ^= 1u << (8 + (w0 & 7));  // test hook: a frame read corrupted
  const uint8_t b[8] = {static_cast<uint8_t>(w0), static_cast<uint8_t>(w0 >> 8), static_cast<uint8_t>(w0 >> 16),
                        static_cast<uint8_t>(w0 >> 24), static_cast<uint8_t>(w1), static_cast<uint8_t>(w1 >> 8),
                        static_cast<uint8_t>(w1 >> 16), static_cast<uint8_t>(w1 >> 24)};
  // N before the CRC: at N = 7 there is no byte 1+N, and 0xffffffff - what an attach leaves
  // in DATA0 - decodes to exactly that.
  if (n > 6 || seqCrc8(b, static_cast<size_t>(1 + n)) != b[1 + n]) {
    // Usually a bad read, and the next poll reads it right. If it stays bad the word may be
    // our own answer, corrupted into a shape with bit 7 set, and then both sides wait.
    // Answering K = the last S we accepted is safe whatever is there: if the target's
    // outstanding frame is that one, it was ours already; if it is a new one, K does not
    // match and the target posts it again.
    if (++seq_bad_run_ >= 3 && seq_synced_) { seq_bad_run_ = 0; seqAnswer(seq_last_s_, false); }
    return;
  }
  seq_bad_run_ = 0;
  const uint8_t s = (w0 >> 5) & 1u, a = (w0 >> 4) & 1u;
  const bool syn = (w0 & 0x08u) != 0;
  // A SYN frame posted again (its answer did not land) is a duplicate like any other; only
  // a SYN that is not one resyncs. Resyncing on every SYN delivered the same payload twice
  // (found in review by the ch32rv side, 2026-09-24).
  const bool duplicate = seq_synced_ && s == seq_last_s_ && (!syn || seq_last_syn_);
  if (!duplicate && (syn || !seq_synced_)) {    // resync, then accept as below
    seq_synced_ = true;
    seq_last_s_ = static_cast<uint8_t>(s ^ 1u);   // so this frame counts as new
    seq_h_ = static_cast<uint8_t>(a ^ 1u);
    seq_chunk_len_ = 0;                           // anything half-sent went to the old session
    seq_syn_drops_ = 0;
    ++seq_resyncs_;
  }
  if (s != seq_last_s_) {
    for (uint8_t i = 0; i < n; ++i) push(b[1 + i]);
    seq_last_s_ = s;
    seq_last_syn_ = syn;
  }
  if (seq_chunk_len_ && a == seq_h_) {           // the target has our last payload
    seq_chunk_len_ = 0;
    seq_h_ ^= 1u;
  }
#if OEP_CONSOLE_FAULT_SYN > 0
  if (syn && seq_syn_drops_ < OEP_CONSOLE_FAULT_SYN) { ++seq_syn_drops_; return; }   // test hook
#endif
  seqAnswer(s, true);
}

void DmConsole::seqAnswer(uint8_t k, bool with_data) {
  if (with_data && !seq_chunk_len_) {
    while (seq_chunk_len_ < 2 && tx_tail_ != tx_head_) {
      seq_chunk_[seq_chunk_len_++] = tx_[tx_tail_];
      tx_tail_ = static_cast<uint16_t>((tx_tail_ + 1) % kTxCapacity);
    }
  }
  const uint8_t m = with_data ? seq_chunk_len_ : 0;
  uint8_t ans[4] = {static_cast<uint8_t>((k << 5) | (seq_h_ << 4) | m), 0, 0, 0};
  for (uint8_t i = 0; i < m; ++i) ans[1 + i] = seq_chunk_[i];
  ans[1 + m] = seqCrc8(ans, static_cast<size_t>(1 + m));   // right after the payload
  uint32_t answer = uint32_t(ans[0]) | (uint32_t(ans[1]) << 8) | (uint32_t(ans[2]) << 16) | (uint32_t(ans[3]) << 24);
  if (seqFault()) return;                                   // test hook: the answer does not land
  if (seqFault()) answer ^= 1u << (answer % 24);            // test hook: it lands corrupted
  phy_.write(0x04, answer);
}

// Every start is a fresh session, even over one that is still open: a runner that moves from one
// sketch to the next reprograms the target in between, and bytes queued for the last sketch must
// not be delivered to this one.
bool DmConsole::start(uint8_t framing) {
  if (framing > 2 || !dm_.attach()) return false;
  tx_head_ = tx_tail_ = 0;
  saw_empty_ = false;
  seq_synced_ = false;
  seq_last_syn_ = false;
  seq_syn_drops_ = 0;
  seq_chunk_len_ = 0;
  seq_resyncs_ = 0;
  // Whatever an earlier session left in the mailbox would read as a frame - including
  // SerialDMDATA's latched timeout, which a host clears by taking the word. Claim it,
  // then let a couple of rounds go by and throw those away, so the first exchange the
  // caller sees is not the tail of somebody else's.
  phy_.write(0x04, 0);
  enabled_ = true;
  framing_ = framing;
  discarding_ = true;
  for (int i = 0; i < 4; ++i) poll();
  discarding_ = false;
  return true;
}

size_t DmConsole::queue(const uint8_t *data, size_t length) {
  if (!enabled_ || framing_ == 0) return 0;
  size_t queued = 0;
  while (queued < length) {
    const uint16_t next = static_cast<uint16_t>((tx_head_ + 1) % kTxCapacity);
    if (next == tx_tail_) break;           // full: the target is not collecting
    tx_[tx_head_] = data[queued++];
    tx_head_ = next;
  }
  return queued;
}

}  // namespace oep
