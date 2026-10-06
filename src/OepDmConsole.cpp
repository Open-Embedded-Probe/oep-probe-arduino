// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepDmConsole.h"

#include <string.h>


namespace oep {

// ---- target.console ------------------------------------------------------
void DmConsole::push(uint8_t byte) {
  if (sink_) sink_(sink_ctx_, byte);
}

// The send queue's head (oep-if-console §2): bytes leave it only on an answer to the target.
uint8_t DmConsole::take() {
  const uint8_t byte = tx_[tx_tail_];
  tx_tail_ = static_cast<uint16_t>((tx_tail_ + 1) % kSendQueue);
  --tx_count_;
  return byte;
}

// A DMI read of the mailbox. Its outcome runs the PHY's wire-loss clock (oep-if-debug §2, shared with the host's
// requests on the connection): the wire is lost once reads have got nothing back for wire_lost_ms with no answer between.
bool DmConsole::readData(uint8_t address, uint32_t &value) {
  const bool ok = phy_.read(address, value);
  if (phy_.loss().lost()) lost_ = true;   // also a DMSTATUS of all zeros / ones (no answer, DmiPhy::read)
  return ok;
}

// A link may miss one DMI access and be up again at the next (a glitch): a write lost, or a read answering the value of the
// read before it - the last word of the poll before (a frame already taken, a DATA1 of bytes), DMSTATUS, a register a
// host's op read. So a word the console acts on - takes bytes from, answers, writes 0 over - is read twice, and taken
// only when both reads agree: DATA0 (the poll's read), DATA1 when the frame reaches it, a read of another register
// (`between`), DATA1 again, DATA0 again. One missed read cannot make them agree on anything but what the mailbox holds:
// the first DATA0 read missed gives the read before it, which the second does not (unless that is what DATA0 holds);
// the first DATA1 read missed gives DATA0's word, the second (after `between`) `between`'s value; the second DATA0 read
// missed gives DATA1's or `between`'s. `between` is a register whose value is no word to act on: DMSTATUS for SDI (its low
// byte, 0x82 / 0x83, is no length), DMCONTROL for DMDATA and dmseq (bit 7 clear). A word read once that would have been
// acted on took a stale frame for a new one (its bytes twice, and the 0 or the answer written over the frame the target
// had just posted: lost), a DATA1 of the frame before for this one's bytes, or DMSTATUS (bit 7 set, L 2: "no slot") for
// a DMDATA word to clear - over the target's real slot. Not agreeing: nothing is done, and the next poll reads again.
// A word that is acted on by nobody (bit 7 clear, L 0) costs one read as before: an idle poll adds nothing.
//
// A lost write is not seen this way: SDI and DMDATA carry no sequence number, so a lost receipt (SDI's 0) or answer
// (DMDATA's) leaves the target's word as it was, which the next poll cannot tell from the same bytes posted again - one
// frame's bytes are then taken twice (and DMDATA's answer's input bytes are lost). dmseq's sequence numbers cover it (the
// target posts the frame again; the host takes it for a duplicate).
bool DmConsole::confirm(uint32_t data0, bool with_data1, uint32_t &data1, uint8_t between) {
  uint32_t d1 = 0, other = 0, d1_again = 0, d0_again = 0;
  if (with_data1 && !readData(0x05, d1)) return false;
  if (!readData(between, other)) return false;
  if (with_data1 && (!readData(0x05, d1_again) || d1_again != d1)) return false;
  if (!readData(0x04, d0_again) || d0_again != data0) return false;
  data1 = d1;
  return true;
}

void DmConsole::poll() {
  // DATA0 and DATA1 are the abstract command's operands too, so leave them alone unless the target is attached and
  // running its own code. Whether the hart runs is judged from DMSTATUS, read every kStatusMs (oep-if-console §3: the
  // host may have halted or resumed it through raw DMI, which Ch32Dm's own view does not see - a raw resume after the
  // probe's halt left the console silent until the next high-level op); in between, a halt by the probe itself counts
  // at once. A reset the target did by itself (havereset) is acknowledged there too (oep-if-debug §4.6: the stream marks
  // a restart, dmseq starts over).
  if (!enabled_ || lost_) return;
  if (!phy_.backgroundTurn()) return;   // the wire paused (DmiPhy::backgroundTurn): nothing read, the next poll reads
  struct TurnDone {
    DmiPhy &phy;
    ~TurnDone() { phy.backgroundDone(); }
  } turn_done{phy_};
  phy_.beginRequest();   // one console read: its own allowance for wire retries (oep-if-debug §2)
  if (!phy_.attached()) {
    // A reset detaches, and the console has to outlive that: the point of it is to watch
    // a target through its own restarts. Retry at a slow rate so a target that is simply
    // gone does not turn every loop into a full attach.
    if (millis() - last_attach_ms_ < 250) return;
    last_attach_ms_ = millis();
    AttachDeadline budget(phy_);   // as any attach (oep-if-debug §1)
    if (!dm_.attach()) {
      phy_.loss().silent();   // the wire did not come back: on the same clock as a read that got nothing
      if (phy_.loss().lost()) lost_ = true;
      return;
    }
  }
  if (millis() - last_status_ms_ >= kStatusMs) {
    last_status_ms_ = millis();
    uint32_t status = 0, control = 0, again = 0;
    if (!readData(0x11, status)) return;
    if (!dmVersionKnown(status)) {
      // No module's (all ones): the link may have dropped - a CH32L103 drops it at every change of hart state, its own
      // restarts too - and back-to-back polls leave the PHY no idle time to revive it. The bus is brought back in step
      // (the wire's configuration sequence, no debug-module register written); it read all ones until wire_lost_ms
      // closed the stream.
      phy_.reinit();
      return;
    }
    // read twice, DMCONTROL between (a missed second read gives DMCONTROL's value, no module's): a missed read gives the
    // read before it - a mailbox word that looks like a running module's had the console read and answer DATA0 over a
    // halted hart's abstract-command operands, one that looks halted stopped it for kStatusMs. Not agreeing: asked again
    // at the next poll.
    if (!readData(0x10, control) || !readData(0x11, again) || again != status) {
      last_status_ms_ = millis() - kStatusMs;
      return;
    }
    if (status & (3u << 18)) {   // havereset: the target restarted on its own
      // Only once ackHaveReset's own read confirms it: one bad read with bit 18 set unsynced dmseq, which dropped the
      // input chunk on its way (up to 2 bytes of a command the target then never answered) and took the next repeat of
      // a frame for a new one (its bytes twice).
      if (dm_.ackHaveReset()) unsync();
      return;
    }
    hart_halted_ = (status & (1u << 9)) != 0;
    // a running hart is running for the ops too (a halted one is brought in line by their checkHalted, which also
    // re-syncs the link after the change of state)
    if (!hart_halted_) dm_.noteRunning();
  } else if (dm_.halted()) {
    hart_halted_ = true;   // the probe halted it since the last look
  }
  if (hart_halted_) return;
  if (mechanism_ == 1) pollDmdata();
  else if (mechanism_ == 2) pollSeq();
  else pollSdi();
}

// SerialSDI: the target waits for DATA0 to read zero, writes DATA1 = bytes 3..6 and
// DATA0 = length | bytes 0..2 << 8, and we zero DATA0 once we have the frame.
void DmConsole::pollSdi() {
  uint32_t data0 = 0;
  if (!readData(0x04, data0)) return;
  const uint8_t length = static_cast<uint8_t>(data0 & 0xff);
  if (length == 0 || length > 7) return;       // 0 = nothing waiting; anything else is not a frame
  uint32_t data1 = 0;
  if (!confirm(data0, true, data1, 0x11)) return;   // the frame read twice (DATA1 too: §3.1 reads it for any L 1-7)
  const uint8_t bytes[7] = {
      static_cast<uint8_t>(data0 >> 8), static_cast<uint8_t>(data0 >> 16), static_cast<uint8_t>(data0 >> 24),
      static_cast<uint8_t>(data1), static_cast<uint8_t>(data1 >> 8),
      static_cast<uint8_t>(data1 >> 16), static_cast<uint8_t>(data1 >> 24)};
  for (uint8_t i = 0; i < length; ++i) push(bytes[i]);
  phy_.write(0x04, 0);                         // taken: this is what the target waits for
}

// SerialDMDATA (oep-if-console §3.2). The status byte is the low byte of DATA0: bit 7 says the word is the target's
// (T), bits 0-5 are L = the byte count + 4. Every target slot is answered exactly once, and the answer is also the
// probe's outgoing frame when the send queue holds bytes - three at a time, since only DATA0 carries host payload - or
// zero when it is empty. A word with bit 7 clear is the probe's own answer or 0: never written over.
void DmConsole::pollDmdata() {
  uint32_t data0 = 0;
  if (!readData(0x04, data0)) return;
  if (!(data0 & 0x80u)) {
    // Bit 7 clear: our own answer not yet replaced, or 0. Not ours to touch - writing into a clear word races the
    // target, which may be posting its slot at that moment; on a CH32X035 that ate the host's frames and PING never
    // came back (2026-09-23).
    saw_empty_ = false;
    return;
  }
  const uint32_t length = data0 & 0x3fu;   // bit 6 ignored
  // a word with bit 7 is acted on whatever its L (bytes taken and answered, the empty slot's sighting, no slot cleared):
  // read twice first (confirm), DATA1 too when the slot reaches it
  const bool slot = length >= 5 && length <= 11;
  uint32_t data1 = 0;
  if (!confirm(data0, slot && length - 4u >= 4, data1, 0x10)) return;
  if (slot) {                              // a target slot with n = L - 4 bytes: 0-2 in DATA0, 3-6 in DATA1 (after)
    const uint32_t count = length - 4u;
    const uint8_t bytes[7] = {
        static_cast<uint8_t>(data0 >> 8), static_cast<uint8_t>(data0 >> 16), static_cast<uint8_t>(data0 >> 24),
        static_cast<uint8_t>(data1), static_cast<uint8_t>(data1 >> 8),
        static_cast<uint8_t>(data1 >> 16), static_cast<uint8_t>(data1 >> 24)};
    for (uint32_t i = 0; i < count; ++i) push(bytes[i]);
    saw_empty_ = false;
    answerDmdata();
    return;
  }
  if (length != 4) {
    // L 0-3 or 12-63 with bit 7: not a target slot (a value a probe or another debugger left - 0xffffffff, say). It
    // carries no bytes and gets 0, no input on it: the queue's bytes stay for a real slot. It took 7 bytes from such
    // a word (L 12-63: 0xff each) and answered L 0-3 like an empty slot, with input.
    saw_empty_ = false;
    phy_.write(0x04, 0);
    return;
  }
  // L = 4, the target's empty slot: "the mailbox is the probe's". The spec leaves when to answer it to the probe; it is
  // answered once it has stood a whole poll - a target that still replaced its own empty slot with one carrying bytes
  // (as some once did) loses nothing then (CH32X035 at 48 MHz behind the P4's fast PHY: every other frame vanished,
  // 2026-09-23). A target that waits for the answer, as §3.2 now says it does, gets it one poll later.
  if (!saw_empty_) {
    saw_empty_ = true;
    return;
  }
  saw_empty_ = false;
  answerDmdata();
}

// The answer to one target slot: up to 3 bytes from the head of the send queue (they leave it), or 0.
void DmConsole::answerDmdata() {
  if (!tx_count_) {
    phy_.write(0x04, 0);
    return;
  }
  uint8_t p[3] = {0, 0, 0};
  const uint8_t chunk = tx_count_ > 3 ? 3 : static_cast<uint8_t>(tx_count_);
  for (uint8_t i = 0; i < chunk; ++i) p[i] = take();
  phy_.write(0x04, uint32_t(chunk + 4u) | (uint32_t(p[0]) << 8) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 24));
}

// dmseq (mechanism 2), oep-spec docs/target-console-dmseq.ja.md. SerialDMDATA's carrier with
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

uint8_t DmConsole::crc8(const uint8_t *p, size_t n) {   // poly 0x07, init 0xFF
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
  ++stats_.polls;
  if (!readData(0x04, w0)) return;
  if (!(w0 & 0x80u)) return;                   // our answer still there, or nothing yet
  ++stats_.frames;
  const uint8_t n = w0 & 0x07u;
  // read twice (confirm), DATA1 too when the frame reaches it: the CRC-8 alone let one stale word in 256 through (a
  // DATA1 read missed gives DATA0's word, a DATA0 read missed the last word of the poll before)
  if (!confirm(w0, n >= 3, w1, 0x10)) return;
  if (seqFault()) w0 ^= 1u << (8 + (w0 & 7));  // test hook: a frame read corrupted
  const uint8_t b[8] = {static_cast<uint8_t>(w0), static_cast<uint8_t>(w0 >> 8), static_cast<uint8_t>(w0 >> 16),
                        static_cast<uint8_t>(w0 >> 24), static_cast<uint8_t>(w1), static_cast<uint8_t>(w1 >> 8),
                        static_cast<uint8_t>(w1 >> 16), static_cast<uint8_t>(w1 >> 24)};
  // N before the CRC: at N = 7 there is no byte 1+N, and 0xffffffff - what an attach leaves
  // in DATA0 - decodes to exactly that.
  if (n > 6 || crc8(b, static_cast<size_t>(1 + n)) != b[1 + n]) {
    ++stats_.invalid;
    // Usually a bad read, and the next poll reads it right. If it stays bad the word may be
    // our own answer, corrupted into a shape with bit 7 set, and then both sides wait.
    // Answering K = the last S we accepted is safe whatever is there: if the target's
    // outstanding frame is that one, it was ours already; if it is a new one, K does not
    // match and the target posts it again.
    // The count (dmseq host rule 1, DS-5) stops at 3 while not synced, which never answers; it restarts at a valid
    // frame, after this answer, and when a session starts (start, unsync).
    if (seq_bad_run_ < 3) ++seq_bad_run_;
    if (seq_bad_run_ >= 3 && seq_synced_) { seq_bad_run_ = 0; seqAnswer(seq_last_s_, false); }
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
    // TO: the target gave up waiting on this frame, and what it wrote after it until an answer came was discarded -
    // mark lost 4 (the target's TO) where that output would have been (oep-if-common §1.3, dmseq "Timeout")
    if ((w0 & 0x40u) && mark_sink_)
      mark_sink_(sink_ctx_, v1::reg::common::kMarkKindLost, v1::reg::common::kMarkDetailLostTargetTimeout);
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
    while (seq_chunk_len_ < 2 && tx_count_) seq_chunk_[seq_chunk_len_++] = take();   // the queue's head (dmseq host rule 5)
  }
  const uint8_t m = with_data ? seq_chunk_len_ : 0;
  uint8_t ans[4] = {static_cast<uint8_t>((k << 5) | (seq_h_ << 4) | m), 0, 0, 0};
  for (uint8_t i = 0; i < m; ++i) ans[1 + i] = seq_chunk_[i];
  ans[1 + m] = crc8(ans, static_cast<size_t>(1 + m));   // right after the payload
  uint32_t answer = uint32_t(ans[0]) | (uint32_t(ans[1]) << 8) | (uint32_t(ans[2]) << 16) | (uint32_t(ans[3]) << 24);
  if (seqFault()) return;                                   // test hook: the answer does not land
  if (seqFault()) answer ^= 1u << (answer % 24);            // test hook: it lands corrupted
  phy_.write(0x04, answer);
  ++stats_.answers;
}

// Every start is a fresh session, even over one that is still open: a runner that moves from one sketch to the next
// reprograms the target in between, and bytes queued for the last sketch must not be delivered to this one (the queue
// is the stream's and goes with it when it closes, oep-if-console §2).
//
// Nothing is written to the mailbox here and nothing read is thrown away (oep-if-console §3): what is in DATA0 is read by
// the mechanism's own rules. dmseq writes DATA0 only while bit 7 is set: zeroing it at the start broke the frame the
// target had out, which then waited out its timeout (up to 1 s per try) before posting again (oep-spec
// probe-cdc-and-persistence §7.5, ch32rv's review); its framing sorts out an earlier session by itself. SDI / DMDATA
// zeroed DATA0 and threw away four polls: that wrote over a bit-7-clear word and dropped the target's output; a leftover
// word that is no slot (all ones) is cleared by DMDATA's own rule, and SDI leaves one alone (§3.1).
bool DmConsole::start(uint8_t mechanism) {
  if (mechanism > 2 || !dm_.attach()) return false;
  tx_tail_ = tx_count_ = 0;
  saw_empty_ = false;
  seq_synced_ = false;
  seq_last_syn_ = false;
  seq_bad_run_ = 0;
  seq_syn_drops_ = 0;
  seq_chunk_len_ = 0;
  seq_resyncs_ = 0;
  enabled_ = true;
  lost_ = false;
  hart_halted_ = false;
  last_status_ms_ = millis();
  mechanism_ = mechanism;
  return true;
}

// Into the send queue (oep-if-console §2): as much of `data` as the free space takes, in order. Nothing on SDI (one way).
size_t DmConsole::queue(const uint8_t *data, size_t length) {
  if (!enabled_ || mechanism_ == 0) return 0;
  size_t queued = 0;
  while (queued < length && tx_count_ < kSendQueue) {
    tx_[(tx_tail_ + tx_count_) % kSendQueue] = data[queued++];
    ++tx_count_;
  }
  return queued;
}

}  // namespace oep
