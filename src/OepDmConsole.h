// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The target's console through the debug module's data registers (SerialSDI, SerialDMDATA, dmseq), over Ch32Dm.
// The oep.target.console stream (TargetConsoleStream) drives it and owns the only buffer: every byte goes straight into
// the stream's sink.
#pragma once

#include "OepCh32Dm.h"

namespace oep {

// A console the target writes through the debug module's own data registers - no UART, no
// pin, no wiring, and the hart is never halted for it. The target blocks until the probe
// zeroes DATA0, so nothing is lost as long as somebody is collecting; bytes that arrive
// with no room left in the stream overwrite its oldest (the stream reports that as a gap).
class DmConsole {
 public:
  using Sink = void (*)(void *ctx, uint8_t byte);
  using MarkSink = void (*)(void *ctx, uint8_t kind, uint8_t detail);
  DmConsole(Ch32Dm &dm, DmiPhy &phy) : dm_(dm), phy_(phy) {}
  // Where the bytes from the target go (the v1 stream's buffer), and the marks the framing finds among them (a dmseq
  // frame with TO: mark lost 4 after its payload, oep-if-common §1.3). Set before start().
  void setSink(Sink sink, void *ctx, MarkSink mark_sink = nullptr) { sink_ = sink; sink_ctx_ = ctx; mark_sink_ = mark_sink; }
  // Call from loop(). Collects at most one frame, and only while the target is attached and running: those two
  // registers are where abstract commands put their operands (oep-if-console §3: the reading stops while the hart is
  // halted, which DMSTATUS says - a host that halted or resumed it through raw DMI counts too; every kStatusMs it is
  // asked, and a running hart there clears Ch32Dm's halted view).
  void poll();
  // Start a fresh session in `mechanism` (an empty send queue; nothing written to the mailbox), stop (the queue goes
  // with the stream), queue bytes for the target.
  bool start(uint8_t mechanism);
  void stop() { enabled_ = false; tx_tail_ = tx_count_ = 0; }
  bool enabled() const { return enabled_; }
  // The send queue (oep-if-console §2, describe send_queue): a write and a bind's input put what fits of their data at
  // its end; the probe feeds the target from its head at the mechanism's pace - up to 2 bytes on each answer to a dmseq
  // frame, 3 on each answer to a DMDATA slot. None on SDI (one way): queue() takes nothing there.
  // A write took at most the mechanism's send slot (2 / 3 bytes, 0 while it held one): a line of input then cost a
  // request every 2-3 bytes, and the bench saw a classic ESP32's console commands 3x slower, replies lost on a CH32V003
  // and captures armed before a command miss its burst; the whole queue, as 0.0.28 had it, fixed all three.
  static constexpr uint16_t kSendQueue = 256;
  static_assert(kSendQueue >= v1::reg::kLimitConsoleSendQueueMinBytes, "send_queue below the registry's minimum");
  size_t queue(const uint8_t *data, size_t length);
  size_t room() const { return enabled_ && mechanism_ != 0 ? kSendQueue - tx_count_ : 0; }   // what queue() takes now
  // How many times the target's side (re)synchronised (dmseq SYN): after the first, a target restart.
  uint32_t resyncs() const { return seq_resyncs_; }
  // The target restarted (havereset seen while reading): dmseq goes back to unsynced (oep-if-console §2).
  void unsync() { seq_synced_ = false; seq_chunk_len_ = 0; seq_bad_run_ = 0; }   // a new dmseq session
  // The wire was found lost while reading (reads got nothing back for wire_lost_ms of real time with no answer between,
  // oep-if-debug §2: the PHY's wire-loss clock): the stream marks link-lost and the connection closes. Cleared by start().
  bool lineLost() const { return lost_; }
  static constexpr uint32_t kStatusMs = v1::reg::kLimitConsoleDmstatusPollMs;   // 20 ms (oep-if-console §3)
  // dmseq's CRC-8 (target-console-dmseq: poly 0x07, init 0xFF, no reflection, no final XOR)
  static uint8_t crc8(const uint8_t *p, size_t n);
  // dmseq diagnostics: polls that read a word with bit 7 set, of those the invalid ones, answers written
  struct SeqStats { uint32_t polls, frames, invalid, answers; };
  SeqStats seqStats() const { return stats_; }

 private:
  Ch32Dm &dm_;
  DmiPhy &phy_;
  bool enabled_ = false;
  uint32_t last_status_ms_ = 0;         // when DMSTATUS was last read (halted? havereset?)
  bool hart_halted_ = false;            // what it said
  bool lost_ = false;
  uint8_t mechanism_ = 0;               // 0 = SerialSDI (one way), 1 = SerialDMDATA, 2 = dmseq (two way)
  bool saw_empty_ = false;              // the target's empty frame was already there last poll
  Sink sink_ = nullptr;
  MarkSink mark_sink_ = nullptr;
  void *sink_ctx_ = nullptr;
  uint16_t tx_tail_ = 0, tx_count_ = 0;   // the send queue: its head, and how many bytes wait
  uint32_t last_attach_ms_ = 0;
  uint8_t tx_[kSendQueue];
  uint8_t take();                       // the queue's head byte, out of the queue
  void push(uint8_t byte);
  bool readData(uint8_t address, uint32_t &value);   // a DMI read that keeps the line-lost clock
  void pollSdi();
  void pollDmdata();
  void answerDmdata();
  // mechanism 2, dmseq (oep-spec docs/target-console-dmseq.ja.md)
  void pollSeq();
  void seqAnswer(uint8_t k, bool with_data);
  bool seq_synced_ = false;             // a target frame has been accepted this session
  uint8_t seq_last_s_ = 0;              // S of the last accepted target frame
  bool seq_last_syn_ = false;           // the last accepted target frame had SYN set
  uint8_t seq_h_ = 0;                   // H of the outstanding host payload
  uint8_t seq_chunk_[2] = {0, 0};
  uint8_t seq_chunk_len_ = 0;           // 0 = nothing outstanding
  uint8_t seq_bad_run_ = 0;             // consecutive invalid words
  uint8_t seq_syn_drops_ = 0;           // test hook OEP_CONSOLE_FAULT_SYN
  uint32_t seq_resyncs_ = 0;
  SeqStats stats_ = {0, 0, 0, 0};
};

}  // namespace oep
