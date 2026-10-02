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
  DmConsole(Ch32Dm &dm, DmiPhy &phy) : dm_(dm), phy_(phy) {}
  // Where the bytes from the target go (the v1 stream's buffer). Set before start().
  void setSink(Sink sink, void *ctx) { sink_ = sink; sink_ctx_ = ctx; }
  // Call from loop(). Collects at most one frame, and only while the target is attached and running: those two
  // registers are where abstract commands put their operands (oep-if-console §2: the reading stops while the hart is
  // halted, which DMSTATUS says - a host that halted it through raw DMI counts too; every kStatusMs it is asked).
  void poll();
  // Start a fresh session in `mechanism` (whatever an earlier one left in the mailbox is thrown away), stop, queue
  // bytes for the target.
  bool start(uint8_t mechanism);
  void stop() { enabled_ = false; }
  bool enabled() const { return enabled_; }
  size_t queue(const uint8_t *data, size_t length);
  size_t room() const { return kTxCapacity - 1 - pending(); }   // what queue() takes now
  // The mechanism's one send slot (oep-if-common §1.4): what a write may hand over right now - the queue's room, and
  // nothing on a one-way mechanism (SDI).
  size_t slot() const { return mechanism_ == 0 ? 0 : room(); }
  // How many times the target's side (re)synchronised (dmseq SYN): after the first, a target restart.
  uint32_t resyncs() const { return seq_resyncs_; }
  // The target restarted (havereset seen while reading): dmseq goes back to unsynced (oep-if-console §2).
  void unsync() { seq_synced_ = false; seq_chunk_len_ = 0; seq_bad_run_ = 0; }   // a new dmseq session
  // The wire was found lost while reading (reads got nothing back for wire_lost_ms of real time with no answer between,
  // oep-if-debug §2: the PHY's wire-loss clock): the stream marks link-lost and the connection closes. Cleared by start().
  bool lineLost() const { return lost_; }
  static constexpr uint32_t kStatusMs = 20;
  // dmseq diagnostics: polls that read a word with bit 7 set, of those the invalid ones, answers written
  struct SeqStats { uint32_t polls, frames, invalid, answers; };
  SeqStats seqStats() const { return stats_; }

 private:
  Ch32Dm &dm_;
  DmiPhy &phy_;
  static constexpr size_t kTxCapacity = 256;
  bool enabled_ = false;
  uint32_t last_status_ms_ = 0;         // when DMSTATUS was last read (halted? havereset?)
  bool hart_halted_ = false;            // what it said
  bool lost_ = false;
  uint8_t mechanism_ = 0;               // 0 = SerialSDI (one way), 1 = SerialDMDATA, 2 = dmseq (two way)
  bool saw_empty_ = false;              // the target's empty frame was already there last poll
  bool discarding_ = false;             // start(): what arrives now is an earlier session's
  Sink sink_ = nullptr;
  void *sink_ctx_ = nullptr;
  uint16_t tx_head_ = 0, tx_tail_ = 0;
  uint32_t last_attach_ms_ = 0;
  uint8_t tx_[kTxCapacity];
  uint16_t pending() const { return static_cast<uint16_t>((tx_head_ - tx_tail_ + kTxCapacity) % kTxCapacity); }
  void push(uint8_t byte);
  bool readData(uint8_t address, uint32_t &value);   // a DMI read that keeps the line-lost clock
  void pollSdi();
  void pollDmdata();
  void sendOrClear();
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
