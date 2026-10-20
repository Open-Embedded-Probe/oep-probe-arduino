// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepCapture.h"

// OEP_HOST_FAKE_PARLIO: a host test with fakes of the PARLIO RX driver and the heap (tests/host/shim)
#if (defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)) || defined(OEP_HOST_FAKE_PARLIO)

#include <esp_cache.h>
#include <esp_cpu.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <hal/hal_utils.h>
#include <soc/hp_sys_clkrst_struct.h>
#include <string.h>

#include "OepEndpoint.h"

namespace oep {
namespace {

namespace cap = reg::fixture_logic;
// configure / query TLVs (oep-spec oep-if-capture §3.3); bit 7 of the tag = critical
enum : uint8_t { kTagMode = cap::kTlvConfigureMode, kTagRate = cap::kTlvConfigureRate,
                 kTagSamples = cap::kTlvConfigureSamples, kTagSegments = cap::kTlvConfigureSegments,
                 kTagTrigger = cap::kTlvConfigureTrigger, kTagPretrigger = cap::kTlvConfigurePretrigger };
enum : uint8_t { kTagActualRate = cap::kTlvConfigureAnswerActualRate, kTagLayout = cap::kTlvConfigureAnswerLayout,
                 kTagActualSamples = cap::kTlvConfigureAnswerActualSamples,
                 kTagActualSegments = cap::kTlvConfigureAnswerActualSegments };
enum : uint8_t { kEventSegment = cap::kEventSegment, kEventStopped = cap::kEventStopped };
enum : uint8_t { kStoppedComplete = cap::kStoppedReasonComplete, kStoppedHost = cap::kStoppedReasonHost };

uint8_t widthFor(uint8_t channels) {
  uint8_t w = 1;
  while (w < channels) w <<= 1;
  return w;
}

uint32_t gcd(uint64_t a, uint64_t b) {
  while (b) { const uint64_t t = a % b; a = b; b = t; }
  return static_cast<uint32_t>(a);
}

// The rate 160 MHz / (int + numerator / denominator) as the answer's num / den (u32 each). A large fractional
// denominator does not fit even reduced (14.886 MHz asked was answered 2685163520/1623 - the numerator cut to
// 32 bits - while it sampled at 14.89 MHz, 2026-09-29): whole Hz then, off by less than 0.5 Hz.
void rateFraction(uint64_t top, uint64_t bottom, uint32_t &num, uint32_t &den) {
  const uint64_t g = gcd(top, bottom);
  if (top / g > UINT32_MAX || bottom / g > UINT32_MAX) {
    num = static_cast<uint32_t>((top + bottom / 2) / bottom);
    den = 1;
  } else {
    num = static_cast<uint32_t>(top / g);
    den = static_cast<uint32_t>(bottom / g);
  }
}

}  // namespace

bool IRAM_ATTR LogicCapture::partialReceive(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *e, void *context) {
  auto *self = static_cast<LogicCapture *>(context);
  self->produced_ += e->recv_bytes;
  // where in the ring it is: on from the last chunk's end, by what the DMA skipped after an early close (Chunk::ring)
  const uint32_t prev = self->ring_end_;
  const uint32_t off = static_cast<uint32_t>(static_cast<const uint8_t *>(e->data) - self->rx_ring_);
  uint32_t ring = (prev & ~(self->rx_ring_bytes_ - 1)) + off;
  if (static_cast<int32_t>(ring - prev) < 0) ring += self->rx_ring_bytes_;
  self->ring_end_ = ring + static_cast<uint32_t>(e->recv_bytes);
  const Chunk chunk = {static_cast<const uint8_t *>(e->data), e->recv_bytes, self->produced_, ring};
  BaseType_t woken = pdFALSE;
  if (xQueueSendFromISR(self->queue_, &chunk, &woken) != pdTRUE) self->queue_overflow_ = self->queue_overflow_ + 1;
  return woken == pdTRUE;
}

// Copy one finished DMA chunk into the segment being filled; without a free segment the bytes are dropped and the
// next segment starts with a gap mark. Runs on the harvest task only.
void LogicCapture::harvest(const Chunk &chunk) {
  if (lost_) return;                           // stopped taking data (capture §2.2): poll() ends the track
  if (rep_trig_) { harvestRepeatTriggered(chunk); return; }
  if (missed(chunk)) { loseSegment(); return; }   // chunks lost before this one: the segment has a hole
  size_t at = 0;
  while (at < chunk.length) {
    if (completed_ - released_ >= segment_count_) {   // every segment is waiting for the host
      const size_t rest = chunk.length - at;
      dropped_ += rest;
      captured_ += rest;
      if (fill_ == 0) gap_pending_ = true;
      paused_ = true;
      return;
    }
    if (paused_ && fill_ == 0) paused_ = false;
    const uint32_t slot = fill_slot_;
    const size_t n = min(static_cast<size_t>(segment_bytes_ - fill_), chunk.length - at);
    if (fill_ == 0) {   // a new segment: remember where in time it starts
      Info &info = infos_[completed_ % kInfos];
      info.serial = completed_;
      info.position = captured_;   // one position space with the discarded bytes (capture §2.2, write_pos §3.2)
      const uint64_t first_sample = captured_ * 8 / width_;
      info.start_ns = start_ns_ + nsOf(first_sample);
      info.flags = gap_pending_ ? 1 : 0;
      info.slot = slot;
      info.trigger_index = 0xFFFFFFFFu;
      gap_pending_ = false;
    }
    memcpy(store_ + static_cast<size_t>(slot) * segment_bytes_ + fill_, chunk.data + at, n);
    if (!ringIntact(chunk.ring + static_cast<uint32_t>(at))) {   // the DMA came round and rewrote these bytes while (or before) we copied
      overruns_ = overruns_ + 1;
      loseSegment();   // a hole in the segment (capture §2.2)
      return;
    }
    __atomic_thread_fence(__ATOMIC_RELEASE);
    fill_ += n;
    at += n;
    captured_ += n;
    if (fill_ == segment_bytes_) finishSegment(segment_bytes_, infos_[completed_ % kInfos].flags);
  }
}

// A repeat with a trigger (capture §2.1): while waiting, each chunk is mapped and searched; a trigger begins a segment
// pretrigger samples before it (not before the last segment's end, nor the oldest the ring still holds), copied through
// the map, and the chunks after it fill it as a plain repeat's. Once full: rearm 1 waits for the next segment's trigger
// from its end (the rest of this chunk searched at once), rearm 0 goes on with the next segment straight after. A
// chunk lost to the queue while waiting only loses history; inside a segment it is a hole (§2.2). Harvest task only.
void LogicCapture::harvestRepeatTriggered(const Chunk &chunk) {
  if (const uint32_t skip = missed(chunk)) {           // chunks lost before this one
    if (!rep_wait_) { loseSegment(); return; }
    mapLost(captured_, skip, chunk.ring);
    captured_ += skip;
    have_level_ = false;
  }
  const uint64_t base = captured_;                     // the stream byte of chunk.data[0]
  addSpan(base, chunk.ring, static_cast<uint32_t>(chunk.length));
  if (!ringIntact(chunk.ring)) {                       // rewritten before it was looked at
    overruns_ = overruns_ + 1;
    if (!rep_wait_) { loseSegment(); return; }
    captured_ = base + chunk.length;
    have_level_ = false;
    return;
  }
  size_t at = 0;
  // a segment its trigger began in this chunk: its info is the trigger's, even with none of its bytes copied yet (it
  // starts inside this chunk) - not rearm 0's (P4X f8b985e: such segments came out with no trigger_index, flags 0)
  bool begun = false;
  while (at < chunk.length) {
    if (rep_wait_) {
      if (!harvestRepeatTrigger(chunk, base, at)) break;   // none in the rest of the chunk
      if (lost_) return;
      begun = !rep_wait_;
      continue;
    }
    if (completed_ - released_ >= segment_count_) {   // rearm 0's next segment, every one waiting for the host
      const size_t rest = chunk.length - at;
      dropped_ += rest;
      data_pos_ += rest;                              // the discarded bytes take positions (§2.2), as a plain repeat's
      if (fill_ == 0) gap_pending_ = true;
      paused_ = true;
      break;
    }
    if (paused_ && fill_ == 0) paused_ = false;
    if (fill_ == 0 && !begun) {   // rearm 0: the next segment straight after the last one
      Info &info = infos_[completed_ % kInfos];
      info.serial = completed_;
      info.position = data_pos_;
      info.start_ns = start_ns_ + nsOf((base + at) * 8 / width_);
      info.flags = gap_pending_ ? cap::kSegmentFlagGap : 0;
      info.slot = fill_slot_;
      info.trigger_index = 0xFFFFFFFFu;
      gap_pending_ = false;
    }
    const size_t n = min(static_cast<size_t>(segment_bytes_ - fill_), chunk.length - at);
    memcpy(store_ + static_cast<size_t>(fill_slot_) * segment_bytes_ + fill_, chunk.data + at, n);
    if (!ringIntact(chunk.ring + static_cast<uint32_t>(at))) {   // the DMA came round while we copied: a hole
      overruns_ = overruns_ + 1;
      loseSegment();
      return;
    }
    __atomic_thread_fence(__ATOMIC_RELEASE);
    fill_ += n;
    at += n;
    if (fill_ == segment_bytes_) {
      finishSegment(segment_bytes_, infos_[completed_ % kInfos].flags);
      begun = false;
      floor_sample_ = (base + at) * 8 / width_;
      if (rearm_) {   // the next segment's own trigger, looked for from this one's end (arm 1: once its pretrigger is in)
        rep_wait_ = true;
        arm_from_ = floor_sample_ + (arm_ ? pre() : 0);
        level_false_ = false;
        const uint8_t k = trig_role_;   // the level before the floor: the last sample's (an edge right at it)
        last_level_ = width_ == 16 ? (chunk.data[at - 2 + (k >> 3)] >> (k & 7)) & 1 : (chunk.data[at - 1] >> k) & 1;
        have_level_ = true;
      }
    }
  }
  captured_ = base + chunk.length;
}

// Waiting for a segment's trigger in chunk [at, end): found (or a force's sample reached) -> the segment begun and `at`
// where its filling goes on in the chunk; true. None: `at` at the chunk's end, false. No free segment: paused, the
// arming starts again after this chunk (§2.1: the next trigger looked for once there is room).
bool LogicCapture::harvestRepeatTrigger(const Chunk &chunk, uint64_t base, size_t &at) {
  const uint64_t end_sample = (base + chunk.length) * 8 / width_;
  if (completed_ - released_ >= segment_count_) {
    paused_ = true;
    gap_pending_ = true;
    floor_sample_ = end_sample;
    arm_from_ = floor_sample_ + (arm_ ? pre() : 0);
    level_false_ = false;
    have_level_ = false;
    at = chunk.length;
    return false;
  }
  if (paused_) paused_ = false;
  const uint64_t first = (base + at) * 8 / width_;
  uint64_t t = first;
  if (force_) {   // a force: the sample of that instant, once the stream has come that far (capture §3.3)
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    t = force_sample_;
    if (t >= end_sample) { at = chunk.length; return false; }
    if (t < floor_sample_) t = floor_sample_;
    force_ = false;
  } else if (!findTrigger(chunk.data + at, chunk.length - at, first, t)) {
    at = chunk.length;
    return false;
  }
  beginTriggeredSegment(t, base);
  if (lost_) return true;
  // where it goes on in the chunk: the segment's next byte, or (done at once from the map) the arming's floor
  const uint64_t next = rep_wait_ ? floor_sample_ * width_ / 8 : seg_first_sample_ * width_ / 8 + fill_;
  at = next > base ? static_cast<size_t>(next - base) : 0;
  return true;
}

// The segment of a trigger at sample `trigger`: from pretrigger samples before it - not before the last segment's end
// (floor_sample_) nor the oldest byte the ring holds in order - its bytes before this chunk (stream byte `base`)
// copied through the map; its info, rearm 1's record and the event for loop(). Harvest task only.
void LogicCapture::beginTriggeredSegment(uint64_t trigger, uint64_t base) {
  uint64_t s0 = trigger > pre() ? trigger - pre() : 0;
  if (s0 < floor_sample_) s0 = floor_sample_;
  const uint64_t oldest = oldestKept();
  if (s0 * width_ / 8 < oldest) s0 = (oldest * 8 + width_ - 1) / width_;
  if (s0 > trigger) s0 = trigger;
  const uint32_t index = static_cast<uint32_t>(trigger - s0);
  const bool contiguous = completed_ == 0 ? true : s0 == floor_sample_;
  Info &info = infos_[completed_ % kInfos];
  info.serial = completed_;
  info.position = data_pos_;
  info.start_ns = start_ns_ + nsOf(s0);
  info.flags = static_cast<uint8_t>((gap_pending_ || !contiguous ? cap::kSegmentFlagGap : 0) |
                                    (rearm_ ? cap::kSegmentFlagRearm : 0));   // its own trigger (§2.1)
  info.slot = fill_slot_;
  info.trigger_index = index;
  gap_pending_ = false;
  seg_first_sample_ = s0;
  fill_ = 0;
  const uint64_t from = s0 * width_ / 8;
  if (from < base) {   // the part before this chunk, through the map
    const uint64_t to = from + segment_bytes_ < base ? from + segment_bytes_ : base;
    if (!copyStreamTo(store_ + static_cast<size_t>(fill_slot_) * segment_bytes_, from, to)) { loseSegment(); return; }
    __atomic_thread_fence(__ATOMIC_RELEASE);
    fill_ = static_cast<uint32_t>(to - from);
  }
  uint32_t record = 0xFFFFFFFFu;
  if (rearm_ && records_) {   // a record for every trigger (§3.10)
    record = record_next_;
    records_[record % kTriggerRecords] = {trigger, start_ns_ + nsOf(trigger), record, periodNs(rate_num_, rate_den_)};
    __atomic_thread_fence(__ATOMIC_RELEASE);
    record_next_ = record + 1;
  }
  found_[found_head_ % kFound] = {trigger, completed_, index, record};
  __atomic_thread_fence(__ATOMIC_RELEASE);
  found_head_ = found_head_ + 1;
  rep_wait_ = false;
  rep_started_ = true;
  if (fill_ == segment_bytes_) {   // all of it already there (pretrigger as long as the segment): done at once
    finishSegment(segment_bytes_, info.flags);
    floor_sample_ = s0 + static_cast<uint64_t>(segment_bytes_) * 8 / width_;
    if (rearm_) { rep_wait_ = true; arm_from_ = floor_sample_ + (arm_ ? pre() : 0); level_false_ = false; }
  }
}

// loop(): the triggered events the harvest found (a repeat with a trigger), in order; the oldest beyond kFound gone.
void LogicCapture::reportFound() {
  const uint32_t head = found_head_;
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  if (head - found_tail_ > kFound) found_tail_ = head - kFound;
  for (; found_tail_ != head; ++found_tail_) {
    if (!subscribed_) continue;
    const TriggerFound &f = found_[found_tail_ % kFound];
    uint8_t e[kTriggeredBytes];
    putTriggered(e, f.serial, f.index, f.sample, start_ns_ + nsOf(f.sample), periodNs(rate_num_, rate_den_), generation_);
    putU32(e + 8, f.record);
    endpoint_.event(*this, cap::kEventTriggered, e, sizeof e);
  }
}

// Streaming, zero-copy transport: copy a finished DMA chunk from `at` into stages while the transport has one free;
// with none the chunk waits in the ring (the stream's buffer) and the harvest comes back to it. Bytes the DMA came
// round over before they were taken, or a chunk the queue had no room for, are dropped: the stream position skips them.
// Harvest task only.
size_t LogicCapture::harvestDirect(const Chunk &chunk, size_t at) {
  if (format_ == cap::kDataFormatLogicBlocks) { harvestBlocks(chunk); return chunk.length; }
  if (lost_) return chunk.length;
  if (at == 0) {
    if (const uint32_t m = missed(chunk)) dropDirect(m);   // chunks lost to the queue
    const uint32_t held = ring_end_ - chunk.ring;
    if (held > ring_peak_) ring_peak_ = held;
  }
  const bool copy = format_ != cap::kDataFormatTestCounter;   // format 4's bytes are written at the send
  while (at < chunk.length) {
    const size_t rest = chunk.length - at;
    if (!ringIntact(chunk.ring + static_cast<uint32_t>(at))) {   // the DMA came round over them: dropped, not a hole -
      overruns_ = overruns_ + 1;                                  // no segment (stage) had them yet
      dropDirect(rest);
      return chunk.length;
    }
    if (stage_cur_ >= 0 && stage_pos_ + stage_fill_ != captured_) sendStage();   // bytes dropped since: a new one
    if (stage_cur_ < 0 && !takeStage()) return at;
    const size_t n = min(static_cast<size_t>(stage_data_ - stage_fill_), rest);
    if (copy) memcpy(stage_[stage_cur_] + kPushHead + stage_fill_, chunk.data + at, n);
    if (!ringIntact(chunk.ring + static_cast<uint32_t>(at))) {   // while copied: the stage ends before them
      overruns_ = overruns_ + 1;
      dropDirect(rest);
      return chunk.length;
    }
    stage_fill_ += n;
    at += n;
    captured_ += n;
    if (stage_fill_ == stage_data_) sendStage();
  }
  return at;
}

void LogicCapture::dropDirect(size_t n) {
  stage_drops_ += n;
  dropped_ += n;
  captured_ += n;
}

void LogicCapture::dropAfter(size_t n) {
  if (!spill_used_ && !spill_drop_) { dropDirect(n); return; }
  stage_drops_ += n;
  dropped_ += n;
  spill_drop_ += n;   // the position jumps once the spill has gone (drainSpill)
}

// Chunk [at, end) into the spill (no stage free, or the spill holds bytes before it). -> where it got to: short when
// the spill is full (the ring holds the rest). Without a spill: at, unchanged. Harvest task only.
size_t LogicCapture::spillIn(const Chunk &chunk, size_t at) {
  if (!spill_ || lost_) return at;
  if (at == 0 && !spill_used_ && !spill_drop_) {}   // harvestDirect looked at this chunk's start already
  else if (at == 0) {
    if (const uint32_t m = missed(chunk)) dropAfter(m);   // chunks lost to the queue
    const uint32_t held = ring_end_ - chunk.ring;
    if (held > ring_peak_) ring_peak_ = held;
  }
  const bool copy = format_ != cap::kDataFormatTestCounter;   // format 4's bytes are written at the send
  while (at < chunk.length) {
    const size_t rest = chunk.length - at;
    if (!ringIntact(chunk.ring + static_cast<uint32_t>(at)) || spill_drop_) {   // rewritten, or after a drop: dropped
      if (!ringIntact(chunk.ring + static_cast<uint32_t>(at))) overruns_ = overruns_ + 1;
      dropAfter(rest);
      return chunk.length;
    }
    const size_t tail = (spill_head_ + spill_used_) % kSpillBytes;
    size_t n = kSpillBytes - spill_used_;
    if (!n) return at;
    if (n > kSpillBytes - tail) n = kSpillBytes - tail;
    if (n > rest) n = rest;
    if (copy) memcpy(spill_ + tail, chunk.data + at, n);
    if (!ringIntact(chunk.ring + static_cast<uint32_t>(at))) {   // while copied
      overruns_ = overruns_ + 1;
      dropAfter(rest);
      return chunk.length;
    }
    spill_used_ += n;
    if (spill_used_ > spill_peak_) spill_peak_ = static_cast<uint32_t>(spill_used_);
    at += n;
  }
  return at;
}

// The spill's bytes into the stages, oldest first, while one is free - kSpillDrainStep at most a call, so the harvest
// comes back to the DMA ring between (draining all of it while the host took stages fast kept the ring unharvested past
// its 26 ms: an overrun with 1.2 MB in the spill, P4X 826eee0); once it is empty, the drop put after it. -> it moved
// bytes (or emptied).
bool LogicCapture::drainSpill() {
  const bool copy = format_ != cap::kDataFormatTestCounter;
  size_t moved = 0;
  while (spill_used_ && moved < kSpillDrainStep) {
    if (stage_cur_ >= 0 && stage_pos_ + stage_fill_ != captured_) sendStage();
    if (stage_cur_ < 0 && !takeStage()) return moved != 0;
    size_t n = stage_data_ - stage_fill_;
    if (n > spill_used_) n = spill_used_;
    if (n > kSpillBytes - spill_head_) n = kSpillBytes - spill_head_;
    if (copy) memcpy(stage_[stage_cur_] + kPushHead + stage_fill_, spill_ + spill_head_, n);
    stage_fill_ += n;
    captured_ += n;
    spill_head_ = (spill_head_ + n) % kSpillBytes;
    spill_used_ -= n;
    moved += n;
    if (stage_fill_ == stage_data_) sendStage();
  }
  if (spill_used_) return true;
  spill_head_ = 0;
  if (spill_drop_) {   // the bytes dropped after it: the position jumps (a new stage, sendStage above)
    captured_ += spill_drop_;
    spill_drop_ = 0;
  }
  return true;
}

// Format 1 (capture §1.3), streaming over the stages: the chunk's bytes gather into the block; each full block is
// encoded into the stage, sent first when the block's worst would not fit. Harvest task only.
void LogicCapture::harvestBlocks(const Chunk &chunk) {
  if (lost_) return;
  if (missed(chunk)) { loseSegment(); return; }   // chunks lost to the queue: a hole (capture §2.2)
  const uint32_t block_bytes = plan_.block * width_ / 8;
  size_t at = 0;
  while (at < chunk.length) {
    const size_t n = min(static_cast<size_t>(block_bytes - block_fill_), chunk.length - at);
    memcpy(block_buf_ + block_fill_, chunk.data + at, n);
    if (!ringIntact(chunk.ring + static_cast<uint32_t>(at))) {   // the DMA rewrote these bytes
      overruns_ = overruns_ + 1;
      loseSegment();
      return;
    }
    block_fill_ += static_cast<uint32_t>(n);
    at += n;
    captured_ += n;
    if (block_fill_ == block_bytes) emitBlock(plan_.block);
  }
}

// One block of `samples` from block_buf_ into the stage (a new one when its worst would not fit). No stage free: the
// block is dropped - the stream's position jumps by its samples' bytes and the next block's start_level is its own
// first sample (not contiguous, §1.3).
void LogicCapture::emitBlock(uint32_t samples) {
  const size_t worst = blocks::worstBytes(plan_, samples);
  if (stage_cur_ >= 0 && stage_data_ - stage_fill_ < worst) sendStage();
  const uint32_t raw = static_cast<uint32_t>((static_cast<uint64_t>(samples) * width_ + 7) / 8);
  if (stage_cur_ < 0 && !takeStage()) {
    stage_drops_ += raw;
    dropped_ += raw;
    enc_pos_ += raw;
    sample_pos_ += samples;
    encoder_.reset();
    block_fill_ = 0;
    return;
  }
  uint8_t *at = stage_[stage_cur_] + kPushHead + stage_fill_;
  const uint32_t t0 = esp_cpu_get_cycle_count();
  const size_t n = encoder_.encode(plan_, block_buf_, samples, at, stage_data_ - stage_fill_, true);
  const uint32_t cycles = esp_cpu_get_cycle_count() - t0;
  if (cycles > enc_max_cycles_) enc_max_cycles_ = cycles;
  enc_cycles_ += cycles;
  ++enc_blocks_;
  enc_fallbacks_ += encoder_.fallbacks();
  stage_fill_ += static_cast<uint32_t>(n);
  stage_samples_ += samples;
  enc_pos_ += n;
  sample_pos_ += samples;
  block_fill_ = 0;
}

// A free stage becomes the current one, starting with the byte held back from the last frame, if any.
bool LogicCapture::takeStage() {
  const uint32_t free = stage_free_;
  if (free == 0) return false;
  stage_cur_ = __builtin_ctz(free);
  __atomic_fetch_and(&stage_free_, ~(1u << stage_cur_), __ATOMIC_ACQ_REL);
  stage_fill_ = 0;
  stage_pos_ = captured_;
  if (format_ == cap::kDataFormatLogicBlocks) {   // whole blocks, nothing held back (format 1)
    stage_pos_ = enc_pos_;
    stage_samples_ = 0;
    stage_first_sample_ = sample_pos_;
    stage_since_ = millis();
    return true;
  }
  stage_since_ = millis();
  return true;
}

// Hand the current stage to the transport as one push frame (or give it back if nobody may receive it now). A full
// stage is a whole number of 512-byte packets and the host's read runs on into the next one. A partial one (sent for
// max_delay_ms, or at stop) that happens to be whole packets would leave the host's read open (the direct build sends
// no zero-length packet), so a frame of length 0 (the transport's keepalive, 2 bytes: a short packet) follows it; so
// does the stream's end (closeTransfer).
// Every frame is one segment (capture §2.2: the probe's own cut, at a sample and a byte boundary) and carries it
// (§3.4 segment_info): its position, its samples, its start - the start's time plus its samples at the rate (not the
// clock read again: no step at the seams) - and the start's uncertainty.
void LogicCapture::sendStage() {
  if (stage_cur_ < 0) return;
  uint8_t *s = stage_[stage_cur_];
  const int index = stage_cur_;
  uint32_t fill = stage_fill_;
  stage_cur_ = -1;
  stage_fill_ = 0;
  const bool blocks = format_ == cap::kDataFormatLogicBlocks;
  uint16_t fn = 0, min_bytes = 0;
  uint32_t max_delay = 0;
  DirectTransport *t = endpoint_.direct();
  if (fill == 0 || !t || !endpoint_.directPush(*this, fn, min_bytes, max_delay)) {
    __atomic_fetch_or(&stage_free_, 1u << index, __ATOMIC_ACQ_REL);
    return;
  }
  // the message: push header, position(u64), len(u16), data, TLV 0x01 generation (core §11.2, oep-if-capture §3.4)
  putU16(s, static_cast<uint16_t>(kPushHead - 2 + fill + kPushTail));
  s[2] = kRolePush;
  putU16(s + 3, fn);
  putU16(s + 5, endpoint_.takeSeq(fn));
  putU64(s + 7, stage_pos_);
  putU16(s + 15, static_cast<uint16_t>(fill));
  if (format_ == cap::kDataFormatTestCounter) putTestCounter(s + kPushHead, stage_pos_, fill);
  putTlvHeader(s + kPushHead + fill, cap::kTlvDataGeneration, 4);
  putU32(s + kPushHead + fill + kTlvHeader, generation_);
  uint8_t *info = s + kPushHead + fill + kTlvHeader + 4;
  putTlvHeader(info, cap::kTlvDataSegmentInfo, kSegmentInfoBytes);
  putU64(info + kTlvHeader, stage_pos_);
  putU32(info + kTlvHeader + 8, blocks ? stage_samples_ : static_cast<uint32_t>(static_cast<uint64_t>(fill) * 8 / width_));
  putU64(info + kTlvHeader + 12, start_ns_ + nsOf(blocks ? stage_first_sample_ : stage_pos_ * 8 / width_));
  putU32(info + kTlvHeader + 20, kStartUncertaintyNs);
  const size_t length = kPushHead + fill + kPushTail;
  if (!t->queueData(s, length, stageDone, this)) {
    stage_unsent_ = stage_unsent_ + fill;   // the transport's queue full: these bytes go nowhere (status's diagnostics)
    __atomic_fetch_or(&stage_free_, 1u << index, __ATOMIC_ACQ_REL);
    return;
  }
  open_transfer_ = length % 512 == 0;
  if (open_transfer_ && fill < stage_data_) closeTransfer();   // a partial frame of whole packets: closed now
}

// The host's read of the last transfer ends only at a short packet: after whole packets, a frame of length 0 (2 bytes,
// the transport's keepalive, which a host skips) ends it.
void LogicCapture::closeTransfer() {
  static uint8_t zero[64] __attribute__((aligned(64))) = {};   // internal RAM, DMA-capable; never written
  DirectTransport *t = endpoint_.direct();
  if (!open_transfer_ || !t) return;
  if (t->queueData(zero, 2, nullptr, nullptr)) open_transfer_ = false;
}

void LogicCapture::stageDone(void *context, const uint8_t *buffer) {
  auto *self = static_cast<LogicCapture *>(context);
  for (uint8_t i = 0; i < self->stage_count_; ++i)
    if (self->stage_[i] == buffer) __atomic_fetch_or(&self->stage_free_, 1u << i, __ATOMIC_ACQ_REL);
}

uint32_t LogicCapture::streamBuffer(uint32_t samples, uint8_t width, uint32_t segments) const {
  if (!endpoint_.directCapable()) return static_cast<uint32_t>(static_cast<uint64_t>(samples) * width / 8 * segments);
  size_t frame = max_read_ + 16 + 2;
  if (frame > kStageFrameMax) frame = kStageFrameMax;
  frame = frame / 512 * 512;
  size_t count = stage_count_;
  if (!count) {   // not taken yet: as openStages would, frame + 48 KiB left over each
    size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    while (count < kStagesMax && largest >= frame + 48 * 1024) { ++count; largest -= frame; }
  }
  if (count < 2) return static_cast<uint32_t>(static_cast<uint64_t>(samples) * width / 8 * segments);   // the copied way
  // and the FIFO behind them (formats 0 and 4; held, or PSRAM to take it)
  const bool psram = streamRingAvailable();
  const size_t ring = format_ == cap::kDataFormatLogicBlocks ? 0 : (psram ? kStreamRingBytes : kRingBytes) - kDmaAhead - kChunkMax;
  // the spill behind the ring (formats 0 and 4): held, or PSRAM room for it beside the ring
  const bool spill = format_ != cap::kDataFormatLogicBlocks && psram &&
                     (spill_ || heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) >= kSpillBytes + (stream_ring_ ? 0 : kStreamRingBytes));
  return static_cast<uint32_t>(count * (frame - kPushHead - kPushTail) + ring + (spill ? kSpillBytes : 0));
}

bool LogicCapture::openStages() {
  // one frame per stage, a whole number of 512-byte packets within the frame limit, so the host's read runs on across
  // frames (every frame ending with a short packet cost gaps at 40-160 MHz: a host read completing per frame over
  // usbipd). The host bounds the latency with its read size (64 KiB: four frames).
  size_t frame = max_read_ + 16 + 2;
  if (frame > kStageFrameMax) frame = kStageFrameMax;
  frame = frame / 512 * 512;
  stage_data_ = static_cast<uint32_t>(frame - kPushHead - kPushTail);
  // Allocated once and kept: freeing and taking 8 x 16 KiB (and the 128 KiB ring) at every configure fragmented the
  // internal heap until the next streaming configure found no block (2026-09-25).
  if (stage_count_) {
    stage_free_ = (1u << stage_count_) - 1;
    stage_cur_ = -1;
    return stage_count_ >= 2;
  }
  for (size_t i = 0; i < kStagesMax; ++i) {
    if (heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL) < frame + 48 * 1024) break;   // leave room
    stage_[i] = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, frame, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (!stage_[i]) break;
    ++stage_count_;
  }
  stage_free_ = stage_count_ ? (1u << stage_count_) - 1 : 0;
  stage_cur_ = -1;
  if (!stream_ring_ && stage_count_ >= 2)   // the stream's DMA ring in PSRAM, kept like the stages (none: ring_)
    stream_ring_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, kStreamRingBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
  if (stream_ring_ && !spill_)   // and the spill behind it, when PSRAM has the room
    spill_ = static_cast<uint8_t *>(heap_caps_malloc(kSpillBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  return stage_count_ >= 2;
}

void LogicCapture::freeStages() {   // the stages stay allocated (openStages); wait until the transport gives them back
  if (!stage_count_) return;
  const uint32_t all = (1u << stage_count_) - 1;
  for (int i = 0; i < 500 && (stage_free_ & all) != all; ++i) delay(2);
}

// The stages' internal RAM back, for a store (streaming the copied way, a repeat): kept across configures otherwise, a
// stream's 8 x 27 KiB left no internal RAM for the next store - configure refused it storage_full (2026-10-20, P4X).
void LogicCapture::releaseStages() {
  freeStages();
  const uint32_t all = stage_count_ ? (1u << stage_count_) - 1 : 0;
  if ((stage_free_ & all) != all) return;   // still in the transport: kept
  for (uint8_t i = 0; i < stage_count_; ++i) {
    heap_caps_free(stage_[i]);
    stage_[i] = nullptr;
  }
  stage_count_ = 0;
  stage_free_ = 0;
  stage_cur_ = -1;
  heap_caps_free(stream_ring_);
  stream_ring_ = nullptr;
  heap_caps_free(spill_);
  spill_ = nullptr;
}

// The first sample of `data` (the stream's sample first_sample on) where the trigger holds: at, its index. With arm 1
// (trigger_opts, the default) a sample before the pretrigger has filled is only seen (an edge keeps its level), so the
// segment always has its pretrigger; with arm 0 one counts too (capture §3.3: the segment is then short, trigger_index
// smaller). A level with when 0 (the default) counts only after a sample not at it since arming. A byte whose samples
// cannot trigger (the channel's bits masked: all at the known level for an edge, none at the level wanted for a level
// - or all at it while a level waits for a sample not at it) is skipped whole; the rest is looked at sample by sample.
// The last level carries over for edges.
bool LogicCapture::findTrigger(const uint8_t *data, size_t length, uint64_t first_sample, uint64_t &at) {
  const uint8_t w = width_, k = trig_role_;
  const bool edge = trig_type_ == cap::kTriggerEdge;
  const uint8_t want = trig_value_ & 1;
  const uint64_t from = arm_from_;   // arm 1: once the pretrigger is in, counted from the arming (capture §3.3)
  auto hits = [&](uint8_t level, uint64_t sample) {
    bool hit = false;
    if (sample >= from) {
      if (edge) hit = have_level_ && level != last_level_ && (trig_value_ == 2 || (trig_value_ == 0) == (level == 1));
      else if (level != want) level_false_ = true;
      else hit = when_ || level_false_;
    }
    last_level_ = level;
    have_level_ = true;
    return hit;
  };
  if (w <= 8) {
    const uint8_t per = 8 / w;
    uint8_t mask = 0;
    for (uint8_t s = 0; s < per; ++s) mask |= static_cast<uint8_t>(1u << (s * w + k));
    for (size_t b = 0; b < length; ++b) {
      const uint64_t first = first_sample + static_cast<uint64_t>(b) * per;
      const uint8_t v = data[b] & mask;
      if (first >= from) {
        if (edge ? have_level_ && v == (last_level_ ? mask : 0)
                 : v == (want ? 0 : mask) ? (level_false_ = true)
                                          : (!when_ && !level_false_ && v == (want ? mask : 0)))
          continue;
      } else if (first + per <= from) {   // arming: only its last level
        last_level_ = (data[b] >> ((per - 1) * w + k)) & 1;
        have_level_ = true;
        continue;
      }
      for (uint8_t s = 0; s < per; ++s)
        if (hits((data[b] >> (s * w + k)) & 1, first + s)) {
          at = first + s;
          return true;
        }
    }
    return false;
  }
  for (size_t i = 0; i + 1 < length; i += 2) {   // 16-bit samples
    const uint16_t v = static_cast<uint16_t>(data[i] | data[i + 1] << 8);
    if (hits((v >> k) & 1, first_sample + i / 2)) { at = first_sample + i / 2; return true; }
  }
  return false;
}

// A one-shot with a trigger: look for it in each chunk; once found, the segment starts exactly pretrigger samples
// before it (trigger_index = pretrigger, capture §2 / §3.3) - or at the oldest the ring still has, the segment then
// that much shorter - and takes the chunks after it until full. A force, or a group's trigger when following, is the
// sample of that instant (force_sample_, ext_sample_), taken once the stream has come that far. What comes before the
// trigger's chunk is copied through the map of the recent chunks (spans_): the ring is not the stream byte for byte
// (Chunk::ring). Harvest task only.
void LogicCapture::harvestTriggered(const Chunk &chunk) {
  if (const uint32_t skip = missed(chunk)) {           // chunks lost before this one: not searched; still in the ring
    mapLost(captured_, skip, chunk.ring);
    captured_ += skip;
    have_level_ = false;
  }
  const uint64_t base = captured_;                     // the stream byte of chunk.data[0]
  const uint64_t end = base + chunk.length;
  captured_ = end;
  if (trig_phase_ == 2) return;
  addSpan(base, chunk.ring, static_cast<uint32_t>(chunk.length));
  if (!ringIntact(chunk.ring)) {                       // rewritten before it was looked at
    if (trig_phase_ == 1) trig_overrun_ = true;
    have_level_ = false;
    overruns_ = overruns_ + 1;
    return;
  }
  if (trig_phase_ == 0) {
    const uint64_t first = base * 8 / width_;
    uint64_t t = first;
    if (follow_ || force_) {   // the group's trigger or a force: at its sample, once the stream has come that far
      if (follow_ && !ext_ready_) return;
      __atomic_thread_fence(__ATOMIC_ACQUIRE);
      t = follow_ ? ext_sample_ : force_sample_;
      if (t >= end * 8 / width_) return;
      force_ = false;
    } else if (!findTrigger(chunk.data, chunk.length, first, t)) {
      return;
    }
    uint64_t s0 = t > pre() ? t - pre() : 0;   // fewer before it than the pretrigger: the segment is short (§3.3)
    const uint64_t oldest = oldestKept();      // not before the oldest byte the ring still holds in order
    if (s0 * width_ / 8 < oldest) s0 = (oldest * 8 + width_ - 1) / width_;
    seg_first_sample_ = s0;
    trigger_index_ = t >= s0 ? static_cast<uint32_t>(t - s0) : 0xFFFFFFFFu;   // before the ring's oldest: gone
    if (!follow_) eventFound(start_ns_ + nsOf(t));   // judging a group's condition: the followers take it now
    if (t < s0) trig_overrun_ = true;
    // the samples before the trigger, the trigger's and the pretrigger's complement after it (pre() < samples_)
    seg_samples_ = static_cast<uint32_t>((t >= s0 ? t - s0 : 0) + (samples_ - pre()));
    seg_shift_ = static_cast<uint8_t>(width_ < 8 ? s0 % (8 / width_) * width_ : 0);
    seg_raw_ = static_cast<uint32_t>((seg_shift_ + static_cast<uint64_t>(seg_samples_) * width_ + 7) / 8);
    seg_aligned_ = false;
    filled_ = 0;
    trig_phase_ = 1;
    const uint64_t from = s0 * width_ / 8;
    if (from < base) {   // the part before this chunk, through the map
      const uint64_t to = from + seg_raw_ < base ? from + seg_raw_ : base;
      if (!copyStream(from, to)) trig_overrun_ = true;
    }
  }
  // this chunk's part of the segment (chunks lost to the queue just before it through the map)
  uint64_t next = seg_first_sample_ * width_ / 8 + filled_;
  if (next < base && filled_ < seg_raw_ && !trig_overrun_) {
    const uint64_t to = next + (seg_raw_ - filled_) < base ? next + (seg_raw_ - filled_) : base;
    if (!copyStream(next, to)) trig_overrun_ = true;
    next = seg_first_sample_ * width_ / 8 + filled_;
  }
  if (next >= base && next < end && filled_ < seg_raw_) {
    size_t n = static_cast<size_t>(end - next);
    if (n > seg_raw_ - filled_) n = seg_raw_ - filled_;
    memcpy(buffer_ + filled_, chunk.data + (next - base), n);
    filled_ += static_cast<uint32_t>(n);
    if (!ringIntact(chunk.ring + static_cast<uint32_t>(next - base))) trig_overrun_ = true;   // rewritten while we copied
  }
  if (filled_ >= seg_raw_) {
    alignSegment(filled_);
    trig_phase_ = 2;
    done_ = true;
  }
}

// Chunks lost to the queue (`length` stream bytes from `stream`, the next chunk at ring position `next_ring`): their
// bytes lie between the last chunk's and the next one's in the ring, with the DMA's skip after an early close in
// between when one fell there - at a multiple of kSegmentBytes of the stream (eof_data_len). Mapped when that is
// where they can only be; otherwise unmapped (a segment that needs them has a hole).
void LogicCapture::mapLost(uint64_t stream, uint32_t length, uint32_t next_ring) {
  if (span_count_) {
    const Span &last = spans_[(span_count_ - 1) % kSpans];
    if (last.ring != kUnmapped && last.stream + last.length == stream) {
      const uint32_t at = last.ring + last.length;
      const uint32_t skipped = next_ring - at - length;   // what the DMA left out between them
      const uint64_t b = (stream / kSegmentBytes + 1) * kSegmentBytes;   // the first early close after `stream`
      if (skipped == 0) {
        addSpan(stream, at, length);
        return;
      }
      if (skipped != 0 && skipped < kDmaAhead && b <= stream + length && b + kSegmentBytes > stream + length) {
        const uint32_t before = static_cast<uint32_t>(b - stream);
        addSpan(stream, at, before);
        addSpan(b, next_ring - (length - before), length - before);
        return;
      }
    }
  }
  addSpan(stream, kUnmapped, length);
}

void LogicCapture::addSpan(uint64_t stream, uint32_t ring, uint32_t length) {
  if (!length) return;
  spans_[span_count_ % kSpans] = {stream, ring, length};
  ++span_count_;
}

// The oldest stream byte from which the recent chunks are all mapped and still in the ring, up to the newest's end.
uint64_t LogicCapture::oldestKept() const {
  if (!span_count_) return 0;
  const uint32_t n = span_count_ < kSpans ? span_count_ : static_cast<uint32_t>(kSpans);
  const Span &newest = spans_[(span_count_ - 1) % kSpans];
  uint64_t oldest = newest.stream + newest.length;
  for (uint32_t k = 0; k < n; ++k) {
    const Span &sp = spans_[(span_count_ - 1 - k) % kSpans];
    if (sp.ring == kUnmapped || sp.stream + sp.length != oldest) break;
    if (!ringIntact(sp.ring)) {   // its first bytes rewritten already: from the first one that is not
      const uint32_t gone = ring_end_ + kDmaAhead - rx_ring_bytes_ - sp.ring;
      if (gone < sp.length) oldest = sp.stream + gone;
      break;
    }
    oldest = sp.stream;
  }
  return oldest;
}

// Stream bytes [from, to) - in the mapped recent chunks - appended to the segment (buffer_ at filled_). false: some of
// them are not mapped, or the DMA came round over them before or while they were copied.
bool LogicCapture::copyStream(uint64_t from, uint64_t to) {
  const bool ok = copyStreamTo(buffer_ + filled_, from, to);
  filled_ += static_cast<uint32_t>(to - from);
  return ok;
}

// Stream bytes [from, to) - in the mapped recent chunks - to dst. false: some of them are not mapped, or the DMA came
// round over them before or while they were copied (dst then holds what was there).
bool LogicCapture::copyStreamTo(uint8_t *dst, uint64_t from, uint64_t to) {
  uint32_t copied = 0;
  const uint32_t n = span_count_ < kSpans ? span_count_ : static_cast<uint32_t>(kSpans);
  uint32_t k = 0;   // the span holding `from`, counted back from the newest
  while (k < n) {
    const Span &sp = spans_[(span_count_ - 1 - k) % kSpans];
    if (sp.stream <= from && from < sp.stream + sp.length) break;
    ++k;
  }
  if (k == n) return false;
  bool ok = true;
  uint32_t first_ring = 0;
  for (uint64_t g = from; g < to; ) {
    if (k == UINT32_MAX) return false;   // past the newest span
    const Span &sp = spans_[(span_count_ - 1 - k) % kSpans];
    if (sp.ring == kUnmapped || g < sp.stream || g >= sp.stream + sp.length) return false;
    const uint32_t q = sp.ring + static_cast<uint32_t>(g - sp.stream);
    if (g == from) first_ring = q;
    const size_t r = q & (rx_ring_bytes_ - 1);
    size_t m = rx_ring_bytes_ - r;
    if (m > sp.stream + sp.length - g) m = static_cast<size_t>(sp.stream + sp.length - g);
    if (m > to - g) m = static_cast<size_t>(to - g);
    memcpy(dst + copied, rx_ring_ + r, m);
    copied += static_cast<uint32_t>(m);
    g += m;
    if (g == sp.stream + sp.length) k = k ? k - 1 : UINT32_MAX;
  }
  if (!ringIntact(first_ring)) ok = false;   // the oldest copied byte last: the DMA only comes round from there
  return ok;
}

// The segment's first `raw` bytes copied from the ring moved down by seg_shift_ bits, so that its first sample is at bit
// 0 of byte 0 (once: the harvest's completion or a stop, whichever comes first).
void LogicCapture::alignSegment(uint32_t raw) {
  if (seg_aligned_) return;
  seg_aligned_ = true;
  if (!seg_shift_ || !raw) return;
  const uint8_t up = static_cast<uint8_t>(8 - seg_shift_);
  for (uint32_t i = 0; i + 1 < raw; ++i) buffer_[i] = static_cast<uint8_t>(buffer_[i] >> seg_shift_ | buffer_[i + 1] << up);
  buffer_[raw - 1] = static_cast<uint8_t>(buffer_[raw - 1] >> seg_shift_);
}

uint32_t LogicCapture::filledSamples() const {
  const uint64_t bits = static_cast<uint64_t>(filled_) * 8;
  const uint64_t n = bits > seg_shift_ ? (bits - seg_shift_) / width_ : 0;
  return static_cast<uint32_t>(n < seg_samples_ ? n : seg_samples_);
}

Result LogicCapture::startTriggered(uint8_t *out, size_t capacity) {
  if (capacity < 4) return failed();
  kept_samples_ = 0;   // the last generation's segment goes
  kept_short_ = false;
  trig_phase_ = 0;
  ext_ready_ = false;
  trig_overrun_ = false;
  have_level_ = level_false_ = false;
  arm_from_ = arm_ ? pre() : 0;
  lost_ = false;
  lost_pos_ = 0;
  error_ = cap::kErrorPeripheral;
  force_sample_ = 0;
  force_ = trig_type_ == cap::kTriggerImmediate && !follow_;   // a ring left by following: start at once
  fill_ = 0;
  filled_ = 0;
  seg_shift_ = 0;
  seg_samples_ = seg_raw_ = 0;
  seg_aligned_ = false;
  trigger_index_ = 0xFFFFFFFFu;
  // every track of a trigger source's group sends its own triggered too, at its own sample (§4.4); an immediate start
  // sends none
  reported_trigger_ = !follow_ && trig_type_ == cap::kTriggerImmediate;
  done_ = false;
  overruns_ = 0;
  queue_overflow_ = 0;
  produced_ = 0;
  ring_end_ = 0;
  span_count_ = 0;
  captured_ = 0;
  if (!reopenUnit()) { fail(cap::kErrorPeripheral); return failed(); }
  xQueueReset(queue_);
  harvesting_ = true;
  if (xTaskCreatePinnedToCore(harvestTask, "oep_harvest", 4096, this, 5, &task_, kHarvestCore) != pdPASS) {
    harvesting_ = false;
    return failed();
  }
  parlio_receive_config_t rc = {};
  rc.delimiter = delimiter_;
  rc.flags.partial_rx_en = true;
  if (parlio_rx_unit_receive(unit_, rx_ring_, rx_ring_bytes_, &rc) != ESP_OK) { stopRepeat(); fail(cap::kErrorPeripheral); return failed(); }
  start_ns_ = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
  if (parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, true) != ESP_OK) { stopRepeat(); fail(cap::kErrorPeripheral); return failed(); }
  state_ = kStateWaiting;
  generation_ = nextGeneration(generation_);   // 1 after 0xFFFFFFFF, never 0 (capture §3.2)
  putU32(out, generation_);
  return completed(4);
}

void LogicCapture::finishSegment(uint32_t bytes, uint8_t flags) {
  Info &info = infos_[completed_ % kInfos];
  info.samples = bytes * 8 / width_;
  info.flags = flags;
  if (rep_trig_) data_pos_ += bytes;   // a repeat with a trigger: the segments' bytes one after another
  __atomic_thread_fence(__ATOMIC_RELEASE);
  fill_ = 0;
  if (segment_count_) fill_slot_ = (fill_slot_ + 1) % segment_count_;
  completed_ = completed_ + 1;   // wraps after 0xFFFFFFFF (capture §2.2)
  __atomic_thread_fence(__ATOMIC_RELEASE);
  if (kept_infos_ < kInfos - 1) kept_infos_ = kept_infos_ + 1;   // after completed_: a reader takes this first (keptSerials)
}

// Harvest task: the data being gathered has a hole (capture §2.2). The segment being filled - in zero-copy streaming the
// stage not yet sent, and the byte held back for the next one - is dropped, never handed out; nothing more is taken.
// write_pos stays at its start. poll() (failLost) stops the track.
void LogicCapture::loseSegment() {
  if (direct_) {
    lost_pos_ = stage_cur_ >= 0 ? stage_pos_ : format_ == cap::kDataFormatLogicBlocks ? enc_pos_ : captured_;
    if (stage_cur_ >= 0) __atomic_fetch_or(&stage_free_, 1u << stage_cur_, __ATOMIC_ACQ_REL);
    stage_cur_ = -1;
    stage_fill_ = 0;
  } else {
    lost_pos_ = fill_ ? infos_[completed_ % kInfos].position : rep_trig_ ? static_cast<uint64_t>(data_pos_) : captured_;
    fill_ = 0;
  }
  __atomic_thread_fence(__ATOMIC_RELEASE);
  lost_ = true;
}

// Loop: a segment was lost (lost_), or a triggered one-shot's segment has a hole (trig_overrun_): the track stops in
// state 6 with stopped reason 3, error 2 (the queue or the ring overflowed; capture §2.2) - no segment handed out.
void LogicCapture::failLost() {
  if (triggered_) {
    harvesting_ = false;
    for (int i = 0; i < 100 && task_; ++i) delay(2);
    kept_samples_ = 0;   // the segment is not handed out; write_pos stays at its start (0)
    kept_short_ = false;
    lost_pos_ = 0;
  } else {
    stopRepeat();        // nothing more to finish: the segment being filled went (loseSegment)
  }
  if (unit_ && delimiter_) parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
  fail(cap::kErrorStorage);
}

// State 6 (capture §3.2): every entry - from state 1, 2, 3 or 5 - sends stopped reason 3 with the error status answers
// (1 the PARLIO or its DMA, 2 a segment lost to the queue or the ring).
void LogicCapture::fail(uint8_t error) {
  const bool entering = state_ != kStateError;
  state_ = kStateError;
  error_ = error;
  if (entering && subscribed_) {
    uint8_t stopped[6] = {cap::kStoppedReasonError, error};   // reason(u8) error(u8) generation(u32)
    putU32(stopped + 2, generation_);
    endpoint_.event(*this, kEventStopped, stopped, sizeof stopped);
  }
}

void LogicCapture::harvestTask(void *context) {
  auto *self = static_cast<LogicCapture *>(context);
  Chunk chunk;
  if (self->direct_) {
    bool &held = self->held_;   // members: a chunk waiting for a stage outlives a pass of the loop
    size_t &at = self->held_at_;
    Chunk &waiting = self->held_chunk_;
    bool drained = false;   // the last drain moved bytes
    while (self->harvesting_) {
      // the spill to drain: no waiting on the queue while it moves (a tick when no stage took any: not spinning)
      const bool spilled = self->spill_used_ || self->spill_drop_;
      const TickType_t wait = !spilled ? pdMS_TO_TICKS(2) : drained ? 0 : 1;
      if (!held && xQueueReceive(self->queue_, &waiting, wait) == pdTRUE) {
        held = true;
        at = 0;
      }
      drained = spilled && self->drainSpill();
      if (held) {
        // straight into the stages - after what the spill holds; none free: into the spill
        if (!self->spill_used_ && !self->spill_drop_) at = self->harvestDirect(waiting, at);
        if (at < waiting.length) at = self->spillIn(waiting, at);
        held = at < waiting.length;
        if (held) { vTaskDelay(1); continue; }   // the stages and the spill full: the ring holds the rest
      }
      if (self->stage_cur_ < 0 || self->stage_fill_ == 0) continue;
      // the subscriber's batching: at least min_bytes, or max_delay_ms after the stage's first byte (0, 0: when idle)
      uint16_t fn = 0, min_bytes = 0;
      uint32_t max_delay = 0;
      if (!self->endpoint_.directPush(*self, fn, min_bytes, max_delay)) continue;
      // 0, 0: out as soon as nothing waits - the harvest's queue and the transport both idle. With the transport busy
      // the stage takes more meanwhile (it would not go out sooner): a format 1 block a frame on quiet lines was 2000
      // transfers a second at 13.3 MHz, more than the host took back, and 35 % of the blocks found no stage (P4X)
      DirectTransport *t = self->endpoint_.direct();
      const bool idle = uxQueueMessagesWaiting(self->queue_) == 0 && (!t || t->queued() == 0);
      if ((min_bytes == 0 && max_delay == 0 && idle) || (min_bytes && self->stage_fill_ >= min_bytes) ||
          (max_delay && millis() - self->stage_since_ >= max_delay))
        self->sendStage();
    }
    // what arrived before the stop, as stages free up (kStopDrainMs at most: then the rest is dropped)
    for (const uint32_t t0 = millis(); held || uxQueueMessagesWaiting(self->queue_) || self->spill_used_;) {
      if (self->spill_used_ || self->spill_drop_) self->drainSpill();
      if (!held && !self->spill_used_ && !self->spill_drop_) {
        if (xQueueReceive(self->queue_, &waiting, 0) != pdTRUE) break;
        held = true;
        at = 0;
      }
      if (held) {
        if (!self->spill_used_ && !self->spill_drop_) at = self->harvestDirect(waiting, at);
        held = at < waiting.length;
      }
      if (!held && !self->spill_used_) continue;
      if (millis() - t0 >= kStopDrainMs) {   // the rest dropped (the spill's, the chunk's, the queue's)
        self->spill_drop_ += self->spill_used_;
        self->spill_used_ = 0;
        self->captured_ += self->spill_drop_;
        self->spill_drop_ = 0;
        if (held) self->dropDirect(waiting.length - at);
        held = false;
        while (xQueueReceive(self->queue_, &waiting, 0) == pdTRUE) self->dropDirect(waiting.length);
        break;
      }
      vTaskDelay(1);
    }
    self->task_ = nullptr;
    vTaskDelete(nullptr);
  }
  while (self->harvesting_) {
    if (xQueueReceive(self->queue_, &chunk, pdMS_TO_TICKS(20)) != pdTRUE) continue;
    if (self->triggered_) self->harvestTriggered(chunk);
    else self->harvest(chunk);
  }
  while (xQueueReceive(self->queue_, &chunk, 0) == pdTRUE) {   // what arrived before the stop
    if (self->triggered_) self->harvestTriggered(chunk);
    else self->harvest(chunk);
  }
  self->task_ = nullptr;
  vTaskDelete(nullptr);
}

bool LogicCapture::receiveDone(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context) {
  static_cast<LogicCapture *>(context)->done_ = true;   // ISR: flag only
  return false;
}

bool IRAM_ATTR LogicCapture::oneShotProgress(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *e, void *context) {
  static_cast<LogicCapture *>(context)->produced_ += e->recv_bytes;   // ISR: count only (the driver synced the node)
  return false;
}

size_t LogicCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  // role k = channel k (oep-if-capture: roles 0..), each on any of the probe's channels
  uint8_t roles[kMaxChannels];
  for (uint8_t k = 0; k < kMaxChannels; ++k) roles[k] = k;
  w.roleChannels(roles, kMaxChannels, table_.allowedMask());
  // no features: revision 1 defines no bit (query, force, subscribe and unsubscribe are in the ops tag, capture §3.6)
  // mode(u8) max_samples(u32) max_segments(u32), one per mode (capture §3.5)
  uint8_t mode[9] = {1};                           // one-shot (DMA: the probe keeps answering)
  // max samples at w = 1: with a trigger, at a rate the harvest keeps up with, the store's most (an immediate one
  // kSegmentBytes: configure / query say which a setting gets)
  const size_t one_shot = storeMax() < kTriggeredSegmentMax ? storeMax() : kTriggeredSegmentMax;
  putU32(mode + 1, static_cast<uint32_t>((one_shot > kSegmentBytes ? one_shot : kSegmentBytes) * 8));
  putU32(mode + 5, 1);                             // one segment
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  mode[0] = 2;                                     // repeat: the most the store can ever hold
  putU32(mode + 1, kSegmentMaxRepeat * 8);         // (a declaration, core §7.3: not the free space of the moment)
  putU32(mode + 5, storeMax() / kSegmentMin);
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  mode[0] = 3;                                     // streaming: segments of 64 KiB, pushed (needs a subscription)
  putU32(mode + 1, kSegmentBytes * 8);
  putU32(mode + 5, kInfos);
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  uint8_t range[9];
  putU32(range, kMinHz);
  putU32(range + 4, kSourceHz);
  range[8] = 1;                                    // any value in range (fractional divider)
  w.put(cap::kTlvDescribeRateRange, range, sizeof range);
  w.u8(cap::kTlvDescribeChannels, kMaxChannels);   // max(u8)
  // immediate, level, edge (one-shot and repeat), linked (one-shot); the most pretrigger the ring can give back (the
  // narrowest sample)
  uint8_t trig[9];                                 // types(u32) max_pretrigger(u32) opts(u8)
  putTriggerDecl(trig, (1u << cap::kTriggerImmediate) | (1u << cap::kTriggerLevel) | (1u << cap::kTriggerEdge) |
                           (1u << cap::kTriggerLinked),
                 kSegmentBytes * 8 - 1, kTriggerOpts);   // at w = 1 (below the segment); a wider sample's is less
  w.put(cap::kTlvDescribeTrigger, trig, sizeof trig);
  w.u16(cap::kTlvDescribeTriggerRecords, kTriggerRecords);   // rearm 1's records kept (§3.10)
  // beyond 0 (capture §3.5): 1 the logic blocks (streaming over the vendor port's stages: configure says), 4 the test
  // sequence
  const uint8_t formats[] = {cap::kDataFormatLogicBlocks, cap::kDataFormatTestCounter};
  w.put(cap::kTlvDescribeFormats, formats, sizeof formats);
  uint8_t carriage[9];   // kinds(u8: raw, events) min_d(u32) max_d(u32): events D 1 - L, powers of 2
  carriage[0] = static_cast<uint8_t>((1u << cap::kLogicCarriageRaw) | (1u << cap::kLogicCarriageEvents));
  putU32(carriage + 1, 1);
  putU32(carriage + 5, blocks::kMaxBlock);
  w.put(cap::kTlvDescribeCarriage, carriage, sizeof carriage);
  return w.ok() ? w.length() : 0;
}

uint8_t LogicCapture::planCheck(const RoleAssignment *roles, size_t count) {
  // capture §3.2: a changed plan while waiting, capturing or paused (2 / 3 / 5): cause 6
  plan_cause_ = reg::core::kUnavailableCauseWrongState;
  if (state_ == kStateWaiting || state_ == kStateCapturing || state_ == kStatePaused) return kRejectUnavailable;
  plan_cause_ = reg::core::kUnavailableCauseLimit;
  uint32_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role >= kMaxChannels) return kRejectUnsupported;   // a role this capture does not have (core §8)
    if (!table_.allowed(roles[i].channel)) return kRejectUnsupported;   // not in role_channels (core §8)
    if (seen & (1u << roles[i].role)) return kRejectUnavailable;
    seen |= 1u << roles[i].role;
  }
  if (count > kMaxChannels) return kRejectUnavailable;   // more channels than it has (a role twice)
  return (count == 0 || seen == (1u << count) - 1) ? 0 : kRejectUnavailable;   // roles 0..count-1, no holes
}

// Taking the plan changes no pin (a logic capture only listens, capture §1.2, core §8): an output idle or another fn's
// output on the channel keeps driving. The parallel IO unit only enables each data pin's input and routes it in
// (parlio_new_rx_unit when a capture is set up), so there is nothing to restore at release either.
bool LogicCapture::planApply(const RoleAssignment *roles, size_t count) {
  close();
  forget();
  takeRing();   // up front, while the internal heap still has the block; a configure that needs it and finds none refuses
  for (size_t i = 0; i < count; ++i) pins_[roles[i].role] = roles[i].channel;
  channels_ = static_cast<uint8_t>(count);
  state_ = kStateUnconfigured;
  return true;
}

void LogicCapture::planRelease() {
  close();
  forget();
  channels_ = 0;
  state_ = kStateUnconfigured;
}

// The plan released or replaced (capture §3.2): back to state 0, the configuration, the data and the segments gone - a
// read finds nothing, status and segments count none.
void LogicCapture::forget() {
  mode_ = cap::kModeOneShot;
  triggered_ = follow_ = linked_ = rep_trig_ = false;
  rearm_ = 0;
  trig_type_ = 0;
  pretrigger_ = 0;
  record_next_ = 0;
  samples_ = bytes_ = filled_ = kept_samples_ = 0;
  kept_short_ = false;
  trig_phase_ = 0;
  trig_overrun_ = false;
  // one assignment each: several of these are volatile (a chained assignment reads a volatile back, -Wvolatile)
  completed_ = 0;
  released_ = 0;
  fill_ = 0;
  queue_overflow_ = 0;
  overruns_ = 0;
  stage_drops_ = 0;
  produced_ = 0;
  ring_end_ = 0;
  captured_ = 0;
  dropped_ = 0;
  sent_seg_ = sent_off_ = reported_ = 0;
  gap_pending_ = false;
  paused_ = false;
}

// The most the segment store can ever take: the PSRAM but a reserve, else the internal store's cap (describe mode).
size_t LogicCapture::storeMax() const {
  const size_t psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
  if (psram > 2 * kPsramReserve) return psram - kPsramReserve;
  return kInternalStoreMax;
}

size_t LogicCapture::storeBudget(uint32_t &caps) const {
  // what it holds now is freed before the next store is taken: the store, a triggered one-shot's segment (its own
  // block, not the ring) - a query answers as the configure after it would
  const size_t held = (store_ ? store_bytes_ : 0) + (buffer_ && !buffer_in_ring_ ? bytes_ + 1 : 0);
  // a stream's spill too: given back with the stages before a store is taken (releaseStages)
  const size_t psram = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) + held + (spill_ ? kSpillBytes : 0);
  if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0 && psram > 2 * kPsramReserve) {
    caps = MALLOC_CAP_SPIRAM;
    return psram - kPsramReserve;
  }
  caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  // the stages kept from a stream are given back before a store is taken (releaseStages)
  size_t internal = heap_caps_get_largest_free_block(caps) + held + static_cast<size_t>(stage_count_) * (stage_data_ + kPushHead + kPushTail);
  internal = internal > kInternalReserve ? internal - kInternalReserve : 0;
  return internal < kInternalStoreMax ? internal : kInternalStoreMax;
}

bool LogicCapture::open(uint32_t rate_hz, uint8_t width, size_t bytes, uint32_t &num, uint32_t &den) {
  parlio_rx_unit_config_t c = {};
  c.trans_queue_depth = 1;
  c.max_recv_size = bytes;
  c.data_width = width;
  c.clk_src = PARLIO_CLK_SRC_PLL_F160M;
  c.exp_clk_freq_hz = rate_hz;
  c.clk_in_gpio_num = GPIO_NUM_NC;
  c.clk_out_gpio_num = GPIO_NUM_NC;
  c.valid_gpio_num = GPIO_NUM_NC;
  for (size_t i = 0; i < PARLIO_RX_UNIT_MAX_DATA_WIDTH; ++i)
    c.data_gpio_nums[i] = i < channels_ ? static_cast<gpio_num_t>(pins_[i]) : GPIO_NUM_NC;
  if (parlio_new_rx_unit(&c, &unit_) != ESP_OK) { unit_ = nullptr; return false; }
  // The rate actually set: 160 MHz / (int + numerator / denominator), read back from the divider.
  const uint32_t n = HP_SYS_CLKRST.peri_clk_ctrl117.reg_parlio_rx_clk_div_num + 1;
  const uint32_t fn = HP_SYS_CLKRST.peri_clk_ctrl118.reg_parlio_rx_clk_div_numerator;
  uint32_t fd = HP_SYS_CLKRST.peri_clk_ctrl118.reg_parlio_rx_clk_div_denominator;
  if (fd == 0) fd = 1;
  const uint64_t top = static_cast<uint64_t>(kSourceHz) * fd, bottom = static_cast<uint64_t>(n) * fd + fn;
  rateFraction(top, bottom, num, den);
  return true;
}

// The PARLIO RX unit for the configuration, its callbacks and soft delimiter, enabled: the segment's own receive
// (an immediate one-shot: done at `bytes`, the DMA nodes counted for a stop) or the DMA ring's (repeat, streaming and
// a one-shot through the ring: partial receives into the harvest's queue).
bool LogicCapture::openUnit(uint32_t rate_hz, uint8_t width, bool ring, uint32_t bytes, uint32_t &num, uint32_t &den) {
  if (!open(rate_hz, width, ring ? rx_ring_bytes_ : bytes, num, den)) return false;
  parlio_rx_event_callbacks_t cb = {};
  if (ring) {
    cb.on_partial_receive = partialReceive;
  } else {
    cb.on_receive_done = receiveDone;
    cb.on_partial_receive = oneShotProgress;   // how far it got, for a stop that cuts it short
  }
  parlio_rx_soft_delimiter_config_t d = {};
  d.sample_edge = PARLIO_SAMPLE_EDGE_POS;
  d.bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB;
  d.eof_data_len = ring ? kSegmentBytes : bytes;
  return parlio_rx_unit_register_event_callbacks(unit_, &cb, this) == ESP_OK &&
         parlio_new_rx_soft_delimiter(&d, &delimiter_) == ESP_OK && parlio_rx_unit_enable(unit_, true) == ESP_OK;
}

void LogicCapture::closeUnit() {
  if (unit_) {
    if (delimiter_) parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
    parlio_rx_unit_disable(unit_);
  }
  if (delimiter_) { parlio_del_rx_delimiter(delimiter_); delimiter_ = nullptr; }
  if (unit_) { parlio_del_rx_unit(unit_); unit_ = nullptr; }
}

// Every start mounts its receive on a unit made for it, as configure leaves one: a unit that had run kept what the last
// transaction left - after a stop of an immediate one-shot the next start's done came after only the rest of the
// window, with a third to a half of the segment never written (zeros where the line was high), and after a completed
// run each start began with 258 bytes (2064 samples at w = 1) of the previous capture (P4 bench, 0.0.28+1c940ca). Same
// rate, width, buffer and callbacks; the divider is the same, read back again.
bool LogicCapture::reopenUnit() {
  closeUnit();
  uint32_t num = 0, den = 1;
  const bool ring = triggered_ || mode_ != cap::kModeOneShot;
  if (!openUnit(rate_hz_, width_, ring, bytes_, num, den)) { closeUnit(); return false; }
  rate_num_ = num;
  rate_den_ = den;
  return true;
}

void LogicCapture::close() {
  stopRepeat();
  // the DMA ring stays allocated (see openStages): taking 128 KiB of internal RAM again and again fragmented it
  if (store_) { heap_caps_free(store_); store_ = nullptr; }
  store_bytes_ = 0;
  freeStages();
  direct_ = false;
  if (queue_) { vQueueDelete(queue_); queue_ = nullptr; }
  closeUnit();
  if (buffer_ && !buffer_in_ring_) heap_caps_free(buffer_);   // the ring stays (takeRing)
  buffer_ = nullptr;
  buffer_in_ring_ = false;
  done_ = false;
}

bool LogicCapture::takeStreamRing() {
  if (!stream_ring_ && streamRingAvailable())
    stream_ring_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, kStreamRingBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
  return stream_ring_ != nullptr;
}

void LogicCapture::useRing(bool stream) {
  rx_ring_ = stream && stream_ring_ ? stream_ring_ : ring_;
  rx_ring_bytes_ = stream && stream_ring_ ? kStreamRingBytes : kRingBytes;
}

bool LogicCapture::takeRing() {
  if (!ring_) ring_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, kRingBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  return ring_ != nullptr;
}

// A configure that could not get the memory it needs: refused unavailable cause 3 (core §4.3, the resources), not
// failed with an empty payload and state 6. What the earlier configuration held is gone (close): state 0.
Result LogicCapture::noStorage(uint8_t *out, size_t capacity) {
  close();
  forget();
  state_ = kStateUnconfigured;
  return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
}

Result LogicCapture::configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query) {
  // core §2.3: each TLV this configure implements is checked the same with or without bit 7 - a length other than its
  // definition or a value the definition excludes is malformed; a value the definition leaves unused or this probe
  // cannot honour refuses the whole configure unsupported, the tag as received. An unknown critical tag is unsupported,
  // an unknown non-critical one passed over. Everything is checked before anything changes (core §4.3): the form, then
  // the values, then the plan, the group and the state (unavailable).
  static const uint8_t kKnown[] = {kTagMode, kTagRate, kTagSamples, kTagSegments, kTagTrigger, kTagPretrigger,
                                   capture_tag::kTriggerOpts, cap::kTlvConfigureFormat, cap::kTlvConfigureCarriage,
                                   cap::kTlvConfigureBlockMax};
  Tail tail;
  Result unknown_critical;
  const Result parsed = tail.parse(p, n, kKnown, out, capacity, &unknown_critical);
  if (refused(parsed)) return parsed;
  struct Item { uint8_t tag; size_t size; const uint8_t *v; bool critical; };
  Item items[] = {{kTagMode, 1, nullptr, false},    {kTagRate, 4, nullptr, false},    {kTagSamples, 4, nullptr, false},
                  {kTagSegments, 4, nullptr, false}, {kTagTrigger, 6, nullptr, false}, {kTagPretrigger, 4, nullptr, false},
                  {capture_tag::kTriggerOpts, kTriggerOptsBytes, nullptr, false}, {cap::kTlvConfigureFormat, 1, nullptr, false},
                  {cap::kTlvConfigureBlockMax, 4, nullptr, false}};
  Item &mode_tlv = items[0], &rate_tlv = items[1], &samples_tlv = items[2], &segments_tlv = items[3],
       &trigger_tlv = items[4], &pretrigger_tlv = items[5], &opts_tlv = items[6], &format_tlv = items[7],
       &block_max_tlv = items[8];
  for (Item &it : items) {   // a length other than its definition: malformed (core §2.3)
    const Result r = tail.fixed(it.tag, it.size, it.v, out, capacity, &it.critical);
    if (refused(r)) return r;
  }
  // capture §3.3's contract: mode and rate required, samples in mode 1 / 2; none of them 0 (malformed)
  auto ask = [](const Item &it) { return CaptureAsk{it.v, it.critical}; };
  if (captureMalformed(ask(mode_tlv), ask(rate_tlv), ask(samples_tlv), ask(segments_tlv))) return rejected(kRejectMalformed);
  if (refused(unknown_critical)) return unknown_critical;
  const uint8_t mode = mode_tlv.v[0];
  if (mode != cap::kModeOneShot && mode != cap::kModeRepeat && mode != cap::kModeStreaming)
    return Tail::refuse(kTagMode, mode_tlv.critical, out, capacity);
  uint32_t rate = getU32(rate_tlv.v);   // outside rate_range (capture §3.3); the driver would silently run at 160 MHz
  if (rate < kMinHz || rate > kSourceHz) return Tail::refuse(kTagRate, rate_tlv.critical, out, capacity);
  // samples in streaming, segments outside repeat, a pretrigger without a trigger: unsupported whatever the value
  const Result misplaced = captureMisplaced(mode, ask(samples_tlv), ask(segments_tlv), ask(trigger_tlv), ask(pretrigger_tlv), out, capacity);
  if (refused(misplaced)) return misplaced;
  // inside the range, one-shot's ceiling for the channel count: above it, the nearest it allows (capture §3.3)
  if (mode == cap::kModeOneShot) {
    const uint32_t limit = channels_ <= kLimitLines8 ? kLimitHz8 : kLimitHz16;
    if (rate > limit) rate = limit;
  }
  uint32_t samples = samples_tlv.v ? getU32(samples_tlv.v) : 0, segments = segments_tlv.v ? getU32(segments_tlv.v) : 0;
  // type(u8) role(u8) value(u32): level and edge in a one-shot and a repeat (streaming starts at once: immediate only).
  // A role outside the plan is refused unavailable cause 6 below, after every unsupported value (core §4.3).
  uint8_t trig_type = 0, trig_role = 0;
  uint32_t trig_value = 0;
  uint32_t pretrigger = 0;
  if (const uint8_t *v = trigger_tlv.v) {
    const uint32_t value = getU32(v + 2);
    const bool level_edge = (v[0] == cap::kTriggerLevel && value <= 1) || (v[0] == cap::kTriggerEdge && value <= 2);
    const bool ok = v[0] == cap::kTriggerImmediate ||
                    (mode == cap::kModeOneShot && (level_edge || v[0] == cap::kTriggerLinked)) ||   // linked: §4.4
                    (mode == cap::kModeRepeat && level_edge);   // every segment's, rearm 1 (§2.1)
    if (!ok) return Tail::refuse(kTagTrigger, trigger_tlv.critical, out, capacity);
    trig_type = v[0];
    trig_role = v[1];
    trig_value = value;
  }
  TriggerOpts opts;   // arm 0 and when 1 as well as the defaults (kTriggerOpts)
  if (!triggerOpts(opts_tlv.v, kTriggerOpts, opts)) return Tail::refuse(capture_tag::kTriggerOpts, opts_tlv.critical, out, capacity);
  // the data format (capture §1): 0, 4 the test sequence, or 1 the logic blocks - format 1 streaming over the zero-copy
  // stages only (describe formats; the answer says what a setting takes); any other unsupported
  const uint8_t format = format_tlv.v ? format_tlv.v[0] : cap::kDataFormatPacked;
  const bool blocks_format = format == cap::kDataFormatLogicBlocks;
  // carriage(role u8, kind u8, d u32), repeated, format 1 only (§1.3, §3.3): its form first over every copy (malformed:
  // another length, d 0, raw with d other than 1, a role twice, carriage without format 1), then the values
  // (unsupported, the tag as received: a kind it does not have, an events D not a power of 2 or past a block)
  uint32_t events_roles = 0, unit = 8;
  uint32_t d_of[kMaxChannels] = {};
  {
    size_t at = 0, vlen = 0;
    uint8_t raw = 0;
    const uint8_t *v = nullptr;
    uint32_t seen = 0;
    while (tail.next(at, raw, v, vlen)) {
      if ((raw & ~kTagCritical) != cap::kTlvConfigureCarriage) continue;
      if (vlen != 6) return rejected(kRejectMalformed);
      const uint32_t d = getU32(v + 2);
      if (!blocks_format || d == 0 || (v[1] == cap::kLogicCarriageRaw && d != 1) || (v[0] < 32 && ((seen >> v[0]) & 1)))
        return rejected(kRejectMalformed);
      if (v[0] < 32) seen |= 1u << v[0];
    }
    at = 0;
    while (tail.next(at, raw, v, vlen)) {
      if ((raw & ~kTagCritical) != cap::kTlvConfigureCarriage) continue;
      const uint32_t d = getU32(v + 2);
      if ((v[1] != cap::kLogicCarriageRaw && v[1] != cap::kLogicCarriageEvents) || d > blocks::kMaxBlock || (d & (d - 1)))
        return unsupportedTag(out, capacity, raw);
      if (v[1] == cap::kLogicCarriageEvents && v[0] < kMaxChannels) {
        events_roles |= 1u << v[0];
        d_of[v[0]] = d;
        if (d > unit) unit = d;
      }
    }
  }
  if (format != cap::kDataFormatPacked && format != cap::kDataFormatTestCounter &&
      !(blocks_format && mode == cap::kModeStreaming && endpoint_.directCapable() && channels_ <= 8))   // w 8: the fast path
    return Tail::refuse(cap::kTlvConfigureFormat, format_tlv.critical, out, capacity);
  if (pretrigger_tlv.v) pretrigger = getU32(pretrigger_tlv.v);   // checked against samples below
  // format 1 takes its samples a byte each (w 8, up to 8 lines): the encoder's fast path works on bytes; so does a repeat
  // with a trigger, whose segments start at any sample (pretrigger before the trigger exactly, §3.3: a byte a sample
  // needs no shift)
  const bool rep_trig = mode == cap::kModeRepeat && trig_type != cap::kTriggerImmediate;
  const uint8_t width = (blocks_format || rep_trig) && widthFor(channels_) < 8 ? 8 : widthFor(channels_);
  // streaming's ceiling: the harvest copies the DMA ring into stages (or the FIFO) at about kStreamMaxBytes a second;
  // above it the ring is rewritten before it is copied (a hole, state 6). The nearest rate it keeps is used (capture
  // §3.3), as the one-shot's ceiling above (P4X, 2026-10-20: 40 MHz w 8 kept, formats 0 and 4, 40.15 MB/s on the wire
  // with no drop; the next divider, 53.3 MHz, not measured)
  if (mode == cap::kModeStreaming) {
    // the highest whole-divider rate at or below it: the nearest the divider picks never lands above
    const uint64_t ceiling = blocks_format ? kStreamBlocksMaxBytes : kStreamMaxBytes;   // format 1: its encoding's
    const uint64_t most = kSourceHz / ((kSourceHz * static_cast<uint64_t>(width) + ceiling * 8ull - 1) / (ceiling * 8ull));
    if (rate > most) rate = static_cast<uint32_t>(most);
    if (rate < kMinHz) return Tail::refuse(kTagRate, rate_tlv.critical, out, capacity);
  }
  uint32_t actual_segments = 1;
  uint32_t store_caps = 0;
  const size_t budget = storeBudget(store_caps);
  if (mode == 3) {   // streaming: the store in at most kInfos segments, of the probe's size
    uint32_t seg = static_cast<uint32_t>(budget / kInfos) / kSegmentMin * kSegmentMin;
    if (seg < 64 * 1024) seg = 64 * 1024;
    if (seg > kSegmentMaxRepeat) seg = kSegmentMaxRepeat;
    while (seg > kSegmentMin && budget / seg < 4) seg /= 2;   // a small store: at least 4 segments
    actual_segments = static_cast<uint32_t>(budget / seg);
    if (actual_segments > kInfos) actual_segments = kInfos;
    samples = seg * 8 / width;
  } else if (mode == 2) {   // repeat: segments of whole 4 KiB, as many as fit the PSRAM budget (or as asked)
    // in 64 bits: a samples above the limit is rounded down to it (capture §3.3), never wrapped (samples x width
    // overflowed u32 from 2^28 samples at w = 16: a segment of 0 bytes, and configure failed)
    const uint64_t asked = (static_cast<uint64_t>(samples) * width + 7) / 8;
    uint32_t seg = asked > kSegmentMaxRepeat ? kSegmentMaxRepeat : static_cast<uint32_t>(asked);
    seg = (seg + kSegmentMin - 1) / kSegmentMin * kSegmentMin;
    if (seg > kSegmentMaxRepeat) seg = kSegmentMaxRepeat;
    while (seg > kSegmentMin && budget / seg < 2) seg /= 2;
    uint32_t count = static_cast<uint32_t>(budget / seg);
    if (segments && segments < count) count = segments;
    if (count < 2) count = 2;
    samples = seg * 8 / width;
    actual_segments = count;
  }
  uint32_t bytes = 0;
  if (mode == 1) {
    // Immediate, the DMA writes the segment itself: one receive, kSegmentBytes at most. With a trigger the harvest
    // copies it out of the ring into the store: as long as the store's room, at a rate the harvest keeps up with (the
    // internal ring 20 MB/s, the PSRAM one a linked track takes 40 MB/s - P4X, 2026-10-20); faster, kSegmentBytes
    const bool linked_psram = trig_type == cap::kTriggerLinked && streamRingAvailable();
    const uint64_t speed = static_cast<uint64_t>(rate) * width / 8;
    size_t most = kSegmentBytes;
    if (trig_type != cap::kTriggerImmediate && speed <= (linked_psram ? kStreamMaxBytes : kTriggeredLongMaxBytes) &&
        budget > kSegmentBytes)
      most = budget < kTriggeredSegmentMax ? budget : kTriggeredSegmentMax;
    const uint64_t asked = (static_cast<uint64_t>(samples) * width + 7) / 8;
    uint64_t b = asked < most ? asked : most;
    b = (b + 127) & ~uint64_t{127};                                  // whole cache lines for the DMA
    if (b > most) b = most / 128 * 128;
    bytes = static_cast<uint32_t>(b);
    samples = static_cast<uint32_t>(static_cast<uint64_t>(bytes) * 8 / width);
  }
  // the pretrigger is what the ring can give back at this sample width (describe declares the most, at w = 1), less
  // than the segment's samples as rounded (capture §3.3); only with a trigger (above), so a one-shot
  // (linked: the PSRAM ring's half when there is one - a linked track takes it, judging or following)
  const size_t history = trig_type == cap::kTriggerLinked && streamRingAvailable() ? kStreamRingBytes / 2 : kPretriggerBytes;
  const uint32_t max_pretrigger = static_cast<uint32_t>(history * 8 / width);
  if (pretrigger && (pretrigger >= samples || pretrigger > max_pretrigger))
    return Tail::refuse(kTagPretrigger, pretrigger_tlv.critical, out, capacity);
  // no plan, or a trigger's role not in it (type 0's role is not looked at): unavailable cause 6 (capture §3.2, §3.3)
  const bool roled = trig_type != cap::kTriggerImmediate && trig_type != cap::kTriggerLinked;
  if (channels_ == 0 || (roled && trig_role >= channels_)) return wrongState(out, capacity);
  if (events_roles >> channels_) return wrongState(out, capacity);   // a carriage role not in the plan (§3.3)
  // format 1's block (§1.3): L a multiple of 8 and of every D, at most block_max and kMaxBlock, its worst within a stage
  blocks::Plan plan;
  if (blocks_format) {
    plan.w = width;
    for (uint8_t k = 0; k < channels_; ++k) {
      if ((events_roles >> k) & 1) {
        plan.ev_bit[plan.events] = k;
        plan.d[plan.events++] = d_of[k];
      } else {
        plan.raw_bit[plan.raw++] = k;
      }
    }
    size_t frame = max_read_ + 16 + 2;
    if (frame > kStageFrameMax) frame = kStageFrameMax;
    const size_t stage_data = frame / 512 * 512 - kPushHead - kPushTail;
    const uint32_t most = block_max_tlv.v ? getU32(block_max_tlv.v) : blocks::kMaxBlock;
    uint32_t L = (most < blocks::kMaxBlock ? most : blocks::kMaxBlock) / unit * unit;
    while (L && blocks::worstBytes(plan, L) > stage_data) L -= unit;
    if (!L) return Tail::refuse(block_max_tlv.v ? cap::kTlvConfigureBlockMax : cap::kTlvConfigureFormat,
                                block_max_tlv.v ? block_max_tlv.critical : format_tlv.critical, out, capacity);
    plan.block = L;
  }
  if (!query && bound()) return boundInGroup(*this, out, capacity);   // the group's now (cause 4)
  if ((state_ == kStateCapturing || state_ == kStatePaused || state_ == kStateWaiting) && !query) return wrongState(out, capacity);
  // Not even two segments of store, or no DMA ring for a mode that runs through it: refused unavailable cause 3
  // before anything is touched (core §4.3; it answered failed with an empty payload).
  // (streaming over the zero-copy path needs no store: its stages are the segments)
  if (mode == 3 && actual_segments < 2 && !endpoint_.directCapable())
    return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
  const bool triggered = mode == cap::kModeOneShot && trig_type != cap::kTriggerImmediate;
  if (!query && (triggered || mode != cap::kModeOneShot) && !takeRing())
    return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
  if (!query && rep_trig && opts.rearm && !records_) {   // rearm 1's records (§3.10): taken once, kept
    records_ = static_cast<TriggerRecord *>(heap_caps_malloc(sizeof(TriggerRecord) * kTriggerRecords, MALLOC_CAP_SPIRAM));
    if (!records_) records_ = static_cast<TriggerRecord *>(heap_caps_malloc(sizeof(TriggerRecord) * kTriggerRecords, MALLOC_CAP_8BIT));
    if (!records_) return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
  }

  uint32_t num = 0, den = 1;
  if (query) {
    // Answered without touching the one PARLIO RX unit (a configured capture and its data stay): the divider the
    // driver would pick, from the same HAL helper. configure reads the real divider back, so a mismatch shows.
    hal_utils_clk_info_t info = {};
    info.src_freq_hz = kSourceHz;
    info.exp_freq_hz = rate;
    info.max_integ = 256;
    info.min_integ = 1;
    info.max_fract = 256;
    hal_utils_clk_div_t div = {};
    if (!hal_utils_calc_clk_div_frac_accurate(&info, &div)) return failed();
    const uint32_t fd = div.denominator ? div.denominator : 1;
    const uint64_t top = static_cast<uint64_t>(kSourceHz) * fd, bottom = static_cast<uint64_t>(div.integer) * fd + div.numerator;
    rateFraction(top, bottom, num, den);
  } else if (mode == 2 || mode == 3) {
    close();
    direct_ = mode == 3 && endpoint_.directCapable();   // the subscriber must come in on it (startRepeat)
    uint32_t got_samples = 0, got_segments = 0;
    const Open opened = openRepeat(rate, width, samples, actual_segments, num, den, got_samples, got_segments);
    if (opened == Open::kNoMemory) return noStorage(out, capacity);
    if (opened != Open::kOk) {
      close();
      fail(cap::kErrorPeripheral);
      return failed();
    }
    samples = got_samples;
    actual_segments = got_segments;
  } else if (triggered) {   // the repeat's ring, searched; the segment copied out of it (harvestTriggered)
    close();
    const Open opened = openTriggered(rate, width, bytes, num, den);
    if (opened == Open::kNoMemory) return noStorage(out, capacity);
    if (opened != Open::kOk) {
      close();
      fail(cap::kErrorPeripheral);
      return failed();
    }
  } else {
    close();
    // The DMA writes the segment itself, into internal RAM: the ring (a segment is at most half of it; nothing else
    // uses the ring while an immediate one-shot is configured). It used to take a block of its own and, finding none,
    // freed the ring for it - and once the rest of the firmware had taken a piece of the freed 128 KiB, every later
    // triggered configure failed until a reboot (523264 samples on 1 line, 0.0.28). Without the ring, a block of its
    // own, PSRAM the last resort; none of them: refused, cause 3.
    if (takeRing()) {
      buffer_ = ring_;
      buffer_in_ring_ = true;
    } else {
      buffer_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
      if (!buffer_) buffer_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, bytes, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM));
    }
    if (!buffer_) return noStorage(out, capacity);
    if (!openUnit(rate, width, false, bytes, num, den)) {
      close();
      fail(cap::kErrorPeripheral);
      return failed();
    }
  }
  if (!query) {
    triggered_ = triggered;
    rep_trig_ = rep_trig;
    rearm_ = opts.rearm;
    linked_ = trig_type == cap::kTriggerLinked;
    trig_type_ = trig_type;
    trig_role_ = trig_role;
    trig_value_ = trig_value;
    pretrigger_ = pretrigger;
    arm_ = opts.arm;
    when_ = opts.when;
    format_ = format;
    if (blocks_format) {
      plan_ = plan;
      if (!block_buf_) block_buf_ = static_cast<uint8_t *>(heap_caps_malloc(blocks::kMaxBlock * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
      if (!block_buf_) return noStorage(out, capacity);   // a block's samples (w 16 at most)
    }
    rate_hz_ = rate;
    mode_ = mode;
    width_ = width;
    samples_ = samples;
    bytes_ = bytes;
    kept_samples_ = 0;   // the buffers are the new configuration's
    kept_short_ = false;
    rate_num_ = num;
    rate_den_ = den;
    state_ = kStateConfigured;
  }
  TlvWriter w(out, capacity);
  uint8_t r[8];
  putU32(r, num);
  putU32(r + 4, den);
  w.put(kTagActualRate, r, sizeof r);
  // format 1: w = C = R, the raw channels (§1.3); else every channel at its bit
  const uint8_t shown = blocks_format ? plan.raw : channels_;
  uint8_t layout[2 + kMaxChannels] = {blocks_format ? plan.raw : width, shown};
  for (uint8_t k = 0; k < shown; ++k) layout[2 + k] = k;
  w.put(kTagLayout, layout, 2 + shown);
  // every answer row of the mode (capture §3.3): actual_samples in modes 1 / 2, actual_segments in mode 2 only
  if (mode != cap::kModeStreaming) w.u32(kTagActualSamples, samples);
  if (mode == cap::kModeRepeat) w.u32(kTagActualSegments, actual_segments);
  if (format != cap::kDataFormatPacked) w.u8(cap::kTlvConfigureAnswerFormat, format);   // always when not 0 (§3.3)
  if (blocks_format) w.u32(cap::kTlvConfigureAnswerBlock, plan.block);                     // L (§1.3, §3.3)
  // streaming: what it holds unsent (capture §3.3 stream_buffer) - the zero-copy stages (those held, or what internal
  // RAM gives now), else the store
  if (mode == cap::kModeStreaming) w.u32(cap::kTlvConfigureAnswerStreamBuffer, streamBuffer(samples, width, actual_segments));
  // judging a capture-group's condition (one-shot, §4.4): the longest from its sample to the group knowing it (§3.3)
  if (mode == cap::kModeOneShot) w.u32(cap::kTlvConfigureAnswerJudgeLatencyNs, static_cast<uint32_t>(latencyNs(num, den, width)));
  return w.ok() ? completed(w.length()) : failed();
}

LogicCapture::Open LogicCapture::openRepeat(uint32_t rate_hz, uint8_t width, uint32_t samples, uint32_t segments, uint32_t &num,
                              uint32_t &den, uint32_t &actual_samples, uint32_t &actual_segments) {
  segment_bytes_ = samples * width / 8;
  segment_count_ = segments;
  // a new store: nothing captured, nothing left to push from the last run (its unsent bytes are gone with it)
  fill_ = 0;
  released_ = 0;
  completed_ = 0;
  sent_seg_ = 0;
  sent_off_ = 0;
  dropped_ = 0;
  captured_ = 0;
  if (!takeRing()) return Open::kNoMemory;
  // Without internal RAM for two stages (taken by the DMA ring, a trigger's segment, the rest of the firmware), the
  // stream goes the copied way instead: the segments in the store, pushed by the endpoint like any notification.
  // It failed configure (0.0.28 on the bench: a streaming configure after other captures answered failed).
  if (direct_ && !openStages()) direct_ = false;
  if (direct_) stage_data_ -= stage_data_ % (width > 8 ? width / 8 : 1);   // a full stage ends at a sample (w 16)
  if (!direct_) {
    releaseStages();
    uint32_t caps = 0;
    storeBudget(caps);
    store_bytes_ = static_cast<size_t>(segment_bytes_) * segment_count_;
    store_ = static_cast<uint8_t *>(heap_caps_malloc(store_bytes_, caps));
    if (!store_) return Open::kNoMemory;
  }
  useRing(direct_);
  // a chunk is a descriptor (3968 - 4032 bytes) or the rest of one an early close cut: the ring's worth queued, so a
  // harvest waiting for a stage loses none
  queue_ = xQueueCreate(rx_ring_bytes_ == kRingBytes ? 128 : rx_ring_bytes_ / 2048, sizeof(Chunk));
  if (!queue_) return Open::kNoMemory;
  if (!openUnit(rate_hz, width, true, 0, num, den)) return Open::kFailed;
  actual_samples = samples;
  actual_segments = segment_count_;
  return Open::kOk;
}

LogicCapture::Open LogicCapture::openTriggered(uint32_t rate_hz, uint8_t width, uint32_t bytes, uint32_t &num, uint32_t &den) {
  queue_ = xQueueCreate(128, sizeof(Chunk));
  uint32_t caps = 0;
  storeBudget(caps);                         // PSRAM when there is some: the segment is filled by the CPU, not the DMA
  buffer_ = static_cast<uint8_t *>(heap_caps_malloc(bytes + 1, caps));   // + the byte a mid-byte start reaches into
  if (!takeRing() || !queue_ || !buffer_) return Open::kNoMemory;
  useRing(false);
  return openUnit(rate_hz, width, true, 0, num, den) ? Open::kOk : Open::kFailed;
}

Result LogicCapture::startRepeat(uint8_t *out, size_t capacity) {
  if (capacity < 4) return failed();
  // configured for the zero-copy path but the subscriber came in on another way (CDC, TCP): the copied way, its store
  // taken now - none to be had, refused cause 3 before anything changes (core §4.3)
  if (direct_ && !endpoint_.direct() && format_ == cap::kDataFormatLogicBlocks)
    return wrongState(out, capacity);   // format 1 goes over the stages only: the subscriber must be on the vendor port
  if (direct_ && !endpoint_.direct()) {
    releaseStages();
    uint32_t caps = 0;
    const size_t budget = storeBudget(caps);
    if (segment_count_ < 2 || static_cast<size_t>(segment_bytes_) * segment_count_ > budget)
      return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
    store_bytes_ = static_cast<size_t>(segment_bytes_) * segment_count_;
    store_ = static_cast<uint8_t *>(heap_caps_malloc(store_bytes_, caps));
    if (!store_) return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
    direct_ = false;
  }
  stage_drops_ = 0;
  stage_unsent_ = 0;
  overruns_ = 0;
  queue_overflow_ = 0;
  enc_pos_ = sample_pos_ = stage_first_sample_ = 0;
  block_fill_ = stage_samples_ = 0;
  enc_max_cycles_ = enc_blocks_ = enc_fallbacks_ = 0;
  ring_peak_ = 0;
  spill_peak_ = 0;
  spill_head_ = spill_used_ = 0;
  spill_drop_ = 0;
  held_ = false;
  held_at_ = 0;
  enc_cycles_ = 0;
  encoder_.reset();
  fill_ = 0;
  released_ = 0;
  completed_ = 0;
  fill_slot_ = 0;
  kept_infos_ = 0;
  lost_ = false;
  lost_pos_ = 0;
  error_ = cap::kErrorPeripheral;
  open_transfer_ = false;
  produced_ = 0;
  ring_end_ = 0;
  sent_seg_ = 0;
  sent_off_ = 0;
  dropped_ = 0;
  captured_ = 0;
  paused_ = false;
  gap_pending_ = false;
  reported_ = 0;
  // a repeat with a trigger: waiting for the first segment's (state 2), arming from the start
  rep_wait_ = rep_trig_;
  rep_started_ = !rep_trig_;
  floor_sample_ = 0;
  arm_from_ = arm_ ? pre() : 0;
  data_pos_ = 0;
  record_next_ = 0;
  found_head_ = found_tail_ = 0;
  span_count_ = 0;
  have_level_ = level_false_ = false;
  force_ = false;
  force_sample_ = 0;
  if (!reopenUnit()) { fail(cap::kErrorPeripheral); return failed(); }
  xQueueReset(queue_);
  harvesting_ = true;
  if (xTaskCreatePinnedToCore(harvestTask, "oep_harvest", 4096, this, 5, &task_, kHarvestCore) != pdPASS) {
    harvesting_ = false;
    return failed();
  }
  parlio_receive_config_t rc = {};
  rc.delimiter = delimiter_;
  rc.flags.partial_rx_en = true;
  if (parlio_rx_unit_receive(unit_, rx_ring_, rx_ring_bytes_, &rc) != ESP_OK) { stopRepeat(); fail(cap::kErrorPeripheral); return failed(); }
  start_ns_ = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
  if (parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, true) != ESP_OK) { stopRepeat(); fail(cap::kErrorPeripheral); return failed(); }
  state_ = rep_trig_ ? kStateWaiting : kStateCapturing;
  generation_ = nextGeneration(generation_);   // 1 after 0xFFFFFFFF, never 0 (capture §3.2)
  putU32(out, generation_);
  return completed(4);
}

void LogicCapture::stopRepeat() {
  if (!harvesting_ && !task_) return;
  if (unit_ && delimiter_) parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
  harvesting_ = false;
  for (uint32_t t0 = millis(); task_ && millis() - t0 < kStopDrainMs + 200;) delay(2);   // the task sends what arrived, then ends
  if (direct_) {   // the task is gone: send what is left, the held byte too
    if (format_ == cap::kDataFormatLogicBlocks && block_fill_ && !lost_)   // the block begun: short (§1.3)
      emitBlock(block_fill_ * 8 / width_);
    sendStage();
    closeTransfer();   // the stream's last transfer ends with a short packet
    return;
  }
  if (fill_ && completed_ - released_ < segment_count_) finishSegment(fill_, infos_[completed_ % kInfos].flags | 2);
}

// A group's follower: the one-shot through the ring (opened now if configure did not), cut around the group's trigger.
bool LogicCapture::trackStartFollowing() {
  if (!trackCanFollow()) return false;
  if (!triggered_) {
    uint32_t num = 0, den = 1;
    close();
    if (openTriggered(rate_hz_, width_, bytes_, num, den) != Open::kOk) { close(); fail(cap::kErrorPeripheral); return false; }
    triggered_ = true;
  }
  // the stream's ring in PSRAM when there is one: a follower holds its pretrigger and the judging track's latency
  // (an analog's: 100 ms of loop()) - 6.5 ms at 10 MHz x 8 lines in the internal ring, 100 ms in 1 MiB
  useRing(takeStreamRing());
  follow_ = true;
  uint8_t out[4];   // generation
  const Result r = startTriggered(out, sizeof out);
  return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess;
}

uint8_t LogicCapture::trackCanJudge(uint8_t type, uint8_t role, const uint8_t *args) const {
  namespace grp = reg::fixture_capture_group;
  if (type == grp::kTriggerCondLevel) {   // level(u8) when(u8)
    if (args[0] > 1 || args[1] > 1) return kJudgeUnsupported;
  } else if (type == grp::kTriggerCondEdge) {   // edge(u8: 0 rising, 1 falling, 2 both)
    if (args[0] > 2) return kJudgeUnsupported;
  } else {
    return kJudgeUnsupported;
  }
  return role < channels_ ? kJudgeOk : kJudgeNoRole;
}

// The group's condition as this track's own trigger (findTrigger): a level or an edge on the role, the source's arm.
void LogicCapture::trackJudge(uint8_t type, uint8_t role, const uint8_t *args, uint8_t arm) {
  namespace grp = reg::fixture_capture_group;
  judged_arm_ = arm_;
  judged_when_ = when_;
  trig_type_ = type == grp::kTriggerCondLevel ? cap::kTriggerLevel : cap::kTriggerEdge;
  trig_role_ = role;
  trig_value_ = args[0];
  when_ = type == grp::kTriggerCondLevel ? args[1] : 0;
  arm_ = arm;
}

void LogicCapture::trackUnjudge() {
  if (!linked_ || trig_type_ == cap::kTriggerLinked) return;
  trig_type_ = cap::kTriggerLinked;
  arm_ = judged_arm_;
  when_ = judged_when_;
}

void LogicCapture::trackTriggerAt(uint64_t ns) {
  ext_sample_ = samplesIn(ns > start_ns_ ? ns - start_ns_ : 0, rate_num_, rate_den_);
  __atomic_thread_fence(__ATOMIC_RELEASE);   // the harvest task (the other core) reads it once ext_ready_ is set
  ext_ready_ = true;
}

// The trigger's time once found - a force's at once (its sample is the instant's, force_sample_): the followers learn it
// without waiting a DMA chunk for the harvest to take it.
bool LogicCapture::trackTriggerNs(uint64_t &ns) const {
  if (!triggered_ || follow_) return false;
  if (trig_phase_ < 1 && force_) {
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    ns = start_ns_ + nsOf(force_sample_);
    return true;
  }
  if (trig_phase_ < 1) return false;
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  ns = start_ns_ + nsOf(seg_first_sample_ + trigger_index_);
  return true;
}

uint64_t LogicCapture::nsOf(uint64_t samples) const {
  // samples x den / num seconds: split so that no product leaves u64 (num <= 160e6, den <= 255)
  const uint64_t whole = samples / rate_num_, part = samples % rate_num_ * rate_den_;
  return whole * rate_den_ * 1000000000ull + part / rate_num_ * 1000000000ull + part % rate_num_ * 1000000000ull / rate_num_;
}

size_t LogicCapture::infoBytes(const Info &info, uint8_t *out) const {
  putU32(out, info.serial);
  putU64(out + 4, info.position);
  putU32(out + 12, info.samples);
  putU64(out + 16, info.start_ns);
  putU32(out + 24, kStartUncertaintyNs);
  putU32(out + 28, info.trigger_index);
  out[32] = info.flags;
  putU32(out + 33, generation_);
  return kInfoBytes;
}

size_t LogicCapture::segmentInfo(uint8_t *out) const {
  putU32(out, 0);                  // serial
  putU64(out + 4, 0);              // position
  putU32(out + 12, kept_samples_);
  putU64(out + 16, triggered_ ? start_ns_ + nsOf(seg_first_sample_) : start_ns_);
  putU32(out + 24, kStartUncertaintyNs);
  // immediate (also through the ring a group's following left): no trigger inside
  putU32(out + 28, triggered_ && (trig_type_ != cap::kTriggerImmediate || follow_) ? trigger_index_ : 0xFFFFFFFFu);
  out[32] = static_cast<uint8_t>((triggered_ && trig_overrun_ ? cap::kSegmentFlagGap : 0) |   // bit0: part of it was lost
                                 (kept_short_ ? cap::kSegmentFlagShort : 0));                // bit1: cut short by stop
  putU32(out + 33, generation_);
  return kInfoBytes;
}

void LogicCapture::pollTriggered() {
  if (trig_phase_ >= 1 && state_ == kStateWaiting) state_ = kStateCapturing;
  if (trig_phase_ >= 1 && !reported_trigger_) {
    reported_trigger_ = true;
    if (subscribed_) {   // capture §3.4: no record (rearm 0); the sample from the generation's start; +- a sample
      uint8_t e[kTriggeredBytes];
      putTriggered(e, 0, trigger_index_, seg_first_sample_ + trigger_index_,
                   start_ns_ + nsOf(seg_first_sample_ + trigger_index_), periodNs(rate_num_, rate_den_), generation_);
      endpoint_.event(*this, cap::kEventTriggered, e, sizeof e);
    }
  }
  if (trig_overrun_ && (state_ == kStateCapturing || state_ == kStateWaiting)) { failLost(); return; }   // a hole (§2.2)
  if (trig_phase_ != 2 || state_ != kStateCapturing) return;
  stopRepeat();
  if (trig_overrun_) { failLost(); return; }
  state_ = kStateDone;
  kept_samples_ = seg_samples_;
  if (subscribed_) {
    uint8_t seg[kInfoBytes];
    endpoint_.event(*this, kEventSegment, seg, segmentInfo(seg));
    uint8_t stopped[6] = {kStoppedComplete, 0};   // reason(u8) error(u8) generation(u32) (capture §3.4)
    putU32(stopped + 2, generation_);
    endpoint_.event(*this, kEventStopped, stopped, sizeof stopped);
  }
}

void LogicCapture::poll() {
  if ((mode_ == 2 || mode_ == 3) && lost_ && (state_ == kStateCapturing || state_ == kStatePaused)) {
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    while (reported_ != completed_) {   // the segments finished before the hole are handed out as usual
      if (subscribed_ && mode_ == 2) {
        uint8_t seg[kInfoBytes];
        endpoint_.event(*this, kEventSegment, seg, infoBytes(infos_[reported_ % kInfos], seg));
      }
      ++reported_;
    }
    failLost();
    return;
  }
  if (mode_ == 2 || mode_ == 3) {
    if (rep_trig_) {   // a repeat with a trigger: the first one found (state 2 -> 3); the triggered events
      if (state_ == kStateWaiting && rep_started_) state_ = kStateCapturing;
      reportFound();
    }
    if (mode_ == 2 && (state_ == kStateCapturing || state_ == kStatePaused)) state_ = paused_ ? kStatePaused : kStateCapturing;
    while (reported_ != completed_) {
      if (subscribed_ && mode_ == 2) {   // streaming sends no segment events (its data frames carry the positions)
        uint8_t seg[kInfoBytes];
        endpoint_.event(*this, kEventSegment, seg, infoBytes(infos_[reported_ % kInfos], seg));
      }
      ++reported_;
    }
    // streaming keeps capturing (dropped bytes show as a position jump); a repeat with no free segment pauses (state 5)
    // and goes on by itself after a release - no stopped event for that (oep-if-capture §3.2)
    return;
  }
  if (triggered_) { pollTriggered(); return; }
  if (state_ != kStateCapturing || !done_) return;
  esp_cache_msync(buffer_, bytes_, ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
  state_ = kStateDone;
  kept_samples_ = samples_;
  if (subscribed_) {
    uint8_t seg[kInfoBytes];
    endpoint_.event(*this, kEventSegment, seg, segmentInfo(seg));
    uint8_t stopped[6] = {kStoppedComplete, 0};   // reason(u8) error(u8) generation(u32) (capture §3.4)
    putU32(stopped + 2, generation_);
    endpoint_.event(*this, kEventStopped, stopped, sizeof stopped);
  }
}

// Streaming: bytes of segment `serial` that may be sent (a finished segment's length; the one being filled so far).
uint32_t LogicCapture::segmentLength(uint32_t serial) const {
  // the one being filled too (capture §2.2 lets its data out before it ends: a later loss in it stops the track with
  // write_pos at its start, and the host drops what it got from there)
  const uint32_t done = completed_;
  if (serialBefore(serial, done)) return infos_[serial % kInfos].samples * width_ / 8;
  if (serial == done && !lost_) return fill_;
  return 0;
}

// Streaming: the stored segment holding stream `position` (not yet reused), and the offset in it.
bool LogicCapture::findSegment(uint64_t position, uint32_t &serial, uint32_t &offset) const {
  uint32_t back = kept_infos_;   // finished segments still in the store: segment_count_ - 1 at most (one is filling)
  const uint32_t done = completed_;
  if (back > segment_count_ - 1) back = segment_count_ - 1;
  for (uint32_t i = 0; i <= back; ++i) {   // newest first, from the one being filled
    const uint32_t k = done - i;
    const uint32_t length = segmentLength(k);
    const uint64_t begin = infos_[k % kInfos].position;
    if (position >= begin && position - begin < length) { serial = k; offset = static_cast<uint32_t>(position - begin); return true; }
  }
  return false;
}

size_t LogicCapture::pending() {
  // the segments finished before a hole still go out after the track stopped on it (state 6)
  if (mode_ != 3 || direct_ || !(state_ == kStateCapturing || state_ == kStateConfigured || state_ == kStateError)) return 0;
  const uint32_t done = completed_;
  uint64_t n = 0;
  for (uint32_t k = sent_seg_; k != done; ++k) n += infos_[k % kInfos].samples * width_ / 8;
  if (!lost_) n += fill_;
  return n > sent_off_ ? static_cast<size_t>(n - sent_off_) : 0;
}

// Streaming push: the next bytes in stream order, from the segment being sent; a segment fully sent is released.
// payload: position(u64) len(u16) data, then TLV 0x01 generation (core §11.2, oep-if-capture §3.4)
size_t LogicCapture::pull(uint8_t *out, size_t capacity) {
  constexpr size_t kHead = 10, kTail = kTlvHeader + 4 + kTlvHeader + kSegmentInfoBytes;   // TLVs generation, segment_info
  if (mode_ != 3 || direct_ || !store_ || capacity <= kHead + kTail) return 0;
  const uint32_t serial = sent_seg_;
  const bool finished = serialBefore(serial, completed_);
  const uint32_t length = segmentLength(serial);
  __atomic_thread_fence(__ATOMIC_ACQUIRE);   // the bytes below length were written before it was published
  if (length <= sent_off_) {
    if (finished) { sent_seg_ = serial + 1; released_ = serial + 1; sent_off_ = 0; }   // an empty short segment
    return 0;
  }
  size_t n = length - sent_off_;
  if (n > capacity - kHead - kTail) n = capacity - kHead - kTail;
  if (n > 0xFFFF) n = 0xFFFF;
  putU64(out, infos_[serial % kInfos].position + sent_off_);
  putU16(out + 8, static_cast<uint16_t>(n));
  memcpy(out + kHead, store_ + static_cast<size_t>(infos_[serial % kInfos].slot) * segment_bytes_ + sent_off_, n);
  if (format_ == cap::kDataFormatTestCounter) putTestCounter(out + kHead, infos_[serial % kInfos].position + sent_off_, n);
  putTlvHeader(out + kHead + n, cap::kTlvDataGeneration, 4);
  putU32(out + kHead + n + kTlvHeader, generation_);
  // the segment (capture §3.4): its position, the samples it has so far, its start's time and uncertainty
  uint8_t *info = out + kHead + n + kTlvHeader + 4;
  const Info &si = infos_[serial % kInfos];
  putTlvHeader(info, cap::kTlvDataSegmentInfo, kSegmentInfoBytes);
  putU64(info + kTlvHeader, si.position);
  putU32(info + kTlvHeader + 8, static_cast<uint32_t>(static_cast<uint64_t>(length) * 8 / width_));
  putU64(info + kTlvHeader + 12, si.start_ns);
  putU32(info + kTlvHeader + 20, kStartUncertaintyNs);
  sent_off_ += n;
  if (finished && sent_off_ >= length) { sent_off_ = 0; sent_seg_ = serial + 1; released_ = serial + 1; }
  return kHead + n + kTail;
}

Result LogicCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  if (op == kOpConfigure || op == kOpQuery) return configure(p, n, out, capacity, op == kOpQuery);
  // Every other request: a fixed part (start / stop / force / status: none; read: generation position max = 14;
  // segments: 4; release: generation serial = 8), then TLVs, none of which these ops read.
  const size_t fixed = op == kOpRead ? 14 : op == kOpSegments ? 4 : op == kOpRelease ? 8 : 0;
  Tail tail;
  if (op >= kOpStart && op <= kOpRelease) {
    const Result parsed = plainTail(tail, p, n, fixed, out, capacity);
    if (refused(parsed)) return parsed;
  }
  // bound in a capture-group: the group starts and stops it (unavailable cause 4, after the request's form, core §4.3)
  if (bound() && !group_op_ && (op == kOpStart || op == kOpStop || op == kOpForce)) return boundInGroup(*this, out, capacity);
  switch (op) {
    case kOpStart: {   // -> generation(u32) [TLV]
      if (state_ != kStateConfigured && state_ != kStateDone && state_ != kStateError) return wrongState(out, capacity);
      if (mode_ == 3 && !subscribed_) return wrongState(out, capacity);   // streaming pushes: subscribe first
      if (mode_ == 2 || mode_ == 3) return startRepeat(out, capacity);
      if (linked_ && !bound()) return wrongState(out, capacity);   // linked: the group's event cuts it (§3.3, cause 6)
      follow_ = false;
      // its own trigger: the internal ring; linked (judging a group's condition), the PSRAM one when there is one -
      // its pretrigger may be that ring's half
      useRing(linked_ && takeStreamRing());
      if (triggered_) return startTriggered(out, capacity);
      if (capacity < 4) return failed();
      if (!reopenUnit()) { fail(cap::kErrorPeripheral); return failed(); }   // a unit made for this start
      done_ = false;
      produced_ = 0;
      kept_samples_ = 0;   // the last generation's segment goes
      kept_short_ = false;
      memset(buffer_, 0, bytes_);
      esp_cache_msync(buffer_, bytes_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
      parlio_receive_config_t rc = {};
      rc.delimiter = delimiter_;
      if (parlio_rx_unit_receive(unit_, buffer_, bytes_, &rc) != ESP_OK) { fail(cap::kErrorPeripheral); return failed(); }
      start_ns_ = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
      if (parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, true) != ESP_OK) { fail(cap::kErrorPeripheral); return failed(); }
      state_ = kStateCapturing;
      generation_ = nextGeneration(generation_);   // 1 after 0xFFFFFFFF, never 0 (capture §3.2)
      putU32(out, generation_);
      return completed(4);
    }
    case kOpStop: {
      uint8_t stopped[6] = {kStoppedHost, 0};   // reason(u8) error(u8) generation(u32) (capture §3.4)
      putU32(stopped + 2, generation_);
      // stop (capture §3.2): waiting (state 2) -> 1 with nothing; capturing (state 3) -> 1 with the segment cut short
      // (flags bit1) holding what the harvest copied; one that filled meanwhile is complete (state 4)
      if (triggered_ && (state_ == kStateWaiting || state_ == kStateCapturing)) {
        stopRepeat();      // the harvest takes what came before the stop
        pollTriggered();   // a trigger found meanwhile: its event; the segment full: complete; a hole: state 6
        if (state_ == kStateDone || state_ == kStateError) return completed();
        uint32_t got = 0;
        if (trig_phase_ >= 1) {   // the harvest has stopped: what it copied, its first sample moved to bit 0
          got = filledSamples();
          alignSegment(filled_);
        }
        state_ = kStateConfigured;
        kept_samples_ = got;
        kept_short_ = got != 0;
        if (subscribed_) {
          if (got) {
            uint8_t seg[kInfoBytes];
            endpoint_.event(*this, kEventSegment, seg, segmentInfo(seg));
          }
          endpoint_.event(*this, kEventStopped, stopped, sizeof stopped);
        }
        return completed();
      }
      if ((mode_ == 2 || mode_ == 3) && (state_ == kStateCapturing || state_ == kStatePaused || (rep_trig_ && state_ == kStateWaiting))) {
        stopRepeat();
        poll();
        if (state_ == kStateError) return completed();   // a hole before the stop: stopped on it (§2.2)
        state_ = kStateConfigured;
        if (subscribed_) endpoint_.event(*this, kEventStopped, stopped, sizeof stopped);
        return completed();
      }
      if (state_ == kStateCapturing) {   // an immediate one-shot
        // capturing (state 3) -> 1 with the segment cut short (flags bit1) holding the DMA nodes finished before the
        // stop (capture §3.2; it went to state 1 with nothing: done 0, write_pos 0); one that filled meanwhile is
        // complete (state 4). The unfinished transaction is erased (disable, enable with the queue reset), so the
        // next start mounts its own (it stayed with the driver, which went on filling the old one).
        poll();
        if (state_ == kStateDone) return completed();
        parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
        parlio_rx_unit_disable(unit_);
        if (done_) {   // it completed while it was being stopped
          parlio_rx_unit_enable(unit_, true);
          poll();
          return completed();
        }
        const uint32_t got_bytes = produced_ < bytes_ ? produced_ : bytes_;
        // the unit enabled again for its next use; a failure here loses nothing (the next start makes a unit anew)
        parlio_rx_unit_enable(unit_, true);
        esp_cache_msync(buffer_, bytes_, ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
        uint32_t got = static_cast<uint32_t>(static_cast<uint64_t>(got_bytes) * 8 / width_);
        if (got > samples_) got = samples_;
        state_ = kStateConfigured;
        kept_samples_ = got;
        kept_short_ = got != 0;
        if (subscribed_) {
          if (got) {
            uint8_t seg[kInfoBytes];
            endpoint_.event(*this, kEventSegment, seg, segmentInfo(seg));
          }
          endpoint_.event(*this, kEventStopped, stopped, sizeof stopped);
        }
      }
      return completed();
    }
    case kOpStatus: {   // -> state(u8) serial_done(u32) write_pos(u64) flags(u8) generation(u32) [TLV error]
      if (capacity < 21) return failed();
      poll();
      out[0] = state_;
      if (mode_ == 2 || mode_ == 3) {
        putU32(out + 1, completed_);
        // the next byte to write, the dropped ones counted (oep-if-capture §3.2); stopped on a hole, the start of the
        // segment it was in (§2.2)
        putU64(out + 5, lost_ ? lost_pos_ : format_ == cap::kDataFormatLogicBlocks ? enc_pos_
                        : rep_trig_ ? data_pos_ + fill_ : static_cast<uint64_t>(captured_ + spill_used_ + spill_drop_));
        // flags (oep-if-capture §3.2): bit0 the probe dropped data - the chunk queue full, the DMA ring overrun, no free
        // stage, streaming's new data discarded with every segment unsent (§2.1 rule 1; a repeat with no free segment
        // pauses instead, state 5). bit1 (a sample taken a sample period or more late) never: PARLIO keeps its clock.
        out[13] = (lost_ || queue_overflow_ || overruns_ || stage_drops_ || (mode_ == 3 && dropped_)) ? cap::kStatusFlagDropped : 0;
        // bit2: rearm 1, between segments waiting for the next one's trigger (§3.2: now, not since start)
        if (rep_trig_ && rearm_ && rep_wait_ && rep_started_ && state_ == kStateCapturing) out[13] |= cap::kStatusFlagRearmWaiting;
      } else if (triggered_) {
        putU32(out + 1, kept_samples_ ? 1 : 0);   // the segment, complete or cut short by stop
        // the segment's bytes so far (a hole: its start, §2.2)
        putU64(out + 5, state_ == kStateError ? 0 : (static_cast<uint64_t>(filledSamples()) * width_ + 7) / 8);
        out[13] = (queue_overflow_ || overruns_ || trig_overrun_) ? cap::kStatusFlagDropped : 0;
      } else {
        putU32(out + 1, kept_samples_ ? 1 : 0);                                    // segments done
        putU64(out + 5, (static_cast<uint64_t>(kept_samples_) * width_ + 7) / 8);   // write position
        out[13] = 0;
      }
      putU32(out + 14, generation_);
      size_t used = 18;
      if (state_ == kStateError) {   // why (TLV 0x01 error): the PARLIO or its DMA would not start / run (1), a
        putTlvHeader(out + 18, cap::kTlvStatusAnswerError, 1);   // segment lost to the queue or the ring (2, §2.2)
        out[21] = error_;
        used = 22;
      }
      // where the dropped data went, a named extension (core §2.2; capture §3.2 lets a probe say more): the bytes no
      // stage was free for, the bytes the transport would not queue, the DMA chunks rewritten before they were copied,
      // the chunks the harvest queue had no room for - u32 each, since the start; then the way the data goes (u8: 1 the
      // zero-copy stages, 0 the store through the endpoint) and the stages held (u8); then the memory a capture draws
      // on now (u32 each): internal RAM's largest free block, DMA-capable internal RAM's, PSRAM's total (0: none); then
      // the most the DMA ring held not yet harvested since the start (u32, bytes; the stream's buffer in use); then the
      // spill behind it: the most it held since the start and its size (u32 each, bytes; 0 0: none)
      static const char kDrops[] = "io.github.open-embedded-probe.capture-drops";
      constexpr size_t kName = sizeof kDrops - 1, kValue = 1 + kName + 42;
      if (used + kTlvHeader + kValue <= capacity) {
        putTlvHeader(out + used, reg::kTagNamedExtension, kValue);
        uint8_t *v = out + used + kTlvHeader;
        v[0] = static_cast<uint8_t>(kName);
        memcpy(v + 1, kDrops, kName);
        putU32(v + 1 + kName, stage_drops_);
        putU32(v + 5 + kName, stage_unsent_);
        putU32(v + 9 + kName, overruns_);
        putU32(v + 13 + kName, queue_overflow_);
        v[17 + kName] = direct_ ? 1 : 0;
        v[18 + kName] = stage_count_;
        putU32(v + 19 + kName, static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
        putU32(v + 23 + kName, static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)));
        putU32(v + 27 + kName, static_cast<uint32_t>(heap_caps_get_total_size(MALLOC_CAP_SPIRAM)));
        putU32(v + 31 + kName, ring_peak_);
        putU32(v + 35 + kName, spill_peak_);
        putU32(v + 39 + kName, spill_ ? static_cast<uint32_t>(kSpillBytes) : 0);
        used += kTlvHeader + kValue;
      }
      // format 1's encoding cost, since the start (named, as above): cycles of the longest block (u32), of all (u64),
      // blocks (u32), channels carried raw summed over the blocks (u32), the CPU's clock (u32, Hz)
      static const char kEncode[] = "io.github.open-embedded-probe.capture-encode";
      constexpr size_t kEncName = sizeof kEncode - 1, kEncValue = 1 + kEncName + 24;
      if (format_ == cap::kDataFormatLogicBlocks && used + kTlvHeader + kEncValue <= capacity) {
        putTlvHeader(out + used, reg::kTagNamedExtension, kEncValue);
        uint8_t *v = out + used + kTlvHeader;
        v[0] = static_cast<uint8_t>(kEncName);
        memcpy(v + 1, kEncode, kEncName);
        putU32(v + 1 + kEncName, enc_max_cycles_);
        putU64(v + 5 + kEncName, enc_cycles_);
        putU32(v + 13 + kEncName, enc_blocks_);
        putU32(v + 17 + kEncName, enc_fallbacks_);
        putU32(v + 21 + kEncName, static_cast<uint32_t>(getCpuFrequencyMhz()) * 1000000u);
        used += kTlvHeader + kEncValue;
      }
      return completed(used);
    }
    case kOpRead: {   // generation(u32) position(u64) max(u16) [TLV]  ->  position(u64) flags(u8: bit0 more, bit1 gap) len(u16) data [TLV]
      constexpr size_t kHead = 11;
      if (getU32(p) != generation_) return wrongState(out, capacity);   // another generation (cause 6)
      if (capacity < kHead) return failed();
      poll();
      uint64_t position = getU64(p + 4);
      uint32_t max = getU16(p + 12);
      size_t room = capacity - kHead;   // how much read returns is the probe's (capture §3.2): max_read_ and the frame
      if (room > max_read_) room = max_read_;
      if (max > room) max = static_cast<uint32_t>(room);
      auto answer = [&](uint64_t pos, uint8_t flags, const uint8_t *data, uint32_t count) {
        putU64(out, pos);
        out[8] = flags;
        putU16(out + 9, static_cast<uint16_t>(count));
        if (count) memcpy(out + kHead, data, count);
        if (count && format_ == cap::kDataFormatTestCounter) putTestCounter(out + kHead, pos, count);
        return completed(kHead + count);
      };
      // no plan or configuration (released: the data went with it), no store: nothing to read
      if (state_ == kStateUnconfigured || (mode_ == 1 ? !buffer_ : (!store_ && !direct_))) return answer(position, 0, nullptr, 0);
      if (mode_ == 3) {   // streaming: what is still in the store, by stream position
        uint32_t serial = 0, offset = 0;
        uint8_t flags = 0;
        // zero-copy streaming keeps nothing to read back; gone (reused) or not captured yet: nothing, gap flag
        if (direct_ || !findSegment(position, serial, offset)) return answer(position, reg::common::kReadFlagsGap, nullptr, 0);
        uint32_t count = segmentLength(serial) - offset;
        if (count > max) { count = max; flags |= reg::common::kReadFlagsMore; }
        return answer(position, flags, store_ + static_cast<size_t>(infos_[serial % kInfos].slot) * segment_bytes_ + offset, count);
      }
      if (mode_ == 2) {   // completed segments the host has not released, by position (u64: no wrap to handle)
        // The positions count the bytes discarded while paused (capture §2.2): a position released, or in such a gap,
        // moves on to the next segment's start with the gap flag; one past the last segment answers empty there.
        const uint32_t done = completed_;
        auto length = [&](uint32_t serial) { return infos_[serial % kInfos].samples * width_ / 8; };
        const uint64_t end = done ? infos_[(done - 1) % kInfos].position + length(done - 1) : 0;
        uint32_t serial = released_;
        while (serial != done && infos_[serial % kInfos].position + length(serial) <= position) ++serial;
        if (serial == done) return answer(end, position < end ? reg::common::kReadFlagsGap : 0, nullptr, 0);
        uint8_t flags = 0;
        const uint64_t begin = infos_[serial % kInfos].position;
        if (position < begin) { position = begin; flags |= reg::common::kReadFlagsGap; }   // released, or discarded
        const uint32_t offset = static_cast<uint32_t>(position - begin);
        uint32_t count = length(serial) - offset;   // one segment per answer
        if (count > max) count = max;
        if (count < length(serial) - offset || serial + 1 != done) flags |= reg::common::kReadFlagsMore;
        return answer(position, flags, store_ + static_cast<size_t>(infos_[serial % kInfos].slot) * segment_bytes_ + offset, count);
      }
      const uint64_t have = (static_cast<uint64_t>(kept_samples_) * width_ + 7) / 8;   // the segment's bytes
      if (position > have) position = have;
      uint32_t count = static_cast<uint32_t>(have - position);
      uint8_t flags = 0;
      if (count > max) { count = max; flags |= reg::common::kReadFlagsMore; }
      return answer(position, flags, buffer_ + position, count);
    }
    case kOpSegments: {   // from_serial(u32) [TLV]  ->  more(u8) count(u8) count x segment info [TLV] (37 bytes each)
      // common §1.3's serial paging (capture §3.2): next is serial_done, kept the segments whose info is remembered
      if (capacity < 2 + kInfoBytes) return failed();
      poll();
      uint32_t next = 0, oldest = 0;
      if (mode_ == 2 || mode_ == 3) {
        const uint32_t kept = kept_infos_;   // before completed_ (finishSegment's order): never more than are there
        next = completed_;
        oldest = next - kept;
      } else {
        next = kept_samples_ ? 1 : 0;   // one-shot: serial 0 once it is there
      }
      uint32_t k = serialPageStart(getU32(p), oldest, next);
      uint8_t count = 0;
      size_t used = 2;
      for (; k != next && used + kInfoBytes <= capacity && count < 255; ++k, ++count)   // no element length (core §2.3)
        used += mode_ == 2 || mode_ == 3 ? infoBytes(infos_[k % kInfos], out + used) : segmentInfo(out + used);
      out[0] = k != next ? 1 : 0;   // more
      out[1] = count;
      return completed(used);
    }
    case kOpRelease: {   // generation(u32) serial(u32) [TLV]: the finished segments up to serial may be reused (repeat)
      if (getU32(p) != generation_) return wrongState(out, capacity);
      if (mode_ != 2) return completed();   // nothing to release: success
      // capture §3.2: finished segments at or before serial (core §2.6), never one not finished yet - a serial at or
      // past serial_done releases every finished one; one already released, nothing
      const uint32_t serial = getU32(p + 4), done = completed_, released = released_;
      if (!serialBefore(serial, released)) released_ = serialBefore(serial, done) ? serial + 1 : done;
      return completed();
    }
    case kOpForce:   // waiting for the trigger: the trigger is the sample of this instant; otherwise nothing to do
      // (a repeat with a trigger: also rearm 1's wait between segments, §2.1)
      if (((state_ == kStateWaiting && trig_phase_ == 0) || (rep_trig_ && rep_wait_ && state_ == kStateCapturing)) && !force_) {
        const uint64_t now = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
        force_sample_ = samplesIn(now > start_ns_ ? now - start_ns_ : 0, rate_num_, rate_den_);
        __atomic_thread_fence(__ATOMIC_RELEASE);   // the harvest task (the other core) reads it once force_ is set
        force_ = true;
      }
      return completed();
    case kOpTriggers: {   // generation(u32) from_serial(u32) [TLV] -> more(u8) count(u8) count x record [TLV] (§3.10)
      const Result parsed = plainTail(tail, p, n, 8, out, capacity);
      if (refused(parsed)) return parsed;
      if (getU32(p) != generation_) return wrongState(out, capacity);   // another generation (cause 6)
      if (capacity < 2) return failed();
      poll();
      constexpr size_t kRecordBytes = 24;   // serial(u32) sample(u64) trigger_ns(u64) uncertainty_ns(u32)
      const uint32_t next = rep_trig_ && records_ ? static_cast<uint32_t>(record_next_) : 0;
      __atomic_thread_fence(__ATOMIC_ACQUIRE);
      const uint32_t kept = next < kTriggerRecords ? next : kTriggerRecords;
      uint32_t k = serialPageStart(getU32(p + 4), next - kept, next);   // common §1.3's serial paging
      uint8_t count = 0;
      size_t used = 2;
      for (; k != next && used + kRecordBytes <= capacity && count < 255; ++k, ++count) {
        const TriggerRecord &r = records_[k % kTriggerRecords];
        putU32(out + used, r.serial);
        putU64(out + used + 4, r.sample);
        putU64(out + used + 12, r.ns);
        putU32(out + used + 20, r.uncertainty);
        used += kRecordBytes;
      }
      out[0] = k != next ? 1 : 0;
      out[1] = count;
      return completed(used);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
