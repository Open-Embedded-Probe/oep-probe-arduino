// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepCapture.h"

// OEP_HOST_FAKE_PARLIO: a host test with fakes of the PARLIO RX driver and the heap (tests/host/shim)
#if (defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)) || defined(OEP_HOST_FAKE_PARLIO)

#include <esp_cache.h>
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
                 kTagActualSegments = cap::kTlvConfigureAnswerActualSegments,
                 kTagBlocking = cap::kTlvConfigureAnswerBlockingMs };
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
  const Chunk chunk = {static_cast<const uint8_t *>(e->data), e->recv_bytes, self->produced_};
  BaseType_t woken = pdFALSE;
  if (xQueueSendFromISR(self->queue_, &chunk, &woken) != pdTRUE) ++self->queue_overflow_;
  return woken == pdTRUE;
}

// Copy one finished DMA chunk into the segment being filled; without a free segment the bytes are dropped and the
// next segment starts with a gap mark. Runs on the harvest task only.
void LogicCapture::harvest(const Chunk &chunk) {
  if (lost_) return;                           // stopped taking data (capture §2.2): poll() ends the track
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
      gap_pending_ = false;
    }
    memcpy(store_ + static_cast<size_t>(slot) * segment_bytes_ + fill_, chunk.data + at, n);
    if (produced_ - static_cast<uint32_t>(captured_) > kRingBytes) {   // the DMA came round and rewrote these bytes while (or before) we copied
      ++overruns_;
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

// Streaming, zero-copy transport: copy one finished DMA chunk into the current stage; with no stage free the bytes are
// dropped and the stream position skips them. Harvest task only.
void LogicCapture::harvestDirect(const Chunk &chunk) {
  if (lost_) return;
  if (missed(chunk)) { loseSegment(); return; }   // chunks lost to the queue: not data to send (capture §2.2)
  size_t at = 0;
  while (at < chunk.length) {
    if (stage_cur_ < 0 && !takeStage()) {
      const size_t rest = chunk.length - at;
      stage_drops_ += rest;
      dropped_ += rest;
      captured_ += rest;
      carry_ = false;   // the held byte goes with the drop (the position jump covers it)
      return;
    }
    const size_t n = min(static_cast<size_t>(stage_data_ - stage_fill_), chunk.length - at);
    memcpy(stage_[stage_cur_] + kPushHead + stage_fill_, chunk.data + at, n);
    if (produced_ - static_cast<uint32_t>(captured_) > kRingBytes) {   // the DMA rewrote these bytes
      ++overruns_;
      loseSegment();
      return;
    }
    stage_fill_ += n;
    at += n;
    captured_ += n;
    if (stage_fill_ == stage_data_) sendStage();
  }
}

// A free stage becomes the current one, starting with the byte held back from the last frame, if any.
bool LogicCapture::takeStage() {
  const uint32_t free = stage_free_;
  if (free == 0) return false;
  stage_cur_ = __builtin_ctz(free);
  __atomic_fetch_and(&stage_free_, ~(1u << stage_cur_), __ATOMIC_ACQ_REL);
  stage_fill_ = 0;
  stage_pos_ = captured_;
  if (carry_) {
    stage_[stage_cur_][kPushHead] = carry_byte_;
    stage_fill_ = 1;
    --stage_pos_;
    carry_ = false;
  }
  stage_since_ = millis();
  return true;
}

// Hand the current stage to the transport as one push frame (or give it back if nobody may receive it now). A full
// stage is a whole number of 512-byte packets and the host's read runs on into the next one. A partial one (sent for
// max_delay_ms, or at stop) that happens to be whole packets would leave the host's read open (the direct build sends
// no zero-length packet), so its last byte is held back for the next frame and this one ends with a short packet.
void LogicCapture::sendStage() {
  if (stage_cur_ < 0) return;
  uint8_t *s = stage_[stage_cur_];
  const int index = stage_cur_;
  uint32_t fill = stage_fill_;
  stage_cur_ = -1;
  stage_fill_ = 0;
  DirectTransport *tr = endpoint_.direct();
  if (fill > 1 && fill < stage_data_ && (kPushHead + fill + kPushTail) % 512 == 0 && tr && tr->queued() == 0) {
    carry_byte_ = s[kPushHead + fill - 1];
    carry_ = true;
    --fill;
  }
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
  putTlvHeader(s + kPushHead + fill, cap::kTlvDataGeneration, 4);
  putU32(s + kPushHead + fill + kTlvHeader, generation_);
  if (!t->queueData(s, kPushHead + fill + kPushTail, stageDone, this)) __atomic_fetch_or(&stage_free_, 1u << index, __ATOMIC_ACQ_REL);
}

void LogicCapture::stageDone(void *context, const uint8_t *buffer) {
  auto *self = static_cast<LogicCapture *>(context);
  for (uint8_t i = 0; i < self->stage_count_; ++i)
    if (self->stage_[i] == buffer) __atomic_fetch_or(&self->stage_free_, 1u << i, __ATOMIC_ACQ_REL);
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
  return stage_count_ >= 2;
}

void LogicCapture::freeStages() {   // the stages stay allocated (openStages); wait until the transport gives them back
  if (!stage_count_) return;
  const uint32_t all = (1u << stage_count_) - 1;
  for (int i = 0; i < 500 && (stage_free_ & all) != all; ++i) delay(2);
}

// The first sample of `data` (the stream's sample first_sample on), at or after min_at, where the trigger holds: at,
// its index (the ones before min_at, before the pretrigger has filled, only move the last level on). A byte whose
// samples cannot trigger (the channel's bits masked: all at the known level for an edge, none at the level wanted for
// a level) is skipped whole; the rest is looked at sample by sample. The last level carries over for edges.
bool LogicCapture::findTrigger(const uint8_t *data, size_t length, uint64_t first_sample, uint64_t &at, uint64_t min_at) {
  const uint8_t w = width_, k = trig_role_;
  const bool edge = trig_type_ == cap::kTriggerEdge;
  const uint8_t want = trig_value_ & 1;
  auto hits = [&](uint8_t level) {
    const bool hit = edge ? have_level_ && level != last_level_ && (trig_value_ == 2 || (trig_value_ == 0) == (level == 1))
                          : level == want;
    last_level_ = level;
    have_level_ = true;
    return hit;
  };
  if (w <= 8) {
    const uint8_t per = 8 / w;
    uint8_t mask = 0;
    for (uint8_t s = 0; s < per; ++s) mask |= static_cast<uint8_t>(1u << (s * w + k));
    for (size_t b = 0; b < length; ++b) {
      const uint8_t v = data[b] & mask;
      if (edge ? have_level_ && v == (last_level_ ? mask : 0) : v == (want ? 0 : mask)) continue;
      for (uint8_t s = 0; s < per; ++s)
        if (hits((data[b] >> (s * w + k)) & 1) && first_sample + static_cast<uint64_t>(b) * per + s >= min_at) {
          at = first_sample + static_cast<uint64_t>(b) * per + s;
          return true;
        }
    }
    return false;
  }
  for (size_t i = 0; i + 1 < length; i += 2) {   // 16-bit samples
    const uint16_t v = static_cast<uint16_t>(data[i] | data[i + 1] << 8);
    if (hits((v >> k) & 1) && first_sample + i / 2 >= min_at) { at = first_sample + i / 2; return true; }
  }
  return false;
}

// A one-shot with a trigger: look for it in each chunk; once found, the segment starts pretrigger samples before it
// (from what the ring still has) and takes the chunks after it until full. Harvest task only.
void LogicCapture::harvestTriggered(const Chunk &chunk) {
  if (const uint32_t skip = missed(chunk)) {           // chunks lost before this one: not searched; still in the ring
    captured_ += skip;
    have_level_ = false;
  }
  const uint64_t base = captured_;                     // the stream byte of chunk.data[0]
  const size_t ring_at = static_cast<size_t>(chunk.data - ring_);
  captured_ += chunk.length;
  if (trig_phase_ == 2) return;
  const uint32_t ahead = produced_ - static_cast<uint32_t>(base);   // what the DMA wrote since this chunk began
  if (ahead > kRingBytes) {                            // rewritten before it was looked at
    if (trig_phase_ == 1) trig_overrun_ = true;
    have_level_ = false;
    ++overruns_;
    return;
  }
  auto ringByte = [&](uint64_t g) {                    // the ring position of stream byte g (g near base)
    const int64_t off = static_cast<int64_t>(ring_at) + static_cast<int64_t>(g - base);
    return static_cast<size_t>(((off % static_cast<int64_t>(kRingBytes)) + kRingBytes) % kRingBytes);
  };
  // the stream bytes this chunk adds to the segment: from where it is filled to (a lost chunk's bytes from the ring)
  uint64_t from = seg_first_sample_ * width_ / 8 + filled_;
  if (trig_phase_ == 0) {
    const uint64_t first = base * 8 / width_;
    uint64_t t = first;
    if (follow_) {   // the group's trigger, at sample ext_sample_: once it is known and captured
      if (!ext_ready_) return;
      __atomic_thread_fence(__ATOMIC_ACQUIRE);
      t = ext_sample_;
      if (t >= (base + chunk.length) * 8 / width_) return;
    } else if (force_) force_ = false;
    else if (!findTrigger(chunk.data, chunk.length, first, t, pretrigger_)) return;   // once the pretrigger is in
    uint64_t s0 = t > pre() ? t - pre() : 0;
    // not before the oldest byte the ring still holds (with a margin for the DMA running on)
    const uint64_t newest = base + ahead;
    const uint64_t oldest = newest > kRingBytes - 8192 ? newest - (kRingBytes - 8192) : 0;
    if (s0 * width_ / 8 < oldest) s0 = (oldest * 8 + width_ - 1) / width_;
    if (width_ < 8) {                                // a whole byte: up to 8 / w - 1 samples earlier, or later when
      s0 -= s0 % (8 / width_);                       // that would leave the trigger past the segment's last sample
      if (t >= s0 && t - s0 >= samples_) s0 += 8 / width_;   // (a pretrigger close to samples: one fewer byte before it)
    }
    seg_first_sample_ = s0;
    trigger_index_ = t >= s0 ? static_cast<uint32_t>(t - s0) : 0xFFFFFFFFu;   // before the ring's oldest: gone
    if (t < s0) trig_overrun_ = true;
    from = s0 * width_ / 8;
    filled_ = 0;
    trig_phase_ = 1;
  }
  for (uint64_t g = from; g < base + chunk.length && filled_ < bytes_; ) {   // copy in runs up to the ring's end
    const size_t r = ringByte(g);
    size_t n = kRingBytes - r;
    if (n > base + chunk.length - g) n = static_cast<size_t>(base + chunk.length - g);
    if (n > bytes_ - filled_) n = bytes_ - filled_;
    memcpy(buffer_ + filled_, ring_ + r, n);
    filled_ += n;
    g += n;
  }
  if (produced_ - static_cast<uint32_t>(from) > kRingBytes) trig_overrun_ = true;   // rewritten while we copied
  if (filled_ >= bytes_) {
    trig_phase_ = 2;
    done_ = true;
  }
}

Result LogicCapture::startTriggered(uint8_t *out, size_t capacity) {
  if (capacity < 8) return failed();
  kept_samples_ = 0;   // the last generation's segment goes
  kept_short_ = false;
  trig_phase_ = 0;
  have_level_ = trig_overrun_ = ext_ready_ = false;
  lost_ = false;
  lost_pos_ = 0;
  error_ = cap::kErrorPeripheral;
  force_ = trig_type_ == cap::kTriggerImmediate && !follow_;   // a ring left by following: start at once
  filled_ = fill_ = 0;
  trigger_index_ = 0xFFFFFFFFu;
  // a follower's trigger is the group's event; an immediate start (the ring a group's following left) sends none
  reported_trigger_ = follow_ || trig_type_ == cap::kTriggerImmediate;
  done_ = false;
  produced_ = queue_overflow_ = overruns_ = 0;
  captured_ = 0;
  if (!reopenUnit()) { fail(cap::kErrorPeripheral); return failed(); }
  xQueueReset(queue_);
  harvesting_ = true;
  if (xTaskCreatePinnedToCore(harvestTask, "oep_harvest", 4096, this, 5, &task_, 0) != pdPASS) {
    harvesting_ = false;
    return failed();
  }
  parlio_receive_config_t rc = {};
  rc.delimiter = delimiter_;
  rc.flags.partial_rx_en = true;
  if (parlio_rx_unit_receive(unit_, ring_, kRingBytes, &rc) != ESP_OK) { stopRepeat(); fail(cap::kErrorPeripheral); return failed(); }
  start_ns_ = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
  if (parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, true) != ESP_OK) { stopRepeat(); fail(cap::kErrorPeripheral); return failed(); }
  state_ = kStateWaiting;
  generation_ = nextGeneration(generation_);   // 1 after 0xFFFFFFFF, never 0 (capture §3.2)
  putU32(out, 0);
  putU32(out + 4, generation_);
  return completed(8);
}

void LogicCapture::finishSegment(uint32_t bytes, uint8_t flags) {
  Info &info = infos_[completed_ % kInfos];
  info.samples = bytes * 8 / width_;
  info.flags = flags;
  __atomic_thread_fence(__ATOMIC_RELEASE);
  fill_ = 0;
  if (segment_count_) fill_slot_ = (fill_slot_ + 1) % segment_count_;
  ++completed_;   // wraps after 0xFFFFFFFF (capture §2.2)
  __atomic_thread_fence(__ATOMIC_RELEASE);
  if (kept_infos_ < kInfos - 1) kept_infos_ = kept_infos_ + 1;   // after completed_: a reader takes this first (keptSerials)
}

// Harvest task: the data being gathered has a hole (capture §2.2). The segment being filled - in zero-copy streaming the
// stage not yet sent, and the byte held back for the next one - is dropped, never handed out; nothing more is taken.
// write_pos stays at its start. poll() (failLost) stops the track.
void LogicCapture::loseSegment() {
  if (direct_) {
    lost_pos_ = stage_cur_ >= 0 ? stage_pos_ : captured_ - (carry_ ? 1 : 0);
    if (stage_cur_ >= 0) __atomic_fetch_or(&stage_free_, 1u << stage_cur_, __ATOMIC_ACQ_REL);
    stage_cur_ = -1;
    stage_fill_ = 0;
    carry_ = false;
  } else {
    lost_pos_ = fill_ ? infos_[completed_ % kInfos].position : captured_;
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
    while (self->harvesting_) {
      const bool got = xQueueReceive(self->queue_, &chunk, pdMS_TO_TICKS(2)) == pdTRUE;
      if (got) self->harvestDirect(chunk);
      if (self->stage_cur_ < 0 || self->stage_fill_ == 0) continue;
      // the subscriber's batching: at least min_bytes, or max_delay_ms after the stage's first byte (0, 0: when idle)
      uint16_t fn = 0, min_bytes = 0;
  uint32_t max_delay = 0;
      if (!self->endpoint_.directPush(*self, fn, min_bytes, max_delay)) continue;
      const bool idle = uxQueueMessagesWaiting(self->queue_) == 0;
      if ((min_bytes == 0 && max_delay == 0 && idle) || (min_bytes && self->stage_fill_ >= min_bytes) ||
          (max_delay && millis() - self->stage_since_ >= max_delay))
        self->sendStage();
    }
    while (xQueueReceive(self->queue_, &chunk, 0) == pdTRUE) self->harvestDirect(chunk);
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
  putU32(mode + 1, kSegmentBytes * 8);             // max samples at w = 1
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
  // immediate, level, edge (one-shot); the most pretrigger the ring can give back (the narrowest sample)
  uint8_t trig[8];                                 // types(u32) max_pretrigger(u32)
  putU32(trig, (1u << cap::kTriggerImmediate) | (1u << cap::kTriggerLevel) | (1u << cap::kTriggerEdge));
  putU32(trig + 4, kSegmentBytes * 8 - 1);         // at w = 1 (below the segment); a wider sample's is less (configure)
  w.put(cap::kTlvDescribeTrigger, trig, sizeof trig);
  return w.ok() ? w.length() : 0;
}

uint8_t LogicCapture::planCheck(const RoleAssignment *roles, size_t count) {
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
  triggered_ = follow_ = false;
  trig_type_ = 0;
  pretrigger_ = 0;
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
  const size_t held = store_ ? store_bytes_ : 0;   // freed before the next store is taken
  const size_t psram = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) + held;
  if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0 && psram > 2 * kPsramReserve) {
    caps = MALLOC_CAP_SPIRAM;
    return psram - kPsramReserve;
  }
  caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
  size_t internal = heap_caps_get_largest_free_block(caps) + held;
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
  if (!open(rate_hz, width, ring ? kRingBytes : bytes, num, den)) return false;
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
  static const uint8_t kKnown[] = {kTagMode, kTagRate, kTagSamples, kTagSegments, kTagTrigger, kTagPretrigger};
  Tail tail;
  Result unknown_critical;
  const Result parsed = tail.parse(p, n, kKnown, out, capacity, &unknown_critical);
  if (refused(parsed)) return parsed;
  struct Item { uint8_t tag; size_t size; const uint8_t *v; bool critical; };
  Item items[] = {{kTagMode, 1, nullptr, false},    {kTagRate, 4, nullptr, false},    {kTagSamples, 4, nullptr, false},
                  {kTagSegments, 4, nullptr, false}, {kTagTrigger, 6, nullptr, false}, {kTagPretrigger, 4, nullptr, false}};
  Item &mode_tlv = items[0], &rate_tlv = items[1], &samples_tlv = items[2], &segments_tlv = items[3],
       &trigger_tlv = items[4], &pretrigger_tlv = items[5];
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
  // type(u8) role(u8) value(u32): level and edge in a one-shot (repeat and streaming start at once: immediate only).
  // A role outside the plan is refused unavailable cause 6 below, after every unsupported value (core §4.3).
  uint8_t trig_type = 0, trig_role = 0;
  uint32_t trig_value = 0;
  uint32_t pretrigger = 0;
  if (const uint8_t *v = trigger_tlv.v) {
    const uint32_t value = getU32(v + 2);
    const bool ok = v[0] == cap::kTriggerImmediate ||
                    (mode == cap::kModeOneShot &&
                     ((v[0] == cap::kTriggerLevel && value <= 1) || (v[0] == cap::kTriggerEdge && value <= 2)));
    if (!ok) return Tail::refuse(kTagTrigger, trigger_tlv.critical, out, capacity);
    trig_type = v[0];
    trig_role = v[1];
    trig_value = value;
  }
  if (pretrigger_tlv.v) pretrigger = getU32(pretrigger_tlv.v);   // checked against samples below
  const uint8_t width = widthFor(channels_);
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
    const uint32_t max_samples = static_cast<uint32_t>(kSegmentBytes * 8 / width);
    if (samples > max_samples) samples = max_samples;
    bytes = (samples * width + 7) / 8;
    bytes = (bytes + 127) & ~127u;                                   // whole cache lines for the DMA
    if (bytes > kSegmentBytes) bytes = kSegmentBytes;
    samples = bytes * 8 / width;
  }
  // the pretrigger is what the ring can give back at this sample width (describe declares the most, at w = 1), less
  // than the segment's samples as rounded (capture §3.3); only with a trigger (above), so a one-shot
  const uint32_t max_pretrigger = static_cast<uint32_t>(kPretriggerBytes * 8 / width);
  if (pretrigger && (pretrigger >= samples || pretrigger > max_pretrigger))
    return Tail::refuse(kTagPretrigger, pretrigger_tlv.critical, out, capacity);
  // no plan, or a trigger's role not in it (type 0's role is not looked at): unavailable cause 6 (capture §3.2, §3.3)
  if (channels_ == 0 || (trig_type != 0 && trig_role >= channels_)) return wrongState(out, capacity);
  if (!query && bound()) return boundInGroup(*this, out, capacity);   // the group's now (cause 4)
  if ((state_ == kStateCapturing || state_ == kStatePaused || state_ == kStateWaiting) && !query) return wrongState(out, capacity);
  // Not even two segments of store, or no DMA ring for a mode that runs through it: refused unavailable cause 3
  // before anything is touched (core §4.3; it answered failed with an empty payload).
  if (mode == 3 && actual_segments < 2) return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);
  const bool triggered = mode == cap::kModeOneShot && trig_type != cap::kTriggerImmediate;
  if (!query && (triggered || mode != cap::kModeOneShot) && !takeRing())
    return unavailable(out, capacity, reg::core::kUnavailableCauseStorageFull);

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
    direct_ = mode == 3 && endpoint_.direct() != nullptr;
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
    trig_type_ = trig_type;
    trig_role_ = trig_role;
    trig_value_ = trig_value;
    pretrigger_ = pretrigger;
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
  uint8_t layout[2 + kMaxChannels] = {width, channels_};
  for (uint8_t k = 0; k < channels_; ++k) layout[2 + k] = k;
  w.put(kTagLayout, layout, 2 + channels_);
  // every answer row of the mode (capture §3.3): actual_samples in modes 1 / 2, actual_segments in mode 2 only
  if (mode != cap::kModeStreaming) w.u32(kTagActualSamples, samples);
  if (mode == cap::kModeRepeat) w.u32(kTagActualSegments, actual_segments);
  w.u32(kTagBlocking, 0);
  return w.ok() ? completed(w.length()) : failed();
}

LogicCapture::Open LogicCapture::openRepeat(uint32_t rate_hz, uint8_t width, uint32_t samples, uint32_t segments, uint32_t &num,
                              uint32_t &den, uint32_t &actual_samples, uint32_t &actual_segments) {
  segment_bytes_ = samples * width / 8;
  segment_count_ = segments;
  // a new store: nothing captured, nothing left to push from the last run (its unsent bytes are gone with it)
  completed_ = released_ = fill_ = 0;
  sent_seg_ = 0;
  sent_off_ = 0;
  captured_ = dropped_ = 0;
  queue_ = xQueueCreate(128, sizeof(Chunk));
  if (!takeRing() || !queue_) return Open::kNoMemory;
  // Without internal RAM for two stages (taken by the DMA ring, a trigger's segment, the rest of the firmware), the
  // stream goes the copied way instead: the segments in the store, pushed by the endpoint like any notification.
  // It failed configure (0.0.28 on the bench: a streaming configure after other captures answered failed).
  if (direct_ && !openStages()) direct_ = false;
  if (!direct_) {
    uint32_t caps = 0;
    storeBudget(caps);
    store_bytes_ = static_cast<size_t>(segment_bytes_) * segment_count_;
    store_ = static_cast<uint8_t *>(heap_caps_malloc(store_bytes_, caps));
    if (!store_) return Open::kNoMemory;
  }
  if (!openUnit(rate_hz, width, true, 0, num, den)) return Open::kFailed;
  actual_samples = samples;
  actual_segments = segment_count_;
  return Open::kOk;
}

LogicCapture::Open LogicCapture::openTriggered(uint32_t rate_hz, uint8_t width, uint32_t bytes, uint32_t &num, uint32_t &den) {
  queue_ = xQueueCreate(128, sizeof(Chunk));
  uint32_t caps = 0;
  storeBudget(caps);                         // PSRAM when there is some: the segment is filled by the CPU, not the DMA
  buffer_ = static_cast<uint8_t *>(heap_caps_malloc(bytes, caps));
  if (!takeRing() || !queue_ || !buffer_) return Open::kNoMemory;
  return openUnit(rate_hz, width, true, 0, num, den) ? Open::kOk : Open::kFailed;
}

Result LogicCapture::startRepeat(uint8_t *out, size_t capacity) {
  if (capacity < 8) return failed();
  completed_ = released_ = fill_ = queue_overflow_ = overruns_ = stage_drops_ = 0;
  fill_slot_ = 0;
  kept_infos_ = 0;
  lost_ = false;
  lost_pos_ = 0;
  error_ = cap::kErrorPeripheral;
  carry_ = false;
  produced_ = 0;
  sent_seg_ = 0;
  sent_off_ = 0;
  captured_ = dropped_ = 0;
  gap_pending_ = paused_ = false;
  reported_ = 0;
  if (!reopenUnit()) { fail(cap::kErrorPeripheral); return failed(); }
  xQueueReset(queue_);
  harvesting_ = true;
  if (xTaskCreatePinnedToCore(harvestTask, "oep_harvest", 4096, this, 5, &task_, 0) != pdPASS) {
    harvesting_ = false;
    return failed();
  }
  parlio_receive_config_t rc = {};
  rc.delimiter = delimiter_;
  rc.flags.partial_rx_en = true;
  if (parlio_rx_unit_receive(unit_, ring_, kRingBytes, &rc) != ESP_OK) { stopRepeat(); fail(cap::kErrorPeripheral); return failed(); }
  start_ns_ = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
  if (parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, true) != ESP_OK) { stopRepeat(); fail(cap::kErrorPeripheral); return failed(); }
  state_ = kStateCapturing;
  generation_ = nextGeneration(generation_);   // 1 after 0xFFFFFFFF, never 0 (capture §3.2)
  putU32(out, 0);
  putU32(out + 4, generation_);
  return completed(8);
}

void LogicCapture::stopRepeat() {
  if (!harvesting_ && !task_) return;
  if (unit_ && delimiter_) parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
  harvesting_ = false;
  for (int i = 0; i < 100 && task_; ++i) delay(2);   // the task drains what arrived, then ends
  if (direct_) {   // the task is gone: send what is left, the held byte too
    sendStage();
    if (carry_ && takeStage()) sendStage();
    carry_ = false;
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
  follow_ = true;
  uint8_t out[8];   // blocking_ms, generation (a 4-byte one failed every follower's start since the generations)
  const Result r = startTriggered(out, sizeof out);
  return r.resolution == kResolutionCompleted && r.detail == kOutcomeSuccess;
}

void LogicCapture::trackTriggerAt(uint64_t ns) {
  ext_sample_ = samplesIn(ns > start_ns_ ? ns - start_ns_ : 0, rate_num_, rate_den_);
  __atomic_thread_fence(__ATOMIC_RELEASE);   // the harvest task (the other core) reads it once ext_ready_ is set
  ext_ready_ = true;
}

bool LogicCapture::trackTriggerNs(uint64_t &ns) const {
  if (!triggered_ || follow_ || trig_phase_ < 1) return false;
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
  putU32(out + 28, 0xFFFFFFFFu);
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
    if (subscribed_) {   // serial(u32) trigger_index(u32) trigger_ns(u64) generation(u32)
      uint8_t e[20];
      putU32(e, 0);
      putU32(e + 4, trigger_index_);
      putU64(e + 8, start_ns_ + nsOf(seg_first_sample_ + trigger_index_));
      putU32(e + 16, generation_);
      endpoint_.event(*this, cap::kEventTriggered, e, sizeof e);
    }
  }
  if (trig_overrun_ && (state_ == kStateCapturing || state_ == kStateWaiting)) { failLost(); return; }   // a hole (§2.2)
  if (trig_phase_ != 2 || state_ != kStateCapturing) return;
  stopRepeat();
  if (trig_overrun_) { failLost(); return; }
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
  constexpr size_t kHead = 10, kTail = kTlvHeader + 4;   // the generation TLV after the data
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
  putTlvHeader(out + kHead + n, cap::kTlvDataGeneration, 4);
  putU32(out + kHead + n + kTlvHeader, generation_);
  sent_off_ += n;
  if (finished && sent_off_ >= length) { sent_off_ = 0; sent_seg_ = serial + 1; released_ = serial + 1; }
  return kHead + n + kTail;
}

Result LogicCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  if (op == kOpConfigure || op == kOpQuery) return configure(p, n, out, capacity, op == kOpQuery);
  // Every other request: a fixed part (start / stop / force / status: none; read: generation position max = 16;
  // segments: 4; release: generation serial = 8), then TLVs, none of which these ops read.
  const size_t fixed = op == kOpRead ? 16 : op == kOpSegments ? 4 : op == kOpRelease ? 8 : 0;
  Tail tail;
  if (op >= kOpStart && op <= kOpRelease) {
    const Result parsed = plainTail(tail, p, n, fixed, out, capacity);
    if (refused(parsed)) return parsed;
  }
  // bound in a capture-group: the group starts and stops it (unavailable cause 4, after the request's form, core §4.3)
  if (bound() && !group_op_ && (op == kOpStart || op == kOpStop || op == kOpForce)) return boundInGroup(*this, out, capacity);
  switch (op) {
    case kOpStart: {   // -> blocking_ms(u32) generation(u32) [TLV]
      if (state_ != kStateConfigured && state_ != kStateDone && state_ != kStateError) return wrongState(out, capacity);
      if (mode_ == 3 && !subscribed_) return wrongState(out, capacity);   // streaming pushes: subscribe first
      if (mode_ == 2 || mode_ == 3) return startRepeat(out, capacity);
      follow_ = false;
      if (triggered_) return startTriggered(out, capacity);
      if (capacity < 8) return failed();
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
      putU32(out, 0);              // blocking_ms: DMA, the probe keeps answering
      putU32(out + 4, generation_);
      return completed(8);
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
        const uint32_t got = trig_phase_ >= 1 ? static_cast<uint32_t>(static_cast<uint64_t>(filled_) * 8 / width_) : 0;
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
      if ((mode_ == 2 || mode_ == 3) && (state_ == kStateCapturing || state_ == kStatePaused)) {
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
        putU64(out + 5, lost_ ? lost_pos_ : static_cast<uint64_t>(captured_));
        // flags (oep-if-capture §3.2): bit0 the probe dropped data - the chunk queue full, the DMA ring overrun, no free
        // stage, streaming's new data discarded with every segment unsent (§2.1 rule 1; a repeat with no free segment
        // pauses instead, state 5). bit1 (a sample taken a sample period or more late) never: PARLIO keeps its clock.
        out[13] = (lost_ || queue_overflow_ || overruns_ || stage_drops_ || (mode_ == 3 && dropped_)) ? cap::kStatusFlagDropped : 0;
      } else if (triggered_) {
        putU32(out + 1, kept_samples_ ? 1 : 0);   // the segment, complete or cut short by stop
        putU64(out + 5, state_ == kStateError ? 0 : filled_);   // a segment with a hole: its start (§2.2)
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
      return completed(used);
    }
    case kOpRead: {   // generation(u32) position(u64) max(u32) [TLV]  ->  position(u64) flags(u8: bit0 more, bit1 gap) len(u32) data [TLV]
      constexpr size_t kHead = 13;
      if (getU32(p) != generation_) return wrongState(out, capacity);   // another generation (cause 6)
      if (capacity < kHead) return failed();
      poll();
      uint64_t position = getU64(p + 4);
      uint32_t max = getU32(p + 12);
      size_t room = capacity - kHead;   // how much read returns is the probe's (capture §3.2): max_read_ and the frame
      if (room > max_read_) room = max_read_;
      if (max > room) max = static_cast<uint32_t>(room);
      auto answer = [&](uint64_t pos, uint8_t flags, const uint8_t *data, uint32_t count) {
        putU64(out, pos);
        out[8] = flags;
        putU32(out + 9, count);
        if (count) memcpy(out + kHead, data, count);
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
    case kOpForce:   // waiting for the trigger: start now (the next chunk); otherwise nothing to do
      if (state_ == kStateWaiting && trig_phase_ == 0) force_ = true;
      return completed();
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
