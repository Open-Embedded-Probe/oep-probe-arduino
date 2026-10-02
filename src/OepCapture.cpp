// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepCapture.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)

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
// configure / query TLVs (oep-spec logic-capture §5.3); bit 7 of the tag = critical
enum : uint8_t { kTagMode = cap::kTlvConfigureMode, kTagRate = cap::kTlvConfigureRate,
                 kTagSamples = cap::kTlvConfigureSamples, kTagSegments = cap::kTlvConfigureSegments,
                 kTagTrigger = cap::kTlvConfigureTrigger, kTagPretrigger = cap::kTlvConfigurePretrigger };
enum : uint8_t { kTagActualRate = cap::kTlvConfigureAnswerActualRate, kTagLayout = cap::kTlvConfigureAnswerLayout,
                 kTagActualSamples = cap::kTlvConfigureAnswerActualSamples,
                 kTagActualSegments = cap::kTlvConfigureAnswerActualSegments, kTagTiming = cap::kTlvConfigureAnswerTiming,
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
  const Chunk chunk = {static_cast<const uint8_t *>(e->data), e->recv_bytes};
  self->produced_ += e->recv_bytes;
  BaseType_t woken = pdFALSE;
  if (xQueueSendFromISR(self->queue_, &chunk, &woken) != pdTRUE) ++self->queue_overflow_;
  return woken == pdTRUE;
}

// Copy one finished DMA chunk into the segment being filled; without a free segment the bytes are dropped and the
// next segment starts with a gap mark. Runs on the harvest task only.
void LogicCapture::harvest(const Chunk &chunk) {
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
    const uint32_t slot = completed_ % segment_count_;
    const size_t n = min(static_cast<size_t>(segment_bytes_ - fill_), chunk.length - at);
    if (fill_ == 0) {   // a new segment: remember where in time it starts
      Info &info = infos_[completed_ % kInfos];
      info.serial = completed_;
      info.position = mode_ == 3 ? captured_ : static_cast<uint64_t>(completed_) * segment_bytes_;
      const uint64_t first_sample = captured_ * 8 / width_;
      info.start_ns = start_ns_ + nsOf(first_sample);
      info.flags = gap_pending_ ? 1 : 0;
      gap_pending_ = false;
    }
    memcpy(store_ + static_cast<size_t>(slot) * segment_bytes_ + fill_, chunk.data + at, n);
    if (produced_ - static_cast<uint32_t>(captured_) > kRingBytes) {   // the DMA came round and rewrote these bytes while (or before) we copied
      const size_t rest = chunk.length - at;
      dropped_ += rest;
      captured_ += rest;
      ++overruns_;
      if (fill_) finishSegment(fill_, infos_[completed_ % kInfos].flags | 2);   // a segment is contiguous inside
      gap_pending_ = true;
      return;
    }
    __atomic_thread_fence(__ATOMIC_RELEASE);   // streaming reads the segment being filled from the other core
    fill_ += n;
    at += n;
    captured_ += n;
    if (fill_ == segment_bytes_) finishSegment(segment_bytes_, infos_[completed_ % kInfos].flags);
  }
}

// Streaming, zero-copy transport: copy one finished DMA chunk into the current stage; with no stage free the bytes are
// dropped and the stream position skips them. Harvest task only.
void LogicCapture::harvestDirect(const Chunk &chunk) {
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
    if (produced_ - static_cast<uint32_t>(captured_) > kRingBytes) {   // the DMA rewrote these bytes: drop them
      const size_t rest = chunk.length - at;
      dropped_ += rest;
      captured_ += rest;
      ++overruns_;
      if (stage_fill_) sendStage();   // a frame is contiguous; the next one starts after the gap
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
  s[kPushHead + fill] = cap::kTlvDataGeneration;
  s[kPushHead + fill + 1] = 4;
  putU32(s + kPushHead + fill + 2, generation_);
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
  uint64_t from = base;                                // the stream bytes this chunk adds to the segment
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
    uint64_t s0 = t > pretrigger_ ? t - pretrigger_ : 0;
    // not before the oldest byte the ring still holds (with a margin for the DMA running on)
    const uint64_t newest = base + ahead;
    const uint64_t oldest = newest > kRingBytes - 8192 ? newest - (kRingBytes - 8192) : 0;
    if (s0 * width_ / 8 < oldest) s0 = (oldest * 8 + width_ - 1) / width_;
    if (width_ < 8) s0 -= s0 % (8 / width_);         // a whole byte
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
  trig_phase_ = 0;
  have_level_ = trig_overrun_ = ext_ready_ = false;
  force_ = trig_type_ == cap::kTriggerImmediate && !follow_;   // a ring left by following: start at once
  filled_ = fill_ = 0;
  trigger_index_ = 0xFFFFFFFFu;
  reported_trigger_ = follow_;   // a follower's trigger is the group's event
  done_ = false;
  produced_ = queue_overflow_ = overruns_ = 0;
  captured_ = 0;
  xQueueReset(queue_);
  harvesting_ = true;
  if (xTaskCreatePinnedToCore(harvestTask, "oep_harvest", 4096, this, 5, &task_, 0) != pdPASS) {
    harvesting_ = false;
    return failed();
  }
  parlio_receive_config_t rc = {};
  rc.delimiter = delimiter_;
  rc.flags.partial_rx_en = true;
  if (parlio_rx_unit_receive(unit_, ring_, kRingBytes, &rc) != ESP_OK) { stopRepeat(); state_ = kStateError; return failed(); }
  start_ns_ = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
  if (parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, true) != ESP_OK) { stopRepeat(); state_ = kStateError; return failed(); }
  state_ = kStateWaiting;
  ++generation_;
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
  ++completed_;
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

size_t LogicCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  // role k = channel k (oep-if-capture: roles 0..), each on any of the probe's channels
  uint8_t roles[kMaxChannels];
  for (uint8_t k = 0; k < kMaxChannels; ++k) roles[k] = k;
  w.roleChannels(roles, kMaxChannels, table_.allowedMask());
  w.u32(kTagFeatures, cap::kFeaturesQuery | cap::kFeaturesForce | cap::kFeaturesNotify);
  uint8_t mode[10] = {1, 1};                       // one-shot, runs in the background (DMA)
  putU32(mode + 2, kSegmentBytes * 8);             // max samples at w = 1
  putU32(mode + 6, 1);                             // one segment
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  mode[0] = 2;                                     // repeat, in the background too: the most the store can ever hold
  putU32(mode + 2, kSegmentMaxRepeat * 8);         // (a declaration, core §7.3: not the free space of the moment)
  putU32(mode + 6, storeMax() / kSegmentMin);
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  mode[0] = 3;                                     // streaming: segments of 64 KiB, pushed (needs a subscription)
  putU32(mode + 2, kSegmentBytes * 8);
  putU32(mode + 6, kInfos);
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  uint8_t range[9];
  putU32(range, kMinHz);
  putU32(range + 4, kSourceHz);
  range[8] = 1;                                    // any value in range (fractional divider)
  w.put(cap::kTlvDescribeRateRange, range, sizeof range);
  const uint32_t list[] = {1000000, 2000000, 5000000, 10000000, 20000000, 40000000, 80000000, 160000000};
  uint8_t l[1 + sizeof list] = {8};                // n(u8) n x rate_hz(u32)
  for (size_t i = 0; i < 8; ++i) putU32(l + 1 + 4 * i, list[i]);
  w.put(cap::kTlvDescribeRateList, l, sizeof l);
  uint8_t lim[6] = {1, 8};                         // wch-protocols E033: 8 lines to 100 MHz, 16 lines to 48 MHz
  putU32(lim + 2, 100000000);
  w.put(cap::kTlvDescribeRateLimit, lim, sizeof lim);
  lim[1] = 16;
  putU32(lim + 2, 48000000);
  w.put(cap::kTlvDescribeRateLimit, lim, sizeof lim);
  uint8_t ch[5] = {kMaxChannels};                  // max(u8) layouts(u32): w in {1, 2, 4, 8, 16}
  putU32(ch + 1, 0b11111);
  w.put(cap::kTlvDescribeChannels, ch, sizeof ch);
  // immediate, level, edge (one-shot); the pretrigger the ring can give back at the widest sample (16 bits)
  uint8_t trig[8];                                 // types(u32) max_pretrigger(u32)
  putU32(trig, (1u << cap::kTriggerImmediate) | (1u << cap::kTriggerLevel) | (1u << cap::kTriggerEdge));
  putU32(trig + 4, kPretriggerBytes * 8 / 16);
  w.put(cap::kTlvDescribeTrigger, trig, sizeof trig);
  w.u32(cap::kTlvDescribeMaxRead, static_cast<uint32_t>(max_read_));
  w.u16(cap::kTlvDescribeSegmentRing, kInfos);                            // segment infos kept
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
  for (size_t i = 0; i < count; ++i) pins_[roles[i].role] = roles[i].channel;
  channels_ = static_cast<uint8_t>(count);
  state_ = kStateUnconfigured;
  return true;
}

void LogicCapture::planRelease() {
  close();
  channels_ = 0;
  state_ = kStateUnconfigured;
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

void LogicCapture::close() {
  stopRepeat();
  // the DMA ring stays allocated (see openStages): taking 128 KiB of internal RAM again and again fragmented it
  if (store_) { heap_caps_free(store_); store_ = nullptr; }
  store_bytes_ = 0;
  freeStages();
  direct_ = false;
  if (queue_) { vQueueDelete(queue_); queue_ = nullptr; }
  if (unit_) {
    if (delimiter_) parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
    parlio_rx_unit_disable(unit_);
  }
  if (delimiter_) { parlio_del_rx_delimiter(delimiter_); delimiter_ = nullptr; }
  if (unit_) { parlio_del_rx_unit(unit_); unit_ = nullptr; }
  if (buffer_) { heap_caps_free(buffer_); buffer_ = nullptr; }
  done_ = false;
}

Result LogicCapture::configure(const uint8_t *p, size_t n, uint8_t *out, size_t capacity, bool query) {
  static const uint8_t kKnown[] = {kTagMode, kTagRate, kTagSamples, kTagSegments, kTagTrigger, kTagPretrigger};
  Tail tail;
  const Result parsed = tail.parse(p, n, kKnown, out, capacity);   // unknown: critical refused, others ignored
  if (refused(parsed)) return parsed;
  // The known ones, each checked for shape (malformed) and for what this probe can do: a value it cannot honour
  // refuses the configure when critical and is ignored (and listed) when not.
  uint8_t mode = cap::kModeOneShot;
  uint32_t rate = 1000000, samples = 0, segments = 0;
  size_t len = 0;
  bool critical = false;
  if (const uint8_t *v = tail.find(kTagMode, len, &critical)) {
    if (len != 1) return rejected(kRejectMalformed);
    if (v[0] == cap::kModeOneShot || v[0] == cap::kModeRepeat || v[0] == cap::kModeStreaming) mode = v[0];
    else {
      const Result r = tail.refuse(kTagMode, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  if (const uint8_t *v = tail.find(kTagRate, len, &critical)) {
    if (len != 4) return rejected(kRejectMalformed);
    // out of range: the driver would silently run at 160 MHz instead
    if (getU32(v) >= kMinHz && getU32(v) <= kSourceHz) rate = getU32(v);
    else {
      const Result r = tail.refuse(kTagRate, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  if (const uint8_t *v = tail.find(kTagSamples, len)) {
    if (len != 4) return rejected(kRejectMalformed);
    samples = getU32(v);
  }
  if (const uint8_t *v = tail.find(kTagSegments, len)) {
    if (len != 4) return rejected(kRejectMalformed);
    segments = getU32(v);
  }
  if (channels_ == 0) return wrongState(out, capacity);   // plan first (unavailable cause 6)
  // type(u8) role(u8) value(u32): level and edge in a one-shot (repeat and streaming start at once: immediate only)
  uint8_t trig_type = 0, trig_role = 0;
  uint32_t trig_value = 0;
  uint32_t pretrigger = 0;
  bool pretrigger_critical = false;
  if (const uint8_t *v = tail.find(kTagTrigger, len, &critical)) {
    if (len != 6) return rejected(kRejectMalformed);
    const uint32_t value = getU32(v + 2);
    const bool ok = v[0] == cap::kTriggerImmediate ||
                    (mode == cap::kModeOneShot && v[1] < channels_ &&
                     ((v[0] == cap::kTriggerLevel && value <= 1) || (v[0] == cap::kTriggerEdge && value <= 2)));
    if (ok) { trig_type = v[0]; trig_role = v[1]; trig_value = value; }
    else {
      const Result r = tail.refuse(kTagTrigger, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  if (const uint8_t *v = tail.find(kTagPretrigger, len, &pretrigger_critical)) {   // checked against samples below
    if (len != 4) return rejected(kRejectMalformed);
    pretrigger = getU32(v);
  }
  if ((state_ == kStateCapturing || state_ == kStatePaused || state_ == kStateWaiting) && !query) return wrongState(out, capacity);
  const uint8_t width = widthFor(channels_);
  uint32_t actual_segments = 1;
  uint32_t store_caps = 0;
  const size_t budget = storeBudget(store_caps);
  if (mode == 3) {   // streaming: the store in at most kInfos segments (the host's samples / segments are hints)
    uint32_t seg = static_cast<uint32_t>(budget / kInfos) / kSegmentMin * kSegmentMin;
    if (seg < 64 * 1024) seg = 64 * 1024;
    if (seg > kSegmentMaxRepeat) seg = kSegmentMaxRepeat;
    while (seg > kSegmentMin && budget / seg < 4) seg /= 2;   // a small store: at least 4 segments
    actual_segments = static_cast<uint32_t>(budget / seg);
    if (actual_segments > kInfos) actual_segments = kInfos;
    if (actual_segments < 2) return failed();
    samples = seg * 8 / width;
  } else if (mode == 2) {   // repeat: segments of whole 4 KiB, as many as fit the PSRAM budget (or as asked)
    // in 64 bits: a samples above the limit is rounded down to it (capture §3.3), never wrapped (samples x width
    // overflowed u32 from 2^28 samples at w = 16: a segment of 0 bytes, and configure failed)
    const uint64_t asked = samples ? (static_cast<uint64_t>(samples) * width + 7) / 8 : 65536;
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
    if (samples == 0 || samples > max_samples) samples = max_samples;
    bytes = (samples * width + 7) / 8;
    bytes = (bytes + 127) & ~127u;                                   // whole cache lines for the DMA
    if (bytes > kSegmentBytes) bytes = kSegmentBytes;
    samples = bytes * 8 / width;
  }
  // the pretrigger is what the ring can give back, before the trigger inside the segment; only with a trigger. The
  // segment starts on a whole byte, up to 7 samples earlier (w < 8): the trigger stays inside it.
  const uint32_t max_pretrigger = static_cast<uint32_t>(kPretriggerBytes * 8 / width);
  // with an immediate trigger it is kept for a group that makes this track follow another's trigger
  if (pretrigger && (mode != cap::kModeOneShot || pretrigger + 8 > samples || pretrigger > max_pretrigger)) {
    const Result r = tail.refuse(kTagPretrigger, pretrigger_critical, out, capacity);
    if (refused(r)) return r;
    pretrigger = 0;
  }
  const bool triggered = mode == cap::kModeOneShot && trig_type != cap::kTriggerImmediate;

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
    if (!openRepeat(rate, width, samples, actual_segments, num, den, got_samples, got_segments)) {
      close();
      state_ = kStateError;
      return failed();
    }
    samples = got_samples;
    actual_segments = got_segments;
  } else if (triggered) {   // the repeat's ring, searched; the segment copied out of it (harvestTriggered)
    close();
    if (!openTriggered(rate, width, bytes, num, den)) {
      close();
      state_ = kStateError;
      return failed();
    }
  } else {
    close();
    if (!open(rate, width, bytes, num, den)) return failed();
    // The DMA writes the segment itself: internal RAM. The ring a trigger or repeat left allocated (128 KiB, kept so
    // that it does not fragment the heap) can leave no 64 KiB piece of it: 5 MHz x 4 ch x 130816 samples failed
    // configure after triggered captures (0.0.15, the X035 jig). Then the ring goes, and PSRAM is the last resort.
    buffer_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    if (!buffer_ && ring_) {
      heap_caps_free(ring_);
      ring_ = nullptr;
      buffer_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    }
    if (!buffer_) buffer_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, bytes, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM));
    parlio_rx_event_callbacks_t cb = {};
    cb.on_receive_done = receiveDone;
    parlio_rx_soft_delimiter_config_t d = {};
    d.sample_edge = PARLIO_SAMPLE_EDGE_POS;
    d.bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB;
    d.eof_data_len = bytes;
    if (!buffer_ || parlio_rx_unit_register_event_callbacks(unit_, &cb, this) != ESP_OK ||
        parlio_new_rx_soft_delimiter(&d, &delimiter_) != ESP_OK || parlio_rx_unit_enable(unit_, true) != ESP_OK) {
      close();
      state_ = kStateError;
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
  w.u32(kTagActualSamples, samples);
  w.u32(kTagActualSegments, actual_segments);
  uint8_t timing[5] = {static_cast<uint8_t>(den == 1 || kSourceHz % rate == 0 ? 0 : 1)};   // 1: fractional divider
  putU32(timing + 1, den == 1 ? 0 : 7);                               // one 160 MHz period, rounded up (ns)
  w.put(kTagTiming, timing, sizeof timing);
  w.u32(kTagBlocking, 0);
  return w.ok() ? tail.finish(completed(w.length()), out, capacity) : failed();
}

bool LogicCapture::openRepeat(uint32_t rate_hz, uint8_t width, uint32_t samples, uint32_t segments, uint32_t &num,
                              uint32_t &den, uint32_t &actual_samples, uint32_t &actual_segments) {
  segment_bytes_ = samples * width / 8;
  segment_count_ = segments;
  // a new store: nothing captured, nothing left to push from the last run (its unsent bytes are gone with it)
  completed_ = released_ = fill_ = 0;
  sent_seg_ = 0;
  sent_off_ = 0;
  captured_ = dropped_ = 0;
  if (!ring_) ring_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, kRingBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  queue_ = xQueueCreate(128, sizeof(Chunk));
  if (!ring_ || !queue_) return false;
  if (direct_) {
    if (!openStages()) return false;
  } else {
    uint32_t caps = 0;
    storeBudget(caps);
    store_bytes_ = static_cast<size_t>(segment_bytes_) * segment_count_;
    store_ = static_cast<uint8_t *>(heap_caps_malloc(store_bytes_, caps));
    if (!store_) return false;
  }
  if (!open(rate_hz, width, kRingBytes, num, den)) return false;
  parlio_rx_event_callbacks_t cb = {};
  cb.on_partial_receive = partialReceive;
  parlio_rx_soft_delimiter_config_t d = {};
  d.sample_edge = PARLIO_SAMPLE_EDGE_POS;
  d.bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB;
  d.eof_data_len = kSegmentBytes;
  if (parlio_rx_unit_register_event_callbacks(unit_, &cb, this) != ESP_OK ||
      parlio_new_rx_soft_delimiter(&d, &delimiter_) != ESP_OK || parlio_rx_unit_enable(unit_, true) != ESP_OK)
    return false;
  actual_samples = samples;
  actual_segments = segment_count_;
  return true;
}

bool LogicCapture::openTriggered(uint32_t rate_hz, uint8_t width, uint32_t bytes, uint32_t &num, uint32_t &den) {
  if (!ring_) ring_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, kRingBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  queue_ = xQueueCreate(128, sizeof(Chunk));
  uint32_t caps = 0;
  storeBudget(caps);                         // PSRAM when there is some: the segment is filled by the CPU, not the DMA
  buffer_ = static_cast<uint8_t *>(heap_caps_malloc(bytes, caps));
  if (!ring_ || !queue_ || !buffer_) return false;
  if (!open(rate_hz, width, kRingBytes, num, den)) return false;
  parlio_rx_event_callbacks_t cb = {};
  cb.on_partial_receive = partialReceive;
  parlio_rx_soft_delimiter_config_t d = {};
  d.sample_edge = PARLIO_SAMPLE_EDGE_POS;
  d.bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB;
  d.eof_data_len = kSegmentBytes;
  return parlio_rx_unit_register_event_callbacks(unit_, &cb, this) == ESP_OK &&
         parlio_new_rx_soft_delimiter(&d, &delimiter_) == ESP_OK && parlio_rx_unit_enable(unit_, true) == ESP_OK;
}

Result LogicCapture::startRepeat(uint8_t *out, size_t capacity) {
  if (capacity < 8) return failed();
  completed_ = released_ = fill_ = queue_overflow_ = overruns_ = stage_drops_ = 0;
  carry_ = false;
  produced_ = 0;
  sent_seg_ = 0;
  sent_off_ = 0;
  captured_ = dropped_ = 0;
  gap_pending_ = paused_ = false;
  reported_ = 0;
  xQueueReset(queue_);
  harvesting_ = true;
  if (xTaskCreatePinnedToCore(harvestTask, "oep_harvest", 4096, this, 5, &task_, 0) != pdPASS) {
    harvesting_ = false;
    return failed();
  }
  parlio_receive_config_t rc = {};
  rc.delimiter = delimiter_;
  rc.flags.partial_rx_en = true;
  if (parlio_rx_unit_receive(unit_, ring_, kRingBytes, &rc) != ESP_OK) { stopRepeat(); state_ = kStateError; return failed(); }
  start_ns_ = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
  if (parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, true) != ESP_OK) { stopRepeat(); state_ = kStateError; return failed(); }
  state_ = kStateCapturing;
  ++generation_;
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
    if (!openTriggered(rate_hz_, width_, bytes_, num, den)) { close(); state_ = kStateError; return false; }
    triggered_ = true;
  }
  follow_ = true;
  uint8_t out[4];
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
  putU32(out + 12, samples_);
  putU64(out + 16, triggered_ ? start_ns_ + nsOf(seg_first_sample_) : start_ns_);
  putU32(out + 24, kStartUncertaintyNs);
  putU32(out + 28, triggered_ ? trigger_index_ : 0xFFFFFFFFu);   // immediate: no trigger inside
  out[32] = triggered_ && trig_overrun_ ? cap::kSegmentFlagGap : 0;   // bit0: part of it was lost
  putU32(out + 33, generation_);
  return kInfoBytes;
}

void LogicCapture::pollTriggered() {
  if (trig_phase_ >= 1 && state_ == kStateWaiting) state_ = kStateCapturing;
  if (trig_phase_ >= 1 && !reported_trigger_) {
    reported_trigger_ = true;
    if (subscribed_) {   // serial(u32) trigger_index(u32) trigger_ns(u64)
      uint8_t e[16];
      putU32(e, 0);
      putU32(e + 4, trigger_index_);
      putU64(e + 8, start_ns_ + nsOf(seg_first_sample_ + trigger_index_));
      endpoint_.event(*this, cap::kEventTriggered, e, sizeof e);
    }
  }
  if (trig_phase_ != 2 || state_ != kStateCapturing) return;
  stopRepeat();
  state_ = kStateDone;
  if (subscribed_) {
    uint8_t seg[kInfoBytes];
    endpoint_.event(*this, kEventSegment, seg, segmentInfo(seg));
    const uint8_t stopped[2] = {kStoppedComplete, 0};
    endpoint_.event(*this, kEventStopped, stopped, sizeof stopped);
  }
}

void LogicCapture::poll() {
  if (mode_ == 2 || mode_ == 3) {
    if (mode_ == 2 && (state_ == kStateCapturing || state_ == kStatePaused)) state_ = paused_ ? kStatePaused : kStateCapturing;
    while (reported_ < completed_) {
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
  if (subscribed_) {
    uint8_t seg[kInfoBytes];
    endpoint_.event(*this, kEventSegment, seg, segmentInfo(seg));
    const uint8_t stopped[2] = {kStoppedComplete, 0};
    endpoint_.event(*this, kEventStopped, stopped, sizeof stopped);
  }
}

// Streaming: bytes of segment `serial` that may be sent (a finished segment's length; the one being filled so far).
uint32_t LogicCapture::segmentLength(uint32_t serial) const {
  const uint32_t done = completed_;
  if (serial < done) return infos_[serial % kInfos].samples * width_ / 8;
  if (serial == done) return fill_;
  return 0;
}

// Streaming: the stored segment holding stream `position` (not yet reused), and the offset in it.
bool LogicCapture::findSegment(uint64_t position, uint32_t &serial, uint32_t &offset) const {
  const uint32_t done = completed_;
  const uint32_t oldest = done >= segment_count_ ? done - segment_count_ + 1 : 0;
  for (uint32_t k = done + 1; k-- > oldest;) {
    const uint32_t length = segmentLength(k);
    if (k == done && fill_ == 0) continue;   // not started
    const uint64_t begin = infos_[k % kInfos].position;
    if (position >= begin && position - begin < length) { serial = k; offset = static_cast<uint32_t>(position - begin); return true; }
  }
  return false;
}

size_t LogicCapture::pending() {
  if (mode_ != 3 || direct_ || !(state_ == kStateCapturing || state_ == kStateConfigured)) return 0;
  const uint32_t done = completed_, fill = fill_;
  uint64_t n = 0;
  for (uint32_t k = sent_seg_; k < done; ++k) n += infos_[k % kInfos].samples * width_ / 8;
  n += fill;
  return n > sent_off_ ? static_cast<size_t>(n - sent_off_) : 0;
}

// Streaming push: the next bytes in stream order, from the segment being sent; a segment fully sent is released.
// payload: position(u64) len(u16) data, then TLV 0x01 generation (core §11.2, oep-if-capture §3.4)
size_t LogicCapture::pull(uint8_t *out, size_t capacity) {
  constexpr size_t kHead = 10, kTail = 6;
  if (mode_ != 3 || direct_ || !store_ || capacity <= kHead + kTail) return 0;
  const uint32_t serial = sent_seg_;
  const bool finished = serial < completed_;
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
  memcpy(out + kHead, store_ + static_cast<size_t>(serial % segment_count_) * segment_bytes_ + sent_off_, n);
  out[kHead + n] = cap::kTlvDataGeneration;
  out[kHead + n + 1] = 4;
  putU32(out + kHead + n + 2, generation_);
  sent_off_ += n;
  if (finished && sent_off_ >= length) { sent_off_ = 0; sent_seg_ = serial + 1; released_ = serial + 1; }
  return kHead + n + kTail;
}

Result LogicCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  if (bound() && !group_op_ && (op == kOpConfigure || op == kOpStart || op == kOpStop || op == kOpForce))
    return boundInGroup(*this, out, capacity);   // bound in a capture-group: the group starts and stops it (cause 4)
  if (op == kOpConfigure || op == kOpQuery) return configure(p, n, out, capacity, op == kOpQuery);
  // Every other request: a fixed part (start / stop / force / status: none; read: generation position max = 16;
  // segments: 4; release: generation serial = 8), then TLVs, none of which these ops read.
  const size_t fixed = op == kOpRead ? 16 : op == kOpSegments ? 4 : op == kOpRelease ? 8 : 0;
  Tail tail;
  if (op >= kOpStart && op <= kOpRelease) {
    const Result parsed = plainTail(tail, p, n, fixed, out, capacity);
    if (refused(parsed)) return parsed;
  }
  switch (op) {
    case kOpStart: {   // -> blocking_ms(u32) generation(u32) [TLV]
      if (state_ != kStateConfigured && state_ != kStateDone && state_ != kStateError) return wrongState(out, capacity);
      if (mode_ == 3 && !subscribed_) return wrongState(out, capacity);   // streaming pushes: subscribe first
      if (mode_ == 2 || mode_ == 3) return tail.finish(startRepeat(out, capacity), out, capacity);
      follow_ = false;
      if (triggered_) return tail.finish(startTriggered(out, capacity), out, capacity);
      if (capacity < 8) return failed();
      if (state_ == kStateDone) parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
      done_ = false;
      memset(buffer_, 0, bytes_);
      esp_cache_msync(buffer_, bytes_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
      parlio_receive_config_t rc = {};
      rc.delimiter = delimiter_;
      if (parlio_rx_unit_receive(unit_, buffer_, bytes_, &rc) != ESP_OK) { state_ = kStateError; return failed(); }
      start_ns_ = static_cast<uint64_t>(esp_timer_get_time()) * 1000u;
      if (parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, true) != ESP_OK) { state_ = kStateError; return failed(); }
      state_ = kStateCapturing;
      ++generation_;
      putU32(out, 0);              // blocking_ms: DMA, the probe keeps answering
      putU32(out + 4, generation_);
      return tail.finish(completed(8), out, capacity);
    }
    case kOpStop: {
      const uint8_t stopped[2] = {kStoppedHost, 0};
      if (triggered_ && (state_ == kStateWaiting || state_ == kStateCapturing)) {
        stopRepeat();
        state_ = kStateConfigured;
        if (subscribed_) endpoint_.event(*this, kEventStopped, stopped, sizeof stopped);
        return tail.finish(completed(), out, capacity);
      }
      if ((mode_ == 2 || mode_ == 3) && (state_ == kStateCapturing || state_ == kStatePaused)) {
        stopRepeat();
        poll();
        state_ = kStateConfigured;
        if (subscribed_) endpoint_.event(*this, kEventStopped, stopped, sizeof stopped);
        return tail.finish(completed(), out, capacity);
      }
      if (state_ == kStateCapturing) {
        parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
        state_ = kStateConfigured;
        if (subscribed_) endpoint_.event(*this, kEventStopped, stopped, sizeof stopped);
      }
      return tail.finish(completed(), out, capacity);
    }
    case kOpStatus: {   // -> state(u8) serial_done(u32) write_pos(u64) flags(u8) generation(u32) [TLV error]
      if (capacity < 21) return failed();
      poll();
      out[0] = state_;
      if (mode_ == 2 || mode_ == 3) {
        putU32(out + 1, completed_);
        putU64(out + 5, captured_);   // the next byte to write, the dropped ones counted (oep-if-capture §3.2)
        // flags (oep-if-capture §3.2): bit0 the probe dropped data - the chunk queue full, the DMA ring overrun, no free
        // stage (the PARLIO keeps its clock: bit1 slipped never)
        out[13] = (queue_overflow_ || overruns_ || stage_drops_) ? cap::kStatusFlagDropped : 0;
      } else if (triggered_) {
        putU32(out + 1, state_ == kStateDone ? 1 : 0);
        putU64(out + 5, filled_);
        out[13] = (queue_overflow_ || overruns_ || trig_overrun_) ? cap::kStatusFlagDropped : 0;
      } else {
        putU32(out + 1, state_ == kStateDone ? 1 : 0);                // segments done
        putU64(out + 5, state_ == kStateDone ? bytes_ : 0);           // write position
        out[13] = 0;
      }
      putU32(out + 14, generation_);
      size_t used = 18;
      if (state_ == kStateError) {   // why (TLV 0x01 error): the PARLIO or its DMA would not start / run
        out[18] = cap::kTlvStatusAnswerError;
        out[19] = 1;
        out[20] = cap::kErrorPeripheral;
        used = 21;
      }
      return tail.finish(completed(used), out, capacity);
    }
    case kOpRead: {   // generation(u32) position(u64) max(u32) [TLV]  ->  position(u64) flags(u8: bit0 more, bit1 gap) len(u32) data [TLV]
      constexpr size_t kHead = 13;
      if (getU32(p) != generation_) return wrongState(out, capacity);   // another generation (cause 6)
      const size_t reserve = tail.anyIgnored() ? 2 + Tail::kMaxIgnored : 0;
      if (capacity < kHead + reserve) return failed();
      poll();
      uint64_t position = getU64(p + 4);
      uint32_t max = getU32(p + 12);
      size_t room = capacity - kHead - reserve;
      if (room > max_read_) room = max_read_;
      if (max > room) max = static_cast<uint32_t>(room);
      auto answer = [&](uint64_t pos, uint8_t flags, const uint8_t *data, uint32_t count) {
        putU64(out, pos);
        out[8] = flags;
        putU32(out + 9, count);
        if (count) memcpy(out + kHead, data, count);
        return tail.finish(completed(kHead + count), out, capacity);
      };
      if (mode_ == 3) {   // streaming: what is still in the store, by stream position
        uint32_t serial = 0, offset = 0;
        uint8_t flags = 0;
        // zero-copy streaming keeps nothing to read back; gone (reused) or not captured yet: nothing, gap flag
        if (direct_ || !findSegment(position, serial, offset)) return answer(position, reg::common::kReadFlagsGap, nullptr, 0);
        uint32_t count = segmentLength(serial) - offset;
        if (count > max) { count = max; flags |= reg::common::kReadFlagsMore; }
        return answer(position, flags, store_ + static_cast<size_t>(serial % segment_count_) * segment_bytes_ + offset, count);
      }
      if (mode_ == 2) {   // completed segments the host has not released (positions are u64: no wrap to handle)
        const uint64_t first = static_cast<uint64_t>(released_) * segment_bytes_;
        const uint64_t end = static_cast<uint64_t>(completed_) * segment_bytes_;
        uint8_t flags = 0;
        if (position < first) { position = first; flags |= reg::common::kReadFlagsGap; }   // already released: gap
        if (position > end) position = end;
        const uint64_t ahead = position - first;                   // bytes past the first unreleased segment
        const uint32_t serial = released_ + static_cast<uint32_t>(ahead / segment_bytes_);
        const uint32_t offset = static_cast<uint32_t>(ahead % segment_bytes_);
        uint64_t left = end - position;
        uint32_t count = left < segment_bytes_ - offset ? static_cast<uint32_t>(left) : segment_bytes_ - offset;   // one segment per answer
        if (count > max) { count = max; flags |= reg::common::kReadFlagsMore; }
        if (left > count) flags |= reg::common::kReadFlagsMore;
        return answer(position, flags, store_ + (serial % segment_count_) * segment_bytes_ + offset, count);
      }
      const uint64_t have = state_ == kStateDone ? bytes_ : 0;
      if (position > have) position = have;
      uint32_t count = static_cast<uint32_t>(have - position);
      uint8_t flags = 0;
      if (count > max) { count = max; flags |= reg::common::kReadFlagsMore; }
      return answer(position, flags, buffer_ + position, count);
    }
    case kOpSegments: {   // from_serial(u32) [TLV]  ->  more(u8) count(u8) count x (len(u8) segment info) [TLV]
      if (capacity < 3 + kInfoBytes) return failed();
      poll();
      const size_t room = tail.anyIgnored() && capacity > 3 + Tail::kMaxIgnored ? capacity - 2 - Tail::kMaxIgnored : capacity;
      if (mode_ == 2 || mode_ == 3) {
        uint32_t from = getU32(p);
        const uint32_t oldest = completed_ > kInfos ? completed_ - kInfos : 0;
        if (from < oldest) from = oldest;
        uint8_t count = 0;
        size_t used = 2;
        uint32_t k = from;
        for (; k < completed_ && used + 1 + kInfoBytes <= room && count < 255; ++k, ++count) {
          out[used] = static_cast<uint8_t>(infoBytes(infos_[k % kInfos], out + used + 1));   // len(u8) first (core §2.3)
          used += 1u + out[used];
        }
        out[0] = k < completed_ ? 1 : 0;   // more
        out[1] = count;
        return tail.finish(completed(used), out, capacity);
      }
      const bool one = state_ == kStateDone && getU32(p) == 0;
      out[0] = 0;
      out[1] = one ? 1 : 0;
      if (one) out[2] = static_cast<uint8_t>(segmentInfo(out + 3));   // len(u8) then the info (core §2.3)
      return tail.finish(completed(one ? 3u + out[2] : 2u), out, capacity);
    }
    case kOpRelease:   // generation(u32) serial(u32) [TLV]: that segment and the ones before it may be reused (repeat)
      if (getU32(p) != generation_) return wrongState(out, capacity);
      if (mode_ != 2) return tail.finish(completed(), out, capacity);   // nothing to release: success
      if (getU32(p + 4) + 1 > released_ && getU32(p + 4) < completed_) released_ = getU32(p + 4) + 1;
      return tail.finish(completed(), out, capacity);
    case kOpForce:   // waiting for the trigger: start now (the next chunk); otherwise nothing to do
      if (state_ == kStateWaiting && trig_phase_ == 0) force_ = true;
      return tail.finish(completed(), out, capacity);
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
