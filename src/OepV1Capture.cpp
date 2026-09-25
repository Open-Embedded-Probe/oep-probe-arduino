#include "OepV1Capture.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)

#include <esp_cache.h>
#include <esp_heap_caps.h>
#include <hal/hal_utils.h>
#include <soc/hp_sys_clkrst_struct.h>
#include <string.h>

#include "OepV1Endpoint.h"

namespace oep {
namespace v1 {
namespace {

namespace cap = reg::fixture_capture;
// configure / query TLVs (oep-spec logic-capture §5.3); bit 7 of the tag = critical
enum : uint8_t { kTagMode = cap::kTlvConfigureMode, kTagRate = cap::kTlvConfigureRate,
                 kTagSamples = cap::kTlvConfigureSamples, kTagSegments = cap::kTlvConfigureSegments,
                 kTagTrigger = cap::kTlvConfigureTrigger, kTagPretrigger = cap::kTlvConfigurePretrigger };
enum : uint8_t { kTagActualRate = cap::kTlvConfigureAnswerActualRate, kTagLayout = cap::kTlvConfigureAnswerLayout,
                 kTagActualSamples = cap::kTlvConfigureAnswerActualSamples,
                 kTagActualSegments = cap::kTlvConfigureAnswerActualSegments, kTagTiming = cap::kTlvConfigureAnswerTiming,
                 kTagBlocking = cap::kTlvConfigureAnswerBlockingMs };
enum : uint8_t { kEventSegment = cap::kEventSegment, kEventStopped = cap::kEventStopped };
enum : uint8_t { kStoppedComplete = cap::kStoppedReasonComplete, kStoppedHost = cap::kStoppedReasonHost,
                 kStoppedNoFreeSegment = cap::kStoppedReasonNoFreeSegment };
inline bool refused(const Result &r) { return r.resolution != kResolutionCompleted; }

uint8_t widthFor(uint8_t channels) {
  uint8_t w = 1;
  while (w < channels) w <<= 1;
  return w;
}

uint32_t gcd(uint64_t a, uint64_t b) {
  while (b) { const uint64_t t = a % b; a = b; b = t; }
  return static_cast<uint32_t>(a);
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
      info.position = mode_ == 3 ? static_cast<uint32_t>(captured_) : completed_ * segment_bytes_;
      const uint64_t first_sample = captured_ * 8 / width_;
      info.start_us = start_us_ + static_cast<uint32_t>(first_sample * rate_den_ * 1000000ull / rate_num_);
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
  stage_pos_ = static_cast<uint32_t>(captured_);
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
  if (fill > 1 && fill < stage_data_ && (kPushHead + fill) % 512 == 0 && tr && tr->queued() == 0) {
    carry_byte_ = s[kPushHead + fill - 1];
    carry_ = true;
    --fill;
  }
  uint16_t fn = 0, min_bytes = 0, max_delay = 0;
  DirectTransport *t = endpoint_.direct();
  if (fill == 0 || !t || !endpoint_.directPush(*this, fn, min_bytes, max_delay)) {
    __atomic_fetch_or(&stage_free_, 1u << index, __ATOMIC_ACQ_REL);
    return;
  }
  putU16(s, static_cast<uint16_t>(9 + fill));   // length prefix (the message: push header + data)
  s[2] = kRolePush;
  putU16(s + 3, fn);
  putU16(s + 5, endpoint_.takeSeq(fn));
  putU32(s + 7, stage_pos_);
  if (!t->queueData(s, kPushHead + fill, stageDone, this)) __atomic_fetch_or(&stage_free_, 1u << index, __ATOMIC_ACQ_REL);
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
  stage_data_ = static_cast<uint32_t>(frame - kPushHead);
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
      uint16_t fn = 0, min_bytes = 0, max_delay = 0;
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
    if (xQueueReceive(self->queue_, &chunk, pdMS_TO_TICKS(20)) == pdTRUE) self->harvest(chunk);
  }
  while (xQueueReceive(self->queue_, &chunk, 0) == pdTRUE) self->harvest(chunk);   // what arrived before the stop
  self->task_ = nullptr;
  vTaskDelete(nullptr);
}

bool LogicCapture::receiveDone(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context) {
  static_cast<LogicCapture *>(context)->done_ = true;   // ISR: flag only
  return false;
}

size_t LogicCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.u32(kTagFeatures, 0b101);                      // bit0 query, bit2 notifications (no force: immediate trigger only)
  uint8_t mode[10] = {1, 1};                       // one-shot, runs in the background (DMA)
  putU32(mode + 2, kSegmentBytes * 8);             // max samples at w = 1
  putU32(mode + 6, 1);                             // one segment
  w.put(cap::kTlvDescribeMode, mode, sizeof mode);
  mode[0] = 2;                                     // repeat, in the background too
  uint32_t caps = 0;
  const size_t budget = storeBudget(caps);
  putU32(mode + 2, kSegmentMaxRepeat * 8);
  putU32(mode + 6, budget / kSegmentMin);
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
  uint8_t l[sizeof list];
  for (size_t i = 0; i < 8; ++i) putU32(l + 4 * i, list[i]);
  w.put(cap::kTlvDescribeRateList, l, sizeof l);
  uint8_t lim[6] = {1, 8};                         // wch-protocols E033: 8 lines to 100 MHz, 16 lines to 48 MHz
  putU32(lim + 2, 100000000);
  w.put(cap::kTlvDescribeRateLimit, lim, sizeof lim);
  lim[1] = 16;
  putU32(lim + 2, 48000000);
  w.put(cap::kTlvDescribeRateLimit, lim, sizeof lim);
  uint8_t ch[2] = {kMaxChannels, 0b11111};         // w in {1, 2, 4, 8, 16}
  w.put(cap::kTlvDescribeChannels, ch, sizeof ch);
  uint8_t trig[5] = {0b1};                         // immediate only; no pretrigger
  w.put(cap::kTlvDescribeTrigger, trig, sizeof trig);
  w.u32(cap::kTlvDescribeMaxRead, static_cast<uint32_t>(max_read_));
  w.u16(cap::kTlvDescribeSegmentRing, kInfos);                            // segment infos kept
  return w.ok() ? w.length() : 0;
}

uint8_t LogicCapture::planCheck(const RoleAssignment *roles, size_t count) {
  if (count > kMaxChannels) return kRejectUnavailable;
  uint32_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role >= kMaxChannels || roles[i].channel >= 64) return kRejectUnavailable;
    if ((reserved_ >> roles[i].channel) & 1) return kRejectUnavailable;
    if (seen & (1u << roles[i].role)) return kRejectUnavailable;
    seen |= 1u << roles[i].role;
  }
  return (count == 0 || seen == (1u << count) - 1) ? 0 : kRejectUnavailable;   // roles 0..count-1, no holes
}

bool LogicCapture::planApply(const RoleAssignment *roles, size_t count) {
  close();
  for (size_t i = 0; i < count; ++i) {
    pins_[roles[i].role] = roles[i].channel;
    pinMode(roles[i].channel, INPUT);            // never driven
  }
  channels_ = static_cast<uint8_t>(count);
  state_ = kStateUnconfigured;
  return true;
}

void LogicCapture::planRelease() {
  close();
  channels_ = 0;
  state_ = kStateUnconfigured;
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
  const uint32_t g = gcd(top, bottom);
  num = static_cast<uint32_t>(top / g);
  den = static_cast<uint32_t>(bottom / g);
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
  uint8_t len = 0;
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
  if (const uint8_t *v = tail.find(kTagTrigger, len, &critical)) {   // type(u8) role(u8) value(u16): immediate only
    if (len != 4) return rejected(kRejectMalformed);
    if (v[0] != cap::kTriggerImmediate) {
      const Result r = tail.refuse(kTagTrigger, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  if (const uint8_t *v = tail.find(kTagPretrigger, len, &critical)) {   // no pretrigger
    if (len != 4) return rejected(kRejectMalformed);
    if (getU32(v)) {
      const Result r = tail.refuse(kTagPretrigger, critical, out, capacity);
      if (refused(r)) return r;
    }
  }
  if (channels_ == 0) return rejected(kRejectUnavailable);   // plan first
  if ((state_ == kStateCapturing || state_ == kStatePaused) && !query) return rejected(kRejectBusy);
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
    uint32_t seg = samples ? (samples * width + 7) / 8 : 65536;
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
    const uint32_t g = gcd(top, bottom);
    num = static_cast<uint32_t>(top / g);
    den = static_cast<uint32_t>(bottom / g);
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
  } else {
    close();
    if (!open(rate, width, bytes, num, den)) return failed();
    buffer_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
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

Result LogicCapture::startRepeat(uint8_t *out, size_t capacity) {
  if (capacity < 4) return failed();
  completed_ = released_ = fill_ = queue_overflow_ = overruns_ = stage_drops_ = 0;
  carry_ = false;
  produced_ = 0;
  sent_seg_ = 0;
  sent_off_ = 0;
  captured_ = dropped_ = 0;
  gap_pending_ = paused_ = false;
  reported_ = 0;
  paused_reported_ = false;
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
  start_us_ = micros();
  if (parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, true) != ESP_OK) { stopRepeat(); state_ = kStateError; return failed(); }
  state_ = kStateCapturing;
  putU32(out, 0);
  return completed(4);
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

size_t LogicCapture::infoBytes(const Info &info, uint8_t *out) const {
  putU32(out, info.serial);
  putU32(out + 4, info.position);
  putU32(out + 8, info.samples);
  putU32(out + 12, info.start_us);
  putU32(out + 16, 0xFFFFFFFFu);
  out[20] = info.flags;
  return 21;
}

size_t LogicCapture::segmentInfo(uint8_t *out) const {
  putU32(out, 0);                  // serial
  putU32(out + 4, 0);              // position
  putU32(out + 8, samples_);
  putU32(out + 12, start_us_);
  putU32(out + 16, 0xFFFFFFFFu);   // no trigger inside (immediate start)
  out[20] = 0;
  return 21;
}

void LogicCapture::poll() {
  if (mode_ == 2 || mode_ == 3) {
    if (mode_ == 2 && (state_ == kStateCapturing || state_ == kStatePaused)) state_ = paused_ ? kStatePaused : kStateCapturing;
    while (reported_ < completed_) {
      if (subscribed_ && mode_ == 2) {   // streaming sends no segment events (its data frames carry the positions)
        uint8_t seg[21];
        endpoint_.event(*this, kEventSegment, seg, infoBytes(infos_[reported_ % kInfos], seg));
      }
      ++reported_;
    }
    if (mode_ == 3) return;   // streaming keeps capturing; dropped bytes show as a position jump
    if (paused_ && !paused_reported_) {
      paused_reported_ = true;
      if (subscribed_) { const uint8_t reason = kStoppedNoFreeSegment; endpoint_.event(*this, kEventStopped, &reason, 1); }
    }
    if (!paused_) paused_reported_ = false;
    return;
  }
  if (state_ != kStateCapturing || !done_) return;
  esp_cache_msync(buffer_, bytes_, ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_INVALIDATE);
  state_ = kStateDone;
  if (subscribed_) {
    uint8_t seg[21];
    endpoint_.event(*this, kEventSegment, seg, segmentInfo(seg));
    const uint8_t reason = kStoppedComplete;
    endpoint_.event(*this, kEventStopped, &reason, 1);
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
bool LogicCapture::findSegment(uint32_t position, uint32_t &serial, uint32_t &offset) const {
  const uint32_t done = completed_;
  const uint32_t oldest = done >= segment_count_ ? done - segment_count_ + 1 : 0;
  for (uint32_t k = done + 1; k-- > oldest;) {
    const uint32_t length = segmentLength(k);
    const uint32_t delta = position - infos_[k % kInfos].position;
    if (k == done && fill_ == 0) continue;   // not started
    if (delta < length) { serial = k; offset = delta; return true; }
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
size_t LogicCapture::pull(uint32_t &position, uint8_t *out, size_t capacity) {
  if (mode_ != 3 || direct_ || !store_) return 0;
  const uint32_t serial = sent_seg_;
  const bool finished = serial < completed_;
  const uint32_t length = segmentLength(serial);
  __atomic_thread_fence(__ATOMIC_ACQUIRE);   // the bytes below length were written before it was published
  if (length <= sent_off_) {
    if (finished) { sent_seg_ = serial + 1; released_ = serial + 1; sent_off_ = 0; }   // an empty short segment
    return 0;
  }
  size_t n = length - sent_off_;
  if (n > capacity) n = capacity;
  position = infos_[serial % kInfos].position + sent_off_;
  memcpy(out, store_ + static_cast<size_t>(serial % segment_count_) * segment_bytes_ + sent_off_, n);
  sent_off_ += n;
  if (finished && sent_off_ >= length) { sent_off_ = 0; sent_seg_ = serial + 1; released_ = serial + 1; }
  return n;
}

Result LogicCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  if (op == kOpConfigure || op == kOpQuery) return configure(p, n, out, capacity, op == kOpQuery);
  // Every other request: a fixed part (start / stop / force / status: none; read: 8; segments / release: 4), then
  // TLVs, none of which these ops read.
  const size_t fixed = op == kOpRead ? 8 : (op == kOpSegments || op == kOpRelease) ? 4 : 0;
  Tail tail;
  if (op >= kOpStart && op <= kOpRelease) {
    const Result parsed = plainTail(tail, p, n, fixed, out, capacity);
    if (refused(parsed)) return parsed;
  }
  switch (op) {
    case kOpStart: {
      if (state_ != kStateConfigured && state_ != kStateDone) return rejected(kRejectUnavailable);
      if (mode_ == 3 && !subscribed_) return rejected(kRejectUnavailable);   // streaming pushes: subscribe first
      if (mode_ == 2 || mode_ == 3) return tail.finish(startRepeat(out, capacity), out, capacity);
      if (capacity < 4) return failed();
      if (state_ == kStateDone) parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
      done_ = false;
      memset(buffer_, 0, bytes_);
      esp_cache_msync(buffer_, bytes_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
      parlio_receive_config_t rc = {};
      rc.delimiter = delimiter_;
      if (parlio_rx_unit_receive(unit_, buffer_, bytes_, &rc) != ESP_OK) { state_ = kStateError; return failed(); }
      start_us_ = micros();
      if (parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, true) != ESP_OK) { state_ = kStateError; return failed(); }
      state_ = kStateCapturing;
      putU32(out, 0);              // blocking_ms: DMA, the probe keeps answering
      return tail.finish(completed(4), out, capacity);
    }
    case kOpStop:
      if ((mode_ == 2 || mode_ == 3) && (state_ == kStateCapturing || state_ == kStatePaused)) {
        stopRepeat();
        poll();
        state_ = kStateConfigured;
        if (subscribed_) { const uint8_t reason = kStoppedHost; endpoint_.event(*this, kEventStopped, &reason, 1); }
        return tail.finish(completed(), out, capacity);
      }
      if (state_ == kStateCapturing) {
        parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
        state_ = kStateConfigured;
        if (subscribed_) { const uint8_t reason = kStoppedHost; endpoint_.event(*this, kEventStopped, &reason, 1); }
      }
      return tail.finish(completed(), out, capacity);
    case kOpStatus: {   // -> state(u8) serial_done(u32) write_pos(u32) flags(u8)
      if (capacity < 10) return failed();
      poll();
      out[0] = state_;
      if (mode_ == 2 || mode_ == 3) {
        putU32(out + 1, completed_);
        putU32(out + 5, mode_ == 3 ? static_cast<uint32_t>(captured_) : completed_ * segment_bytes_ + fill_);
        out[9] = (queue_overflow_ ? 1 : 0) | (overruns_ ? 2 : 0) | (stage_drops_ ? 4 : 0);   // bit0 chunk queue full,
                                                                     // bit1 DMA ring overrun, bit2 no free stage
      } else {
        putU32(out + 1, state_ == kStateDone ? 1 : 0);                // segments done
        putU32(out + 5, state_ == kStateDone ? bytes_ : 0);           // write position
        out[9] = 0;
      }
      return tail.finish(completed(10), out, capacity);
    }
    case kOpRead: {   // position(u32) max(u32) [TLV]  ->  position(u32) flags(u8: bit0 more, bit1 gap) data (closed tail)
      if (capacity < 5) return failed();
      poll();
      uint32_t position = getU32(p), max = getU32(p + 4);
      if (mode_ == 3) {   // streaming: what is still in the store, by stream position
        uint32_t serial = 0, offset = 0;
        uint8_t flags = 0;
        // zero-copy streaming keeps nothing to read back; gone (reused) or not captured yet: nothing, gap flag
        if (direct_ || !findSegment(position, serial, offset)) {
          putU32(out, position);
          out[4] = 2;
          return completed(5);
        }
        uint32_t count = segmentLength(serial) - offset;
        size_t room = capacity - 5;
        if (room > max_read_) room = max_read_;
        if (max > room) max = static_cast<uint32_t>(room);
        if (count > max) { count = max; flags |= 1; }
        putU32(out, position);
        out[4] = flags;
        memcpy(out + 5, store_ + static_cast<size_t>(serial % segment_count_) * segment_bytes_ + offset, count);
        return completed(5 + count);
      }
      if (mode_ == 2) {
        // Completed segments the host has not released. Positions wrap at 2^32 and a segment's length need not
        // divide that, so the segment is found by its serial: counted from the first unreleased one, whose position
        // is released_ * segment_bytes_ (mod 2^32) - the whole store is far below 2 GiB, so differences are exact.
        const uint32_t first = released_ * segment_bytes_, end = completed_ * segment_bytes_;
        uint8_t flags = 0;
        if (static_cast<int32_t>(position - first) < 0) { position = first; flags |= 2; }   // already released: gap
        if (static_cast<int32_t>(position - end) > 0) position = end;
        const uint32_t ahead = position - first;                       // bytes past the first unreleased segment
        const uint32_t serial = released_ + ahead / segment_bytes_, offset = ahead % segment_bytes_;
        uint32_t count = end - position;
        const uint32_t in_segment = segment_bytes_ - offset;
        if (count > in_segment) count = in_segment;                    // one segment per answer (contiguous)
        size_t room = capacity - 5;
        if (room > max_read_) room = max_read_;
        if (max > room) max = static_cast<uint32_t>(room);
        if (count > max) { count = max; flags |= 1; }
        if (end - position > count) flags |= 1;
        putU32(out, position);
        out[4] = flags;
        const size_t slot = serial % segment_count_;
        if (count) memcpy(out + 5, store_ + slot * segment_bytes_ + offset, count);
        return completed(5 + count);
      }
      const uint32_t have = state_ == kStateDone ? bytes_ : 0;
      if (position > have) position = have;
      uint32_t count = have - position;
      size_t room = capacity - 5;
      if (room > max_read_) room = max_read_;
      if (max > room) max = static_cast<uint32_t>(room);
      uint8_t flags = 0;
      if (count > max) { count = max; flags |= 1; }                   // more
      putU32(out, position);
      out[4] = flags;
      if (count) memcpy(out + 5, buffer_ + position, count);
      return completed(5 + count);
    }
    case kOpSegments: {   // from_serial(u32) [TLV]  ->  count(u8) segment infos
      if (capacity < 1 + 21) return failed();
      poll();
      const size_t room = tail.anyIgnored() && capacity > 2 + Tail::kMaxIgnored ? capacity - 2 - Tail::kMaxIgnored : capacity;
      if (mode_ == 2 || mode_ == 3) {
        uint32_t from = getU32(p);
        const uint32_t oldest = completed_ > kInfos ? completed_ - kInfos : 0;
        if (from < oldest) from = oldest;
        uint8_t count = 0;
        size_t used = 1;
        for (uint32_t k = from; k < completed_ && used + 21 <= room && count < 255; ++k, ++count)
          used += infoBytes(infos_[k % kInfos], out + used);
        out[0] = count;
        return tail.finish(completed(used), out, capacity);
      }
      const bool one = state_ == kStateDone && getU32(p) == 0;
      out[0] = one ? 1 : 0;
      return tail.finish(completed(1 + (one ? segmentInfo(out + 1) : 0)), out, capacity);
    }
    case kOpRelease:   // serial(u32) [TLV]: that segment and the ones before it may be reused
      if (mode_ != 2) return rejected(kRejectUnavailable);
      if (getU32(p) + 1 > released_ && getU32(p) < completed_) released_ = getU32(p) + 1;
      return tail.finish(completed(), out, capacity);
    default:
      return rejected(kRejectUnknownOperation);   // force: not in this implementation (immediate trigger only)
  }
}

}  // namespace v1
}  // namespace oep

#endif
