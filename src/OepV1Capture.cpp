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

// configure TLVs (oep-spec logic-capture §5.3); bit 7 of the tag = critical
enum : uint8_t { kTagMode = 0x40, kTagQuery = 0x41, kTagRate = 0x42, kTagSamples = 0x43, kTagSegments = 0x44,
                 kTagTrigger = 0x45, kTagPretrigger = 0x46 };
enum : uint8_t { kTagActualRate = 0x50, kTagLayout = 0x51, kTagActualSamples = 0x52, kTagActualSegments = 0x53,
                 kTagTiming = 0x54, kTagBlocking = 0x56, kTagIgnored = 0x57 };
enum : uint8_t { kEventSegment = 0x01, kEventStopped = 0x02 };
constexpr uint8_t kCritical = 0x80;

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
  w.put(0x40, mode, sizeof mode);
  mode[0] = 2;                                     // repeat, in the background too
  uint32_t caps = 0;
  const size_t budget = storeBudget(caps);
  putU32(mode + 2, kSegmentMaxRepeat * 8);
  putU32(mode + 6, budget / kSegmentMin);
  w.put(0x40, mode, sizeof mode);
  mode[0] = 3;                                     // streaming: segments of 64 KiB, pushed (needs a subscription)
  putU32(mode + 2, kSegmentBytes * 8);
  putU32(mode + 6, kInfos);
  w.put(0x40, mode, sizeof mode);
  uint8_t range[9];
  putU32(range, kMinHz);
  putU32(range + 4, kSourceHz);
  range[8] = 1;                                    // any value in range (fractional divider)
  w.put(0x41, range, sizeof range);
  const uint32_t list[] = {1000000, 2000000, 5000000, 10000000, 20000000, 40000000, 80000000, 160000000};
  uint8_t l[sizeof list];
  for (size_t i = 0; i < 8; ++i) putU32(l + 4 * i, list[i]);
  w.put(0x42, l, sizeof l);
  uint8_t lim[6] = {1, 8};                         // wch-protocols E033: 8 lines to 100 MHz, 16 lines to 48 MHz
  putU32(lim + 2, 100000000);
  w.put(0x43, lim, sizeof lim);
  lim[1] = 16;
  putU32(lim + 2, 48000000);
  w.put(0x43, lim, sizeof lim);
  uint8_t ch[2] = {kMaxChannels, 0b11111};         // w in {1, 2, 4, 8, 16}
  w.put(0x44, ch, sizeof ch);
  uint8_t trig[5] = {0b1};                         // immediate only; no pretrigger
  w.put(0x45, trig, sizeof trig);
  w.u32(0x47, static_cast<uint32_t>(max_read_));
  w.u16(0x48, kInfos);                             // segment infos kept
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
  if (ring_) { heap_caps_free(ring_); ring_ = nullptr; }
  if (store_) { heap_caps_free(store_); store_ = nullptr; }
  store_bytes_ = 0;
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
  if (channels_ == 0) return rejected(kRejectUnavailable);   // plan first
  uint8_t mode = 1, ignored[16], ignored_count = 0;
  uint32_t rate = 1000000, samples = 0, segments = 0;
  for (size_t at = 0; at + 2 <= n;) {
    const uint8_t raw = p[at], tag = raw & ~kCritical, len = p[at + 1];
    const uint8_t *v = p + at + 2;
    if (at + 2 + len > n) return rejected(kRejectMalformed);
    bool understood = true;
    switch (tag) {
      case kTagMode: if (len != 1) return rejected(kRejectMalformed); mode = v[0]; break;
      case kTagRate: if (len != 4) return rejected(kRejectMalformed); rate = getU32(v); break;
      case kTagSamples: if (len != 4) return rejected(kRejectMalformed); samples = getU32(v); break;
      case kTagSegments: if (len != 4) return rejected(kRejectMalformed); segments = getU32(v); break;
      case kTagTrigger:
        if (len != 4) return rejected(kRejectMalformed);
        if (v[0] != 0) understood = false;                           // immediate only
        break;
      case kTagPretrigger: if (len != 4 || getU32(v)) understood = false; break;
      default: understood = false;
    }
    if (!understood) {
      if (raw & kCritical) {                                         // cannot do what the host requires
        if (capacity < 1) return rejected(kRejectUnavailable);
        out[0] = tag;
        return {kResolutionRejected, kRejectUnavailable, 1};
      }
      if (ignored_count < sizeof ignored) ignored[ignored_count++] = tag;
    }
    at += 2 + len;
  }
  if (mode != 1 && mode != 2 && mode != 3) {
    if (capacity < 1) return rejected(kRejectUnavailable);
    out[0] = kTagMode;
    return {kResolutionRejected, kRejectUnavailable, 1};
  }
  if (rate < kMinHz || rate > kSourceHz) {   // the driver would silently run at 160 MHz instead
    if (capacity < 1) return rejected(kRejectUnavailable);
    out[0] = kTagRate;
    return {kResolutionRejected, kRejectUnavailable, 1};
  }
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
  if (ignored_count) w.put(kTagIgnored, ignored, ignored_count);
  return w.ok() ? completed(w.length()) : failed();
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
  uint32_t caps = 0;
  storeBudget(caps);
  store_bytes_ = static_cast<size_t>(segment_bytes_) * segment_count_;
  store_ = static_cast<uint8_t *>(heap_caps_malloc(store_bytes_, caps));
  ring_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, kRingBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  queue_ = xQueueCreate(128, sizeof(Chunk));
  if (!store_ || !ring_ || !queue_) return false;
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
  completed_ = released_ = fill_ = queue_overflow_ = overruns_ = 0;
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
      if (subscribed_) {
        uint8_t seg[21];
        endpoint_.event(*this, kEventSegment, seg, infoBytes(infos_[reported_ % kInfos], seg));
      }
      ++reported_;
    }
    if (mode_ == 3) return;   // streaming keeps capturing; dropped bytes show as a position jump
    if (paused_ && !paused_reported_) {
      paused_reported_ = true;
      if (subscribed_) { const uint8_t reason = 2; endpoint_.event(*this, kEventStopped, &reason, 1); }
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
    const uint8_t reason = 0;      // complete
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
  if (mode_ != 3 || !(state_ == kStateCapturing || state_ == kStateConfigured)) return 0;
  const uint32_t done = completed_, fill = fill_;
  uint64_t n = 0;
  for (uint32_t k = sent_seg_; k < done; ++k) n += infos_[k % kInfos].samples * width_ / 8;
  n += fill;
  return n > sent_off_ ? static_cast<size_t>(n - sent_off_) : 0;
}

// Streaming push: the next bytes in stream order, from the segment being sent; a segment fully sent is released.
size_t LogicCapture::pull(uint32_t &position, uint8_t *out, size_t capacity) {
  if (mode_ != 3 || !store_) return 0;
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
  switch (op) {
    case kOpConfigure: return configure(p, n, out, capacity, false);
    case kOpQuery: return configure(p, n, out, capacity, true);
    case kOpStart: {
      if (state_ != kStateConfigured && state_ != kStateDone) return rejected(kRejectUnavailable);
      if (mode_ == 3 && !subscribed_) return rejected(kRejectUnavailable);   // streaming pushes: subscribe first
      if (mode_ == 2 || mode_ == 3) return startRepeat(out, capacity);
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
      return completed(4);
    }
    case kOpStop:
      if ((mode_ == 2 || mode_ == 3) && (state_ == kStateCapturing || state_ == kStatePaused)) {
        stopRepeat();
        poll();
        state_ = kStateConfigured;
        if (subscribed_) { const uint8_t reason = 1; endpoint_.event(*this, kEventStopped, &reason, 1); }
        return completed();
      }
      if (state_ == kStateCapturing) {
        parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
        state_ = kStateConfigured;
        if (subscribed_) { const uint8_t reason = 1; endpoint_.event(*this, kEventStopped, &reason, 1); }
      }
      return completed();
    case kOpStatus: {
      if (capacity < 10) return failed();
      poll();
      out[0] = state_;
      if (mode_ == 2 || mode_ == 3) {
        putU32(out + 1, completed_);
        putU32(out + 5, mode_ == 3 ? static_cast<uint32_t>(captured_) : completed_ * segment_bytes_ + fill_);
        out[9] = (queue_overflow_ ? 1 : 0) | (overruns_ ? 2 : 0);     // bit0 chunk queue full, bit1 DMA ring overrun
      } else {
        putU32(out + 1, state_ == kStateDone ? 1 : 0);                // segments done
        putU32(out + 5, state_ == kStateDone ? bytes_ : 0);           // write position
        out[9] = 0;
      }
      return completed(10);
    }
    case kOpRead: {
      if (n != 8 || capacity < 5) return rejected(kRejectMalformed);
      poll();
      uint32_t position = getU32(p), max = getU32(p + 4);
      if (mode_ == 3) {   // streaming: what is still in the store, by stream position
        uint32_t serial = 0, offset = 0;
        uint8_t flags = 0;
        if (!findSegment(position, serial, offset)) {   // gone (reused) or not captured yet: nothing, gap flag
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
      if (mode_ == 2) {   // completed segments the host has not released
        const uint32_t first = released_ * segment_bytes_, end = completed_ * segment_bytes_;
        uint8_t flags = 0;
        if (static_cast<int32_t>(position - first) < 0) { position = first; flags |= 2; }   // already released: gap
        if (static_cast<int32_t>(position - end) > 0) position = end;
        uint32_t count = end - position;
        const uint32_t in_segment = segment_bytes_ - position % segment_bytes_;
        if (count > in_segment) count = in_segment;                    // one segment per answer (contiguous)
        size_t room = capacity - 5;
        if (room > max_read_) room = max_read_;
        if (max > room) max = static_cast<uint32_t>(room);
        if (count > max) { count = max; flags |= 1; }
        if (end - position > count) flags |= 1;
        putU32(out, position);
        out[4] = flags;
        const size_t slot = (position / segment_bytes_) % segment_count_;
        if (count) memcpy(out + 5, store_ + slot * segment_bytes_ + position % segment_bytes_, count);
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
    case kOpSegments: {
      if (n != 4 || capacity < 1 + 21) return rejected(kRejectMalformed);
      poll();
      if (mode_ == 2 || mode_ == 3) {
        uint32_t from = getU32(p);
        const uint32_t oldest = completed_ > kInfos ? completed_ - kInfos : 0;
        if (from < oldest) from = oldest;
        uint8_t count = 0;
        size_t used = 1;
        for (uint32_t k = from; k < completed_ && used + 21 <= capacity && count < 255; ++k, ++count)
          used += infoBytes(infos_[k % kInfos], out + used);
        out[0] = count;
        return completed(used);
      }
      const bool one = state_ == kStateDone && getU32(p) == 0;
      out[0] = one ? 1 : 0;
      return completed(1 + (one ? segmentInfo(out + 1) : 0));
    }
    case kOpRelease:
      if (mode_ != 2 || n != 4) return rejected(kRejectUnavailable);
      if (getU32(p) + 1 > released_ && getU32(p) < completed_) released_ = getU32(p) + 1;
      return completed();
    default:
      return rejected(kRejectUnknownOperation);   // force: not in this implementation (immediate trigger only)
  }
}

}  // namespace v1
}  // namespace oep

#endif
