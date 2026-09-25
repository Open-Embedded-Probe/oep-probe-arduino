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

bool LogicCapture::receiveDone(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context) {
  static_cast<LogicCapture *>(context)->done_ = true;   // ISR: flag only
  return false;
}

size_t LogicCapture::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.u32(kTagFeatures, 0b101);                      // bit0 query, bit2 events (no force: immediate trigger only)
  uint8_t mode[10] = {1, 1};                       // one-shot, runs in the background (DMA)
  putU32(mode + 2, kSegmentBytes * 8);             // max samples at w = 1
  putU32(mode + 6, 1);                             // one segment
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
  w.u16(0x48, 1);                                  // segment ring
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
  uint32_t rate = 1000000, samples = 0;
  for (size_t at = 0; at + 2 <= n;) {
    const uint8_t raw = p[at], tag = raw & ~kCritical, len = p[at + 1];
    const uint8_t *v = p + at + 2;
    if (at + 2 + len > n) return rejected(kRejectMalformed);
    bool understood = true;
    switch (tag) {
      case kTagMode: if (len != 1) return rejected(kRejectMalformed); mode = v[0]; break;
      case kTagRate: if (len != 4) return rejected(kRejectMalformed); rate = getU32(v); break;
      case kTagSamples: if (len != 4) return rejected(kRejectMalformed); samples = getU32(v); break;
      case kTagSegments: break;                                      // one segment in one-shot
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
  if (mode != 1) { if (capacity < 1) return rejected(kRejectUnavailable); out[0] = kTagMode; return {kResolutionRejected, kRejectUnavailable, 1}; }
  if (rate < kMinHz || rate > kSourceHz) {   // the driver would silently run at 160 MHz instead
    if (capacity < 1) return rejected(kRejectUnavailable);
    out[0] = kTagRate;
    return {kResolutionRejected, kRejectUnavailable, 1};
  }
  if (state_ == kStateCapturing && !query) return rejected(kRejectBusy);
  const uint8_t width = widthFor(channels_);
  const uint32_t max_samples = static_cast<uint32_t>(kSegmentBytes * 8 / width);
  if (samples == 0 || samples > max_samples) samples = max_samples;
  uint32_t bytes = (samples * width + 7) / 8;
  bytes = (bytes + 127) & ~127u;                                     // whole cache lines for the DMA
  if (bytes > kSegmentBytes) bytes = kSegmentBytes;
  samples = bytes * 8 / width;

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
  w.u32(kTagActualSegments, 1);
  uint8_t timing[5] = {static_cast<uint8_t>(den == 1 || kSourceHz % rate == 0 ? 0 : 1)};   // 1: fractional divider
  putU32(timing + 1, den == 1 ? 0 : 7);                               // one 160 MHz period, rounded up (ns)
  w.put(kTagTiming, timing, sizeof timing);
  w.u32(kTagBlocking, 0);
  if (ignored_count) w.put(kTagIgnored, ignored, ignored_count);
  return w.ok() ? completed(w.length()) : failed();
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

Result LogicCapture::handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *out, size_t capacity) {
  switch (op) {
    case kOpConfigure: return configure(p, n, out, capacity, false);
    case kOpQuery: return configure(p, n, out, capacity, true);
    case kOpStart: {
      if (state_ != kStateConfigured && state_ != kStateDone) return rejected(kRejectUnavailable);
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
      putU32(out + 1, state_ == kStateDone ? 1 : 0);                  // segments done
      putU32(out + 5, state_ == kStateDone ? bytes_ : 0);             // write position
      out[9] = 0;
      return completed(10);
    }
    case kOpRead: {
      if (n != 8 || capacity < 5) return rejected(kRejectMalformed);
      poll();
      uint32_t position = getU32(p), max = getU32(p + 4);
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
      const bool one = state_ == kStateDone && getU32(p) == 0;
      out[0] = one ? 1 : 0;
      return completed(1 + (one ? segmentInfo(out + 1) : 0));
    }
    default:
      return rejected(kRejectUnknownOperation);   // force, release: not in this implementation (one-shot, immediate)
  }
}

}  // namespace v1
}  // namespace oep

#endif
