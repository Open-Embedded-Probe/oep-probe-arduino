#include "OepFixtureCapture.h"

#include <string.h>

#include "OepTlv.h"

#if OEP_CAPTURE_PARLIO
#include <esp_cache.h>
#include <esp_heap_caps.h>
#endif

namespace oep {

uint8_t FixtureCapture::planCheck(const RoleAssignment *roles, size_t count) {
  if (!count || count > kMaxLines || line_count_) return count ? OEP_V0_REJECT_UNAVAILABLE : OEP_V0_REJECT_MALFORMED_PAYLOAD;
  uint8_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role >= kMaxLines || (seen >> roles[i].role) & 1) return OEP_V0_REJECT_MALFORMED_PAYLOAD;
    seen |= 1u << roles[i].role;
    // Observer: the channel must exist on this probe, but it may be owned by
    // another function (we only sample it).
    if (!pins_.allowed(roles[i].channel)) return OEP_V0_REJECT_UNAVAILABLE;
  }
  if (seen != (1u << count) - 1) return OEP_V0_REJECT_MALFORMED_PAYLOAD;  // lines 0..count-1
  return 0;
}

bool FixtureCapture::planApply(const RoleAssignment *roles, size_t count) {
  for (size_t i = 0; i < count; ++i) lines_[roles[i].role] = roles[i].channel;
  line_count_ = static_cast<uint8_t>(count);
  return true;
}

void FixtureCapture::planRelease() {
  teardown();
  for (int &l : lines_) l = -1;
  line_count_ = 0;
}

size_t FixtureCapture::describe(uint8_t first, uint8_t *out, size_t capacity) {
  size_t used = 0, index = 0;
  if (index++ >= first) { used = tlvPutU32(out, capacity, used, OEP_V0_TLV_CORE_MAX_CLOCK_HZ, kMaxSampleRateHz); if (!used) return 0; }
  if (index++ >= first) { const size_t n = tlvPutU32(out, capacity, used, OEP_V0_TLV_CORE_MIN_CLOCK_HZ, kMinSampleRateHz); if (!n) return used; used = n; }
  if (index++ >= first) { const size_t n = tlvPutU16(out, capacity, used, OEP_V0_TLV_CORE_MAX_LENGTH, kBufferBytes); if (!n) return used; used = n; }
  for (uint8_t c = 0; c < PinTable::kChannels; ++c) {
    if (!pins_.allowed(c)) continue;
    if (index++ < first) continue;
    const size_t n = tlvPutU16(out, capacity, used, OEP_V0_TLV_CORE_CHANNEL_CANDIDATE, c);
    if (!n) break;
    used = n;
  }
  return used;
}

uint8_t FixtureCapture::sampleAt(uint32_t index) const {
  const uint32_t bit = index * width_;
  return (buffer_[bit / 8] >> (bit % 8)) & ((1u << width_) - 1);
}

Result FixtureCapture::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (operation) {
    case OEP_V0_FIXTURE_CAPTURE_OP_CONFIGURE: {
      struct oep_v0_fixture_capture_configure_request request;
      if (!oep_v0_fixture_capture_configure_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!line_count_) return rejected(OEP_V0_REJECT_UNAVAILABLE);  // needs a plan first
      if (request.sample_rate_hz < kMinSampleRateHz || request.sample_rate_hz > kMaxSampleRateHz || !request.samples) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!setup(request.sample_rate_hz, request.samples)) return failed();
      struct oep_v0_fixture_capture_configure_result result = {sample_rate_, samples_, line_count_};
      return completed(oep_v0_fixture_capture_configure_result_pack(&result, out, capacity));
    }
    case OEP_V0_FIXTURE_CAPTURE_OP_ARM: {
      struct oep_v0_fixture_capture_arm_request request;
      if (!oep_v0_fixture_capture_arm_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!configured_) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      return arm() ? completed() : failed();
    }
    case OEP_V0_FIXTURE_CAPTURE_OP_STATUS: {
      struct oep_v0_fixture_capture_status_request request;
      if (!oep_v0_fixture_capture_status_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      const bool complete = armed_ && done_;
      struct oep_v0_fixture_capture_status_result result = {
          static_cast<uint8_t>((configured_ ? 1 : 0) | (armed_ && !done_ ? 2 : 0) | (complete ? 4 : 0) | (error_ ? 8 : 0)),
          complete ? samples_ : 0};
      return completed(oep_v0_fixture_capture_status_result_pack(&result, out, capacity));
    }
    case OEP_V0_FIXTURE_CAPTURE_OP_READ: {
      struct oep_v0_fixture_capture_read_request request;
      if (!oep_v0_fixture_capture_read_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!(armed_ && done_)) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      if (request.offset > samples_) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      size_t n = request.maximum;
      if (n > samples_ - request.offset) n = samples_ - request.offset;
      if (n > capacity) n = capacity;
      // The result payload is the sample bytes themselves (one byte per sample);
      // unpack the PARLIO bit fields straight into the result buffer.
      for (size_t i = 0; i < n; ++i) out[i] = sampleAt(request.offset + i);
      return completed(n);
    }
    default:
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

#if OEP_CAPTURE_PARLIO

bool FixtureCapture::receiveDone(parlio_rx_unit_handle_t, const parlio_rx_event_data_t *, void *context) {
  static_cast<FixtureCapture *>(context)->done_ = true;  // ISR: flag only
  return false;
}

bool FixtureCapture::setup(uint32_t sample_rate_hz, uint32_t samples) {
  teardown();
  width_ = line_count_ <= 1 ? 1 : line_count_ <= 2 ? 2 : line_count_ <= 4 ? 4 : 8;
  const uint32_t max_samples = static_cast<uint32_t>(kBufferBytes) * 8 / width_;
  if (samples > max_samples) samples = max_samples;
  bytes_ = (static_cast<size_t>(samples) * width_ + 7) / 8;
  bytes_ = (bytes_ + 127) & ~static_cast<size_t>(127);  // cache line multiple for msync
  if (bytes_ > kBufferBytes) bytes_ = kBufferBytes;
  samples_ = static_cast<uint32_t>(bytes_ * 8 / width_);
  buffer_ = static_cast<uint8_t *>(heap_caps_aligned_alloc(128, bytes_, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  if (!buffer_) return false;
  parlio_rx_unit_config_t unit = {};
  unit.trans_queue_depth = 1;
  unit.max_recv_size = bytes_;
  unit.data_width = width_;
  unit.clk_src = PARLIO_CLK_SRC_DEFAULT;
  unit.exp_clk_freq_hz = sample_rate_hz;
  unit.clk_in_gpio_num = GPIO_NUM_NC;
  unit.clk_out_gpio_num = GPIO_NUM_NC;
  unit.valid_gpio_num = GPIO_NUM_NC;
  for (size_t lane = 0; lane < PARLIO_RX_UNIT_MAX_DATA_WIDTH; ++lane)
    unit.data_gpio_nums[lane] = lane < line_count_ ? static_cast<gpio_num_t>(lines_[lane]) : GPIO_NUM_NC;
  unit.flags.io_loop_back = false;
  if (parlio_new_rx_unit(&unit, &unit_) != ESP_OK) { teardown(); return false; }
  parlio_rx_event_callbacks_t callbacks = {};
  callbacks.on_receive_done = receiveDone;
  if (parlio_rx_unit_register_event_callbacks(unit_, &callbacks, this) != ESP_OK) { teardown(); return false; }
  parlio_rx_soft_delimiter_config_t delimiter = {};
  delimiter.sample_edge = PARLIO_SAMPLE_EDGE_POS;
  delimiter.bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB;
  delimiter.eof_data_len = bytes_;
  if (parlio_new_rx_soft_delimiter(&delimiter, &delimiter_) != ESP_OK) { teardown(); return false; }
  if (parlio_rx_unit_enable(unit_, true) != ESP_OK) { teardown(); return false; }
  sample_rate_ = sample_rate_hz;  // the driver does not report its divider; the HIL checks the SCL period
  configured_ = true;
  armed_ = false;
  return true;
}

bool FixtureCapture::arm() {
  if (armed_) parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
  done_ = false;
  error_ = false;
  memset(buffer_, 0, bytes_);
  esp_cache_msync(buffer_, bytes_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  parlio_receive_config_t config = {};
  config.delimiter = delimiter_;
  config.flags.partial_rx_en = false;
  config.flags.indirect_mount = false;
  if (parlio_rx_unit_receive(unit_, buffer_, bytes_, &config) != ESP_OK) { error_ = true; return false; }
  if (parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, true) != ESP_OK) { error_ = true; return false; }
  armed_ = true;
  return true;
}

void FixtureCapture::teardown() {
  if (unit_) {
    if (armed_) parlio_rx_soft_delimiter_start_stop(unit_, delimiter_, false);
    parlio_rx_unit_disable(unit_);
  }
  if (delimiter_) { parlio_del_rx_delimiter(delimiter_); delimiter_ = nullptr; }
  if (unit_) { parlio_del_rx_unit(unit_); unit_ = nullptr; }
  if (buffer_) { heap_caps_free(buffer_); buffer_ = nullptr; }
  configured_ = armed_ = done_ = error_ = false;
}

#else
bool FixtureCapture::setup(uint32_t, uint32_t) { return false; }
bool FixtureCapture::arm() { return false; }
void FixtureCapture::teardown() {}
#endif

}  // namespace oep
