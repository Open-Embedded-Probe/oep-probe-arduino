#include "OepP4SpiTarget.h"

#include <string.h>

#include <initializer_list>

#include "OepTlv.h"

namespace oep {

uint8_t P4SpiTarget::planCheck(const RoleAssignment *roles, size_t count) {
  if (count != 4) return OEP_V0_REJECT_MALFORMED_PAYLOAD;
  int pin[5] = {-1, -1, -1, -1, -1};
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role < kRoleSck || roles[i].role > kRoleCs || pin[roles[i].role] >= 0) return OEP_V0_REJECT_MALFORMED_PAYLOAD;
    pin[roles[i].role] = roles[i].channel;
  }
  for (int r = kRoleSck; r <= kRoleCs; ++r) {
    for (int q = r + 1; q <= kRoleCs; ++q) if (pin[r] == pin[q]) return OEP_V0_REJECT_MALFORMED_PAYLOAD;
    if (!pins_.free(pin[r])) return OEP_V0_REJECT_UNAVAILABLE;
  }
  if (sck_ >= 0) return OEP_V0_REJECT_UNAVAILABLE;
  return 0;
}

bool P4SpiTarget::planApply(const RoleAssignment *roles, size_t count) {
  int pin[5] = {-1, -1, -1, -1, -1};
  for (size_t i = 0; i < count; ++i) pin[roles[i].role] = roles[i].channel;
  for (int r = kRoleSck; r <= kRoleCs; ++r) {
    if (!pins_.claim(pin[r], kOwnerId)) { pins_.release(kOwnerId); return false; }
  }
  sck_ = pin[kRoleSck]; mosi_ = pin[kRoleMosi]; miso_ = pin[kRoleMiso]; cs_ = pin[kRoleCs];
  return true;
}

void P4SpiTarget::planRelease() {
  stop();
  for (int p : {sck_, mosi_, miso_, cs_}) if (p >= 0) pinMode(p, INPUT);
  pins_.release(kOwnerId);
  sck_ = mosi_ = miso_ = cs_ = -1;
}

size_t P4SpiTarget::describe(uint8_t first, uint8_t *out, size_t capacity) {
  size_t used = 0, index = 0;
  // 64-byte FIFO transactions without DMA. Clock limit measured with the CH32 SPI1 masters on
  // 2026-09-22: ESP32-P4 exchanged 4 bytes both ways at 24 MHz (X035, /2); the classic ESP32 was
  // right up to 3 MHz and one bit late on MISO at 6 MHz (V003), so it declares 3 MHz.
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  constexpr uint32_t kMaxClockHz = 24000000u;
#else
  constexpr uint32_t kMaxClockHz = 3000000u;
#endif
  if (index++ >= first) { used = tlvPutU16(out, capacity, used, OEP_V0_TLV_CORE_MAX_LENGTH, kMaxFrame); if (!used) return 0; }
  if (index++ >= first) { const size_t n = tlvPutU32(out, capacity, used, OEP_V0_TLV_CORE_MAX_CLOCK_HZ, kMaxClockHz); if (!n) return used; used = n; }
  for (uint8_t c = 0; c < PinTable::kChannels; ++c) {
    if (!pins_.allowed(c)) continue;
    if (index++ < first) continue;
    const size_t n = tlvPutU16(out, capacity, used, OEP_V0_TLV_CORE_CHANNEL_CANDIDATE, c);
    if (!n) break;
    used = n;
  }
  return used;
}

#if defined(ARDUINO_ARCH_ESP32)

bool P4SpiTarget::start() {
  if (started_) return true;
  spi_bus_config_t bus = {};
  bus.mosi_io_num = mosi_; bus.miso_io_num = miso_; bus.sclk_io_num = sck_;
  bus.quadwp_io_num = -1; bus.quadhd_io_num = -1;
  bus.max_transfer_sz = kMaxFrame;
  spi_slave_interface_config_t cfg = {};
  cfg.spics_io_num = cs_;
  cfg.flags = bit_order_ ? SPI_SLAVE_BIT_LSBFIRST : 0;
  cfg.queue_size = 1;
  cfg.mode = mode_;
  if (spi_slave_initialize(SPI2_HOST, &bus, &cfg, SPI_DMA_DISABLED) != ESP_OK) return false;
  started_ = true;
  return true;
}

void P4SpiTarget::stop() {
  if (started_) spi_slave_free(SPI2_HOST);
  started_ = false; armed_ = false; queue_count_ = 0;
}

bool P4SpiTarget::arm(const uint8_t *tx, size_t tx_length, size_t length) {
  if (!started_ || armed_ || !length || length > kMaxFrame || tx_length > length) return false;
  memset(tx_buffer_, 0, sizeof tx_buffer_); memcpy(tx_buffer_, tx, tx_length);
  memset(rx_buffer_, 0, sizeof rx_buffer_);
  trans_ = {};
  trans_.length = length * 8; trans_.tx_buffer = tx_buffer_; trans_.rx_buffer = rx_buffer_;
  if (spi_slave_queue_trans(SPI2_HOST, &trans_, 0) != ESP_OK) return false;
  armed_ = true; armed_length_ = length;
  return true;
}

void P4SpiTarget::service() {
  if (!armed_) return;
  spi_slave_transaction_t *done = nullptr;
  if (spi_slave_get_trans_result(SPI2_HOST, &done, 0) != ESP_OK) return;
  armed_ = false;
  ++transactions_;
  if (queue_count_ == kQueueDepth) { ++errors_; return; }
  const size_t bytes = (done->trans_len + 7) / 8 > armed_length_ ? armed_length_ : (done->trans_len + 7) / 8;
  memcpy(queue_[queue_count_], rx_buffer_, bytes);
  queue_length_[queue_count_] = static_cast<uint8_t>(bytes);
  queue_bits_[queue_count_] = done->trans_len;
  ++queue_count_;
}

#else
bool P4SpiTarget::start() { return false; }
void P4SpiTarget::stop() { started_ = false; armed_ = false; }
bool P4SpiTarget::arm(const uint8_t *, size_t, size_t) { return false; }
void P4SpiTarget::service() {}
#endif

Result P4SpiTarget::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (operation) {
    case OEP_V0_P4_SPI_TARGET_OP_CONFIGURE: {
      struct oep_v0_p4_spi_target_configure_request request;
      if (!oep_v0_p4_spi_target_configure_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (request.mode > 3 || request.bit_order > 1) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (sck_ < 0) return rejected(OEP_V0_REJECT_UNAVAILABLE);  // needs a lease
      stop();
      mode_ = request.mode; bit_order_ = request.bit_order;
      if (!start()) return failed();
      return completed();
    }
    case OEP_V0_P4_SPI_TARGET_OP_ARM: {
      struct oep_v0_p4_spi_target_arm_request request;
      if (!oep_v0_p4_spi_target_arm_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!started_) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      if (request.length == 0 || request.length > kMaxFrame || request.tx_length > request.length) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (armed_) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      if (!arm(request.tx, request.tx_length, request.length)) return failed();
      return completed();
    }
    case OEP_V0_P4_SPI_TARGET_OP_READ_RX: {
      struct oep_v0_p4_spi_target_read_rx_request request;
      if (!oep_v0_p4_spi_target_read_rx_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      struct oep_v0_p4_spi_target_read_rx_result result = {0, 0, nullptr, 0};
      if (queue_count_) {
        result.pending = static_cast<uint8_t>(queue_count_ - 1);
        result.bits = queue_bits_[0];
        result.data = queue_[0]; result.data_length = queue_length_[0];
      }
      const size_t n = oep_v0_p4_spi_target_read_rx_result_pack(&result, out, capacity);
      if (queue_count_) {
        --queue_count_;
        memmove(queue_[0], queue_[1], sizeof(queue_[0]) * queue_count_);
        memmove(queue_length_, queue_length_ + 1, queue_count_);
        memmove(queue_bits_, queue_bits_ + 1, sizeof(queue_bits_[0]) * queue_count_);
      }
      return completed(n);
    }
    case OEP_V0_P4_SPI_TARGET_OP_STATUS: {
      struct oep_v0_p4_spi_target_status_request request;
      if (!oep_v0_p4_spi_target_status_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      struct oep_v0_p4_spi_target_status_result result = {
          static_cast<uint8_t>((started_ ? 1 : 0) | (armed_ ? 2 : 0) | (mode_ << 2) | (bit_order_ << 4) | (queue_count_ << 5)),
          transactions_, errors_};
      return completed(oep_v0_p4_spi_target_status_result_pack(&result, out, capacity));
    }
    case OEP_V0_P4_SPI_TARGET_OP_RESET: {
      struct oep_v0_p4_spi_target_reset_request request;
      if (!oep_v0_p4_spi_target_reset_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!started_) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      stop(); transactions_ = 0; errors_ = 0;
      if (!start()) return failed();
      return completed();
    }
    default:
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

}  // namespace oep
