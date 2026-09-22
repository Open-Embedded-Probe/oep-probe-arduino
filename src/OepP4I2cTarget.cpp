#include "OepP4I2cTarget.h"

#include <string.h>

#include "OepTlv.h"

namespace oep {

uint8_t P4I2cTarget::planCheck(const RoleAssignment *roles, size_t count) {
  if (count != 2) return OEP_V0_REJECT_MALFORMED_PAYLOAD;
  int sda = -1, scl = -1;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role == kRoleSda) sda = roles[i].channel;
    else if (roles[i].role == kRoleScl) scl = roles[i].channel;
    else return OEP_V0_REJECT_MALFORMED_PAYLOAD;
  }
  if (sda < 0 || scl < 0 || sda == scl) return OEP_V0_REJECT_MALFORMED_PAYLOAD;
  if (!pins_.free(sda) || !pins_.free(scl) || sda_ >= 0) return OEP_V0_REJECT_UNAVAILABLE;
  return 0;
}

bool P4I2cTarget::planApply(const RoleAssignment *roles, size_t count) {
  int sda = -1, scl = -1;
  for (size_t i = 0; i < count; ++i) (roles[i].role == kRoleSda ? sda : scl) = roles[i].channel;
  if (!pins_.claim(sda, kOwnerId)) return false;
  if (!pins_.claim(scl, kOwnerId)) { pins_.release(kOwnerId); return false; }
  sda_ = sda; scl_ = scl;
  return true;
}

void P4I2cTarget::planRelease() {
  stop();
  if (sda_ >= 0) pinMode(sda_, INPUT);
  if (scl_ >= 0) pinMode(scl_, INPUT);
  pins_.release(kOwnerId);
  sda_ = scl_ = -1;
  mode_ = kModeNone;
}

size_t P4I2cTarget::describe(uint8_t first, uint8_t *out, size_t capacity) {
  size_t used = 0, index = 0;
  // Declared limits: 128-byte frames at 1 MHz (E148, and the two-board HIL with the
  // peer P4 controller: framed 128 B at 1 MHz 20/20 on 2026-09-22). The "lost tail
  // bytes at 1 MHz" seen earlier were the peer's 256-byte HWCDC RX ring truncating
  // the 269-character FRAME command, not the slave.
  if (index++ >= first) { used = tlvPutU16(out, capacity, used, OEP_V0_TLV_CORE_MAX_LENGTH, kMaxFrame); if (!used) return 0; }
  if (index++ >= first) { const size_t n = tlvPutU32(out, capacity, used, OEP_V0_TLV_CORE_MAX_CLOCK_HZ, 1000000u); if (!n) return used; used = n; }
  for (uint8_t c = 0; c < PinTable::kChannels; ++c) {
    if (!pins_.allowed(c)) continue;
    if (index++ < first) continue;
    const size_t n = tlvPutU16(out, capacity, used, OEP_V0_TLV_CORE_CHANNEL_CANDIDATE, c);
    if (!n) break;
    used = n;
  }
  return used;
}

void P4I2cTarget::pushFrame(const uint8_t *data, size_t length) {
  if (queue_count_ == kQueueDepth) { ++errors_; return; }  // host did not drain in time
  memcpy(queue_[queue_count_], data, length);
  queue_length_[queue_count_] = static_cast<uint8_t>(length);
  ++queue_count_;
  ++rx_frames_;
}

#if defined(ARDUINO_ARCH_ESP32)

bool P4I2cTarget::receiveDone(i2c_slave_dev_handle_t, const i2c_slave_rx_done_event_data_t *, void *context) {
  static_cast<P4I2cTarget *>(context)->rx_done_ = true;  // ISR: flag only (E147)
  return false;
}

bool P4I2cTarget::start() {
  if (started_) return true;
  i2c_slave_config_t cfg = {};
  cfg.i2c_port = I2C_NUM_0;
  cfg.sda_io_num = static_cast<gpio_num_t>(sda_);
  cfg.scl_io_num = static_cast<gpio_num_t>(scl_);
  cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  cfg.send_buf_depth = 4096;  // preloaded TX ring (E150: 129-byte slots)
  cfg.slave_addr = address_;
  cfg.addr_bit_len = I2C_ADDR_BIT_LEN_7;
  if (i2c_new_slave_device(&cfg, &slave_) != ESP_OK) { slave_ = nullptr; return false; }
  i2c_slave_event_callbacks_t callbacks = {};
  callbacks.on_recv_done = receiveDone;
  if (i2c_slave_register_event_callbacks(slave_, &callbacks, this) != ESP_OK) { stop(); return false; }
  started_ = true;
  return true;
}

void P4I2cTarget::stop() {
  if (slave_) { i2c_del_slave_device(slave_); slave_ = nullptr; }
  started_ = false; armed_ = 0; rx_done_ = false; queue_count_ = 0; tx_slots_ = 0;
}

bool P4I2cTarget::arm(size_t length) {
  if (!started_ || !length || length > kMaxFrame) return false;
  armed_ = length;
  rx_done_ = false;
  return i2c_slave_receive(slave_, rx_buffer_, length) == ESP_OK;
}

void P4I2cTarget::service() {
  if (!rx_done_) return;
  rx_done_ = false;
  const size_t got = armed_;
  armed_ = 0;
  if (mode_ == kModeFixedRx) {
    pushFrame(rx_buffer_, got);
    if (!arm(got)) ++errors_;                       // keep accepting the same length
  } else if (mode_ == kModeFramedRx) {
    if (framed_header_) {
      const uint8_t length = rx_buffer_[0];
      if (!length || length > kMaxFrame) { ++errors_; arm(1); return; }
      framed_header_ = false;
      if (!arm(length)) { ++errors_; framed_header_ = true; arm(1); }
    } else {
      pushFrame(rx_buffer_, got);
      framed_header_ = true;
      if (!arm(1)) ++errors_;
    }
  }
}

#else
bool P4I2cTarget::start() { return false; }
void P4I2cTarget::stop() { started_ = false; }
bool P4I2cTarget::arm(size_t) { return false; }
void P4I2cTarget::service() {}
#endif

Result P4I2cTarget::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (operation) {
    case OEP_V0_P4_I2C_TARGET_OP_CONFIGURE: {
      struct oep_v0_p4_i2c_target_configure_request request;
      if (!oep_v0_p4_i2c_target_configure_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (request.address > 0x7f || request.mode < kModeFixedRx || request.mode > kModePreloadedTx) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (sda_ < 0) return rejected(OEP_V0_REJECT_UNAVAILABLE);  // needs a lease
      stop();
      address_ = request.address; mode_ = request.mode; framed_header_ = true;
      if (!start()) return failed();
      if (mode_ == kModeFramedRx && !arm(1)) return failed();
      return completed();
    }
    case OEP_V0_P4_I2C_TARGET_OP_ARM_RX: {
      struct oep_v0_p4_i2c_target_arm_rx_request request;
      if (!oep_v0_p4_i2c_target_arm_rx_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!started_ || mode_ != kModeFixedRx) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      if (!request.length || request.length > kMaxFrame) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (armed_) {
        // The v1 driver has no cancel for a pending receive job; a second
        // i2c_slave_receive() while one is armed took the P4 down. Recreate the
        // device to change the expected length.
        const uint8_t mode = mode_, address = address_;
        stop();
        mode_ = mode; address_ = address;
        if (!start()) return failed();
      }
      return arm(request.length) ? completed() : failed();
    }
    case OEP_V0_P4_I2C_TARGET_OP_READ_RX: {
      struct oep_v0_p4_i2c_target_read_rx_request request;
      if (!oep_v0_p4_i2c_target_read_rx_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!started_) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      struct oep_v0_p4_i2c_target_read_rx_result result = {0, nullptr, 0};
      if (queue_count_) {
        result.data = queue_[0]; result.data_length = queue_length_[0];
        const size_t n = oep_v0_p4_i2c_target_read_rx_result_pack(&result, out, capacity);
        // pop
        for (uint8_t i = 1; i < queue_count_; ++i) { memcpy(queue_[i - 1], queue_[i], queue_length_[i]); queue_length_[i - 1] = queue_length_[i]; }
        --queue_count_;
        out[0] = queue_count_;  // pending after this frame
        return completed(n);
      }
      return completed(oep_v0_p4_i2c_target_read_rx_result_pack(&result, out, capacity));
    }
    case OEP_V0_P4_I2C_TARGET_OP_PRELOAD_TX: {
      struct oep_v0_p4_i2c_target_preload_tx_request request;
      if (!oep_v0_p4_i2c_target_preload_tx_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!started_ || mode_ != kModePreloadedTx) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      if (!request.data_length || request.data_length > kMaxFrame) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
#if defined(ARDUINO_ARCH_ESP32)
      uint8_t slot[kMaxFrame + 1];
      memcpy(slot, request.data, request.data_length);
      slot[request.data_length] = 0x00;  // filler consumed at the master's NACK boundary (E150)
      if (i2c_slave_transmit(slave_, slot, request.data_length + 1, 50) != ESP_OK) return failed();
      ++tx_slots_;
#endif
      struct oep_v0_p4_i2c_target_preload_tx_result result = {tx_slots_};
      return completed(oep_v0_p4_i2c_target_preload_tx_result_pack(&result, out, capacity));
    }
    case OEP_V0_P4_I2C_TARGET_OP_STATUS: {
      struct oep_v0_p4_i2c_target_status_request request;
      if (!oep_v0_p4_i2c_target_status_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      struct oep_v0_p4_i2c_target_status_result result = {
          static_cast<uint8_t>((started_ ? 1 : 0) | (mode_ << 1) | (armed_ ? 0x10 : 0) | (queue_count_ << 5)),
          rx_frames_, tx_slots_, errors_};
      return completed(oep_v0_p4_i2c_target_status_result_pack(&result, out, capacity));
    }
    case OEP_V0_P4_I2C_TARGET_OP_RESET: {
      struct oep_v0_p4_i2c_target_reset_request request;
      if (!oep_v0_p4_i2c_target_reset_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!started_) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      const uint8_t mode = mode_, address = address_;
      stop();
      rx_frames_ = 0; errors_ = 0; mode_ = mode; address_ = address; framed_header_ = true;
      if (!start()) return failed();
      if (mode_ == kModeFramedRx && !arm(1)) return failed();
      return completed();
    }
    default:
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

}  // namespace oep
