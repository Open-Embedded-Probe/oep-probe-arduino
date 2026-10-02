// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepP4SpiTarget.h"

#include <string.h>

#include <initializer_list>


namespace oep {

uint8_t P4SpiTarget::planCheck(const RoleAssignment *roles, size_t count) {
  if (count != 4) return kRejectMalformed;
  int pin[5] = {-1, -1, -1, -1, -1};
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role < kRoleSck || roles[i].role > kRoleCs || pin[roles[i].role] >= 0) return kRejectMalformed;
    pin[roles[i].role] = roles[i].channel;
  }
  for (int r = kRoleSck; r <= kRoleCs; ++r) {
    for (int q = r + 1; q <= kRoleCs; ++q) if (pin[r] == pin[q]) return kRejectMalformed;
    if (!pins_.free(pin[r])) return kRejectUnavailable;
  }
  if (sck_ >= 0) return kRejectUnavailable;
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
  pins_.release(kOwnerId);   // each pin to its idle state
  sck_ = mosi_ = miso_ = cs_ = -1;
}

size_t P4SpiTarget::describe(uint8_t *out, size_t capacity) {
  // 64-byte FIFO transactions without DMA. Clock limit measured with the CH32 SPI1 masters on
  // 2026-09-22: ESP32-P4 exchanged 4 bytes both ways at 24 MHz (X035, /2); the classic ESP32 was
  // right up to 3 MHz and one bit late on MISO at 6 MHz (V003), so it declares 3 MHz.
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  constexpr uint32_t kMaxClockHz = 24000000u;
#else
  constexpr uint32_t kMaxClockHz = 3000000u;
#endif
  static const uint8_t kRoles[] = {kRoleSck, kRoleMosi, kRoleMiso, kRoleCs};
  TlvWriter w(out, capacity);
  w.roleChannels(kRoles, sizeof kRoles, pins_.allowedMask());
  w.u16(kTagMaxLength, kMaxFrame);
  w.u32(kTagMaxClockHz, kMaxClockHz);
  w.u32(kTagFeatures, 1);   // bit0 LSB first
  w.u8(kTagImplementation, 2);   // a dedicated peripheral
  w.u8(reg::fixture_spi_target::kTlvDescribeQueueDepth, kQueueDepth);
  return w.ok() ? w.length() : 0;
}

#if defined(OEP_SPI_SLAVE_DRIVER)

bool P4SpiTarget::begin() {
  spi_bus_config_t bus = {};
  bus.mosi_io_num = mosi_; bus.miso_io_num = miso_; bus.sclk_io_num = sck_;
  bus.quadwp_io_num = -1; bus.quadhd_io_num = -1;
  bus.max_transfer_sz = kMaxFrame;
  spi_slave_interface_config_t cfg = {};
  cfg.spics_io_num = cs_;
  cfg.flags = bit_order_ ? SPI_SLAVE_BIT_LSBFIRST : 0;
  cfg.queue_size = 1;
  cfg.mode = mode_;
  return spi_slave_initialize(SPI2_HOST, &bus, &cfg, SPI_DMA_DISABLED) == ESP_OK;
}

// While nothing is armed: a transaction that takes whatever the master sends (MISO 0), so it can be counted.
bool P4SpiTarget::queueIdle() {
  idle_trans_ = {};
  idle_trans_.length = kMaxFrame * 8; idle_trans_.tx_buffer = idle_tx_; idle_trans_.rx_buffer = idle_rx_;
  idle_queued_ = spi_slave_queue_trans(SPI2_HOST, &idle_trans_, 0) == ESP_OK;
  return idle_queued_;
}

bool P4SpiTarget::start() {
  if (started_) return true;
  if (!begin()) return false;
  started_ = true;
  queueIdle();
  return true;
}

void P4SpiTarget::stop() {
  if (started_) spi_slave_free(SPI2_HOST);
  started_ = false; armed_ = false; idle_queued_ = false; queue_count_ = 0;
}

bool P4SpiTarget::arm(const uint8_t *tx, size_t tx_length, size_t length) {
  if (!started_ || armed_ || !length || length > kMaxFrame || tx_length > length) return false;
  // The discard transaction sits in the driver and cannot be taken back: count what it got, then restart the target
  // with nothing queued (a transfer under way right now is cut; it was not armed).
  service();
  if (idle_queued_) {
    spi_slave_free(SPI2_HOST);
    idle_queued_ = false;
    if (!begin()) { started_ = false; return false; }
  }
  memset(tx_buffer_, 0, sizeof tx_buffer_); memcpy(tx_buffer_, tx, tx_length);
  memset(rx_buffer_, 0, sizeof rx_buffer_);
  trans_ = {};
  trans_.length = length * 8; trans_.tx_buffer = tx_buffer_; trans_.rx_buffer = rx_buffer_;
  if (spi_slave_queue_trans(SPI2_HOST, &trans_, 0) != ESP_OK) { queueIdle(); return false; }
  armed_ = true; armed_length_ = length;
  return true;
}

void P4SpiTarget::service() {
  if (!started_) return;
  spi_slave_transaction_t *done = nullptr;
  while (spi_slave_get_trans_result(SPI2_HOST, &done, 0) == ESP_OK && done) {
    if (done == &idle_trans_) {   // a transfer nobody armed: MOSI dropped, counted (fixture §4)
      idle_queued_ = false;
      if (done->trans_len) { ++transactions_; ++errors_; }
      queueIdle();
      continue;
    }
    if (!armed_ || done != &trans_) continue;
    if (done->trans_len == 0) {   // CS without a clock: noise, the arm keeps waiting
      if (spi_slave_queue_trans(SPI2_HOST, &trans_, 0) != ESP_OK) { armed_ = false; queueIdle(); }
      continue;
    }
    armed_ = false;
    ++transactions_;
    if (done->trans_len > armed_length_ * 8) ++errors_;   // over length: the rest was dropped
    if (queue_count_ == kQueueDepth) {
      ++errors_;
    } else {
      const size_t bytes = (done->trans_len + 7) / 8 > armed_length_ ? armed_length_ : (done->trans_len + 7) / 8;
      memcpy(queue_[queue_count_], rx_buffer_, bytes);
      queue_length_[queue_count_] = static_cast<uint8_t>(bytes);
      queue_bits_[queue_count_] = done->trans_len;
      ++queue_count_;
    }
    queueIdle();
  }
}

#else
bool P4SpiTarget::start() { return false; }
void P4SpiTarget::stop() { started_ = false; armed_ = false; }
bool P4SpiTarget::arm(const uint8_t *, size_t, size_t) { return false; }
void P4SpiTarget::service() {}
#endif

Result P4SpiTarget::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (operation) {
    case kOpConfigure: {   // mode(u8) bit_order(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
      if (refused(parsed)) return parsed;
      if (payload[0] > 3 || payload[1] > 1) return rejected(kRejectMalformed);   // not a mode / order of the table
      if (sck_ < 0) return wrongState(out, capacity);  // needs a plan (fixture §4: unavailable cause 6)
      stop();
      mode_ = payload[0]; bit_order_ = payload[1];
      if (!start()) return failed();
      return tail.finish(completed(), out, capacity);
    }
    case kOpArm: {   // length(u16) count(u16) tx [TLV]: MISO bytes for the next CS-framed transaction of length bytes
      if (length < 4) return rejected(kRejectMalformed);
      const uint16_t want = getU16(payload), count = getU16(payload + 2);
      const Result parsed = plainTail(tail, payload, length, 4u + count, out, capacity);
      if (refused(parsed)) return parsed;
      if (want == 0 || count > want) return rejected(kRejectMalformed);
      if (want > kMaxFrame) return unsupportedValue(out, capacity);   // over max_length
      if (!started_ || armed_) return wrongState(out, capacity);       // not configured, or one is waiting already
      if (!arm(payload + 4, count, want)) return failed();
      return tail.finish(completed(), out, capacity);
    }
    case kOpReadRx: {   // [TLV] -> pending(u8) bits(u32) count(u16) data: the oldest finished transaction
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      const size_t data = queue_count_ ? queue_length_[0] : 0;
      if (capacity < 7 + data) return failed();
      out[0] = queue_count_ ? static_cast<uint8_t>(queue_count_ - 1) : 0;
      putU32(out + 1, queue_count_ ? queue_bits_[0] : 0);
      putU16(out + 5, static_cast<uint16_t>(data));
      if (data) memcpy(out + 7, queue_[0], data);
      if (queue_count_) {
        --queue_count_;
        memmove(queue_[0], queue_[1], sizeof(queue_[0]) * queue_count_);
        memmove(queue_length_, queue_length_ + 1, queue_count_);
        memmove(queue_bits_, queue_bits_ + 1, sizeof(queue_bits_[0]) * queue_count_);
      }
      return tail.finish(completed(7 + data), out, capacity);
    }
    case kOpStatus: {   // [TLV] -> state mode bit_order armed queued (u8 each) transactions(u32) errors(u32) [TLV]
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 13) return failed();
      out[0] = started_ ? 1 : 0;   // state: 0 not configured, 1 running
      out[1] = mode_;
      out[2] = bit_order_;
      out[3] = armed_ ? 1 : 0;
      out[4] = static_cast<uint8_t>(queue_count_);
      putU32(out + 5, transactions_);
      putU32(out + 9, errors_);
      return tail.finish(completed(13), out, capacity);
    }
    case kOpReset: {
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!started_) return wrongState(out, capacity);   // state 0 (fixture §4)
      stop(); transactions_ = 0; errors_ = 0;
      if (!start()) return failed();
      return tail.finish(completed(), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep
