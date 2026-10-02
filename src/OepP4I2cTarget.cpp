// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepP4I2cTarget.h"

#include <string.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/gpio.h>
#include <esp_timer.h>
#include <hal/i2c_ll.h>
#include <soc/i2c_periph.h>
#endif

namespace oep {

uint8_t P4I2cTarget::planCheck(const RoleAssignment *roles, size_t count) {
  if (count != 2) return kRejectMalformed;
  int sda = -1, scl = -1;
  for (size_t i = 0; i < count; ++i) {
    int &pin = roles[i].role == kRoleSda ? sda : scl;
    if ((roles[i].role != kRoleSda && roles[i].role != kRoleScl) || pin >= 0) return kRejectMalformed;   // each role once
    pin = roles[i].channel;
  }
  if (sda < 0 || scl < 0 || sda == scl) return kRejectMalformed;
  if (!pins_.free(sda) || !pins_.free(scl) || sda_ >= 0) return kRejectUnavailable;
  return 0;
}

bool P4I2cTarget::planApply(const RoleAssignment *roles, size_t count) {
  int sda = -1, scl = -1;
  for (size_t i = 0; i < count; ++i) (roles[i].role == kRoleSda ? sda : scl) = roles[i].channel;
  if (!pins_.claim(sda, kOwnerId)) return false;
  if (!pins_.claim(scl, kOwnerId)) { pins_.release(kOwnerId); return false; }
  sda_ = sda; scl_ = scl;
  pins_.ownStrength(sda);
  pins_.ownStrength(scl);
  return true;
}

// Releasing or replacing the plan: back to the state right after describe (fixture §3).
void P4I2cTarget::planRelease() {
  stop();
  pins_.release(kOwnerId);   // each pin to its idle state
  sda_ = scl_ = -1;
  mode_ = kModeNone; address_ = 0; stretch_us_ = 0;
  clearTarget();
}

size_t P4I2cTarget::describe(uint8_t *out, size_t capacity) {
  // Declared limits: 128-byte frames at 1 MHz (E148, and the two-board HIL with the peer P4 controller: framed 128 B
  // at 1 MHz 20/20 on 2026-09-22). The "lost tail bytes at 1 MHz" seen earlier were the peer's 256-byte HWCDC RX ring
  // truncating the 269-character FRAME command, not the slave.
  static const uint8_t kRoles[] = {kRoleSda, kRoleScl};
  TlvWriter w(out, capacity);
  w.roleChannels(kRoles, sizeof kRoles, pins_.allowedMask());
  w.u16(kTagMaxLength, kMaxFrame);
  w.u32(kTagMaxClockHz, 1000000u);
  w.u32(kTagFeatures, kStretch ? 0b11 : 0b01);   // bit0 preloaded tx, bit1 clock stretching
  w.u8(kTagImplementation, 2);   // a dedicated peripheral
  w.u8(reg::fixture_i2c_target::kTlvDescribeQueueDepth, kQueueDepth);
  if (kStretch) w.u32(reg::fixture_i2c_target::kTlvDescribeMaxStretchUs, kMaxStretchUs);
  return w.ok() ? w.length() : 0;
}

void P4I2cTarget::clearTarget() {
  lock();
  rx_frames_ = 0; errors_ = 0; armed_ = 0; queue_count_ = 0;
  slot_head_ = 0; slot_count_ = 0; slot_serial_ = 0;
  rx_count_ = 0; tx_loaded_ = 0; tx_pos_ = 0; tx_stale_ = false;
  unlock();
}

uint8_t P4I2cTarget::txByte(size_t pos) const {
  if (mode_ != kModePreloadedTx || !slot_count_ || pos >= slot_length_[slot_head_]) return 0xFF;
  return slot_[slot_head_][pos];
}

void P4I2cTarget::pushFrame(const uint8_t *data, size_t length) {
  if (queue_count_ == kQueueDepth) { ++errors_; return; }   // the host did not drain in time: dropped, not a frame
  memcpy(queue_[queue_count_], data, length);
  queue_length_[queue_count_] = static_cast<uint8_t>(length);
  ++queue_count_;
  ++rx_frames_;
}

// One transaction ended (STOP). Its write, by byte count (fixture §3): none (the address alone) counts nothing; mode 1
// takes exactly the armed length (else, and when not armed, it is dropped with an error and the wait goes on); mode 2
// takes a length byte L (1..max_length) and exactly L more; mode 3 takes no write. A read used up the head slot.
void P4I2cTarget::onTransaction(bool read) {
  const size_t count = rx_count_;
  rx_count_ = 0;
  if (count) {
    if (mode_ == kModeFixedRx) {
      if (armed_ && count == armed_) pushFrame(rx_, count);
      else ++errors_;
    } else if (mode_ == kModeFramedRx) {
      const size_t frame = rx_[0];
      if (frame && frame <= kMaxFrame && count == frame + 1) pushFrame(rx_ + 1, frame);
      else ++errors_;
    } else {
      ++errors_;
    }
  }
  if (read && mode_ == kModePreloadedTx && slot_count_) {
    slot_head_ = static_cast<uint8_t>((slot_head_ + 1) % kQueueDepth);
    --slot_count_;
  }
}

#if defined(ARDUINO_ARCH_ESP32)

namespace {
i2c_dev_t *const kDev = I2C_LL_GET_HW(0);
#if defined(CONFIG_IDF_TARGET_ESP32P4)
constexpr uint32_t kIntrMask = I2C_LL_SLAVE_RX_INT | I2C_LL_SLAVE_TX_INT | I2C_SLAVE_STRETCH_INT_ENA_M;
#else
constexpr uint32_t kIntrMask = I2C_LL_SLAVE_RX_INT | I2C_LL_SLAVE_TX_INT;
#endif
uint32_t txFifoHeld() {
  uint32_t room = 0;
  i2c_ll_get_txfifo_len(kDev, &room);
  return SOC_I2C_FIFO_LEN - room;
}
}  // namespace

void P4I2cTarget::lock() { portENTER_CRITICAL_SAFE(&lock_); }
void P4I2cTarget::unlock() { portEXIT_CRITICAL_SAFE(&lock_); }

// The received bytes so far into rx_ (past its size only counted: such a write fits no mode anyway).
void P4I2cTarget::drainRx() {
  uint32_t n = 0;
  i2c_ll_get_rxfifo_cnt(kDev, &n);
  while (n) {
    uint8_t chunk[SOC_I2C_FIFO_LEN];
    const uint32_t take = n > sizeof chunk ? sizeof chunk : n;
    i2c_ll_read_rxfifo(kDev, chunk, static_cast<uint8_t>(take));
    for (uint32_t i = 0; i < take; ++i, ++rx_count_) if (rx_count_ < sizeof rx_) rx_[rx_count_] = chunk[i];
    n -= take;
  }
}

// Keep the TX FIFO full: the head slot from tx_pos_, then 0xFF (fixture §3: an empty set, and modes 1 / 2, send 0xFF).
void P4I2cTarget::fillTx() {
  uint32_t room = 0;
  i2c_ll_get_txfifo_len(kDev, &room);
  uint8_t chunk[SOC_I2C_FIFO_LEN];
  uint32_t n = 0;
  while (n < room && n < sizeof chunk) chunk[n++] = txByte(tx_pos_++);
  if (n) i2c_ll_write_txfifo(kDev, chunk, static_cast<uint8_t>(n));
  tx_loaded_ += n;
}

// The next read starts from the head slot's first byte (a read of another length does not shift the slots).
void P4I2cTarget::reloadTx() {
  i2c_ll_txfifo_rst(kDev);
  tx_loaded_ = 0; tx_pos_ = 0; tx_stale_ = false;
  fillTx();
}

void P4I2cTarget::isr(void *context) {
  P4I2cTarget *self = static_cast<P4I2cTarget *>(context);
  uint32_t status = 0;
  i2c_ll_get_intr_mask(kDev, &status);
  status &= kIntrMask;
  i2c_ll_clear_intr_mask(kDev, status);   // handled here: the driver's own handler (after this one) sees none of it
  portENTER_CRITICAL_ISR(&self->lock_);
  self->drainRx();
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  if (status & I2C_SLAVE_STRETCH_INT_ENA_M) {   // SCL held: at a received byte's ACK, or a read's address
    self->fillTx();
    if (self->stretch_us_) {
      self->stretch_held_ = true;
      self->stretch_since_ = static_cast<uint32_t>(esp_timer_get_time());   // service() lets go
    } else {
      i2c_ll_slave_clear_stretch(kDev);
    }
  }
#endif
  if (status & I2C_LL_SLAVE_TX_INT) self->fillTx();
  if (status & I2C_TRANS_COMPLETE_INT_ENA_M) {
    // the read, if any, took bytes out of the TX FIFO (a STOP of another target's transaction took none)
    const bool read = self->tx_loaded_ > txFifoHeld();
    self->onTransaction(read);
    if (read || self->tx_stale_) self->reloadTx();
  }
  portEXIT_CRITICAL_ISR(&self->lock_);
}

void P4I2cTarget::applyStretch() {
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  const bool on = stretch_us_ != 0;
  kDev->scl_stretch_conf.slave_byte_ack_lvl = 0;   // the ACK it then sends
  kDev->scl_stretch_conf.slave_byte_ack_ctl_en = on;
  i2c_ll_slave_enable_scl_stretch(kDev, on);
  i2c_ll_update(kDev);
  if (!on && stretch_held_) { stretch_held_ = false; i2c_ll_slave_clear_stretch(kDev); }
#endif
}

bool P4I2cTarget::start() {
  if (started_) return true;
  i2c_slave_config_t cfg = {};
  cfg.i2c_port = I2C_NUM_0;
  cfg.sda_io_num = static_cast<gpio_num_t>(sda_);
  cfg.scl_io_num = static_cast<gpio_num_t>(scl_);
  cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  cfg.send_buf_depth = 32;   // the driver's transmit ring: unused (the TX FIFO is filled here)
  cfg.slave_addr = address_;
  cfg.addr_bit_len = I2C_ADDR_BIT_LEN_7;
  if (i2c_new_slave_device(&cfg, &slave_) != ESP_OK) { slave_ = nullptr; return false; }
  // The slave driver enables no pull-ups and the fixture has none (2026-09-22: an X035 pin left as
  // input read 0 on the wire), so a real bus needs them here: the GPIO matrix lets the pad keep its
  // ~45 kOhm internal pull-up while the I2C peripheral owns it, as the IDF master driver does with
  // enable_internal_pullup. Weak, but a bus for master tests at <= 400 kHz rather than no bus.
  gpio_set_pull_mode(static_cast<gpio_num_t>(sda_), GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(static_cast<gpio_num_t>(scl_), GPIO_PULLUP_ONLY);
  // This handler on the driver's shared interrupt: added later, so it runs first (esp_intr_alloc chains the newest
  // first) and clears what it handled. The driver never gets a receive or transmit job, so its handler has none.
  if (esp_intr_alloc_intrstatus(i2c_periph_signal[I2C_NUM_0].irq, ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED,
                                reinterpret_cast<uint32_t>(i2c_ll_get_interrupt_status_reg(kDev)), kIntrMask, isr, this,
                                &intr_) != ESP_OK) {
    intr_ = nullptr;
    i2c_del_slave_device(slave_); slave_ = nullptr;
    return false;
  }
  lock();
  i2c_ll_rxfifo_rst(kDev);
  rx_count_ = 0;
  reloadTx();
  applyStretch();
  i2c_ll_clear_intr_mask(kDev, kIntrMask);
  i2c_ll_enable_intr_mask(kDev, kIntrMask);
  unlock();
  started_ = true;
  return true;
}

void P4I2cTarget::stop() {
  if (slave_) {
    i2c_ll_disable_intr_mask(kDev, kIntrMask);
    if (intr_) { esp_intr_free(intr_); intr_ = nullptr; }
    if (stretch_held_) i2c_ll_slave_clear_stretch(kDev);
    i2c_del_slave_device(slave_);
    slave_ = nullptr;
  }
  stretch_held_ = false;
  started_ = false;
}

void P4I2cTarget::service() {
  if (!started_) return;
  lock();
  if (stretch_held_ && static_cast<uint32_t>(esp_timer_get_time()) - stretch_since_ >= stretch_us_) {
    stretch_held_ = false;
    i2c_ll_slave_clear_stretch(kDev);
  }
  if (tx_stale_ && !i2c_ll_is_bus_busy(kDev)) reloadTx();
  unlock();
}

#else   // no hardware: the host test's fake controller, or nothing

void P4I2cTarget::lock() {}
void P4I2cTarget::unlock() {}
void P4I2cTarget::service() {}

#if defined(OEP_HOST_FAKE_I2C_SLAVE)
bool P4I2cTarget::start() { started_ = true; rx_count_ = 0; tx_pos_ = 0; return true; }
void P4I2cTarget::stop() { started_ = false; }

bool P4I2cTarget::hostTransaction(const uint8_t *data, size_t count, size_t read, uint8_t *got) {
  if (!started_) return false;
  for (size_t i = 0; i < count; ++i, ++rx_count_) if (rx_count_ < sizeof rx_) rx_[rx_count_] = data[i];
  for (size_t i = 0; i < read; ++i) got[i] = txByte(i);
  onTransaction(read != 0);
  return true;
}
#else
bool P4I2cTarget::start() { return false; }
void P4I2cTarget::stop() { started_ = false; }
#endif

#endif

Result P4I2cTarget::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (operation) {
    case kOpConfigure: {   // address(u8) mode(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
      if (refused(parsed)) return parsed;
      if (payload[0] > 0x7f || payload[1] < kModeFixedRx || payload[1] > kModePreloadedTx) return rejected(kRejectMalformed);
      if (sda_ < 0) return wrongState(out, capacity);  // needs a plan (fixture §3: unavailable cause 6)
      stop();
      clearTarget();   // frames, the wait, slots, rx_frames and errors go; stretch stays
      address_ = payload[0]; mode_ = payload[1];
      if (!start()) return failed();
      return tail.finish(completed(), out, capacity);
    }
    case kOpArmRx: {   // length(u16) [TLV]
      const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
      if (refused(parsed)) return parsed;
      const uint16_t want = getU16(payload);
      if (!want) return rejected(kRejectMalformed);
      if (want > kMaxFrame) return unsupportedValue(out, capacity);   // over max_length (fixture §3)
      if (!started_ || mode_ != kModeFixedRx) return wrongState(out, capacity);
      lock();
      armed_ = want;   // replaces a wait already there; it lasts until the next arm_rx, reset, configure or plan
      unlock();
      return tail.finish(completed(), out, capacity);
    }
    case kOpReadRx: {   // [TLV] -> pending(u8) count(u16) data: the oldest received frame, then the ones still queued
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!started_) return wrongState(out, capacity);   // state 0 (fixture §3)
      lock();
      const size_t data = queue_count_ ? queue_length_[0] : 0;
      if (capacity < 3 + data) { unlock(); return failed(); }
      if (data) memcpy(out + 3, queue_[0], data);
      if (queue_count_) {   // pop
        for (uint8_t i = 1; i < queue_count_; ++i) { memcpy(queue_[i - 1], queue_[i], queue_length_[i]); queue_length_[i - 1] = queue_length_[i]; }
        --queue_count_;
      }
      out[0] = queue_count_;   // pending after this frame
      unlock();
      putU16(out + 1, static_cast<uint16_t>(data));
      return tail.finish(completed(3 + data), out, capacity);
    }
    case kOpPreloadTx: {   // count(u16) data [TLV] -> slots(u8)
      if (length < 2) return rejected(kRejectMalformed);
      const uint16_t count = getU16(payload);
      const Result parsed = plainTail(tail, payload, length, 2u + count, out, capacity);
      if (refused(parsed)) return parsed;
      if (!count) return rejected(kRejectMalformed);
      if (count > kMaxFrame) return unsupportedValue(out, capacity);
      if (!started_ || mode_ != kModePreloadedTx) return wrongState(out, capacity);
      if (capacity < 1) return failed();
      lock();
      if (slot_count_ == kQueueDepth) {   // queue_depth unread slots already (fixture §3): nothing placed
        unlock();
        return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
      }
      const uint8_t at = static_cast<uint8_t>((slot_head_ + slot_count_) % kQueueDepth);
      memcpy(slot_[at], payload + 2, count);
      slot_length_[at] = static_cast<uint8_t>(count);
      ++slot_count_;
      out[0] = ++slot_serial_;
#if defined(ARDUINO_ARCH_ESP32)
      if (slot_count_ == 1) {   // the TX FIFO held 0xFF: put this slot there, unless a read may be under way
        if (i2c_ll_is_bus_busy(kDev)) tx_stale_ = true;
        else reloadTx();
      }
#endif
      unlock();
      return tail.finish(completed(1), out, capacity);
    }
    case kOpStatus: {   // [TLV] -> state mode armed queued (u8 each) rx_frames(u32) tx_slots(u8) errors(u32) [TLV]
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 13) return failed();
      lock();
      out[0] = started_ ? 1 : 0;   // state: 0 not configured, 1 running
      out[1] = mode_;
      out[2] = armed_ ? 1 : 0;
      out[3] = queue_count_;
      putU32(out + 4, rx_frames_);
      out[8] = mode_ == kModePreloadedTx ? slot_count_ : 0;   // unread slots
      putU32(out + 9, errors_);
      unlock();
      return tail.finish(completed(13), out, capacity);
    }
    case kOpStretch: {   // stretch_us(u32) [TLV]: in any state, from the next byte on (fixture §3)
      if (!kStretch) return rejected(kRejectUnknownOperation);   // features bit1 not declared
      const Result parsed = plainTail(tail, payload, length, 4, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t us = getU32(payload);
      if (us > kMaxStretchUs) return unsupportedValue(out, capacity);   // over max_stretch_us
      lock();
      stretch_us_ = us;
#if defined(ARDUINO_ARCH_ESP32)
      if (started_) applyStretch();
#endif
      unlock();
      return tail.finish(completed(), out, capacity);
    }
    case kOpReset: {
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!started_) return wrongState(out, capacity);   // state 0 (fixture §3)
      stop();
      clearTarget();   // as right after configure: mode, address and stretch stay
      if (!start()) return failed();
      return tail.finish(completed(), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep
