// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepP4I2cTarget.h"

#include <string.h>


#if defined(ARDUINO_ARCH_ESP32)
#include <soc/i2c_struct.h>
#endif

namespace oep {

namespace {
}  // namespace

uint8_t P4I2cTarget::planCheck(const RoleAssignment *roles, size_t count) {
  if (count != 2) return kRejectMalformed;
  int sda = -1, scl = -1;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role == kRoleSda) sda = roles[i].channel;
    else if (roles[i].role == kRoleScl) scl = roles[i].channel;
    else return kRejectMalformed;
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
  return true;
}

void P4I2cTarget::planRelease() {
  stop();
  pins_.release(kOwnerId);   // each pin to its idle state
  sda_ = scl_ = -1;
  mode_ = kModeNone;
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
  w.u32(kTagFeatures, 0b11);   // bit0 preloaded tx, bit1 clock stretching (set_stretch)
  w.u8(kTagImplementation, 2);   // a dedicated peripheral
  return w.ok() ? w.length() : 0;
}

void P4I2cTarget::pushFrame(const uint8_t *data, size_t length) {
  if (queue_count_ == kQueueDepth) { ++errors_; return; }  // host did not drain in time
  memcpy(queue_[queue_count_], data, length);
  queue_length_[queue_count_] = static_cast<uint8_t>(length);
  ++queue_count_;
  ++rx_frames_;
}

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_rom_sys.h>
#include <driver/gpio.h>

bool P4I2cTarget::receiveDone(i2c_slave_dev_handle_t, const i2c_slave_rx_done_event_data_t *, void *context) {
  static_cast<P4I2cTarget *>(context)->rx_done_ = true;  // ISR: flag only (E147)
  return false;
}

bool P4I2cTarget::start() {
  if (started_) return true;
  i2c_slave_config_t cfg = {};
#if SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE
  cfg.flags.stretch_en = stretch_us_ ? 1 : 0;
#endif
  cfg.i2c_port = I2C_NUM_0;
  cfg.sda_io_num = static_cast<gpio_num_t>(sda_);
  cfg.scl_io_num = static_cast<gpio_num_t>(scl_);
  cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  cfg.send_buf_depth = 4096;  // preloaded TX ring (E150: 129-byte slots)
  cfg.slave_addr = address_;
  cfg.addr_bit_len = I2C_ADDR_BIT_LEN_7;
  if (i2c_new_slave_device(&cfg, &slave_) != ESP_OK) { slave_ = nullptr; return false; }
  // The slave driver enables no pull-ups and the fixture has none (2026-09-22: an X035 pin left as
  // input read 0 on the wire), so a real bus needs them here: the GPIO matrix lets the pad keep its
  // ~45 kOhm internal pull-up while the I2C peripheral owns it, as the IDF master driver does with
  // enable_internal_pullup. Weak, but a bus for master tests at <= 400 kHz rather than no bus.
  gpio_set_pull_mode(static_cast<gpio_num_t>(sda_), GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(static_cast<gpio_num_t>(scl_), GPIO_PULLUP_ONLY);
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
  // set_stretch: the ESP-IDF v1 slave driver enables the hardware stretch (address match on a master
  // read, TX empty, RX full) but never releases it - the raw flag stays set and SCL stays low until the
  // device is deleted (measured 2026-09-22: Wire timed out at 25 ms, SCL low for the whole capture).
  // So the hold is done here, from loop(): wait stretch_us after the stretch is seen, then release.
#if SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE
  if (stretch_us_ && started_ && I2C0.int_raw.slave_stretch_int_raw) {
    ++stretch_events_;
    esp_rom_delay_us(stretch_us_);
    I2C0.scl_stretch_conf.slave_scl_stretch_clr = 1;
    I2C0.int_clr.slave_stretch_int_clr = 1;
  }
#endif
  if (!rx_done_) return;
  rx_done_ = false;
  const size_t got = armed_;
  armed_ = 0;
  if (mode_ == kModeFixedRx) {
    // v1 driver limitation (2026-09-22, worklist B trace): rx_done also fires for a transaction that
    // ended after the address byte (NACK, 0 data bytes) and the event carries no length, so the
    // frame pushed here may be stale buffer content. Hosts must not treat a frame as proof of ACK.
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
  Tail tail;
  switch (operation) {
    case kOpConfigure: {   // address(u8) mode(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
      if (refused(parsed)) return parsed;
      if (payload[0] > 0x7f) return rejected(kRejectMalformed);
      if (payload[1] < kModeFixedRx || payload[1] > kModePreloadedTx) return rejected(kRejectUnsupported);
      if (sda_ < 0) return rejected(kRejectUnavailable);  // needs a plan
      stop();
      address_ = payload[0]; mode_ = payload[1]; framed_header_ = true;
      if (!start()) return failed();
      if (mode_ == kModeFramedRx && !arm(1)) return failed();
      return tail.finish(completed(), out, capacity);
    }
    case kOpArmRx: {   // length(u16) [TLV]
      const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
      if (refused(parsed)) return parsed;
      const uint16_t want = getU16(payload);
      if (!started_ || mode_ != kModeFixedRx) return rejected(kRejectUnavailable);
      if (!want || want > kMaxFrame) return rejected(kRejectMalformed);
      if (armed_) {
        // The v1 driver has no cancel for a pending receive job; a second
        // i2c_slave_receive() while one is armed took the P4 down. Recreate the
        // device to change the expected length.
        const uint8_t mode = mode_, address = address_;
        stop();
        mode_ = mode; address_ = address;
        if (!start()) return failed();
      }
      return arm(want) ? tail.finish(completed(), out, capacity) : failed();
    }
    case kOpReadRx: {   // [TLV] -> pending(u8) count(u16) data: the oldest received frame, then the ones still queued
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!started_) return rejected(kRejectUnavailable);
      const size_t data = queue_count_ ? queue_length_[0] : 0;
      if (capacity < 3 + data) return failed();
      if (data) memcpy(out + 3, queue_[0], data);
      if (queue_count_) {   // pop
        for (uint8_t i = 1; i < queue_count_; ++i) { memcpy(queue_[i - 1], queue_[i], queue_length_[i]); queue_length_[i - 1] = queue_length_[i]; }
        --queue_count_;
      }
      out[0] = queue_count_;   // pending after this frame
      putU16(out + 1, static_cast<uint16_t>(data));
      return tail.finish(completed(3 + data), out, capacity);
    }
    case kOpPreloadTx: {   // count(u16) data [TLV] -> slots(u8)
      if (length < 2) return rejected(kRejectMalformed);
      const uint16_t count = getU16(payload);
      const Result parsed = plainTail(tail, payload, length, 2u + count, out, capacity);
      if (refused(parsed)) return parsed;
      if (!started_ || mode_ != kModePreloadedTx) return rejected(kRejectUnavailable);
      if (!count || count > kMaxFrame) return rejected(kRejectMalformed);
      if (capacity < 1) return failed();
#if defined(ARDUINO_ARCH_ESP32)
      uint8_t slot[kMaxFrame + 1];
      memcpy(slot, payload + 2, count);
      // E150 (ESP32-P4): the slave's FIFO hands the master one byte more than the request at the NACK
      // boundary, so each slot carries one filler byte. The classic ESP32 slave does not (2026-09-22:
      // with the filler the second slot read back as 00 11 22 33), so it preloads the payload as is.
#if defined(CONFIG_IDF_TARGET_ESP32P4)
      constexpr size_t kFiller = 1;
      slot[count] = 0x00;
#else
      constexpr size_t kFiller = 0;
#endif
      if (i2c_slave_transmit(slave_, slot, count + kFiller, 50) != ESP_OK) return failed();
      ++tx_slots_;
#endif
      out[0] = tx_slots_;
      return tail.finish(completed(1), out, capacity);
    }
    case kOpStatus: {   // [TLV] -> flags(u8) rx_frames(u32) tx_slots(u8) errors(u16)
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 8) return failed();
      out[0] = static_cast<uint8_t>((started_ ? 1 : 0) | (mode_ << 1) | (armed_ ? 0x10 : 0) | (queue_count_ << 5));
      putU32(out + 1, rx_frames_);
      out[5] = tx_slots_;
      putU16(out + 6, errors_);
      return tail.finish(completed(8), out, capacity);
    }
    case kOpReadHw: {   // [TLV] -> sr int_raw fifo_st ctr slave_addr filter_cfg scl_stretch_conf (u32 each; zeros off the P4)
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 28) return failed();
      memset(out, 0, 28);
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_IDF_TARGET_ESP32P4)   // register images of the P4 I2C block only
      const uint32_t regs[7] = {I2C0.sr.val, I2C0.int_raw.val, I2C0.fifo_st.val, I2C0.ctr.val, I2C0.slave_addr.val,
                                I2C0.filter_cfg.val, I2C0.scl_stretch_conf.val};
      for (int i = 0; i < 7; ++i) putU32(out + 4 * i, regs[i]);
#endif
      return tail.finish(completed(28), out, capacity);
    }
    case kOpSetStretch: {   // stretch_us(u32) [TLV]
      const Result parsed = plainTail(tail, payload, length, 4, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t us = getU32(payload);
      if (us > 100000) return rejected(kRejectUnsupported);  // spins in the ISR; keep under the interrupt watchdog
      stretch_us_ = us;
      stretch_events_ = 0;
      return tail.finish(completed(), out, capacity);
    }
    case kOpReset: {
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!started_) return rejected(kRejectUnavailable);
      const uint8_t mode = mode_, address = address_;
      stop();
      rx_frames_ = 0; errors_ = 0; mode_ = mode; address_ = address; framed_header_ = true;
      if (!start()) return failed();
      if (mode_ == kModeFramedRx && !arm(1)) return failed();
      return tail.finish(completed(), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep
