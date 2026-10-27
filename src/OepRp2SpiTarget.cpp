// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepRp2SpiTarget.h"

#if defined(ARDUINO_ARCH_RP2040)

#include <hardware/dma.h>
#include <hardware/gpio.h>
#include <hardware/pio_instructions.h>
#include <string.h>

#include <initializer_list>

namespace oep {

namespace {

// The programs: start (CS low: MISO an output), then a byte at a time - a tx byte (or X, 0), its bits out and the MOSI
// bits in at the mode's edges, Y counting every bit down from 0xFFFFFFFF. CPHA 0: a bit out before the leading edge,
// in at it; CPHA 1: out at the leading edge, in at the trailing one. Edges after CPOL's inversion: leading = rising.
constexpr uint8_t kLength = 9;
uint16_t gProgram[2][kLength];
uint8_t gLength[2];
void build(int sck, int cs) {
  const uint s = static_cast<uint>(sck), c = static_cast<uint>(cs);
  uint16_t *p = gProgram[0];   // CPHA 0
  p[0] = pio_encode_wait_gpio(false, c);      // start: CS low
  p[1] = pio_encode_set(pio_pindirs, 1);      // MISO driven
  p[2] = pio_encode_pull(false, false);       // byte: the next tx byte, X (0) past it
  p[3] = pio_encode_out(pio_pins, 1);         // bit: out before the leading edge
  p[4] = pio_encode_wait_gpio(true, s);
  p[5] = pio_encode_in(pio_pins, 1);          // in at it
  p[6] = pio_encode_jmp_y_dec(7);             // count (always on: Y does not reach 0 in a transfer)
  p[7] = pio_encode_wait_gpio(false, s);      // the trailing edge
  p[8] = pio_encode_jmp_not_osre(3);          // the byte's next bit, else the next byte (wrap to 2)
  gLength[0] = 9;
  p = gProgram[1];   // CPHA 1
  p[0] = pio_encode_wait_gpio(false, c);
  p[1] = pio_encode_set(pio_pindirs, 1);
  p[2] = pio_encode_pull(false, false);
  p[3] = pio_encode_wait_gpio(true, s);       // bit: the leading edge
  p[4] = pio_encode_out(pio_pins, 1);         // out at it
  p[5] = pio_encode_wait_gpio(false, s);      // the trailing edge
  p[6] = pio_encode_in(pio_pins, 1);          // in at it
  p[7] = pio_encode_jmp_y_dec(8);
  p[8] = pio_encode_jmp_not_osre(3);
  gLength[1] = 9;
}

Rp2SpiTarget *gTarget = nullptr;   // the one whose CS interrupt is on
void csRise() { if (gTarget) gTarget->onCsRise(); }

}  // namespace

uint8_t Rp2SpiTarget::planCheck(const RoleAssignment *roles, size_t count) {
  if (count != 4) return kRejectUnsupported;
  int pin[5] = {-1, -1, -1, -1, -1};
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role < kRoleSck || roles[i].role > kRoleCs || pin[roles[i].role] >= 0) return kRejectUnsupported;
    pin[roles[i].role] = roles[i].channel;
  }
  for (int r = kRoleSck; r <= kRoleCs; ++r) {
    if (!pins_.allowed(pin[r]) || pin[r] > 31) return kRejectUnsupported;
    for (int q = r + 1; q <= kRoleCs; ++q) if (pin[r] == pin[q]) return kRejectUnsupported;
  }
  if (!pins_.canOutput(pin[kRoleMiso])) return kRejectUnsupported;
  refusal_cause_ = reg::core::kUnavailableCauseWrongState;
  refusal_channel_ = 0xffff;
  for (size_t i = 0; i < count; ++i) {
    const uint8_t idle = pins_.idle(roles[i].channel);
    if (idle == PinTable::kIdleOutputLow || idle == PinTable::kIdleOutputHigh) {
      refusal_cause_ = reg::core::kUnavailableCauseHeldBySettings;
      refusal_channel_ = roles[i].channel;
      return kRejectUnavailable;
    }
  }
  for (int r = kRoleSck; r <= kRoleCs; ++r) if (!pins_.freeFor(pin[r], kOwnerId)) return kRejectUnavailable;
  if (sck_ >= 0 && started_) return kRejectUnavailable;   // running: no change (cause 6)
  return 0;
}

bool Rp2SpiTarget::planApply(const RoleAssignment *roles, size_t count) {
  int pin[5] = {-1, -1, -1, -1, -1};
  for (size_t i = 0; i < count; ++i) pin[roles[i].role] = roles[i].channel;
  for (int r = kRoleSck; r <= kRoleCs; ++r) {
    if (!pins_.claim(pin[r], kOwnerId)) { pins_.release(kOwnerId); return false; }
  }
  sck_ = pin[kRoleSck]; mosi_ = pin[kRoleMosi]; miso_ = pin[kRoleMiso]; cs_ = pin[kRoleCs];
  for (int r = kRoleSck; r <= kRoleCs; ++r) pins_.ownStrength(static_cast<uint16_t>(pin[r]));
  return true;
}

void Rp2SpiTarget::planRelease() {
  stop();
  pins_.release(kOwnerId);
  sck_ = mosi_ = miso_ = cs_ = -1;
  mode_ = bit_order_ = 0;
  transactions_ = errors_ = 0;
  queue_count_ = 0;
}

size_t Rp2SpiTarget::describe(uint8_t *out, size_t capacity) {
  static const uint8_t kInputs[] = {kRoleSck, kRoleMosi, kRoleCs}, kMiso[] = {kRoleMiso};
  namespace spi = reg::fixture_spi_target;
  constexpr uint64_t kGpios = 0xFFFFFFFFull;   // the programs' waits name GPIO 0-31
  TlvWriter w(out, capacity);
  w.roleChannels(kInputs, sizeof kInputs, pins_.allowedMask() & kGpios);
  w.roleChannels(kMiso, sizeof kMiso, pins_.outputMask() & kGpios);
  w.u16(kTagMaxLength, kMaxFrame);
  w.u32(kTagMaxClockHz, kMaxClockHz);
  w.u32(kTagFeatures, spi::kFeaturesLsbFirst);
  w.u8(spi::kTlvDescribeQueueDepth, kQueueDepth);
  w.u8(spi::kTlvDescribeModes, kModes);
  w.u32(spi::kTlvDescribeCsSetupNs, kCsSetupNs);
  return w.ok() ? w.length() : 0;
}

bool Rp2SpiTarget::start() {
  build(sck_, cs_);
  const uint8_t which = mode_ & 1;   // CPHA
  pio_program_t program = {};
  program.instructions = gProgram[which];
  program.length = gLength[which];
  program.origin = -1;
  uint sm = 0, offset = 0;
  PIO pio = nullptr;
  if (!pio_claim_free_sm_and_add_program(&program, &pio, &sm, &offset)) return false;
  const int rx = dma_claim_unused_channel(false), tx = dma_claim_unused_channel(false);
  if (rx < 0 || tx < 0) {
    if (rx >= 0) dma_channel_unclaim(rx);
    if (tx >= 0) dma_channel_unclaim(tx);
    pio_remove_program_and_unclaim_sm(&program, pio, sm, offset);
    return false;
  }
  pio_ = pio; sm_ = static_cast<int>(sm); offset_ = static_cast<int>(offset); dma_rx_ = rx; dma_tx_ = tx;
  program_ = which;
  // SCK / MOSI / CS inputs (SCK inverted for CPOL 1: its leading edge a rise), MISO the PIO's out / set pin, undriven
  for (const int p : {sck_, mosi_, cs_}) { gpio_init(static_cast<uint>(p)); gpio_set_dir(static_cast<uint>(p), false); }
  gpio_set_inover(static_cast<uint>(sck_), (mode_ >> 1) ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
  pio_gpio_init(pio_, static_cast<uint>(miso_));
  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, static_cast<uint>(offset_ + 2), static_cast<uint>(offset_ + gLength[which] - 1));
  sm_config_set_in_pins(&c, static_cast<uint>(mosi_));
  sm_config_set_out_pins(&c, static_cast<uint>(miso_), 1);
  sm_config_set_set_pins(&c, static_cast<uint>(miso_), 1);
  const bool msb = bit_order_ == 0;
  sm_config_set_in_shift(&c, !msb, true, 8);     // autopush every byte
  sm_config_set_out_shift(&c, !msb, false, 8);   // pull by hand (noblock), 8 bits a byte
  pio_sm_init(pio_, sm, static_cast<uint>(offset_), &c);
  gTarget = this;
  attachInterrupt(digitalPinToInterrupt(cs_), csRise, RISING);
  started_ = true;
  restart();
  return true;
}

void Rp2SpiTarget::stop() {
  if (!started_) return;
  detachInterrupt(digitalPinToInterrupt(cs_));
  gTarget = nullptr;
  pio_sm_set_enabled(pio_, static_cast<uint>(sm_), false);
  dma_channel_abort(static_cast<uint>(dma_rx_));
  dma_channel_abort(static_cast<uint>(dma_tx_));
  dma_channel_unclaim(static_cast<uint>(dma_rx_));
  dma_channel_unclaim(static_cast<uint>(dma_tx_));
  pio_program_t program = {};
  program.instructions = gProgram[program_];
  program.length = gLength[program_];
  program.origin = -1;
  pio_remove_program_and_unclaim_sm(&program, pio_, static_cast<uint>(sm_), static_cast<uint>(offset_));
  gpio_set_inover(static_cast<uint>(sck_), GPIO_OVERRIDE_NORMAL);
  for (const int ch : {sck_, mosi_, miso_, cs_}) platformGpio(ch, kGpioInputFloating);
  sm_ = dma_rx_ = dma_tx_ = offset_ = -1;
  started_ = false;
  armed_ = arm_pending_ = false;
}

// Ready for the next CS: the state machine at its start (MISO undriven, X 0, Y 0xFFFFFFFF), the RX DMA into rx_, the tx
// (armed: tx_'s bytes; else nothing: X's 0 throughout)
void Rp2SpiTarget::restart() {
  const uint sm = static_cast<uint>(sm_);
  pio_sm_set_enabled(pio_, sm, false);
  dma_channel_abort(static_cast<uint>(dma_rx_));
  dma_channel_abort(static_cast<uint>(dma_tx_));
  pio_sm_clear_fifos(pio_, sm);
  pio_sm_restart(pio_, sm);
  pio_sm_exec(pio_, sm, pio_encode_set(pio_pindirs, 0));
  pio_sm_exec(pio_, sm, pio_encode_set(pio_x, 0));
  pio_sm_exec(pio_, sm, pio_encode_mov_not(pio_y, pio_null));
  pio_sm_exec(pio_, sm, pio_encode_jmp(static_cast<uint>(offset_)));
  const bool msb = bit_order_ == 0;
  dma_channel_config r = dma_channel_get_default_config(static_cast<uint>(dma_rx_));
  channel_config_set_transfer_data_size(&r, DMA_SIZE_8);
  channel_config_set_read_increment(&r, false);
  channel_config_set_write_increment(&r, true);
  channel_config_set_dreq(&r, pio_get_dreq(pio_, sm, false));
  // the byte the shift left it in: the low one (MSB first, shifted left) or the top one (LSB first, shifted right)
  const volatile uint8_t *from = reinterpret_cast<const volatile uint8_t *>(&pio_->rxf[sm]) + (msb ? 0 : 3);
  dma_channel_configure(static_cast<uint>(dma_rx_), &r, rx_, from, kRxRoom, true);
  if (armed_ && armed_count_) {
    dma_channel_config t = dma_channel_get_default_config(static_cast<uint>(dma_tx_));
    channel_config_set_transfer_data_size(&t, DMA_SIZE_8);   // a byte written to the FIFO fills every lane
    channel_config_set_read_increment(&t, true);
    channel_config_set_write_increment(&t, false);
    channel_config_set_dreq(&t, pio_get_dreq(pio_, sm, true));
    dma_channel_configure(static_cast<uint>(dma_tx_), &t, &pio_->txf[sm], tx_, armed_count_, true);
  }
  pio_sm_set_enabled(pio_, sm, true);
}

void Rp2SpiTarget::count(bool error) {
  if (transactions_ != 0xffffffffu) transactions_ = transactions_ + 1;
  if (error && errors_ != 0xffffffffu) errors_ = errors_ + 1;
}

// CS rose: the transfer ends. Its bits from Y, its full bytes from the RX DMA, the partial one from the ISR.
void Rp2SpiTarget::onCsRise() {
  if (!started_) return;
  const uint sm = static_cast<uint>(sm_);
  pio_sm_set_enabled(pio_, sm, false);
  pio_sm_exec(pio_, sm, pio_encode_set(pio_pindirs, 0));   // MISO let go
  for (uint32_t t = micros(); !pio_sm_is_rx_fifo_empty(pio_, sm) && micros() - t < 50;) {}   // the DMA takes the last
  const uint32_t full = kRxRoom - dma_channel_hw_addr(static_cast<uint>(dma_rx_))->transfer_count;
  dma_channel_abort(static_cast<uint>(dma_rx_));
  pio_sm_exec(pio_, sm, pio_encode_push(false, false));     // the partial byte (any)
  pio_sm_exec(pio_, sm, pio_encode_mov_not(pio_isr, pio_y));
  pio_sm_exec(pio_, sm, pio_encode_push(false, false));     // the bit count
  const uint32_t partial = pio_sm_is_rx_fifo_empty(pio_, sm) ? 0 : pio_sm_get(pio_, sm);
  const uint32_t bits = pio_sm_is_rx_fifo_empty(pio_, sm) ? 0 : pio_sm_get(pio_, sm);
  if (bits) {   // a CS with no clock in it is no transfer: the arm keeps waiting (fixture §4)
    const bool was_armed = armed_;
    if (!was_armed) {
      count(true);   // nobody armed it: MOSI dropped, counted
    } else {
      const uint32_t r = bits % 8, bytes = (bits + 7) / 8;
      if (r && full < kRxRoom) {   // the partial byte's bits where a full byte would have them, the rest 0
        const bool msb = bit_order_ == 0;
        rx_[full] = msb ? static_cast<uint8_t>((partial & ((1u << r) - 1)) << (8 - r)) : static_cast<uint8_t>(partial >> (32 - r));
      }
      const size_t data = bytes < armed_length_ ? bytes : armed_length_;
      const bool long_one = bytes > armed_length_, room = queue_count_ < kQueueDepth;
      if (room) {
        memcpy(queue_[queue_count_], rx_, data);
        queue_length_[queue_count_] = static_cast<uint8_t>(data);
        queue_bits_[queue_count_] = bits;
        queue_count_ = queue_count_ + 1;
      }
      count(long_one || !room);
      armed_ = false;
    }
  }
  if (arm_pending_) { arm_pending_ = false; armed_ = true; }
  restart();
}

Result Rp2SpiTarget::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (operation) {
    case kOpConfigure: {   // mode(u8) bit_order(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
      if (refused(parsed)) return parsed;
      if (payload[0] > 3 || payload[1] > 1) return rejected(kRejectMalformed);
      if (!(kModes >> payload[0] & 1)) return unsupportedValue(out, capacity);
      if (sck_ < 0) return wrongState(out, capacity);
      stop();
      mode_ = payload[0]; bit_order_ = payload[1];
      transactions_ = errors_ = 0;
      queue_count_ = 0;
      if (!start()) {   // no free state machine / DMA channel: state 0 (fixture §4)
        mode_ = bit_order_ = 0;
        return failed();
      }
      return completed();
    }
    case kOpArm: {   // length(u16) count(u16) tx [TLV]
      if (length < 4) return rejected(kRejectMalformed);
      const uint16_t want = getU16(payload), n = getU16(payload + 2);
      const Result parsed = plainTail(tail, payload, length, 4u + n, out, capacity);
      if (refused(parsed)) return parsed;
      if (want == 0 || n > want) return rejected(kRejectMalformed);
      if (want > kMaxFrame) return unsupportedValue(out, capacity);
      if (!started_ || armed_ || arm_pending_) return wrongState(out, capacity);
      noInterrupts();
      memcpy(tx_, payload + 4, n);
      memset(tx_ + n, 0, want - n);   // tx short of length: 0 (fixture §4)
      armed_length_ = want;
      armed_count_ = want;
      if (!gpio_get(static_cast<uint>(cs_))) {
        arm_pending_ = true;   // a transfer under way: this arm is the next one's
      } else {
        armed_ = true;
        restart();             // CS high: the tx loaded now
      }
      interrupts();
      return completed();
    }
    case kOpReadRx: {   // [TLV] -> pending(u8) bits(u32) count(u16) data
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!started_) return wrongState(out, capacity);
      noInterrupts();
      const size_t data = queue_count_ ? queue_length_[0] : 0;
      if (capacity < 7 + data) { interrupts(); return failed(); }
      out[0] = queue_count_ ? static_cast<uint8_t>(queue_count_ - 1) : 0;
      putU32(out + 1, queue_count_ ? queue_bits_[0] : 0);
      putU16(out + 5, static_cast<uint16_t>(data));
      if (data) memcpy(out + 7, queue_[0], data);
      if (queue_count_) {
        const uint8_t left = queue_count_ - 1;
        memmove(queue_[0], queue_[1], sizeof(queue_[0]) * left);
        memmove(queue_length_, queue_length_ + 1, left);
        memmove(queue_bits_, queue_bits_ + 1, sizeof(queue_bits_[0]) * left);
        queue_count_ = left;
      }
      interrupts();
      return completed(7 + data);
    }
    case kOpStatus: {   // [TLV] -> state mode bit_order armed queued (u8 each) transactions(u32) errors(u32)
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 13) return failed();
      out[0] = started_ ? 1 : 0;
      out[1] = mode_;
      out[2] = bit_order_;
      out[3] = (armed_ || arm_pending_) ? 1 : 0;
      out[4] = queue_count_;
      putU32(out + 5, transactions_);
      putU32(out + 9, errors_);
      return completed(13);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep

#endif
