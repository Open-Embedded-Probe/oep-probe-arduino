// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepCh32SpiTarget.h"

#if (defined(ARDUINO_ARCH_CH32RV) && !defined(ARDUINO_CH32X035)) || defined(OEP_HOST_FAKE_CH32_SPI_SLAVE)

#include <string.h>

namespace oep {

uint8_t Ch32SpiTarget::planCheck(const RoleAssignment *roles, size_t count) {
  // roles 1 to 4 exactly once each, on four channels (fixture §4): another set is unsupported (oep-if-plan §2.4)
  if (count != 4) return kRejectUnsupported;
  int pin[5] = {-1, -1, -1, -1, -1};
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role < kRoleSck || roles[i].role > kRoleCs || pin[roles[i].role] >= 0) return kRejectUnsupported;
    pin[roles[i].role] = roles[i].channel;
  }
  for (int r = kRoleSck; r <= kRoleCs; ++r) {
    for (int q = r + 1; q <= kRoleCs; ++q) if (pin[r] == pin[q]) return kRejectUnsupported;
  }
  for (int r = kRoleSck; r <= kRoleCs; ++r) if (!pins_.allowed(pin[r])) return kRejectUnsupported;
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
  if (sck_ >= 0 && started_) return kRejectUnavailable;   // running (state 1): no change, cause 6 (fixture §4)
  return 0;
}

bool Ch32SpiTarget::planApply(const RoleAssignment *roles, size_t count) {
  int pin[5] = {-1, -1, -1, -1, -1};
  for (size_t i = 0; i < count; ++i) pin[roles[i].role] = roles[i].channel;
  for (int r = kRoleSck; r <= kRoleCs; ++r) {
    if (!pins_.claim(pin[r], kOwnerId)) { pins_.release(kOwnerId); return false; }
  }
  sck_ = pin[kRoleSck]; mosi_ = pin[kRoleMosi]; miso_ = pin[kRoleMiso]; cs_ = pin[kRoleCs];
  for (int r = kRoleSck; r <= kRoleCs; ++r) pins_.ownStrength(static_cast<uint16_t>(pin[r]));
  return true;
}

// Releasing or replacing the plan: back to the state right after describe (fixture §4).
void Ch32SpiTarget::planRelease() {
  stop();
  pins_.release(kOwnerId);
  sck_ = mosi_ = miso_ = cs_ = -1;
  mode_ = bit_order_ = 0;
  transactions_ = errors_ = 0;
  queue_count_ = 0;
}

size_t Ch32SpiTarget::describe(uint8_t *out, size_t capacity) {
  static const uint8_t kInputs[] = {kRoleSck, kRoleMosi, kRoleCs}, kMiso[] = {kRoleMiso};
  namespace spi = reg::fixture_spi_target;
  TlvWriter w(out, capacity);
  w.roleChannels(kInputs, sizeof kInputs, pins_.allowedMask());
  w.roleChannels(kMiso, sizeof kMiso, pins_.outputMask());
  w.u16(kTagMaxLength, kMaxFrame);
  w.u32(kTagMaxClockHz, kMaxClockHz);
  w.u32(kTagFeatures, spi::kFeaturesLsbFirst);
  w.u8(spi::kTlvDescribeQueueDepth, kQueueDepth);
  w.u8(spi::kTlvDescribeModes, kModes);
  w.u32(spi::kTlvDescribeCsSetupNs, kCsSetupNs);
  return w.ok() ? w.length() : 0;
}

bool Ch32SpiTarget::start() {
  static const uint8_t kSpiModes[] = {SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3};
  if (!slave_.begin(platformPad(sck_), platformPad(miso_), platformPad(mosi_), platformPad(cs_), kSpiModes[mode_],
                    bit_order_ ? LSBFIRST : MSBFIRST))
    return false;
  slave_.setFill(0);   // MISO 0 outside tx (fixture §4)
  missed_ = slave_.missed();
  started_ = true;
  return true;
}

void Ch32SpiTarget::stop() {
  if (started_) slave_.end();   // the pins and the SPI back; what was queued dropped
  started_ = false;
  armed_ = false;
}

void Ch32SpiTarget::count(uint32_t n, bool error) {
  transactions_ = transactions_ > 0xffffffffu - n ? 0xffffffffu : transactions_ + n;
  if (error) errors_ = errors_ > 0xffffffffu - n ? 0xffffffffu : errors_ + n;
}

void Ch32SpiTarget::service() {
  if (!started_) return;
  CH32RVSPISlave::Result r;
  while (slave_.poll(r)) {
    if (r.rx != rx_ || !armed_) continue;
    armed_ = false;
    const size_t data = r.length < armed_length_ ? r.length : armed_length_;
    const bool long_one = r.length > armed_length_, full = queue_count_ == kQueueDepth;
    if (!full) {
      memcpy(queue_[queue_count_], rx_, data);
      queue_length_[queue_count_] = static_cast<uint8_t>(data);
      queue_bits_[queue_count_] = r.length > 0xffffffffu / 8 ? 0xffffffffu : static_cast<uint32_t>(r.length * 8);
      ++queue_count_;
    }
    count(1, long_one || full);
  }
  const uint32_t missed = slave_.missed();   // transfers nobody armed: MOSI dropped, counted (fixture §4)
  if (missed != missed_) { count(missed - missed_, true); missed_ = missed; }
}

Result Ch32SpiTarget::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (operation) {
    case kOpConfigure: {   // mode(u8) bit_order(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 2, out, capacity);
      if (refused(parsed)) return parsed;
      if (payload[0] > 3 || payload[1] > 1) return rejected(kRejectMalformed);
      if (!(kModes >> payload[0] & 1)) return unsupportedValue(out, capacity);
      if (sck_ < 0) return wrongState(out, capacity);   // needs a plan (fixture §4: unavailable cause 6)
      stop();
      mode_ = payload[0]; bit_order_ = payload[1];
      transactions_ = errors_ = 0;
      queue_count_ = 0;
      if (!start()) {   // the pins no SPI's route, or that SPI in use: state 0 (fixture §4)
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
      service();   // a transfer that ended before this arm: no longer armed, and counted before it
      if (!started_ || armed_) return wrongState(out, capacity);
      memcpy(tx_, payload + 4, n);
      memset(tx_ + n, 0, want - n);   // tx short of length: 0 (fixture §4)
      if (!slave_.queue(tx_, rx_, want)) return failed();
      armed_ = true;
      armed_length_ = want;
      return completed();
    }
    case kOpReadRx: {   // [TLV] -> pending(u8) bits(u32) count(u16) data
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (!started_) return wrongState(out, capacity);
      service();
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
      return completed(7 + data);
    }
    case kOpStatus: {   // [TLV] -> state mode bit_order armed queued (u8 each) transactions(u32) errors(u32)
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 13) return failed();
      service();
      out[0] = started_ ? 1 : 0;
      out[1] = mode_;
      out[2] = bit_order_;
      out[3] = armed_ ? 1 : 0;
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
