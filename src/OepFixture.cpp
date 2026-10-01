// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepFixture.h"

namespace oep {
namespace {

namespace gp = reg::fixture_gpio;
namespace ua = reg::fixture_uart;

// Wire mode (v1 table, 0-7) -> the platform's pin mode.
uint8_t platformMode(uint8_t mode) {
  static const uint8_t kMap[] = {kGpioInputFloating, kGpioInputPullUp, kGpioInputPullDown, kGpioOutputLow,
                                 kGpioOutputHigh, kGpioOpenDrainLow, kGpioOpenDrainRelease, kGpioInputPullUpDown};
  return kMap[mode];
}

// set / read refused for a channel not planned: unavailable with the channel and its position in the list (fixture §1:
// TLV 0x40 index after core §4.3's)
// A channel the settings disable says cause 5 (held by settings, probe.config §1).
Result refusedAt(const PinTable &pins, size_t index, uint16_t channel, uint8_t *out, size_t capacity) {
  const uint8_t extra[3] = {gp::kTlvUnavailablePayloadIndex, 1, static_cast<uint8_t>(index)};
  const uint8_t cause = pins.disabled(channel) ? reg::core::kUnavailableCauseHeldBySettings : 0;
  return unavailable(out, capacity, cause, channel, 0xFFFF, 0, extra, sizeof extra);
}

}  // namespace

// ---- oep.fixture.gpio ----------------------------------------------------------------------------

size_t FixtureGpio::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.roleChannels(kGpioRoles, sizeof kGpioRoles, pins_.allowedMask());
  w.u32(gp::kTlvDescribeModes, kModes);   // modes 0-7 (u32 bit set)
  return w.ok() ? w.length() : 0;
}

uint8_t FixtureGpio::planCheck(const RoleAssignment *roles, size_t count) {
  uint64_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    const uint16_t c = roles[i].channel;
    if (roles[i].role != gp::kRoleLine || !pins_.allowed(c) || ((seen >> c) & 1)) return kRejectUnsupported;
    if (!pins_.free(c)) return kRejectUnavailable;
    seen |= uint64_t{1} << c;
  }
  return 0;
}

bool FixtureGpio::planApply(const RoleAssignment *roles, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    if (!pins_.claim(roles[i].channel, owner_)) { planRelease(); return false; }
    planned_ |= uint64_t{1} << roles[i].channel;
  }
  return true;
}

void FixtureGpio::planRelease() {
  planned_ = 0;
  pins_.release(owner_);   // each channel to its idle state
}

Result FixtureGpio::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  if (length < 1) return rejected(kRejectMalformed);
  const uint8_t n = payload[0];
  switch (op) {
    case kOpSet: {   // n(u8) n x (channel u16, mode u8) [TLV]: in order; nothing done if any entry cannot be
      // (fixture §1, core §4.3's order: a mode outside the table malformed, one not handled unsupported with the channel
      // and its position, a channel not planned unavailable with the same)
      const Result parsed = plainTail(tail, payload, length, 1u + 3u * n, out, capacity);
      if (refused(parsed)) return parsed;
      for (uint8_t i = 0; i < n; ++i)
        if (payload[3 + 3 * i] > gp::kModeInputPullupPulldown) return rejected(kRejectMalformed);
      for (uint8_t i = 0; i < n; ++i)
        if (!((kModes >> payload[3 + 3 * i]) & 1)) return unsupportedAt(out, capacity, getU16(payload + 1 + 3 * i), i);
      for (uint8_t i = 0; i < n; ++i)
        if (!planned(getU16(payload + 1 + 3 * i))) return refusedAt(pins_, i, getU16(payload + 1 + 3 * i), out, capacity);
      for (uint8_t i = 0; i < n; ++i) platformGpio(getU16(payload + 1 + 3 * i), platformMode(payload[3 + 3 * i]));
      return tail.finish(completed(), out, capacity);
    }
    case kOpRead: {   // n(u8) n x channel(u16) [TLV]  ->  n(u8) n x level(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 1u + 2u * n, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 1u + n) return failed();
      for (uint8_t i = 0; i < n; ++i)
        if (!planned(getU16(payload + 1 + 2 * i))) return refusedAt(pins_, i, getU16(payload + 1 + 2 * i), out, capacity);
      out[0] = n;
      for (uint8_t i = 0; i < n; ++i) out[1 + i] = digitalRead(getU16(payload + 1 + 2 * i)) ? 1 : 0;
      return tail.finish(completed(1u + n), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

// ---- oep.fixture.uart ----------------------------------------------------------------------------

size_t FixtureUart::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.roleChannels(kUartRoles, sizeof kUartRoles, pins_.allowedMask());
  w.put(kImplementationPeripheral[0], kImplementationPeripheral + 2, kImplementationPeripheral[1]);
  // formats: data bits 8 / 7 (bits 0-1), parity none / even / odd (bits 2-3), stop 1 / 2 (bit 4) - every combination
  uint8_t formats[12], n = 0;
  for (uint8_t stop = 0; stop < 2; ++stop)
    for (uint8_t parity = 0; parity < 3; ++parity)
      for (uint8_t data = 0; data < 2; ++data) formats[n++] = static_cast<uint8_t>(data | parity << 2 | stop << 4);
  w.u8List(ua::kTlvDescribeFormats, formats, n);
  return w.ok() ? w.length() : 0;
}

bool FixtureUart::formatDefined(uint8_t format) {
  return (format & ua::kFormatFieldDataBitsMask) <= 1 && (format & ua::kFormatFieldParityMask) != ua::kFormatFieldParityMask &&
         (format & 0xe0) == 0;
}

uint8_t FixtureUart::planCheck(const RoleAssignment *roles, size_t count) {
  if (count == 0 || count > 2) return kRejectUnsupported;
  int rx = -1, tx = -1;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role == ua::kRoleRx && rx < 0) rx = roles[i].channel;
    else if (roles[i].role == ua::kRoleTx && tx < 0) tx = roles[i].channel;
    else return kRejectUnsupported;
    if (!pins_.allowed(roles[i].channel)) return kRejectUnsupported;
  }
  if (rx >= 0 && tx >= 0 && rx == tx) return kRejectUnsupported;
  if ((rx >= 0 && !pins_.free(rx)) || (tx >= 0 && !pins_.free(tx)) || rx_ >= 0 || tx_ >= 0) return kRejectUnavailable;
  return 0;
}

// Park TX at the UART idle level: high through the pull-up first (pinMode(OUTPUT) alone starts low), then driven.
// A dip reaches the DUT as a framing-error byte that sits in its line buffer (2026-09-22, X035 testcmd).
void FixtureUart::idleHigh() {
  if (tx_ < 0) return;
  pinMode(tx_, INPUT_PULLUP);
  digitalWrite(tx_, HIGH);
  pinMode(tx_, OUTPUT);
}

bool FixtureUart::planApply(const RoleAssignment *roles, size_t count) {
  int rx = -1, tx = -1;
  for (size_t i = 0; i < count; ++i) (roles[i].role == ua::kRoleRx ? rx : tx) = roles[i].channel;
  if (rx >= 0 && !pins_.claim(rx, owner_)) return false;
  if (tx >= 0 && !pins_.claim(tx, owner_)) { pins_.release(owner_); return false; }
  rx_ = rx;
  tx_ = tx;
  session_configured_ = false;
  if (rx_ >= 0) pinMode(rx_, INPUT);
  idleHigh();   // before the UART starts, too: the DUT's RX stays connected
  applySettings();   // the plan makes the stream: the UART runs from here (fixture §2)
  return true;
}

void FixtureUart::planRelease() {
  poll();
  if (running_) serial_.end();
  // Released pins go to their idle state (oep-core §8, default Hi-Z). A jig whose DUT RX must not float (2026-09-22,
  // X035 USART4 stopped answering after 0.5 s of a floating PB1) sets that pin's idle to pull-up.
  pins_.release(owner_);
  rx_ = tx_ = -1;
  running_ = false;
  session_configured_ = false;   // the next plan starts from the item (or the default) again
  configured_ = ua::kUartConfiguredDefault;
}

// The settings the UART starts with when its plan gives it pins (oep-if-probe-config §1): a session's configure since
// the plan, else the probe.config uart item, else 115200 8N1.
void FixtureUart::applySettings() {
  if (rx_ < 0 && tx_ < 0) return;
  if (session_configured_) {
    configured_ = ua::kUartConfiguredSession;
    begin(baud_, format_);
  } else if (item_set_) {   // the item; a baud the divider cannot make within 5 % falls back to the default (status 3)
    configured_ = begin(item_baud_, item_format_) ? ua::kUartConfiguredItem : ua::kUartConfiguredItemFallback;
    if (configured_ == ua::kUartConfiguredItemFallback) begin(kDefaultBaud, 0);
  } else {
    configured_ = ua::kUartConfiguredDefault;
    begin(kDefaultBaud, 0);
  }
}

void FixtureUart::setItem(uint32_t baud, uint8_t format) {
  item_set_ = true;
  item_baud_ = baud;
  item_format_ = format;
  if (!session_configured_) applySettings();
}

void FixtureUart::clearItem() {
  item_set_ = false;
  if (!session_configured_) applySettings();
}

void FixtureUart::poll() {
  if (!running_) return;
  uint8_t chunk[256];   // in chunks: a byte at a time through the UART driver capped a 2 Mbaud bridge near 88 kB/s (P7)
  for (int n; (n = serial_.available()) > 0;) {
    const size_t k = serial_.readBytes(chunk, static_cast<size_t>(n) < sizeof chunk ? static_cast<size_t>(n) : sizeof chunk);
    if (!k) break;
    for (size_t i = 0; i < k; ++i) stream_.put(chunk[i]);
  }
}

size_t FixtureUart::bindInput(const uint8_t *data, size_t length) {
  // only what the UART takes without waiting, so the RX side keeps being emptied (a blocking write let the UART's
  // receive buffer overflow while a 64 KiB echo was going out)
  if (!running_ || tx_ < 0) return 0;
  const int room = serial_.availableForWrite();
  if (room <= 0) return 0;
  return serial_.write(data, static_cast<size_t>(room) < length ? static_cast<size_t>(room) : length);
}

// The UART (re)started at baud / format. The stream and its positions stay (fixture §2: configure again keeps what was
// collected). false: the core refused the pins, or the rate came out more than 5 % off.
bool FixtureUart::begin(uint32_t baud, uint8_t format) {
  poll();
  if (running_) serial_.end();
  running_ = false;
  // A peer may echo while the probe is still writing; hold a full window of it.
  platformUartBuffers(serial_, 4096, 1024);
  idleHigh();
  const uint8_t data_bits = (format & ua::kFormatFieldDataBitsMask) ? 7 : 8;
  const uint8_t parity = (format & ua::kFormatFieldParityMask) >> 2;
  const uint8_t stop_bits = (format & ua::kFormatFieldStopBits2) ? 2 : 1;
  if (!platformUartBegin(serial_, baud, rx_, tx_, platformUartConfig(data_bits, parity, stop_bits))) return false;
  const uint32_t actual = platformUartBaud(serial_, baud);
  const uint64_t diff = actual > baud ? actual - baud : baud - actual;
  if (diff * 100 > static_cast<uint64_t>(baud) * 5) { serial_.end(); idleHigh(); return false; }
  baud_ = actual;
  format_ = format;
  running_ = true;
  return true;
}

Result FixtureUart::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case kOpConfigure: {   // baud(u32) [TLV 0x01 format, critical]  ->  baud(u32, the rate the UART runs at) [TLV]
      static const uint8_t kKnown[] = {ua::kTlvConfigureFormat};
      if (length < 4) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 4, length - 4, kKnown, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t baud = getU32(payload);
      // format: bits 0-1 data bits (0 = 8, 1 = 7), bits 2-3 parity (0 none, 1 even, 2 odd), bit 4 stop bits (0 = 1,
      // 1 = 2). 8N1 when absent. An undefined value is malformed; every defined one is declared here (fixture §2).
      uint8_t format = 0;
      size_t len = 0;
      if (const uint8_t *f = tail.find(ua::kTlvConfigureFormat, len)) {
        if (len != 1 || !formatDefined(f[0])) return rejected(kRejectMalformed);
        format = f[0];
      }
      if (!baudWithinReach(baud)) return unsupportedValue(out, capacity);   // a rate this UART cannot run (core §4.3)
      if (rx_ < 0 && tx_ < 0) return wrongState(out, capacity);               // no plan: no pins
      if (capacity < 4) return failed();
      if (!begin(baud, format)) {   // the rate came out more than 5 % off: not a rate this UART runs
        applySettings();
        return unsupportedValue(out, capacity);
      }
      session_configured_ = true;
      configured_ = ua::kUartConfiguredSession;
      putU32(out, baud_);
      return tail.finish(completed(4), out, capacity);
    }
    case kOpStatus: {   // [TLV]  ->  configured(u8: uart_configured) baud(u32) format(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 6) return failed();
      out[0] = running_ ? configured_ : ua::kUartConfiguredDefault;
      putU32(out + 1, running_ ? baud_ : 0);
      out[5] = running_ ? format_ : 0;
      return tail.finish(completed(6), out, capacity);
    }
    case kOpRead: {   // from(u8) arg(u64) max(u16) [TLV]  ->  start(u64) flags(u8) len(u16) data [TLV]
      const Result parsed = plainTail(tail, payload, length, PositionStream::kReadRequest, out, capacity);
      if (refused(parsed)) return parsed;
      if (payload[0] > reg::common::kReadFromLastMark) return rejected(kRejectMalformed);
      poll();
      const size_t reserve = tail.anyIgnored() ? 2 + Tail::kMaxIgnored : 0;
      return tail.finish(stream_.read(payload, out, capacity, max_read_, reserve), out, capacity);
    }
    case kOpMarks: {   // from_serial(u32) [TLV]  ->  more(u8) count(u8) entries [TLV]
      const Result parsed = plainTail(tail, payload, length, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 2) return failed();
      poll();
      const size_t room = tail.anyIgnored() && capacity > 2 + Tail::kMaxIgnored ? capacity - 2 - Tail::kMaxIgnored : capacity;
      return tail.finish(completed(stream_.marks(getU32(payload), out, room)), out, capacity);
    }
    case kOpClear: {
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      poll();
      stream_.clear();
      return tail.finish(completed(), out, capacity);
    }
    case kOpMark: {   // value(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 1, out, capacity);
      if (refused(parsed)) return parsed;
      poll();
      stream_.mark(reg::common::kMarkKindHost, payload[0]);
      return tail.finish(completed(), out, capacity);
    }
    case kOpWrite: {   // count(u16) data [TLV]  ->  accepted(u16) [TLV]: what the UART's send buffer took now
      if (length < 2) return rejected(kRejectMalformed);
      const uint16_t count = getU16(payload);
      const Result parsed = plainTail(tail, payload, length, 2u + count, out, capacity);
      if (refused(parsed)) return parsed;
      if (count == 0) return rejected(kRejectMalformed);
      if (!running_ || tx_ < 0) return wrongState(out, capacity);
      if (capacity < 2) return failed();
      const int room = serial_.availableForWrite();
      const size_t written = room > 0 ? serial_.write(payload + 2, static_cast<size_t>(room) < count ? static_cast<size_t>(room) : count) : 0;
      putU16(out, static_cast<uint16_t>(written));
      const Result r = written == count ? completed(2) : written ? partial(2) : failed(2);
      return tail.finish(r, out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep
