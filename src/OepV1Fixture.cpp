#include "OepV1Fixture.h"

namespace oep {
namespace v1 {
namespace {

inline bool refused(const Result &r) { return r.resolution != kResolutionCompleted; }

// Wire mode (v1 table, 0-6) -> the platform's pin mode.
uint8_t platformMode(uint8_t mode) {
  static const uint8_t kMap[] = {kGpioInputFloating, kGpioInputPullUp, kGpioInputPullDown, kGpioOutputLow,
                                 kGpioOutputHigh, kGpioOpenDrainLow, kGpioOpenDrainRelease};
  return kMap[mode];
}

}  // namespace

// ---- oep.fixture.gpio ----------------------------------------------------------------------------

size_t FixtureGpio::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.roleChannels(kGpioRoles, sizeof kGpioRoles, pins_.allowedMask());
  return w.ok() ? w.length() : 0;
}

uint8_t FixtureGpio::planCheck(const RoleAssignment *roles, size_t count) {
  uint64_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    const uint16_t c = roles[i].channel;
    if (roles[i].role != reg::fixture_gpio::kRoleLine || !pins_.free(c) || ((seen >> c) & 1)) return kRejectUnavailable;
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
  for (uint8_t c = 0; c < 64; ++c)
    if (planned(c)) platformGpio(c, kGpioInputFloating);
  planned_ = 0;
  pins_.release(owner_);
}

Result FixtureGpio::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  if (length < 1) return rejected(kRejectMalformed);
  const uint8_t n = payload[0];
  switch (op) {
    case kOpSet: {   // n(u8) n x (channel u16, mode u8) [TLV]: in order; nothing done if any entry cannot be
      // (a channel not planned or a mode not handled: unavailable, payload = its position)
      const Result parsed = plainTail(tail, payload, length, 1u + 3u * n, out, capacity);
      if (refused(parsed)) return parsed;
      for (uint8_t i = 0; i < n; ++i)
        if (!planned(getU16(payload + 1 + 3 * i)) || payload[3 + 3 * i] > reg::fixture_gpio::kModeOpenDrainRelease)
          return rejectedWith(kRejectUnavailable, out, capacity, i);
      for (uint8_t i = 0; i < n; ++i) platformGpio(getU16(payload + 1 + 3 * i), platformMode(payload[3 + 3 * i]));
      return tail.finish(completed(), out, capacity);
    }
    case kOpRead: {   // n(u8) n x channel(u16) [TLV]  ->  n x level(u8)
      const Result parsed = plainTail(tail, payload, length, 1u + 2u * n, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < n) return failed();
      for (uint8_t i = 0; i < n; ++i)
        if (!planned(getU16(payload + 1 + 2 * i))) return rejectedWith(kRejectUnavailable, out, capacity, i);
      for (uint8_t i = 0; i < n; ++i) out[i] = digitalRead(getU16(payload + 1 + 2 * i)) ? 1 : 0;
      return tail.finish(completed(n), out, capacity);
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
  return w.ok() ? w.length() : 0;
}

uint8_t FixtureUart::planCheck(const RoleAssignment *roles, size_t count) {
  if (count != 2) return kRejectUnavailable;
  int rx = -1, tx = -1;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role == reg::fixture_uart::kRoleRx) rx = roles[i].channel;
    else if (roles[i].role == reg::fixture_uart::kRoleTx) tx = roles[i].channel;
    else return kRejectUnavailable;
  }
  if (rx < 0 || tx < 0 || rx == tx) return kRejectUnavailable;
  if (!pins_.free(rx) || !pins_.free(tx) || rx_ >= 0) return kRejectUnavailable;
  return 0;
}

// Park TX at the UART idle level: high through the pull-up first (pinMode(OUTPUT) alone starts low), then driven.
// A dip reaches the DUT as a framing-error byte that sits in its line buffer (2026-09-22, X035 testcmd).
void FixtureUart::idleHigh() {
  pinMode(tx_, INPUT_PULLUP);
  digitalWrite(tx_, HIGH);
  pinMode(tx_, OUTPUT);
}

bool FixtureUart::planApply(const RoleAssignment *roles, size_t count) {
  int rx = -1, tx = -1;
  for (size_t i = 0; i < count; ++i) (roles[i].role == reg::fixture_uart::kRoleRx ? rx : tx) = roles[i].channel;
  if (!pins_.claim(rx, owner_)) return false;
  if (!pins_.claim(tx, owner_)) { pins_.release(owner_); return false; }
  rx_ = rx;
  tx_ = tx;
  configured_ = false;
  pinMode(rx_, INPUT);
  idleHigh();   // before configure, too: the DUT's RX stays connected
  return true;
}

void FixtureUart::planRelease() {
  poll();
  if (configured_) serial_.end();
  if (rx_ >= 0) pinMode(rx_, INPUT);
  // The DUT's RX stays connected: leave the line at UART idle (high) instead of floating, or the DUT's command
  // parser sees noise (2026-09-22, X035 USART4 stopped answering after 0.5 s of a floating PB1).
  if (tx_ >= 0) pinMode(tx_, INPUT_PULLUP);
  pins_.release(owner_);
  rx_ = tx_ = -1;
  configured_ = false;
}

void FixtureUart::poll() {
  if (!configured_) return;
  for (int n = serial_.available(); n > 0; --n) {
    const int c = serial_.read();
    if (c < 0) break;
    stream_.put(static_cast<uint8_t>(c));
  }
}

Result FixtureUart::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case kOpConfigure: {   // baud(u32) [TLV 0x01 format]  ->  baud(u32, the rate the UART runs at)
      static const uint8_t kKnown[] = {reg::fixture_uart::kTlvConfigureFormat};
      if (length < 4) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 4, length - 4, kKnown, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t baud = getU32(payload);
      if (baud < 1200 || baud > 2000000) return rejected(kRejectMalformed);
      // format: bits 0-1 data bits (0 = 8, 1 = 7), bits 2-3 parity (0 none, 1 even, 2 odd), bit 4 stop bits (0 = 1,
      // 1 = 2). 8N1 when absent, or when a value this probe does not know came without the critical bit.
      uint8_t data_bits = 8, parity = 0, stop_bits = 1, len = 0;
      bool critical = false;
      if (const uint8_t *f = tail.find(reg::fixture_uart::kTlvConfigureFormat, len, &critical)) {
        if (len != 1) return rejected(kRejectMalformed);
        if ((f[0] & 3) > 1 || ((f[0] >> 2) & 3) > 2 || (f[0] & 0xe0)) {
          const Result r = tail.refuse(reg::fixture_uart::kTlvConfigureFormat, critical, out, capacity);
          if (refused(r)) return r;
        } else {
          data_bits = (f[0] & 3) ? 7 : 8;
          parity = (f[0] >> 2) & 3;
          stop_bits = (f[0] & 0x10) ? 2 : 1;
        }
      }
      if (rx_ < 0) return rejected(kRejectUnavailable);   // no plan
      if (capacity < 4) return failed();
      poll();
      if (configured_) serial_.end();
      // A peer may echo while the probe is still writing; hold a full window of it.
      platformUartBuffers(serial_, 4096, 1024);
      idleHigh();
      if (!platformUartBegin(serial_, baud, rx_, tx_, platformUartConfig(data_bits, parity, stop_bits))) {
        configured_ = false;
        return failed();
      }
      configured_ = true;
      putU32(out, platformUartBaud(serial_, baud));
      return tail.finish(completed(4), out, capacity);
    }
    case kOpRead: {   // from(u8) arg(u32) max(u16) [TLV]  ->  start(u32) flags(u8) data (closed tail)
      const Result parsed = plainTail(tail, payload, length, 7, out, capacity);
      if (refused(parsed)) return parsed;
      if (payload[0] > reg::target_console::kReadFromLastMark) return rejected(kRejectUnsupported);
      poll();
      return stream_.read(payload, out, capacity, max_read_);
    }
    case kOpMarks: {   // from_serial(u32) [TLV]  ->  more(u8) count(u8) entries
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
      stream_.mark(reg::target_console::kMarkKindHost, payload[0]);
      return tail.finish(completed(), out, capacity);
    }
    case kOpWrite: {   // count(u16) data [TLV]  ->  accepted(u16)
      if (length < 2) return rejected(kRejectMalformed);
      const uint16_t count = getU16(payload);
      const Result parsed = plainTail(tail, payload, length, 2u + count, out, capacity);
      if (refused(parsed)) return parsed;
      if (!configured_) return rejected(kRejectUnavailable);
      if (capacity < 2) return failed();
      const size_t written = serial_.write(payload + 2, count);
      serial_.flush();
      putU16(out, static_cast<uint16_t>(written));
      const Result r = written == count ? completed(2) : written ? partial(2) : failed(2);
      return tail.finish(r, out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace v1
}  // namespace oep
