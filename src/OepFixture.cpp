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
  return unavailable(out, capacity, cause, channel, 0xFFFF, cause ? reg::core::kHolderKindDisabled : 0, extra, sizeof extra);
}

}  // namespace

// ---- oep.fixture.gpio ----------------------------------------------------------------------------

size_t FixtureGpio::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.roleChannels(kGpioRoles, sizeof kGpioRoles, pins_.allowedMask());
  w.u32(gp::kTlvDescribeModes, kModes);   // modes 0-7 (u32 bit set)
  // drive_levels (fixture §1.1): default(u8) n(u8) n x ma(u16), the chip's levels (OepPlatform.h); none: not switched
  const DriveLevels d = platformDriveLevels();
  if (d.count) {
    uint8_t v[2 + 2 * 8];
    const uint8_t n = d.count < 8 ? d.count : 8;
    v[0] = d.default_level;
    v[1] = n;
    for (uint8_t i = 0; i < n; ++i) putU16(v + 2 + 2 * i, d.ma[i]);
    w.put(gp::kTlvDescribeDriveLevels, v, 2u + 2u * n);
  }
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
    case kOpSet: {   // n(u8) n x (channel u16, mode u8) [TLV 0x01 drive, repeated]: in order; nothing done if any
      // entry cannot be (fixture §1, core §4.3's order: a mode outside the table malformed, one not handled unsupported
      // with the channel and its position, a channel not planned unavailable with the same)
      const DriveLevels levels = platformDriveLevels();
      static const uint8_t kKnown[] = {gp::kTlvSetDrive};
      const size_t fixed = 1u + 3u * n;
      if (length < fixed) return rejected(kRejectMalformed);
      // drive (fixture §1.1) is known only where this chip declares drive_levels; elsewhere it is an unknown tag
      tail.repeats(kKnown);   // one drive TLV per element (fixture §1.1)
      const Result parsed = tail.parse(payload + fixed, length - fixed, kKnown, levels.count ? 1 : 0, out, capacity);
      if (refused(parsed)) return parsed;
      auto isOutput = [&](uint8_t i) {
        const uint8_t m = payload[3 + 3 * i];
        return m == gp::kModeOutputLow || m == gp::kModeOutputHigh;
      };
      // a drive on an element whose mode is undefined (8+) is not the contradiction "not mode 3 / 4": that mode is
      // refused unsupported below (core §4.3 "Contradictions and undefined values")
      auto undefinedMode = [&](uint8_t i) { return payload[3 + 3 * i] > gp::kModeInputPullupPulldown; };
      // drive TLVs, one per element: index n or more, an index twice, or an element not mode 3 / 4 is malformed (the
      // whole request); an undefined kind (2+) or a level this probe does not have ignores that TLV (listed in ignored;
      // unsupported when critical, fixture §1.1). Every form is checked first (pass 0), so a malformed one anywhere wins
      // over a critical one's unsupported (core §4.3).
      uint8_t drive[255];
      memset(drive, PinTable::kDriveDefault, n);
      for (int pass = 0; pass < 2; ++pass) {
        uint8_t seen[32] = {};
        size_t at = 0, len = 0;
        uint8_t raw = 0;
        const uint8_t *v = nullptr;
        while (tail.next(at, raw, v, len)) {
          if ((raw & ~kTagCritical) != gp::kTlvSetDrive) continue;
          // without drive_levels an unknown tag: listed by the parse once per TLV, no form checked (a critical one was
          // refused there)
          if (!levels.count) continue;
          if (pass == 0) {
            if (len != 4) return rejected(kRejectMalformed);
            const uint8_t index = v[0], kind = v[1];
            if (index >= n || ((seen[index / 8] >> (index % 8)) & 1) || (!isOutput(index) && !undefinedMode(index)))
              return rejected(kRejectMalformed);
            (void)kind;
            seen[index / 8] |= static_cast<uint8_t>(1u << (index % 8));
            continue;
          }
          uint8_t level = 0;
          if (undefinedMode(v[0])) continue;   // the request is refused for the mode
          if (PinTable::driveLevelOf(levels, v[1], getU16(v + 2), level)) drive[v[0]] = level;
          else if (raw & kTagCritical) return unsupportedTag(out, capacity, raw);   // critical: not ignored (core §2.3)
          else tail.ignore(gp::kTlvSetDrive);   // listed once per TLV ignored
        }
      }
      // a mode this probe does not declare, or an undefined one (8+: a later revision may define it, core §2.5): unsupported
      for (uint8_t i = 0; i < n; ++i)
        if (payload[3 + 3 * i] > gp::kModeInputPullupPulldown || !((kModes >> payload[3 + 3 * i]) & 1))
          return unsupportedAt(out, capacity, getU16(payload + 1 + 3 * i), i);
      for (uint8_t i = 0; i < n; ++i)
        if (!planned(getU16(payload + 1 + 3 * i))) return refusedAt(pins_, i, getU16(payload + 1 + 3 * i), out, capacity);
      // the strength of a mode 3 / 4 element: its drive, else its channel's idle drive, else the default level; kept
      // until the channel is set again
      for (uint8_t i = 0; i < n; ++i) {
        const uint16_t c = getU16(payload + 1 + 3 * i);
        pins_.setPad(static_cast<uint8_t>(c), platformMode(payload[3 + 3 * i]),
                     drive[i] != PinTable::kDriveDefault ? drive[i] : pins_.idleDrive(c));
      }
      return tail.finish(completed(), out, capacity);
    }
    case kOpRead: {   // n(u8) n x channel(u16) [TLV]  ->  n(u8) n x level(u8) [TLV 0x01 drive]
      const Result parsed = plainTail(tail, payload, length, 1u + 2u * n, out, capacity);
      if (refused(parsed)) return parsed;
      const bool levels = platformDriveLevels().count != 0;
      const size_t answer = 1u + n + (levels ? tlvSize(n) : 0);
      if (capacity < answer) return failed();
      for (uint8_t i = 0; i < n; ++i)
        if (!planned(getU16(payload + 1 + 2 * i))) return refusedAt(pins_, i, getU16(payload + 1 + 2 * i), out, capacity);
      out[0] = n;
      for (uint8_t i = 0; i < n; ++i) out[1 + i] = digitalRead(getU16(payload + 1 + 2 * i)) ? 1 : 0;
      if (levels) {   // drive (fixture §1.1): the level each channel is driven at in mode 3 / 4, 0xFF when not
        TlvWriter w(out + 1 + n, capacity - 1 - n);
        uint8_t v[255];
        for (uint8_t i = 0; i < n; ++i) v[i] = pins_.drivenLevel(getU16(payload + 1 + 2 * i));
        w.put(gp::kTlvReadAnswerDrive, v, n);
      }
      return tail.finish(completed(answer), out, capacity);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

// ---- oep.fixture.uart ----------------------------------------------------------------------------

size_t FixtureUart::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.roleChannels(&kUartRoles[0], 1, pins_.allowedMask() & rx_mask_);   // RX: the pins the peripheral can listen on
  w.roleChannels(&kUartRoles[1], 1, pins_.outputMask() & tx_mask_);   // TX: driven, no input-only pin
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
    const uint16_t c = roles[i].channel;
    const uint64_t role_mask = roles[i].role == ua::kRoleRx ? rx_mask_ : tx_mask_;
    if (roles[i].role == ua::kRoleRx && rx < 0) rx = c;
    else if (roles[i].role == ua::kRoleTx && tx < 0) tx = c;
    else return kRejectUnsupported;
    if (!pins_.allowed(c) || c > 63 || !((role_mask >> c) & 1)) return kRejectUnsupported;   // not a pin this role can take
    if (roles[i].role == ua::kRoleTx && !pins_.canOutput(c)) return kRejectUnsupported;     // TX on an input-only pin
  }
  if (rx >= 0 && tx >= 0 && rx == tx) return kRejectUnsupported;
  if ((rx >= 0 && !pins_.free(rx)) || (tx >= 0 && !pins_.free(tx)) || rx_ >= 0 || tx_ >= 0) return kRejectUnavailable;
  return 0;
}

// Park TX at the UART idle level: high through the pull-up first (pinMode(OUTPUT) alone starts low), then driven.
// A dip reaches the DUT as a framing-error byte that sits in its line buffer (2026-09-22, X035 testcmd). The level goes
// through platformGpio: on the RP2 a digitalWrite after INPUT_PULLUP leaves the latch low, and OUTPUT then drove a dip.
void FixtureUart::idleHigh() {
  if (tx_ < 0) return;
  pinMode(tx_, INPUT_PULLUP);
  platformGpio(tx_, kGpioOutputHigh);
}

bool FixtureUart::planApply(const RoleAssignment *roles, size_t count) {
  int rx = -1, tx = -1;
  for (size_t i = 0; i < count; ++i) (roles[i].role == ua::kRoleRx ? rx : tx) = roles[i].channel;
  if (rx >= 0 && !pins_.claim(rx, owner_)) return false;
  if (tx >= 0 && !pins_.claim(tx, owner_)) { pins_.release(owner_); return false; }
  rx_ = rx;
  tx_ = tx;
  if (rx_ >= 0) pins_.ownStrength(rx_);
  if (tx_ >= 0) pins_.ownStrength(tx_);
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

void FixtureUart::markReceiveErrors() {
  struct { volatile uint16_t &seen; uint16_t &marked; uint8_t detail; } kinds[] = {
      {rx_overflows_, marked_overflows_, reg::common::kMarkDetailLostOverflow},
      {rx_framing_, marked_framing_, reg::common::kMarkDetailLostFraming},
      {rx_parity_, marked_parity_, reg::common::kMarkDetailLostParity}};
  for (auto &k : kinds) {
    const uint16_t now = k.seen;
    if (now != k.marked) { stream_.mark(reg::common::kMarkKindLost, k.detail); k.marked = now; }
  }
}

void FixtureUart::poll() {
  if (!running_) return;
  uint8_t chunk[256];   // in chunks: a byte at a time through the UART driver capped a 2 Mbaud bridge near 88 kB/s (P7)
  for (int n; (n = serial_.available()) > 0;) {
    const size_t k = serial_.readBytes(chunk, static_cast<size_t>(n) < sizeof chunk ? static_cast<size_t>(n) : sizeof chunk);
    if (!k) break;
    for (size_t i = 0; i < k; ++i) stream_.put(chunk[i]);
  }
#if defined(ARDUINO_ARCH_RP2040)
  // arduino-pico's SerialUART reports a full receive queue and a break, read here after the bytes kept before them;
  // it drops a byte with a framing or parity error without telling (SerialUART::_handleIRQ, 6.1.1), so those leave no mark
  if (serial_.overflow()) ++rx_overflows_;
  if (serial_.getBreakReceived()) ++rx_framing_;   // a break is a framing error (as the ESP32's UART_BREAK_ERROR)
#endif
  markReceiveErrors();
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
#if defined(ARDUINO_ARCH_ESP32)
  serial_.onReceiveError([this](hardwareSerial_error_t e) {
    if (e == UART_FIFO_OVF_ERROR || e == UART_BUFFER_FULL_ERROR) ++rx_overflows_;
    else if (e == UART_FRAME_ERROR || e == UART_BREAK_ERROR) ++rx_framing_;
    else if (e == UART_PARITY_ERROR) ++rx_parity_;
  });
#endif
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
      // 1 = 2). 8N1 when absent. An undefined value or reserved bit is unsupported with the tag as received (a later
      // revision may define it, core §2.5); every defined one is declared here (fixture §2).
      uint8_t format = 0;
      size_t len = 0;
      bool format_critical = false;
      if (const uint8_t *f = tail.find(ua::kTlvConfigureFormat, len, &format_critical)) {
        if (len != 1) return rejected(kRejectMalformed);
        if (!formatDefined(f[0]))
          return unsupportedTag(out, capacity, ua::kTlvConfigureFormat | (format_critical ? kTagCritical : 0));
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
      const Result values = PositionStream::checkRead(payload, out, capacity);   // from 3's arg, from 4+ (common §1.2)
      if (refused(values)) return values;
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
