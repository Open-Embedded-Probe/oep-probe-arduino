#include "OepFixtureServices.h"

#include "OepTlv.h"

namespace oep {

// ---- fixture.gpio --------------------------------------------------------
Result FixtureGpio::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (operation) {
    case OEP_V0_FIXTURE_GPIO_OP_CONFIGURE: {
      struct oep_v0_fixture_gpio_configure_request request;
      if (!oep_v0_fixture_gpio_configure_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!pins_.allowed(request.channel) || request.mode > kGpioOpenDrainRelease) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (pins_.owner(request.channel) != 0 && pins_.owner(request.channel) != kOwner) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      platformGpio(request.channel, request.mode);
      configured_ |= uint64_t{1} << request.channel;
      return completed();
    }
    case OEP_V0_FIXTURE_GPIO_OP_READ_BANK: {
      struct oep_v0_fixture_gpio_read_bank_request request;
      if (!oep_v0_fixture_gpio_read_bank_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      struct oep_v0_fixture_gpio_read_bank_result result = {pins_.allowedMask(), 0};
      for (uint8_t c = 0; c < PinTable::kChannels; ++c)
        if (pins_.allowed(c) && digitalRead(c)) result.values |= uint64_t{1} << c;
      return completed(oep_v0_fixture_gpio_read_bank_result_pack(&result, out, capacity));
    }
    default:
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

size_t FixtureGpio::describe(uint8_t first, uint8_t *out, size_t capacity) {
  size_t used = 0, index = 0;
  for (uint8_t c = 0; c < PinTable::kChannels; ++c) {
    if (!pins_.allowed(c)) continue;
    if (index++ < first) continue;
    const size_t n = tlvPutU16(out, capacity, used, OEP_V0_TLV_CORE_CHANNEL_CANDIDATE, c);
    if (!n) break;
    used = n;
  }
  return used;
}

void FixtureGpio::abandon() {
  for (uint8_t c = 0; c < PinTable::kChannels; ++c)
    if ((configured_ >> c) & 1) pinMode(c, INPUT);
  configured_ = 0;
}

// ---- fixture.uart --------------------------------------------------------
uint8_t FixtureUart::planCheck(const RoleAssignment *roles, size_t count) {
  if (count != 2) return OEP_V0_REJECT_MALFORMED_PAYLOAD;
  int rx = -1, tx = -1;
  for (size_t i = 0; i < count; ++i) {
    if (roles[i].role == kRoleRx) rx = roles[i].channel;
    else if (roles[i].role == kRoleTx) tx = roles[i].channel;
    else return OEP_V0_REJECT_MALFORMED_PAYLOAD;
  }
  if (rx < 0 || tx < 0 || rx == tx) return OEP_V0_REJECT_MALFORMED_PAYLOAD;
  if (!pins_.free(rx) || !pins_.free(tx)) return OEP_V0_REJECT_UNAVAILABLE;
  if (rx_ >= 0) return OEP_V0_REJECT_UNAVAILABLE;  // already leased
  return 0;
}

bool FixtureUart::planApply(const RoleAssignment *roles, size_t count) {
  int rx = -1, tx = -1;
  for (size_t i = 0; i < count; ++i) (roles[i].role == kRoleRx ? rx : tx) = roles[i].channel;
  if (!pins_.claim(rx, kOwner)) return false;
  if (!pins_.claim(tx, kOwner)) { pins_.release(kOwner); return false; }
  rx_ = rx; tx_ = tx; configured_ = false;
  return true;
}

void FixtureUart::planRelease() {
  if (configured_) serial_.end();
  if (rx_ >= 0) pinMode(rx_, INPUT);
  // The DUT's RX stays connected: leave the line at UART idle (high) instead of
  // floating, or the DUT's command parser sees noise (2026-09-22, X035 USART4 stopped
  // answering after 0.5 s of a floating PB1).
  if (tx_ >= 0) pinMode(tx_, INPUT_PULLUP);
  pins_.release(kOwner);
  rx_ = tx_ = -1;
  configured_ = false;
}

Result FixtureUart::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (operation) {
    case OEP_V0_FIXTURE_UART_OP_CONFIGURE: {
      struct oep_v0_fixture_uart_configure_request request;
      if (!oep_v0_fixture_uart_configure_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (request.baud < 1200 || request.baud > 2000000) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (rx_ < 0) return rejected(OEP_V0_REJECT_UNAVAILABLE);  // no lease
      if (configured_) serial_.end();
      // A peer may echo while the probe is still writing; hold a full window of it.
      platformUartBuffers(serial_, 4096, 1024);
      // Park TX at the UART idle level before the peripheral takes the pin: begin()
      // otherwise lets the line dip and the DUT receives a framing-error byte that
      // sits in its line buffer until the next newline (2026-09-22, X035 testcmd).
      pinMode(tx_, INPUT_PULLUP);   // high through the pull-up first: pinMode(OUTPUT) alone starts low
      digitalWrite(tx_, HIGH);
      pinMode(tx_, OUTPUT);
      if (!platformUartBegin(serial_, request.baud, rx_, tx_)) return failed();
      configured_ = true;
      struct oep_v0_fixture_uart_configure_result result = {request.baud};
      return completed(oep_v0_fixture_uart_configure_result_pack(&result, out, capacity));
    }
    case OEP_V0_FIXTURE_UART_OP_WRITE: {
      struct oep_v0_fixture_uart_write_request request;
      if (!oep_v0_fixture_uart_write_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!configured_) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      const size_t written = serial_.write(request.data, request.data_length);
      serial_.flush();
      struct oep_v0_fixture_uart_write_result result = {static_cast<uint16_t>(written)};
      const size_t n = oep_v0_fixture_uart_write_result_pack(&result, out, capacity);
      return written == request.data_length ? completed(n) : failed(n);
    }
    case OEP_V0_FIXTURE_UART_OP_READ: {
      struct oep_v0_fixture_uart_read_request request;
      if (!oep_v0_fixture_uart_read_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!configured_) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      size_t n = 0;
      const size_t limit = request.maximum < capacity ? request.maximum : capacity;
      while (n < limit && serial_.available()) out[n++] = static_cast<uint8_t>(serial_.read());
      return completed(n);
    }
    default:
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

size_t FixtureUart::describe(uint8_t first, uint8_t *out, size_t capacity) {
  size_t used = 0, index = 0;
  for (uint8_t c = 0; c < PinTable::kChannels; ++c) {
    if (!pins_.allowed(c)) continue;
    if (index++ < first) continue;
    const size_t n = tlvPutU16(out, capacity, used, OEP_V0_TLV_CORE_CHANNEL_CANDIDATE, c);
    if (!n) break;
    used = n;
  }
  return used;
}

}  // namespace oep
