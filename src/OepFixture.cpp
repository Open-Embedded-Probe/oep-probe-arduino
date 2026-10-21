// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepFixture.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/uart.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "esp32-hal-uart.h"
#endif

namespace oep {
namespace {

namespace gp = reg::fixture_gpio;
namespace ua = reg::fixture_uart;

// Wire mode (v1 table, 0-6) -> the platform's pin mode.
uint8_t platformMode(uint8_t mode) {
  static const uint8_t kMap[] = {kGpioInputFloating, kGpioInputPullUp, kGpioInputPullDown, kGpioOutputLow,
                                 kGpioOutputHigh, kGpioOpenDrainLow, kGpioOpenDrainRelease};
  return kMap[mode];
}

// set / read refused for a channel not planned: unavailable with the channel and its position in the list (fixture §1:
// TLV 0x40 index after core §4.3's)
// A channel the settings disable says cause 5 (held by settings, probe.config §1).
Result refusedAt(const PinTable &pins, size_t index, uint16_t channel, uint8_t *out, size_t capacity) {
  const uint8_t extra[4] = {gp::kTlvUnavailablePayloadIndex, 1, 0, static_cast<uint8_t>(index)};   // tag len(u16) index
  const uint8_t cause = pins.disabled(channel) ? reg::core::kUnavailableCauseHeldBySettings : 0;
  return unavailable(out, capacity, cause, channel, extra, sizeof extra);
}

}  // namespace

// ---- oep.fixture.gpio ----------------------------------------------------------------------------

size_t FixtureGpio::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.roleChannels(kGpioRoles, sizeof kGpioRoles, pins_.allowedMask());
  w.u32(gp::kTlvDescribeModes, kModes);   // modes 0-6 (u32 bit set)
  // the output strength is the probe's (fixture §1): not declared
  return w.ok() ? w.length() : 0;
}

uint8_t FixtureGpio::planCheck(const RoleAssignment *roles, size_t count) {
  uint64_t seen = 0;
  for (size_t i = 0; i < count; ++i) {
    const uint16_t c = roles[i].channel;
    if (roles[i].role != gp::kRoleLine || !pins_.allowed(c) || ((seen >> c) & 1)) return kRejectUnsupported;
    if (!pins_.freeFor(c, owner_)) return kRejectUnavailable;
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
    case kOpSet: {   // n(u8) n x (channel u16, mode u8) [TLV]: in order; nothing done if any entry cannot be (fixture
      // §1, core §4.3: a malformed request, then a mode this probe does not handle unsupported, then a channel not
      // planned unavailable with the channel and its position). The output strength is the probe's (fixture §1).
      const size_t fixed = 1u + 3u * n;
      if (length < fixed) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + fixed, length - fixed, out, capacity);
      if (refused(parsed)) return parsed;
      // a mode this probe does not declare, or one the table leaves unused (7+): unsupported with the channel and index
      for (uint8_t i = 0; i < n; ++i)
        if (payload[3 + 3 * i] > gp::kModeOpenDrainRelease || !((kModes >> payload[3 + 3 * i]) & 1))
          return unsupportedAt(out, capacity, getU16(payload + 1 + 3 * i), i);
      for (uint8_t i = 0; i < n; ++i)
        if (!planned(getU16(payload + 1 + 3 * i))) return refusedAt(pins_, i, getU16(payload + 1 + 3 * i), out, capacity);
      // a mode 3 / 4 element at its channel's idle strength, else the default level (the probe's choice)
      for (uint8_t i = 0; i < n; ++i) {
        const uint16_t c = getU16(payload + 1 + 3 * i);
        pins_.setPad(static_cast<uint8_t>(c), platformMode(payload[3 + 3 * i]), pins_.idleDrive(c));
      }
      return completed();
    }
    case kOpRead: {   // n(u8) n x channel(u16) [TLV]  ->  n(u8) n x level(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 1u + 2u * n, out, capacity);
      if (refused(parsed)) return parsed;
      const size_t answer = 1u + n;
      if (capacity < answer) return failed();
      for (uint8_t i = 0; i < n; ++i)
        if (!planned(getU16(payload + 1 + 2 * i))) return refusedAt(pins_, i, getU16(payload + 1 + 2 * i), out, capacity);
      out[0] = n;
      for (uint8_t i = 0; i < n; ++i) out[1 + i] = digitalRead(platformPad(getU16(payload + 1 + 2 * i))) ? 1 : 0;
      return completed(answer);
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
  // formats: data bits 8 / 7 (bits 0-1), parity none / even / odd (bits 2-3), stop 1 / 2 (bit 4) - every combination
  uint8_t formats[12], n = 0;
  for (uint8_t stop = 0; stop < 2; ++stop)
    for (uint8_t parity = 0; parity < 3; ++parity)
      for (uint8_t data = 0; data < 2; ++data) {
        const uint8_t f = static_cast<uint8_t>(data | parity << 2 | stop << 4);
        if (formatDeclared(f)) formats[n++] = f;
      }
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
  refusal_cause_ = reg::core::kUnavailableCauseWrongState;
  refusal_channel_ = 0xffff;
  // a channel whose idle is an output: the UART does not take it (core §8: no interface here says it does), cause 5
  const int planned[2] = {rx, tx};
  for (int c : planned) {
    const uint8_t idle = c >= 0 ? pins_.idle(static_cast<uint16_t>(c)) : PinTable::kIdleUnset;
    if (idle == PinTable::kIdleOutputLow || idle == PinTable::kIdleOutputHigh) {
      refusal_cause_ = reg::core::kUnavailableCauseHeldBySettings;
      refusal_channel_ = static_cast<uint16_t>(c);
      return kRejectUnavailable;
    }
  }
  if ((rx_ >= 0 || tx_ >= 0) && session_configured_ && !pass_sink_) return kRejectUnavailable;   // configured: no change (cause 6)
  auto mine = [&](int c) { return c >= 0 && pins_.freeFor(static_cast<uint16_t>(c), owner_); };
  if (pass_sink_ && !session_configured_) { /* the pass-through gives way at planApply */ }
  if ((rx >= 0 && !mine(rx)) || (tx >= 0 && !mine(tx))) return kRejectUnavailable;
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

// probe.config §1.1: the UART on `rx` alone (TX unused, nothing driven), its bytes to `sink` - while no plan of this fn is
// there; the pin is not claimed (a session's plan or connection takes it at once: passStop first).
bool FixtureUart::passStart(uint16_t rx, uint32_t baud, uint8_t format, PassSink sink, void *context) {
  if (rx_ >= 0 || tx_ >= 0) return false;
  rx_ = rx;
  pass_sink_ = sink;
  pass_ctx_ = context;
  pinMode(rx_, INPUT);
  if (!begin(baud, format)) { passStop(); return false; }
  return true;
}

void FixtureUart::passStop() {
  if (!pass_sink_) return;
  stopUart();
  const int rx = rx_;
  rx_ = -1;
  pass_sink_ = nullptr;
  pass_ctx_ = nullptr;
  if (rx >= 0) pins_.rest(static_cast<uint16_t>(rx));   // to its free state (core §8)
}

bool FixtureUart::planApply(const RoleAssignment *roles, size_t count) {
  passStop();   // a session's plan wins over the pass-through (config §1.1)
  int rx = -1, tx = -1;
  for (size_t i = 0; i < count; ++i) (roles[i].role == ua::kRoleRx ? rx : tx) = roles[i].channel;
  if (rx_ >= 0 || tx_ >= 0) {   // an unconfigured plan replaced (fixture §2): its buffer and marks go, positions stay
    pins_.release(owner_);
    stream_.erase();
    rx_ = tx_ = -1;
  }
  if (rx >= 0 && !pins_.claim(rx, owner_)) return false;
  if (tx >= 0 && !pins_.claim(tx, owner_)) { pins_.release(owner_); return false; }
  rx_ = rx;
  tx_ = tx;
  if (rx_ >= 0) pins_.ownStrength(rx_);
  if (tx_ >= 0) pins_.ownStrength(tx_);
  session_configured_ = false;
  if (rx_ >= 0) pinMode(rx_, INPUT);
  idleHigh();   // TX rests high from the plan on, before any configure (fixture §2): the DUT's RX stays connected
  return true;   // nothing is received before a configure (fixture §2): the UART starts there
}

void FixtureUart::planRelease() {
  stopUart();
  // Released pins go to their idle state (oep-core §8, default Hi-Z). A jig whose DUT RX must not float (2026-09-22,
  // X035 USART4 stopped answering after 0.5 s of a floating PB1) sets that pin's idle to pull-up.
  pins_.release(owner_);
  rx_ = tx_ = -1;
  running_ = false;
  // The stream disappears with the plan (oep-if-fixture §2): the next plan's stream does not show this one's bytes and
  // marks. Its positions and mark serials go on (common §1.1).
  stream_.erase();
  session_configured_ = false;   // the next plan starts from the item (or the default) again
}

// The settings the UART starts with when its plan gives it pins (oep-if-probe-config §1): a session's configure since
// the plan, else the probe.config uart item, else 115200 8N1.
void FixtureUart::applySettings() {
  if (rx_ < 0 && tx_ < 0) return;
  if (session_configured_) {
    begin(baud_, format_);
  } else if (item_set_) {   // the item; a baud the divider cannot make within 5 % falls back to the default (fixture §2)
    if (!begin(item_baud_, item_format_)) begin(kDefaultBaud, 0);
  } else {
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

void FixtureUart::addPending(const UartLoss &loss) {
  if (pending_n_ == kPending) {   // full: merged into the last, moved to the earlier place (at or before both)
    UartLoss &last = pending_[kPending - 1];
    if (static_cast<int32_t>(loss.at - last.at) < 0) last.at = loss.at;
    return;
  }
  size_t i = pending_n_++;
  for (; i > 0 && static_cast<int32_t>(loss.at - pending_[i - 1].at) < 0; --i) pending_[i] = pending_[i - 1];
  pending_[i] = loss;
}

void FixtureUart::placeDue() {
  size_t done = 0;
  while (done < pending_n_ && static_cast<int32_t>(pending_[done].at - taken_) <= 0)
    stream_.mark(reg::common::kMarkKindLost, pending_[done++].detail);
  if (!done) return;
  for (size_t i = done; i < pending_n_; ++i) pending_[i - done] = pending_[i];
  pending_n_ -= done;
}

#if defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_UART_RP2)
// arduino-pico: the interrupt moves the PL011's FIFO into the receive queue (and available() / overflow() /
// getBreakReceived() do too). The queue drops what comes while it is full and says so (overflow()); it is emptied only
// here, so a drop seen before this poll takes anything left it full and unread: the gap is right after what it holds. A
// drop seen after taking (the end of poll) came while this poll took the bytes counted at its look, with the queue full:
// at or after the end of them. The PL011's overrun and a break are seen only as flags: they came after the bytes counted
// at the last look that saw neither (counted_, read before the flags).
uint32_t FixtureUart::look() {
  if (serial_.overflow()) addPending({taken_ + static_cast<uint32_t>(serial_.available()), reg::common::kMarkDetailLostOverflow});
  const uint32_t counted = taken_ + static_cast<uint32_t>(serial_.available());
  if (platformUartTakeOverrun(serial_)) addPending({counted_, reg::common::kMarkDetailLostOverflow});
  // a break is a framing error (as the ESP32's UART_BREAK_ERROR); arduino-pico drops the break byte itself
  if (serial_.getBreakReceived()) addPending({counted_, reg::common::kMarkDetailLostFraming});
  counted_ = counted;
  return counted;
}
#else
// ESP-IDF: the event task (or the fake's events) keeps the ledger; poll takes the losses handed over and no byte past
// what the events counted.
uint32_t FixtureUart::look() {
#if defined(OEP_HOST_FAKE_UART)
  uint8_t type;
  uint32_t size, left;
  while (serial_.fakeNextEvent(type, size, left)) {
    if (left + 1 >= serial_.fake_event_queue) ledger_.queueFull(left + 1);
    ledger_.event(static_cast<UartEvent>(type), size);
  }
  if (!serial_.fakeEventsHeld()) {
    ledger_.check(taken_ + static_cast<uint32_t>(serial_.available()));
    ledger_.publish();
  }
#endif
#if defined(ARDUINO_ARCH_ESP32)
  if (!ledger_on_) return taken_ + static_cast<uint32_t>(serial_.available());   // no event task could be made: no places
#endif
  const uint32_t limit = ledger_.limit();   // first: every loss before it is in the ledger by then
  UartLoss loss;
  while (pending_n_ < kPending && ledger_.next(loss)) {
    addPending(loss);
    ledger_.pop();
  }
  return limit;
}
#endif

void FixtureUart::poll() {
  if (!running_) return;
  const uint32_t limit = look();
  uint8_t chunk[256];   // in chunks: a byte at a time through the UART driver capped a 2 Mbaud bridge near 88 kB/s (P7)
  for (;;) {
    placeDue();
    uint32_t want = limit - taken_;
    if (static_cast<int32_t>(want) <= 0) break;
    if (pending_n_ && pending_[0].at - taken_ < want) want = pending_[0].at - taken_;   // up to the next place
    if (want > sizeof chunk) want = sizeof chunk;
    const int n = serial_.available();
    if (n <= 0) break;
    const size_t k = serial_.readBytes(chunk, static_cast<size_t>(n) < want ? static_cast<size_t>(n) : want);
    if (!k) break;
    if (pass_sink_) pass_sink_(pass_ctx_, chunk, k);   // a pass-through: to its port, never the stream (config §1.1)
    else for (size_t i = 0; i < k; ++i) stream_.put(chunk[i]);
    taken_ += static_cast<uint32_t>(k);
#if !(defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_UART_RP2))
    taken_seen_.store(taken_, std::memory_order_release);
#endif
  }
#if defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_UART_RP2)
  // a drop while this poll took (the queue refilled to full): after every byte counted at the look
  if (serial_.overflow()) {
    addPending({limit, reg::common::kMarkDetailLostOverflow});
    placeDue();
  }
#endif
}

#if !(defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_UART_RP2))
#if defined(ARDUINO_ARCH_ESP32)
namespace {
// arduino-esp32 keeps the driver's handle and number to itself (protected); its own event task, made only for
// onReceive / onReceiveError (not used here), would take the events this UART needs in order.
struct SerialPeek : HardwareSerial {
  static uart_t *driver(HardwareSerial &s) { return s.*(&SerialPeek::_uart); }
  static uint8_t number(HardwareSerial &s) { return s.*(&SerialPeek::_uart_nr); }
};

UartEvent eventType(uart_event_type_t type) {
  switch (type) {
    case UART_DATA: return UartEvent::kData;
    case UART_BUFFER_FULL: return UartEvent::kBufferFull;
    case UART_FIFO_OVF: return UartEvent::kFifoOverflow;
    case UART_BREAK: return UartEvent::kBreak;
    case UART_FRAME_ERR: return UartEvent::kFrameError;
    case UART_PARITY_ERR: return UartEvent::kParityError;
    default: return UartEvent::kOther;
  }
}
}  // namespace

// On the driver's interrupt core, at arduino-esp32's event-task priority: the interrupt never runs halfway through
// this task's look, so "no event waiting" means every byte in the ring was counted unless an event was dropped.
void FixtureUart::eventTask(void *arg) {
  FixtureUart *self = static_cast<FixtureUart *>(arg);
  QueueHandle_t events = static_cast<QueueHandle_t>(self->events_);
  const UBaseType_t length = uxQueueMessagesWaiting(events) + uxQueueSpacesAvailable(events);   // arduino-esp32: 20
  uart_event_t e;
  uint32_t unpublished = 0;
  while (!self->stopping_.load(std::memory_order_acquire)) {
    if (xQueueReceive(events, &e, pdMS_TO_TICKS(10)) == pdTRUE) {
      const UBaseType_t left = uxQueueMessagesWaiting(events);
      if (left + 1 >= length) self->ledger_.queueFull(left + 1);   // full (or one short, the interrupt may refill it)
      self->ledger_.event(eventType(e.type), static_cast<uint32_t>(e.size));
      // the rest of the interrupt's batch first (an error comes after the chunk that brought its byte); a queue that
      // never empties is published every 16 events all the same
      if (left && ++unpublished < 16) continue;
    }
    unpublished = 0;
    const uint32_t taken = self->taken_seen_.load(std::memory_order_acquire);   // before the driver's count
    size_t held = 0;
    if (uart_get_buffered_data_len(static_cast<uart_port_t>(self->uart_num_), &held) == ESP_OK && !uxQueueMessagesWaiting(events))
      self->ledger_.check(taken + static_cast<uint32_t>(held));
    self->ledger_.publish();
  }
  while (xQueueReceive(events, &e, 0) == pdTRUE) self->ledger_.event(eventType(e.type), static_cast<uint32_t>(e.size));
  self->ledger_.publish();
  xSemaphoreGive(static_cast<SemaphoreHandle_t>(self->stopped_));
  vTaskDelete(nullptr);
}

void FixtureUart::startEvents() {
  ledger_.reset();
  ledger_on_ = false;
  QueueHandle_t events = nullptr;
  uartGetEventQueue(SerialPeek::driver(serial_), &events);
  uart_num_ = SerialPeek::number(serial_);
  events_ = events;
  if (!events) return;
  if (!stopped_) stopped_ = xSemaphoreCreateBinary();
  stopping_.store(false, std::memory_order_release);
  const int core = irq_core_ >= 0 ? irq_core_ : static_cast<int>(xPortGetCoreID());   // where begin() put the interrupt
  TaskHandle_t task = nullptr;
  if (!stopped_ || xTaskCreatePinnedToCore(eventTask, "oep_uart_events", 3072, this, configMAX_PRIORITIES - 1, &task, core) != pdPASS)
    task = nullptr;
  task_ = task;
  ledger_on_ = task != nullptr;
}

void FixtureUart::stopEvents() {
  if (!task_) return;
  stopping_.store(true, std::memory_order_release);
  xSemaphoreTake(static_cast<SemaphoreHandle_t>(stopped_), portMAX_DELAY);   // within the task's 10 ms wait
  task_ = nullptr;
}
#else
void FixtureUart::startEvents() { ledger_.reset(); }
void FixtureUart::stopEvents() {}
#endif
#endif

size_t FixtureUart::bindInput(const uint8_t *data, size_t length) {
  // only what the UART takes without waiting, so the RX side keeps being emptied (a blocking write let the UART's
  // receive buffer overflow while a 64 KiB echo was going out)
  if (!running_ || tx_ < 0) return 0;
  const int room = serial_.availableForWrite();
  if (room <= 0) return 0;
  return serial_.write(data, static_cast<size_t>(room) < length ? static_cast<size_t>(room) : length);
}

// The UART stopped, what it received taken first (the event task ends after the events queued before).
void FixtureUart::stopUart() {
  if (!running_) return;
#if !(defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_UART_RP2))
  stopEvents();
#endif
  poll();
  // places not reached (bytes the driver had not counted, or not handed over) go at the end: every byte after them comes
  // from the next begin
  for (size_t i = 0; i < pending_n_; ++i) stream_.mark(reg::common::kMarkKindLost, pending_[i].detail);
  pending_n_ = 0;
#if defined(ARDUINO_ARCH_ESP32)
  ledger_on_ = false;
#endif
  serial_.end();
  running_ = false;
}

// The UART (re)started at baud / format. The stream and its positions stay (fixture §2: configure again keeps what was
// collected). false: the core refused the pins, or the rate came out more than 5 % off.
bool FixtureUart::begin(uint32_t baud, uint8_t format) {
  stopUart();
  // A peer may echo while the probe is still writing; hold a full window of it (and loop() away, kDriverRx).
  platformUartBuffers(serial_, kDriverRx, 1024);
  idleHigh();
  const uint8_t data_bits = (format & ua::kFormatFieldDataBitsMask) ? 7 : 8;
  const uint8_t parity = (format & ua::kFormatFieldParityMask) >> 2;
  const uint8_t stop_bits = (format & ua::kFormatFieldStopBits2) ? 2 : 1;
  if (!platformUartBegin(serial_, baud, rx_, tx_, platformUartConfig(data_bits, parity, stop_bits), irq_core_)) return false;
  const uint32_t actual = platformUartBaud(serial_, baud);
  const uint64_t diff = actual > baud ? actual - baud : baud - actual;
  if (diff * 100 > static_cast<uint64_t>(baud) * 5) { serial_.end(); idleHigh(); return false; }
  baud_ = actual;
  format_ = format;
  taken_ = 0;   // the driver's count starts again (what the last one held but had not counted went with it)
  pending_n_ = 0;
#if defined(ARDUINO_ARCH_RP2040) || defined(OEP_HOST_FAKE_UART_RP2)
  counted_ = 0;
  serial_.overflow();   // flags of the run before: nothing of this one is lost yet
  platformUartTakeOverrun(serial_);
  serial_.getBreakReceived();
#else
  taken_seen_.store(0, std::memory_order_release);
  startEvents();
#endif
  running_ = true;
  return true;
}

Result FixtureUart::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  Tail tail;
  switch (op) {
    case kOpConfigure: {   // baud(u32) [TLV 0x01 format]  ->  baud(u32, the rate the UART runs at) [TLV]
      static const uint8_t kKnown[] = {ua::kTlvConfigureFormat};
      if (length < 4) return rejected(kRejectMalformed);
      const Result parsed = tail.parse(payload + 4, length - 4, kKnown, out, capacity);
      if (refused(parsed)) return parsed;
      const uint32_t baud = getU32(payload);
      if (baud == 0) return rejected(kRejectMalformed);   // a value the definition excludes (fixture §2)
      // format: bits 0-1 data bits (0 = 8, 1 = 7), bits 2-3 parity (0 none, 1 even, 2 odd), bit 4 stop bits (0 = 1,
      // 1 = 2). 8N1 when absent. A value the definition leaves unused (or a reserved bit) is unsupported with the tag as
      // received, critical or not (core §2.3); every defined one is declared here (fixture §2).
      uint8_t format = 0;
      bool format_critical = false;
      const uint8_t *f = nullptr;
      const Result form = tail.fixed(ua::kTlvConfigureFormat, 1, f, out, capacity, &format_critical);
      if (refused(form)) return form;
      if (f) {
        if (!formatDefined(f[0]) || !formatDeclared(f[0])) return Tail::refuse(ua::kTlvConfigureFormat, format_critical, out, capacity);
        format = f[0];
      }
      if (!baudWithinReach(baud)) return unsupportedValue(out, capacity);   // a rate this UART cannot run (core §4.3)
      if (rx_ < 0 && tx_ < 0) return wrongState(out, capacity);               // no plan: no pins
      if (capacity < 4) return failed();
      if (!begin(baud, format)) {   // taken, not brought up: stopped, unconfigured, the plan and the data kept (fixture §2)
        stopUart();
        session_configured_ = false;
        idleHigh();
        putU32(out, 0);
        return failed(4);
      }
      session_configured_ = true;
      putU32(out, baud_);
      return completed(4);
    }
    case kOpStatus: {   // [TLV]  ->  baud(u32) format(u8) [TLV]: what the UART runs at (0 when not planned)
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 5) return failed();
      putU32(out, running_ ? baud_ : 0);
      out[4] = running_ ? format_ : 0;
      return completed(5);
    }
    case kOpRead: {   // from(u8) arg(u64) max(u16) [TLV]  ->  start(u64) flags(u8) len(u16) data [TLV]
      const Result parsed = plainTail(tail, payload, length, PositionStream::kReadRequest, out, capacity);
      if (refused(parsed)) return parsed;
      const Result values = PositionStream::checkRead(payload, out, capacity);   // from 4+ unused (common §1.2)
      if (refused(values)) return values;
      poll();
      return stream_.read(payload, out, capacity, max_read_);
    }
    case kOpMarks: {   // from_serial(u32) [TLV]  ->  more(u8) count(u8) entries [TLV]
      const Result parsed = plainTail(tail, payload, length, 4, out, capacity);
      if (refused(parsed)) return parsed;
      if (capacity < 2) return failed();
      poll();
      return completed(stream_.marks(getU32(payload), out, capacity));
    }
    case kOpClear: {
      const Result parsed = plainTail(tail, payload, length, 0, out, capacity);
      if (refused(parsed)) return parsed;
      poll();
      stream_.clear();
      return completed();
    }
    case kOpMark: {   // value(u8) [TLV]
      const Result parsed = plainTail(tail, payload, length, 1, out, capacity);
      if (refused(parsed)) return parsed;
      poll();
      stream_.mark(reg::common::kMarkKindHost, payload[0]);
      return completed();
    }
    case kOpWrite: {   // count(u16) data [TLV]  ->  accepted(u16) [TLV]: what the UART's send buffer took now (count 0:
                       // accepted 0, success - common §1.4)
      if (length < 2) return rejected(kRejectMalformed);
      const uint16_t count = getU16(payload);
      const Result parsed = plainTail(tail, payload, length, 2u + count, out, capacity);
      if (refused(parsed)) return parsed;
      if (!running_ || tx_ < 0) return wrongState(out, capacity);
      if (capacity < 2) return failed();
      const int room = serial_.availableForWrite();
      const size_t written = room > 0 ? serial_.write(payload + 2, static_cast<size_t>(room) < count ? static_cast<size_t>(room) : count) : 0;
      putU16(out, static_cast<uint16_t>(written));
      return written == count ? completed(2) : written ? partial(2) : failed(2);
    }
    default:
      return rejected(kRejectUnknownOperation);
  }
}

}  // namespace oep
