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
  }
  // a channel outside role_channels is unsupported, a declared one something else holds unavailable (core §8)
  for (int r = kRoleSck; r <= kRoleCs; ++r) if (!pins_.allowed(pin[r])) return kRejectUnsupported;
  for (int r = kRoleSck; r <= kRoleCs; ++r) if (!pins_.free(pin[r])) return kRejectUnavailable;
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
  for (int r = kRoleSck; r <= kRoleCs; ++r) pins_.ownStrength(static_cast<uint16_t>(pin[r]));
  return true;
}

// Releasing or replacing the plan: back to the state right after describe (fixture §4).
void P4SpiTarget::planRelease() {
  stop();
  pins_.release(kOwnerId);   // each pin to its idle state
  sck_ = mosi_ = miso_ = cs_ = -1;
  mode_ = 0; bit_order_ = 0; transactions_ = 0; errors_ = 0;
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
#if defined(OEP_SPI_MISO_GATE)
  // cs_setup_ns (u32): how long after CS falls MISO may still be undriven - the gate's worst case (kCsSetupNs).
  w.u32(reg::fixture_spi_target::kTlvDescribeCsSetupNs, kCsSetupNs);
#endif
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
  // No result queue: post_trans_cb sees every end and loads the next transaction there.
  cfg.flags = (bit_order_ ? SPI_SLAVE_BIT_LSBFIRST : 0) | SPI_SLAVE_NO_RETURN_RESULT;
  cfg.queue_size = 1;   // never more than one waits: the one post_trans_cb or arm puts in, loaded at once
  cfg.mode = mode_;
  cfg.post_trans_cb = onDone;
  isr_armed_done_ = false; isr_armed_bits_ = 0; isr_unarmed_ = 0; isr_load_failed_ = false;
  // The discard: MISO 0, whatever the master sends.
  idle_trans_ = {};
  idle_trans_.length = kMaxFrame * 8; idle_trans_.tx_buffer = idle_tx_; idle_trans_.rx_buffer = idle_rx_;
  idle_trans_.user = this;
  return spi_slave_initialize(SPI2_HOST, &bus, &cfg, SPI_DMA_DISABLED) == ESP_OK;
}

// In the driver's interrupt, at the CS rising edge that ended `done` (its result already stored): note what it was and
// queue the next one, which the same interrupt loads right after this returns - while CS is high, before the master
// can start another frame. Loading from loop() instead came at a moment of its own: inside the next frame, the slave
// was reset there and the transaction held only the bits from that point (0.0.28, the classic ESP32).
void IRAM_ATTR P4SpiTarget::onDone(spi_slave_transaction_t *done) {
  P4SpiTarget *self = static_cast<P4SpiTarget *>(done->user);
  const spi_slave_transaction_t *next = &self->idle_trans_;
  portENTER_CRITICAL_ISR(&self->lock_);
  if (done == &self->trans_) {
    if (done->trans_len == 0) {
      next = &self->trans_;   // CS without a clock: noise, the arm keeps waiting
    } else {
      self->isr_armed_done_ = true;
      self->isr_armed_bits_ = done->trans_len;
    }
  } else if (done->trans_len) {
    ++self->isr_unarmed_;   // a transfer nobody armed: MOSI dropped, counted (fixture §4)
  }
  portEXIT_CRITICAL_ISR(&self->lock_);
  if (spi_slave_queue_trans_isr(SPI2_HOST, next) != ESP_OK) self->isr_load_failed_ = true;
}

#if defined(OEP_SPI_MISO_GATE)
// MISO driven only while selected: the pad's output enable moves from the slave to the GPIO enable bit, set from the
// CS level now and at every CS edge after. The slave keeps the pad's output value (the bits it shifts). The level is
// read in the handler rather than taken from the edge's kind, so a late or merged interrupt still leaves it right.
void IRAM_ATTR P4SpiTarget::onCs(void *arg) {
  const P4SpiTarget *self = static_cast<const P4SpiTarget *>(arg);
  if (gpio_ll_get_level(&GPIO, self->cs_)) gpio_ll_output_disable(&GPIO, self->miso_);
  else gpio_ll_output_enable(&GPIO, self->miso_);
}

// The GPIO ISR service, once per boot, from an ordinary task pinned to kGateCore: the service takes the core of its
// first install (its interrupts go there and its handlers run there). Not from the IPC task: the install allocates the
// interrupt through esp_ipc_call_blocking itself, and a call from inside an IPC call waits for the IPC lock its caller
// holds - forever (0.0.28+ec38b1d: configure never answered, the probe stopped). Once only: a second install is refused
// with an error log, and on the classic the log goes out on UART0, the OEP port.
namespace {
struct ServiceJob {
  SemaphoreHandle_t done;
  esp_err_t result;
};
void serviceTask(void *arg) {
  ServiceJob *job = static_cast<ServiceJob *>(arg);
  job->result = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
  xSemaphoreGive(job->done);
  vTaskDelete(nullptr);
}
}  // namespace

bool P4SpiTarget::gateService() {
  static bool installed = false;
  if (installed) return true;
  static StaticSemaphore_t buffer;
  static SemaphoreHandle_t done = xSemaphoreCreateBinaryStatic(&buffer);
  ServiceJob job = {done, ESP_FAIL};
  if (xTaskCreatePinnedToCore(serviceTask, "oep_gpio_isr", kServiceStack, &job, uxTaskPriorityGet(nullptr), nullptr,
                              kGateCore) != pdPASS)
    return false;
  xSemaphoreTake(done, portMAX_DELAY);
  installed = job.result == ESP_OK || job.result == ESP_ERR_INVALID_STATE;   // INVALID_STATE: the sketch installed it
  return installed;
}

// On kGateCore, in the IPC task (nothing here makes an IPC call of its own): the CS handler added and MISO set from CS
// once - inside a critical section, so no CS edge's handler on this core falls between the look at CS and the output
// enable it sets.
void P4SpiTarget::gateInstall(void *arg) {
  P4SpiTarget *self = static_cast<P4SpiTarget *>(arg);
  self->gate_ok_ = false;
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);
  if (gpio_isr_handler_add(static_cast<gpio_num_t>(self->cs_), onCs, self) == ESP_OK) {
    gpio_set_intr_type(static_cast<gpio_num_t>(self->cs_), GPIO_INTR_ANYEDGE);
    onCs(self);
    self->gate_ok_ = true;
  }
  portEXIT_CRITICAL(&mux);
}

// On kGateCore: the handler goes and MISO is left undriven, with no handler on this core in between to drive it again.
void P4SpiTarget::gateRemove(void *arg) {
  P4SpiTarget *self = static_cast<P4SpiTarget *>(arg);
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);
  gpio_set_intr_type(static_cast<gpio_num_t>(self->cs_), GPIO_INTR_DISABLE);
  gpio_isr_handler_remove(static_cast<gpio_num_t>(self->cs_));
  gpio_ll_output_disable(&GPIO, self->miso_);
  portEXIT_CRITICAL(&mux);
}

bool P4SpiTarget::gateBegin() {
  gpio_ll_output_disable(&GPIO, miso_);   // undriven until the handler looks at CS
  gpio_ll_set_output_enable_ctrl(&GPIO, static_cast<uint8_t>(miso_), false, false);
  if (!gateService() || esp_ipc_call_blocking(kGateCore, gateInstall, this) != ESP_OK || !gate_ok_) {
    gpio_ll_output_disable(&GPIO, miso_);
    return false;
  }
  gated_ = true;
  return true;
}

// Before the driver goes: MISO stays off from here; spi_slave_free and the pin table's release put the pad to idle.
void P4SpiTarget::gateEnd() {
  if (!gated_) return;
  esp_ipc_call_blocking(kGateCore, gateRemove, this);
  gpio_ll_output_disable(&GPIO, miso_);
  gated_ = false;
}
#endif

bool P4SpiTarget::start() {
  if (started_) return true;
  if (!begin()) return false;
#if defined(OEP_SPI_MISO_GATE)
  if (!gateBegin()) { spi_slave_free(SPI2_HOST); return false; }
#endif
  started_ = true;
  if (spi_slave_queue_trans(SPI2_HOST, &idle_trans_, 0) != ESP_OK) isr_load_failed_ = true;
  return true;
}

void P4SpiTarget::stop() {
#if defined(OEP_SPI_MISO_GATE)
  gateEnd();
#endif
  if (started_) spi_slave_free(SPI2_HOST);
#if defined(OEP_SPI_MISO_GATE)
  if (started_) gpio_ll_set_output_enable_ctrl(&GPIO, static_cast<uint8_t>(miso_), true, false);   // as before begin()
#endif
  started_ = false; armed_ = false; queue_count_ = 0;
}

bool P4SpiTarget::arm(const uint8_t *tx, size_t tx_length, size_t length) {
  if (!started_ || armed_ || !length || length > kMaxFrame || tx_length > length) return false;
  service();   // count what the discard got so far
  memset(tx_buffer_, 0, sizeof tx_buffer_); memcpy(tx_buffer_, tx, tx_length);
  memset(rx_buffer_, 0, sizeof rx_buffer_);
  trans_ = {};
  trans_.length = length * 8; trans_.tx_buffer = tx_buffer_; trans_.rx_buffer = rx_buffer_; trans_.user = this;
  // The loaded discard is taken back (the driver forgets it; the next load replaces it) and the armed one goes in its
  // place, loaded at once. A transfer under way right now is cut: it was not armed.
  spi_slave_queue_reset(SPI2_HOST);
  isr_load_failed_ = false;
  if (spi_slave_queue_trans(SPI2_HOST, &trans_, 0) != ESP_OK) {
    if (spi_slave_queue_trans(SPI2_HOST, &idle_trans_, 0) != ESP_OK) isr_load_failed_ = true;
    return false;
  }
  armed_ = true; armed_length_ = length;
  return true;
}

void P4SpiTarget::service() {
  if (!started_) return;
  portENTER_CRITICAL(&lock_);
  const bool armed_done = isr_armed_done_;
  const uint32_t armed_bits = isr_armed_bits_, unarmed = isr_unarmed_;
  const bool load_failed = isr_load_failed_;
  isr_armed_done_ = false; isr_unarmed_ = 0; isr_load_failed_ = false;
  portEXIT_CRITICAL(&lock_);
  transactions_ += unarmed; errors_ += unarmed;
  if (armed_done && armed_) {
    armed_ = false;
    ++transactions_;
    if (armed_bits > armed_length_ * 8) ++errors_;   // over length: the rest was dropped
    if (queue_count_ == kQueueDepth) {
      ++errors_;
    } else {
      const size_t bytes = (armed_bits + 7) / 8 > armed_length_ ? armed_length_ : (armed_bits + 7) / 8;
      memcpy(queue_[queue_count_], rx_buffer_, bytes);
      queue_length_[queue_count_] = static_cast<uint8_t>(bytes);
      queue_bits_[queue_count_] = armed_bits;
      ++queue_count_;
    }
  }
  // The interrupt could not queue the next one (not expected: nothing else waits): nothing is loaded, put it in now.
  if (load_failed) spi_slave_queue_trans(SPI2_HOST, armed_ ? &trans_ : &idle_trans_, 0);
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
      stop();   // the queue and the wait go
      mode_ = payload[0]; bit_order_ = payload[1]; transactions_ = 0; errors_ = 0;
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
      if (!started_) return wrongState(out, capacity);   // state 0 (fixture §4)
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
