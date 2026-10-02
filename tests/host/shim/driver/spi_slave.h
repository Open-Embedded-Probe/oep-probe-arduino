// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of the ESP-IDF spi_slave driver for host tests (OEP_HOST_FAKE_SPI_SLAVE), with the classic ESP32's no-DMA
// slave under it as far as the target depends on it:
// - one transaction is loaded at a time; the driver's interrupt runs at a CS rising edge (trans_done), stores the
//   loaded one's result, calls post_trans_cb, returns it (unless SPI_SLAVE_NO_RETURN_RESULT), then loads the next
//   queued one, if any - or, with none, leaves the interrupt off and trans_done set, so the next queue_trans loads at
//   once (from wherever it is called, CS high or low);
// - the slave shifts whatever is loaded or not: MISO out of the 64-byte work buffer, MOSI into it, bit by bit (MSB
//   first), counting bits since CS fell or since the last load; a load rewrites the buffer from tx and restarts the
//   count, so a load while CS is low restarts the frame at that bit (counted in loads_in_frame);
// - with nothing loaded a frame still shifts the buffer (MISO what is left in it) and nobody sees it.
// The master: fakeCsLow / fakeClock / fakeCsHigh, or fakeSpiTransfer for a whole frame; g_fake_spi.miso is what the
// master read in the last frame.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <deque>
#include <vector>

typedef int esp_err_t;
typedef uint32_t TickType_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NOT_SUPPORTED 0x106
#define SPI_SLAVE_BIT_LSBFIRST 3
#define SPI_SLAVE_NO_RETURN_RESULT (1 << 2)
enum spi_host_device_t { SPI1_HOST = 0, SPI2_HOST = 1 };
enum spi_dma_chan_t { SPI_DMA_DISABLED = 0 };

struct spi_bus_config_t {
  int mosi_io_num, miso_io_num, sclk_io_num, quadwp_io_num, quadhd_io_num, max_transfer_sz;
};
struct spi_slave_transaction_t;
typedef void (*slave_transaction_cb_t)(spi_slave_transaction_t *trans);
struct spi_slave_interface_config_t {
  int spics_io_num;
  uint32_t flags;
  int queue_size;
  uint8_t mode;
  slave_transaction_cb_t post_setup_cb;
  slave_transaction_cb_t post_trans_cb;
};
struct spi_slave_transaction_t {
  size_t length;      // bits
  size_t trans_len;   // bits that came
  const void *tx_buffer;
  void *rx_buffer;
  void *user;
};

struct FakeSpiSlave {
  bool up = false;
  int queue_size = 0;
  int inits = 0;
  uint32_t flags = 0;
  slave_transaction_cb_t post_trans_cb = nullptr;
  std::deque<spi_slave_transaction_t *> queued, done;
  spi_slave_transaction_t *cur = nullptr;   // the loaded one
  bool trans_done = false, intr_on = false;
  uint8_t buf[64] = {};   // the work registers
  size_t bit = 0;         // bits since CS fell or the last load
  bool cs_low = false;
  size_t frame_bits = 0;
  int loads_in_frame = 0;
  std::vector<uint8_t> miso;   // what the master read in the last frame
};
inline FakeSpiSlave g_fake_spi;

inline void fakeSpiLoad() {
  FakeSpiSlave &f = g_fake_spi;
  f.cur = f.queued.front();
  f.queued.pop_front();
  if (f.cur->tx_buffer) memcpy(f.buf, f.cur->tx_buffer, (f.cur->length + 7) / 8);
  f.bit = 0;
  f.trans_done = false;
  if (f.cs_low) ++f.loads_in_frame;
}
// The driver's interrupt (spi_intr): runs while trans_done is set and the interrupt is on.
inline void fakeSpiIsr() {
  FakeSpiSlave &f = g_fake_spi;
  if (!f.up || !f.trans_done || !f.intr_on) return;
  if (f.cur) {
    spi_slave_transaction_t *t = f.cur;
    const size_t keep = f.bit < t->length ? f.bit : t->length;
    if (t->rx_buffer) memcpy(t->rx_buffer, f.buf, (keep + 7) / 8);
    t->trans_len = f.bit;
    if (f.post_trans_cb) f.post_trans_cb(t);
    if (!(f.flags & SPI_SLAVE_NO_RETURN_RESULT)) f.done.push_back(t);
    f.cur = nullptr;
  }
  f.intr_on = false;
  if (!f.queued.empty()) {
    f.intr_on = true;
    fakeSpiLoad();
  }
}

inline esp_err_t spi_slave_initialize(spi_host_device_t, const spi_bus_config_t *, const spi_slave_interface_config_t *c,
                                      spi_dma_chan_t) {
  FakeSpiSlave &f = g_fake_spi;
  if (f.up) return ESP_FAIL;
  f.up = true; f.queue_size = c->queue_size; f.flags = c->flags; f.post_trans_cb = c->post_trans_cb; ++f.inits;
  f.queued.clear(); f.done.clear(); f.cur = nullptr;
  f.trans_done = true; f.intr_on = false;   // forced: the first queue_trans loads at once
  memset(f.buf, 0, sizeof f.buf); f.bit = 0;
  return ESP_OK;
}
inline esp_err_t spi_slave_free(spi_host_device_t) {
  FakeSpiSlave &f = g_fake_spi;
  if (!f.up) return ESP_FAIL;
  f.up = false; f.queued.clear(); f.done.clear(); f.cur = nullptr; f.intr_on = false;
  return ESP_OK;
}
// The queue holds queue_size waiting transactions besides the one loaded.
inline esp_err_t spi_slave_queue_trans(spi_host_device_t, const spi_slave_transaction_t *t, TickType_t) {
  FakeSpiSlave &f = g_fake_spi;
  if (!f.up || f.queued.size() >= static_cast<size_t>(f.queue_size)) return ESP_FAIL;
  f.queued.push_back(const_cast<spi_slave_transaction_t *>(t));
  f.intr_on = true;
  fakeSpiIsr();
  return ESP_OK;
}
// From an interrupt (post_trans_cb): only queued; the interrupt under way loads it after the callback.
inline esp_err_t spi_slave_queue_trans_isr(spi_host_device_t, const spi_slave_transaction_t *t) {
  FakeSpiSlave &f = g_fake_spi;
  if (!f.up || f.queued.size() >= static_cast<size_t>(f.queue_size)) return ESP_FAIL;
  f.queued.push_back(const_cast<spi_slave_transaction_t *>(t));
  return ESP_OK;
}
// The queue emptied and the loaded one forgotten (no result); the next queue_trans loads at once.
inline esp_err_t spi_slave_queue_reset(spi_host_device_t) {
  FakeSpiSlave &f = g_fake_spi;
  if (!f.up) return ESP_FAIL;
  f.intr_on = false; f.trans_done = true; f.queued.clear(); f.cur = nullptr;
  return ESP_OK;
}
inline esp_err_t spi_slave_get_trans_result(spi_host_device_t, spi_slave_transaction_t **t, TickType_t) {
  FakeSpiSlave &f = g_fake_spi;
  if (!f.up) return ESP_FAIL;
  if (f.flags & SPI_SLAVE_NO_RETURN_RESULT) return ESP_ERR_NOT_SUPPORTED;
  if (f.done.empty()) return ESP_FAIL;
  *t = f.done.front();
  f.done.pop_front();
  return ESP_OK;
}

// The master. CS falling: true if a transaction is loaded to take the frame.
inline bool fakeCsLow() {
  FakeSpiSlave &f = g_fake_spi;
  f.cs_low = true; f.bit = 0; f.frame_bits = 0; f.miso.clear();
  return f.up && f.cur;
}
// `bits` clocks; MOSI from `mosi` at the frame's own bit position (0 with none).
inline void fakeClock(size_t bits, const uint8_t *mosi = nullptr) {
  FakeSpiSlave &f = g_fake_spi;
  for (size_t i = 0; i < bits; ++i) {
    const size_t fb = f.frame_bits++;
    const int mo = mosi ? (mosi[fb / 8] >> (7 - fb % 8)) & 1 : 0;
    const size_t pos = f.bit++;
    int mi = 0;
    if (pos < sizeof f.buf * 8) {
      uint8_t &b = f.buf[pos / 8];
      const uint8_t m = static_cast<uint8_t>(0x80u >> (pos % 8));
      mi = (b & m) ? 1 : 0;
      b = static_cast<uint8_t>(mo ? (b | m) : (b & ~m));
    }
    if (fb % 8 == 0) f.miso.push_back(0);
    if (mi) f.miso.back() = static_cast<uint8_t>(f.miso.back() | (0x80u >> (fb % 8)));
  }
}
// CS rising: trans_done, and the interrupt if it is on.
inline void fakeCsHigh() {
  FakeSpiSlave &f = g_fake_spi;
  f.cs_low = false;
  f.trans_done = true;
  fakeSpiIsr();
}
// One whole CS-framed transfer of `bits` clocks. false: nothing took it.
inline bool fakeSpiTransfer(size_t bits, const uint8_t *mosi = nullptr) {
  const bool taken = fakeCsLow();
  fakeClock(bits, mosi);
  fakeCsHigh();
  return taken;
}
