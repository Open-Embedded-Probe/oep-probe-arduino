// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of the ESP-IDF spi_slave driver for host tests (OEP_HOST_FAKE_SPI_SLAVE): the transactions queued wait in
// order; fakeSpiTransfer() is the master doing one CS-framed transfer - the oldest queued transaction takes it and its
// result is ready for spi_slave_get_trans_result. With nothing queued the transfer goes unseen, as on the hardware.
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
#define SPI_SLAVE_BIT_LSBFIRST 1
enum spi_host_device_t { SPI1_HOST = 0, SPI2_HOST = 1 };
enum spi_dma_chan_t { SPI_DMA_DISABLED = 0 };

struct spi_bus_config_t {
  int mosi_io_num, miso_io_num, sclk_io_num, quadwp_io_num, quadhd_io_num, max_transfer_sz;
};
struct spi_slave_interface_config_t {
  int spics_io_num;
  uint32_t flags;
  int queue_size;
  uint8_t mode;
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
  std::deque<spi_slave_transaction_t *> queued, done;
  std::vector<uint8_t> miso;   // what the last transfer sent
};
inline FakeSpiSlave g_fake_spi;

inline esp_err_t spi_slave_initialize(spi_host_device_t, const spi_bus_config_t *, const spi_slave_interface_config_t *c,
                                      spi_dma_chan_t) {
  if (g_fake_spi.up) return ESP_FAIL;
  g_fake_spi.up = true; g_fake_spi.queue_size = c->queue_size; ++g_fake_spi.inits;
  g_fake_spi.queued.clear(); g_fake_spi.done.clear();
  return ESP_OK;
}
inline esp_err_t spi_slave_free(spi_host_device_t) {
  if (!g_fake_spi.up) return ESP_FAIL;
  g_fake_spi.up = false; g_fake_spi.queued.clear(); g_fake_spi.done.clear();
  return ESP_OK;
}
// The queue holds queue_size waiting transactions besides the one loaded; the fake keeps it to queue_size in all.
inline esp_err_t spi_slave_queue_trans(spi_host_device_t, const spi_slave_transaction_t *t, TickType_t) {
  if (!g_fake_spi.up || g_fake_spi.queued.size() >= static_cast<size_t>(g_fake_spi.queue_size)) return ESP_FAIL;
  g_fake_spi.queued.push_back(const_cast<spi_slave_transaction_t *>(t));
  return ESP_OK;
}
inline esp_err_t spi_slave_get_trans_result(spi_host_device_t, spi_slave_transaction_t **t, TickType_t) {
  if (!g_fake_spi.up || g_fake_spi.done.empty()) return ESP_FAIL;
  *t = g_fake_spi.done.front();
  g_fake_spi.done.pop_front();
  return ESP_OK;
}
// The master: one CS frame of `bits` clocks, MOSI from `mosi` (bits / 8 bytes, rounded up). false: nothing took it.
inline bool fakeSpiTransfer(size_t bits, const uint8_t *mosi = nullptr) {
  g_fake_spi.miso.clear();
  if (!g_fake_spi.up || g_fake_spi.queued.empty()) return false;
  spi_slave_transaction_t *t = g_fake_spi.queued.front();
  g_fake_spi.queued.pop_front();
  const size_t keep = (bits < t->length ? bits : t->length) / 8;
  if (mosi) memcpy(t->rx_buffer, mosi, keep);
  g_fake_spi.miso.assign(static_cast<const uint8_t *>(t->tx_buffer), static_cast<const uint8_t *>(t->tx_buffer) + keep);
  t->trans_len = bits;
  g_fake_spi.done.push_back(t);
  return true;
}
