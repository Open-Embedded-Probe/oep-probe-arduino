// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of the RP2's DMA (pico-sdk hardware/dma.h) for host tests (OEP_HOST_FAKE_RP2_ADC): one channel. A test says
// how many transfers it has made (g_fake_dma.written) and writes the values itself at g_fake_dma.to; transfer_count and
// busy follow from that.
#pragma once
#include <stdint.h>

enum dma_channel_transfer_size { DMA_SIZE_8 = 0, DMA_SIZE_16 = 1, DMA_SIZE_32 = 2 };
#define DREQ_ADC 36
struct dma_channel_config { bool ring; };
struct FakeDmaHw { uint32_t transfer_count; };
struct FakeDma {
  bool claimed, ring, aborted;
  uint16_t *to;
  uint32_t count, written;
  FakeDmaHw hw;
};
inline FakeDma g_fake_dma = {};
inline int dma_claim_unused_channel(bool) { g_fake_dma.claimed = true; return 0; }
inline dma_channel_config dma_channel_get_default_config(unsigned) { return {}; }
inline void channel_config_set_transfer_data_size(dma_channel_config *, dma_channel_transfer_size) {}
inline void channel_config_set_read_increment(dma_channel_config *, bool) {}
inline void channel_config_set_write_increment(dma_channel_config *, bool) {}
inline void channel_config_set_ring(dma_channel_config *c, bool, unsigned) { c->ring = true; }
inline void channel_config_set_dreq(dma_channel_config *, unsigned) {}
inline void dma_channel_configure(unsigned, const dma_channel_config *c, void *to, const volatile void *, uint32_t count, bool) {
  g_fake_dma.ring = c->ring;
  g_fake_dma.to = static_cast<uint16_t *>(to);
  g_fake_dma.count = count;
  g_fake_dma.written = 0;
  g_fake_dma.aborted = false;
}
inline FakeDmaHw *dma_channel_hw_addr(unsigned) {
  g_fake_dma.hw.transfer_count = g_fake_dma.count - (g_fake_dma.written < g_fake_dma.count ? g_fake_dma.written : g_fake_dma.count);
  return &g_fake_dma.hw;
}
inline bool dma_channel_is_busy(unsigned) { return !g_fake_dma.aborted && g_fake_dma.written < g_fake_dma.count; }
inline void dma_channel_abort(unsigned) { g_fake_dma.aborted = true; }
