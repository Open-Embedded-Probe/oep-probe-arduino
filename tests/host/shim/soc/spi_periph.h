// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's SPI signal table: SPI2's slave signals the SPI target routes.
#pragma once
#include <stdint.h>

#include <driver/spi_slave.h>

struct spi_signal_conn_t {
  uint8_t spiclk_in, spid_in, spiq_out, spics_in;
};
inline const spi_signal_conn_t spi_periph_signal[2] = {
    {kFakeSpiClkIn - 8, kFakeSpiMosiIn - 8, kFakeSpiMisoOut - 8, kFakeSpiCsIn - 8},
    {kFakeSpiClkIn, kFakeSpiMosiIn, kFakeSpiMisoOut, kFakeSpiCsIn}};
