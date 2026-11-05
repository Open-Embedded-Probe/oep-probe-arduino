// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's SPI signal table: the slave's CS input signal, all the SPI target uses.
#pragma once
#include <stdint.h>

#include <driver/spi_slave.h>

struct spi_signal_conn_t {
  uint8_t spics_in;
};
inline const spi_signal_conn_t spi_periph_signal[2] = {{kFakeSpiCsIn - 1}, {kFakeSpiCsIn}};
