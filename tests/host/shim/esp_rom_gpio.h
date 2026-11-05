// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of the ESP ROM's GPIO matrix input routing, for the SPI target's CS (driver/spi_slave.h's fake takes it).
#pragma once
#include <stdint.h>

#include <driver/spi_slave.h>

inline void esp_rom_gpio_connect_in_signal(uint32_t pin, uint32_t signal, bool) { fakeSpiConnectIn(pin, signal); }
