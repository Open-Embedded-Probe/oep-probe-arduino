// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of the ESP32-P4 HP_SYS_CLKRST registers PARLIO RX's divider is read back from (OEP_HOST_FAKE_PARLIO): the
// parlio_rx fake writes them.
#pragma once
#include <stdint.h>
struct FakeHpSysClkrst {
  struct { uint32_t reg_parlio_rx_clk_div_num; } peri_clk_ctrl117;
  struct { uint32_t reg_parlio_rx_clk_div_numerator, reg_parlio_rx_clk_div_denominator; } peri_clk_ctrl118;
};
inline FakeHpSysClkrst HP_SYS_CLKRST = {};
