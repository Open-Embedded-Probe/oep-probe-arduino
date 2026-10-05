// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's hal_utils clock divider helper for host tests (OEP_HOST_FAKE_PARLIO): an integer divider only.
#pragma once
#include <stdint.h>
typedef struct { uint32_t src_freq_hz, exp_freq_hz, max_integ, min_integ, max_fract; } hal_utils_clk_info_t;
typedef struct { uint32_t integer, denominator, numerator; } hal_utils_clk_div_t;
inline uint32_t hal_utils_calc_clk_div_frac_accurate(const hal_utils_clk_info_t *info, hal_utils_clk_div_t *div) {
  uint32_t n = (info->src_freq_hz + info->exp_freq_hz / 2) / info->exp_freq_hz;
  if (n < 1) n = 1;
  if (n > info->max_integ - 1) n = info->max_integ - 1;
  *div = {n, 1, 0};
  return info->src_freq_hz / n;
}
