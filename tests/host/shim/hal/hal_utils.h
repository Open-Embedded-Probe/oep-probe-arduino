// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A fake of ESP-IDF's hal_utils clock divider helper for host tests (OEP_HOST_FAKE_PARLIO): an integer divider and a
// fraction of it in 1 / (max_fract - 1) steps, the nearest to the asked frequency.
#pragma once
#include <stdint.h>
typedef struct { uint32_t src_freq_hz, exp_freq_hz, max_integ, min_integ, max_fract; } hal_utils_clk_info_t;
typedef struct { uint32_t integer, denominator, numerator; } hal_utils_clk_div_t;
inline uint32_t hal_utils_calc_clk_div_frac_accurate(const hal_utils_clk_info_t *info, hal_utils_clk_div_t *div) {
  const uint64_t src = info->src_freq_hz, exp = info->exp_freq_hz ? info->exp_freq_hz : 1;
  const uint32_t den = info->max_fract > 1 ? info->max_fract - 1 : 1;
  uint64_t total = (src * den + exp / 2) / exp;   // the divider in 1 / den steps, the nearest
  if (total < den) total = den;
  if (total > static_cast<uint64_t>(info->max_integ - 1) * den) total = static_cast<uint64_t>(info->max_integ - 1) * den;
  const uint32_t n = static_cast<uint32_t>(total / den), num = static_cast<uint32_t>(total % den);
  *div = {n, num ? den : 1, num};
  return static_cast<uint32_t>(src * div->denominator / (static_cast<uint64_t>(n) * div->denominator + div->numerator));
}
