// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the label convention (oep-spec oep-if-probe-config §1.3) - a slot's line is the label "S.N", else, with at
// most one slot item, "N"; ASCII case ignored; two channels at the step that matches name none.
#include <stdio.h>

#include <string>
#include <vector>

#include "OepConfig.h"

using namespace oep;
using Bytes = std::vector<uint8_t>;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

static void label(Bytes &items, uint16_t channel, const std::string &text) {
  items.push_back(reg::probe_config::kTlvItemLabel);
  items.push_back(static_cast<uint8_t>(2 + text.size()));
  items.push_back(channel & 0xff);
  items.push_back(channel >> 8);
  items.insert(items.end(), text.begin(), text.end());
}
static void slot(Bytes &items, uint8_t number) {   // only its tag counts here
  items.push_back(reg::probe_config::kTlvItemSlot);
  items.push_back(1);
  items.push_back(number);
}
static uint16_t find(const Bytes &items, const char *s, const char *n) { return findLine(items.data(), items.size(), s, n); }

int main() {
  {   // no slots: "N", any case; two channels named the same: none
    Bytes items;
    label(items, 5, "NRST");
    label(items, 6, "power_hi");
    CHECK(find(items, nullptr, "nrst") == 5);
    CHECK(find(items, nullptr, "power_hi") == 6);
    CHECK(find(items, nullptr, "power_lo") == 0xffff);
    CHECK(find(items, nullptr, "nrs") == 0xffff);   // the whole text, not a prefix
    label(items, 7, "nrst");
    CHECK(find(items, nullptr, "nrst") == 0xffff);
  }
  {   // one slot: "S.N" first, then "N"
    Bytes items;
    label(items, 3, "nrst");
    slot(items, 0);
    CHECK(find(items, "dut", "nrst") == 3);
    label(items, 9, "Dut.Nrst");
    CHECK(find(items, "dut", "nrst") == 9);
    label(items, 10, "dut.nrst");   // two at the "S.N" step: none (no fall back to "N")
    CHECK(find(items, "dut", "nrst") == 0xffff);
  }
  {   // two slots: "N" alone names nothing for a slot
    Bytes items;
    label(items, 3, "nrst");
    label(items, 4, "b.nrst");
    slot(items, 0);
    slot(items, 1);
    CHECK(find(items, "a", "nrst") == 0xffff);
    CHECK(find(items, "b", "nrst") == 4);
    CHECK(find(items, "a", "b.nrst") == 0xffff);
  }
  printf("lines: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
