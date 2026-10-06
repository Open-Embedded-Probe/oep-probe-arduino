// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host tests: the label convention (oep-spec oep-if-probe-config §1.3) - a slot's line is the label "S.N", else, with at
// most one slot item, "N", else, likewise, the firmware label "N"; ASCII case ignored; two channels at the step that
// matches name none.
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
static uint16_t find(const Bytes &items, const Bytes &firmware, const char *s, const char *n) {
  return findLine(items.data(), items.size(), s, n, firmware.data(), firmware.size());
}
static void firmwareLabel(Bytes &tlv, uint16_t channel, const std::string &text) {   // fn 0 describe's label 0x46
  tlv.push_back(reg::core::kTlvDescribeLabel);
  tlv.push_back(static_cast<uint8_t>(2 + text.size()));
  tlv.push_back(channel & 0xff);
  tlv.push_back(channel >> 8);
  tlv.insert(tlv.end(), text.begin(), text.end());
}

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
  {   // step (c): with at most one slot, a firmware label "N" when no settings label names the line
    Bytes firmware;
    firmwareLabel(firmware, 0x20, "x");   // another TLV of fn 0's describe is passed over
    firmware[0] = reg::core::kTlvDescribeChannels;
    firmwareLabel(firmware, 12, "NRST");
    Bytes items;
    CHECK(find(items, firmware, nullptr, "nrst") == 12);   // no slots
    slot(items, 0);
    CHECK(find(items, firmware, "dut", "nrst") == 12);     // one slot
    CHECK(find(items, firmware, "dut", "power_hi") == 0xffff);
    label(items, 4, "nrst");                                // (b) first
    CHECK(find(items, firmware, "dut", "nrst") == 4);
    label(items, 5, "dut.nrst");                            // (a) first
    CHECK(find(items, firmware, "dut", "nrst") == 5);
    Bytes two;
    slot(two, 0);
    slot(two, 1);
    CHECK(find(two, firmware, "dut", "nrst") == 0xffff);   // two slots: no (b), no (c)
    Bytes twice = firmware;
    firmwareLabel(twice, 13, "nrst");
    CHECK(find(Bytes{}, twice, nullptr, "nrst") == 0xffff);   // two channels at (c): none
    Bytes settings2;
    label(settings2, 4, "nrst");
    label(settings2, 6, "nrst");
    CHECK(find(settings2, firmware, nullptr, "nrst") == 0xffff);   // two at (b) end the search before (c)
  }
  printf("lines: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
