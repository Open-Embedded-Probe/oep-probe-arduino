// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// This implementation's own times and counts on the debug wires (docs/implementation-limits.ja.md §1.2). The OEP
// specification leaves them to the probe (oep-if-debug §1-§4: attach, scan and riscv-dm's reset answer within
// max_op_ms; the probe chooses how long it retries, waits for the DM and waits after a reset line before it gives up
// and when it declares the wire lost). A host does not rely on them: it waits max_op_ms for those ops.
#pragma once

#include <stdint.h>

namespace oep {
namespace limits {

constexpr uint32_t kWireRetryMs = 200;      // the wire retries inside one request, slower speeds included
constexpr uint32_t kWireLostMs = 1000;      // failing with no answer this long (real time, no success between): lost
constexpr uint32_t kAttachBudgetMs = 1000;  // one attach's speed search and retries (a reset's hold and the DM wait aside)
constexpr uint32_t kScanBudgetMs = 500;     // no new pair is started after this (at least one is tried)
constexpr uint32_t kResetSettleMs = 700;    // after a reset line or ndmreset is let go: the DM's silence waited out
constexpr uint32_t kDmWaitMs = 100;         // one wait for DM state inside a high-level op (busy, allhalted, allresumeack)
constexpr uint32_t kDmiBusyRetries = 100;   // DMI busy retried, then status wait
constexpr uint32_t kResetRetries = 1;       // riscv-dm reset: the sequence redone at most this often
constexpr uint32_t kSwdWaitRetries = 100;   // SWD WAIT retried, then status wait
constexpr uint32_t kTarRewriteBytes = 1024; // arm-adi block ops write TAR again at every boundary of this many bytes

}  // namespace limits
}  // namespace oep
