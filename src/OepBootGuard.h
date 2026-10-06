// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A probe that never stays wedged, and a crash at boot that cannot repeat for ever.
//
//   Reset, never halt. RP2040 / RP2350: the hardware watchdog runs (kWatchdogMs), fed from a timer interrupt only while
//   loop() keeps coming round (kStallMs: longer than any one request, max_op_ms). A panic (the SDK's panic() ends in
//   _exit) or a HardFault would otherwise stop the core at a breakpoint with the USB pull-up still on - the host then
//   fails the device descriptor request until a replug; the sketch points both at crashed(), which takes the device off
//   the bus and resets the chip. A core locked up, or interrupts off for good, stops the feeding: the watchdog resets.
//   ESP32 / ESP32-P4: a panic and the interrupt watchdog already reset the chip (the core's sdkconfig: panic print and
//   reboot, INT_WDT 300 ms); begin() also puts loop() under the task watchdog with kStallMs (the core watches only
//   CPU0's idle task, and loop() runs on CPU1).
//
//   Fast crash-boots counted. A boot whose reset was a crash (RP2: a watchdog reset while the record said running; ESP:
//   esp_reset_reason a panic or a watchdog) and that came within kStableMs of the boot before it counts one more; a boot
//   up kStableMs, a restart the firmware makes on purpose (planned) or any other reset (power-on, the reset pin) starts
//   again at 0. After kSafeAfter in a row the boot is a safe one (safe): the sketch does not start what the saved settings
//   would start by themselves - the at-boot slots' attach and the consoles that ride it (ProbeConfig::skipBootAttach) -
//   so a crash those set off at boot cannot bring the probe down again and again. One boot only: the next starts them.
//   The count is kept where a reset leaves it (RP2: watchdog scratch 0 / 1; ESP: RTC memory not initialised at boot).
//
//   The USB gate. attachReady(configured) is the at-boot attach's gate on a probe whose transport is its own USB device
//   (ProbeConfig::setAttachGate): open once the host has kept the device configured for kUsbSettleMs, or kAttachGraceMs
//   after boot without a host (a probe on a charger still attaches).
#pragma once

#include <Arduino.h>

#include "Oep.h"

namespace oep {

class BootGuard {
 public:
  static constexpr uint8_t kSafeAfter = 3;          // fast crash-boots in a row before a safe boot
  static constexpr uint32_t kStableMs = 30000;      // a boot up this long ends the run of fast crash-boots
  static constexpr uint32_t kStallMs = kMaxOpMs + 5000;   // loop() not round for this long: reset (a request takes <= max_op_ms)
  static constexpr uint32_t kWatchdogMs = 3000;     // RP2 hardware watchdog (RP2040 at most 8388 ms)
  static constexpr uint32_t kFeedMs = 250;          // RP2: its feeding timer's period
  static constexpr uint32_t kUsbSettleMs = 1000;    // the device configured this long before the at-boot attach
  static constexpr uint32_t kAttachGraceMs = 5000;  // ... or this long after boot without a host
  static_assert(kWatchdogMs <= 8388 && kFeedMs * 4 <= kWatchdogMs, "the RP2040 watchdog counts at most 8388 ms");
  static_assert(kStableMs > kStallMs + kWatchdogMs, "a stall's reset still counts as a fast crash-boot");
  static_assert(kStableMs > kAttachGraceMs + kUsbSettleMs + kStallMs,
                "a stall in the at-boot attach still counts as a fast crash-boot");

  // First thing in setup(): reads and updates the count, starts the watchdogs.
  static void begin();
  // This boot is a safe one (kSafeAfter fast crash-boots before it).
  static bool safe() { return crashes() >= kSafeAfter; }
  // Fast crash-boots in a row just before this boot.
  static uint8_t crashes();
  // From loop(): loop() is alive (the stall watch); kStableMs up, the count goes back to 0.
  static void poll();
  // Before a restart the firmware makes on purpose (oep.probe.restart, a firmware update): not a crash. A firmware
  // update written to the other app slot is noted (the next boot tells if the bootloader did not start it).
  static void planned();
  // What ended the boot before this one, as text, "" for a power-on or a restart on purpose: the reset ("panic",
  // "task-wdt", "int-wdt", "wdt", "brownout", "usb", "jtag", "reset-pin", "cpu-lockup", "software" not by the probe,
  // "other"; RP2 "wdt": a crash, a stall or a hard watchdog reset) and the seconds that boot was up, as
  // "<reset> at <n> s"; an update the bootloader did not start (the image failed its check, the bootloader went back
  // to the one before), or that ended before setup(): "update to <slot> did not reach setup: <reset>". The
  // reference firmware puts it after its version in fn 0's describe firmware text (describeCore), so a host's describe
  // shows it with no field of its own.
  static const char *lastBoot();
  // RP2: the panic / HardFault handlers' end - the USB device off the bus for kRestartDetachMs, then a reset (not on
  // the host).
  [[noreturn]] static void crashed();
  // The at-boot attach's gate on a USB probe: `configured` is the host having configured the device now.
  static bool attachReady(bool configured);
};

#if defined(OEP_HOST_FAKE_BOOT)
// Host tests: what a reset leaves (the record, valid or not) and whether the reset before this boot was a crash.
struct FakeBootRecord { bool valid = false; uint32_t state = 0, count = 0, up_s = 0, next = 0; };
extern FakeBootRecord g_boot_record;
extern bool g_boot_reset_crash;
extern uint8_t g_boot_reset_kind;          // the reset when not a crash: 0 power-on, 1 software, 9 the reset pin, ...
extern uint32_t g_boot_running_slot, g_boot_next_slot;   // the app slot running, the one the bootloader boots next
#endif

}  // namespace oep
