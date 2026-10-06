// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepBootGuard.h"

#if defined(ARDUINO_ARCH_RP2040)
#include <hardware/resets.h>
#include <hardware/structs/usb.h>
#include <hardware/structs/watchdog.h>
#include <hardware/sync.h>
#include <hardware/watchdog.h>
#include <pico/time.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <esp_attr.h>
#include <esp_idf_version.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#endif

namespace oep {
namespace {

constexpr uint32_t kRunning = 0x4f45b007u;   // the record while a boot is not yet stable: a reset now was a crash
constexpr uint32_t kSettled = 0x4f455354u;   // stable, or a restart on purpose

uint8_t gCrashes = 0;
bool gSettled = false, gUsbSeen = false, gAttachOpen = false;
uint32_t gUsbSince = 0;
volatile uint32_t gBeatMs = 0;   // loop()'s last round (RP2: read by the feeding timer's interrupt)

// ---- the record a reset leaves, and what the reset was ----
#if defined(ARDUINO_ARCH_RP2040)
// Watchdog scratch 0 / 1 (4-7 are the boot ROM's): kept over a watchdog reset, cleared by power-on and the RUN pin.
bool recordRead(uint32_t &state, uint32_t &count) {
  state = watchdog_hw->scratch[0];
  count = watchdog_hw->scratch[1];
  return true;
}
void recordWrite(uint32_t state, uint32_t count) {
  watchdog_hw->scratch[0] = state;
  watchdog_hw->scratch[1] = count;
}
// Every crash path ends in a watchdog reset (crashed(), a timeout); so does rp2040.reboot(), which planned() marks.
bool resetWasCrash() { return watchdog_caused_reboot(); }
#elif defined(ARDUINO_ARCH_ESP32)
RTC_NOINIT_ATTR uint32_t gRecordState, gRecordCount, gRecordCheck;
bool recordRead(uint32_t &state, uint32_t &count) {
  state = gRecordState;
  count = gRecordCount;
  return gRecordCheck == (state ^ count ^ 0xa5a5a5a5u);   // power-on leaves the memory as it comes up
}
void recordWrite(uint32_t state, uint32_t count) {
  gRecordState = state;
  gRecordCount = count;
  gRecordCheck = state ^ count ^ 0xa5a5a5a5u;
}
bool resetWasCrash() {
  switch (esp_reset_reason()) {
    case ESP_RST_PANIC: case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT:
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    case ESP_RST_CPU_LOCKUP:
#endif
      return true;
    default:
      return false;   // power-on, the reset pin, esp_restart (a restart on purpose), a brownout, ...
  }
}
#elif defined(OEP_HOST_FAKE_BOOT)
bool recordRead(uint32_t &state, uint32_t &count) {
  state = g_boot_record.state;
  count = g_boot_record.count;
  return g_boot_record.valid;
}
void recordWrite(uint32_t state, uint32_t count) { g_boot_record = {true, state, count}; }
bool resetWasCrash() { return g_boot_reset_crash; }
#else
bool recordRead(uint32_t &, uint32_t &) { return false; }
void recordWrite(uint32_t, uint32_t) {}
bool resetWasCrash() { return false; }
#endif

// ---- the watchdogs ----
#if defined(ARDUINO_ARCH_RP2040)
repeating_timer_t gFeed;
bool feed(repeating_timer_t *) {
  if (static_cast<uint32_t>(time_us_64() / 1000u) - gBeatMs > BootGuard::kStallMs) BootGuard::crashed();   // loop() stuck
  watchdog_update();
  return true;
}
void startWatchdogs() {
  watchdog_enable(BootGuard::kWatchdogMs, true);   // paused while a debugger holds a core
  add_repeating_timer_ms(-static_cast<int32_t>(BootGuard::kFeedMs), feed, nullptr, &gFeed);
}
#elif defined(ARDUINO_ARCH_ESP32)
void startWatchdogs() {
  // the task watchdog's timeout raised to kStallMs (it is 5 s, shorter than one request may take), the idle tasks it
  // watches as the core set them, and loop() added: the core's loopTask feeds it on every round
  esp_task_wdt_config_t config = {};
  config.timeout_ms = BootGuard::kStallMs;
  config.idle_core_mask = 0;
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
  config.idle_core_mask |= 1u << 0;
#endif
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
  config.idle_core_mask |= 1u << 1;
#endif
  config.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&config) == ESP_OK) enableLoopWDT();
}
#else
void startWatchdogs() {}
#endif

}  // namespace

#if defined(OEP_HOST_FAKE_BOOT)
FakeBootRecord g_boot_record;
bool g_boot_reset_crash = false;
#endif

void BootGuard::begin() {
  uint32_t state = 0, count = 0;
  const bool crash = recordRead(state, count) && state == kRunning && resetWasCrash();
  gCrashes = crash ? static_cast<uint8_t>(count >= 254 ? 255 : count + 1) : 0;
  gSettled = gUsbSeen = gAttachOpen = false;
  gUsbSince = 0;
  recordWrite(kRunning, gCrashes);
  gBeatMs = millis();
  startWatchdogs();
}

uint8_t BootGuard::crashes() { return gCrashes; }

void BootGuard::poll() {
  const uint32_t now = millis();
  gBeatMs = now;
  if (!gSettled && now >= kStableMs) {
    gSettled = true;
    recordWrite(kSettled, 0);
  }
}

void BootGuard::planned() { recordWrite(kSettled, 0); }

#if defined(ARDUINO_ARCH_RP2040)
void BootGuard::crashed() {
  (void)save_and_disable_interrupts();
  // off the bus (the pull-up off) for the host to see an unplug, unless the controller is still held in reset
  if (!(resets_hw->reset & RESETS_RESET_USBCTRL_BITS)) hw_clear_bits(&usb_hw->sie_ctrl, USB_SIE_CTRL_PULLUP_EN_BITS);
  busy_wait_us_32(kRestartDetachMs * 1000u);
  watchdog_reboot(0, 0, 0);   // at once (the record still says running: this boot counts as a crash)
  for (;;) {}
}
#elif defined(ARDUINO_ARCH_ESP32)
void BootGuard::crashed() { abort(); }   // the panic handler resets the chip
#endif

bool BootGuard::attachReady(bool configured) {
  if (gAttachOpen) return true;
  const uint32_t now = millis();
  if (!configured) gUsbSeen = false;
  else if (!gUsbSeen) { gUsbSeen = true; gUsbSince = now; }
  gAttachOpen = (gUsbSeen && now - gUsbSince >= kUsbSettleMs) || now >= kAttachGraceMs;
  return gAttachOpen;
}

}  // namespace oep
