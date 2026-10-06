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
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#endif

#include <stdio.h>

namespace oep {
namespace {

constexpr uint32_t kRunning = 0x4f45b007u;   // the record while a boot is not yet stable: a reset now was a crash
constexpr uint32_t kSettled = 0x4f455354u;   // stable
constexpr uint32_t kPlanned = 0x4f45504cu;   // a restart on purpose (planned): the next boot's reset is no crash

uint8_t gCrashes = 0;
bool gSettled = false, gUsbSeen = false, gAttachOpen = false;
uint32_t gUsbSince = 0;
volatile uint32_t gBeatMs = 0;   // loop()'s last round (RP2: read by the feeding timer's interrupt)
char gLastBoot[72] = "";          // lastBoot()

// What a reset leaves for the next boot: the state (kRunning / kSettled / kPlanned), the fast crash-boots so far, the
// seconds the boot was up (poll) and the slot a planned restart was to boot (an update just written). Slots: the
// partition's flash address, 0 none.
struct Record { uint32_t state, count, up_s, next; };
Record gRecord{};

// What reset the chip before this boot.
enum class Reset : uint8_t { kPowerOn, kSoftware, kPanic, kIntWdt, kTaskWdt, kWdt, kBrownout, kUsb, kJtag, kPin, kLockup, kOther };
const char *resetName(Reset r) {
  static const char *const kNames[] = {"power-on", "software", "panic", "int-wdt", "task-wdt", "wdt",
                                       "brownout", "usb", "jtag", "reset-pin", "cpu-lockup", "other"};
  return kNames[static_cast<uint8_t>(r)];
}

// ---- the record a reset leaves, what the reset was, the app slots ----
#if defined(ARDUINO_ARCH_RP2040)
// Watchdog scratch 0 / 1 / 2 (4-7 are the boot ROM's): kept over a watchdog reset, cleared by power-on and the RUN pin.
bool recordRead(Record &r) {
  r = {watchdog_hw->scratch[0], watchdog_hw->scratch[1], watchdog_hw->scratch[2], 0};
  return true;
}
void recordWrite(const Record &r) {
  watchdog_hw->scratch[0] = r.state;
  watchdog_hw->scratch[1] = r.count;
  watchdog_hw->scratch[2] = r.up_s;
}
// Every crash path ends in a watchdog reset (crashed(), a timeout); so does rp2040.reboot(), which planned() marks.
Reset resetKind() { return watchdog_caused_reboot() ? Reset::kWdt : Reset::kPowerOn; }
bool crashReset(Reset r) { return r == Reset::kWdt; }
bool plannedReset(Reset r) { return r == Reset::kWdt; }
uint32_t runningSlot() { return 0; }
uint32_t bootSlot() { return 0; }
const char *slotName(uint32_t) { return "?"; }
#elif defined(ARDUINO_ARCH_ESP32)
RTC_NOINIT_ATTR Record gKept;
RTC_NOINIT_ATTR uint32_t gKeptCheck;
uint32_t recordCheck(const Record &r) { return r.state ^ r.count ^ r.up_s ^ r.next ^ 0xa5a5a5a5u; }
bool recordRead(Record &r) {
  r = gKept;
  return gKeptCheck == recordCheck(r);   // power-on leaves the memory as it comes up
}
void recordWrite(const Record &r) {
  gKept = r;
  gKeptCheck = recordCheck(r);
}
Reset resetKind() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return Reset::kPowerOn;
    case ESP_RST_SW: return Reset::kSoftware;
    case ESP_RST_PANIC: return Reset::kPanic;
    case ESP_RST_INT_WDT: return Reset::kIntWdt;
    case ESP_RST_TASK_WDT: return Reset::kTaskWdt;
    case ESP_RST_WDT: return Reset::kWdt;
    case ESP_RST_BROWNOUT: return Reset::kBrownout;
    case ESP_RST_EXT: return Reset::kPin;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
    case ESP_RST_USB: return Reset::kUsb;
    case ESP_RST_JTAG: return Reset::kJtag;
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    case ESP_RST_CPU_LOCKUP: return Reset::kLockup;
#endif
    default: return Reset::kOther;
  }
}
bool crashReset(Reset r) {
  return r == Reset::kPanic || r == Reset::kIntWdt || r == Reset::kTaskWdt || r == Reset::kWdt || r == Reset::kLockup;
}
bool plannedReset(Reset r) { return r == Reset::kSoftware; }   // esp_restart
uint32_t runningSlot() {
  const esp_partition_t *p = esp_ota_get_running_partition();
  return p ? p->address : 0;
}
uint32_t bootSlot() {
  const esp_partition_t *p = esp_ota_get_boot_partition();
  return p ? p->address : 0;
}
const char *slotName(uint32_t address) {   // the app partition's label
  for (esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, nullptr); it;
       it = esp_partition_next(it)) {   // the last next() releases the iterator
    const esp_partition_t *p = esp_partition_get(it);
    if (p->address == address) {
      esp_partition_iterator_release(it);
      return p->label;
    }
  }
  return "?";
}
#elif defined(OEP_HOST_FAKE_BOOT)
bool recordRead(Record &r) {
  r = {g_boot_record.state, g_boot_record.count, g_boot_record.up_s, g_boot_record.next};
  return g_boot_record.valid;
}
void recordWrite(const Record &r) { g_boot_record = {true, r.state, r.count, r.up_s, r.next}; }
Reset resetKind() { return g_boot_reset_crash ? Reset::kPanic : static_cast<Reset>(g_boot_reset_kind); }
bool crashReset(Reset r) { return r == Reset::kPanic || r == Reset::kTaskWdt || r == Reset::kWdt; }
bool plannedReset(Reset r) { return r == Reset::kSoftware; }
uint32_t runningSlot() { return g_boot_running_slot; }
uint32_t bootSlot() { return g_boot_next_slot; }
const char *slotName(uint32_t address) { return address == 0x10000 ? "app0" : address == 0x150000 ? "app1" : "?"; }
#else
bool recordRead(Record &) { return false; }
void recordWrite(const Record &) {}
Reset resetKind() { return Reset::kPowerOn; }
bool crashReset(Reset) { return false; }
bool plannedReset(Reset) { return false; }
uint32_t runningSlot() { return 0; }
uint32_t bootSlot() { return 0; }
const char *slotName(uint32_t) { return "?"; }
#endif

// What ended the boot before this one, as lastBoot() says it ("" for a power-on or a restart on purpose).
void describeLastBoot(bool valid, const Record &before, Reset reset) {
  gLastBoot[0] = 0;
  if (!valid) return;
  const uint32_t running = runningSlot();
  // An update written and the bootloader went back to the image before: the new one failed the bootloader's check, or
  // a reset came before the ESP32 core confirmed it (at its start, before setup(): this boot's begin() never ran).
  if (before.state == kPlanned && before.next && running != before.next) {
    snprintf(gLastBoot, sizeof gLastBoot, "update to %s did not reach setup: %s", slotName(before.next), resetName(reset));
  } else if (reset != Reset::kPowerOn && !(before.state == kPlanned && plannedReset(reset))) {
    snprintf(gLastBoot, sizeof gLastBoot, "%s at %lu s", resetName(reset), static_cast<unsigned long>(before.up_s));
  }
}

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
uint8_t g_boot_reset_kind = 0;
uint32_t g_boot_running_slot = 0, g_boot_next_slot = 0;
#endif

void BootGuard::begin() {
  Record before{};
  const bool valid = recordRead(before);
  const Reset reset = resetKind();
  const bool crash = valid && before.state == kRunning && crashReset(reset);
  gCrashes = crash ? static_cast<uint8_t>(before.count >= 254 ? 255 : before.count + 1) : 0;
  describeLastBoot(valid, before, reset);
  gSettled = gUsbSeen = gAttachOpen = false;
  gUsbSince = 0;
  gRecord = {kRunning, gCrashes, 0, 0};
  recordWrite(gRecord);
  gBeatMs = millis();
  startWatchdogs();
}

uint8_t BootGuard::crashes() { return gCrashes; }

const char *BootGuard::lastBoot() { return gLastBoot; }

void BootGuard::poll() {
  const uint32_t now = millis();
  gBeatMs = now;
  bool write = false;
  if (now / 1000 != gRecord.up_s) {   // the seconds up, for the next boot to tell when this one ended
    gRecord.up_s = now / 1000;
    write = true;
  }
  if (!gSettled && now >= kStableMs) {
    gSettled = true;
    gRecord.state = kSettled;
    gRecord.count = 0;
    write = true;
  }
  if (write) recordWrite(gRecord);
}

void BootGuard::planned() {
  gRecord.state = kPlanned;
  gRecord.count = 0;
  const uint32_t next = bootSlot();
  gRecord.next = next != runningSlot() ? next : 0;   // an update written: the slot the restart is to boot
  recordWrite(gRecord);
}

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
