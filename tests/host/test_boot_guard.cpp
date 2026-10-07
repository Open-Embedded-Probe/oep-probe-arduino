// Host tests: BootGuard (OepBootGuard.h) - fast crash-boots counted over resets, a safe boot after kSafeAfter of them in
// a row, the count back to 0 after a boot up kStableMs, a restart on purpose, a reset that was no crash or a record a
// power-on left; the USB gate of the at-boot attach (configured for kUsbSettleMs, or kAttachGraceMs without a host);
// lastBoot(), what ended the boot before: nothing for a power-on or a restart on purpose, the reset and the seconds up,
// an update the bootloader did not start; an update that started is not undone by any later reset.
#include <stdio.h>

#include <initializer_list>
#include <string.h>

#include "OepBootGuard.h"

uint32_t g_millis = 0;
void (*g_on_wait)() = nullptr;

using namespace oep;

static int failures = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    ++checks;                                                                    \
    if (!(cond)) { ++failures; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// A reset: the clock from 0, the record as the reset left it, the reset a crash or not; then the boot's begin().
static void boot(bool crash) {
  g_millis = 0;
  g_boot_reset_crash = crash;
  BootGuard::begin();
}
// The boot running for `ms` with loop() coming round.
static void run(uint32_t ms) {
  for (uint32_t t = 0; t < ms; t += 100) { g_millis += 100; BootGuard::poll(); }
}

// lastBoot(): what the reset before this boot was, the seconds the boot before was up, and the app slots (app0 0x10000,
// app1 0x150000 in the fake).
static void testLastBoot() {
  auto reset = [](uint8_t kind, uint32_t running) {
    g_millis = 0;
    g_boot_reset_crash = false;
    g_boot_reset_kind = kind;   // 0 power-on, 1 software, 2 panic, 4 task-wdt, 9 the reset pin
    g_boot_running_slot = running;
    g_boot_next_slot = running;
    BootGuard::begin();
  };
  auto is = [](const char *text) { return strcmp(BootGuard::lastBoot(), text) == 0; };
  g_boot_record = {};
  reset(0, 0x10000);
  CHECK(is(""));                              // power-on
  run(12000);
  reset(2, 0x10000);
  CHECK(is("panic at 12 s") && BootGuard::crashes() == 1);
  run(5000);
  reset(1, 0x10000);
  CHECK(is(""));                              // a restart the probe did not make: from outside, not a crash
  run(3000);
  BootGuard::planned();
  reset(1, 0x10000);
  CHECK(is(""));                              // oep.probe.restart
  // a DFU update into app1 that the bootloader did not start
  run(40000);
  g_boot_next_slot = 0x150000;
  BootGuard::planned();
  reset(1, 0x10000);
  CHECK(is("update to app1 did not reach setup: software"));
  // a DFU update into app1 that started (confirmed as it starts), then the task watchdog at 12 s: still app1, the
  // reset said as any other and counted as a fast crash-boot
  run(40000);
  g_boot_next_slot = 0x150000;
  BootGuard::planned();
  reset(1, 0x150000);
  CHECK(is(""));
  run(12000);
  reset(4, 0x150000);
  CHECK(is("task-wdt at 12 s") && BootGuard::crashes() == 1);
  // then the reset pin at 40 s
  run(40000);
  reset(9, 0x150000);
  CHECK(is("") && BootGuard::crashes() == 0);   // the reset pin (esptool's DTR / RTS): from outside
  // the other resets from outside: no note, not counted (kinds 5 wdt, 7 usb, 8 jtag)
  for (uint8_t kind : {uint8_t(5), uint8_t(7), uint8_t(8)}) {
    run(3000);
    reset(kind, 0x150000);
    CHECK(is("") && BootGuard::crashes() == 0);
  }
  // a brownout is told (not a crash: not counted)
  run(7000);
  reset(6, 0x150000);
  CHECK(is("brownout at 7 s") && BootGuard::crashes() == 0);
  // RP2: every reboot is a watchdog reset; only crashed()'s mark makes it a crash (picotool load -x, 0.0.29-dev
  // 8bcecca: "wdt at 5 s" after a plain load)
  run(5000);
  reset(5, 0x150000);
  CHECK(is("") && BootGuard::crashes() == 0);   // picotool / the boot ROM: unmarked
  run(4000);
  BootGuard::crashed();                          // a panic, a HardFault, loop() stalled: marked, then the reset
  reset(5, 0x150000);
  CHECK(is("crash at 4 s") && BootGuard::crashes() == 1);
  run(2000);
  BootGuard::crashed();
  reset(5, 0x150000);
  CHECK(is("crash at 2 s") && BootGuard::crashes() == 2);   // fast: counted on
  run(BootGuard::kStableMs + 2000);
  BootGuard::crashed();                          // after kStableMs: told, but not a fast one
  reset(5, 0x150000);
  CHECK(is("crash at 32 s") && BootGuard::crashes() == 0);
  run(1000);
  reset(5, 0x150000);                            // the mark is gone with the boot that read it
  CHECK(is("") && BootGuard::crashes() == 0);
}

int main() {
  // power-on: no record
  g_boot_record = {};
  boot(false);
  CHECK(BootGuard::crashes() == 0 && !BootGuard::safe());
  // crashes within kStableMs of each boot, counted; the kSafeAfter-th makes a safe boot
  for (uint8_t n = 1; n <= BootGuard::kSafeAfter; ++n) {
    run(2000);
    boot(true);
    CHECK(BootGuard::crashes() == n);
    CHECK(BootGuard::safe() == (n >= BootGuard::kSafeAfter));
  }
  // the safe boot crashing fast too stays safe
  run(1000);
  boot(true);
  CHECK(BootGuard::safe() && BootGuard::crashes() == BootGuard::kSafeAfter + 1);
  // a boot up kStableMs ends the run: a crash after it is not a fast one
  run(BootGuard::kStableMs + 100);
  boot(true);
  CHECK(BootGuard::crashes() == 0 && !BootGuard::safe());
  // a reset that was no crash (power-on, the reset pin, esp_restart) starts again at 0, whatever the record says
  run(1000);
  boot(true);
  CHECK(BootGuard::crashes() == 1);
  run(1000);
  boot(false);
  CHECK(BootGuard::crashes() == 0);
  // a restart on purpose (planned) within kStableMs: not counted, even through a watchdog reset (RP2's rp2040.reboot)
  run(500);
  boot(true);
  run(500);
  BootGuard::planned();
  boot(true);
  CHECK(BootGuard::crashes() == 0);
  // a record a power-on left unreadable: 0
  run(500);
  g_boot_record.valid = false;
  boot(true);
  CHECK(BootGuard::crashes() == 0);
  // the count saturates
  g_boot_record = {true, 0x4f45b007u, 300};
  boot(true);
  CHECK(BootGuard::crashes() == 255 && BootGuard::safe());

  // ---- the USB gate ----
  boot(false);
  CHECK(!BootGuard::attachReady(false));
  g_millis = 800;
  CHECK(!BootGuard::attachReady(true));   // configured: kUsbSettleMs from now
  g_millis = 800 + BootGuard::kUsbSettleMs - 1;
  CHECK(!BootGuard::attachReady(true));
  g_millis = 800 + BootGuard::kUsbSettleMs;
  CHECK(BootGuard::attachReady(true));
  CHECK(BootGuard::attachReady(false));   // open for the rest of the boot
  boot(false);
  g_millis = 500;
  CHECK(!BootGuard::attachReady(true));
  g_millis = 1000;
  CHECK(!BootGuard::attachReady(false));   // the host let go: the settle starts again
  g_millis = 1200;
  CHECK(!BootGuard::attachReady(true));
  g_millis = 1200 + BootGuard::kUsbSettleMs - 1;
  CHECK(!BootGuard::attachReady(true));
  g_millis = 1200 + BootGuard::kUsbSettleMs;
  CHECK(BootGuard::attachReady(true));
  boot(false);   // no host at all: kAttachGraceMs
  g_millis = BootGuard::kAttachGraceMs - 1;
  CHECK(!BootGuard::attachReady(false));
  g_millis = BootGuard::kAttachGraceMs;
  CHECK(BootGuard::attachReady(false));

  // ---- kStableMs: a crash just before it is a fast one, just after it not ----
  g_boot_record = {};
  boot(false);
  run(BootGuard::kStableMs - 100);
  boot(true);
  CHECK(BootGuard::crashes() == 1);
  run(BootGuard::kStableMs);
  boot(true);
  CHECK(BootGuard::crashes() == 0);
  // a stall in the at-boot attach (gate open at the latest kAttachGraceMs + kUsbSettleMs, reset kStallMs later) is fast
  CHECK(BootGuard::kStableMs > BootGuard::kAttachGraceMs + BootGuard::kUsbSettleMs + BootGuard::kStallMs);

  testLastBoot();
  printf("boot_guard: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
