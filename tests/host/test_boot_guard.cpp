// Host tests: BootGuard (OepBootGuard.h) - fast crash-boots counted over resets, a safe boot after kSafeAfter of them in
// a row, the count back to 0 after a boot up kStableMs, a restart on purpose, a reset that was no crash or a record a
// power-on left; the USB gate of the at-boot attach (configured for kUsbSettleMs, or kAttachGraceMs without a host).
#include <stdio.h>

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

  printf("boot_guard: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
