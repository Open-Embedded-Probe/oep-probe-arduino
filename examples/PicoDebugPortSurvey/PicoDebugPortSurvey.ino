// What is on the wires: a bring-up survey for the Pico bench, for either board.
//
// Three passes, all read-only apart from briefly driving a pin:
//   1. which pins are tied to something, told apart from the RP2350's floating-input latch
//      (erratum E9 makes a released input read high) by applying the pad's own pull;
//   2. ordinary ARM SWD over every ordered pin pair, and in detail on the wired pair, across
//      the three wake sequences (JTAG-to-SWD, dormant-to-SWD, plain line reset) and the
//      multidrop TARGETSEL candidates;
//   3. CH32 RVSWD over every ordered pair, since the same two wires carry a different frame.
//
// On the RP2040-Zero this found the Pro Micro RP2350's debug port on SWCLK=GP0, SWDIO=GP1
// (DPIDR 0x4c013477) on 2026-09-23.
#include <initializer_list>

#include <OepRp2BitBang.h>
#include <OepRvswdPhy.h>
#include <OepSwdFrame.h>

using oep::rp2::BitBang;
namespace swd = oep::swd;

static const uint32_t kTargets[] = {
    0x01002927u, 0x11002927u, 0xf1002927u,   // RP2040 core 0 / core 1 / rescue
    0x00040927u, 0x10040927u, 0xf0040927u,   // Raspberry Pi designer, part 0x0004
    0x01040927u, 0x11040927u, 0xf1040927u,   // same with the RP2040's instance nibbles
    0x00002927u, 0x10002927u,
};
static const char *kTargetNames[] = {"rp2040-core0", "rp2040-core1", "rp2040-rescue",
                                     "0x00040927", "0x10040927", "0xf0040927",
                                     "0x01040927", "0x11040927", "0xf1040927",
                                     "0x00002927", "0x10002927"};

// How the port is woken before the transfer.
enum Wake : uint8_t { kWakeJtagToSwd = 0, kWakeDormant = 1, kWakeLineResetOnly = 2 };
static const char *kWakeNames[] = {"jtag->swd", "dormant->swd", "line reset"};

static void wake(BitBang &b, uint8_t how) {
  if (how == kWakeDormant) swd::dormantToSwd(b);
  else if (how == kWakeJtagToSwd) swd::jtagToSwd(b);
  else swd::lineReset(b);
}

// The pins each board brings out. The Pro Micro keeps GP19 for its PSRAM chip select.
#if defined(ARDUINO_SPARKFUN_PROMICRO_RP2350)
static const uint8_t kCandidates[] = {0, 1, 4, 5, 6, 7, 8, 9, 16, 17, 18, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29};
#else
static const uint8_t kCandidates[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 26, 27, 28, 29};
#endif
static constexpr size_t kCandidateCount = sizeof kCandidates / sizeof kCandidates[0];

static BitBang io;

static bool tryOne(int dio, int clk, uint32_t half_ns, uint8_t how, const uint32_t *target, const char *name) {
  if (!io.setup(dio, clk)) return false;
  io.setHalfNs(half_ns);
  io.driveBoth();
  wake(io, how);
  if (target) swd::targetSelect(io, *target);
  uint32_t dpidr = 0;
  const uint8_t ack = swd::transfer(io, false, true, 0x0, dpidr);
  io.releaseBoth();
  const bool good = ack == swd::kOk && dpidr != 0 && dpidr != 0xffffffffu;
  Serial.printf("  SWDIO=GP%-2d SWCLK=GP%-2d half=%4u ns %-12s targetsel=%-13s ack=%u dpidr=0x%08lx%s\n",
                dio, clk, (unsigned)half_ns, kWakeNames[how], name, ack, (unsigned long)dpidr,
                good ? "   <== answered" : "");
  return good;
}

// Which pins are wired to something, before trying any protocol on them. A CH32 or an Arm
// target holds SWCLK down and SWDIO up with internal pulls. The pad's own pull is applied
// while reading, because on the RP2350 a released input latches high (erratum E9) and would
// otherwise look like an external pull-up everywhere.
static void pullSignature() {
  Serial.println("pin pull signature (our own pull applied, so the RP2350 latch cannot lie):");
  for (uint8_t pin : kCandidates) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_down(pin); delay(2);
    const bool high_against_pulldown = gpio_get(pin);
    gpio_pull_up(pin); delay(2);
    const bool low_against_pullup = !gpio_get(pin);
    gpio_disable_pulls(pin);
    const char *verdict = high_against_pulldown ? "external pull-up   <== SWDIO / SDA candidate"
                        : low_against_pullup    ? "external pull-down <== SWCLK candidate"
                                                : "follows our pull (nothing attached)";
    Serial.printf("  GP%-2d  %s\n", pin, verdict);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}
  delay(200);
  Serial.println("pico debug port survey");
  Serial.printf("spin loop: %u ps per iteration, %u candidate pins\n",
                (unsigned)oep::rp2::loopPicoseconds(), (unsigned)kCandidateCount);
}

// Quiet pass over many ordered pairs: one half period and the two most likely TARGETSEL
// settings, reporting only a pair that answers.
static bool sweepPairs() {
  Serial.println("wide pair sweep (only answering pairs are printed):");
  bool any = false;
  for (uint8_t dio : kCandidates) {
    for (uint8_t clk : kCandidates) {
      if (dio == clk) continue;
      if (!io.setup(dio, clk)) continue;
      io.setHalfNs(500);
      io.driveBoth();
      for (int attempt = 0; attempt < 4; ++attempt) {
        wake(io, attempt < 2 ? kWakeJtagToSwd : kWakeDormant);
        if (attempt == 1) swd::targetSelect(io, 0x01002927u);
        if (attempt == 3) swd::targetSelect(io, 0x00040927u);
        uint32_t dpidr = 0;
        const uint8_t ack = swd::transfer(io, false, true, 0x0, dpidr);
        if (ack == swd::kOk && dpidr != 0 && dpidr != 0xffffffffu) {
          Serial.printf("  SWDIO=GP%-2d SWCLK=GP%-2d attempt %d ack=%u dpidr=0x%08lx  <== answered\n",
                        dio, clk, attempt, ack, (unsigned long)dpidr);
          any = true;
        }
      }
      io.releaseBoth();
      pinMode(dio, INPUT);
      pinMode(clk, INPUT);
    }
  }
  if (!any) Serial.println("  none of the 380 ordered pairs answered");
  return any;
}

// The same wires might carry a CH32 debug port rather than an ARM one: RVSWD uses the
// same two lines with a different frame, and its attach reports DMSTATUS.
static bool tryRvswd(int dio, int clk, bool verbose) {
  oep::RvswdPhy phy;
  if (!phy.begin(dio, clk)) return false;
  uint32_t dmstatus = 0;
  bool ok = false;
  for (uint32_t half : {100u, 500u}) if ((ok = phy.probeOnce(half, dmstatus))) break;
  phy.release();
  pinMode(dio, INPUT);
  pinMode(clk, INPUT);
  if (verbose || ok)
    Serial.printf("  RVSWD SWDIO=GP%-2d SWCLK=GP%-2d %s dmstatus=0x%08lx%s\n", dio, clk,
                  ok ? "answered" : "silent  ", (unsigned long)dmstatus,
                  ok ? "   <== a CH32 debug module" : "");
  return ok;
}

// Every ordered pair, reporting only the ones that answer.
static bool sweepRvswdPairs(const uint8_t *pins, size_t count) {
  Serial.println("RVSWD pair sweep (only answering pairs are printed):");
  bool any = false;
  for (size_t i = 0; i < count; ++i)
    for (size_t j = 0; j < count; ++j)
      if (i != j) any |= tryRvswd(pins[i], pins[j], false);
  if (!any) Serial.println("  no ordered pair answered");
  return any;
}

void loop() {
  static uint32_t round = 0;
  Serial.printf("---- sweep %lu\n", (unsigned long)++round);
  pullSignature();
  bool any = sweepPairs();
  Serial.println("detail on GP0/GP1: both orientations x wake sequence x targetsel");
  for (int swap = 0; swap < 2; ++swap) {
    const int dio = swap ? 1 : 0, clk = swap ? 0 : 1;
    for (uint8_t how = 0; how < 3; ++how) {
      any |= tryOne(dio, clk, 500, how, nullptr, "none");
      for (size_t i = 0; i < sizeof kTargets / sizeof kTargets[0]; ++i)
        any |= tryOne(dio, clk, 500, how, &kTargets[i], kTargetNames[i]);
    }
  }
  Serial.println("CH32 RVSWD on the same wires:");
  for (int swap = 0; swap < 2; ++swap) any |= tryRvswd(swap ? 1 : 0, swap ? 0 : 1, true);
  any |= sweepRvswdPairs(kCandidates, kCandidateCount);
  Serial.println(any ? "swd_survey: a debug port answered" : "swd_survey: no answer on any combination");
  delay(8000);
}
