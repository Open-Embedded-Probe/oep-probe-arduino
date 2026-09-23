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
// Mirrors the probe firmware's ordering: begin() once in setup(), attach() much later.
static oep::RvswdPhy gEarlyPhy;

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
#if defined(ARDUINO_SPARKFUN_PROMICRO_RP2350)
  Serial.printf("early begin(0,1) in setup(): %s\n", gEarlyPhy.begin(0, 1) ? "ok" : "failed");
#endif
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

// How good the link actually is on a known pair: how often a fresh bus init answers, and
// how many retries steady-state reads need once it has.
static void linkQuality(int dio, int clk) {
  Serial.printf("RVSWD link quality on SWDIO=GP%d SWCLK=GP%d:\n", dio, clk);
  oep::RvswdPhy phy;
  if (!phy.begin(dio, clk)) { Serial.println("  begin failed"); return; }
  for (uint32_t half : {0u, 25u, 50u, 100u, 200u, 500u, 1000u, 2000u}) {
    uint32_t dm = 0;
    int fresh = 0;
    for (int i = 0; i < 50; ++i) if (phy.probeOnce(half, dm)) ++fresh;
    int steady_ok = 0;
    uint32_t retries_before = phy.retries(), first = 0;
    bool stable = true;
    if (phy.probeOnce(half, dm)) {
      for (int i = 0; i < 200; ++i) {
        uint32_t v = 0;
        if (!phy.read(0x11, v)) break;
        if (i == 0) first = v; else if (v != first) stable = false;
        ++steady_ok;
      }
    }
    Serial.printf("  half=%4u ns  fresh init %2d/50   steady reads %3d/200 retries %4lu %s dmstatus=0x%08lx\n",
                  (unsigned)half, fresh, steady_ok, (unsigned long)(phy.retries() - retries_before),
                  stable ? "stable" : "VARIED", (unsigned long)dm);
  }
  // The path the probe firmware actually takes: attach() picks a half period with its own
  // margin check and leaves the bus driven, then the services read through it.
  phy.release();
  oep::RvswdPhy phy2;
  phy2.begin(dio, clk);
  const bool attached = phy2.attach();
  Serial.printf("  attach() -> %s, half=%lu ns, one DMI read %lu ns\n", attached ? "yes" : "no",
                (unsigned long)phy2.halfNs(), (unsigned long)phy2.dmiNs());
  if (attached) {
    uint32_t before = phy2.retries(), first = 0;
    int ok = 0; bool stable = true;
    for (int i = 0; i < 200; ++i) {
      uint32_t v = 0;
      if (!phy2.read(0x11, v)) break;
      if (i == 0) first = v; else if (v != first) stable = false;
      ++ok;
    }
    Serial.printf("  after attach(): %3d/200 reads, retries %lu, %s dmstatus=0x%08lx\n",
                  ok, (unsigned long)(phy2.retries() - before), stable ? "stable" : "VARIED", (unsigned long)first);
  }
  phy2.release();
  pinMode(dio, INPUT); pinMode(clk, INPUT);
}

void loop() {
  static uint32_t round = 0;
  Serial.printf("---- sweep %lu\n", (unsigned long)++round);
#if defined(ARDUINO_SPARKFUN_PROMICRO_RP2350)
  {   // why does a cold attach() fail? Run its own check by hand and count the outcomes.
    oep::RvswdPhy d;
    d.begin(0, 1);
    Serial.println("cold bus: how many wake attempts before the debug module answers?");
    for (uint32_t half : {500u, 200u, 100u}) {
      int first_ok = -1, ok = 0;
      for (int i = 0; i < 100; ++i) {
        uint32_t dm = 0;
        if (d.probeOnce(half, dm)) { if (first_ok < 0) first_ok = i; ++ok; }
      }
      Serial.printf("  half=%4u ns  first success at attempt %d, %d/100 total\n", (unsigned)half, first_ok, ok);
    }
    Serial.println("cold attach, counted per half period (what attach() requires: 1000 identical clean reads):");
    for (uint32_t half : {0u, 25u, 50u, 100u, 200u, 500u}) {
      uint32_t dm = 0;
      if (!d.probeOnce(half, dm, true)) { Serial.printf("  half=%4u ns  first read failed\n", (unsigned)half); continue; }
      int same = 0, differed = 0, dirty = 0;
      const uint32_t first = dm;
      for (int i = 0; i < 1000; ++i) {
        uint32_t v = 0;
        if (!d.readOnce(0x11, v)) ++dirty; else if (v != first) ++differed; else ++same;
      }
      Serial.printf("  half=%4u ns  first=0x%08lx  identical %4d  differed %4d  parity-fail %4d\n",
                    (unsigned)half, (unsigned long)first, same, differed, dirty);
    }
    d.release();
  }
  {   // does the hart halt? write DMCONTROL.haltreq and watch DMSTATUS
    oep::RvswdPhy h;
    h.begin(0, 1);
    if (h.attach()) {
      uint32_t v = 0;
      h.read(0x11, v);
      Serial.printf("halt test: attached at %lu ns, DMSTATUS 0x%08lx\n", (unsigned long)h.halfNs(), (unsigned long)v);
      h.write(0x10, 0x80000001);          // haltreq | dmactive
      for (int i = 0; i < 12; ++i) {
        delay(10);
        uint32_t st = 0;
        const bool ok = h.read(0x11, st);
        Serial.printf("  +%3d ms DMSTATUS %s0x%08lx  allhalted=%lu anyrunning=%lu havereset=%lu\n",
                      (i + 1) * 10, ok ? "" : "(read failed) ", (unsigned long)st,
                      (unsigned long)((st >> 9) & 1), (unsigned long)((st >> 11) & 1), (unsigned long)((st >> 19) & 1));
        if ((st >> 9) & 1) break;
      }
      h.write(0x10, 0x90000001);          // ackhavereset | haltreq | dmactive, then try again
      delay(20);
      uint32_t st2 = 0; h.read(0x11, st2);
      Serial.printf("  after ackhavereset+haltreq: DMSTATUS 0x%08lx allhalted=%lu\n",
                    (unsigned long)st2, (unsigned long)((st2 >> 9) & 1));
      h.write(0x10, 0x00000001);          // leave it running
    } else {
      Serial.println("halt test: attach failed");
    }
    h.release();
  }
  {   // cold contact, before anything else has touched the bus this round
    oep::RvswdPhy cold;
    const bool b = cold.begin(0, 1);
    const bool a = b && cold.attach();
    uint32_t v = 0;
    const bool r = a && cold.read(0x11, v);
    Serial.printf("cold attach at the top of the round: begin=%s attach=%s half=%lu read=%s dmstatus=0x%08lx\n",
                  b ? "ok" : "no", a ? "yes" : "no", (unsigned long)cold.halfNs(), r ? "ok" : "FAILED", (unsigned long)v);
    cold.release();
  }
#endif
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
#if defined(ARDUINO_SPARKFUN_PROMICRO_RP2350)
  {   // the same object begun back in setup(): the global pin state it recorded may be stale
    const bool a = gEarlyPhy.attach();
    uint32_t v = 0;
    const bool r = a && gEarlyPhy.read(0x11, v);
    Serial.printf("phy begun in setup(): attach=%s half=%lu ns read=%s dmstatus=0x%08lx\n",
                  a ? "yes" : "no", (unsigned long)gEarlyPhy.halfNs(), r ? "ok" : "FAILED", (unsigned long)v);
    gEarlyPhy.release();
  }
  linkQuality(0, 1);   // the pair the sweep found for the CH32L103
#endif
  Serial.println(any ? "swd_survey: a debug port answered" : "swd_survey: no answer on any combination");
  delay(8000);
}
