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

// Does the debug port take writes, not just reads? Power up the debug domain through DP
// CTRL/STAT and read the acknowledge back. Harmless: it powers a domain, it halts nothing.
static void swdWriteCheck(int dio, int clk) {
  if (!io.setup(dio, clk)) return;
  io.setHalfNs(500);
  io.driveBoth();
  swd::jtagToSwd(io);
  uint32_t dpidr = 0;
  const uint8_t ack = swd::transfer(io, false, true, 0x0, dpidr);
  if (ack != swd::kOk) { Serial.printf("  no DPIDR on GP%d/GP%d\n", dio, clk); io.releaseBoth(); return; }
  uint32_t abort = 0x1e;                       // clear any sticky error
  swd::transfer(io, false, false, 0x0, abort);
  uint32_t ctrl = 0x50000000u;                 // CDBGPWRUPREQ | CSYSPWRUPREQ
  const uint8_t wack = swd::transfer(io, false, false, 0x1, ctrl);
  uint32_t back = 0;
  uint8_t rack = 0;
  for (int i = 0; i < 20; ++i) {
    rack = swd::transfer(io, false, true, 0x1, back);
    if (rack == swd::kOk && (back & 0xa0000000u) == 0xa0000000u) break;
    delay(1);
  }
  io.releaseBoth();
  Serial.printf("  DPIDR 0x%08lx | CTRL/STAT write ack=%u | read ack=%u value 0x%08lx -> %s\n",
                (unsigned long)dpidr, wack, rack, (unsigned long)back,
                (back & 0xa0000000u) == 0xa0000000u ? "debug domain powered up: writes land" : "no power-up acknowledge");
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
  {   // is the haltreq write landing at all? DMCONTROL reads back what was written.
    oep::RvswdPhy w;
    w.begin(0, 1);
    if (w.attach()) {
      uint32_t before = 0, after = 0, status = 0;
      w.read(0x10, before);
      w.write(0x10, 0x80000001);
      const bool ok = w.read(0x10, after);
      w.read(0x11, status);
      Serial.printf("write check: DMCONTROL before 0x%08lx, wrote 0x80000001, reads back %s0x%08lx (haltreq=%lu) DMSTATUS 0x%08lx\n",
                    (unsigned long)before, ok ? "" : "(failed) ", (unsigned long)after,
                    (unsigned long)((after >> 31) & 1), (unsigned long)status);
      // Does a bus re-init after haltreq make the hart visibly halt?
      for (int i = 0; i < 10; ++i) {
        w.wakeBus();
        w.write(0x10, 0x80000001);
        uint32_t st = 0;
        const bool ok = w.read(0x11, st);
        Serial.printf("  re-init #%d: DMSTATUS %s0x%08lx allhalted=%lu\n", i, ok ? "" : "(failed) ",
                      (unsigned long)st, (unsigned long)((st >> 9) & 1));
        if (ok && ((st >> 9) & 1) && st != 0xffffffffu) break;
        delay(5);
      }
      // Halt through reset: if the running application is what refuses to stop (a low-power
      // mode gates the core clock), the hart still halts at its reset vector.
      w.wakeBus();
      w.write(0x10, 0x80000003);   // haltreq | ndmreset | dmactive
      delay(2);
      w.wakeBus();
      w.write(0x10, 0x80000001);   // release ndmreset, keep haltreq
      for (int i = 0; i < 6; ++i) {
        delay(5);
        w.wakeBus();
        uint32_t st = 0;
        const bool ok = w.read(0x11, st);
        Serial.printf("  halt-through-reset #%d: DMSTATUS %s0x%08lx allhalted=%lu\n", i,
                      ok ? "" : "(failed) ", (unsigned long)st, (unsigned long)((st >> 9) & 1));
        if (ok && st != 0xffffffffu && ((st >> 9) & 1)) break;
      }
      w.wakeBus();
      w.write(0x10, 0x40000001);   // resumereq
      delay(2);
      w.wakeBus();
      w.write(0x10, 0x00000001);
    }
    w.release();
  }
  {   // does the hart halt? write DMCONTROL.haltreq and watch DMSTATUS
    oep::RvswdPhy h;
    h.begin(0, 1);
    if (h.attach()) {
      uint32_t v = 0;
      h.read(0x11, v);
      // DMSTATUS lies on this jig: reads taken after a state change come back mostly ones
      // and their allhalted bit means nothing. The proof of a halt is an abstract register
      // read, which only a halted hart can serve (cmderr 4 = still running). Repeat the
      // request the way minichlink does; one is not enough (2026-09-23).
      uint32_t chipid = 0, hartinfo = 0;
      h.readOnce(0x7f, chipid);
      h.readOnce(0x12, hartinfo);
      Serial.printf("halt test: attached at %lu ns, DMSTATUS 0x%08lx, DMCHIPID 0x%08lx, HARTINFO 0x%08lx\n",
                    (unsigned long)h.halfNs(), (unsigned long)v, (unsigned long)chipid,
                    (unsigned long)hartinfo);
      for (int k = 0; k < 40; ++k)
        for (int j = 0; j < 4; ++j) h.write(0x10, 0x80000001);
      uint32_t st = 0, acs = 0, hartid = 0;
      h.readOnce(0x11, st);
      h.write(0x16, 0x00000700);          // clear cmderr
      h.write(0x17, 0x00220f14);          // access register: read CSR mhartid
      h.readOnce(0x16, acs);
      h.readOnce(0x04, hartid);
      Serial.printf("  after 160 halt requests: DMSTATUS 0x%08lx, cmderr=%lu, mhartid 0x%08lx -> %s\n",
                    (unsigned long)st, (unsigned long)((acs >> 8) & 7), (unsigned long)hartid,
                    ((acs >> 8) & 7) == 0 ? "HALTED" : "still running");
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
#if !defined(ARDUINO_SPARKFUN_PROMICRO_RP2350)
  Serial.println("ARM SWD write path (DP CTRL/STAT power-up):");
  for (int swap = 0; swap < 2; ++swap) swdWriteCheck(swap ? 1 : 0, swap ? 0 : 1);
#endif
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
