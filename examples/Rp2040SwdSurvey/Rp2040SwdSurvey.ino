// Bring-up for the ordinary ARM SWD side of the Pico bench: a Waveshare RP2040-Zero
// drives the SparkFun Pro Micro RP2350's SWD header from GP0/GP1 (which line is which is
// not recorded, so both orientations are tried) and reads the debug port's DPIDR.
//
// The RP2040 and RP2350 both expose a multidrop SW-DP: after a line reset no target
// answers until the host writes DP TARGETSEL, and the instance ids differ per part, so
// the sweep below reports which candidate replies instead of assuming one.
//
// Read-only: it reads DPIDR and CTRL/STAT and never powers up or halts anything.
#include <initializer_list>

#include <OepRp2BitBang.h>
#include <OepSwdFrame.h>

using oep::rp2::BitBang;
namespace swd = oep::swd;

static const uint32_t kTargets[] = {
    0x01002927u, 0x11002927u, 0xf1002927u,   // RP2040 core 0 / core 1 / rescue
    0x00040927u, 0x10040927u, 0xf0040927u,   // RP2350 guess, same designer, part 0x0004
};
static const char *kTargetNames[] = {"rp2040-core0", "rp2040-core1", "rp2040-rescue",
                                     "rp2350-core0?", "rp2350-core1?", "rp2350-rescue?"};

static BitBang io;

static bool tryOne(int dio, int clk, uint32_t half_ns, const uint32_t *target, const char *name) {
  if (!io.setup(dio, clk)) return false;
  io.setHalfNs(half_ns);
  io.driveBoth();
  swd::jtagToSwd(io);
  if (target) swd::targetSelect(io, *target);
  uint32_t dpidr = 0;
  const uint8_t ack = swd::transfer(io, false, true, 0x0, dpidr);
  io.releaseBoth();
  Serial.printf("  SWDIO=GP%-2d SWCLK=GP%-2d half=%4u ns  targetsel=%-14s ack=%u dpidr=0x%08lx%s\n",
                dio, clk, (unsigned)half_ns, name, ack, (unsigned long)dpidr,
                (ack == swd::kOk && dpidr != 0 && dpidr != 0xffffffffu) ? "   <== answered" : "");
  return ack == swd::kOk && dpidr != 0 && dpidr != 0xffffffffu;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}
  delay(200);
  Serial.println("swd_survey: RP2040-Zero -> Pro Micro RP2350 SWD header");
  Serial.printf("spin loop: %u ps per iteration\n", (unsigned)oep::rp2::loopPicoseconds());
}

void loop() {
  static uint32_t round = 0;
  Serial.printf("---- sweep %lu\n", (unsigned long)++round);
  bool any = false;
  for (int swap = 0; swap < 2; ++swap) {
    const int dio = swap ? 1 : 0, clk = swap ? 0 : 1;
    for (uint32_t half : {200u, 1000u}) {
      any |= tryOne(dio, clk, half, nullptr, "none");
      for (size_t i = 0; i < sizeof kTargets / sizeof kTargets[0]; ++i)
        any |= tryOne(dio, clk, half, &kTargets[i], kTargetNames[i]);
    }
  }
  Serial.println(any ? "swd_survey: a debug port answered" : "swd_survey: no answer on any combination");
  delay(5000);
}
