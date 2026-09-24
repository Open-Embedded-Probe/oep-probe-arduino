#include "OepRvswdPhy.h"

#include "OepRvswdFrame.h"

// One frame implementation (OepRvswdFrame.h) over a per-core pin backend. Each
// backend supplies the Io primitives, a critical section, and pad setup; the
// public methods below are shared so the two cannot drift apart.

#if defined(ARDUINO_ARCH_ESP32) && defined(SOC_DEDICATED_GPIO_SUPPORTED)
#define OEP_RVSWD_BACKEND 1
#include <driver/dedic_gpio.h>
#include <driver/gpio.h>
#include <esp_cpu.h>
#include <hal/dedic_gpio_cpu_ll.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>

namespace oep {
namespace {

// Dedicated GPIO bundle: bit0 = SWDIO, bit1 = SWCLK. One bundle per process.
dedic_gpio_bundle_handle_t gOut = nullptr;
dedic_gpio_bundle_handle_t gIn = nullptr;
int gDio = -1;
uint32_t gHalfCycles = 0;

struct Io {
  inline void spin() const {
    if (!gHalfCycles) return;
    const uint32_t start = esp_cpu_get_cycle_count();
    while (esp_cpu_get_cycle_count() - start < gHalfCycles) {}
  }
  inline void bothHigh() const { dedic_gpio_cpu_ll_write_mask(0x3, 0x3); }
  inline void clkLowDio(bool v) const { dedic_gpio_cpu_ll_write_mask(0x3, v ? 0x1 : 0x0); }
  inline void clkHigh() const { dedic_gpio_cpu_ll_write_mask(0x2, 0x2); }
  inline void clk(bool v) const { dedic_gpio_cpu_ll_write_mask(0x2, v ? 0x2 : 0); }
  inline void dio(bool v) const { dedic_gpio_cpu_ll_write_mask(0x1, v ? 0x1 : 0); }
  inline bool dioRead() const { return dedic_gpio_cpu_ll_read_in() & 0x1; }
  inline void hostDrives(bool yes) const {
    if (yes) gpio_ll_output_enable(&GPIO, gDio); else gpio_ll_output_disable(&GPIO, gDio);
  }
};
Io gIo;

struct Critical {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  Critical() { portENTER_CRITICAL(&mux); }
  ~Critical() { portEXIT_CRITICAL(&mux); }
};

bool ioBegin(int dio, int clk) {
  gDio = dio;
  pinMode(dio, INPUT);
  pinMode(clk, INPUT);
  if (!gOut) {
    pinMode(dio, OUTPUT | PULLUP);
    pinMode(clk, OUTPUT);
    const int outPins[] = {dio, clk};
    dedic_gpio_bundle_config_t outCfg = {};
    outCfg.gpio_array = outPins; outCfg.array_size = 2; outCfg.flags.out_en = 1;
    if (dedic_gpio_new_bundle(&outCfg, &gOut) != ESP_OK) return false;
    const int inPins[] = {dio};
    dedic_gpio_bundle_config_t inCfg = {};
    inCfg.gpio_array = inPins; inCfg.array_size = 1; inCfg.flags.in_en = 1;
    if (dedic_gpio_new_bundle(&inCfg, &gIn) != ESP_OK) return false;
    gpio_ll_od_disable(&GPIO, dio);
    gpio_ll_pullup_en(&GPIO, dio);
    gpio_ll_input_enable(&GPIO, dio);
    gpio_set_drive_capability(gpio_num_t(dio), GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(gpio_num_t(clk), GPIO_DRIVE_CAP_0);
    gIo.bothHigh();
  }
  return true;
}

void ioDrive(int dio, int clk) { gpio_ll_output_enable(&GPIO, clk); gpio_ll_output_enable(&GPIO, dio); }
void ioRelease(int dio, int clk) { gpio_ll_output_disable(&GPIO, dio); gpio_ll_output_disable(&GPIO, clk); }
uint32_t ioSetHalf(uint32_t half_ns) {
  gHalfCycles = (uint32_t)((uint64_t)half_ns * getCpuFrequencyMhz() / 1000);
  return gHalfCycles;
}

}  // namespace
}  // namespace oep

#elif defined(ARDUINO_ARCH_RP2040)
#define OEP_RVSWD_BACKEND 1
#include "OepRp2BitBang.h"

namespace oep {
namespace {

using Io = rp2::BitBang;
Io gIo;

struct Critical {
  Critical() { noInterrupts(); }
  ~Critical() { interrupts(); }
};

bool ioBegin(int dio, int clk) { return gIo.setup(dio, clk); }
void ioDrive(int, int) { gIo.driveBoth(); }
void ioRelease(int, int) { gIo.releaseBoth(); }
uint32_t ioSetHalf(uint32_t half_ns) { return gIo.setHalfNs(half_ns); }

}  // namespace
}  // namespace oep

#endif  // backend selection

#if OEP_RVSWD_BACKEND

namespace oep {
namespace {
constexpr uint8_t kDmControl = 0x10, kDmStatus = 0x11, kAbstractCs = 0x16, kDmAbstractAuto = 0x18,
                  kDmProgBuf0 = 0x20;
constexpr uint8_t kDmShadowCfgr = 0x7e, kDmCfgr = 0x7d;
constexpr uint32_t kCfgr = 0x5aa50400;   // WCH key | (1 << 10) "allow output from slave"
}  // namespace

bool RvswdPhy::begin(int swdio, int swclk) {
  swdio_ = swdio;
  swclk_ = swclk;
  if (!ioBegin(swdio, swclk)) return false;
  release();
  ready_ = true;
  return true;
}

void RvswdPhy::setHalf(uint32_t half_ns) {
  half_ns_ = half_ns;
  half_cycles_ = ioSetHalf(half_ns);
}

void RvswdPhy::release() {
  if (swdio_ < 0) return;
  ioRelease(swdio_, swclk_);
  attached_ = false;
}

void RvswdPhy::park() {
  if (swdio_ < 0) { attached_ = false; return; }
  if (!park_low_) { release(); return; }
  ioDrive(swdio_, swclk_);
  gIo.clkLowDio(true);
  attached_ = false;
}


// with_wake runs the hundred-clock wake burst. That burst resets the target, not just the
// debug interface: with it on every re-sync, the CH32L103's application restarted each
// time the probe halted it - its SysTick read the same 8 ms however long we had waited
// (2026-09-23). So only a cold bring-up wakes; re-syncing just rewrites the config.
void RvswdPhy::configureBus(bool with_wake) {
  gIo.bothHigh();
  ioDrive(swdio_, swclk_);
  delayMicroseconds(20);
  if (with_wake) {
    rvswd::wake(gIo);
    delayMicroseconds(20);
  }
  // DMSHDWCFGR then DMCFGR with the WCH key and "allow output from slave". minichlink writes
  // this pair on every bring-up with the note that a part coming out of cold boot will not
  // communicate without it, and writes each one twice because once is not always enough.
  // The CH32X035 answered without it; the CH32L103 on the Pico's wiring did not halt
  // (2026-09-23), which is what sent us looking at what a known-good host does.
  for (int i = 0; i < 2; ++i) {
    writeRaw(kDmShadowCfgr, kCfgr);
    writeRaw(kDmCfgr, kCfgr);
  }
  // Leave the abstract-command block in a known state. A session that ended mid-sequence
  // can leave autoexec armed on DATA0 or a sticky cmderr behind, and then the next
  // attach's write check reads back something it never wrote (2026-09-23).
  writeRaw(kDmAbstractAuto, 0);
  writeRaw(kAbstractCs, 0x700);
}

// The CH32's two-wire debug interface drops the link when the bus goes quiet. Measured on
// the CH32L103 (2026-09-23): idle for 500 us and everything still answers; 1 ms and every
// read comes back all ones; 5 ms and one bring-up is no longer enough to get it back. A
// host that talks over USB is always past that - halt and the read that follows it are
// separate requests tens of ms apart - and the damage is silent, because the debug module
// keeps answering DATA0 with whatever was left there and the abstract command never runs.
// So bring the bus back before the first transaction after any pause.
void RvswdPhy::reviveIfIdle() {
  if (!ready_ || !attached_) return;
  if (micros() - last_activity_us_ < kIdleUs) return;
  uint32_t status = 0;
  if (readRaw(kDmStatus, status) && (status & 0xf) == 2 && (status & 0x80)) return;   // still there
  configureBus(false);                                                                // re-sync, no reset
  if (readRaw(kDmStatus, status) && (status & 0xf) == 2 && (status & 0x80)) return;
  // Parking the clock low keeps the link, so this is the path for a target that was left
  // parked high by something else, or that lost the bus for its own reasons. It takes
  // about a dozen bring-ups to come back, and the debug module resets on the way, so the
  // hart will be running again: the caller finds out through cmderr, not through a lie.
  for (int i = 0; i < 12; ++i) {
    configureBus(true);
    if (readRaw(kDmStatus, status) && (status & 0xf) == 2 && (status & 0x80)) break;
  }
  last_activity_us_ = micros();
}

bool RvswdPhy::readRaw(uint8_t address, uint32_t &value) {
  bool ok;
  {
    Critical lock;
    ok = rvswd::readWord(gIo, address, value);
    if (park_low_) gIo.clkLowDio(true);
  }
  ++transactions_;
  last_activity_us_ = micros();
  return ok;
}

bool RvswdPhy::read(uint8_t address, uint32_t &value) {
  reviveIfIdle();
  for (int attempt = 0; attempt < 200; ++attempt) {
    if (readRaw(address, value)) return true;
    ++retries_;
  }
  return false;
}

void RvswdPhy::writeRaw(uint8_t address, uint32_t data) {
  {
    Critical lock;
    rvswd::writeWord(gIo, address, data);
    if (park_low_) gIo.clkLowDio(true);
  }
  ++transactions_;
  last_activity_us_ = micros();
}

void RvswdPhy::write(uint8_t address, uint32_t data) {
  reviveIfIdle();
  writeRaw(address, data);
}

bool RvswdPhy::probeOnce(uint32_t half_ns, uint32_t &dmstatus, bool keep_driven) {
  if (!ready_) return false;
  setHalf(half_ns);
  configureBus(true);
  write(0x10, 1);  // DMCONTROL.dmactive
  dmstatus = 0;
  const bool ok = readRaw(0x11, dmstatus);
  if (!keep_driven) { ioRelease(swdio_, swclk_); attached_ = false; }
  // A debug module reports a nonzero DMSTATUS.version; an idle bus reads all ones or zeros.
  return ok && ((dmstatus >> 8) & 0xf) != 0 && dmstatus != 0xffffffffu;
}

namespace {
const uint32_t kHalfNs[] = {0, 25, 50, 100, 200, 500};
const size_t kCount = sizeof kHalfNs / sizeof kHalfNs[0];
}  // namespace

bool RvswdPhy::readsStable(uint32_t &first) {
  // Let anything the bring-up disturbed settle before the reference read, or a hart that
  // is still coming to a stop makes a good half period look unstable.
  for (int i = 0; i < 8; ++i) readRaw(kDmStatus, first);
  if (((first >> 8) & 0xf) == 0 || first == 0xffffffffu) return false;
  const uint32_t t0 = micros();
  for (int i = 0; i < 1000; ++i) {
    uint32_t value = 0;
    if (!readRaw(kDmStatus, value) || value != first) return false;
  }
  dmi_ns_ = micros() - t0;   // 1000 reads -> ns per read
  return true;
}

// Reads can be clean at a half period whose writes are not. Measured on a CH32L103 over
// the Pico's flying wires (2026-09-23): DMSTATUS read the same 1000 times at 100 ns, yet
// the halt requests written at that speed were silently mangled - the hart kept running
// and abstract commands failed cmderr=4. So prove the write path at the same speed.
// DATA0 is the debug module's own scratch register while no abstract command runs.
// A handful of patterns is not enough: at 200 ns on that jig every pattern came back
// intact, and the multi-transaction sequences behind a memory read still broke. Match
// the read check's weight - a few hundred round trips - so a half period only survives
// if its writes land as reliably as its reads.
//
// The scratch is the first program buffer word, not DATA0. DATA0 belongs to whatever is
// running: a target printing through the debug module's console writes it continuously,
// and attaching to one failed every time while this check used it (2026-09-23). Nothing
// reads the program buffer until an abstract command runs one.
bool RvswdPhy::writesLand() {
  static const uint32_t kPatterns[] = {0xa5a5a5a5u, 0x5a5a5a5au, 0xffffffffu, 0x00000001u,
                                       0x0f0f0f0fu, 0xf0f0f0f0u, 0x80000000u, 0x7fffffffu};
  // DATA0 is only scratch while nothing is armed on it: a previous session that left
  // ABSTRACTAUTO set would re-run its command on every access here and the readback
  // would never match (2026-09-23: an aborted flash sequence did exactly that, and
  // every later attach failed until the probe was power-cycled).
  write(kDmAbstractAuto, 0);
  bool ok = true;
  for (int round = 0; round < 32 && ok; ++round) {
    for (uint32_t pattern : kPatterns) {
      write(kDmProgBuf0, pattern);
      uint32_t read_back = 0;
      if (!readRaw(kDmProgBuf0, read_back) || read_back != pattern) { ok = false; break; }
    }
  }
  write(kDmProgBuf0, 0);
  return ok;
}

void RvswdPhy::useSafeSpeed() {
  // Also when attach() did not take: a CH32 held in reset may not answer, and the writes that follow the release
  // must still go out at a speed its default clock can follow.
  if (!ready_) return;
  setHalf(kHalfNs[kCount - 1] > min_half_ns_ ? kHalfNs[kCount - 1] : min_half_ns_);
  configureBus(false);
}

// attach()'s search without the wake: the wake burst resets the target, and this runs on a hart the caller has
// just stopped. It starts from the slowest period, which a reset has to be done at anyway, and speeds up while
// both checks pass. A period the target cannot follow leaves its debug module out of step, so after the first
// failure it steps back to the last good one and proves that again rather than trusting it.
bool RvswdPhy::retune() {
  if (!attached_) return false;
  const uint32_t floor = min_half_ns_;
  size_t good = kCount;   // index of the fastest period that passed
  bool failed = false;
  for (size_t i = kCount; i-- > 0;) {
    if (kHalfNs[i] < floor && i != kCount - 1) break;
    setHalf(kHalfNs[i] > floor ? kHalfNs[i] : floor);
    configureBus(false);
    uint32_t first = 0;
    if (!readsStable(first) || !writesLand()) { failed = true; break; }
    good = i;
  }
  if (good == kCount) { useSafeSpeed(); return false; }
  if (!failed) return true;   // the fastest candidate passed and is still set: nothing is out of step
  for (int tries = 0; tries < 3; ++tries) {
    setHalf(kHalfNs[good] > floor ? kHalfNs[good] : floor);
    configureBus(false);
    uint32_t first = 0;
    if (readsStable(first) && writesLand()) return true;
    if (good + 1 < kCount) ++good;   // slower
  }
  useSafeSpeed();
  return false;
}

bool RvswdPhy::attach() {
  if (!ready_) return false;
  if (attached_) return true;
  // Margin check (E156/E157): half 0 ns sometimes fails for a whole run. The bus is brought
  // up again for each candidate, because a half period the target cannot follow leaves its
  // debug module out of step and the next, slower attempt would inherit that (2026-09-23:
  // on the Pico's flying wires to a CH32L103, one probe with a fresh init answered while
  // this loop without one failed at every half period).
  // One candidate: bring the bus up, then insist on 1000 identical DMSTATUS reads.
  auto clean_at = [this](uint32_t half) {
    setHalf(half);
    // A cold debug module does not answer the first wake. Measured on a CH32L103 over the
    // Pico's flying wires (2026-09-23): the first clean read came on attempt 5 at a 500 ns
    // half period, and on attempt 0 once the module had answered. So give each candidate a
    // few tries before judging it, or a cold target looks like no target at all.
    uint32_t first = 0;
    bool awake = false;
    for (int wake = 0; wake < 8 && !awake; ++wake) {
      configureBus(true);
      // Only skip the dmactive write when the module is plainly up: that write clears
      // haltreq, so attaching to a target somebody halted earlier would set it running
      // again. "Plainly" matters - a module that is not active leaves the bus floating,
      // and all ones has bit 0 set too. Trusting that, the CH32X035 was never activated
      // and attach failed every time (2026-09-23), so an all-ones read counts as no answer.
      uint32_t control = 0;
      const bool up = readRaw(kDmControl, control) && control != 0xffffffffu && (control & 1);
      if (!up) writeRaw(kDmControl, 1);
      awake = readRaw(kDmStatus, first);
    }
    // DMSTATUS.version is nonzero on a real module; an idle bus reads all ones or zeros.
    if (!awake || ((first >> 8) & 0xf) == 0) return false;
    return readsStable(first);
  };
  // Two passes. A cold debug module can need more waking than one candidate's eight
  // attempts, and the candidates that fail warm it up as a side effect: with a floor that
  // leaves a single candidate, the first pass failed where the same period attached
  // immediately on the second (2026-09-23).
  for (int pass = 0; pass < 2; ++pass) {
    for (size_t i = 0; i < kCount; ++i) {
      if (kHalfNs[i] < min_half_ns_) continue;
      if (!clean_at(kHalfNs[i]) || !writesLand()) continue;
      attached_ = true;
      return true;
    }
  }
  release();
  return false;
}

}  // namespace oep

#else  // stub for cores without a backend

namespace oep {
bool RvswdPhy::begin(int, int) { return false; }
bool RvswdPhy::attach() { return false; }
void RvswdPhy::release() { attached_ = false; }
void RvswdPhy::park() { attached_ = false; }
bool RvswdPhy::read(uint8_t, uint32_t &) { return false; }
void RvswdPhy::write(uint8_t, uint32_t) {}
void RvswdPhy::setHalf(uint32_t) {}
void RvswdPhy::configureBus(bool) {}
void RvswdPhy::writeRaw(uint8_t, uint32_t) {}
void RvswdPhy::reviveIfIdle() {}
bool RvswdPhy::readRaw(uint8_t, uint32_t &) { return false; }
bool RvswdPhy::probeOnce(uint32_t, uint32_t &, bool) { return false; }
}  // namespace oep

#endif
