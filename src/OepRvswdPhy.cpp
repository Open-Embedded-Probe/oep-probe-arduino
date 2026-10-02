// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

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
uint32_t gDioSig = 0, gClkSig = 0;   // the pins' output signals once in the bundle (ioReclaim)

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
#if defined(CONFIG_IDF_TARGET_ESP32P4)
    gDioSig = GPIO.func_out_sel_cfg[dio].out_sel;
    gClkSig = GPIO.func_out_sel_cfg[clk].out_sel;
#endif
  }
  return true;
}

// Another pair (host-chosen pins): the bundles are made again on it. The old pins are left Hi-Z inputs, set through the
// GPIO registers - not pinMode - like everything this backend does to its pins (unverified on hardware, 2026-09-30).
bool ioMove(int old_dio, int old_clk, int dio, int clk) {
  if (gOut) { dedic_gpio_del_bundle(gOut); gOut = nullptr; }
  if (gIn) { dedic_gpio_del_bundle(gIn); gIn = nullptr; }
  if (old_dio >= 0) {
    gpio_ll_output_disable(&GPIO, old_dio);
    gpio_ll_output_disable(&GPIO, old_clk);
    gpio_ll_pullup_dis(&GPIO, old_dio);
  }
  return ioBegin(dio, clk);
}

// The pins back in the bundles if something else routed them away since (ESP32-P4: oep.wire.swio takes any channel,
// the RVSWD pair's too, into its own bundle and leaves it a plain GPIO; a pair unchanged since would otherwise stay
// cut off from its bundle). Nothing done while they are still routed here.
// The weakest strength is put back too: a fixture gpio output or an output idle on these pins since (a host-chosen
// pair is free between connections) leaves the pad at its own strength (oep-if-fixture §1.1), never the wire's.
void ioReclaim(int dio, int clk) {
  if (dio >= 0) {
    gpio_set_drive_capability(gpio_num_t(dio), GPIO_DRIVE_CAP_0);
    gpio_set_drive_capability(gpio_num_t(clk), GPIO_DRIVE_CAP_0);
    gpio_ll_pullup_en(&GPIO, dio);   // the data line idles high while the target drives it (ioFree took it off)
  }
#if defined(CONFIG_IDF_TARGET_ESP32P4)
  if (!gOut || dio < 0) return;
  if (GPIO.func_out_sel_cfg[dio].out_sel == gDioSig && GPIO.func_out_sel_cfg[clk].out_sel == gClkSig) return;
  ioMove(dio, clk, dio, clk);
#else
  (void)dio; (void)clk;
#endif
}

void ioDrive(int dio, int clk) { gpio_ll_output_enable(&GPIO, clk); gpio_ll_output_enable(&GPIO, dio); }
void ioRelease(int dio, int clk) { gpio_ll_output_disable(&GPIO, dio); gpio_ll_output_disable(&GPIO, clk); }
// The free state (oep-core §8): released, and SWDIO without the pull-up ioBegin gave it (ioReclaim puts it back).
void ioFree(int dio, int clk) { ioRelease(dio, clk); gpio_ll_pullup_dis(&GPIO, dio); }
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
bool ioMove(int old_dio, int old_clk, int dio, int clk) {   // the old pins back to plain Hi-Z inputs
  if (old_dio >= 0) {
    gIo.releaseBoth();
    gpio_disable_pulls(old_dio);
    gpio_disable_pulls(old_clk);
  }
  return gIo.setup(dio, clk);
}
// The weakest strength put back (a fixture gpio output, an output idle or arduino-pico's pinMode(OUTPUT) - 4 mA - on a
// host-chosen pair since its last connection, oep-if-fixture §1.1): the wire always runs at 2 mA (setup).
void ioReclaim(int dio, int clk) {
  if (dio < 0) return;
  gpio_set_drive_strength(dio, GPIO_DRIVE_STRENGTH_2MA);
  gpio_set_drive_strength(clk, GPIO_DRIVE_STRENGTH_2MA);
  gpio_pull_up(dio);   // the data line idles high while the target drives it (ioFree took it off)
}
void ioDrive(int, int) { gIo.driveBoth(); }
void ioRelease(int, int) { gIo.releaseBoth(); }
// The free state (oep-core §8): released, and SWDIO without the pull-up setup gave it (ioReclaim puts it back).
void ioFree(int dio, int) { gIo.releaseBoth(); gpio_disable_pulls(dio); }
uint32_t ioSetHalf(uint32_t half_ns) { return gIo.setHalfNs(half_ns); }

}  // namespace
}  // namespace oep

#elif defined(OEP_HOST_FAKE_RVSWD)
// Host tests (tests/host): the frames against a simulated target, which keeps the pins' state and the time.
#define OEP_RVSWD_BACKEND 1
#include <fake_rvswd_io.h>

namespace oep {
namespace {

using Io = FakeRvswdIo;
Io gIo;
struct Critical { ~Critical() {} };   // nothing to mask on the host

bool ioBegin(int dio, int clk) { return gIo.begin(dio, clk); }
bool ioMove(int, int, int dio, int clk) { return gIo.begin(dio, clk); }
void ioReclaim(int dio, int) { if (dio >= 0) gIo.pullUp(true); }
void ioDrive(int, int) { gIo.driveBoth(true); }
void ioRelease(int, int) { gIo.driveBoth(false); }
void ioFree(int, int) { gIo.driveBoth(false); gIo.pullUp(false); }
uint32_t ioSetHalf(uint32_t half_ns) { return gIo.setHalfNs(half_ns); }

}  // namespace
}  // namespace oep

#endif  // backend selection

#if OEP_RVSWD_BACKEND

namespace oep {
namespace {
constexpr uint8_t kDmControl = 0x10, kDmStatus = 0x11, kDmAbstractAuto = 0x18, kDmProgBuf0 = 0x20;
constexpr uint8_t kDmShadowCfgr = 0x7e, kDmCfgr = 0x7d;
constexpr uint32_t kCfgr = 0x5aa50400;   // WCH key | (1 << 10) "allow output from slave"
// DMSTATUS bits 8-19 are the harts' state (any / all halted, running, unavailable, nonexistent, resumeack, havereset):
// they change on their own while the speed is checked - a hart coming to a stop, a target resetting itself - and say
// nothing about the link. The check compares the rest (version, authenticated, impebreak and the like).
constexpr uint32_t kHartStateBits = 0x000fff00u;
// At the slowest period there is nothing slower to fall back to: an isolated parity or turnaround error there would
// fail the whole attach. So the checks there take a few retries; the faster periods take none (a period with any
// error is not used).
constexpr uint8_t kSlowestRetries = 3;
// How long one check may take. 1000 reads and 256 round trips take 79 + 40 ms at the 500 ns half period (RP2350, CH32L103);
// a host's max_speed far below it would stretch them past the attach budget, so a slower period checks fewer.
constexpr uint32_t kReadCheckUs = 100000, kWriteCheckUs = 60000;
constexpr int kReadChecks = 1000, kMinReadChecks = 64, kWriteRounds = 32, kMinWriteRounds = 2;
}  // namespace

bool RvswdPhy::begin(int swdio, int swclk) {
  swdio_ = swdio;
  swclk_ = swclk;
  if (!ioBegin(swdio, swclk)) return false;
  free();   // until a connection takes them: the free state (oep-core §8), the same as after a release
  ready_ = true;
  return true;
}

bool RvswdPhy::usePins(int swdio, int swclk) {
  if (swdio == swdio_ && swclk == swclk_) return true;
  if (attached_) return false;
  if (!ioMove(swdio_, swclk_, swdio, swclk)) { swdio_ = swclk_ = -1; ready_ = false; return false; }
  swdio_ = swdio;
  swclk_ = swclk;
  free();
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

void RvswdPhy::free() {
  if (swdio_ < 0) { attached_ = false; return; }
  ioFree(swdio_, swclk_);
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
// (2026-09-23). So only a cold bring-up wakes; re-syncing just rewrites the config. A CH32X035 is not reset by it:
// neither the 100-clock burst nor 100-236 clocks with SWDIO held high or low set havereset there (wch-protocols
// E170, 2026-09-26) - the reset is the L103's, so keep the wake off re-syncs for the parts that do it.
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
  // The abstract-command block (ABSTRACTAUTO, cmderr) is the debug module's business, not the bus's: Ch32Dm
  // clears it when it attaches and whenever it brings the link up again (Ch32Dm::relink).
}

// Back in step at `half` before its speed is verified (oep-if-debug §1): the configuration pair goes out at the slowest
// period - the only one writes may use before the check - and the link then moves to `half` for the reads.
void RvswdPhy::resyncAt(uint32_t half) {
  setHalf(slowestNs());
  configureBus(false);
  setHalf(half);
}

// The CH32's two-wire debug interface drops the link when the bus goes quiet. Measured on
// the CH32L103 (2026-09-23): idle for 500 us and everything still answers; 1 ms and every
// read comes back all ones; 5 ms and one bring-up is no longer enough to get it back. A
// host that talks over USB is always past that - halt and the read that follows it are
// separate requests tens of ms apart - and the damage is silent, because the debug module
// keeps answering DATA0 with whatever was left there and the abstract command never runs.
// So bring the bus back before the first transaction after any pause.
//
// Everything after the first read that did not answer is a retry of the request's (oep-if-debug §2): the re-sync and
// each wake are charged to its wire_retry_ms, and none starts that would end past it (one wake at a slow max_speed
// takes tens of ms; at 10 kHz twelve of them and the read's own retries made one request 691 ms).
void RvswdPhy::reviveIfIdle() {
  if (!ready_ || !attached_) return;
  if (micros() - last_activity_us_ < kIdleUs) return;
  uint32_t status = 0;
  auto answers = [&]() { return readRaw(kDmStatus, status) && dmVersionKnown(status) && (status & 0x80); };
  if (answers()) return;   // still there
  uint32_t t0 = micros();
  configureBus(false);     // re-sync, no reset
  const bool back = answers();
  uint32_t cost = micros() - t0;
  spentRetrying(cost);
  if (back) return;
  // Parking the clock low keeps the link, so this is the path for a target that was left
  // parked high by something else, or that lost the bus for its own reasons. It takes
  // about a dozen bring-ups to come back, and the debug module resets on the way, so the
  // hart will be running again: the caller finds out through cmderr, not through a lie.
  // A wake costs more than the re-sync: the first one is let start only with twice the re-sync's time left.
  cost *= 2;
  for (int i = 0; i < 12 && retryFits(cost); ++i) {
    t0 = micros();
    configureBus(true);
    const bool up = answers();
    cost = micros() - t0;
    spentRetrying(cost);
    if (up) break;
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

// A failed read is retried while the request's allowance lasts (oep-if-debug §2: at most wire_retry_ms of one request
// goes to retries, the revive's re-sync and wakes included), and no more than 200 times in a row.
bool RvswdPhy::readWire(uint8_t address, uint32_t &value) {
  reviveIfIdle();
  uint32_t t0 = micros();
  if (readRaw(address, value)) return true;
  uint32_t cost = micros() - t0;   // one read: the next retry starts only if it still ends inside the allowance
  for (int attempt = 1; attempt < 200 && retryFits(cost); ++attempt) {
    ++retries_;
    t0 = micros();
    const bool ok = readRaw(address, value);
    cost = micros() - t0;
    spentRetrying(cost);
    if (ok) return true;
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

// dmactive (oep-if-debug §1 item 2), written only when DMCONTROL does not already read it set: that write clears
// haltreq, so attaching to a target somebody halted earlier would set it running again. "Plainly" set matters - a
// module that is not active leaves the bus floating, and all ones has bit 0 set too (2026-09-23, CH32X035).
void RvswdPhy::activate() {
  uint32_t control = 0;
  if (readRaw(kDmControl, control) && control != 0xffffffffu && (control & 1)) return;
  writeRaw(kDmControl, 1);
  // The CFGR pair above went to a module that was not active (a detach writes DMCONTROL = 0): the configuration
  // sequence again now that it is, the order a WCH-LinkE uses (dmactive, then CFGR; wch-protocols link-to-target §5). It
  // did not cure the dmseq console coming back 1-10 s late after a refused automatic attach (probe-cdc-and-persistence
  // §7.5.1); oep_smoke x035 14/14 with it.
  for (int i = 0; i < 2; ++i) {
    writeRaw(kDmShadowCfgr, kCfgr);
    writeRaw(kDmCfgr, kCfgr);
  }
}

// The writes allowed before the speed is verified, at the slowest period (oep-if-debug §1, §3): the wake and the
// configuration pair, then dmactive - until DMSTATUS shows a debug module. A cold one does not answer the first wake.
// Measured on a CH32L103 over the RP2350 probe (2026-09-23): the first clean read came on attempt 5 at a 500 ns half
// period, and on attempt 0 once the module had answered.
bool RvswdPhy::wakeModule(uint32_t &dmstatus) {
  setHalf(slowestNs());
  for (int wake = 0; wake < 8; ++wake) {
    if (wake && pastDeadline()) return false;
    configureBus(true);
    activate();
    if (readRaw(kDmStatus, dmstatus) && dmVersionKnown(dmstatus)) return true;
  }
  return false;
}

bool RvswdPhy::bringUp(uint32_t &dmstatus) {
  if (!ready_) return false;
  if (attached_) return read(kDmStatus, dmstatus);
  ioReclaim(swdio_, swclk_);
  const bool ok = wakeModule(dmstatus);
  ioRelease(swdio_, swclk_);
  attached_ = false;
  return ok;
}

bool RvswdPhy::probeOnce(uint32_t half_ns, uint32_t &dmstatus, bool keep_driven) {
  if (!ready_) return false;
  ioReclaim(swdio_, swclk_);
  setHalf(half_ns);
  configureBus(true);
  activate();
  dmstatus = 0;
  const bool ok = readRaw(kDmStatus, dmstatus);
  if (!keep_driven) { ioRelease(swdio_, swclk_); attached_ = false; }
  // A debug module reports a known DMSTATUS.version; an idle bus reads all ones or zeros.
  return ok && dmVersionKnown(dmstatus);
}

namespace {
const uint32_t kHalfNs[] = {0, 25, 50, 100, 200, 500};
const size_t kCount = sizeof kHalfNs / sizeof kHalfNs[0];
}  // namespace

uint32_t RvswdPhy::slowestNs() const {
  return kHalfNs[kCount - 1] > floorNs() ? kHalfNs[kCount - 1] : floorNs();
}

// Up to 1000 DMSTATUS reads at the current period that all pass their parity and agree with the first, the harts'
// state bits left out (kHartStateBits); fewer at a period too slow for 1000 in kReadCheckUs. `retries` failed reads are
// taken again (at the slowest period, kSlowestRetries; elsewhere none). `first`: the reference value.
bool RvswdPhy::readsStable(uint32_t &first, uint8_t retries) {
  // Let anything the bring-up disturbed settle before the reference read, or a hart that
  // is still coming to a stop makes a good half period look unstable.
  bool ok = false;
  for (int i = 0; i < 8; ++i) ok = readRaw(kDmStatus, first);
  uint8_t missed = 0;
  while (!ok || !dmVersionKnown(first)) {
    if (missed++ >= retries) return false;
    ++search_retries_;
    ok = readRaw(kDmStatus, first);
  }
  const uint32_t t0 = micros();
  uint32_t reads = 0;
  for (int good = 0; good < kReadChecks;) {
    if (good >= kMinReadChecks && micros() - t0 >= kReadCheckUs) break;
    uint32_t value = 0;
    ++reads;
    if (readRaw(kDmStatus, value) && ((value ^ first) & ~kHartStateBits) == 0) { ++good; continue; }
    if (missed++ >= retries) return false;
    ++search_retries_;
  }
  dmi_ns_ = static_cast<uint32_t>((uint64_t)(micros() - t0) * 1000u / reads);   // ns per read
  return true;
}

// The scratch register's value before the write check (oep-if-debug §1), read at the slowest period once the reads
// there have checked out. false: it could not be read.
bool RvswdPhy::keepScratch(uint8_t retries) {
  uint8_t missed = 0;
  while (!readRaw(kDmProgBuf0, scratch_)) {
    if (missed++ >= retries) return false;
    ++search_retries_;
  }
  return true;
}

// Reads can be clean at a half period whose writes are not. Measured on a CH32L103 over
// the RP2350 probe (2026-09-23): DMSTATUS read the same 1000 times at 100 ns, yet
// the halt requests written at that speed were silently mangled - the hart kept running
// and abstract commands failed cmderr=4. So prove the write path at the same speed.
// A handful of patterns is not enough: at 200 ns on that jig every pattern came back
// intact, and the multi-transaction sequences behind a memory read still broke. Match
// the read check's weight - a few hundred round trips - so a half period only survives
// if its writes land as reliably as its reads.
//
// Only what oep-if-debug §3 names is written: the scratch, the first program buffer word (not DATA0: that belongs to
// whatever is running - a target printing through the debug module's console writes it continuously, and attaching to
// one failed every time while this check used it, 2026-09-23), and ABSTRACTAUTO = 0, which makes it free (an
// autoexec a previous session left armed would re-run its command on every access here and the readback would never
// match: 2026-09-23, an aborted flash sequence did exactly that, and every later attach failed until the probe was
// power-cycled). ABSTRACTAUTO is not restored; the scratch gets the value keepScratch read back - at this period when
// the check passed, at the slowest one when it did not (a period whose writes do not land would garble it).
bool RvswdPhy::writesLand(uint8_t retries) {
  static const uint32_t kPatterns[] = {0xa5a5a5a5u, 0x5a5a5a5au, 0xffffffffu, 0x00000001u,
                                       0x0f0f0f0fu, 0xf0f0f0f0u, 0x80000000u, 0x7fffffffu};
  writeRaw(kDmAbstractAuto, 0);
  bool ok = true;
  uint8_t missed = 0;
  const uint32_t t0 = micros();
  for (int round = 0; round < kWriteRounds && ok; ++round) {
    if (round >= kMinWriteRounds && micros() - t0 >= kWriteCheckUs) break;
    for (uint32_t pattern : kPatterns) {
      for (;;) {
        writeRaw(kDmProgBuf0, pattern);
        uint32_t read_back = 0;
        if (readRaw(kDmProgBuf0, read_back) && read_back == pattern) break;
        if (missed++ >= retries) { ok = false; break; }
        ++search_retries_;
      }
      if (!ok) break;
    }
  }
  const uint32_t half = half_ns_;
  if (!ok) resyncAt(slowestNs());
  for (int attempt = 0; attempt <= kSlowestRetries; ++attempt) {   // the value back, read to confirm
    writeRaw(kDmProgBuf0, scratch_);
    uint32_t read_back = 0;
    if (readRaw(kDmProgBuf0, read_back) && read_back == scratch_) break;
    if (ok && missed++ >= retries) { ok = false; resyncAt(slowestNs()); }   // not at this period after all
  }
  if (half_ns_ != half) setHalf(half);
  return ok;
}

void RvswdPhy::useSafeSpeed() {
  // Also when attach() did not take: a CH32 held in reset may not answer, and the writes that follow the release
  // must still go out at a speed its default clock can follow.
  if (!ready_) return;
  setHalf(slowestNs());
  configureBus(false);
}

// attach()'s search without the wake: the wake burst resets the target, and this runs on a hart the caller has
// just stopped. It starts from the slowest period, which a reset has to be done at anyway, and speeds up while
// both checks pass. A period the target cannot follow leaves its debug module out of step, so after the first
// failure it steps back to the last good one and proves that again rather than trusting it.
bool RvswdPhy::retune() {
  if (!attached_) return false;
  const uint32_t floor = floorNs(), slowest = slowestNs();
  size_t good = kCount;   // index of the fastest period that passed
  resyncAt(slowest);
  if (!keepScratch(kSlowestRetries)) { useSafeSpeed(); return false; }
  for (size_t i = kCount; i-- > 0;) {
    if ((kHalfNs[i] < floor && i != kCount - 1) || (good != kCount && pastDeadline())) break;
    const uint32_t half = kHalfNs[i] > floor ? kHalfNs[i] : floor;
    const uint8_t retries = half == slowest ? kSlowestRetries : 0;
    resyncAt(half);
    uint32_t first = 0;
    if (!readsStable(first, retries) || !writesLand(retries)) break;
    good = i;
  }
  if (good == kCount) { useSafeSpeed(); return false; }
  // Prove the chosen period once more even when nothing failed on the way down: the fastest candidates are
  // marginal (half 0 ns sometimes fails for a whole run), and skipping this broke the X035's reset-halt (2026-09-25).
  for (int tries = 0; tries < 3 && !(tries && pastDeadline()); ++tries) {   // an attach's budget (attach under reset)
    const uint32_t half = kHalfNs[good] > floor ? kHalfNs[good] : floor;
    const uint8_t retries = half == slowest ? kSlowestRetries : 0;
    resyncAt(half);
    uint32_t first = 0;
    if (readsStable(first, retries) && writesLand(retries)) return true;
    if (good + 1 < kCount) ++good;   // slower
  }
  useSafeSpeed();
  return false;
}

// oep-if-debug §1: until the link speed is verified the probe writes only the wake / configuration sequence and
// dmactive, at the slowest period (the one every target follows and the one the resets use), and the speed is chosen
// by reads alone - a write garbled by a period the target cannot follow can land anywhere in the debug module. The
// slowest period's reads are checked first (margin check E156 / E157: 1000 identical reads), then faster periods are
// tried with DMSTATUS reads only; the fastest that passed must then pass the write check (writesLand) before it is
// kept - otherwise the next slower one is proved the same way. When only the slowest period is left, its reads were
// just checked: only the writes are. Each step starts only while the attach budget lasts (setDeadline).
bool RvswdPhy::attach() {
  if (!ready_) return false;
  if (attached_) return true;
  ioReclaim(swdio_, swclk_);
  const uint32_t floor = floorNs();
  const uint32_t slowest = slowestNs();
  // Two passes. A cold debug module can need more waking than one pass's eight attempts (2026-09-23: the first pass
  // failed where the same period attached immediately on the second).
  for (int pass = 0; pass < 2; ++pass) {
    if (pass) ++search_retries_;
    if (pastDeadline()) break;
    uint32_t first = 0;
    if (!wakeModule(first) || pastDeadline() || !readsStable(first, kSlowestRetries) ||
        !keepScratch(kSlowestRetries))
      continue;
    const uint32_t slowest_dmi_ns = dmi_ns_;
    // Faster, reading only. The first period that fails ends the search.
    size_t chosen = kCount;   // kCount = the slowest period (which may be a floor between the table's entries)
    for (size_t i = kCount - 1; i-- > 0;) {
      if (kHalfNs[i] < floor || pastDeadline()) break;
      setHalf(kHalfNs[i]);
      uint32_t value = 0;
      if (!readsStable(value, 0)) break;
      chosen = i;
    }
    // Settle there: bring the bus back in step (a failed faster read may have left it out), then prove the writes. A
    // period whose writes do not land gives way to the next slower one.
    while (!pastDeadline()) {
      const bool at_slowest = chosen == kCount;
      resyncAt(at_slowest ? slowest : kHalfNs[chosen]);
      uint32_t value = 0;
      const bool reads = at_slowest || readsStable(value, 0);
      if (at_slowest) dmi_ns_ = slowest_dmi_ns;
      if (reads && writesLand(at_slowest ? kSlowestRetries : 0)) {
        attached_ = true;
        return true;
      }
      if (at_slowest) break;
      ++search_retries_;
      ++chosen;
      if (chosen < kCount && kHalfNs[chosen] >= slowest) chosen = kCount;
    }
  }
  release();
  return false;
}

}  // namespace oep

#else  // stub for cores without a backend

namespace oep {
bool RvswdPhy::begin(int, int) { return false; }
bool RvswdPhy::usePins(int, int) { return false; }
bool RvswdPhy::attach() { return false; }
void RvswdPhy::release() { attached_ = false; }
void RvswdPhy::free() { attached_ = false; }
void RvswdPhy::park() { attached_ = false; }
bool RvswdPhy::readWire(uint8_t, uint32_t &) { return false; }
void RvswdPhy::write(uint8_t, uint32_t) {}
void RvswdPhy::setHalf(uint32_t) {}
void RvswdPhy::configureBus(bool) {}
void RvswdPhy::useSafeSpeed() {}
bool RvswdPhy::retune() { return false; }
void RvswdPhy::writeRaw(uint8_t, uint32_t) {}
void RvswdPhy::reviveIfIdle() {}
bool RvswdPhy::readRaw(uint8_t, uint32_t &) { return false; }
bool RvswdPhy::probeOnce(uint32_t, uint32_t &, bool) { return false; }
bool RvswdPhy::bringUp(uint32_t &) { return false; }
}  // namespace oep

#endif
