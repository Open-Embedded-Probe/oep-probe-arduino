// OEP v1 draft ARM SWD interfaces (oep-spec docs/capability-name-hierarchy.ja.md; op tables provisional, 2026-09-24):
//
//   oep.wire.swd         scan / attach / detach on the probe's fixed SWD pair; attach wakes the port (JTAG-to-SWD,
//                        then the dormant wake), sends TARGETSEL when the host gives one, and returns DPIDR
//   oep.target.arm-adi   ADI (v5 / v6) access on that connection: a list of raw DP / AP transfers, and MEM-AP block
//                        reads / writes through TAR / DRW
//
// Like the RISC-V side, the probe knows nothing about the target: power-up requests, SELECT, CSW, the AP layout, the
// Cortex-M debug registers are all the host's. RP2040 / RP2350 SIO bit-bang only for now.
#pragma once

#if defined(ARDUINO_ARCH_RP2040)

#include "OepRp2BitBang.h"
#include "OepV1.h"

namespace oep {
namespace v1 {

struct SwdPort {
  uint16_t swdio, swclk;       // probe channels (GPIO numbers)
  uint32_t half_ns = 500;      // SWCLK half period
  bool connected = false;
  rp2::BitBang io;
};

class WireSwd final : public Interface {
 public:
  enum : uint8_t { kOpScan = 0x01, kOpAttach = 0x02, kOpDetach = 0x03 };
  WireSwd(SwdPort &port, uint16_t instance) : port_(port), instance_(instance) {}
  const char *name() const override { return "oep.wire.swd"; }
  uint16_t instance() const override { return instance_; }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  bool wake(const uint32_t *targetsel, uint32_t &dpidr, bool &dormant);
  SwdPort &port_;
  uint16_t instance_;
};

class TargetArmAdi final : public Interface {
 public:
  enum : uint8_t { kOpTransfer = 0x01, kOpReadBlock = 0x02, kOpWriteBlock = 0x03 };
  TargetArmAdi(SwdPort &port, uint16_t instance) : port_(port), instance_(instance) {}
  const char *name() const override { return "oep.target.arm-adi"; }
  uint16_t instance() const override { return instance_; }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  uint8_t xfer(bool ap, bool read, uint8_t a23, uint32_t &data);   // with WAIT retries; returns the last ACK
  SwdPort &port_;
  uint16_t instance_;
};

}  // namespace v1
}  // namespace oep

#endif
