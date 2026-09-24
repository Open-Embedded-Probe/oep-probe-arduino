// Experiment F3/F4 (2026-09-24, oep-spec docs/target-connection-use-cases.ja.md "flash の書き込み"):
// generic target parts with which the host, not the probe, knows how to program flash.
// Owner 0x0100, id 0x00F0; payloads are raw bytes decoded here and in the experiment script,
// on purpose outside the v0 registry while their shape is still being measured.
//
// op 0x01 steps: a list run in order, stopping at the first failure. Each step is
//   0x01 write32  address u32, value u32
//   0x02 read32   address u32                       (the value is appended to the result)
//   0x03 poll     address u32, mask u32, value u32, max_reads u16   ((read & mask) == value)
//   0x04 block    address u32, count u16, count x u32   (consecutive words, autoexec writer)
// result: done u16 (steps completed), status u8 (0 ok, 1 malformed, 2 target access, 3 poll
// gave up), then the read32 values.
//
// op 0x02 run: pc u32, timeout_ms u16, n u8, n x (regno u16, value u32)
// result: stopped u8, dpc u32, a0 u32, elapsed_us u32. The hart runs from pc until it stops
// (its own ebreak) or the timeout forces a halt; the host judges success from dpc and a0.
#pragma once

#include "OepCh32Dm.h"
#include "OepService.h"

namespace oep {

class ExpTargetPrimitives final : public Service {
 public:
  explicit ExpTargetPrimitives(Ch32Dm &dm) : dm_(dm) {}
  uint16_t owner() const override { return 0x0100; }
  uint16_t id() const override { return 0x00F0; }
  uint8_t revision() const override { return 0; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  Result steps(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Result run(const uint8_t *payload, size_t length, uint8_t *out, size_t capacity);
  Ch32Dm &dm_;
  uint32_t block_[256];
};

}  // namespace oep
