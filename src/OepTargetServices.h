// target.control / target.memory / target.flash / target.console (owner 0, ids 0x10..0x13)
// over Ch32Dm.
// Failure is reported through outcomes and CRC/read-back, never through DMI parity alone.
#pragma once

#include "OepCh32Dm.h"
#include "OepService.h"

namespace oep {

uint32_t crc32Ieee(uint32_t crc, const uint8_t *data, size_t length);

class TargetControl final : public Service {
 public:
  // reset_pin: probe GPIO wired to the target's NRST (open-drain low pulse for reset mode 3), -1 = none.
  TargetControl(Ch32Dm &dm, DmiPhy &phy, int reset_pin = -1) : dm_(dm), phy_(phy), reset_pin_(reset_pin) {}
  uint16_t owner() const override { return OEP_V0_DEF_TARGET_CONTROL_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_TARGET_CONTROL_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_TARGET_CONTROL_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  size_t describe(uint8_t first, uint8_t *out, size_t capacity) override;
  void abandon() override;

 private:
  Ch32Dm &dm_;
  DmiPhy &phy_;
  int reset_pin_;
};

class TargetMemory final : public Service {
 public:
  explicit TargetMemory(Ch32Dm &dm) : dm_(dm) {}
  uint16_t owner() const override { return OEP_V0_DEF_TARGET_MEMORY_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_TARGET_MEMORY_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_TARGET_MEMORY_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  Ch32Dm &dm_;
};

// A console the target writes through the debug module's own data registers - no UART, no
// pin, no wiring, and the hart is never halted for it. The target blocks until the probe
// zeroes DATA0, so nothing is lost as long as somebody is collecting; bytes that arrive
// with no room left are counted instead.
class TargetConsole final : public Service {
 public:
  TargetConsole(Ch32Dm &dm, DmiPhy &phy) : dm_(dm), phy_(phy) {}
  uint16_t owner() const override { return OEP_V0_DEF_TARGET_CONSOLE_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_TARGET_CONSOLE_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_TARGET_CONSOLE_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
  void abandon() override;
  // Call from loop(). Collects at most one frame, and only while the target is attached
  // and running: those two registers are where abstract commands put their operands.
  void poll();

 private:
  static constexpr size_t kCapacity = 2048;
  Ch32Dm &dm_;
  DmiPhy &phy_;
  static constexpr size_t kTxCapacity = 256;
  bool enabled_ = false;
  uint8_t framing_ = 0;                 // 0 = SerialSDI (one way), 1 = SerialDMDATA, 2 = dmseq (two way)
  bool saw_empty_ = false;              // the target's empty frame was already there last poll
  uint16_t head_ = 0, tail_ = 0;
  uint16_t tx_head_ = 0, tx_tail_ = 0;
  uint32_t dropped_ = 0;
  uint32_t last_attach_ms_ = 0;
  uint8_t buffer_[kCapacity];
  uint8_t tx_[kTxCapacity];
  uint16_t buffered() const { return static_cast<uint16_t>((head_ - tail_ + kCapacity) % kCapacity); }
  uint16_t pending() const { return static_cast<uint16_t>((tx_head_ - tx_tail_ + kTxCapacity) % kTxCapacity); }
  void push(uint8_t byte);
  void pollSdi();
  void pollDmdata();
  void sendOrClear();
  // framing 2, dmseq (oep-spec docs/target-console-dmseq.ja.md)
  void pollSeq();
  void seqAnswer(uint8_t k, bool with_data);
  bool seq_synced_ = false;             // a target frame has been accepted this session
  uint8_t seq_last_s_ = 0;              // S of the last accepted target frame
  bool seq_last_syn_ = false;           // the last accepted target frame had SYN set
  uint8_t seq_h_ = 0;                   // H of the outstanding host payload
  uint8_t seq_chunk_[2] = {0, 0};
  uint8_t seq_chunk_len_ = 0;           // 0 = nothing outstanding
  uint8_t seq_bad_run_ = 0;             // consecutive invalid words
  uint8_t seq_syn_drops_ = 0;           // test hook OEP_CONSOLE_FAULT_SYN
};

class TargetFlash final : public Service {
 public:
  explicit TargetFlash(Ch32Dm &dm) : dm_(dm) {}
  uint16_t owner() const override { return OEP_V0_DEF_TARGET_FLASH_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_TARGET_FLASH_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_TARGET_FLASH_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  Ch32Dm &dm_;
  uint32_t page_[64];  // read-back buffer for one 256-byte page
  bool inRange(uint32_t address, uint32_t bytes) const;
};

}  // namespace oep
