// OEP v1 oep.probe.config (oep-spec v1-core-wire-delta.ja.md §5.10, revision 1) - prototype P4 of
// probe-cdc-and-persistence §7: the probe's settings, set by the host and kept only when the host saves them.
//
//   0x01 get(first u16) -> more(u8) hash(u32) items      (no lock)
//   0x02 set(items) -> hash(u32)    0x03 save -> hash(u32)    0x04 erase    0x05 reboot (answers, then restarts)
//
// Items (TLV, tag u8 len u8 value): 0x01 boot_mode(u8), 0x02 plan (fn u16 role u8 channel u16) x n - the endpoint's
// plan itself, 0x04 bind (port u8 source u8 attach u8 flags u8, source args: fixture.uart fn u16 / target.console
// wire_fn u16 mechanism u8), 0x05 target (wire_fn u16 chip_id u32; needed by a bind that attaches by itself). A set
// replaces every item of the tags it carries; a tag sent with length 0 clears it. label (0x03) is not in this
// prototype (refused as unsupported). No defaults: nothing the host did not set is done - the boot mode without a saved one is the sketch's
// own choice. Saved to NVS on ESP32 (Preferences, namespace "oepcfg").
#pragma once

#include <Arduino.h>

#include "OepV1.h"

#if defined(ARDUINO_ARCH_ESP32)

namespace oep {
namespace v1 {

class Endpoint;

class ProbeConfig final : public Interface {
 public:
  struct Mode {
    uint8_t functions;   // bit0 vendor bulk, bit1 CDC OEP port, bit2 HID OEP port, bit3 Mass Storage, bit4 DFU runtime
    uint8_t ports;       // data CDC ports
    const char *name;
  };
  static constexpr size_t kMaxPorts = 4, kMaxBindArgs = 8, kMaxSaved = 512;
  struct Bind {
    bool set = false;
    uint8_t source = 0, attach = 0, flags = 0, arg_length = 0;
    uint8_t args[kMaxBindArgs] = {};
  };
  struct Target {
    bool set = false;
    uint16_t wire_fn = 0;
    uint32_t chip_id = 0;
  };
  // A bind to apply (source 0: clear the port). false: the probe cannot do it (the set is refused, nothing changes).
  using BindHook = bool (*)(uint8_t port, const Bind &bind, void *context);

  // port_interfaces: the USB interface number of each data port of the running mode (describe port, for the OS);
  // as many as the mode's ports. The configuration does not depend on the mode: a bind to a port the running mode
  // does not have is kept and does nothing until a mode with that port runs (probe-cdc-and-persistence §7.3).
  ProbeConfig(Endpoint &endpoint, const Mode *modes, size_t mode_count, const uint8_t *port_interfaces)
      : endpoint_(endpoint), modes_(modes), mode_count_(mode_count), port_interfaces_(port_interfaces) {
    for (size_t i = 0; i < mode_count; ++i) if (modes[i].ports > port_count_) port_count_ = modes[i].ports;
    if (port_count_ > kMaxPorts) port_count_ = kMaxPorts;
  }
  void setBindHook(BindHook hook, void *context) { hook_ = hook; hook_context_ = context; }

  // Before USB starts: read what was saved. bootMode(fallback): the mode to enumerate now (the saved one, else the
  // sketch's own). Then, once the interfaces are added: applySaved() applies the saved plan and binds.
  void load();
  uint8_t bootMode(uint8_t fallback);
  void applySaved();
  void poll();   // from loop(): a reboot asked for goes after its answer has left
  const Target &target() const { return target_; }
  const Bind &bind(uint8_t port) const { return binds_[port < kMaxPorts ? port : 0]; }

  const char *name() const override { return reg::probe_config::kName; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return reg::probe_config::kRevision; }
  bool lockFree(uint8_t op) const override { return lockFreeIn(reg::probe_config::kLockFreeOps, op); }
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

 private:
  Endpoint &endpoint_;
  const Mode *modes_;
  size_t mode_count_;
  const uint8_t *port_interfaces_;
  size_t port_count_ = 0;   // the most ports any mode has: binds may name any of them
  BindHook hook_ = nullptr;
  void *hook_context_ = nullptr;
  uint8_t current_mode_ = 0, boot_mode_ = 0xFF;   // boot_mode_: the item (0xFF: not set)
  Bind binds_[kMaxPorts];
  Target target_;
  uint8_t storage_state_ = reg::probe_config::kStorageStateNone;
  uint32_t saved_hash_ = 0, save_ms_ = 0;
  uint8_t saved_[kMaxSaved];
  size_t saved_length_ = 0;
  uint32_t reboot_at_ = 0;
  bool reboot_ = false;

  size_t canonical(uint8_t *out, size_t capacity) const;   // the items in their canonical order (the hash's input)
  uint32_t hash() const;
  Result apply(const uint8_t *items, size_t length);
  size_t livePorts() const { return current_mode_ < mode_count_ ? modes_[current_mode_].ports : 0; }
};

uint32_t crc32Ieee(const uint8_t *data, size_t length);

}  // namespace v1
}  // namespace oep

#endif
