// OEP v1 oep.probe.config (oep-spec docs/oep-if-probe-config.ja.md, revision 1) - prototype P4 of
// probe-cdc-and-persistence §7: the probe's settings, set by the host and kept only when the host saves them.
//
//   0x01 get(first u16) -> more(u8) hash(u32) items      (no lock)
//   0x02 set(items) -> hash(u32)    0x03 save -> hash(u32)    0x04 erase    0x05 reboot (answers, then restarts)
//
// Items (TLV, tag u8 len u8 value), each with a key; a set replaces the keys it carries and leaves the others:
//   0x01 boot_mode(u8)                                   (one; length 0 clears)
//   0x02 plan fn(u16) role(u8) channel(u16)              key fn: the items of a fn are that fn's plan (fn alone clears)
//   0x04 bind port(u8) source(u8) attach(u8) flags(u8) args   key port (port alone clears); args fixture.uart fn u16
//        baud u32 format u8, target.console wire_fn u16 mechanism u8 swdio u16 swclk u16 max_speed u32
//   0x05 target wire_fn(u16) scheme(u8) mask(n) value(n) key wire_fn: what an automatic attach's target_id must match
//   0x06 idle channel(u16) mode(u8)                      key channel: 0 Hi-Z, 1 pull-up, 2 pull-down while free
// label (0x03) is not in this prototype (refused as unsupported). No defaults: nothing the host did not set is done -
// the boot mode without a saved one is the sketch's own choice. Saved to NVS on ESP32 (Preferences, namespace
// "oepcfg") with the identity of the interface list; a saved copy made for another list is not applied.
#pragma once

#include <Arduino.h>

#include "OepFixtureServices.h"
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
  static constexpr size_t kMaxPorts = 4, kMaxBindArgs = 16, kMaxSaved = 512, kMaxTargets = 2, kMaxIdMask = 8;
  static constexpr size_t kUartArgs = 7, kConsoleArgs = 11;
  struct Bind {
    bool set = false;
    uint8_t source = 0, attach = 0, flags = 0, arg_length = 0;
    uint8_t args[kMaxBindArgs] = {};
  };
  struct Target {
    bool set = false;
    uint16_t wire_fn = 0;
    uint8_t scheme = 0, length = 0;   // length: bytes of mask and of value
    uint8_t mask[kMaxIdMask] = {}, value[kMaxIdMask] = {};
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
    memset(idle_, PinTable::kIdleUnset, sizeof idle_);
  }
  void setBindHook(BindHook hook, void *context) { hook_ = hook; hook_context_ = context; }
  // The pins whose idle state the idle item sets (without it, idle items are refused).
  void setPins(PinTable *pins) { pins_ = pins; }

  // Before USB starts: read what was saved. bootMode(fallback): the mode to enumerate now (the saved one, else the
  // sketch's own). Then, once the interfaces are added: applySaved() applies the saved idle states, plan and binds.
  void load();
  uint8_t bootMode(uint8_t fallback);
  void applySaved();
  // The saved boot mode could not be built (the probe's own safety): drop it from the saved copy and the items.
  // true: there was one to drop.
  bool forgetBootMode();
  void poll();   // from loop(): a reboot asked for goes after its answer has left
  const Bind &bind(uint8_t port) const { return binds_[port < kMaxPorts ? port : 0]; }
  // The target item of a wire (nullptr: none), and whether a target_id read by an automatic attach matches it.
  const Target *target(uint16_t wire_fn) const;
  bool matches(uint16_t wire_fn, uint8_t scheme, const uint8_t *id, size_t length) const;
  // What became of a bind's activation (describe bind_state): the sketch reports it.
  void setBindState(uint8_t port, uint8_t state) { if (port < kMaxPorts) bind_state_[port] = state; }

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
  PinTable *pins_ = nullptr;
  uint8_t current_mode_ = 0, boot_mode_ = 0xFF;   // boot_mode_: the item (0xFF: not set)
  Bind binds_[kMaxPorts];
  uint8_t bind_state_[kMaxPorts] = {};
  Target targets_[kMaxTargets];
  uint8_t idle_[PinTable::kChannels];             // the idle items (kIdleUnset: none)
  uint8_t storage_state_ = reg::probe_config::kStorageStateNone;
  uint32_t saved_hash_ = 0, save_ms_ = 0, saved_list_ = 0;
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
