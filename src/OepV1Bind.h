// What each serial port carries outside the frames (oep-spec docs/oep-if-probe-config.ja.md §1.2, core §3.4): the
// binds. A bind is a list of streams (a slot's console, a fixture UART's receive side) and a mode:
//
//   last-reset  the stream of the target the host reset last (riscv-dm reset, attach_under_reset); the first at first
//   manual      the selected one (changed only by setting the bind again)
//   mixed       all of them, line by line, each line as "[name] line\n"; the port's raw bytes are dropped
//
// In last-reset and manual the port's raw bytes go to the selected stream's other end (console write, UART TX). The
// port follows each stream from a position of its own: never consumed, the port holds its place while nobody reads
// it, and falls to the oldest byte kept when a whole buffer went past it. While a session holds a port the endpoint
// asks nothing; when the session ends the port starts again from that session's last host reset of each stream (from
// now if there was none). The endpoint calls all of this from its poll(): one writer per port.
#pragma once

#include <Arduino.h>

#include "OepV1Endpoint.h"
#include "OepV1Stream.h"

namespace oep {
namespace v1 {

class Binds final : public RawPorts {
 public:
  static constexpr size_t kMaxPorts = Endpoint::kMaxTransports, kMaxStreams = 4;
  static constexpr size_t kLineMax = 128;              // a mixed line with no LF is closed at this many bytes
  static constexpr uint32_t kQuietMs = 100;            // ... or after this long without a byte
  enum : uint8_t {
    kLastReset = reg::probe_config::kBindModeLastReset, kManual = reg::probe_config::kBindModeManual,
    kMixed = reg::probe_config::kBindModeMixed,
  };
  enum : uint8_t { kSlotConsole = reg::probe_config::kBindStreamSlotConsole, kFixtureUart = reg::probe_config::kBindStreamFixtureUart };
  struct Source { uint8_t kind = 0; uint16_t id = 0; BindSource *stream = nullptr; };
  struct Spec {
    bool set = false;
    uint8_t mode = kLastReset, selected = 0, count = 0;
    Source sources[kMaxStreams];
  };
  // The mark in front of a mixed line: write the name of (kind, id) into out (at most `room`), return its length.
  using Namer = size_t (*)(void *context, uint8_t kind, uint16_t id, char *out, size_t room);

  void setNamer(Namer namer, void *context) { namer_ = namer; namer_context_ = context; }
  // A new bind for `port` (spec.set false: none). Its streams are followed from now.
  void set(uint8_t port, const Spec &spec);
  const Spec &spec(uint8_t port) const { return specs_[port < kMaxPorts ? port : 0]; }
  uint8_t selected(uint8_t port) const { return port < kMaxPorts ? selected_[port] : 0; }
  bool streaming(uint8_t port) const;   // something to carry now (the selected stream, or any in mixed)

  void rawIn(uint8_t port, const uint8_t *data, size_t length) override;
  size_t rawOut(uint8_t port, uint8_t *out, size_t room) override;
  void rawSent(uint8_t port, size_t length) override;
  void poll(bool session) override;
  void sessionOver(uint32_t held) override;

 private:
  struct Flow {
    bool known = false;       // followed at least once: a new stream number then starts at that stream's oldest
    uint16_t number = 0;
    uint64_t pos = 0;
    uint8_t line[kLineMax];   // mixed: the line so far
    uint8_t line_length = 0;
    uint32_t last_ms = 0;
  };
  struct Reset { bool have = false; uint16_t number = 0; uint64_t pos = 0; };
  Spec specs_[kMaxPorts];
  uint8_t selected_[kMaxPorts] = {};
  Flow flows_[kMaxPorts][kMaxStreams];
  uint32_t seen_resets_[kMaxPorts][kMaxStreams] = {};
  Reset resets_[kMaxPorts][kMaxStreams];
  uint8_t pending_[kMaxPorts][kLineMax + 48];          // mixed: a whole marked line waiting for room
  uint16_t pending_length_[kMaxPorts] = {}, pending_at_[kMaxPorts] = {};
  Namer namer_ = nullptr;
  void *namer_context_ = nullptr;
  const PositionStream *follow(uint8_t port, uint8_t index);   // the stream, with its flow brought up to date
  bool mixedLine(uint8_t port);                                // one closed line into pending_, if there is one
};

}  // namespace v1
}  // namespace oep
