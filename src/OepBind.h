// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// What each serial port carries outside the frames (oep-spec interfaces/oep-if-probe-config.ja.md §1.2, transports §4):
// the binds. A bind is one stream - a slot's console or a fixture UART's receive side - and the port's raw bytes go to
// that stream's other end (console write, UART TX) as far as it takes them.
//
// The port follows the stream from a position of its own: never consumed, the port holds its place while nobody reads
// it, and falls to the oldest byte kept when a whole buffer went past it. While a session holds the port the endpoint
// asks nothing, so the position stays; when the session ends the port goes on from there (from the oldest byte kept
// if the stream overflowed meanwhile). The endpoint calls all of this from its poll(): one writer per port.
#pragma once

#include <Arduino.h>

#include "OepEndpoint.h"
#include "OepStream.h"

namespace oep {

class Binds final : public RawPorts {
 public:
  static constexpr size_t kMaxPorts = Endpoint::kMaxTransports;
  enum : uint8_t { kSlotConsole = reg::probe_config::kBindStreamSlotConsole, kFixtureUart = reg::probe_config::kBindStreamFixtureUart };
  struct Source { uint8_t kind = 0; uint16_t id = 0; BindSource *stream = nullptr; };
  struct Spec {
    bool set = false;
    Source source;
  };

  // A new bind for `port` (spec.set false: none). Its stream is followed from now.
  void set(uint8_t port, const Spec &spec);
  const Spec &spec(uint8_t port) const { return specs_[port < kMaxPorts ? port : 0]; }
  bool streaming(uint8_t port) const;   // a stream to carry now

  void rawIn(uint8_t port, const uint8_t *data, size_t length) override;
  size_t rawOut(uint8_t port, uint8_t *out, size_t room) override;
  void rawSent(uint8_t port, size_t length) override;
  void sessionOver(uint32_t held) override { (void)held; }   // the position stayed where it stopped

 private:
  struct Flow {
    bool known = false;       // followed at least once: a new stream number then starts at that stream's oldest
    uint16_t number = 0;
    uint64_t pos = 0;
  };
  Spec specs_[kMaxPorts];
  Flow flows_[kMaxPorts];
  const PositionStream *follow(uint8_t port);   // the stream, with its flow brought up to date
};

}  // namespace oep
