// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// What a serial port carries outside the frames (oep-spec transports §4, fixture §2.1 / console §2.1): a pass-through's
// bytes - what a fixture UART received, or a target's console output - sent between frames while no session uses the
// port (the endpoint asks nothing of a port a session's requests came on). Each port has a small ring the
// pass-through fills; what the port could not take before the pass-through stopped is dropped (config §1.1: not kept).
// The other direction is not defined (input goes by OEP write): bytes outside frames that come in are dropped.
#pragma once

#include <Arduino.h>

#include "OepEndpoint.h"

namespace oep {

class Binds final : public RawPorts {
 public:
  static constexpr size_t kMaxPorts = Endpoint::kMaxTransports, kRing = OEP_SMALL_RAM ? 256 : 512;

  // The pass-through of `port` adds bytes (the oldest go when the ring is full: a serial monitor sees the newest).
  void put(uint8_t port, const uint8_t *data, size_t length);
  // The pass-through of `port` stopped: what it had not sent goes.
  void drop(uint8_t port);

  void rawIn(uint8_t, const uint8_t *, size_t) override {}
  size_t rawOut(uint8_t port, uint8_t *out, size_t room) override;
  void rawSent(uint8_t port, size_t length) override;
  void sessionOver(uint32_t) override {}

 private:
  struct Ring { uint8_t data[kRing]; size_t head = 0, count = 0; };
  Ring rings_[kMaxPorts];
};

}  // namespace oep
