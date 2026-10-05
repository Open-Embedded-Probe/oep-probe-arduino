// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Your own interface. OEP fixes the protocol (frames, requests and results, the lock, discovery) and leaves what a
// probe can do open: anyone adds an interface under a reverse-DNS name of theirs, with a revision of its own, and asks
// nobody (oep-spec docs/oep-core.ja.md §2.7, §7.2). A host that does not know the name leaves it alone; one that does
// finds it by name with list and reads its limits with describe.
//
// This one, io.github.example.blink revision 1, blinks a pin:
//
//   op 0x01 set    level(u8: 0 / 1)          ->  -
//   op 0x02 blink  count(u8), half_ms(u16)   ->  -        (runs in loop(), the answer comes at once)
//   op 0x03 state  -                         ->  level(u8), left(u8)   (no lock: it changes nothing)
//
// Every request may end with TLVs (core §2.3): an unknown critical one refuses the request, an unknown other one is
// listed as ignored in the result - Tail does both. Which pin it drives is planned by the host (role 1), like the
// standard fixtures, so it never takes a pin something else holds. From Python:
//
//   from oep_client import core, link
//   hst = link.open_host("<port>"); hst.open(3000)
//   fn = core.find(hst, "io.github.example.blink")
//   core.plan_apply(hst, [(fn, 1, 25)])          # the Pico's LED
//   hst.call(fn, 0x02, bytes([5]) + (200).to_bytes(2, "little"))
//
// Pick the name from something you own (a GitHub account: io.github.<you>.<name>), never oep.*: that prefix is the
// specification's. Change the revision when the fixed part of a payload changes.
#include <OepEndpoint.h>
#include <OepPinTable.h>
#include <OepPlatform.h>

#if defined(ARDUINO_ARCH_RP2040)
static constexpr uint8_t kTransport = oep::Endpoint::kUsbCdc;
static constexpr uint64_t kChannels = ((1ull << 23) - 1) | (0x7ull << 26) | (1ull << 25);   // with the LED (GP25)
static constexpr uint16_t kChannelCount = 30;
#else
static constexpr uint8_t kTransport = oep::Endpoint::kUartBridge;
static constexpr uint64_t kChannels = (1ull << 2) | (1ull << 4) | (1ull << 5) | (0xfull << 12) | (0xfull << 16) |
                                      (0x7ull << 21) | (0x7ull << 25) | (0x3ull << 32);   // with GPIO2 (the LED)
static constexpr uint16_t kChannelCount = 40;
#endif

class Blink final : public oep::Interface {
 public:
  static constexpr uint8_t kOpSet = 0x01, kOpBlink = 0x02, kOpState = 0x03;
  static constexpr uint8_t kRoleLine = 1;

  explicit Blink(oep::PinTable &pins) : pins_(pins) {}
  const char *name() const override { return "io.github.example.blink"; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }
  bool lockFree(uint8_t op) const override { return op == kOpState; }

  // What the host reads before using it: the pins it may plan for role 1 (role_channels, core §7.4).
  size_t describe(uint8_t *out, size_t capacity) override {
    oep::TlvWriter w(out, capacity);
    static const uint8_t kRoles[] = {kRoleLine};
    w.roleChannels(kRoles, 1, pins_.allowedMask());
    return w.ok() ? w.length() : 0;
  }

  // The plan (core §8): check without changing anything, then take the pin; release gives it back. A role or channel
  // this interface does not declare is unsupported; a declared channel something else holds is unavailable. Taking the
  // pin changes nothing on it (an output idle keeps driving): the first set makes it this interface's output.
  bool planRoles() const override { return true; }   // its line is a plan role (core §1.2, §8)
  uint8_t planCheck(const oep::RoleAssignment *roles, size_t count) override {
    if (count != 1) return oep::kRejectMalformed;
    if (roles[0].role != kRoleLine || !pins_.allowed(roles[0].channel)) return oep::kRejectUnsupported;
    return pins_.free(roles[0].channel) ? 0 : oep::kRejectUnavailable;
  }
  bool planApply(const oep::RoleAssignment *roles, size_t count) override {
    if (count != 1 || !pins_.claim(roles[0].channel, kOwner)) return false;
    pin_ = roles[0].channel;
    driving_ = false;
    return true;
  }
  void planRelease() override {
    left_ = 0;
    pins_.release(kOwner);   // back to its idle state (Hi-Z unless set)
    pin_ = -1;
  }

  oep::Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override {
    oep::Tail tail;
    switch (op) {
      case kOpSet: {   // level(u8) [TLV]
        const oep::Result parsed = oep::plainTail(tail, payload, length, 1, out, capacity);
        if (oep::refused(parsed)) return parsed;
        if (pin_ < 0) return oep::wrongState(out, capacity);   // not planned: unavailable cause 6 (core §4.3)
        if (payload[0] > 1) return oep::rejected(oep::kRejectMalformed);
        left_ = 0;
        level_ = payload[0];
        drive();
        return tail.finish(oep::completed(), out, capacity);
      }
      case kOpBlink: {   // count(u8) half_ms(u16) [TLV]
        const oep::Result parsed = oep::plainTail(tail, payload, length, 3, out, capacity);
        if (oep::refused(parsed)) return parsed;
        if (pin_ < 0) return oep::wrongState(out, capacity);
        const uint16_t half = oep::getU16(payload + 1);
        if (half == 0) return oep::rejected(oep::kRejectMalformed);
        left_ = static_cast<uint16_t>(payload[0]) * 2;   // edges to go
        half_ms_ = half;
        last_ms_ = millis();
        return tail.finish(oep::completed(), out, capacity);
      }
      case kOpState: {   // [TLV] -> level(u8) left(u8: blinks to go)
        const oep::Result parsed = oep::plainTail(tail, payload, length, 0, out, capacity);
        if (oep::refused(parsed)) return parsed;
        if (capacity < 2) return oep::failed();
        out[0] = level_;
        out[1] = static_cast<uint8_t>((left_ + 1) / 2);
        return tail.finish(oep::completed(2), out, capacity);
      }
      default:
        return oep::rejected(oep::kRejectUnknownOperation);
    }
  }

  void poll() {   // the blinking: from loop(), never waiting
    if (!left_ || pin_ < 0 || millis() - last_ms_ < half_ms_) return;
    last_ms_ = millis();
    level_ ^= 1;
    drive();
    --left_;
  }

 private:
  static constexpr uint8_t kOwner = 1;   // this interface's id in the pin table
  oep::PinTable &pins_;
  int pin_ = -1;
  uint8_t level_ = 0;
  bool driving_ = false;   // the pin is this interface's output (from its first set or blink)
  void drive() {
    digitalWrite(pin_, level_);   // the level first, so the output starts at it
    if (!driving_) { pinMode(pin_, OUTPUT); digitalWrite(pin_, level_); driving_ = true; }
  }
  uint16_t left_ = 0, half_ms_ = 0;
  uint32_t last_ms_ = 0;
};

static uint8_t rxBuffer[1100];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8}, kTransport);
static oep::PinTable pins(kChannels);
static Blink blink(pins);
static uint8_t probeTlv[64];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, "custom-interface", id, oep::platformUnitId(id, sizeof id), kChannelCount,
                    ((1ull << kChannelCount) - 1) & ~kChannels);
  return w.ok() ? w.length() : 0;
}

void setup() {
#if defined(ARDUINO_ARCH_RP2040)
  Serial.ignoreFlowControl(true);   // answer whatever DTR the host left (probe-development-guide §1)
#elif defined(ARDUINO_ARCH_ESP32)
  esp_log_level_set("*", ESP_LOG_NONE);   // no log on a port that carries OEP (probe guide §3): UART0 is the transport
#endif
  Serial.begin(115200);
  endpoint.setProbeDescription(probeTlv, describeProbe());
  endpoint.add(blink);
}

void loop() {
  endpoint.poll();
  blink.poll();
}
