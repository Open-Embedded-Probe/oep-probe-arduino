// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Your first interface: a counter, no pins. An interface is a class with a name and a few operations (ops); the endpoint
// numbers it (fn 1 here) and the host finds it by its name. This one, io.github.example.hello revision 1:
//
//   op 0x01 add    value(u32)  ->  total(u32)   (changes the total: run only for the session holding the lock)
//   op 0x02 total  -           ->  total(u32)   (changes nothing: lock-free, no session needed)
//
// From Python (pip install oep-client-python):
//
//   from oep_client import core, link
//   hst = link.open_host("<port>"); hst.open(3000)       # a session, holding the lock
//   fn = core.find(hst, "io.github.example.hello")
//   hst.call(fn, 0x01, (5).to_bytes(4, "little")).payload    # b'\x05\x00\x00\x00'
//   hst.call(fn, 0x02, locked=False).payload                 # the total again, no lock needed
//
// Next: CustomInterface (a pin the host plans, describe), and
// docs/guide/your-first-interface.md. The name is yours to pick under something you own (io.github.<you>.<name>); oep.*
// is the specification's.
#include <OepEndpoint.h>
#include <OepPlatform.h>

#if defined(ARDUINO_ARCH_RP2040)
static constexpr uint8_t kTransport = oep::Endpoint::kUsbCdc;
static constexpr uint8_t kInterface = 0;   // the CDC communication interface's bInterfaceNumber (core §7.5)
#else
static constexpr uint8_t kTransport = oep::Endpoint::kUartBridge;
static constexpr uint8_t kInterface = 0xff;   // a UART bridge: no USB interface of the probe's (core §7.5)
#endif

class Hello final : public oep::Interface {
 public:
  static constexpr uint8_t kOpAdd = 0x01, kOpTotal = 0x02;

  // Who it is: the name the host looks for, which one of that name (0: the only one), the shape of its payloads
  const char *name() const override { return "io.github.example.hello"; }
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }
  // The ops it has: the endpoint answers any other unknown_operation for it
  bool offers(uint8_t op) const override { return op == kOpAdd || op == kOpTotal; }
  // The ops that change nothing run without the lock
  bool lockFree(uint8_t op) const override { return op == kOpTotal; }

  // One request in, its result out: write the answer's payload into out, say how it went
  oep::Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override {
    // Every request may end with TLVs (core §2.3). plainTail reads past the fixed part (its size: 4 bytes for add, none
    // for total), refuses a request too short or with a TLV it must not ignore, and lets the rest through.
    oep::Tail tail;
    const oep::Result parsed = oep::plainTail(tail, payload, length, op == kOpAdd ? 4 : 0, out, capacity);
    if (oep::refused(parsed)) return parsed;
    if (capacity < 4) return oep::failed();
    if (op == kOpAdd) total_ += oep::getU32(payload);
    oep::putU32(out, total_);
    return oep::completed(4);   // done, 4 bytes of payload
  }

  // The lock holder's session ended (end, a lapsed lease, another host's force): what it made goes with it
  void sessionOver() override { total_ = 0; }

 private:
  uint32_t total_ = 0;
};

static uint8_t rxBuffer[1100];
static uint8_t txBuffer[1024];
static oep::Endpoint endpoint(Serial, rxBuffer, sizeof rxBuffer, txBuffer, sizeof txBuffer, {1024, 4096, 8}, kTransport,
                              kInterface);
static Hello hello;
static uint8_t probeTlv[64];

static size_t describeProbe() {
  oep::TlvWriter w(probeTlv, sizeof probeTlv);
  uint8_t id[17];
  oep::describeCore(w, "hello-interface", id, oep::platformUnitId(id, sizeof id), 0);
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
  endpoint.add(hello);   // fn 1
}

void loop() {
  endpoint.poll();
}
