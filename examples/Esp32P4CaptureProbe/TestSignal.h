// oep.test.signal (a test interface of this example, not a standard one): an LEDC square on a pin, for trying
// captures on an unwired board.
//   0x01 start(pin u8, hz u32, duty_percent u8)   0x02 stop(pin u8)
#pragma once
#include <OepV1.h>

class TestSignal final : public oep::v1::Interface {
 public:
  const char *name() const override { return "oep.test.signal"; }
  uint16_t instance() const override { return 0; }
  oep::Result handle(uint8_t op, const uint8_t *p, size_t n, uint8_t *, size_t) override {
    if (op == 0x01 && n == 6) {
      const uint8_t pin = p[0];
      const uint32_t hz = oep::v1::getU32(p + 1);
      uint8_t bits = 10;                          // the finest resolution the LEDC clock allows at hz
      ledcDetach(pin);
      while (bits > 1 && !ledcAttach(pin, hz, bits)) --bits;
      if (bits == 1 && !ledcAttach(pin, hz, bits)) return oep::failed();
      ledcWrite(pin, (uint32_t)p[5] * (1u << bits) / 100);
      gpio_input_enable((gpio_num_t)pin);   // so a capture on the same pad sees it
      return oep::completed();
    }
    if (op == 0x02 && n == 1) { ledcDetach(p[0]); pinMode(p[0], INPUT); return oep::completed(); }
    return oep::rejected(oep::kRejectMalformed);
  }
};
