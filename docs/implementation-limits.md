# Implementation values and limits

[日本語](implementation-limits.ja.md)

**This implementation's document**, not part of the OEP specification. The Japanese text,
[implementation-limits.ja.md](implementation-limits.ja.md), is the current one; this page only says what it covers until it
is translated.

It gathers the values the OEP specification leaves to the probe and the limits this library and `examples/Firmware/OepProbe`
have on each platform. Another probe may choose other values, and a host relies on the specification's waits and
declarations, not on these.

1. **Every platform**: the declared values (max_op_ms, restart_max_ms, the lease default, the resend table, the console
   send queue); the debug wire's times and counts (retries, wire loss, the attach and scan budgets, the DM waits);
   how search_retries is counted; the per-request link check of dmi; no write repeated; DATA0 / DATA1 put back; one
   max_frame on every transport; the capture and fixture details (pretrigger limits, capture-group followers, segments
   that lost data, spi-target's bits and arm, the I2C / SPI controllers).
2. **Start-up and updates**: the boot guard and safe boot, the note on the previous boot in the firmware string, the
   ESP32-P4's DFU image, restart, where settings are saved.
3. **The wire PHYs**: RVSWD / SWIO recovery and acceptance, SWIO's bit timing, SWD's speeds.
4. **Each platform**: the classic ESP32's core assignment and its sampler windows against the SWIO wire and console;
   the ESP32-P4's USB stack; the RP2040 / RP2350 UART and flash.
5. **Hosts and bridges**: USB-UART bridges that drop bytes on long streams (500000 bps the practical rate), Linux
   cdc_acm bursts.
6. **TCP and Wi-Fi (classic ESP32)**: port 7450, up to three connections, mDNS `_oep._tcp`, the Wi-Fi settings, rough
   measurements and the limits.
