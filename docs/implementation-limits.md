# Implementation values and limits

[日本語](implementation-limits.ja.md)

Status: **this implementation's document** (not part of the OEP specification). The Japanese text,
[implementation-limits.ja.md](implementation-limits.ja.md), is the current one; this English page is a stub until it is translated.

It gathers, in one place, the values the OEP specification leaves to the probe and the limits this library and
`examples/Firmware/OepProbe` have on each platform: the declared values (max_op_ms 10000 ms, restart_max_ms, the lease default, the
resend table, the console send queue), the wire's times and counts (200 ms of retries in a request, 1000 ms to wire loss, the 1000 ms
attach and 500 ms scan budgets, 700 ms for a silent debug module after a reset, 100 ms DM waits, 100 DMI busy / SWD WAIT retries),
search_retries' counting, the per-request link check of dmi, no wake on a connection outside attach and reset, no write repeated,
DATA0 / DATA1 put back after the probe's own abstract commands, the boot guard and safe boot, the reset note in the firmware text, the
ESP32-P4's DFU image (valid at start-up), the RVSWD / SWIO / SWD PHY details, the classic ESP32's sampler windows and the wire / console,
the ESP32-P4's TinyUSB DWC2 panic on a SETUP in a status stage (EspUsbDevice 2.5.1; the chip resets), and the USB-UART bridges that drop
bytes on long probe-to-host streams (500000 bps the practical rate). A host relies on the specification's waits and declarations, not on
these values. §6 covers TCP over Wi-Fi on the classic ESP32: port 7450, up to three connections (each its own transport,
the listener one describe entry), mDNS `_oep._tcp` with host `oep-<unit_id>.local` and TXT `unit_id`, the probe.config
wifi item 0x08 (several networks tried in index order, the passphrase write-only), restart_max_ms 15000 with Wi-Fi, the
measurements on the ATOM, and the limits (sampler windows pause the network, ESP32-P4 and RP2040 / RP2350 without Wi-Fi).
The firmware follows oep-spec 0f455a0 (0.0.29 development builds) and, with the external review re-check of 2026-10-07,
oep-spec 30b2b36 (with the wifi item, TCP discovery and unset's len as the key's length); §2.6 records what 0f455a0 removed (boot_reset and the rest). §1.6 adds: one max_frame for every transport
(the describe and max_length fit them all) and frames written whole, never paused inside.
