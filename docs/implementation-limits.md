# Implementation values and limits

[日本語](implementation-limits.ja.md)

Status: **this implementation's document** (not part of the OEP specification). The Japanese text,
[implementation-limits.ja.md](implementation-limits.ja.md), is the current one; this English page is a stub until it is translated.

It gathers, in one place, the values the OEP specification leaves to the probe and the limits this library and
`examples/Firmware/OepProbe` have on each platform: the declared values (max_op_ms 10000 ms, restart_max_ms, the lease default, the
resend table, the console send queue), the wire's times and counts (200 ms of retries in a request, 1000 ms to wire loss, the 1000 ms
attach and 500 ms scan budgets, 700 ms for a silent debug module after a reset, 100 ms DM waits, 100 DMI busy / SWD WAIT retries),
search_retries' counting, the per-request link check of dmi, the boot guard and safe boot, the reset note in the firmware text, the
ESP32-P4's DFU image (valid at start-up), the RVSWD / SWIO / SWD PHY details, the classic ESP32's sampler windows and the wire / console,
the ESP32-P4's TinyUSB DWC2 panic on a SETUP in a status stage (EspUsbDevice 2.5.1; the chip resets), and the USB-UART bridges that drop
bytes on long probe-to-host streams (500000 bps the practical rate). A host relies on the specification's waits and declarations, not on
these values.
