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
§4.1 (classic ESP32): loop(), its interrupts, the Arduino events, the Wi-Fi driver and the TCP/IP stack run on core 0
and the logic sampler alone on core 1 (LoopCore=0, EventsCore=0; a Wi-Fi build refuses another arrangement), so the radio
changes none of the sampler's limits (radio traffic may still make a sample late: slipped); rate_range is 1 kHz to
300 kHz, from the bench (873c8f0: 400 kHz the fastest rate at which no immediate window and no forced capture without
SWIO traffic slipped, 500 kHz slipped; the table is in §4.1), samples round down to 200 ms of them (60000 at 300 kHz,
the most a segment holds; the 400 kHz floor kept a full window under the watchdog before spans ended by the clock), a trigger search does per sample what an immediate window does plus the trigger's test
(the clock, the wire's turns and force every 64 samples), and only host requests and the console's reads get turns in
a search - an at-boot slot's attach, retries and liveness checks and the console's re-attach wait while a sampler window
is open (the liveness check's frames, every retry_ms, slipped the bench's forced captures at every rate); every interrupts-off span
of the sampler ends by the clock at 250 ms (a search burst counted in samples ran past the 300 ms interrupt watchdog and
reset the probe; a segment still reading at the limit ends there, fewer samples, slipped). §6.1: a TCP slot's send buffer (6 KiB)
holds all a host within its window can have outstanding, so the probe never waits on or closes such a connection for a
network stall. §6.5: the measured extra loss on the UART0 link at 500000 bps with the radio on (1.64 / 0.76 / 0.55 %
against 0.47 / 0.08 / 0.17 % without Wi-Fi in the build), whole frames, no cause found in the firmware.
The firmware follows oep-spec 0f455a0 (0.0.29 development builds) and, with the external review re-check of 2026-10-07,
oep-spec 30b2b36 (with the wifi item, TCP discovery and unset's len as the key's length); §2.6 records what 0f455a0 removed (boot_reset and the rest). §1.6 adds: one max_frame for every transport
(the describe and max_length fit them all) and frames written whole, never paused inside; with oep-spec 9118dc0, a probe
with the wifi item answers max_frame 112 (`wifi_min_max_frame`) or more on every transport - `ProbeConfig::setWifi`
returns false and leaves the item undeclared below that, and the Wi-Fi sketches static_assert it (512 and 1024) - and
advertising over mDNS is the TCP probe's choice (transports §3; this one advertises).
§1.7 (oep-spec 0098b56..78fb561): the capture's pretrigger limits by board (ESP32-P4 logic declares 523263 at w = 1, less
for wider samples, up to samples - 1; the ESP32 analog keeps 129 short of samples), a capture-group follower without a
pretrigger (only a non-immediate trigger may have one), segment serials wrapping (255 infos kept), a segment that lost
data inside never handed out (the track stops in state 6, stopped reason 3, error 2, write_pos at its start), a
follower's ring that holds its segment plus what comes in while the trigger is on its way (the trigger track's latency:
the P4 logic's DMA chunk - 4032 bytes at most - plus 100 ms for loop(), refused cause 2 at bind when it cannot), the
P4's force at the sample of its instant, copied
streaming sending finished segments only, spi-target's bits at most length x 8 with no ns TLV, arm answering success once the slave holds the armed
transaction (post_setup_cb; 2 ms at most, else failed) and preload_tx once the slot is in the TX FIFO (or at the STOP
of the transfer under way), and the classic's cs_setup_ns (15 us: a master clocking sooner after CS falls cannot rely on
the first bit - the bench's back-to-back 4-byte frames read BC / FC for 3C, 0 / 5 at 3 MHz).
