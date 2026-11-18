# Values and limits of the reference implementation (implementation limits)

[日本語](implementation-limits.ja.md)

Status: **this implementation's document** (not the OEP specification). It gathers in one place the values the OEP
specification (oep-spec) leaves to the probe, and the limits this implementation (the OpenEmbeddedProbe library and
`examples/Firmware/OepProbe`) has per platform. The specification's norms do not depend on this document: another probe may
choose other values, and the host does not rely on the values here (it relies only on the specification's waits and the
declarations).

TCP and Wi-Fi (port, mDNS, Wi-Fi settings, measured values) are in §6.

## 1. Common to every platform

### 1.1 Declared values

| Value | This implementation | Specification |
|---|---|---|
| max_op_ms (fn 0's describe) | 10000 ms | 1-600000 ms, the probe chooses the value (core §7.5) |
| restart_max_ms (`oep.probe.restart`) | ESP32-P4 3000 ms, classic ESP32 15000 ms (the Wi-Fi build, §6.1; 1500 ms in a build without Wi-Fi (`kWifi`)), RP2040 / RP2350 2000 ms, CH32 2000 ms | the probe chooses (oep-if-restart §1) |
| default lease (open's lease_ms 0) | 3000 ms | 1000-60000 ms (core §6.4) |
| resend table | 8 entries; one answer up to 1029 bytes (resending a larger answer is result_lost) | at least max_inflight entries (core §5.1) |
| console send queue | 256 bytes per stream | the probe chooses the size (oep-if-console §2) |
| model, chip | the name of the chip built for, in lowercase without hyphens (`esp32p4`, `esp32p4x`, `esp32`, `rp2040`, `rp2350`, `ch32x035`, `ch32v307`, `ch32h417`); the running part's part number and revision (for example `esp32p4 v1.3`) | any free string (core §7.5) |
| unit_id | the chip's unique number in lowercase hex | core §7.5 |

### 1.2 Wire times and counts

The specification (oep-if-debug §1-§4) only requires attach, scan and riscv-dm's reset to answer within max_op_ms, and the
host waits the time in their arguments as max_op_ms. This implementation's actual values:

| Value | This implementation | Meaning |
|---|---|---|
| wire retries within one request | 200 ms | includes retries at slower speeds. When used up, status line. That alone does not make the wire lost |
| wire lost | 1000 ms of real time | failures with no answer from the wire continue, with no success in between. The time the reset line is held, and 1000 ms after it is released, do not count |
| attach budget | 1000 ms | speed search and retries. Does not include the reset TLV's hold_ms or the DM wait below |
| scan budget | 500 ms | no pair is started later than this after the request arrives (at least one pair is tried). One pair is bounded by the attach budget |
| pairs in one scan | at most 255 | tried is u8 |
| DM wait after releasing the reset line | 700 ms | re-reads DMSTATUS at the slowest speed. attach's reset TLV and riscv-dm's reset op. Not counted as wire retries |
| DM waits inside high-level ops | 100 ms per wait | abstractcs.busy, allhalted, allresumeack |
| hart wait of the reset op | 100 ms per sequence, the sequence redone at most once | |
| DMI busy | retried up to 100 times, then status wait | |
| SWD WAIT | retried up to 100 times, then status wait | |
| arm-adi TAR rewrite | at every 1 KiB boundary | the range in which ADI guarantees auto-increment |
| read_block / write_block max_length | max_frame − 24 or less | the request and answer fit in max_frame |
| console DMSTATUS | after answering that connection's riscv-dm requests (and the wire's attach), before the next DATA0 read. Besides that, every 20 ms | the specification (console §3) requires only the former. The 20 ms reads are this implementation's interval for seeing that a halted hart started running and that the target restarted by itself (havereset) |

This implementation's upper bound on waiting for the attach answer is 1000 ms + hold_ms + 700 ms (shorter than max_op_ms's
10000 ms). The values are gathered in the library's `src/OepLimits.h` (`oep::limits`).

**Retries that do not change the target's state (specification oep-if-debug §2)**: while there is a connection, re-establishing
the wire sends only the setup pair (RVSWD: DMI 0x7E / 0x7D; SWIO: the same pair, plus dmactive when DMCONTROL reads dmactive as
0). The RVSWD wake pattern, which can reset the target itself, is sent on a connection only inside attach (bringing up a new
connection, re-establishing an existing connection that does not answer, the reset TLV) and riscv-dm's reset op
(`DmiPhy::holdWakes` / `DmiPhy::WakeScope`). Re-establishing inside a console read sends no wake, so a wire that comes back
only with a wake closes as wire lost (1000 ms) and comes back with the host's attach. SWD's line reset and wake from dormant
do not change the target's state, so they are outside this restriction.

**Writes are not repeated (specification oep-if-debug §2)**: the PHY sends a DMI write only once (wire retries are for reads
only). The host's dmi steps are not redone; on failure, the answer gives done as the number of steps before it. write_block's
word writes (stores into the target's memory) happen once. An SWD write transfer is redone only when there was no ACK (the
target did not take the write) and on WAIT. Inside a high-level op, a group in which a wire drop was seen (§1.4) is redone
only for settings that put a value the probe chose into a register (a0 / a1, program buffer, DATA1's address, dcsr, run's host
registers and dpc, abstractauto and the write-back of DATA0 / DATA1), putting the same value again. Writes into the target's
memory and writes whose meaning changes after one time (resumereq, ndmreset) are not redone.

### 1.3 How search_retries is counted

attach's answer's search_retries (in the specification "for diagnostics, the implementation decides how to count") is the
number of tries needed beyond the first try of each stage of bring-up (the wake / setup sequence, choosing a speed, confirming
it). Each of the following counts as 1: one redone confirming read, or one write with its read-back. A redone pass (a group of
read or write round trips at one speed). The switch to a slower speed after a write check fails at the chosen speed. Redoing
the whole bring-up. On wires with a wake, the unanswered wakes before the answered one. A read that failed at a fast speed and
ended the search of fast speeds is not counted. It stops at 0xFFFF. It is sent only in an attach that did a bring-up (a new
connection, with a reset TLV, an existing connection whose speed was lowered for max_speed).

### 1.4 Wire check per dmi request

On some wires, after the hart's state changes the wire drops for 0.7-2 ms. During that, writes vanish and reads return the
previous value or all ones (not an error). This implementation looks at the wire once before and once after the steps of each
dmi request (DMSTATUS is the module's with authenticated set, and DMCONTROL's bit 7 is 0: a dropped wire returns all ones or
the previous value to both, so both cannot hold). It also checks by count that no PHY re-establishment came in between. A
request that fails the check answers status line without values, and done is the number of steps completed before the check
failed (writes among them may have happened). High-level ops (block, run, step, reset) split their steps into groups and check
the wire per group (also looking at DMCONTROL's dmactive and hart 0). The specification requires only that a request that
found a wire failure does not return success, and that writes are not repeated (oep-if-debug §2).

### 1.5 Writing back DATA0 / DATA1

The console mechanisms use DATA0 / DATA1 as a mailbox with the target. The probe restores the DATA0 / DATA1 used by the
abstract commands of its own high-level ops before answering (specification oep-if-debug §4): read_block / write_block, step,
the reset check, attach's read of dpc. run remembers DATA0 / DATA1 before the preparation (dcsr, host registers, dpc), writes
them back before letting the hart run, and restores them once more after reading dpc and the values after it halts. If the
preparation fails, it writes them back and then answers stopped 3 (did not run).

### 1.6 Other

- **Raw byte delay**: because of how serial ports are received (transports §4), raw bytes that came after 0x00 reach the
  bound stream only when the next 0x00 comes or the input pauses for 200 ms. A binary stream containing 0x00 is delayed up to
  200 ms at each 0x00.
- **Limits of SDI and DMDATA**: these two console mechanisms have no numbers on their frames, so neither the probe nor the
  target can tell that a wire write was lost (DATA0's "received" 0 does not arrive, an answer does not arrive). The same
  frame can arrive twice, and with DMDATA an input byte carried on an answer can be lost. If neither duplicates nor losses are
  allowed, use dmseq.
- **max_frame is the same on every transport**: one `Endpoint`'s max_frame / window / max_inflight are the same values
  (`Limits`) on every serial port, USB and TCP connection. describe's TLVs and max_length follow from those values, so they fit
  in every transport's max_frame (core §7.3, §7.4). There is no per-transport max_frame. A probe with the Wi-Fi items answers
  max_frame 83 (`wifi_min_max_frame`) or more on every transport (transports §7: the get answer for the single longest value
  is 19 + 64 = 83 bytes). `Settings::setWifi` returns false and does not declare the Wi-Fi items when the `Endpoint`'s
  max_frame is below that, and sketches with Wi-Fi (the classic ESP32 firmware's `kMaxFrame` 512, `03.Transports/WifiTcp`'s
  1024) check it with static_assert.
- **Frames are written without stopping midway**: answers and notifications are written one whole frame at a time within one
  `poll()`, with no other work in the middle of a frame (transports §2). The ESP32-P4's vendor bulk puts a whole frame into one
  buffer and drops it whole if it does not fit. It can stall only when the host stops reading and the transport's buffer fills.
- **Opening does not restart it**: on every transport, opening, closing and DTR / RTS do not restart the probe (the
  USB-Serial/JTAG's DTR / RTS reset is disabled).

### 1.7 Capture and fixtures

- **The pretrigger limit** (capture §3.3: above max_pretrigger, or samples or more, is unsupported):
  - ESP32-P4 logic: a one-shot with a trigger uses a 1 MiB ring in PSRAM (an internal 64 KiB on boards without PSRAM), and
    returns the pre-trigger part from it. How much it can return depends on the data rate (rate × w / 8): up to 30 MB/s a
    quarter of the ring (256 KiB), faster than that an eighth (128 KiB). 32 KiB on boards without PSRAM. This is because the
    pre-trigger part is copied all at once when the trigger comes, and a segment is lost if the ring wraps before the copy
    catches up with the DMA. describe's max_pretrigger is the value at w = 1 and the fastest 160 MS/s (20 MB/s) (2097152 with
    PSRAM). A wider w gives less (524288 at w = 2, 160 MS/s). A pretrigger above that is refused by configure / query as
    unsupported (the pretrigger tag). A pretrigger up to samples − 1 is accepted. The segment starts pretrigger samples before
    the trigger (even mid-byte when w < 8; trigger_index = pretrigger). A trigger or force before the pretrigger has filled
    gives a short segment (capture §3.3: trigger_index is what there was, and samples is that much less). force's trigger is
    the sample at that moment (decided by the time since start; it does not wait for a DMA chunk). The ring is not the stream
    laid out as is (the DMA closes a descriptor midway every eof_data_len = 65408 bytes and continues from the head of the
    next descriptor), so the pre-trigger part is copied through a map of the recent chunks.
  - classic ESP32 logic: up to samples − 1 (describe gives 59999).
  - Analog: RP2040 / RP2350 up to samples − 1 (describe gives 8191 for 1 channel). On ESP32 / ESP32-P4 the segment is the ring
    itself, so a pretrigger + 129 above samples is refused as unsupported (the driver's single read of 128 records runs ahead
    onto the samples of the ring's old segment).
- **capture-group event sources** (capture §4.4): this implementation accepts only event sources with 1 stage (count 1, no
  time limit), one condition (on one of the grouped tracks: a logic level or edge, an analog crossing) and rearm 0. Others are
  unsupported with the tag as received. Every grouped track is a one-shot with a linked trigger (the ESP32-P4's logic and
  analog; the RP2's PIO logic and the classic ESP32's sampler have no linked). The condition's track judges it as its own
  trigger; the other tracks (following tracks) keep capturing into their rings from start, receive the event's time, and cut
  their own pretrigger segment around the sample nearest that time (P4 logic: up to what the ring can return at that width,
  less than samples. Analog: up to max_pretrigger, on ESP32 up to samples − 129). If it cannot hold that, bind refuses with
  unavailable cause 2 (TLV fn).
  - **Delay until the event arrives**: a following track learns the event's time only after the judging track finds it and
    hands it to the group. Meanwhile the following track's samples keep entering its ring, so the ring holds the segment, that
    delay, and a margin. The upper bound of the delay is set per judging track: ESP32-P4 logic is one DMA chunk (one
    descriptor, up to 4032 bytes; 32 ms at w = 1 and 1 MHz, 51 ms at the lowest 627451 Hz) + 100 ms (an estimate of one
    loop() pass); force is known at once. Analog is one conversion frame + 100 ms. A following analog track (ESP32 /
    ESP32-P4) widens its segment buffer as a ring to samples + the delay's frames + 129 (up to 16384 frames per channel).
    RP2 uses a ring of 16384 / channel count values. P4 logic: the pretrigger and the delay must fit in its 128 KiB ring. If
    they do not fit, bind refuses with unavailable cause 2 (TLV fn). Example: logic at 1 MHz, analog at 20 kHz, delay 132 ms
    (2646 frames): analog samples up to 13609. When the event arrives later than the delay bound (loop() stalled longer than
    100 ms), the head of the segment has been overwritten, so it stops with state 6, error 2.
  - The grouped tracks and the sum of their rates have a budget of the resources the tracks share (declared nowhere; the host
    learns it by trying bind, capture §4.3).
- **Segment serial** (capture §2.2): u32, wraps around. The ESP32-P4's repeat and streaming remember the information of up to
  255 segments (one is the segment being filled), and segments answers within that range by the serial-number paging of [common](https://github.com/Open-Embedded-Probe/oep-spec/blob/main/interfaces/oep-if-common.md) §1.3.
- **Non-contiguous segments are not delivered** (capture §2.2): when the ESP32-P4 logic's capture queue (128 entries)
  overflows or the DMA ring (128 KiB) wraps over bytes not yet copied, the segment being filled is dropped (not listed in
  segments, not returned by read or streaming), and the track stops with state 6, stopped reason 2, error 2. status's
  write_pos is the start of that segment, and flags bit0 is set. Segments that ended before it stay readable. A one-shot with
  a trigger stops the same way when the ring wraps before the segment is copied (also when the trigger was before the ring's
  oldest byte) (write_pos 0). Analog is the same (the ESP32's conversion overflow, the ring wrapping over a segment on ESP32 /
  RP2). The classic ESP32's sampler does not drop samples (late samples are shown with slipped).
- **ESP32-P4 streaming**: it also sends the data of the segment being filled (capture §2.2 allows it). If something is dropped
  in that segment, the track stops in state 6 and write_pos goes back to that segment's start. The host discards what it
  received from write_pos on. The vendor bulk zero-copy path drops the frames it has not sent and stops. Repeat decides the
  free space at the start of a segment (it starts only when there is room for all of it); if there is no room it skips the
  whole segment, and the next segment gets flags bit0. Other drops (queue or ring overflow) are errors even at a boundary.
- **Entering state 6 always gives stopped reason 2** (also from state 2, also for peripheral failures; capture §3.2). In a
  capture-group, the group's stopped carries that track's error, and the other tracks stop as with the group's stop (stopped
  reason 1).
- **Type 0 triggers' role and value are not looked at** (not refused even for a role not in the plan; capture §3.3).
- **spi-target bits**: the ESP-IDF SPI slave does not count beyond the armed length × 8, so bits does not exceed length × 8
  (it does not reach 0xFFFFFFFF). read_rx does not return the ns TLV (the same for i2c-target). The bits of a partial byte that
  did not come are 0 (the SPI work register still holds the armed tx).
- **ESP32-P4 / P4X spi-target inputs**: CS always gets the pad's glitch filter (drops pulses shorter than 25 ns). SCK always
  gets the pad's input hysteresis while running, and also the glitch filter when configure's sck_filter (fixture §4) is chosen
  (describe's sck_filter_ns is 25). With the filter, only transfers where SCK's High and Low are both longer than 25 ns are
  received (up to about 16 MHz). Choose sck_filter for wiring where glitches ride on slow SCK edges (a peer that makes SCK
  with GPIO).
- **What arm's answer means** (fixture §4: arm waits for the next single transfer): arm answers success after the slave holds
  the armed transfer (the driver's interrupt loads it; known through post_setup_cb). A transfer whose CS becomes active after
  that is the armed one. The driver's interrupt is on loop()'s core, and while that core has interrupts stopped (an SWIO frame)
  it waits. If it is not loaded within 2 ms, arm is failed (nothing armed). i2c-target's preload_tx also answers after the
  buffer is loaded into the TX FIFO (if the bus is busy, it is loaded at that transfer's STOP: a read that starts after that
  answers from that buffer).
- **classic ESP32 spi-target and cs_setup_ns** (15000 ns): MISO starts being driven by the CS interrupt (software) after CS
  becomes active, so a master that starts SCK before that cannot rely on the first bit (fixture §4). With a pull-up on the line
  that bit reads as 1 (with an MHz-class master that starts SCK right after lowering CS, the first 1-2 bits are 1). The ESP32-P4
  has no time in which it does not drive MISO itself (it declares no cs_setup_ns).
- **CH32 spi-target** (ArduinoCore-CH32RV's CH32RVSPISlave): the slave counts bytes, so bits is the byte count x 8 (for a
  transfer where CS rose mid-byte, the number of bits that came is unknown). CS and each byte are interrupts, so cs_setup_ns is
  5000 ns and max_clock_hz 200 kHz (the core's self-test value). SCK / MISO / MOSI must be one SPI route; otherwise configure
  is failed. Not included on X035 (merely referencing the core's SPI slave links its interrupt handlers).
- **CH32 analog** (ArduinoCore-CH32RV's CH32RVAdcStream): the ADC free-runs without a timer, so the rate is one of 8 sample
  time steps (measured at boot on the first ADC pin; during that the pin is analog, and afterwards it goes back to a floating
  input). A rate slower than the slowest step cannot be made (tens of thousands per second on V307, about 200 thousand per
  second on H417). Values are copied from DMA in halves of 512, so if loop() does not come round within that time, a half is
  overwritten and that segment is not delivered as a hole (state 6, error 2). One segment is 16 KiB.
- **External clock** (capture §3.8): only for a one-shot with an immediate trigger (otherwise configure is unsupported with
  the clock tag). The clock role is assigned in the plan and is not among the sample lines. The answer's actual_rate is 0 / 1.
  The segment's start_ns is the time of start, and its uncertainty is the time until the segment is complete (when the first
  edge came is unknown; the classic ESP32 measures the first edge's time).
  - ESP32-P4 / P4X: PARLIO's external clock input. Rising or falling (not both). The qualifier is the level delimiter's
    valid line (up to 15 data lines). max_hz is declared as 40 MHz. stop keeps the partial segment.
  - RP2040 / RP2350: a small program in the other PIO waits for edges and samples. Rising, falling, both (both without a
    qualifier). max_hz is declared as 10 MHz. stop keeps what the DMA wrote as a short segment.
  - classic ESP32: the sampler's window (interrupts stopped, up to 200 ms) keeps watching the clock line and samples on edges.
    Up to 7 data lines (the clock is seen in bit 7 of the sample). Format 3's delta is in CPU cycles (the timebase is the CPU
    clock). Edges that did not come within the window give a short segment. max_hz is declared as 1 MHz.
- **Glitch filter** (ESP32-P4 / P4X, capture §3.8): the GPIO pin filter is applied per data line. The width is 25 ns only (2
  cycles of PLL_F80M; shorter pulses are dropped, exactly 25 ns passes). The flex filter (selectable window) is not used:
  on slow, noisy edges (I2C with a 45 kΩ pull-up), whatever the clock source, its output stops following the input or
  oscillates, and data is lost silently. The filter belongs to the pad, so while applied it also affects other fns reading
  the same pin.
- **Input hysteresis** (ESP32-P4 / P4X, capture §3.8): when configure says 1, the data lines' pad hysteresis is turned on
  (off by default). It reduces how often wobble on slow edges shows up as short pulses (on 45 kΩ I2C, SCL edges against an
  ideal 1000: 1108-1142 without, 1008-1014 with), and keeps the trigger from misfiring on wobble. In exchange, real pulses
  that cross the threshold only for a moment no longer show (on the same line, SCL highs cut to 20-40 ns by the controller
  that were not captured rose from 2-4 to 6-8 out of 450). Edge positions are delayed by the threshold (about 50 ns). Clean
  edges up to 10 MHz change neither in width nor in count. Removing the plan turns it off.
- **Analog format 2** (capture §1.4): one-shot only (asked for with streaming, configure is unsupported with the format tag).
  The segment is kept as the captured frames, and read builds the blocks of the requested range on the spot. The block length
  L is up to 256 (D up to 256 too).
- **I2C / SPI controller**: the lines are driven with GPIO, and time is counted in CPU cycles on ESP32 and with micros()
  elsewhere (1 µs steps, up to 250 kHz). Interrupts only lengthen half periods and CS times, never shorten them. An edge's time
  is taken at the moment the edge is written (an ESP32 GPIO write returns after about 300 ns, so counting from the return would
  lengthen each half period by that much). The timing part is placed in internal RAM on ESP32.
  - SPI SCK upper limit: ESP32-P4 750 kHz (GPIO read and write each take about 300 ns, and a half period holds a write and a
    read; asking for 1 MHz gives 768 kHz). The classic ESP32 declares 2 MHz.
  - I2C SCL high is counted from the moment SCL is released and read high (the same as UM10204's controller clock
    synchronization; the time the read takes is subtracted). On lines with slow rises the period gets longer: with only the
    P4X's internal pull-up, asking for 400 kHz gives about 307 kHz, and 100 kHz gives about 93 kHz. The internal pull-up is
    weak and SDA takes 1 µs or more to rise, so use an external pull-up at 400 kHz (Fast mode).

## 2. Boot and update

### 2.1 Boot guard (not left stalled)

- RP2040 / RP2350: the hardware watchdog (3000 ms) is fed by a timer (every 250 ms) only while loop() is running. If loop()
  does not come round for max_op_ms + 5000 ms (15000 ms), it resets. On panic and HardFault, the device is taken off the bus
  and then the chip is reset.
- ESP32 / ESP32-P4: panic and the interrupt watchdog (300 ms) reset the chip (the core's sdkconfig). loop() runs under the task
  watchdog (15000 ms).
- **Safe boot**: a boot ended by the firmware itself crashing (RP2: a watchdog reset marked by `BootGuard::crashed()`. ESP:
  panic, interrupt watchdog, task watchdog, CPU lockup) counts 1 if it comes within 30 seconds of the previous boot. After 3 in
  a row, that one boot is a safe boot: the saved settings' pass-through (console §2.1) attach and the console on it are not
  started. A boot that ran for 30 seconds, a restart the firmware did itself, and other resets (power on, the reset pin, an
  external reset) set the count back to 0. On RP2 every restart is a watchdog reset, and those without the mark (picotool, boot
  ROM, debugger) are treated as external restarts. An RP2 that stalls with interrupts off (the feeding timer stops too, and the
  watchdog fires before the mark is set) cannot be told apart from an external restart (not counted, not marked).
- **USB gate**: a probe whose transport is its own USB device starts the saved pass-through's attach after the host has kept
  the device configured for 1000 ms, or, if there is no host, 5000 ms after boot.

### 2.2 How the previous boot ended (the firmware string, this implementation's behavior)

fn 0's describe firmware appends, after the version, what ended the previous boot in parentheses: `<version> (<note>)`. The
note is `<reset> at <n> s` or `update to <slot> did not reach setup: <reset>` (the updated image did not reach setup()).
`<reset> at <n> s` is only for resets the firmware did not intend (crashed, stalled) and brownout: on ESP panic, task-wdt,
int-wdt, cpu-lockup, brownout; on RP2 crash (marked by `BootGuard::crashed()`: panic, HardFault, loop() stalled). After power
on, a restart the probe did itself, or an external reset, nothing is appended. External resets: the reset pin (including
esptool's DTR / RTS), the USB-Serial/JTAG reset, a debugger's JTAG reset, a software restart the probe did not do, an RTC
watchdog reset without panic (esptool's watchdog reset), an RP2 watchdog reset without the mark (restarts such as
`picotool load -x`, boot ROM, debugger).

- Tools that compare the version exactly strip everything from the first space on before comparing.
- describe is declarations only (core §7.3), and the specification does not define this mark. This implementation leaves the
  mark in the firmware string (implementation behavior, not a specification rule). The host treats firmware as a free string
  and does not rely on the mark.

### 2.3 An image updated by DFU (ESP32-P4)

An image written by DFU is confirmed when it boots (there is no trial period). arduino-esp32's initArduino marks the image
valid before setup() (esp_ota_mark_app_valid_cancel_rollback), so later panics, watchdogs, brownouts or re-enumeration by the
host never go back to the previous image. The bootloader can still go back only when the new image fails its own check, or
when a reset comes before the core confirms it; then §2.2's `update to <slot> did not reach setup: <reset>` is appended. A
broken image is recovered with the ESP32-P4's USB-Serial/JTAG and esptool. The classic ESP32 and RP2040 / RP2350 have no
update from the running firmware.

### 2.4 restart (`oep.probe.restart`)

After the restart answer, the probe restarts once it has finished sending the answer (ESP: esp_restart. RP2: a watchdog reset,
60 ms later). The target is not reset, and a halted hart stays halted. restart_max_ms is in §1.1.

### 2.5 Saving

The settings are saved in the file `/s` on the probe's LittleFS (ESP32 / ESP32-P4: the partition `spiffs`; RP2040 / RP2350:
the FS area). Everything is written to `/s.w` and then renamed to `/s`, so after a power loss either the previous save or the
new one remains (core §13.7's recommendation). When the FS area cannot be mounted after 3 tries, the area is formatted. The
save then counts as absent (storage_state 0). A normal firmware write (arduino-cli upload) does not erase the FS area.

### 2.6 Memo area (`oep.probe.memo`)

- max_object is 2048 bytes. max_objects is the number of FS blocks minus 12, capped at 64 (52 on the RP2040's 256 KB, 64 on
  RP2350 and ESP32). The 12 reserved blocks are for FS management, the file being written, and the settings file and its copy.
  Settings can be saved even when the memo area is full.
- An object is the file `/m/<CRC-32 of the name>.<n>` (up to 8 names with the same CRC-32). write keeps the temporary file
  `/m/w` open while writing, and at the last write checks the CRC-32 and then renames it into place. After a power loss either
  the previous object or the new one remains (oep-if-memo §3 allows weaker than this). Until the write finishes, the previous
  object stays readable.
- A build that cannot mount an FS (such as an RP2 build without an FS area) does not offer the memo area. CH32 has no memo area.

## 3. Per wire PHY

### 3.1 RVSWD / SWIO

- **Re-establishing**: before the first frame after the wire has rested 300 µs or more, DMSTATUS is read. If that fails or is
  not a "found" with authenticated set, the setup pair (DMI 0x7E, 0x5AA50400 to 0x7D) is sent twice (no wake pattern), and
  DMSTATUS is read again. A wake pattern is sent to a wire that still does not come back only when there is no connection, and
  inside attach and the reset op (§1.2's "retries that do not change the target's state").
- **After dmactive**: right after writing dmactive, the setup pair is written twice more at the same speed.
- **Acceptance**: a speed is accepted when 1000 consecutive DMSTATUS reads give the same value and parity, and writes are
  accepted when 256 round trips of writing a scratch register (PROGBUF0) and reading it back match. The speed search starts
  with a read at the slowest speed and moves to faster ones.
- **Long form (85 clocks)**: attach first reads DMCONTROL in the long form (CH32V103 answers only the long form). With a peer
  that answered, the long form is used for the whole connection and the short form is never sent. With no answer it moves to
  the short form.
- **Peers with a slow clock**: CH32V103 runs on the 8 MHz HSI when its flash is empty or right after reset (before the program
  raises the clock), and answers about 1.2 µs after the clock edge. At the slowest half period of the speed table (500 ns) the
  answer overlaps the sampling edge, so the long-form query and the speed re-selection after reset also try 1000 ns and
  2000 ns when there is no answer at 500 ns. The period found becomes the slowest period of that connection (the next attach or
  re-selection starts again from the table's periods). Writes during that are slow (about 12 seconds for 6.4 KB).

### 3.2 SWIO (classic ESP32)

- Bit times are counted in CPU cycles: 1 is low for 262.5 ns, 0 is low for 862.5 ns, high is 262.5 ns. The speed set by one 0 bit's time
  (its low and high, 1125 ns) is declared as min_clock_hz (the wire runs at that speed only).
- After stopping driving low and after reading a 0, the line is driven high with a charging pulse of 30 ns or less.
- Before attach's first frame, the line is left on the pull-up only for 2 ms; if it reads low, there is taken to be no target.
- Interrupts are stopped during a frame (at most 200 µs).

### 3.3 SWD (RP2040 / RP2350)

- The speed cannot be chosen by reading, so it starts from `min(max_speed, describe's max_clock_hz)`. The slowest speed is
  10 kHz (a scan without max_speed also tries with it).
- Wire retries redo the line reset and the wake from dormant.

## 4. Per platform

### 4.1 classic ESP32

- **Core assignment**: loop() (all of OEP, the SWIO wire, the interrupts the sketch attaches: UART0, fixture UART, SPI / I2C
  devices, ADC) and Arduino's events are on core 0, the same core as the Wi-Fi driver and the TCP/IP stack (pinned to core 0
  by the core's build). Core 1 is used only by the logic sampler (`examples/Firmware/OepProbe`'s profile esp32 is
  `LoopCore=0,EventsCore=0`; the Wi-Fi build refuses other assignments with `#error`). The sampler runs on a core different
  from the one that called start (loop()'s core). loop() rests 1 tick every 50 ms and lets core 0's idle task (watched by the
  task watchdog) run.
- **Capture windows and the debug wire**: during a window (immediate: up to 200 ms; trigger search: one slice up to 250 ms),
  the logic sampler stops interrupts on core 1 and keeps reading GPIO. SWIO frames, which use the same peripheral bus, can
  shift during that (SWIO has no parity and DMI writes are not read back, so it cannot be seen on the wire).
- **Rule: an immediate capture window and SWIO wire traffic (requests and console alike) do not happen at the same time.**
  While an immediate (trigger type 0) window is sampling, not a single SWIO frame goes out: requests (riscv-dm, wire ops) wait
  for the window to end, and the console reads neither DMSTATUS nor DATA0 (it is refused its turn on the wire and reads at
  the poll after the window closes). The window opens after the request in progress ends. The immediate window's length is
  samples ÷ the actual rate, and configure rounds samples down to 200 ms worth (actual_samples; 60000 at 300 kHz, 20000 at
  100 kHz), so the window is at most 200 ms. So **console commands or debug resets sent after the capture starts do not show
  in an immediate capture** (they reach the target after the window closes).
- **Trigger search**: within a slice (one up to 250 ms) the wire is likewise not passed, but within it the wire is given
  turns: if a request is waiting for the wire, at the next sample (as many times as needed per slice); if a console read was
  refused, once per slice, up to 5 ms (`kWireTurnMs`). During a turn the sampler keeps reading, and frames go through
  alternating with it one at a time (it does not read during a frame and catches up afterwards: that sample is late, and if
  it lands in the segment it is slipped). The console ends its turn early once it has handed what it sends to the target (the
  target then acts where there is no wire traffic). In the gap between slices (about 1 ms, interrupts restored and core 1's
  tasks run) the wire is not handed out (if a command sent there made the target act, nobody would be sampling). The rest of
  the segment after the trigger fires is exclusive (a request that started within a turn continues frame by frame). A host
  that wants to make the target do something after arm and capture it uses the trigger: send a command or reset after arm, it
  reaches the target within a turn, the event it causes fires the trigger, and it shows in the capture. The trigger is checked
  from the slice's first sample (edges from the second): when the pre-trigger part has not filled yet, the segment is short and
  trigger_index is small (capture §3.3). What is missed is events in the gaps and pulses that start and end within a frame
  (one is about 60-150 µs).
- **Background wire use during the search**: within the search only host requests (within 64 samples if waiting for the wire)
  and console reads (once per slice) take turns. Wire work the probe does by itself with no waiting request (the saved
  pass-through's attach and its retries, the liveness DMSTATUS read (every retry_ms), redoing the console's attach) does not
  take the wire or ask for a turn while a sampler window (immediate window and trigger search) is open. It is done at the
  first poll after the window closes. While waiting long for a trigger it is delayed by that much (the specification does not
  set the pass-through's liveness interval). Otherwise the sampler could not read during the background wire frames, and the
  segment would be slipped.
- **Frame length within a turn**: for an SWIO frame, the sampler stops core 0's interrupts within a few instructions of
  receiving the signal and restores them after the end-of-frame signal (frameEnd). The sampler waits only for the frame itself
  (read about 60 µs, write about 46 µs), not for interrupts or tasks that came in between (the Wi-Fi driver has a higher
  priority than loop()).
- **Declaration (slipped in triggered capture)**: when something uses the SWIO wire within the trigger search (a host request
  that took a turn, a console read at the start of a slice), the samples during that frame are late, and if they land in the
  segment, slipped is set. Independent of whether the radio is on. The segment's data is correct, and the delay is shown with
  slipped (it does not claim no slip). With nothing using the wire, slipped does not appear within rate_range, as with
  immediate.
- slipped is shown only when really late samples (waits for interrupts or the peripheral bus, frames within a turn) land in
  the segment. No new frame enters the immediate window or the rest of the segment after the trigger.
- Stopping console reads during the window does not break the specification's rule (console §3) (it does not set the read
  interval). On the target side, SDI / DMDATA wait for the write during that time, and a dmseq target comes back within its long
  wait (1 s or more).
- **The length of stopping interrupts is bounded by the clock**: one interrupts-off stretch (the immediate window, a search
  slice, the rest after the trigger) is bounded to at most 250 ms (`kOffNs`; the interrupt watchdog is 300 ms) by the clock
  (the cycle counter), not by the sample count. At fast rates late samples cannot be caught up, so frames within a turn,
  continuing frames of a request that was using the wire at the trigger, and bus waits for the other core's or the radio's DMA
  lengthen one stretch by that much. A search slice ends at "one stretch's length minus the nominal time of the segment's
  rest". If a segment (the immediate window too) reaches that length before it has finished reading, it ends there: **samples
  is less than configure's value, and slipped is set** (because samples were late). The trigger's position and trigger_index
  are correct.
- **The limits are the same with the radio on**: the sampler's window does not stop the radio's core (core 0), so there is no
  segment or stretch-length limit for the radio (a segment is at most 60000 samples, one stretch at most 250 ms, a search gap
  about 1 ms; unchanged with or without Wi-Fi). Radio traffic (the Wi-Fi driver runs on core 0 and uses the peripheral bus) can
  delay samples, and then slipped is set (it does not claim the data is correct).
- **Rate range**: rate_range is 1 kHz-300 kHz. The upper limit is a value with a margin a little below the fastest rate at
  which an immediate window and a force capture without SWIO traffic pass without slipped. samples is rounded down to 200 ms
  worth (the window is within 200 ms, and a search slice holds 50 ms or more of searching after the segment's rest). 200 ms at
  300 kHz is 60000 samples, which is the largest segment (describe's max_samples; max_pretrigger is 59999). Lower rates give
  more margin per sample. A plan including GPIO32..39 has the same range (one more register is read per sample, but it keeps up
  to 1 MHz).
- **Work per sample**: the immediate window and the trigger search do the same per sample (look at the gate's mark, compare
  the clock with the sample's time, wait, read GPIO.in and pack it with a table, store). The search adds only the trigger test
  (edge: comparison with the previous level; level: comparison). The slice's clock, the wire turn and control (force and
  abort) are looked at every 64 samples (force and turns wait up to 64 samples: 213 µs at 300 kHz). Looking every sample would
  make the search slower than immediate, through reads of the shared word.
- **The sampler task's priority is 17** (core 1 has no other heavy task. When a sketch runs the TCP/IP stack (18) or the
  Wi-Fi driver (23) on the same core as the sampler, they go first).
- The fixture UART's receive interrupt is on loop()'s core (not the sampler's), and the FIFO threshold is 32 bytes. Speeds up
  to 2000000 bps.
- i2c-target: the slave peripheral has no START interrupt, so writes are delimited only by STOP. Writes joined by repeated
  START become one frame up to STOP (fixture §3 also delimits at START).

### 4.2 ESP32-P4

- **USB device stack (TinyUSB's DWC2)**: the TinyUSB DWC2 driver that EspUsbDevice uses panics when the next SETUP arrives
  during a control transfer's status stage (an upstream defect). On the P4 a panic becomes a chip reset, so to the host the
  device drops off the bus, enumerates again, and boot_id seems to change (the firmware string has `panic at <n> s`, §2.2).
  The session, resources and unsaved settings are lost. The host learns it from the boot_id change and reopens.
- The fixture UART's receive interrupt is on core 0 (loop() is on core 1), the FIFO threshold is 32 bytes, and the driver's
  receive buffer is 16 KiB. Up to 2000000 bps.
- i2c-target: ESP-IDF's slave driver (v1) does not report the number of bytes received, and its sending is a stream without
  boundaries, so this implementation takes the peripheral's interrupts itself, counts a transaction's bytes, and fills the TX
  FIFO from the buffer (0xFF when empty). This implementation absorbs the chip's FIFO quirks (such as sending an extra byte at
  the end of a read). stretch is up to 100000 µs (max_stretch_us).
  Only the P4 has nack_from (features bit1). It holds SCL before the ACK of a received byte and decides ACK or NACK in that
  interrupt, so while nack_from is in use SCL is lengthened per byte by the interrupt latency (up to about 50 µs). The address
  byte is always ACKed.
  Writes are delimited by the STOP and START (including repeated START) interrupts. If the START interrupt is delayed beyond the
  arrival of the next transaction's first data byte (18 clocks including the address), that byte goes into the previous frame.

### 4.3 RP2040 / RP2350

- fixture UART: PL011 overrun is turned into the lost (overflow) mark by reading UARTRSR.OE in poll(). The FIFO headroom is 28
  bytes (140 µs at 2000000 bps).
- Saving is a LittleFS file (§2.5). While writing, it does not answer other requests (within max_op_ms).

## 5. The host and bridge side (for users of this implementation)

- **USB-UART bridges** (a UART bridge port; the CH340 kind): may drop probe → host bytes in long continuous flows (writes
  followed by max_frame-sized answers, capture reads). Short checks pass, but long transfers break. With the bridges measured
  (CH340, a CH552 claiming FTDI compatibility), 1-4 max_frame-sized answers broke on every write of about 9 KiB at 921600 bps,
  and 500000 bps broke on neither. The practical speed is **500000 bps**. A CH340 can drop 0-10 % for seconds at a time even at
  115200. There is no flow control, so the host resends broken frames with the same corr (the probe's resend table prevents
  double execution).
- **Linux cdc_acm**: when a probe → host burst exceeds about 8 KiB, it loses data silently with no error. oep-client-python
  keeps the amount of answers waited for at once to 6 KiB and the subscription's min_bytes to 2 KiB.

## 6. TCP and Wi-Fi (classic ESP32)

OEP is carried over TCP (oep-spec transports §1, §2, §3). The probe chooses the listening port (this implementation's value is
written here). Discovery (DNS-SD `_oep._tcp`, TXT `unit_id`) is defined by transports §3, and the Wi-Fi items by transports §7.
Whether a TCP probe announces itself is the probe's choice (transports §3). This implementation announces itself.
Use TCP only on a trusted local network or inside an authenticated tunnel (OEP has no authentication; transports §1,
security §1).

### 6.1 Transport

- **The port is 7450** (`kTcpPort` in `src/firmware/Esp32.h`, `kPort` in `03.Transports/WifiTcp`). One listening socket is one
  transport in fn 0's describe (kind 6, interface 0xFF). In the classic ESP32 firmware the UART bridge is 0 and TCP is 1. The
  listener is added after all other transports (`Endpoint::addTcpListener`; transports added later are refused, because the
  serial ports' index stays the describe index).
- **3 simultaneous connections** (`TcpListener<3>`). Each connection is a separate transport, and confirm's transport TLV
  returns 1. max_frame / window / max_inflight are the same per connection (classic ESP32: 512 / 1024 / 2). A fourth
  connection is accepted and closed at once.
- Each connection has a 1 KiB receive buffer and a 6 KiB send buffer. The answers of one poll are sent together with flush (the
  length and the body go out in one segment). The send buffer is big enough for everything a host that respects window can
  make that connection hold (max_inflight answers, the 1024-byte push queue plus one push, the event queue) (the firmware's
  static_assert): for such a host the probe does not wait and does not close the connection however long the network stalls
  (seconds with Wi-Fi retransmissions). Only for a host that sends beyond window, when the buffer is full and the socket does
  not take anything for 2000 ms (`TcpSlot::kWriteWaitMs`), the connection is closed as dead (the bound on a host that stopped
  reading stalling the probe). TCP keepalive: after 10 s of silence, 3 probes every 5 s, closed if there is no answer.
- On reading a length above max_frame, the connection is closed (transports §1). A pause in the middle of a frame does not
  cause a re-read (transports §2).
- When a connection closes, the session, lock, subscriptions and resend table remain (transports §3). Answers and
  notifications meant for the closed connection are dropped. The next connection in the same slot does not receive the
  previous connection's notifications (a partly read frame is dropped too). An open with the same session id from another
  connection restarts the lease, and notifications move to that connection (core §6.2).
- restart (`oep.probe.restart`): after sending the answer, it closes the listener (so the probe before restarting accepts no
  new connection), hands the send buffer to the socket, waits 100 ms, and restarts. **restart_max_ms is 15000 in the Wi-Fi
  build**: booting, connecting and getting an address take a few seconds on ATOM from the answer until a new connection's
  confirm answers; twice that. When entries earlier in the settings do not connect, each one adds up to 15 s (§6.3).

### 6.2 Discovery (mDNS, transports §3)

- Host name `oep-<unit_id>.local`, service `_oep._tcp` (port 7450), instance name `OEP <unit_id>`, TXT `unit_id=<unit_id>`.
  The host checks a probe named by unit_id with describe's unit_id, as on other transports.
- The address is also available from another transport (the serial port) through fn 0's network (transports §7.3).

### 6.3 Wi-Fi settings (the Wi-Fi items of the core's settings, transports §7)

No credentials go into the build. Which network to join is a probe setting, set / saved / unset like other items. The items'
form and rules are in oep-spec's transports §7. This implementation's values and behavior are as follows.

| What | Value |
|---|---|
| `wifi.{}.ssid`, `wifi.{}.passphrase` (secret) | the key is the entry (index, 0-3; the setting's keys is 4) |
| `ipv4.{}.mode` / `address` / `prefix` / `gateway` / `dns` | the key is the netif (1 = `wifi`, this implementation's only port) |
| network (fn 0's op 0x25) | tcp_port and the state per netif (state, entry, reason, rssi, address). transports §7.3 |

- ssid is 1-32 bytes. passphrase is 8-63 bytes of 0x20-0x7E or 64 hex characters (anything else is malformed). With no value,
  an open network.
- **passphrase is secret**: get returns no data (len 0). A set with len 0 keeps the current value (malformed if there is no
  value). It does not appear in network either.
- It is saved in the same LittleFS file as the other settings (not encrypted). Anyone who can read the flash can read it.
- Behavior (the times are this implementation's choice; the specification leaves intervals to the probe, transports §7): 300 ms
  (`kApplyDelayMs`) after a set or the apply at boot (the set's answer goes out first), it tries every entry in index order (it
  does not skip by scan: a hidden SSID does not appear in a scan). Each takes up to 15 s (`kTryMs`; it moves to the next when the
  driver reports that the network is absent or authentication failed). The first one that gets an IPv4 address is used. If all
  fail, it waits 5 s (`kRetryMs`) and starts over. If the connection drops after connecting, it starts over at once. A set /
  unset that changes or removes the entry in use disconnects and starts over after the answer is sent (300 ms later). A set
  that does not change the entry in use does not disconnect (the new list is used at the next start-over). A change while not
  connected (trying, waiting) starts over 300 ms later. With 0 entries the radio is stopped. Modem sleep is off (so requests do
  not wait for the next beacon). The Wi-Fi driver's own storage is not used.
- The failure reason is the driver's reason rounded. Neither passphrase nor SSID goes into logs (the UART does not log on an OEP
  transport, probe guide §3).

### 6.4 Measured values (M5Stack ATOM, a home AP; rough values)

- clock round trip: median about 10 ms, sometimes tens of ms.
- `oep linktest`: tens to 100 KB/s or so. No corruption, no loss. Faster than UART at 115200 (about 11 KB/s).
- With 2 in flight and max_frame-sized answers in a row, answers sometimes stall for 1-2 s (at most 4 s) before the rest comes
  (Wi-Fi retransmissions; the same directly from Windows). Answers are not lost. The host's TCP wait (oep-client-python: 15 s)
  exceeds this.
- Two connections: one opens, the other's open is locked, and lock_state shows the owner. When a connection closes the lock
  remains, and an open with the same id from another connection continues. The serial port answers at the same time too.
- From boot to connected: about 4-7 s (scan, connect, DHCP).

### 6.5 Limits

- **The sampler window**: the logic sampler stops that core's interrupts on core 1 during the window. The Wi-Fi and lwIP
  tasks are on core 0, so TCP does not stop during the window (§4.1).
- **Console reply delay**: Wi-Fi network stalls (1-4 s from retransmissions, §6.4) and UART0 loss while the radio is on (below)
  delay or drop console replies (console notifications) by the same amount. The host waits for the console longer than that
  (over 4 s), and re-reads gaps in the notification seq with read (console read does not consume).
- **UART0 loss while the radio is on (classic ESP32)**: when connected to Wi-Fi, whole-frame losses in UART0's probe → host flow
  (500000 bps) become several times those of a build without Wi-Fi (both below a few %). When using the serial port together
  with Wi-Fi, assume recovery through the host's resends.
- A build without Wi-Fi (`begin<oep::kDefaults & ~oep::kWifi>()`) is much smaller in size and RAM and has no wifi items (they do
  not appear in describe's items).
- The **ESP32-P4** has no radio. arduino-esp32 has the ESP-Hosted route that uses another chip (such as an ESP32-C6) over SDIO,
  but this firmware does not use it (`OepWifi.h` is only for chips with SOC_WIFI_SUPPORTED).
- **RP2040 / RP2350** (including the Pro Micro RP2350): no Wi-Fi. The CYW43 of the Pico W / Pico 2 W is not used. WCH CH32 has
  no Wi-Fi either.
- IPv4 only. No TLS and no authentication (outside OEP; if needed, use it inside a tunnel).
