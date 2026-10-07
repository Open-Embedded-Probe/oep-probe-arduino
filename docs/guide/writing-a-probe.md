# Writing a probe

[日本語](writing-a-probe.ja.md)

How the library is put together, and how to build your own probe or your own interface with it. Read it with the
examples open: `01.Basics/MinimalProbe` is the frame, `02.Interfaces/CustomInterface` the interface, and
`Firmware/OepProbe` everything together.

## 1. The shape of a probe

```text
 host  ==frames==>  transport (Stream) --> Endpoint --> Interface (fn 1)   oep.wire.rvswd
                    transport 2 .......        |    --> Interface (fn 2)   oep.target.riscv-dm
                                                |    --> Interface (fn 3)   io.github.you.thing
                                                |    --> oep.probe.plan, oep.probe.restart (the endpoint's own, listed last)
                                        fn 0 (the core, no name): confirm, list, describe, clock, the lock
```

- The **Endpoint** owns the protocol: it reads frames from each transport, checks the session and the lock, finds the
  interface by fn, and writes the result back on the transport the request came from. fn 0 (the core, which has no name
  and is not listed) is built in, and so are `oep.probe.plan` (listed when an interface has plan roles) and
  `oep.probe.restart` (listed with `setRestart`): the endpoint numbers them after every interface you add, at the first
  `poll()`, so the same firmware gives the same fns at every boot.
- An **interface** is a class with a name and operations. The endpoint numbers them in the order you `add()` them
  (fn 1, 2, ...); hosts find them by name, never by number. An interface must offer at least one op.
- A sketch is the wiring of these: the objects, `setup()` adding them, `loop()` calling `poll()`.

## 2. The endpoint

```cpp
static uint8_t rx[1100], tx[1024];
static oep::Endpoint endpoint(Serial, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, oep::Endpoint::kUsbCdc);
```

- The **transport** is any `Stream`. Its **kind** decides the framing: a serial port (`kUsbCdc`, `kUsbSerialJtag`,
  `kUartBridge`) carries `0x00 <COBS> 0x00` frames with a CRC, and may carry raw bytes between them (a bind, §7); a
  message transport (`kVendorBulk`, `kHid`, `kTcp`) carries length-prefixed messages. More transports:
  `endpoint.addTransport(stream, rx, sizeof rx, kind, usb_interface)` (`03.Transports/MultipleTransports`).
- **TCP**: a listening socket is one transport entry (kind 6) whose connections are each a transport of their own:
  `endpoint.addTcpListener(tcp.slots(), rx, sizeof rx[0], n)` after every other transport, with `oep::TcpListener<n>`
  (`OepTcp.h`, lwIP sockets) polled from `loop()` before the endpoint. A connection's answers and notifications stay on
  it, a length over max_frame closes it, and a closed connection ends nothing (transports §1-§3). `OepWifi.h` joins the
  networks of the settings' wifi item and announces `_oep._tcp` by mDNS (`03.Transports/WifiTcp`).
- **Limits** `{max_frame, window, max_inflight}` are what confirm promises the host. A serial port's rx buffer holds an
  encoded frame: a little more than max_frame (`cobsFrameMax`).
- fn 0's **describe** is yours to fill: `describeCore(w, model, unit_id, ...)` writes the firmware version, the model,
  a unit id that is the same on every transport (`platformUnitId`) and the channel count (the chip, with
  `describeChip`, after it). Hand it over with `setProbeDescription`. The transports and `max_op_ms` are added by the
  endpoint.
  describe is declarations only (core §7.3): nothing that changes while the probe runs goes in it.
  unit_id is mandatory (core §7.5): on a chip the library has no unique number for, the build stops until you give
  `-DOEP_UNIT_ID='"..."'` (1 to 16 of `a-z 0-9 -`, a different value per unit).
- The boot_id (core §6.5): the endpoint picks it itself when the first message arrives - a hardware random source where
  the platform has one, else the timer's count at that moment. A sketch with a better source (a boot counter it keeps in
  non-volatile storage) passes it with `setBootId` in `setup()`.
- At boot, before the first `poll()`, put every channel you do not reserve in its free state - Hi-Z, no pull
  (`platformParkMask`; core §8), or the idle of the saved settings when the probe has them.
- `poll()` from `loop()`: it never blocks long. Neither may your interfaces.

## 3. An interface

```cpp
class Blink final : public oep::Interface {
 public:
  const char *name() const override { return "io.github.you.blink"; }   // yours: reverse DNS, never oep.*
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }                      // the fixed parts' shape
  bool lockFree(uint8_t op) const override { return op == 0x03; }      // reads that change nothing
  bool offers(uint8_t op) const override { return op >= 0x01 && op <= 0x03; }   // the ops it has (the ops tag)
  size_t describe(uint8_t *out, size_t capacity) override;             // what a host reads before using it
  oep::Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
};
```

- **The name** is how hosts find it. Standard interfaces are `oep.*` (specified in oep-spec); yours take a reverse-DNS
  name of something you own (`io.github.<you>.<name>`). Nobody has to approve it, and a host that does not know it leaves
  it alone - this is how OEP is extended.
- **The revision** fixes the shape of every fixed form (core §2.3, §2.7): the fixed parts of every payload, every TLV's
  value, every element of a sequence. A fixed form never grows at its end; add a new TLV, an optional op or a new
  value, and change the revision only when a fixed form changes.
- **offers** says which ops it has (core §1.2): every required op of its table and the optional ones this probe has.
  The endpoint writes the describe's `ops` tag (0x09, base + bitmap) from it, first in every fn's describe, and answers
  any other op `unknown_operation` before the session is looked at. The default offers nothing: an interface that does
  not override it answers nothing. Optional ops are declared there and never by `features`.
- **handle** gets one request and writes the result's payload to `out`:
  - `completed(n)` - it ran and succeeded, n bytes of payload;
  - `failed(n)` / `partial(n)` - it ran and did not work (all / part), with the success-shaped payload;
  - `rejected(reason)` - not run: `kRejectMalformed`, `kRejectUnavailable`, `kRejectUnsupported`, `kRejectUnknownOperation`.
- **TLVs** are `tag(u8) len(u16) value`, one form whatever the length (core §2.2; `TlvWriter`, `tlvAt`,
  `kTlvHeader`). Sequences are `count, count x element` with no element length.
- **TLV tails**: any request may end with TLVs. `plainTail(tail, payload, length, fixed, out, capacity)` parses what
  follows the fixed part: an unknown critical TLV refuses the request (`refused()` tells), an unknown other one is
  ignored and nothing in the answer says so (core §2.3). With the tags you implement: `tail.parse(...)`,
  `tail.find(tag, len, &critical)` (the first one when a tag repeats), `tail.fixed(tag, size, value, ...)` for a TLV
  of one fixed size (any other length: malformed, critical or not), and `Tail::refuse(tag, critical, ...)` for a value
  you do not handle - unsupported with the tag as received, critical or not: a TLV you implement is never ignored.
- **The lock** is the endpoint's: every request carries a session_id in its header (0 = none); an op not `lockFree`
  only runs for the session holding the lock (session_id 0: `session_required`). `sessionOver()` is called whenever
  that session's lock ends - end, its lease running out, another host's force, all alike (core §6.4, §9): drop
  everything it created or shared (a wire's use of its connection, a console's share of its stream). Nothing passes to
  the next session; what the settings keep (a slot's connection) stays. An interface whose resources sit on another
  interface's (a console stream on a wire's connection) says `sessionOverFirst()`, so its share goes first.
- **describe** uses `TlvWriter`: common tags (`roleChannels`, `u32(kTagMaxClockHz, ...)`, `u32(kTagFeatures, ...)` for
  optional functions that are not ops) and your own (0x40 and up). Do not write the `ops` tag: the endpoint does.
- **oep.probe.link** (the link test, and port_speed on a UART bridge) is an optional interface: `oep::Link link(endpoint);
  endpoint.add(link);` - add it last so the fns before it keep their numbers. `endpoint.setPortSpeed(...)` puts
  port_speed in its ops.
- **oep.probe.restart** (optional, oep-if-restart): `endpoint.setRestart(oep::platformRestart, max_ms)` in `setup()`,
  before the first `poll()`, lists it (restart 0x01, `restart_max_ms` in its describe). `max_ms` is the longest from the answer until the probe answers confirm again
  on the same transport - the boot and a USB re-enumeration included: estimate it for your board with a margin. The
  endpoint answers first, flushes, ends the session, calls every interface's `probeRestart()` (let go of what the
  settings keep - a slot's connection - without touching the target), releases every plan, then calls the handler,
  which does not return (`esp_restart`, `rp2040.reboot()`). Wrap it to detach a USB device of your own first.

## 4. Pins: the table and the plan

- `oep::PinTable pins(mask)` is the set of channels (GPIO numbers) the sketch hands to interfaces. Each interface
  `claim()`s what it uses under its own owner id and `release()`s it; a claimed channel is refused to anyone else, so two
  interfaces never drive one pin (core §8.1). A released channel goes to its idle state (Hi-Z, or what the settings say).
- The **plan** is how a host assigns pins at run time (`oep.probe.plan`, oep-if-plan): `plan_apply` names (fn, role,
  channel); a fn whose interface has no plan role (`planRoles()` false) is refused unsupported; the endpoint asks
  each interface `planCheck()` (no side effects: 0 or a reject reason), then `planApply()`, and `planRelease()` gives them
  back. Declare the roles and their candidate pins in describe (`roleChannels`). planCheck refuses a role or a channel
  it does not declare with `kRejectUnsupported` (the endpoint adds the tag 0x90) and a declared channel something else
  holds with `kRejectUnavailable`. planApply changes nothing on the pin (an output idle keeps driving); the interface's
  first operation does. A debug wire's pins go to their free state whenever no connection holds them
  (`DmiPhy::free`).
- Owner ids must differ between interfaces sharing a table (the Firmware sketches list theirs).

## 5. The standard interfaces in this library

| Class | Interface | Notes |
|---|---|---|
| `WireRvswd` + `DebugPort` + `Ch32Dm` + `RvswdPhy` | `oep.wire.rvswd` | RVSWD on the RP2's SIO or an ESP32's dedicated GPIO |
| `WireRvswd` (named `oep.wire.swio`) + `SwioPhy` | `oep.wire.swio` | the one-wire link, classic ESP32 |
| `TargetRiscvDm` | `oep.target.riscv-dm` | halt / resume / step / reset, block read / write, run; puts the target's registers and DATA0/1 back before it runs |
| `TargetConsoleStream` + `DmConsole` | `oep.target.console` | the target's console through the debug module (dmseq, DMDATA, SDI) |
| `WireSwd` + `SwdPort`, `TargetArmAdi` | `oep.wire.swd`, `oep.target.arm-adi` | ARM SWD, RP2040 / RP2350 |
| `FixtureGpio`, `FixtureUart` | `oep.fixture.gpio` / `uart` | on planned pins |
| `LogicCapture` / `SamplerCapture` | `oep.fixture.logic` | ESP32-P4 PARLIO / classic ESP32 GPIO sampler; level / edge triggers |
| `AnalogCapture` | `oep.fixture.analog` | ADC channels in turn, one-shot, raw values, a threshold trigger (ESP32 continuous mode, RP2 FIFO + DMA); frontends, reference, factory calibration |
| `CaptureGroup` (+ `GroupTrack`) | `oep.fixture.capture-group` | tracks started together; a capture that implements `GroupTrack` can be bound |
| `P4I2cTarget`, `P4SpiTarget` | `oep.fixture.i2c-target` / `spi-target` | on the ESP-IDF I2C / SPI slaves |
| `ProbeConfig` + `Binds` | `oep.probe.config` | settings saved in flash, applied at boot (§7) |

Each source file starts with the spec sections it follows.

## 6. Debug wires: a fixed pair or host-chosen pins

- A fixed pair: `DebugPort port{dm, swdio, swclk}` and `phy.begin(swdio, swclk)` (`04.Debug/*`).
- Host-chosen pins: `port.pin_choice = mask; port.pins = &pins;` with the pair unset. The wire declares the channels as
  role_channels, takes any free pair a host names in scan / attach, moves the PHY there (`usePins`), and a live connection
  holds its pins in the table. `reset_allowed` is the channels attach's reset TLV may pull - always named by the host.
- The line's settings (how it rests, how fast it may go) are the target's: the host passes them at attach; the probe
  keeps no per-chip defaults.
- **Drive the debug wire at the weakest strength the wire's timing allows.** Its sharp edges couple into the fixture
  lines next to it on the same jig: at the classic ESP32's default 20 mA the SWIO line made a 1 MHz SPI target lose or
  shift bits while the console was read (23 of 36 frames good; 72 of 72 at the weakest, wire speed unchanged,
  2026-10-02). Every PHY in this library does so (RVSWD: ESP32-P4 `GPIO_DRIVE_CAP_0`, RP2 2 mA; SWIO: classic and P4
  `GPIO_DRIVE_CAP_0`, `OEP_SWIO_DRIVE_CAP`). A new PHY or a port to another chip must set it too - the P4 SWIO port
  first left it at the default. Set it again when the wire takes its pins (the RVSWD PHY does at every attach): a
  host-chosen pair is free between connections, and a fixture gpio output or an output idle may leave another strength
  on it.
- **Fixture gpio's output strength** (oep-if-fixture §1.1) comes from `platformDriveLevels()` in `OepPlatform.h`: the
  levels, their approximate mA and the default (the pad's reset strength) - classic ESP32 / ESP32-P4 `GPIO_DRIVE_CAP_0..3`
  about 5 / 10 / 20 / 40 mA, default 2; RP2040 / RP2350 2 / 4 / 8 / 12 mA, default 4 mA; other chips none (no
  drive_levels, and set's drive TLV is an unknown tag). A port to another chip adds its levels there, or leaves none.
  The strength applies to modes 3 / 4 only (a set, an output idle); `PinTable::setPad` puts a pad back to the default
  when it leaves them, and a fixture that takes a pin for its own peripheral (UART, I2C / SPI target) calls
  `PinTable::ownStrength` after its claim, so it starts from the pad's own strength even straight from an output idle.
  A wire sets its own (the weakest) instead.

## 7. Serial ports, binds and settings

- A serial port carries OEP frames and, between them, raw bytes: what its **bind** says (a slot's console, a fixture
  UART: one stream per port). `endpoint.setRawPorts(&binds)` turns it on; while a session holds the lock the raw
  transfer on the port it uses waits, and resumes afterwards where it stopped - from the oldest byte left if the stream
  overflowed meanwhile (transports §4, probe.config §1.2).
- `ProbeConfig` keeps slots, binds, plans, labels, idle states, the fixture UARTs' settings (the uart item) and the
  disabled channels (the disable item), saves them (NVS on ESP32, the flash's last sector on RP2) and applies them at
  boot; `state` (op 0x06) tells how the slots and binds are doing, `unset` (0x05) removes items by key. Add it last
  (`add(config)`), then `addPlace(wire, console)`, `addUart(uart)`, `setPins(&pins)`, `applySaved()`
  (`06.Settings/ProbeConfig`). Call `load()` before the sketch parks its pins and leave out what the saved settings
  disable: `config.load(); pins.setDisabled(config.savedDisabled()); platformParkMask(mask & ~pins.disabledMask());`
  (a disabled channel is never touched, not even at boot). The PinTable keeps the settings' disabled channels apart
  from `forbid` (the firmware's own, permanent): no item gives a forbidden pin back.
- A probe whose transport is its own USB device holds the at-boot slots' attach back until the host has configured
  the device: `config.setAttachGate([] { return BootGuard::attachReady(tud_mounted()); })` before `applySaved()` (the
  P4: `usbDevice.ready()`). A console polling its target from boot keeps interrupts off for each frame, and the USB
  interrupts it delays while the host enumerates the device can crash the USB stack (RP2: TinyUSB 0.18's "Can't
  continue xfer on inactive ep" panic). `BootGuard` (`OepBootGuard.h`): `begin()` first in `setup()`, `poll()` in
  `loop()`; it runs the watchdogs, counts fast crash-boots, and after `kSafeAfter` of them in a row `safe()` says to
  call `config.skipBootAttach()` for that boot. On the RP2 point the SDK's `_exit` and `isr_hardfault` at
  `BootGuard::crashed()` (`Firmware/OepProbe/Rp2.h`): a panic then resets the chip instead of halting it. A firmware
  update (an ESP32 with bootloader rollback) is taken as working and confirmed as it starts: leave `verifyRollbackLater`
  undefined and the ESP32 core confirms the image before `setup()`, so no later reset goes back to the one before
  (`Firmware/OepProbe/Esp32P4.h`); the safe boot covers a crash loop the saved settings set off. `BootGuard::lastBoot()`
  says what ended the boot before when the firmware did not intend it (a crash or a hang and the seconds it was up, a brownout, an update the bootloader did not start; nothing for a reset from outside or a restart on purpose - on the RP2 `crashed()` marks its own reset, so an unmarked watchdog reboot such as picotool's is no crash);
  `Firmware/OepProbe` puts it after the version in fn 0's describe firmware text (`describeCore`'s last argument), e.g.
  `0.0.29 (panic at 12 s)`.
- A fixture UART receives through its UART's interrupt, which the bit-banged wires' frames hold off on their core
  (interrupts off one frame at a time: SWIO up to `SwioPhy::kIrqOffMaxUs`, RVSWD up to about 1.1 ms at the slowest
  max_speed). On an ESP32 the interrupt fires at `kUartRxFifoFull` (32) bytes of the 128-byte RX FIFO, leaving 480 us at
  2000000; a dual-core ESP32 whose `loop()` runs such a wire puts it on the other core with
  `uart.setInterruptCore(0)` before `applySaved()` (`Firmware/OepProbe/Esp32P4.h`), unless that core turns its own
  interrupts off (the classic's sampler: `Esp32.h` keeps it on `loop()`'s core and checks the FIFO against SWIO).
  Bytes the UART still drops are marked lost (detail 1 overflow), never left out silently, and the mark is at or before
  the first byte after the gap - never after it: `FixtureUart` places it where the driver tells. ESP32: it takes the
  ESP-IDF driver's events itself, in order, in a task on the interrupt's core (do not give the UART an
  `onReceiveError` / `onReceive` of your own: arduino-esp32's event task would take them), and reads no byte the events
  have not counted - a FIFO overflow lands exactly at the gap. RP2: arduino-pico's receive queue overflow lands exactly
  at the gap; the PL011's overrun and a break, read in `poll()` as flags, at the bytes counted at the look before.
- The classic ESP32's sampler and a SWIO wire share the GPIO registers' bus: the sampler's back-to-back GPIO.in reads
  move the SWIO pulses of the other core enough to garble frames, and SWIO has no parity. `OepWireGate.h` keeps them
  apart: every frame waits for the sampler to stop reading (one sample at most) and the sampler reads again after it;
  the wire is taken by a frame and held until `loop()` comes round, and while a window is exclusive (an immediate
  window, a triggered segment after its trigger, a trigger search outside its turns) no take succeeds - a request
  waits, and a search gives it a turn inside its burst at once (the console one turn a burst, ended early by
  `backgroundSent()` once it has sent what it had); an immediate window waits for the holder. `SamplerCapture::poll()`
  lets the wire go, so a sketch with both calls it every `loop()` (or no immediate window opens). The sampler's loop is
  `OepSamplerRun.h`, the same code the host tests run. The console's poll skips while it is not given the wire (the
  next poll reads): oep-if-console
  §3 sets no reading interval, only that DMSTATUS is read before DATA0 after the connection's request and that DATA0 /
  DATA1 are left alone while the hart is halted (docs/implementation-limits.ja.md §4.1).

## 8. Pushes and events

An interface that sends notifications says `notifies()`: the endpoint then puts subscribe and unsubscribe (0x30 / 0x32,
core §11.3) in its ops and answers them itself, calling `subscribe(true / false)`. A streaming one implements `pull()`
and `pending()` too; the endpoint sends data frames to the subscriber while it holds the lock, batched by the
subscription's min_bytes / max_delay_ms. `endpoint.event(*this, kind, payload, length)` sends an event, never batched:
it goes as soon as the answers ahead of it have. The logic and analog captures and the capture-group do this; an
interface that sends nothing has neither op (fn 0 sends nothing: there is no heartbeat - a host reads the probe's time
with fn 0's clock).

## 9. USB identity

A USB probe says the project's VID:PID, `1209:4F45` (`oep::reg::kUsbProjectVid` / `kUsbProjectPid`; who may ship
firmware with it: [PID-USE.md](../../PID-USE.md)), and a serial number unique per unit (the unit_id: how a host tells
probes apart and finds a probe named by it). iProduct is free text for people (`OEP probe (ESP32-P4)` in the firmware); no
host identifies a probe by it. Interfaces outside OEP on the same device, such as the ESP32-P4's in-app DFU, are part of it.
The vendor bulk interface carries bInterfaceSubClass 0x4F / bInterfaceProtocol 0x45 and a vendor HID says usage page
0xFF4F, usage 0x45 (transports §3; `Firmware/OepProbe/Esp32P4.h` patches EspUsbDevice's descriptors for that).
describe declares nothing about the VID:PID (the old discoverable is gone, core §7.5): a host finds a probe by the
project's VID:PID, by the unit id it was given, or by the port the user chose (transports §3).

## 10. Testing

- `tests/host/run.sh` builds the portable parts on the PC (g++) and runs their tests: the serial-port reader, the
  endpoint's rules (lock, plan, sharing), binds.
- The host side has a fake probe that follows the spec (`oep-client-python`: `python -m oep_client.fake_serve`); use it
  to try a host before a probe exists, and compare your probe's answers with it.
- On hardware: `oep dump --port <port>` reads everything your probe declares; then drive it from Python.

## 11. Things that bite

- `Arduino.h` makes `word(...)` a macro: a function or lambda named `word` returns its argument.
- On the ESP32-P4, `pinMode` / `digitalWrite` on RVSWD pins after `RvswdPhy::begin` take them out of the dedicated GPIO
  bundle for good (a chip reset is needed). The PHY and the pin table avoid it (`releaseQuiet`); your code must too.
- `build_opt.h` sketches (the P4's direct HS vendor build) are built with `--clean` after it changes.
- Never block in `handle()` or `loop()`: a probe that stops polling stops answering, and the host times out.
- A fixture that fails only while the debug wire is busy (the console read, a flash write) is more likely the wire's
  edges than the CPU: test it with the wire idle (detach the console) and with the wire busy, and look at the drive
  strength first. Measure "is it fixed" with the same procedure before and after, and read the result right after the
  frame you armed - a wait longer than the DUT's period lets the next, unarmed frame in and looks like a failure.
- When you fix a problem, look for the same mechanism elsewhere (the other PHYs, the other SoCs, the fixtures that
  drive lines) and check each one; write down which are fine and which are untested.
