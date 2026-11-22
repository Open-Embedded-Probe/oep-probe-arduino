# Writing a probe

[日本語](writing-a-probe.ja.md)

How the library is put together, and how to use it to make your own probe or your own interface. Open the examples and read
them. `01.Basics/MinimalProbe` is the skeleton, `02.Interfaces/CustomInterface` an interface, and `Firmware/OepProbe` everything
combined.

This is the detailed guide (the reference for the whole). Depending on your purpose, start from one of these.

- Beginners: [Your first interface](your-first-interface.md). Step by step, starting from one that uses no pins.
- Adding your own interface to the ready-made firmware, leaving features out of the firmware: [Firmware features](firmware-features.md).
- Adding an `oep.*` interface to this library: [Adding a standard interface](adding-a-standard-interface.md).

§1-§5 and §8 are needed for every probe. §6, §7 and §11 are the details of debug wires, shared serial ports and per-chip
quirks; read them when you build those.

## 1. The shape of a probe

```text
 host  ==frames==>  transport (Stream) --> Endpoint --> interface (fn 1)   oep.wire.rvswd
                    transport 2 .......        |     --> interface (fn 2)   oep.target.riscv-dm
                                               |     --> interface (fn 3)   io.github.you.thing
                                               |     --> oep.probe.plan, oep.probe.restart (the endpoint's own, last)
                                     fn 0 (the core, no name): confirm, list, describe, clock, lock
```

- The **Endpoint** holds the protocol. It reads frames from each transport, checks the session and the lock, finds the
  interface by fn, and returns the result on the transport the request came from. fn 0 (the core; it has no name and is not in
  list) is built in. `oep.probe.plan` (offered if some interface has plan roles) and `oep.probe.restart` (offered with
  `setRestart`) are built in too, and at the first `poll()` the endpoint numbers them in this order after all the interfaces
  added. With the same firmware they get the same fn on every boot.
- An **interface** is a class with a name and operations. The endpoint numbers them in the order of `add()` (fn 1, 2, ...). The
  host finds them by name, not by number. An interface has at least one op.
- The sketch wires these together. It places the objects, adds them in `setup()`, and calls `poll()` in `loop()`.

## 2. The endpoint

```cpp
static uint8_t rx[1100], tx[1024];
static oep::Endpoint endpoint(Serial, rx, sizeof rx, tx, sizeof tx, {1024, 4096, 8}, oep::Endpoint::kUsbCdc);
```

- The **transport** can be any `Stream`. The **kind** decides the frame form. Serial ports (`kUsbCdc`, `kUsbSerialJtag`,
  `kUartBridge`) carry `0x00 <COBS> 0x00` frames with a CRC, and can carry raw bytes between them (bind, §7). Message transports
  (`kVendorBulk`, `kHid`, `kTcp`) carry messages prefixed with a length. A transport is added with
  `endpoint.addTransport(stream, rx, sizeof rx, kind, usb_interface)` (`03.Transports/MultipleTransports`).
- **TCP**: the listening socket is one transport entry (kind 6), and each accepted connection is a separate transport. After
  all other transports are added, `endpoint.addTcpListener(tcp.slots(), rx, sizeof rx[0], n)`, and `oep::TcpListener<n>`
  (`OepTcp.h`, lwIP sockets) is polled in `loop()` before the endpoint. A connection's answers and notifications go back to that
  connection, a length above max_frame closes the connection, and a closed connection ends nothing (transports §1-§3).
  `OepWifi.h` joins the networks of the settings' wifi items and announces `_oep._tcp` over mDNS (`03.Transports/WifiTcp`).
- The **limits** `{max_frame, window, max_inflight}` are the values promised to the host in confirm. A serial port's rx holds
  encoded frames, so make it a little larger than max_frame (`cobsFrameMax`).
- You write fn 0's **describe** yourself. `describeCore(w, model, unit_id, ...)` writes the firmware version, model, the unit id
  that is the same on every transport (`platformUnitId`), and the channel count (the chip follows with `describeChip`). Pass it
  with `setProbeDescription`. The endpoint adds the transport list's transport and `max_op_ms`. describe is declarations only
  (core §7.3): nothing that changes while running goes in.
  unit_id is required (core §7.5). On chips where the library does not know a unique number, the build stops until you give
  `-DOEP_UNIT_ID='"..."'` (1-16 characters of `a-z 0-9 -`, different per unit).
- boot_id (core §6.5) is chosen by the endpoint itself when the first message arrives: from the platform's hardware random source
  if it has one, otherwise from the timer value at that moment. A sketch with a better source (such as a boot count kept in
  non-volatile memory) passes it with `setBootId` in `setup()`.
- At boot, before the first `poll()`, all unreserved channels are put in the idle state: Hi-Z, no pull (`platformParkMask`;
  core §8), or their idle if the probe has saved settings.
- `poll()` from `loop()`. It does not stall for long. Interfaces must not stall either.

## 3. Interfaces

```cpp
class Blink final : public oep::Interface {
 public:
  const char *name() const override { return "io.github.you.blink"; }   // your own reverse-DNS name. Not oep.*
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }                      // the form of the fixed parts
  bool lockFree(uint8_t op) const override { return op == 0x12; }      // a read that changes nothing
  bool offers(uint8_t op) const override { return op >= 0x10 && op <= 0x12; }   // the ops it has (the ops tag). 0x01-0x0F are the common ops
  size_t describe(uint8_t *out, size_t capacity) override;             // what the host reads before use
  oep::Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
};
```

- The host finds it by **name**. Standard interfaces are `oep.*` (defined by oep-spec), and your own use the reverse DNS of a
  name you own (`io.github.<you>.<name>`). Nobody's approval is needed, and a host that does not know it just does not use it.
  This is how OEP is extended.
- **revision** fixes every fixed form (core §2.3, §2.7): the fixed part of the payload, TLV values, list elements. Fixed forms
  are not extended at the end. Additions are new TLVs, optional ops or new values, and revision is raised only when a fixed form
  changes.
- **offers** states the ops it has (core §1.2): every required op of the table and the optional ops this probe has. From it the
  endpoint writes describe's `ops` tag (0x07, base + bitmap) at the start of every fn's describe, and answers other ops with
  `unknown_operation` before looking at the session. The default has nothing, so an interface that does not override it answers
  nothing. Optional ops are declared here, not in `features`.
- **handle** takes one request and writes the result payload to `out`.
  - `completed(n)`: executed and succeeded. The payload is n bytes.
  - `failed(n)` / `partial(n)`: executed but did not succeed (wholly / partly). The payload has the success form.
  - `rejected(reason)`: not executed. `kRejectMalformed`, `kRejectUnavailable`, `kRejectUnsupported`, `kRejectUnknownOperation`.
- A **TLV** is `tag(u8) len(u16) value`, one form whatever the length (core §2.2; `TlvWriter`, `tlvAt`, `kTlvHeader`). A list is
  `count, count × element`, with no element length.
- **TLV tails**: every request can carry TLVs at its end. `plainTail(tail, payload, length, fixed, out, capacity)` reads what
  follows the fixed part. An unknown critical TLV refuses the request (seen with `refused()`), other unknown TLVs are ignored,
  and nothing is added to the answer (core §2.3). When you implement tags, use `tail.parse(...)`, `tail.find(tag, len, &critical)`
  (the first one if repeated), `tail.fixed(tag, size, value, ...)` for fixed-length TLVs (other lengths are malformed whatever
  critical says), and `Tail::refuse(tag, critical, ...)` for values you do not handle (unsupported with the tag as received,
  whatever critical says; an implemented TLV is not ignored).
- The endpoint looks at the **lock**. Every request carries a session_id in its header (0 = none). An op that is not `lockFree`
  is executed only for the session that holds the lock (session_id 0 is `session_required`). Every time that session's lock ends
  (end, lease expiry, another host's force, all the same; core §6.4, §9), `sessionOver()` is called, so remove everything it made
  and its shared shares (its share of wire connections and console streams). Nothing passes to the next session. What the
  settings hold (pass-through connections) is not an OEP resource, so it is outside this. A resource that sits on another
  interface's resource (a console stream on a wire connection) says so with `sessionOverFirst()` and is removed first.
- Write **describe** with `TlvWriter`: the common tags (`roleChannels`, `u32(kTagMaxClockHz, ...)`, `u32(kTagFeatures, ...)` for
  optional features that are not ops) and your own tags (from 0x40). Do not write the `ops` tag (the endpoint writes it).
- **oep.probe.link** (the wire test, and the UART bridge's port_speed) is an optional interface: `oep::Link link(endpoint);
  endpoint.add(link);`. Add it last so the earlier fn numbers do not change. `endpoint.setPortSpeed(...)` puts port_speed into
  ops.
- **oep.probe.restart** (optional, oep-if-restart): calling `endpoint.setRestart(oep::platformRestart, max_ms)` in `setup()`
  before the first `poll()` offers it (restart 0x01, `restart_max_ms` in describe). `max_ms` is the longest time from the answer
  until confirm answers again on the same transport, including boot and USB re-enumeration. Estimate it per board, with a margin.
  The endpoint answers first and flushes, ends the session, calls `probeRestart()` on every interface (what the settings hold -
  pass-through connections - is released too, without touching the target), undoes all plans, and then calls the handler. The
  handler does not return (`esp_restart`, `rp2040.reboot()`). To take your own USB device off first, wrap it before passing it.

## 4. Pins: the table and the plan

- `oep::PinTable pins(mask)` is the set of channels (GPIO numbers) the sketch hands to the interfaces. An interface `claim()`s
  what it uses with its own owner number and gives it back with `release()`. A held channel is refused to the others, so two
  interfaces never drive the same pin (core §8.1). A channel given back goes to its idle state (Hi-Z, or as the settings say).
- The **plan** is how the host assigns pins at run time (`oep.probe.plan`, oep-if-plan). The fn of an interface without plan
  roles (`planRoles()` false) is refused as unsupported. When `plan_apply` lists (fn, role, channel), the endpoint asks each
  interface `planCheck()` (no side effects: 0 or a refusal reason), then `planApply()`, and has it give back with
  `planRelease()`. Roles and their candidate pins are declared in describe (`roleChannels`). planCheck refuses undeclared roles
  or channels with `kRejectUnsupported` (the endpoint adds tag 0x90), and a declared channel held by someone else with
  `kRejectUnavailable`. planApply changes nothing on the pins (an output idle keeps driving). The interface's first operation
  changes them. Debug wire pins go back to the idle state while no connection holds them (`DmiPhy::free`).
- Owner numbers of the interfaces sharing one table must not overlap. The library's are listed in `oep::owners`
  (`OepPinTable.h`). The sketch's own interfaces use `oep::owners::kSketch` (0x80) and up.

## 5. This library's standard interfaces

| Class | Interface | Notes |
|---|---|---|
| `WireRvswd` + `DebugPort` + `Ch32Dm` + `RvswdPhy` | `oep.wire.rvswd` | RVSWD. The PHY is per chip (the RP2's SIO, the ESP32's dedicated GPIO and so on) |
| `WireRvswd` (named `oep.wire.swio`) + `SwioPhy` | `oep.wire.swio` | one wire |
| `TargetRiscvDm` | `oep.target.riscv-dm` | halt / resume / step / reset, block reads and writes, run. Restores the target's registers and DATA0/1 before letting the target run |
| `TargetConsoleStream` + `DmConsole` | `oep.target.console` | the target console through the debug module (dmseq, DMDATA, SDI) |
| `WireSwd` + `SwdPort`, `TargetArmAdi` | `oep.wire.swd`, `oep.target.arm-adi` | ARM SWD (except CH32X035) |
| `FixtureGpio`, `FixtureUart` | `oep.fixture.gpio` / `uart` | on pins decided by the plan |
| `LogicCapture` / `SamplerCapture` | `oep.fixture.logic` | the ESP32-P4's PARLIO / the classic ESP32's GPIO sampler. Level / edge trigger (linked too on P4) |
| `AnalogCapture` | `oep.fixture.analog` | ADC channels in turn, one-shot, raw values, threshold trigger (continuous conversion on ESP32, FIFO + DMA on RP2). Frontend, reference voltage, factory calibration |
| `CaptureGroup` (+ `GroupTrack`) | `oep.fixture.capture-group` | starts tracks together. Groups captures that have a `GroupTrack`. Cuts linked tracks with an event source (1 stage, 1 condition) |
| `P4I2cTarget`, `P4SpiTarget`, `Rp2SpiTarget`, `Ch32SpiTarget` | `oep.fixture.i2c-target` / `spi-target` | implemented on the ESP-IDF I2C / SPI slaves (I2C on other chips: the core's Wire slave), the RP2's PIO, the CH32 core's SPI slave |
| `Rp2LogicCapture`, `Ch32LogicCapture`, `H41xLogicCapture` | `oep.fixture.logic` | the RP2's PIO + DMA (repeat with rearm), the CH32 core's logic capture, the CH32H41x's UHSIF (streaming only) |
| `I2cController`, `SpiController` | `oep.fixture.i2c-controller` / `spi-controller` | bit-banged on any GPIO the host decides with the plan |
| `FixturePwm`, `FixtureCounter`, `FixtureDac` | `oep.fixture.pwm` / `counter` / `dac` | LEDC / the RP2's PWM slices. Counts edges with GPIO interrupts (listens only, shares the pin). The DAC of the classic ESP32 and CH32 |
| `Settings` + `Binds` | fn 0's settings (core §13) | saved by name in a LittleFS file, settings applied at boot (§7) |
| `Link`, `Memo` | `oep.probe.link`, `oep.probe.memo` | the wire test and port_speed, the memo area that keeps the host's named objects on LittleFS |
| `SimpleInterface` (+ `Worker`, `ProbeLog`) | an interface under your own name | an interface made by just registering functions ([Adding your own commands](simple-interface.md)) |

Each source file starts with the specification sections it follows.

## 6. Debug wires: a fixed pair, or pins the host chooses

- A fixed pair: `DebugPort port{dm, swdio, swclk}` and `phy.begin(swdio, swclk)` (`04.Debug/*`).
- Pins the host chooses: without fixing a pair, `port.pin_choice = mask; port.pins = &pins;`. The wire declares its channels with
  role_channels, accepts any free pair the host names in scan / attach, moves the PHY there (`usePins`), and a live connection
  holds those pins in the table. `reset_allowed` is the channels attach's reset TLV may pull, always named by the host.
- The wire's settings (rest state, speed limit) belong to the target, and the host passes them in attach. The probe has no
  per-chip defaults.
- **Debug wires are driven with the weakest output strength the wire's timing allows.** The wire's sharp edges couple into the
  lines of neighboring fixtures on the same jig. At the classic ESP32's default strength, while reading the console, the SWIO
  wire made a neighboring SPI target drop or shift bits (not at the weakest, and the wire's speed does not change). Every PHY in
  this library does this (RVSWD: `GPIO_DRIVE_CAP_0` on ESP32-P4, 2 mA on RP2. SWIO: `GPIO_DRIVE_CAP_0` on classic and P4,
  `OEP_SWIO_DRIVE_CAP`). Always set it in a new PHY or a port to another chip too.
  Set it again every time the wire takes the pins (the RVSWD PHY does so on every attach). A pair the host chooses is free
  between connections, and a peripheral function (such as the spi-target's MISO) can leave another strength behind.
- The probe decides the **output strength** (fixture §1). The levels, their rough mA, and the default (the pad's strength at
  reset) are in `platformDriveLevels()` in `OepPlatform.h`. classic ESP32 / ESP32-P4: `GPIO_DRIVE_CAP_0..3`, about
  5 / 10 / 20 / 40 mA, default 2. RP2040 / RP2350: 2 / 4 / 8 / 12 mA, default 4 mA. Other chips have none. fixture gpio's
  outputs and output idles use the default strength. Peripheral functions set the strength of their own lines with
  `PinTable::ownStrength(channel, level)` (the spi-target's MISO is one level weaker than the default). When porting to another
  chip, add levels there or leave it without. Strength applies only in modes 3 / 4 (set and output idle). `PinTable::setPad`
  restores the default strength when the pad moves to another mode. Fixtures that use the pin for their own peripheral (UART,
  I2C / SPI target) call `PinTable::ownStrength` after claim, so even when taken straight from an output idle they start from the
  pad's own strength. Wires set their own strength (the weakest) themselves.

## 7. Serial ports, binds, settings

- A serial port carries OEP frames and raw bytes between them. The raw bytes follow the **bind** (console pass-through, UART
  pass-through; one per port). It is enabled with `endpoint.setRawPorts(&binds)`. While a session holds the lock, the raw flow of
  the ports it uses stops, and after it ends it continues from where it stopped (if it overflowed meanwhile, from the oldest byte
  remaining; transports §4, console §2.1).
- `Settings` is the core's settings (core §13): fn 0's get / set / unset / save / state, and the items in each fn's describe.
  Channel items (label, idle, disable) are declared with `setPins(&pins)`, network items with `setWifi(&wifi)`, pass-through
  items with `addUart(uart)` and `addPlace(wire, console)` after `usePassthroughs()` (also the wire's hold and rvswd's rest), and
  SWD's hold with `addHold(wireSwd, swd.hold, checkSwdHold, &swd)`. The sketch's own items go under its own interface with
  `addSetting(owner, "name.{}", form, keys, max_len)`. Initial values, with `initialValue(owner, "name", key, data, len)`, or for
  port names `initialLabel(channel, "V003.swio")` (host guide §5.3), go in on a boot without a save (on every boot for a probe
  without saving), and afterwards behave like any other value (core §13.4). `setChangeable(false)` makes a probe with initial
  values only (no set / unset; core §13.5). Saving (a LittleFS file on ESP32 and RP2) is by name, so it reads back even if fn
  numbers change. It is not an interface, so it is not `endpoint.add`ed (creating it attaches it to the endpoint). Call `load()`
  before the sketch puts pins in their idle state, and leave out the channels the save disabled: `settings.load();
  pins.setDisabled(settings.savedDisabled()); platformParkMask(mask & ~pins.disabledMask());` (disabled channels are not touched
  even at boot). Finally, after all `endpoint.add` and add* calls, `applySaved()` (`06.Settings/Settings`). The PinTable keeps the
  settings' disable (removable) apart from `forbid` (the firmware's, permanent), and no value makes a forbidden pin usable.
- A probe whose transport is its own USB device holds back the boot-time attach of saved pass-throughs until the host has
  finished configuring the device: `settings.setAttachGate([] { return BootGuard::attachReady(tud_mounted()); })` before
  `applySaved()` (`usbDevice.ready()` on P4). A console that reads the target from boot stops interrupts per frame, and a delayed
  USB interrupt while the host enumerates the device can bring down the USB stack (RP2: TinyUSB's "Can't continue xfer on
  inactive ep" panic). `BootGuard` (`OepBootGuard.h`): `begin()` at the start of `setup()`, `poll()` in `loop()`. It runs the
  watchdog, counts boots that crashed quickly, and after `kSafeAfter` in a row `safe()` becomes true, so on that boot call
  `settings.skipBootAttach()`. On RP2, the SDK's `_exit` and `isr_hardfault` are pointed at `BootGuard::crashed()`
  (`src/firmware/Rp2.h`). It does not stop on panic; it resets the chip. A firmware update (ESP32 with bootloader rollback) is
  treated as working and confirmed when it boots: without defining `verifyRollbackLater`, the ESP32 core confirms the image
  before `setup()`, so no later reset goes back (`src/firmware/Esp32P4.h`). Safe boot handles repeated crashes caused by saved
  settings. `BootGuard::lastBoot()` says in a string what ended the previous boot (a reset the firmware did not intend: the kind
  of crash or stall and the seconds it had been up, brownout, whether the bootloader did not boot an update. External resets and
  its own restarts say nothing. On RP2, the mark `crashed()` sets separates its own crashes from external watchdog restarts).
  `Firmware/OepProbe` appends it after the version in fn 0's describe firmware string (the last argument of `describeCore`;
  for example `<version> (panic at 12 s)`).
- The fixture UART receives with the UART interrupt. Frames of wires bit-banged by the firmware stop that core's interrupts (per
  frame: SWIO up to `SwioPhy::kIrqOffMaxUs`, RVSWD up to about 1.1 ms at the slowest max_speed). On ESP32 the interrupt fires at
  `kUartRxFifoFull` (32) bytes of the 128-byte RX FIFO, leaving 480 us of headroom at 2000000. A dual-core ESP32 whose `loop()`
  drives such wires puts the interrupt on the other core with `uart.setInterruptCore(0)` before `applySaved()`
  (`src/firmware/Esp32P4.h`). If that core stops interrupts itself, it is not moved (the classic's sampler:
  `src/firmware/Esp32.h` keeps it on `loop()`'s core, and checks the FIFO against SWIO). Bytes the UART still drops become a lost
  mark (detail 1, overflow), never a silent gap. The mark sits at or before the position of the byte right after the gap, never
  after it: `FixtureUart` places it where the driver says. On ESP32 it takes the ESP-IDF driver's events itself, in order, in a
  task on the interrupt's core, and does not read bytes the events have not counted (do not attach your own `onReceiveError` /
  `onReceive` to the UART; arduino-esp32's event task would take them). FIFO overflow is marked exactly at the gap. On RP2,
  overflow of arduino-pico's receive queue is exactly at the gap, and PL011 overrun and break (read as marks in `poll()`) are at
  the bytes counted when last looked at.
- The classic ESP32's sampler and the SWIO wire share the GPIO register bus: the sampler's continuous reads of GPIO.in move the
  other core's SWIO pulses enough to disturb frames, and SWIO has no parity. `OepWireGate.h` keeps them apart: every frame waits
  for the sampler to stop reading (up to 1 sample), and the sampler re-reads after the frame. The wire is taken per frame and held
  until `loop()` comes round. It cannot be taken while a window is exclusive (the immediate window, the rest of the segment after
  the trigger, trigger search outside turns): requests wait, and the search gives a turn at once within a slice (once per slice
  for the console; it ends early with `backgroundSent()` once it has handed over what it sends). The immediate window waits for
  the holder. `SamplerCapture::poll()` releases the wire, so a sketch with both calls it on every `loop()` (otherwise the
  immediate window does not open). The sampler loop is `OepSamplerRun.h`, and the host tests run the same code. The console's
  poll does not read when it cannot get the wire (it reads at the next poll). oep-if-console §3 does not set a read interval, and
  only requires reading DMSTATUS before DATA0 after that connection's requests, and not touching DATA0 / DATA1 while the hart is
  halted ([implementation limits §4.1](../implementation-limits.md#41-classic-esp32)).

## 8. Pushes and events

An interface that sends notifications returns true from `notifies()`. The endpoint sets subscribe and unsubscribe (0x01 / 0x02,
core §10.3) in its ops, answers them itself, and calls `subscribe(true / false)`. Streaming interfaces also have `pull()` and
`pending()`. While a subscribed host holds the lock, the endpoint sends data frames batched by the subscription's min_bytes /
max_delay_ms. `endpoint.event(*this, kind, payload, length)` sends an event. Events are not batched; they go out as soon as the
answers ahead of them have been sent. Logic and analog capture and capture-group use this. An interface that sends nothing has
neither op (fn 0 sends nothing; there is no heartbeat, and the host reads the probe's time with fn 0's clock).

## 9. USB identity

A USB probe identifies itself with the project's VID:PID `1209:4F45` (`oep::reg::kUsbProjectVid` / `kUsbProjectPid`; the
conditions for shipping firmware with it are in [PID-USE.md](../../PID-USE.md)) and a serial number different per unit (unit_id).
The host tells units apart by it and finds named probes by it. iProduct is a free string for people to read (`OEP probe (ESP32-P4)`
in the firmware), and the host does not tell probes apart by it. Interfaces outside OEP on the same device (such as the
ESP32-P4's in-app DFU) are part of that device too. The vendor bulk interface has bInterfaceSubClass 0x4F / bInterfaceProtocol
0x45, and the vendor HID usage page 0xFF4F / usage 0x45 (transports §3; `src/firmware/Esp32P4.h` fixes EspUsbDevice's
descriptors that way). describe declares nothing about the VID:PID (core §7.5). The host finds probes by the project's VID:PID,
a named unit id, or a port the user chose (transports §3).

## 10. Trying it

- `tests/host/run.sh` builds the portable parts on a PC (g++) and tests them: reading serial ports, the endpoint's rules (lock,
  plan, sharing), binds.
- On the host side there is a virtual bench that answers as the specification says (`oep-client-python`:
  `python -m oep_client.virtual_bench_serve`). You can try a host without a probe, and compare your probe's answers.
- On hardware, `oep dump --port <port>` reads all of the probe's declarations. Then drive it from Python.

## 11. Pitfalls

- `Arduino.h` makes `word(...)` a macro. A function or lambda named `word` returns its argument as is.
- On the ESP32-P4, using `pinMode` / `digitalWrite` on the RVSWD pins after `RvswdPhy::begin` takes them out of the dedicated
  GPIO bundle for good (a chip reset is needed). The PHY and the pin table avoid this (`releaseQuiet`). Avoid it in your own code
  too.
- Do not stall in `handle()` or `loop()`. A probe that stops polling stops answering, and the host times out.
- For a fixture that breaks only while the debug wire is busy (console reads, flash writes), suspect the wire's edges before the
  CPU. Try with the wire resting (detach the console) and busy, and look at the output strength first. Measure before and after
  with the same steps to see whether it is fixed, and read right after the armed frame (waiting longer than the DUT's period lets
  the next, unarmed frame in, which looks like a failure).
- After fixing a problem, look for places with the same mechanism (other PHYs, other SoCs, fixtures that drive lines) and check
  them one by one.
