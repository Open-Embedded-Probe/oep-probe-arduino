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
                                        oep.core (fn 0): confirm, list, describe, the lock, the plan, subscriptions
```

- The **Endpoint** owns the protocol: it reads frames from each transport, checks the session and the lock, finds the
  interface by fn, and writes the result back on the transport the request came from. oep.core (fn 0) is built in.
- An **interface** is a class with a name and operations. The endpoint numbers them in the order you `add()` them
  (fn 1, 2, ...); hosts find them by name, never by number.
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
- **Limits** `{max_frame, window, max_inflight}` are what confirm promises the host. A serial port's rx buffer holds an
  encoded frame: a little more than max_frame (`cobsFrameMax`).
- oep.core's **describe** is yours to fill: `describeCore(w, model, unit_id, ...)` writes the firmware version, the model,
  a unit id that is the same on every transport (`platformUnitId`), the channel count and the reserved channels. Hand it
  over with `setProbeDescription`. The transports, `discoverable`, `plan_roles` and `max_op_ms` are added by the endpoint.
  describe is declarations only (core §7.3): nothing that changes while the probe runs goes in it.
  unit_id is mandatory (core §7.5): on a chip the library has no unique number for, the build stops until you give
  `-DOEP_UNIT_ID='"..."'` (1 to 16 of `a-z 0-9 -`, a different value per unit).
- The boot_id (core §6.5): the endpoint picks it itself when the first message arrives - a hardware random source where
  the platform has one, else the timer's count at that moment. A sketch with a better source (a boot counter it keeps in
  non-volatile storage) passes it with `setBootId` in `setup()`.
- `poll()` from `loop()`: it never blocks long. Neither may your interfaces.

## 3. An interface

```cpp
class Blink final : public oep::Interface {
 public:
  const char *name() const override { return "io.github.you.blink"; }   // yours: reverse DNS, never oep.*
  uint16_t instance() const override { return 0; }
  uint8_t revision() const override { return 1; }                      // the fixed parts' shape
  bool lockFree(uint8_t op) const override { return op == 0x03; }      // reads that change nothing
  size_t describe(uint8_t *out, size_t capacity) override;             // what a host reads before using it
  oep::Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;
};
```

- **The name** is how hosts find it. Standard interfaces are `oep.*` (specified in oep-spec); yours take a reverse-DNS
  name of something you own (`io.github.<you>.<name>`). Nobody has to approve it, and a host that does not know it leaves
  it alone - this is how OEP is extended.
- **The revision** fixes the shape of the fixed parts of every payload. Change it when they change; add optional TLVs
  without changing it.
- **handle** gets one request and writes the result's payload to `out`:
  - `completed(n)` - it ran and succeeded, n bytes of payload;
  - `failed(n)` / `partial(n)` - it ran and did not work (all / part), with the success-shaped payload;
  - `rejected(reason)` - not run: `kRejectMalformed`, `kRejectUnavailable`, `kRejectUnsupported`, `kRejectUnknownOperation`.
- **TLV tails**: any request may end with TLVs. `plainTail(tail, payload, length, fixed, out, capacity)` parses what
  follows the fixed part: an unknown critical TLV refuses the request (`refused()` tells), an unknown other one is
  recorded, and `tail.finish(result, out, capacity)` lists it as ignored. With known tags: `tail.parse(...)`,
  `tail.find(tag, len, &critical)`, and `tail.refuse(tag, critical, ...)` for a value you cannot honour. Every
  completed answer carries ignored, a failed status too (core §2.3); the list lives for the request, so the endpoint
  appends it when a handler returns `failed(n)` early without `finish` - leave room for it after the payload.
- **The lock** is the endpoint's: an op not `lockFree` only runs for the session holding the lock. `sessionLapsed()` is
  called when that session's lease ran out - drop what it left (a wire's connection).
- **describe** uses `TlvWriter`: common tags (`roleChannels`, `u32(kTagMaxClockHz, ...)`, `u32(kTagFeatures, ...)`) and
  your own (0x40 and up).

## 4. Pins: the table and the plan

- `oep::PinTable pins(mask)` is the set of channels (GPIO numbers) the sketch hands to interfaces. Each interface
  `claim()`s what it uses under its own owner id and `release()`s it; a claimed channel is refused to anyone else, so two
  interfaces never drive one pin (core §8.1). A released channel goes to its idle state (Hi-Z, or what the settings say).
- The **plan** is how a host assigns pins at run time (core §8): `plan_apply` names (fn, role, channel); the endpoint asks
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
  UART, several marked by name). `endpoint.setRawPorts(&binds)` turns it on; while a session holds the lock the raw
  transfer on the port it uses waits, and resumes afterwards from the target's last reset (core §3.4).
- `ProbeConfig` keeps slots, binds, plans, labels, idle states, the fixture UARTs' settings (the uart item) and the
  disabled channels (the disable item), saves them (NVS on ESP32, the flash's last sector on RP2) and applies them at
  boot; `state` (op 0x06) tells how the slots and binds are doing, `unset` (0x05) removes items by key. Add it last
  (`add(config)`), then `addPlace(wire, console)`, `addUart(uart)`, `setPins(&pins)`, `applySaved()`
  (`06.Settings/ProbeConfig`). Call `load()` before the sketch parks its pins and leave out what the saved settings
  disable: `config.load(); pins.setDisabled(config.savedDisabled()); platformParkMask(mask & ~pins.disabledMask());`
  (a disabled channel is never touched, not even at boot). The PinTable keeps the settings' disabled channels apart
  from `forbid` (the firmware's own, permanent): no item gives a forbidden pin back.

## 8. Pushes and events

An interface that streams implements `subscribe()`, `pull()` and `pending()`; the endpoint sends data frames to the
subscriber while it holds the lock. `endpoint.event(*this, kind, payload, length)` sends an event. The console, the
fixture UART and the capture do this.

## 9. USB identity

A USB probe says an iProduct starting `OEP` (free text; a host's temporary clue until the project's VID:PID, oep-spec host
guide §4), a serial number unique per unit (the unit_id: how a host finds a probe named by it), and the VID:PID:
the board's default (`303a:0002` on the ESP32-P4). That is a temporary USB ID, which may not be used for distribution; when the
project obtains a PID of its own, the firmware will switch to it ([PID-USE.md](../../PID-USE.md)).
The vendor bulk interface carries bInterfaceSubClass 0x4F / bInterfaceProtocol 0x45 and a vendor HID says usage page
0xFF4F, usage 0x45 (core §3.3; `Firmware/OepProbe/Esp32P4.h` patches EspUsbDevice's descriptors for that).
Only a probe on the project's own VID:PID calls `endpoint.setDiscoverable(true)` (describe discoverable, core §7.5);
until that VID:PID is listed in the registry, no probe does.

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
