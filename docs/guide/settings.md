# Settings

[日本語](settings.ja.md)

What you decide when you use an OEP probe, sorted by where it is kept and what it is for. The flow per use is in
[What OEP does for you](use-cases.md).

## Four kinds of decisions

| Kind | Examples | Where it is kept | When you decide |
|---|---|---|---|
| **Probe settings** | pin labels, idle states, disabled pins, log passthrough, Wi-Fi | in the probe (saved, at every boot) | when building or fixing a jig |
| **Memo** | the part on a port, wiring notes | the probe's memo store (`oep.probe.memo`) | to keep a record |
| **Operation conditions** | UART baud, I2C address, SPI mode, debug speed limit, capture rate and trigger | nowhere (the tool gives them each time) | when starting the operation |
| **Pin assignments** | which function uses which pin | nowhere (for the session only) | when starting the operation; released when the session ends |

Probe settings are only what should apply regardless of sessions (idle states, passthrough) and what should be the same
from any PC and tool (wiring names). What changes per operation, like a speed or an address, is not a probe setting.

## Handling probe settings

```sh
oep settings items <probe>                  # the items this probe has, their forms and choices
oep settings show <probe>                   # the values now and the storage state
oep settings set <probe> core channel.GPIO2.label=mcu.swdio channel.GPIO3.label=mcu.swclk
oep settings set <probe> core channel.GPIO5.idle=pull-down --save
oep settings unset <probe> core channel.GPIO5.idle
oep settings save <probe>                   # save every value now in effect
oep settings export <probe> jig.oep-settings.json
oep settings import <probe> jig.oep-settings.json   # shows what changes first
```

- **set applies at once, save keeps**: a value set applies right away; until saved, a restart drops it. save keeps
  every value in effect.
- **Category**: the probe's own items are `core`; an interface's items go by its name (`oep.wire.rvswd`,
  `oep.fixture.uart/0` and so on).
- **Items differ per probe**: only the items `oep settings items` lists exist. A probe without Wi-Fi has no Wi-Fi items.
  A probe with no storage (the CH32 probes) loses what was set at a restart.
- **Copies**: before a firmware update, or to make the same jig on another unit, import an exported file. Secret values
  (the Wi-Fi passphrase) are not exported; enter them again after the import.

## Pin labels (port and line names)

| Item | Category | Value | Purpose |
|---|---|---|---|
| `channel.<pin>.label` | `core` | text (up to 32 characters) | name a pin; as `port.line@target-pin`, it names a port |

- **Form**: `S.N@P`. `S` is the port name (starts with a letter; letters, digits, `-`; up to 16 characters), `N` the
  line name, `P` the target pin (`pa13` and so on). `@P` may be left out. A line with no role is `S@P`. Examples:
  `mcu.swdio@pa13`, `mcu.rx@pa9`, `mcu.nrst`, `mcu@pb6`.
- **Usual line names**: `swdio`, `swclk` (two-wire debug), `swio` (one-wire debug), `rx` (the probe receives; to the
  target's TX), `tx` (the probe sends), `nrst` (the target's reset), `power_hi`, `power_lo`.
- **Where they are used**: the address `oep://<probe>/<port>`, the IDE's port list (one line per port), WireSkein's
  channels (`mcu.swdio`), test code finding a line by name. The probe itself never moves a pin because of a label.
- **Case**: unique regardless of case; shown as written.
- Labels that do not take this form (`LED` and the like) are allowed: they name the pin and make no port.

## Idle states and disabled pins

| Item | Category | Value | Purpose |
|---|---|---|---|
| `channel.<pin>.idle` | `core` | `hi-z`, `pull-up`, `pull-down`, `low`, `high` | the pin's state while no function uses it; without a value, Hi-Z (no pull) |
| `channel.<pin>.disable` | `core` | bool | keep the pin from every function; the probe never touches it |

- **idle is for**: lines whose target input must not float (BOOT0 low, a chip select high, a reset pulled up). A pin
  with an output idle (`low` / `high`) cannot be the log passthrough's input.
- **disable is for**: pins wired to the board's own parts (LEDs, buttons, boot straps, a PSRAM select), the second pin
  of a signal wired twice. A disabled pin is left alone from boot.
- A pin a function lets go returns to its idle state (Hi-Z without a value).

## Debug lines

| Item | Category | Key | Value | Purpose |
|---|---|---|---|---|
| `channel.<SWDIO pin>.rest` | `oep.wire.rvswd` | the port's SWDIO | `high`, `low` | fix how the port's SWCLK rests; without a value the probe chooses from the target |
| `channel.<SWDIO pin>.hold` | each wire (`oep.wire.rvswd`, `oep.wire.swio`, `oep.wire.swd`) | the port's SWDIO | bool | keep the lines driven after a connection closes; without a value, kept |

- **rest**: the CH32L103 connects only resting low, the CH32X035 only resting high. Normally leave it to the probe;
  use it only to fix the choice for a given target.
- **hold**: some targets lose their debug state within milliseconds of the lines being let go (CH32L103), or fall into
  an error state (CH32V103), so the lines are held by default. Set false only for jigs that want the lines let go after
  a close. The hold ends when another function takes the pins.
- The wire, the speed limit, halting and attaching under reset are given by the tool at each connection (not settings).
  Values per target are in [Debug targets](debug-targets.md).

## Passthrough

Sends the target's output to the probe's serial port as it is, while no session is open: a plain serial monitor shows
the log. The key is the probe's serial port name (`cdc0`, `bridge0`, `usj0` and so on, in the order of `oep dump`'s
links). One passthrough per serial port.

**UART passthrough** (category `oep.fixture.uart`)

| Item | Value | Purpose |
|---|---|---|
| `passthrough.<port>.rx` | pin | the receiving pin (to the target's TX); with a value, passthrough runs |
| `passthrough.<port>.baud` | number | the rate; 115200 without a value |
| `passthrough.<port>.format` | `8n1`, `7e2` and so on | the format; 8N1 without a value |

**Debug-line log passthrough** (category `oep.target.console`)

| Item | Value | Purpose |
|---|---|---|
| `passthrough.<port>.wire` | `oep.wire.rvswd` or `oep.wire.swio` | the wire |
| `passthrough.<port>.swdio` | pin | SWDIO (SWIO on one wire) |
| `passthrough.<port>.swclk` | pin | SWCLK (two wires only) |
| `passthrough.<port>.mechanism` | `sdi`, `dmdata`, `dmseq` | how the target prints |
| `passthrough.<port>.max-speed` | number | the speed limit; the probe chooses without a value |

- Passthrough stops when another tool starts using the probe on that port, and comes back after. No operation is
  refused because of passthrough.
- It goes target → PC only. To send to the target, use the tool's UART or console send.
- Debug-line passthrough neither halts nor resets the target. A two-wire (RVSWD) target may be reset when attached, so
  the probe attaches only once each time the conditions come together.

## Network (probes with Wi-Fi)

| Item | Category | Value | Purpose |
|---|---|---|---|
| `wifi.<n>.ssid` | `core` | text | the network to join; tried in order of n |
| `wifi.<n>.passphrase` | `core` | text (write only) | the passphrase; never read back |
| `ipv4.<netif>.mode`, `.address`, `.prefix`, `.gateway`, `.dns` | `core` | addresses | for a fixed address |

- Set them from the serial port and save. Once joined, `oep settings network <probe>` shows the address and `oep find` finds it.
- OEP has no authentication. Use it only on a network you trust.

## The memo store

Apart from settings, the probe keeps short records (`oep.probe.memo`). The part on a port and notes kept at
`oep.probe.memo/port/<port>` read the same from any PC's tools. Labels are the wiring's truth; the memo is a record for
people. The probe never moves a pin because of the memo.

## Items a firmware adds

A firmware you build may add its own items (`addSetting`). They show in `oep settings items` like the others. Items that
apply at a restart carry that mark in items.

## By what you want to do

| You want to | Probe settings | Operation conditions |
|---|---|---|
| Flash and debug a CH32 | labels (`swdio`, `swclk` / `swio`, `nrst`); `rest` if needed | speed limit, halt or not, reset or not |
| Read a UART log | labels (`rx`, `tx`); UART passthrough to read it in a plain serial monitor | baud, format |
| Read a log over the debug lines | labels; console passthrough to read it in a plain serial monitor | mechanism (`dmseq` and so on) |
| Capture waveforms | none (with labels, channels go by name) | channels, rate, samples, trigger |
| Talk I2C / SPI, stand in for a part | none (idle for a chip select and the like if needed) | address, speed, mode, data to answer |
| GPIO, PWM, pulses, analog | idle for pins whose idle state matters | values, frequency, time |
| Keep a fixed jig | disable, labels, idle, passthrough, save and a copy | — |
| Use Wi-Fi | `wifi.*`, `ipv4.*` if needed | — |
