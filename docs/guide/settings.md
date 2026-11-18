# Settings

[日本語](settings.ja.md)

What you decide when using an OEP probe, sorted by where it stays and what it is for. The flow per use is in
[What OEP does for you](use-cases.md).

## Four kinds of things to decide

| Kind | Examples | Where it stays | When to decide |
|---|---|---|---|
| **Probe settings** | pin labels, idle states, forbidden pins, log pass-through, Wi-Fi | in the probe (once saved, applied at every boot) | when building a fixture, when installing it |
| **Memos** | the part number of the peer behind a port, wiring notes | the probe's memo area (`oep.probe.memo`) | when you want to keep a record |
| **Operation conditions** | UART baud, I2C address, SPI mode, debug speed limit, capture rate and trigger | not kept (the tool passes them on every operation) | when starting an operation |
| **Pin assignment** | which feature uses which pin | not kept (only during the session) | when starting an operation. Released when the session ends |

Only what should work regardless of sessions (idle states, pass-through) and what should be the same from every PC and every
tool (wiring names) become probe settings. Things that change per operation, such as communication speeds and addresses, are
not probe settings.

## Handling probe settings

```sh
oep settings items <probe>                 # the items this probe has, their forms and choices
oep settings show <probe>                  # the current values and the save state
oep settings set <probe> core channel.GPIO2.label=mcu.swdio channel.GPIO3.label=mcu.swclk
oep settings set <probe> core channel.GPIO5.idle=pull-down --save
oep settings unset <probe> core channel.GPIO5.idle
oep settings save <probe>                  # save all current values
oep settings export <probe> jig.oep-settings.json
oep settings import <probe> jig.oep-settings.json   # shows what changes first
```

- **set applies at once, save keeps it**: a set value takes effect on the spot. Until save, a restart loses it. save saves all
  values at that time.
- **Categories**: probe-wide items are under `core`, and interface items under that interface's name (`oep.wire.rvswd`,
  `oep.fixture.uart/0` and so on).
- **Items differ per probe**: only the items shown by `oep settings items` can be used. A probe without Wi-Fi has no Wi-Fi items.
  On a probe with no place to save (a CH32 probe), set values are lost on restart.
- **Backups**: before updating the firmware, or to build the same fixture on another unit, import an exported file. Secret
  values (the Wi-Fi passphrase) are not exported, so enter them again after import.

## Pin labels (port and line names)

| Item | Category | Value | Purpose |
|---|---|---|---|
| `channel.<pin>.label` | `core` | text (up to 32 characters) | names a pin. In the form `port.line@peer pin` it becomes a port name |

- **Form**: `S.N@P`. `S` is the port name (starts with a letter; letters, digits and `-`, up to 16 characters), `N` the line
  name, `P` the peer's pin (`pa13` and so on). `@P` can be left out. A line without a role is written `S@P`. Examples:
  `mcu.swdio@pa13`, `mcu.rx@pa9`, `mcu.nrst`, `mcu@pb6`.
- **Line name conventions**: `swdio`, `swclk` (2-wire debug), `swio` (1-wire debug), `rx` (the probe receives; connect to the
  peer's TX), `tx` (the probe sends), `nrst` (the peer's reset), `power_hi`, `power_lo`.
- **Where they are used**: `oep://<probe>/<port>` addresses, the IDE's port list (per port), WireSkein's channel names
  (`mcu.swdio`), and test code that looks lines up by name. The probe itself does not drive pins by label.
- **Case**: unique without regard to case. Shown as written.
- Labels that do not fit the form (such as `LED`) can be set too. They work as pin names and do not become ports.

## Idle states and forbidden pins

| Item | Category | Value | Purpose |
|---|---|---|---|
| `channel.<pin>.idle` | `core` | `hi-z`, `pull-up`, `pull-down`, `low`, `high` | the pin's state while no feature uses it. Without a value, Hi-Z (no pull) |
| `channel.<pin>.disable` | `core` | boolean | no feature may use the pin. The probe does not touch it |

- **When to use idle**: lines where the peer's input must not float (BOOT0 low, a chip select high, a reset pull-up). A pin with
  an output idle (`low` / `high`) cannot be the receiving end of a log pass-through.
- **When to use disable**: pins connected to parts on the board (LED, buttons, boot straps, PSRAM select lines), pins that carry
  the same signal a second time. A disabled pin is not touched from boot on.
- When a feature is done with a pin, the pin goes back to its idle state (Hi-Z without a value).

## Debug wires

| Item | Category | key | Value | Purpose |
|---|---|---|---|---|
| `channel.<SWDIO pin>.rest` | `oep.wire.rvswd` | the port's SWDIO | `high`, `low` | fixes the rest state of that port's SWCLK. Without a value, the probe chooses from the peer |
| `channel.<SWDIO pin>.hold` | each wire (`oep.wire.rvswd`, `oep.wire.swio`, `oep.wire.swd`) | the port's SWDIO | boolean | whether to keep driving the wire after the connection closes. Without a value, it keeps it |

- **rest**: CH32L103 connects only with low, CH32X035 only with high. Normally leave it to the probe's choice, and use it only to
  fix it for a known peer.
- **hold**: some peers lose their debug state within a few ms when the wire is released (CH32L103), or enter an error state
  (CH32V103), so by default it is kept. Set false only for fixtures that want the wire released after closing. When another
  feature takes the pin, holding ends there.
- The kind of wire, the speed limit, whether to halt when connecting, and whether to connect while holding reset are passed by
  the tool on every connection (not probe settings). Values per peer are in [Debug targets](debug-targets.md).

## Pass-through

A setting that carries the peer's output to the probe's serial port as is while there is no session. You can watch the log in an
ordinary serial monitor. The key is the name of the probe's serial port (`cdc0`, `bridge0`, `usj0` and so on, in the order of
the transport list in `oep dump`). One port takes one pass-through.

**UART pass-through** (category `oep.fixture.uart`)

| Item | Value | Purpose |
|---|---|---|
| `passthrough.<port>.rx` | pin | the receiving pin (connect to the peer's TX). With a value, pass-through runs |
| `passthrough.<port>.baud` | number | speed. Without a value, 115200 |
| `passthrough.<port>.format` | `8n1`, `7e2` and so on | format. Without a value, 8N1 |

**Debug wire log pass-through** (category `oep.target.console`)

| Item | Value | Purpose |
|---|---|---|
| `passthrough.<port>.wire` | `oep.wire.rvswd` or `oep.wire.swio` | the wire used |
| `passthrough.<port>.swdio` | pin | SWDIO (SWIO for 1 wire) |
| `passthrough.<port>.swclk` | pin | SWCLK (2 wires only) |
| `passthrough.<port>.mechanism` | `sdi`, `dmdata`, `dmseq` | how the peer outputs its log |
| `passthrough.<port>.max-speed` | number | speed limit. Without a value, the probe decides |

- Pass-through stops when another tool starts using the probe on that port, and comes back when it ends. No operation is refused
  because of pass-through.
- The direction is peer → PC only. To send to the peer, use UART send or console send from a tool.
- Debug wire pass-through neither halts nor resets the peer. Connecting to a 2-wire (RVSWD) peer can reset it, so it tries to
  connect only once each time the conditions come together.

## Network (probes with Wi-Fi)

| Item | Category | Value | Purpose |
|---|---|---|---|
| `wifi.<number>.ssid` | `core` | text | the network to join. Tried in number order |
| `wifi.<number>.passphrase` | `core` | text (write-only) | the passphrase. Cannot be read back |
| `ipv4.<netif>.mode`, `.address`, `.prefix`, `.gateway`, `.dns` | `core` | addresses and so on | for a fixed address |

- Set them from the serial port and save. Once connected, `oep settings network <probe>` shows the address, and `oep find` finds
  it.
- OEP has no authentication. Use it only on a trusted network.

## Memo area

Apart from settings, a probe has a place for short records (`oep.probe.memo`). Put the part number of the peer behind a port or
notes in `oep.probe.memo/port/<port>`, and tools on any PC read the same record. The labels are the authority on wiring; the memo
is a record for people to read. The probe never drives pins from the memo's contents.

## Items added by the firmware

Firmware you build yourself can add its own items (`addSetting`). They appear in `oep settings items` in the same form as other
items. Items that take effect on restart carry that mark in items.

## Lookup by purpose

| What you want to do | Probe settings | Operation conditions |
|---|---|---|
| Flash and debug a CH32 | labels (`swdio`, `swclk` / `swio`, `nrst`). `rest` if needed | speed limit, whether to halt, whether to reset |
| Read a UART log | labels (`rx`, `tx`). UART pass-through to watch in an ordinary serial monitor | baud, format |
| Read the log over the debug wire | labels. Console pass-through to watch in an ordinary serial monitor | mechanism (`dmseq` and so on) |
| Capture waveforms | none (with labels, lines can be named) | channels, rate, sample count, trigger |
| Talk I2C / SPI, stand in for the peer | none (idle of chip selects and the like if needed) | address, speed, mode, data to return |
| GPIO, PWM, pulses, analog | idle of pins whose waiting state you want to fix | values, frequency, time |
| Install as a fixture | disable, labels, idle, pass-through, saving and backups | — |
| Use over Wi-Fi | `wifi.*`, `ipv4.*` if needed | — |
