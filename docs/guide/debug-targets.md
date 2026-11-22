# Debug targets

[日本語](debug-targets.ja.md)

The peers an OEP probe can connect to for debugging, and the wiring and settings each needs. The flow of use is in
[What OEP does for you](use-cases.md#1-flash-and-debug-a-ch32).

## Kinds of wire

| Wire | interface | Peer | Lines | PC tools |
|---|---|---|---|---|
| RVSWD | `oep.wire.rvswd` | WCH RISC-V (the CH32 2-wire families) | 2 (SWDIO, SWCLK) | ch32rv, ArduinoCore-CH32RV, `oep_client.riscv` |
| SWIO | `oep.wire.swio` | WCH RISC-V (the 1-wire families such as CH32V00x) | 1 (SWIO) | ch32rv, ArduinoCore-CH32RV, `oep_client.riscv` |
| SWD | `oep.wire.swd` | ARM Cortex-M (ADIv5 MEM-AP) | 2 (SWDIO, SWCLK) | `oep_client.arm` |

Which wires a probe has depends on the probe's board ([Boards](boards.md#what-each-chip-can-do)). Every wire can be assigned to
any of the pins the probe offers.

## WCH RISC-V (CH32)

On the debug wire, the RISC-V standard debug module is used. It can halt, run, step one instruction, read and write memory and
registers, and log through the debug module (`sdi`, `dmdata`, `dmseq`). Flashing is handled by ch32rv, and every family is
written with the same RAM loader. The authoritative status of flashing per family is ArduinoCore-CH32RV's
[upload routes](https://github.com/ch32-riscv-ug/ArduinoCore-CH32RV/blob/main/docs/upload-routes.ja.md#系列ごとの対応).

### Wiring and settings per family

| Family | Wire | Debug pins | Reset (NRST) | Wire rest state, speed | Notes per peer |
|---|---|---|---|---|---|
| CH32V002 / V003 / V004 | SWIO | PD1 | PD7. Disabled at shipping (option bytes RST_MODE is 11) | the probe decides | Whether NRST is used is decided by the application written, through the option bytes. While it stays disabled, reset goes through the debug wire |
| CH32V005 / V006 / V007, CH32M007 | SWIO (RVSWD is supported too, but OEP uses SWIO) | PD1 (PD1 / PB3 for RVSWD) | differs by package (PA7, PC5, PD7 and so on). Disabled at shipping | the probe decides | The chip ID does not tell the package, so check the reset pin in the board's documentation |
| CH32X033 / X035 | RVSWD | PC18 (SWDIO), PC19 (SWCLK) | PA21. Only when enabled with the option bytes' RST_MODE | SWCLK rests high (no answer when low). The probe chooses from the peer | Depending on the package, PC16 / PC11 and PC17 / PC10 share one pin |
| CH32L103 / M103 | RVSWD | PA13, PA14 | dedicated NRST pin | SWCLK rests low (left high, the debug connection drops). Speed up to 1 MHz | While running at 72 / 96 MHz it may not connect. To try, put a wait into the application's startup (ArduinoCore-CH32RV's "startup wait") |
| CH32V103 | RVSWD | PA13, PA14 | dedicated NRST pin | the probe decides | Only the V103 talks in long-form frames. The probe tells it apart automatically. BOOT0 / BOOT1 are used to enter the factory ISP |
| CH32V203 / V208, CH32V303 / V305 / V307 / V317 | RVSWD | PA13, PA14 | dedicated NRST pin | the probe decides | BOOT0 is used to enter the factory ISP |
| CH32V205, CH32V407 / V467, CH32X305 / X315 | RVSWD (1-wire supported too) | PA13, PA14 | dedicated NRST pin | the probe decides | To use 1 wire with OEP, make the probe side SWIO only |
| CH32M030 | RVSWD (1-wire supported too) | PA3 (SWDIO), PA2 (SWCLK) | check the board's documentation | the probe decides | the same as above |

- Families not in the table are checked in ch32rv's support table (`ch32rv db`, `ch32rv capabilities`).
- "The probe decides" means the probe chooses the wire's rest state (whether SWCLK rests high or low) from the peer's answers.
  To fix it for a known peer, set it with the port setting `channel.{}.rest` ([Settings](settings.md#debug-wires)).
- The tool gives the speed limit on every connection (`max_speed`). The probe chooses a speed that passes reliably within that
  limit.

### Reset

A reset from the debug wire (the reset of OEP's `oep.target.riscv-dm`, `reset` of `oep_client.riscv`) is a system reset. Not
only the CPU but also the peripherals go back to their initial values, so a newly written application does not inherit the
previous application's peripheral settings (timers, interrupts, UART and so on).

- On CH32 the PFIC system reset is used. The NRST line is not used.
- A reset done while halted stops at the first instruction after the reset.
- For this the probe temporarily uses the first 16 bytes of the peer's RAM, and restores them afterwards.
- To reset with the NRST line, use attach's reset (connecting while holding reset).

### Wiring tips

- **Connect NRST**: it is used to recover when the application has turned the debug pins into GPIO or does not answer in deep
  sleep (`ch32rv recover --method nrst`). On families where NRST is disabled by the option bytes, it works only while the
  application has it enabled.
- **When connecting BOOT0, fix its idle state**: the probe leaves unused pins Hi-Z (no pull). If BOOT0 is connected to the
  probe instead of fixed by a jumper on the board, set `channel.{}.idle` of that pin to `pull-down` (or `low`) and keep it low
  normally. If you do not test ISP, keeping it fixed to GND with the jumper is more reliable.
- **Keep wires short**: long wires and wiring that splits one line into several cause debug wire malfunctions. When the same
  signal is connected to two pins, make one of them `channel.{}.disable`.
- **Name the port**: `oep pins --wire rvswd --name <port> --save` (`--wire swio` for 1 wire) scans for the wire, finds it and
  turns it into labels. The scan puts signals on free pins, so leave out pins connected to other devices with `--exclude`.

### Recovering when stuck

- The debug wire does not answer: suspect that the application uses the debug pins for something else or has entered deep
  sleep, and recover with NRST.
- A CH32V103 keeps returning a status error to every request: the V103 enters this state when it receives a short-form frame
  while running, when the wire is released, or when the wire breaks mid-frame, and does not come back until power is cut.
  Power-cycle the peer. An OEP probe queries in the long form first and keeps driving the wire after closing, so normal use does
  not lead to this state.

## ARM Cortex-M (SWD)

Over SWD it uses the DP and the MEM-AP: reading and writing memory and peripheral registers, halting and resuming the core,
reading and writing registers, calling functions placed in the peer's RAM, and system reset. Use it from Python's
`oep_client.arm` (`SwdWire`, `ArmAdi`, `MemAp`, `CortexM`).

| Item | Contents |
|---|---|
| Wiring | SWDIO, SWCLK, GND. NRST if needed |
| Multidrop peers | give TARGETSEL in attach |
| Flashing | RP2040 / RP2350: `python -m oep_client.rp2040_swd flash`. For other peers, do their flash procedure from Python |
| GDB | RP2040 / RP2350: `oep_client.gdb_server`. Not provided for other peers |

## Summary of settings on the probe side

What to set on the probe for debugging (once saved, it applies at every boot):

| Setting | Purpose | When needed |
|---|---|---|
| `channel.{}.label` (`<port>.swdio`, `.swclk`, `.swio`, `.nrst`) | name the peer by port name | peers used repeatedly. Each port also shows up in the IDE's port list |
| `channel.{}.rest` (an item of `oep.wire.rvswd`) | fix SWCLK's rest state per port | only peers that cannot be left to the probe's choice |
| `channel.{}.hold` (an item of each wire) | whether to keep driving the wire after closing | the default keeps it. false only for fixtures that want the wire released |
| `channel.{}.idle` | keep BOOT0 and the like in a fixed state while free | BOOT-type pins connected to the probe |
| `passthrough.{}.*` (items of `oep.target.console`) | carry the debug module's log to a serial port | to see the log in an ordinary serial monitor |

What the tool gives on every connection (not probe settings): the kind of wire, which pin pair, the speed limit, whether to
halt when connecting, whether to connect while holding reset.
