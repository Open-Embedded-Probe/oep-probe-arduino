# Debug targets

[日本語](debug-targets.ja.md)

The targets an OEP probe debugs, and the wiring and settings each needs. The flow of use is in
[What OEP does for you](use-cases.md#1-flash-and-debug-a-ch32).

## The wires

| Wire | Interface | Targets | Lines | PC tools |
|---|---|---|---|---|
| RVSWD | `oep.wire.rvswd` | WCH's RISC-V (the two-wire CH32 series) | 2 (SWDIO, SWCLK) | ch32rv, ArduinoCore-CH32RV, `oep_client.riscv` |
| SWIO | `oep.wire.swio` | WCH's RISC-V (CH32V00x and the other one-wire series) | 1 (SWIO) | ch32rv, ArduinoCore-CH32RV, `oep_client.riscv` |
| SWD | `oep.wire.swd` | ARM Cortex-M (ADIv5 MEM-AP) | 2 (SWDIO, SWCLK) | `oep_client.arm` |

Which wires a probe has depends on its board ([Boards](boards.md#what-each-chip-does)). Any wire goes on any pin the probe offers.

## WCH's RISC-V (CH32)

Over the debug lines, the standard RISC-V debug module: halt, run, single-step, read and write memory and registers, and
the log through the debug module (`sdi`, `dmdata`, `dmseq`). Flashing is ch32rv's, with the same RAM loader for every
series. Which series have been flashed on hardware is kept in ArduinoCore-CH32RV's
[upload routes](https://github.com/ch32-riscv-ug/ArduinoCore-CH32RV/blob/main/docs/upload-routes.ja.md#系列ごとの対応).

### Wiring and settings per series

| Series | Wire | Debug pins | Reset (NRST) | Line rest, speed | Notes |
|---|---|---|---|---|---|
| CH32V002 / V003 / V004 | SWIO | PD1 | PD7, off as shipped (option bytes RST_MODE 11) | the probe chooses | Whether NRST is on is the flashed application's choice through the option bytes. While it is off, reset goes over the debug line |
| CH32V005 / V006 / V007, CH32M007 | SWIO (RVSWD too, OEP uses SWIO) | PD1 (PD1 / PB3 for RVSWD) | depends on the package (PA7, PC5, PD7 and others), off as shipped | the probe chooses | The chip ID does not tell the package; find the reset pin in the board's documents |
| CH32X033 / X035 | RVSWD | PC18 (SWDIO), PC19 (SWCLK) | PA21, only while the option bytes' RST_MODE turns it on | SWCLK rests high (low gets no answer); the probe chooses it from the target | Some packages bond PC16 / PC11 and PC17 / PC10 to one lead each |
| CH32L103 / M103 | RVSWD | PA13, PA14 | a dedicated NRST pin | SWCLK rests low (high drops the debug link); up to 1 MHz | May not connect while running at 72 / 96 MHz; build test images with a wait at start (ArduinoCore-CH32RV's boot hold) |
| CH32V103 | RVSWD | PA13, PA14 | a dedicated NRST pin | the probe chooses | Only the V103 talks the long frame; the probe tells it apart by itself. BOOT0 / BOOT1 enter the factory ISP |
| CH32V203 / V208, CH32V303 / V305 / V307 / V317 | RVSWD | PA13, PA14 | a dedicated NRST pin | the probe chooses | BOOT0 enters the factory ISP |
| CH32V205, CH32V407 / V467, CH32X305 / X315 | RVSWD (one wire too) | PA13, PA14 | a dedicated NRST pin | the probe chooses | To use one wire through OEP, give the probe SWIO only |
| CH32M030 | RVSWD (one wire too) | PA3 (SWDIO), PA2 (SWCLK) | check the board's documents | the probe chooses | As above |

- For series not in the table, see ch32rv's tables (`ch32rv db`, `ch32rv capabilities`).
- "The probe chooses" means the probe picks how the lines rest (SWCLK resting high or low) from the target's answer. To
  fix it for a given target, use the port's `channel.{}.rest` setting ([Settings](settings.md#debug-lines)).
- The speed limit is given by the tool at each connection (`max_speed`). The probe picks a speed that holds within it.

### Wiring tips

- **Wire NRST**: to recover when the application has turned the debug pin into a GPIO or sleeps too deeply to answer
  (`ch32rv recover --method nrst`). On series whose NRST is off in the option bytes, it works only while the application
  turns it on.
- **Wiring BOOT0? Choose its idle state**: the probe leaves unused pins Hi-Z (no pull). If BOOT0 goes to the probe
  instead of a jumper, give that pin `channel.{}.idle` = `pull-down` (or `low`) to keep it low. Without ISP tests, a
  jumper to GND is the surer way.
- **Short lines**: long lines and a line split many ways cause debug errors. When one signal goes to two probe pins,
  set one of them `channel.{}.disable`.
- **Name the port**: `oep pins --wire rvswd --name <port> --save` (`--wire swio` for one wire) scans for the lines and
  makes them labels. The scan drives free pins; leave out pins wired to other things with `--exclude`.

### When it gets stuck

- The debug line does not answer: suspect the application using the debug pin for something else, or a deep sleep, and
  recover with NRST.
- A CH32V103 answers every access with a status error: an active V103 gets there on a short frame, on its lines let go,
  or on a frame cut in the middle, and stays there until its power goes. Power the target off and on. An OEP probe asks
  in the long frame first and keeps the lines driven after a close, so normal use does not get there.

## ARM Cortex-M (SWD)

Over SWD with the DP and a MEM-AP: read and write memory and peripheral registers, halt and resume the core, read and
write its registers, call a function placed in the target's RAM, system reset. From Python's `oep_client.arm`
(`SwdWire`, `ArmAdi`, `MemAp`, `CortexM`).

| Item | |
|---|---|
| Wiring | SWDIO, SWCLK, GND; NRST when needed |
| Multidrop targets | give TARGETSEL at attach |
| Flashing | no flash loader is provided; run the target's flash procedure from Python |
| GDB | not provided |

## The probe's settings for debugging

What to set in the probe itself for debugging (saved, they apply at every boot):

| Setting | Purpose | When |
|---|---|---|
| `channel.{}.label` (`<port>.swdio`, `.swclk`, `.swio`, `.nrst`) | reach the target by port name | targets you come back to; the IDE's port list shows one line per port |
| `channel.{}.rest` (an `oep.wire.rvswd` item) | fix how SWCLK rests, per port | only targets the probe's choice does not suit |
| `channel.{}.hold` (an item of each wire) | keep the lines driven after a close or not | held by default; false only for jigs that want the lines let go |
| `channel.{}.idle` | keep BOOT0 and the like in a set state while unused | BOOT pins wired to the probe |
| `passthrough.{}.*` (`oep.target.console` items) | send the debug module's log to the serial port | to see the log in a plain serial monitor |

Given by the tool at each connection (not settings): the wire, the pin pair, the speed limit, whether to halt, whether
to attach under reset.
