# Using the OEP USB VID:PID

[日本語](PID-USE.ja.md)

The MIT license of this library covers its source code. It does **not** by itself grant the use of the USB VID:PID below; this
file does, under the conditions it states.

## The ID

The OEP USB VID:PID (on pid.codes' VID) identifies **a USB device running firmware built from the
OpenEmbeddedProbe library** - on any board the library supports (ESP32-P4, RP2350, RP2040, and later ones) - that speaks the
Open Embedded Probe protocol ([oep-spec](https://github.com/Open-Embedded-Probe/oep-spec)).

Hosts use it to find OEP probes without opening every serial port (discovery); which functions a probe has is then read from
the probe itself (confirm, list, describe), never from the ID. One ID for every such probe is enough for that.

## Who may use it

You may ship firmware with this VID:PID - your own board, your own jig, a changed example - when all of these hold:

1. **It is built from this library** (a fork is fine), and the source of what you ship is published under an OSS license, as
   pid.codes requires.
2. **It speaks OEP as specified**: it answers confirm, list and describe as oep-spec's core says, and every interface it lists
   under an `oep.` name follows that interface's specification. Your own interfaces use your own reverse-DNS names
   (`io.github.<you>.<name>`), never `oep.`.
3. **It identifies itself honestly**: iProduct starts with `OEP`, the USB serial number is the probe's `unit_id` - unique per
   unit and unchanged across firmware versions (oep-spec core §3.3 / §7.5; the reference firmware derives it from the chip's
   unique ID) - and oep.core's describe carries `unit_id`, the transport list and `discoverable = 1`.
4. **It does not change the protocol incompatibly.** A firmware that changes the wire format, or answers OEP requests in a way
   the specification does not allow, is not an OEP probe and must use a VID:PID of its own.

## Who should not

- Firmware that does not speak OEP (a board that only shares the hardware) - use your own ID.
- Another, independent implementation of OEP (not built from this library) - get your own VID:PID; hosts find it by
  its iProduct and describe the same way. The protocol does not depend on this ID.
- Test builds on your desk may use pid.codes' test PID `1209:0001`; never ship it.

## Changes

This file changes only through a reviewed pull request to this repository. The ID's entry is on pid.codes,
owned by `Open-Embedded-Probe`.
