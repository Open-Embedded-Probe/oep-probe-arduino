# The OEP USB ID

[日本語](PID-USE.ja.md)

The project's USB VID:PID is **`1209:4F45`** (VID 0x1209, PID 0x4F45), allocated by [pid.codes](https://pid.codes/1209/4F45/)
(oep-spec registry `usb`: `project_vid` / `project_pid`). Hosts identify an OEP probe automatically by this VID:PID alone
(oep-spec core §3.3); iProduct is a name for people and identifies nothing. A probe whose port cannot carry this VID:PID - a
USB-UART bridge, a built-in USB serial whose ID the hardware fixes - is opened by the user choosing its port, or naming the
probe by its unit_id (`oep://<unit_id>`). What a probe offers is read from the probe itself (confirm, list, describe), never
from the ID.

## Who may use it

The MIT license of this library covers its source code. It does **not** by itself grant the use of the project's VID:PID;
this file does, under the conditions below. The VID:PID identifies **a USB device running firmware built from the
OpenEmbeddedProbe library** - on any board the library supports - that speaks the Open Embedded Probe protocol
([oep-spec](https://github.com/Open-Embedded-Probe/oep-spec)). One ID for every such probe is enough, because what a probe can
do is read from the probe itself, never from the ID.

The whole USB device is covered, including interfaces outside OEP that belong to the same device, such as the probe's own
in-app DFU interface for updating its firmware. A chip's ROM bootloader keeps the chip's own ID.

You may ship firmware with that VID:PID - your own board, your own jig, a changed example - when all of these hold:

1. **It is built from this library** (a fork is fine), and the source of what you ship is published under an OSS license.
2. **It speaks OEP as specified**: it answers confirm, list and describe as oep-spec's core says, and every interface it lists
   under an `oep.` name follows that interface's specification. Your own interfaces use your own reverse-DNS names
   (`io.github.<you>.<name>`), never `oep.`.
3. **It identifies itself honestly**: the USB serial number is the probe's `unit_id` - unique per unit and unchanged across
   firmware versions (oep-spec core §3.3 / §7.5) - and oep.core's describe carries `unit_id`, the transport list and
   `discoverable = 1`.
4. **It does not change the protocol incompatibly.** A firmware that changes the wire format, or answers OEP requests in a way
   the specification does not allow, is not an OEP probe and must use a USB ID of its own.

It should not be used by:

- Firmware that does not speak OEP (a board that only shares the hardware) - use your own ID.
- Another, independent implementation of OEP (not built from this library) - use your own ID; hosts open it when the user
  names it or chooses its port, or by their own support for that ID. The protocol does not depend on the ID.

## Changes

This file changes only through a reviewed pull request to this repository.
