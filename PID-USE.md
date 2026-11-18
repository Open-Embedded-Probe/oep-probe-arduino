# The OEP USB ID

[日本語](PID-USE.ja.md)

The project's USB VID:PID is **`1209:4F45`** (VID 0x1209, PID 0x4F45), allocated by [pid.codes](https://pid.codes/1209/4F45/)
(oep-spec registry `usb`: `project_vid` / `project_pid`). The host identifies an OEP probe automatically by this VID:PID alone
(oep-spec transports §3). iProduct is a name for people and identifies nothing. A probe on a port that cannot carry this
VID:PID (a USB-UART bridge, a built-in USB serial whose ID the hardware fixes) is opened when the user chooses its port, or
names the probe by its unit_id (`oep://<unit_id>`). What a probe can do is read from the probe itself (confirm, list,
describe), not from the ID.

## Who may use it

The MIT License of this library covers the source code. That alone does not decide whether the project's VID:PID may be used.
This document allows it, under the conditions below. This VID:PID identifies **a USB device running firmware built from the
OpenEmbeddedProbe library** (on any board the library supports) that speaks the Open Embedded Probe protocol
([oep-spec](https://github.com/Open-Embedded-Probe/oep-spec)). What a probe can do is read from the probe itself, not from the
ID, so one ID for all such probes is enough.

The scope is the whole USB device, including interfaces outside OEP that belong to the same device (such as an in-app DFU
interface that updates the probe's own firmware). A chip's ROM bootloader keeps the chip's own ID.

You may ship firmware with that VID:PID (your own board, your own jig, a changed example) when all of these hold:

1. **It is built from this library** (a fork is fine). The source of what you ship is published under an OSS license.
2. **It speaks OEP as specified**: it answers confirm, list and describe as oep-spec's core says, and every interface it
   offers under an `oep.` name follows that interface's specification. Your own interfaces get your own reverse-DNS names
   (`io.github.<you>.<name>`) and do not use `oep.`.
3. **It identifies itself honestly**: the USB serial number is the probe's `unit_id` (different per unit, unchanged across
   firmware versions; oep-spec transports §3 / core §7.5). fn 0's describe carries `unit_id` and the transport list.
4. **It does not change the protocol incompatibly.** A firmware that changes the wire format, or answers OEP requests in a
   way the specification does not allow, is not an OEP probe and uses its own USB ID.

Please do not use it for:

- Firmware that does not speak OEP (it only shares the board): use your own ID.
- An independent OEP implementation not built from this library: use your own ID. The host opens it when the user names it
  or chooses its port, or through its own support for that ID. The protocol does not depend on this ID.

## Changes

This document changes only through a reviewed pull request to this repository.
