# The OEP USB ID

[日本語](PID-USE.ja.md)

The reference firmware currently runs with a **temporary USB ID**: the board's default VID:PID (on the ESP32-P4,
`303a:0002`, arduino-esp32's TinyUSB default), with an iProduct starting `OEP`. This temporary ID is **not for
distribution**: do not ship products or firmware that rely on it.

When the project obtains a PID of its own, the reference firmware will switch to it. Until then, as a **temporary rule**,
hosts look for OEP probes by an iProduct starting `OEP` (and similar clues, oep-spec host-development-guide §1.7). Such a clue
is only a candidate: whether a device is an OEP probe is known only after the host opens it and a confirm is answered, so a
host may open an unrelated USB device that happens to match (it sends a confirm only and closes the device when no valid
answer comes, oep-spec core §3.3). Some setups need the target named explicitly - the probe by its unit_id
(`oep://<unit_id>`), or the port chosen by the user. What a probe offers is then read from the probe itself (confirm, list,
describe). Firmware that needs a USB ID for distribution before then should use one of its own.

## Who may use the project's own PID, once obtained

The MIT license of this library covers its source code. It does **not** by itself grant the use of the project's PID; this
file does, under the conditions below. The PID will identify **a USB device running firmware built from the OpenEmbeddedProbe
library** - on any board the library supports - that speaks the Open Embedded Probe protocol
([oep-spec](https://github.com/Open-Embedded-Probe/oep-spec)). One ID for every such probe is enough, because what a probe can
do is read from the probe itself, never from the ID.

You may ship firmware with that PID - your own board, your own jig, a changed example - when all of these hold:

1. **It is built from this library** (a fork is fine), and the source of what you ship is published under an OSS license.
2. **It speaks OEP as specified**: it answers confirm, list and describe as oep-spec's core says, and every interface it lists
   under an `oep.` name follows that interface's specification. Your own interfaces use your own reverse-DNS names
   (`io.github.<you>.<name>`), never `oep.`.
3. **It identifies itself honestly**: iProduct starts with `OEP`, the USB serial number is the probe's `unit_id` - unique per
   unit and unchanged across firmware versions (oep-spec core §3.3 / §7.5) - and oep.core's describe carries `unit_id`, the
   transport list and `discoverable = 1`.
4. **It does not change the protocol incompatibly.** A firmware that changes the wire format, or answers OEP requests in a way
   the specification does not allow, is not an OEP probe and must use a USB ID of its own.

It should not be used by:

- Firmware that does not speak OEP (a board that only shares the hardware) - use your own ID.
- Another, independent implementation of OEP (not built from this library) - use your own ID; hosts open it when the user
  names it or chooses its port, or by their own support for that ID. The protocol does not depend on the ID.

## Changes

This file changes only through a reviewed pull request to this repository.
