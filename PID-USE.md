# The OEP USB ID

[日本語](PID-USE.ja.md)

The reference firmware currently runs with a **temporary USB ID**: the board's default VID:PID (on the ESP32-P4,
`303a:0002`, arduino-esp32's TinyUSB default), with an iProduct starting `OEP`. This temporary ID is **not for
distribution**: do not ship products or firmware that rely on it.

When the project obtains a PID of its own, the reference firmware will switch to it. This file will then say how it may be
used.

Hosts do not depend on the USB ID. They find OEP probes by an iProduct starting `OEP` and then read what each probe offers
from the probe itself (confirm, list, describe), as oep-spec's core says. Firmware that needs a USB ID for distribution today
should use one of its own.
