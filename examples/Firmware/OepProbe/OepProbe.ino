// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The distributed OEP probe firmware, one per chip family (OepFirmware.h): the chip's set of transports and interfaces,
// every pin chosen by the host at run time. Your own interfaces: oep::firmware.add / onSetup / onLoop before begin()
// (docs/guide/firmware-features.md).
#include <OepFirmware.h>

void setup() { oep::firmware.begin(); }

void loop() { oep::firmware.loop(); }
