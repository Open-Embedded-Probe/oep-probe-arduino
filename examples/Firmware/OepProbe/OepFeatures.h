// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// The firmware's choices: each oepWithX() is weak in the chip's header, its default adding X (oepAddX()) or not; a file
// of the sketch's own (a .cpp beside OepProbe.ino) defines the ones it picks otherwise, e.g.
//
//   #include "OepFeatures.h"
//   void oepWithSwd() {}                   // no oep.wire.swd / oep.target.arm-adi
//   void oepWithCdc() { oepAddCdc(); }     // a USB CDC port too
//
// A left-out X links none of its code. Which X a chip has is its header's (an oepAddX() it does not define is not
// there to call).
#pragma once

// The transports past the chip's own
void oepWithHid();
void oepWithCdc();
void oepWithBridge();
void oepAddHid();
void oepAddCdc();
void oepAddBridge();

// The interfaces
void oepWithRvswd();
void oepWithConsole();
void oepWithSwd();
void oepWithGpio();
void oepWithUart();
void oepWithConfig();
void oepWithLink();
void oepAddRvswd();
void oepAddConsole();
void oepAddSwd();
void oepAddGpio();
void oepAddUart();
void oepAddConfig();
void oepAddLink();
