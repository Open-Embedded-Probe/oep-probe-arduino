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
void oepWithWifi();
void oepAddHid();
void oepAddCdc();
void oepAddBridge();
void oepAddWifi();

// The interfaces, and fn 0's port_speed
void oepWithPortSpeed();
void oepWithRvswd();
void oepWithSwio();
void oepWithConsole();
void oepWithSwd();
void oepWithGpio();
void oepWithUart();
void oepWithI2c();
void oepWithSpi();
void oepWithCapture();
void oepWithConfig();
void oepWithAnalog();
void oepWithGroup();
void oepWithLink();
void oepAddPortSpeed();
void oepAddRvswd();
void oepAddSwio();
void oepAddConsole();
void oepAddSwd();
void oepAddGpio();
void oepAddUart();
void oepAddI2c();
void oepAddSpi();
void oepAddCapture();
void oepAddConfig();
void oepAddAnalog();
void oepAddGroup();
void oepAddLink();
