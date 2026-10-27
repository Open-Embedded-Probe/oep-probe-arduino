// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A firmware's choices without build options (examples/Firmware/OepProbe/OepFeatures.h): each part X is added by a
// function of its own (oepAddX), the only code that names X's objects, so a firmware that never calls it links none of
// X (LTO or not, -ffunction-sections or not). Its objects are Slots made by that one call; what X leaves to run later
// (its loop poll) goes through Hooks. What needs two parts (the settings' place for the console, a group's track) is the
// code of the one using it: X only sets a pointer to its object, the other checks it - a probe without the user links
// none of it, one without X the user's own part only (folded away too where the link has -ffunction-sections and LTO).
//
// A part chosen by a constant instead (a weak bool and an if around its uses) is dropped only once the constant is
// folded, after the link's code is laid out: without -ffunction-sections at the link it stays, and an object it names
// still pulls in the archive member defining it (measured on the CH32X035: 0.8 KB more at its default set).
#pragma once

#include <new>
#include <stddef.h>
#include <stdint.h>

namespace oep {

// An object made in place by the one call that adds it: no guard, no destructor run at exit (static storage only: the
// parts it is handed to keep pointing at it)
template <typename T>
class Slot {
 public:
  template <typename... A>
  T &make(A &&...a) {
    return *new (bytes_) T(static_cast<A &&>(a)...);
  }
  T *operator->() { return reinterpret_cast<T *>(bytes_); }
  T &operator*() { return *operator->(); }

 private:
  alignas(T) uint8_t bytes_[sizeof(T)];
};

// A Slot whose object is on the heap, taken by the call that makes it and never given back: for a large part on a chip
// whose static RAM is the tight one (the classic ESP32's DRAM segment), the heap having room at boot. The parts it is
// handed to keep pointing at it, as at a Slot's.
template <typename T>
class HeapSlot {
 public:
  template <typename... A>
  T &make(A &&...a) {
    p_ = new T(static_cast<A &&>(a)...);
    return *p_;
  }
  T *operator->() { return p_; }
  T &operator*() { return *p_; }

 private:
  T *p_ = nullptr;
};

// What the parts leave to run later, in the order added (N at most: a part past it is not run)
template <typename F, uint8_t N>
struct Hooks {
  F f[N] = {};
  uint8_t n = 0;
  void add(F h) {
    if (n < N) f[n++] = h;
  }
  template <typename... A>
  void run(A &...a) const {
    for (uint8_t i = 0; i < n; ++i) f[i](a...);
  }
};

}  // namespace oep
