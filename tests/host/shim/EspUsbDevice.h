// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host-test stand-in for the EspUsbDevice vendor class DirectBulkStream uses: IN transfers complete when the test says.
#pragma once
#include <functional>
#include <vector>
#include <stdint.h>
#include <stddef.h>
class EspUsbDeviceVendor {
 public:
  std::function<void(const uint8_t *, size_t)> rx;
  std::function<void(size_t)> tx_done;
  std::vector<std::vector<uint8_t>> sent;   // transfers written, in order
  int in_flight = 0;
  bool mounted_ = true;
  static bool directWriteSupported() { return true; }
  void onRxData(std::function<void(const uint8_t *, size_t)> f) { rx = f; }
  void onTxComplete(std::function<void(size_t)> f) { tx_done = f; }
  bool writeDirect(const uint8_t *b, size_t n) { sent.emplace_back(b, b + n); ++in_flight; return true; }
  bool mounted() const { return mounted_; }
  void complete() { if (in_flight) { --in_flight; tx_done(0); } }   // the host took one IN transfer
};
