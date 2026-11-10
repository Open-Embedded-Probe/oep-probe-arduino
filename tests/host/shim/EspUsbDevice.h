// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Host-test stand-in for the EspUsbDevice (3.x) vendor class DirectBulkStream uses: IN transfers complete when the test
// says.
#pragma once
#include <functional>
#include <vector>
#include <stdint.h>
#include <stddef.h>
typedef int esp_err_t;
#define ESP_OK 0
enum class EspUsbBulkRxMode : uint8_t { Stream, Callback, Borrowed, Disabled };
enum class EspUsbTransferTermination : uint8_t { Continue, EndWithShortPacket };
enum class EspUsbTransferError : uint8_t { None = 0, NotMounted, Busy };
struct EspUsbTransferSubmission {
  EspUsbTransferError error = EspUsbTransferError::None;
  explicit operator bool() const { return error == EspUsbTransferError::None; }
};
struct EspUsbTransferResult { const void *buffer = nullptr; size_t actualBytes = 0; };
struct EspUsbBulkStreamConfig { size_t rxCapacity = 512; EspUsbBulkRxMode rxMode = EspUsbBulkRxMode::Stream; };
using EspUsbVendorConfig = EspUsbBulkStreamConfig;
class EspUsbDeviceVendor {
 public:
  std::function<void(const uint8_t *, size_t)> rx;
  std::function<void(const EspUsbTransferResult &)> tx_done;
  std::vector<std::vector<uint8_t>> sent;   // transfers written, in order
  std::vector<bool> ended;                  // each one's termination: EndWithShortPacket
  EspUsbBulkStreamConfig config;
  int in_flight = 0;
  int max_in_flight = 1;                    // what the library takes at once (3.0.0: one; more answer Busy)
  bool mounted_ = true;
  static bool directWriteSupported() { return true; }
  const EspUsbBulkStreamConfig &configuration() const { return config; }
  esp_err_t configure(const EspUsbBulkStreamConfig &c) { config = c; return ESP_OK; }
  esp_err_t onRxData(std::function<void(const uint8_t *, size_t)> f) { rx = f; return ESP_OK; }
  esp_err_t onDirectTxComplete(std::function<void(const EspUsbTransferResult &)> f) { tx_done = f; return ESP_OK; }
  EspUsbTransferSubmission writeDirect(const void *p, size_t n, EspUsbTransferTermination t) {
    const uint8_t *b = static_cast<const uint8_t *>(p);
    if (in_flight >= max_in_flight) { EspUsbTransferSubmission busy; busy.error = EspUsbTransferError::Busy; return busy; }
    sent.emplace_back(b, b + n);
    ended.push_back(t == EspUsbTransferTermination::EndWithShortPacket);
    ++in_flight;
    return {};
  }
  bool mounted() const { return mounted_; }
  void complete() { if (in_flight) { --in_flight; tx_done(EspUsbTransferResult{}); } }   // the host took one IN transfer
};
