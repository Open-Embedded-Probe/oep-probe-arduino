#pragma once

#include "OepPrototype.h"

namespace oep::prototype {

class StaticProbeInfo final : public ProbeInfoBackend {
 public:
  explicit StaticProbeInfo(ProbeInfoStatus status) : status_(status) {}
  BackendResult getInfo(ProbeInfoStatus& status) override {
    status = status_;
    return BackendResult::Success;
  }

 private:
  ProbeInfoStatus status_;
};

class StaticPinMatrix final : public PinMatrixBackend {
 public:
  StaticPinMatrix(const PinMatrixEntry* entries, size_t count) : entries_(entries), count_(count) {}
  BackendResult getPin(uint8_t signal, PinMatrixEntry& entry) override {
    for (size_t i = 0; i < count_; ++i) if (entries_[i].signal == signal) { entry = entries_[i]; return BackendResult::Success; }
    return BackendResult::Unavailable;
  }
 private:
  const PinMatrixEntry* entries_; size_t count_;
};

}  // namespace oep::prototype
