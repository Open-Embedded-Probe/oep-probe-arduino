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

}  // namespace oep::prototype
