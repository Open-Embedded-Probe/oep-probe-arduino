// probe.identity (owner 0, id 1): static facts about this probe build.
#pragma once

#include "OepService.h"

namespace oep {

class ProbeIdentity final : public Service {
 public:
  explicit ProbeIdentity(struct oep_v0_probe_identity_get_result identity) : identity_(identity) {}
  uint16_t owner() const override { return OEP_V0_DEF_PROBE_IDENTITY_OWNER; }
  uint16_t id() const override { return OEP_V0_DEF_PROBE_IDENTITY_ID; }
  uint8_t revision() const override { return OEP_V0_DEF_PROBE_IDENTITY_REVISION; }
  Result handle(uint8_t operation, const uint8_t *payload, size_t length,
                uint8_t *out, size_t capacity) override {
    if (operation != OEP_V0_PROBE_IDENTITY_OP_GET) return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
    struct oep_v0_probe_identity_get_request request;
    if (!oep_v0_probe_identity_get_request_unpack(payload, length, &request))
      return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
    return completed(oep_v0_probe_identity_get_result_pack(&identity_, out, capacity));
  }

 private:
  struct oep_v0_probe_identity_get_result identity_;
};

}  // namespace oep
