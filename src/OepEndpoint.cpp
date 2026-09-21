#include "OepEndpoint.h"

#include <string.h>

namespace oep {

bool Endpoint::addService(Service &service) {
  if (service_count_ >= kMaxServices) return false;
  service.function = static_cast<uint16_t>(service_count_ + 1);
  services_[service_count_++] = &service;
  return true;
}

void Endpoint::poll() {
  while (stream_.available()) {
    if (!reader_.push(static_cast<uint8_t>(stream_.read()))) continue;
    handleMessage(reader_.message(), reader_.length());
    reader_.consume();
  }
}

bool Endpoint::idleFor(uint32_t milliseconds) const {
  return last_request_millis_ &&
         static_cast<uint32_t>(millis() - last_request_millis_) >= milliseconds;
}

void Endpoint::abandonAll() {
  for (size_t i = 0; i < service_count_; ++i) services_[i]->abandon();
  last_request_millis_ = 0;
}

void Endpoint::handleMessage(const uint8_t *message, size_t length) {
  if (oep_v0_message_role(message, length) != OEP_V0_ROLE_REQUEST) return;  // unknown/other roles: drop
  struct oep_v0_request_header header;
  size_t header_length = 0;
  if (!oep_v0_request_header_unpack(message, length, &header, &header_length)) return;
  last_request_millis_ = millis();
  ++requests_;
  const uint8_t *payload = message + header_length;
  const size_t payload_length = length - header_length;
  uint8_t *out = tx_ + kResultHeader;
  size_t capacity = tx_capacity_ - kResultHeader;
  if (capacity > static_cast<size_t>(limits_.max_frame) - kResultHeader)
    capacity = limits_.max_frame - kResultHeader;

  Result result;
  if (header.function == OEP_V0_DEF_CORE_FUNCTION) {
    result = handleCore(header.operation, payload, payload_length, out, capacity);
  } else if (header.function == 0 || header.function > service_count_) {
    result = rejected(OEP_V0_REJECT_UNKNOWN_FUNCTION);
  } else {
    result = services_[header.function - 1]->handle(header.operation, payload, payload_length, out, capacity);
  }
  if (result.length > capacity) result = failed(0);
  sendResult(header.correlation, result);
}

Result Endpoint::handleCore(uint8_t operation, const uint8_t *payload, size_t length,
                            uint8_t *out, size_t capacity) {
  switch (operation) {
    case OEP_V0_CORE_OP_CONFIRM: {
      struct oep_v0_core_confirm_request request;
      if (!oep_v0_core_confirm_request_unpack(payload, length, &request))
        return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (memcmp(request.magic, OEP_V0_CONST_CONFIRM_REQUEST_MAGIC, 4) != 0 ||
          request.min_revision > OEP_V0_PROTOCOL_REVISION ||
          request.max_revision < OEP_V0_PROTOCOL_REVISION)
        return rejected(OEP_V0_REJECT_UNAVAILABLE);
      struct oep_v0_core_confirm_result result;
      memcpy(result.magic, OEP_V0_CONST_CONFIRM_RESULT_MAGIC, 4);
      result.revision = OEP_V0_PROTOCOL_REVISION;
      result.max_frame = limits_.max_frame;
      result.window_bytes = limits_.window_bytes;
      result.max_inflight = limits_.max_inflight;
      result.flags = 0;
      return completed(oep_v0_core_confirm_result_pack(&result, out, capacity));
    }
    case OEP_V0_CORE_OP_LIST: {
      struct oep_v0_core_list_request request;
      if (!oep_v0_core_list_request_unpack(payload, length, &request))
        return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      struct oep_v0_core_list_result result;
      result.total = static_cast<uint8_t>(service_count_ + 1);
      result.entries_count = 0;
      const size_t entry_size = 8;
      for (size_t index = request.first; index <= service_count_ && result.entries_count < 16; ++index) {
        if ((result.entries_count + 1) * entry_size + 2 > capacity) break;
        struct oep_v0_offered_function &entry = result.entries[result.entries_count++];
        if (index == 0) {
          entry.function = OEP_V0_DEF_CORE_FUNCTION;
          entry.owner = OEP_V0_DEF_CORE_OWNER;
          entry.id = OEP_V0_DEF_CORE_ID;
          entry.revision = OEP_V0_DEF_CORE_REVISION;
          entry.flags = 0;
        } else {
          const Service *service = services_[index - 1];
          entry.function = service->function;
          entry.owner = service->owner();
          entry.id = service->id();
          entry.revision = service->revision();
          entry.flags = service->flags();
        }
      }
      return completed(oep_v0_core_list_result_pack(&result, out, capacity));
    }
    case OEP_V0_CORE_OP_DESCRIBE: {
      struct oep_v0_core_describe_request request;
      if (!oep_v0_core_describe_request_unpack(payload, length, &request))
        return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (request.function == OEP_V0_DEF_CORE_FUNCTION) return completed(0);
      if (request.function > service_count_) return rejected(OEP_V0_REJECT_UNKNOWN_FUNCTION);
      return completed(services_[request.function - 1]->describe(request.first, out, capacity));
    }
    case OEP_V0_CORE_OP_PING: {
      if (length > capacity) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      memcpy(out, payload, length);
      return completed(length);
    }
    case OEP_V0_CORE_OP_PLAN_APPLY:
    case OEP_V0_CORE_OP_PLAN_RELEASE:
    case OEP_V0_CORE_OP_STOP:
      return rejected(OEP_V0_REJECT_UNAVAILABLE);  // leases and activities arrive with the fixture services
    default:
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

void Endpoint::sendResult(uint16_t correlation, Result result) {
  struct oep_v0_result_header header = {correlation, result.resolution, result.detail};
  const size_t header_length = oep_v0_result_header_pack(&header, tx_, kResultHeader);
  if (header_length != kResultHeader) return;
  writeFrame(stream_, tx_, kResultHeader + result.length);
}

}  // namespace oep
