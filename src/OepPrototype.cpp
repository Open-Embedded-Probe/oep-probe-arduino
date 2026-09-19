#include "OepPrototype.h"

#include <string.h>

namespace oep::prototype {
namespace {

uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xffff;
  while (length--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

size_t cobsDecode(uint8_t* data, size_t length) {
  size_t read = 0, write = 0;
  while (read < length) {
    const uint8_t code = data[read++];
    if (!code || read + code - 1 > length) return 0;
    for (uint8_t index = 1; index < code; ++index) data[write++] = data[read++];
    if (code != 0xff && read < length) data[write++] = 0;
  }
  return write;
}

size_t cobsEncode(const uint8_t* input, size_t length, uint8_t* output,
                  size_t capacity) {
  if (!capacity) return 0;
  size_t code_index = 0, write = 1;
  uint8_t code = 1;
  for (size_t index = 0; index < length; ++index) {
    if (!input[index]) {
      if (write >= capacity) return 0;
      output[code_index] = code;
      code_index = write++;
      code = 1;
    } else {
      if (write >= capacity) return 0;
      output[write++] = input[index];
      if (++code == 0xff) {
        if (write >= capacity) return 0;
        output[code_index] = code;
        code_index = write++;
        code = 1;
      }
    }
  }
  output[code_index] = code;
  return write;
}

void put16(uint8_t* output, uint16_t value) {
  output[0] = static_cast<uint8_t>(value);
  output[1] = static_cast<uint8_t>(value >> 8);
}

constexpr uint8_t kRoleCoreRequest = 0x01;
constexpr uint8_t kRoleCoreResult = 0x81;
constexpr uint8_t kRoleFunctionRequest = 0x10;
constexpr uint8_t kRoleFunctionResult = 0x90;

constexpr uint8_t kResolutionRejected = 0;
constexpr uint8_t kResolutionCompleted = 1;
constexpr uint8_t kRejectTarget = 1;
constexpr uint8_t kRejectOperation = 2;
constexpr uint8_t kRejectPayload = 3;
constexpr uint8_t kRejectUnavailable = 4;
constexpr uint8_t kOutcomeSuccess = 0;
constexpr uint8_t kOutcomeFailed = 1;

}  // namespace

void Endpoint::poll() {
  while (stream_.available()) {
    const uint8_t byte = static_cast<uint8_t>(stream_.read());
    if (byte) {
      if (discard_) continue;
      if (encoded_length_ == sizeof(encoded_)) {
        encoded_length_ = 0;
        discard_ = true;
      } else {
        encoded_[encoded_length_++] = byte;
      }
      continue;
    }
    if (discard_) {
      discard_ = false;
      encoded_length_ = 0;
    } else if (encoded_length_) {
      consumeFrame();
      encoded_length_ = 0;
    }
  }
}

void Endpoint::consumeFrame() {
  const size_t raw_length = cobsDecode(encoded_, encoded_length_);
  if (raw_length < 3 || encoded_[0] != raw_length - 3) return;
  const uint16_t expected = encoded_[raw_length - 2] |
      static_cast<uint16_t>(encoded_[raw_length - 1]) << 8;
  if (crc16(encoded_, raw_length - 2) != expected) return;
  handleMessage(encoded_ + 1, encoded_[0]);
}

void Endpoint::handleMessage(uint8_t* message, size_t length) {
  if (length < 4) return;
  if (message[0] == kRoleCoreRequest) {
    handleCoreRequest(message, length);
  } else if (message[0] == kRoleFunctionRequest) {
    handleFunctionRequest(message, length);
  }
}

void Endpoint::handleCoreRequest(uint8_t* message, size_t length) {
  const uint8_t operation = message[1];
  const uint8_t correlation_low = message[2];
  const uint8_t correlation_high = message[3];
  uint8_t response[kMaximumMessage] = {kRoleCoreResult, operation,
                                       correlation_low, correlation_high};
  size_t response_length = 5;

  if (operation == 0x01) {
    if (length != 10 || memcmp(message + 4, "OEP?", 4) ||
        message[8] > 1 || message[9] < 1) {
      response[4] = 2;  // rejected payload / incompatible range
    } else {
      memcpy(response + 4, "OEP!", 4);
      response[8] = 0;
      response[9] = 1;
      put16(response + 10, kMaximumMessage);
      response[12] = 1;
      response[13] = 1;
      response_length = 14;
    }
  } else if (operation == 0x02) {
    if (length != 5 || message[4] != 0) {
      response[4] = 2;
    } else {
      response[4] = 0;
      response[5] = target_ ? 1 : 0;
      response_length = 6;
      if (target_) {
        put16(response + response_length, TargetControl);
        response[response_length + 2] = 1;
        response[response_length + 3] = 0;
        response_length += 4;
      }
    }
  } else {
    response[4] = 1;
  }
  sendMessage(response, response_length);
}

void Endpoint::handleFunctionRequest(uint8_t* message, size_t length) {
  if (length < 6) return;
  const uint8_t operation = message[1];
  const uint16_t target = message[4] |
      static_cast<uint16_t>(message[5]) << 8;
  uint8_t response[kMaximumMessage] = {
      kRoleFunctionResult, kResolutionRejected, message[2], message[3],
      message[4], message[5], kRejectTarget};
  size_t response_length = 7;

  if (target != TargetControl) {
    sendMessage(response, response_length);
    return;
  }
  if (!target_) {
    response[6] = kRejectUnavailable;
    sendMessage(response, response_length);
    return;
  }
  if (length != 6) {
    response[6] = kRejectPayload;
    sendMessage(response, response_length);
    return;
  }

  BackendResult result;
  TargetStatus status;
  switch (operation) {
    case TargetGetStatus:
      result = target_->getStatus(status);
      break;
    case TargetNormalizeUser:
      result = target_->normalizeUser();
      break;
    case TargetEnterProductBootloader:
      result = target_->enterProductBootloader();
      break;
    default:
      response[6] = kRejectOperation;
      sendMessage(response, response_length);
      return;
  }

  if (result == BackendResult::Unavailable) {
    response[6] = kRejectUnavailable;
  } else {
    response[1] = kResolutionCompleted;
    response[6] = result == BackendResult::Success ?
        kOutcomeSuccess : kOutcomeFailed;
    if (operation == TargetGetStatus && result == BackendResult::Success) {
      response[7] = status.flags;
      response[8] = status.start_mode;
      response[9] = status.boot_status;
      response_length = 10;
    }
  }
  sendMessage(response, response_length);
}

void Endpoint::sendMessage(const uint8_t* message, size_t length) {
  if (length > kMaximumMessage) return;
  uint8_t raw[kMaximumMessage + 3];
  uint8_t wire[kMaximumWire];
  raw[0] = static_cast<uint8_t>(length);
  memcpy(raw + 1, message, length);
  const uint16_t crc = crc16(raw, length + 1);
  put16(raw + length + 1, crc);
  const size_t wire_length = cobsEncode(raw, length + 3, wire, sizeof(wire) - 1);
  if (!wire_length) return;
  wire[wire_length] = 0;
  stream_.write(wire, wire_length + 1);
}

}  // namespace oep::prototype
