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

uint32_t get32(const uint8_t* input) {
  return input[0] | static_cast<uint32_t>(input[1]) << 8 |
      static_cast<uint32_t>(input[2]) << 16 |
      static_cast<uint32_t>(input[3]) << 24;
}

void put32(uint8_t* output, uint32_t value) {
  output[0] = value;
  output[1] = value >> 8;
  output[2] = value >> 16;
  output[3] = value >> 24;
}

void put64(uint8_t* output, uint64_t value) {
  for (size_t index = 0; index < 8; ++index) output[index] = value >> (index * 8);
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
  last_request_millis_ = millis();
  if (length < 4) return;
  if (message[0] == kRoleCoreRequest) {
    handleCoreRequest(message, length);
  } else if (message[0] == kRoleFunctionRequest) {
    handleFunctionRequest(message, length);
  }
}

bool Endpoint::idleFor(uint32_t milliseconds) const {
  return last_request_millis_ &&
      static_cast<uint32_t>(millis() - last_request_millis_) >= milliseconds;
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
      response[5] = (target_ ? 1 : 0) + (memory_ ? 1 : 0) + (info_ ? 1 : 0) +
          (capabilities_ ? 1 : 0) + (configuration_ ? 1 : 0) +
          (flash_ ? 1 : 0) + (gpio_ ? 1 : 0) + (i2c_ ? 1 : 0);
      if (uart_) ++response[5];
      response_length = 6;
      if (info_) {
        put16(response + response_length, ProbeInfo);
        response[response_length + 2] = 1;
        response[response_length + 3] = 0;
        response_length += 4;
      }
      if (capabilities_) {
        put16(response + response_length, ProbeCapabilities);
        response[response_length + 2] = 1;
        response[response_length + 3] = 0;
        response_length += 4;
      }
      if (configuration_) {
        put16(response + response_length, ProbeConfiguration);
        response[response_length + 2] = 1;
        response[response_length + 3] = 0;
        response_length += 4;
      }
      if (target_) {
        put16(response + response_length, TargetControl);
        response[response_length + 2] = 1;
        response[response_length + 3] = 0;
        response_length += 4;
      }
      if (memory_) {
        put16(response + response_length, TargetMemory);
        response[response_length + 2] = 1;
        response[response_length + 3] = 0;
        response_length += 4;
      }
      if (flash_) {
        put16(response + response_length, TargetFlash);
        response[response_length + 2] = 2;
        response[response_length + 3] = 0;
        response_length += 4;
      }
      if (gpio_) {
        put16(response + response_length, FixtureGpio);
        response[response_length + 2] = 1;
        response[response_length + 3] = 0;
        response_length += 4;
      }
      if (uart_) {
        put16(response + response_length, FixtureUart);
        response[response_length + 2] = 1;
        response[response_length + 3] = 0;
        response_length += 4;
      }
      if (i2c_) {
        put16(response + response_length, FixtureI2c);
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

  if (target == ProbeInfo) {
    if (!info_) response[6] = kRejectUnavailable;
    else if (operation != ProbeInfoGet || length != 6) response[6] =
        operation == ProbeInfoGet ? kRejectPayload : kRejectOperation;
    else {
      ProbeInfoStatus status;
      const BackendResult result = info_->getInfo(status);
      response[1] = kResolutionCompleted;
      response[6] = result == BackendResult::Success ? kOutcomeSuccess : kOutcomeFailed;
      if (result == BackendResult::Success) {
        put32(response + 7, status.profile_id);
        put32(response + 11, status.firmware_revision);
        put64(response + 15, status.reserved_pin_mask);
        put64(response + 23, status.fixture_pin_mask);
        response_length = 31;
      }
    }
    sendMessage(response, response_length);
    return;
  }

  if (target == ProbeCapabilities) {
    if (!capabilities_) {
      response[6] = kRejectUnavailable;
    } else if (operation == ProbeCapabilitiesGetSummary && length == 6) {
      ProbeCapabilitiesSummary summary;
      const BackendResult result = capabilities_->getSummary(summary);
      response[1] = kResolutionCompleted;
      response[6] = result == BackendResult::Success ?
          kOutcomeSuccess : kOutcomeFailed;
      if (result == BackendResult::Success) {
        response[7] = summary.revision;
        response[8] = summary.channel_count;
        response[9] = summary.group_count;
        response[10] = summary.voltage_domain_count;
        response_length = 11;
      }
    } else if (operation == ProbeCapabilitiesGetChannel && length == 7) {
      ProbeChannelCapability channel;
      const BackendResult result = capabilities_->getChannel(message[6], channel);
      if (result == BackendResult::Unavailable) {
        response[6] = kRejectUnavailable;
      } else {
        response[1] = kResolutionCompleted;
        response[6] = result == BackendResult::Success ?
            kOutcomeSuccess : kOutcomeFailed;
        if (result == BackendResult::Success) {
          put16(response + 7, channel.id);
          response[9] = channel.flags;
          response[10] = channel.voltage_domain_mask;
          put64(response + 11, channel.function_mask);
          response_length = 19;
        }
      }
    } else if (operation == ProbeCapabilitiesGetGroup && length == 7) {
      ProbeGroupCapability group;
      const BackendResult result = capabilities_->getGroup(message[6], group);
      if (result == BackendResult::Unavailable) {
        response[6] = kRejectUnavailable;
      } else {
        response[1] = kResolutionCompleted;
        response[6] = result == BackendResult::Success ?
            kOutcomeSuccess : kOutcomeFailed;
        if (result == BackendResult::Success) {
          put16(response + 7, group.id);
          response[9] = group.kind;
          response[10] = group.instance;
          put64(response + 11, group.role_mask);
          put64(response + 19, group.exclusive_group_mask);
          response_length = 27;
        }
      }
    } else if (operation == ProbeCapabilitiesGetVoltageDomain && length == 7) {
      ProbeVoltageDomainCapability domain;
      const BackendResult result = capabilities_->getVoltageDomain(
          message[6], domain);
      if (result == BackendResult::Unavailable) {
        response[6] = kRejectUnavailable;
      } else {
        response[1] = kResolutionCompleted;
        response[6] = result == BackendResult::Success ?
            kOutcomeSuccess : kOutcomeFailed;
        if (result == BackendResult::Success) {
          response[7] = domain.id;
          response[8] = domain.flags;
          put16(response + 9, domain.nominal_mv);
          put16(response + 11, domain.input_max_mv);
          response_length = 13;
        }
      }
    } else if (operation == ProbeCapabilitiesGetGroupRole && length == 8) {
      ProbeGroupRoleCapability role;
      const BackendResult result = capabilities_->getGroupRole(
          message[6], message[7], role);
      if (result == BackendResult::Unavailable) {
        response[6] = kRejectUnavailable;
      } else {
        response[1] = kResolutionCompleted;
        response[6] = result == BackendResult::Success ?
            kOutcomeSuccess : kOutcomeFailed;
        if (result == BackendResult::Success) {
          put16(response + 7, role.group_id);
          response[9] = role.role_id;
          response[10] = role.function;
          response_length = 11;
        }
      }
    } else {
      const bool known = operation >= ProbeCapabilitiesGetSummary &&
          operation <= ProbeCapabilitiesGetGroupRole;
      response[6] = known ? kRejectPayload : kRejectOperation;
    }
    sendMessage(response, response_length);
    return;
  }

  if (target == ProbeConfiguration) {
    if (!configuration_) {
      response[6] = kRejectUnavailable;
    } else if (operation == ProbeConfigurationApply) {
      // Revision 1: {group:u16, function:u8, channel:u16}; revision 2 adds
      // the stable role id between group and function.
      const uint8_t revision = length >= 7 ? message[6] : 0;
      const size_t role_size = revision == 1 ? 5 : revision == 2 ? 6 : 0;
      if (length < 8 || !role_size || !message[7] ||
          length != 8 + static_cast<size_t>(message[7]) * role_size) {
        response[6] = kRejectPayload;
      } else {
        constexpr size_t kMaximumConfigurationRoles =
            (kMaximumMessage - 8) / 5;
        ProbeConfigurationRole roles[kMaximumConfigurationRoles];
        const uint8_t count = message[7];
        for (uint8_t index = 0; index < count; ++index) {
          const uint8_t* encoded = message + 8 + index * role_size;
          roles[index].group_id = encoded[0] |
              static_cast<uint16_t>(encoded[1]) << 8;
          roles[index].role_id = revision == 2 ? encoded[2] : 0;
          roles[index].function = encoded[revision == 2 ? 3 : 2];
          const uint8_t channel_offset = revision == 2 ? 4 : 3;
          roles[index].channel_id = encoded[channel_offset] |
              static_cast<uint16_t>(encoded[channel_offset + 1]) << 8;
        }
        uint32_t lease_id = 0;
        const BackendResult result = configuration_->apply(roles, count, lease_id);
        if (result == BackendResult::Unavailable) {
          response[6] = kRejectUnavailable;
        } else {
          response[1] = kResolutionCompleted;
          response[6] = result == BackendResult::Success ?
              kOutcomeSuccess : kOutcomeFailed;
          if (result == BackendResult::Success) {
            put32(response + 7, lease_id);
            response_length = 11;
          }
        }
      }
    } else if (operation == ProbeConfigurationRelease && length == 10) {
      const BackendResult result = configuration_->release(get32(message + 6));
      if (result == BackendResult::Unavailable) {
        response[6] = kRejectUnavailable;
      } else {
        response[1] = kResolutionCompleted;
        response[6] = result == BackendResult::Success ?
            kOutcomeSuccess : kOutcomeFailed;
      }
    } else {
      response[6] = operation == ProbeConfigurationApply ||
          operation == ProbeConfigurationRelease ? kRejectPayload : kRejectOperation;
    }
    sendMessage(response, response_length);
    return;
  }

  if (target == TargetMemory) {
    if (!memory_) {
      response[6] = kRejectUnavailable;
    } else if (operation != TargetReadMemory) {
      response[6] = kRejectOperation;
    } else if (length != 11 || (message[6] & 3) || !message[10] ||
               (message[10] & 3) || message[10] > 88) {
      response[6] = kRejectPayload;
    } else {
      const size_t requested = message[10];
      const BackendResult result = memory_->readMemory(
          get32(message + 6), response + 7, requested);
      if (result == BackendResult::Unavailable) {
        response[6] = kRejectUnavailable;
      } else {
        response[1] = kResolutionCompleted;
        response[6] = result == BackendResult::Success ?
            kOutcomeSuccess : kOutcomeFailed;
        if (result == BackendResult::Success) response_length += requested;
      }
    }
    sendMessage(response, response_length);
    return;
  }
  if (target == TargetFlash) {
    if (!flash_) {
      response[6] = kRejectUnavailable;
    } else {
      uint8_t diagnostic = 0;
      BackendResult result = BackendResult::Unavailable;
      if (operation == TargetProgramPage64 || operation == TargetStagePage64) {
        if (length != 74 || (message[6] & 63)) {
          response[6] = kRejectPayload;
          sendMessage(response, response_length);
          return;
        }
        result = operation == TargetProgramPage64
            ? flash_->programPage64(get32(message + 6), message + 10, diagnostic)
            : flash_->stagePage64(get32(message + 6), message + 10, diagnostic);
      } else if (operation == TargetCommitPage256) {
        if (length != 10 || (message[6] & 255)) {
          response[6] = kRejectPayload;
          sendMessage(response, response_length);
          return;
        }
        result = flash_->commitPage256(get32(message + 6), diagnostic);
      } else {
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
        if (result == BackendResult::Failed) {
          response[7] = diagnostic;
          response_length = 8;
        }
      }
    }
    sendMessage(response, response_length);
    return;
  }
  if (target == FixtureGpio) {
    if (!gpio_) {
      response[6] = kRejectUnavailable;
    } else if (operation == FixtureReadDigital && length == 7) {
      uint8_t value = 0;
      const BackendResult result = gpio_->readDigital(message[6], value);
      if (result == BackendResult::Unavailable) {
        response[6] = kRejectUnavailable;
      } else {
        response[1] = kResolutionCompleted;
        response[6] = result == BackendResult::Success ?
            kOutcomeSuccess : kOutcomeFailed;
        if (result == BackendResult::Success) {
          response[7] = value;
          response_length = 8;
        }
      }
    } else if (operation == FixtureReadDigitalBank && length == 6) {
      uint64_t available = 0, values = 0;
      const BackendResult result = gpio_->readDigitalBank(available, values);
      response[1] = kResolutionCompleted;
      response[6] = result == BackendResult::Success ?
          kOutcomeSuccess : kOutcomeFailed;
      if (result == BackendResult::Success) {
        put64(response + 7, available);
        put64(response + 15, values);
        response_length = 23;
      }
    } else if (operation == FixtureConfigureDigital && length == 8) {
      const BackendResult result = gpio_->configureDigital(message[6], message[7]);
      if (result == BackendResult::Unavailable) {
        response[6] = kRejectUnavailable;
      } else {
        response[1] = kResolutionCompleted;
        response[6] = result == BackendResult::Success ?
            kOutcomeSuccess : kOutcomeFailed;
      }
    } else {
      response[6] = operation == FixtureReadDigital ||
          operation == FixtureReadDigitalBank ||
          operation == FixtureConfigureDigital ? kRejectPayload : kRejectOperation;
    }
    sendMessage(response, response_length);
    return;
  }
  if (target == FixtureUart) {
    if (!uart_) {
      response[6] = kRejectUnavailable;
      sendMessage(response, response_length);
      return;
    }
    BackendResult result = BackendResult::Unavailable;
    if (operation == FixtureUartConfigure) {
      if (length != 10) {
        response[6] = kRejectPayload;
        sendMessage(response, response_length);
        return;
      }
      uint32_t actual = 0;
      result = uart_->configure(get32(message + 6), actual);
      if (result == BackendResult::Success) {
        put32(response + 7, actual);
        response_length = 11;
      }
    } else if (operation == FixtureUartWrite) {
      if (length < 7 || length > 70) {
        response[6] = kRejectPayload;
        sendMessage(response, response_length);
        return;
      }
      size_t written = 0;
      result = uart_->writeBytes(message + 6, length - 6, written);
      if (result == BackendResult::Success) {
        response[7] = written;
        response_length = 8;
      }
    } else if (operation == FixtureUartReadAvailable) {
      if (length != 7 || !message[6] || message[6] > 80) {
        response[6] = kRejectPayload;
        sendMessage(response, response_length);
        return;
      }
      size_t received = 0;
      result = uart_->readAvailable(response + 8, message[6], received);
      if (result == BackendResult::Success) {
        response[7] = received;
        response_length = 8 + received;
      }
    } else {
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
    }
    sendMessage(response, response_length);
    return;
  }
  if (target == FixtureI2c) {
    if (!i2c_) {
      response[6] = kRejectUnavailable;
    } else if (operation != FixtureI2cGetStatus || length != 6) {
      response[6] = operation == FixtureI2cGetStatus ? kRejectPayload :
          kRejectOperation;
    } else {
      FixtureI2cStatus status;
      const BackendResult result = i2c_->getStatus(status);
      if (result == BackendResult::Unavailable) {
        response[6] = kRejectUnavailable;
      } else {
        response[1] = kResolutionCompleted;
        response[6] = result == BackendResult::Success ?
            kOutcomeSuccess : kOutcomeFailed;
        if (result == BackendResult::Success) {
          response[7] = status.flags;
          response[8] = status.last_rx_length;
          put16(response + 9, status.rx_transactions);
          put16(response + 11, status.request_transactions);
          put32(response + 13, status.frequency_hz);
          response_length = 17;
        }
      }
    }
    sendMessage(response, response_length);
    return;
  }
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

BackendResult FixtureGpioBackend::readDigitalBank(
    uint64_t& available, uint64_t& values) {
  available = 0;
  values = 0;
  for (uint8_t pin = 0; pin < 64; ++pin) {
    uint8_t value = 0;
    const BackendResult result = readDigital(pin, value);
    if (result == BackendResult::Failed) return result;
    if (result == BackendResult::Success) {
      available |= uint64_t(1) << pin;
      if (value) values |= uint64_t(1) << pin;
    }
  }
  return BackendResult::Success;
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
