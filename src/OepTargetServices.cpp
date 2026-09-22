#include "OepTargetServices.h"

#include <string.h>

namespace oep {

uint32_t crc32Ieee(uint32_t crc, const uint8_t *data, size_t length) {
  crc = ~crc;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

// ---- target.control ------------------------------------------------------
Result TargetControl::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (operation) {
    case OEP_V0_TARGET_CONTROL_OP_STATUS: {
      struct oep_v0_target_control_status_request request;
      if (!oep_v0_target_control_status_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      // flags: bit0 attached, bit1 halted (probe bookkeeping), bit2 DMSTATUS.allhalted read live.
      struct oep_v0_target_control_status_result result = {
          static_cast<uint8_t>((dm_.attached() ? 1 : 0) | (dm_.halted() ? 2 : 0) | (dm_.readHalted() ? 4 : 0)),
          dm_.resetDiag(), dm_.lastCmderr()};
      return completed(oep_v0_target_control_status_result_pack(&result, out, capacity));
    }
    case OEP_V0_TARGET_CONTROL_OP_HALT: {
      struct oep_v0_target_control_halt_request request;
      if (!oep_v0_target_control_halt_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm_.attach()) return rejected(OEP_V0_REJECT_UNAVAILABLE);  // no target answered
      return dm_.halt() ? completed() : failed();
    }
    case OEP_V0_TARGET_CONTROL_OP_RESUME: {
      struct oep_v0_target_control_resume_request request;
      if (!oep_v0_target_control_resume_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm_.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      const bool ok = dm_.resume();
      dm_.detach();
      return ok ? completed() : failed();
    }
    case OEP_V0_TARGET_CONTROL_OP_RESET: {
      struct oep_v0_target_control_reset_request request;
      if (!oep_v0_target_control_reset_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (request.mode != 0) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);  // 0 = system reset, target runs
      if (!dm_.attach()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      dm_.reset();
      return completed();
    }
    case OEP_V0_TARGET_CONTROL_OP_READ_DMI: {
      struct oep_v0_target_control_read_dmi_request request;
      if (!oep_v0_target_control_read_dmi_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      struct oep_v0_target_control_read_dmi_result result = {0};
      if (!dm_.readDmi(request.address, result.value)) return failed();
      return completed(oep_v0_target_control_read_dmi_result_pack(&result, out, capacity));
    }
    case OEP_V0_TARGET_CONTROL_OP_READ_REGISTER: {
      struct oep_v0_target_control_read_register_request request;
      if (!oep_v0_target_control_read_register_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm_.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      struct oep_v0_target_control_read_register_result result = {0};
      if (!dm_.readRegister(request.regno, result.value)) return failed();
      return completed(oep_v0_target_control_read_register_result_pack(&result, out, capacity));
    }
    default:
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

void TargetControl::abandon() {
  // The host vanished mid-session: never leave the DUT halted.
  if (dm_.attached()) dm_.reset();
}

// ---- target.memory -------------------------------------------------------
Result TargetMemory::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  switch (operation) {
    case OEP_V0_TARGET_MEMORY_OP_READ: {
      struct oep_v0_target_memory_read_request request;
      if (!oep_v0_target_memory_read_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if ((request.address & 3) || !request.length || (request.length & 3) || request.length > capacity)
        return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm_.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      uint32_t *words = reinterpret_cast<uint32_t *>(out);  // out is 4-byte aligned (buffer + 5? no) -> copy below
      uint32_t local[256];
      const size_t count = request.length / 4;
      if (count > sizeof local / sizeof local[0]) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      (void)words;
      if (!dm_.readWords(request.address, local, count)) return failed();
      for (size_t i = 0; i < count; ++i) {
        out[4 * i] = local[i]; out[4 * i + 1] = local[i] >> 8; out[4 * i + 2] = local[i] >> 16; out[4 * i + 3] = local[i] >> 24;
      }
      return completed(request.length);
    }
    case OEP_V0_TARGET_MEMORY_OP_WRITE: {
      struct oep_v0_target_memory_write_request request;
      if (!oep_v0_target_memory_write_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if ((request.address & 3) || !request.data_length || (request.data_length & 3)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm_.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      uint16_t written = 0;
      for (; written < request.data_length; written += 4) {
        const uint8_t *p = request.data + written;
        const uint32_t value = uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
        if (!dm_.writeWord(request.address + written, value)) break;
      }
      struct oep_v0_target_memory_write_result result = {written};
      const size_t n = oep_v0_target_memory_write_result_pack(&result, out, capacity);
      return written == request.data_length ? completed(n) : failed(n);
    }
    default:
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

// ---- target.flash --------------------------------------------------------
bool TargetFlash::inRange(uint32_t address, uint32_t bytes) const {
  const FlashGeometry &g = dm_.geometry();
  return address >= g.base && bytes <= g.size && address - g.base <= g.size - bytes;
}

Result TargetFlash::handle(uint8_t operation, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  const FlashGeometry &g = dm_.geometry();
  switch (operation) {
    case OEP_V0_TARGET_FLASH_OP_GEOMETRY: {
      struct oep_v0_target_flash_geometry_request request;
      if (!oep_v0_target_flash_geometry_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      struct oep_v0_target_flash_geometry_result result = {g.base, g.size, g.page, g.program_unit};
      return completed(oep_v0_target_flash_geometry_result_pack(&result, out, capacity));
    }
    case OEP_V0_TARGET_FLASH_OP_ERASE_PAGE: {
      struct oep_v0_target_flash_erase_page_request request;
      if (!oep_v0_target_flash_erase_page_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (request.address % g.page || !inRange(request.address, g.page)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm_.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      if (!dm_.flashUnlock()) return failed();
      return dm_.flashErasePage(request.address) ? completed() : failed();
    }
    case OEP_V0_TARGET_FLASH_OP_PROGRAM_PAGE: {
      struct oep_v0_target_flash_program_page_request request;
      if (!oep_v0_target_flash_program_page_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (request.address % g.page || request.data_length != g.page || !inRange(request.address, g.page) || g.page > sizeof page_)
        return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm_.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      // One transaction: erase -> autoexec program -> read back -> compare (E157).
      if (!dm_.flashUnlock() || !dm_.flashErasePage(request.address) ||
          !dm_.flashProgramPage(request.address, request.data)) return failed();
      if (!dm_.readWords(request.address, page_, g.page / 4)) return failed();
      for (size_t off = 0; off < g.page; off += 4) {
        const uint8_t *p = request.data + off;
        const uint32_t expected = uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
        if (page_[off / 4] != expected) return failed();
      }
      return completed();
    }
    case OEP_V0_TARGET_FLASH_OP_VERIFY_CRC32: {
      struct oep_v0_target_flash_verify_crc32_request request;
      if (!oep_v0_target_flash_verify_crc32_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if ((request.address & 3) || (request.length & 3) || !request.length || !inRange(request.address, request.length))
        return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm_.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      uint32_t crc = 0;
      for (uint32_t off = 0; off < request.length; off += sizeof page_) {
        const uint32_t chunk = request.length - off < sizeof page_ ? request.length - off : sizeof page_;
        if (!dm_.readWords(request.address + off, page_, chunk / 4)) return failed();
        uint8_t bytes[sizeof page_];
        for (size_t i = 0; i < chunk / 4; ++i) {
          bytes[4 * i] = page_[i]; bytes[4 * i + 1] = page_[i] >> 8; bytes[4 * i + 2] = page_[i] >> 16; bytes[4 * i + 3] = page_[i] >> 24;
        }
        crc = crc32Ieee(crc, bytes, chunk);
      }
      struct oep_v0_target_flash_verify_crc32_result result = {crc};
      return completed(oep_v0_target_flash_verify_crc32_result_pack(&result, out, capacity));
    }
    default:
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

}  // namespace oep
