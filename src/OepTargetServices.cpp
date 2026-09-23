#include "OepTargetServices.h"

#include <string.h>

#include "OepTlv.h"

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
      if (request.mode > 3 || request.confirm > 1) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);  // 0 system, 1 boot payload, 2 user payload, 3 NRST pin
      if (request.mode == 3) {   // hardware reset through the probe's NRST line: the only reset that sets RCC_RSTSCKR.PINRSTF
        if (reset_pin_ < 0) return rejected(OEP_V0_REJECT_UNAVAILABLE);
        if (dm_.attached()) dm_.detach();
        pinMode(reset_pin_, OUTPUT); digitalWrite(reset_pin_, LOW); delay(20); pinMode(reset_pin_, INPUT);   // release to Hi-Z, never drive high
        delay(20);
        struct oep_v0_target_control_reset_result result = {1, 1, 0};
        return completed(oep_v0_target_control_reset_result_pack(&result, out, capacity));
      }
      if (!dm_.attach()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      if (request.mode != 0) {   // 1 = product bootloader, 2 = normalise to user mode: CPU-executed payloads, V2 only
        if (!dm_.hasPayloads()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
        const bool ran = dm_.runPayload(request.mode == 1 ? Ch32Dm::Payload::kPrepareBoot : Ch32Dm::Payload::kNormalizeUser);
        struct oep_v0_target_control_reset_result result = {static_cast<uint8_t>(ran ? 1 : 0), 1, 0};
        const size_t n = oep_v0_target_control_reset_result_pack(&result, out, capacity);
        return ran ? completed(n) : failed(n);
      }
      const Ch32Dm::ResetReport report = dm_.reset(request.confirm == 1);
      struct oep_v0_target_control_reset_result result = {report.flags, report.attempts, report.pc};
      const size_t n = oep_v0_target_control_reset_result_pack(&result, out, capacity);
      // Completion means "the target runs": without confirmation the DM view is
      // all there is; with it, only a PC sample counts.
      const bool ok = request.confirm ? (report.flags & 2) != 0 : (report.flags & 1) != 0;
      return ok ? completed(n) : failed(n);
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
  if (dm_.attached()) dm_.reset(true);
}

size_t TargetControl::describe(uint8_t first, uint8_t *out, size_t capacity) {
  // The SWCLK rate the PHY settled on at the last attach (0 before the first attach).
  if (first > 0) return 0;
  return tlvPutU32(out, capacity, 0, OEP_V0_TLV_CORE_MAX_CLOCK_HZ, phy_.clockHz());
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
      // Two bytes at a two-byte address is a half-word store, which is how option bytes are
      // programmed; everything else stays word aligned.
      const bool half_word = request.data_length == 2 && (request.address & 1) == 0;
      if (!half_word && ((request.address & 3) || !request.data_length || (request.data_length & 3)))
        return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!dm_.halted()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      uint16_t written = 0;
      if (half_word) {
        const uint16_t value = uint16_t(request.data[0]) | uint16_t(request.data[1]) << 8;
        if (dm_.writeHalfWord(request.address, value)) written = 2;
      } else {
        for (; written < request.data_length; written += 4) {
          const uint8_t *p = request.data + written;
          const uint32_t value = uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
          if (!dm_.writeWord(request.address + written, value)) break;
        }
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
