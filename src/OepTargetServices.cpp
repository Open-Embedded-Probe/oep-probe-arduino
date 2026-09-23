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
void TargetConsole::push(uint8_t byte) {
  const uint16_t next = static_cast<uint16_t>((head_ + 1) % kCapacity);
  if (next == tail_) { ++dropped_; return; }   // full: the reader is not keeping up
  buffer_[head_] = byte;
  head_ = next;
}

void TargetConsole::poll() {
  // DATA0 and DATA1 are the abstract command's operands too, so leave them alone unless
  // the target is attached and running its own code.
  if (!enabled_ || dm_.halted()) return;
  if (!phy_.attached()) {
    // A reset detaches, and the console has to outlive that: the point of it is to watch
    // a target through its own restarts. Retry at a slow rate so a target that is simply
    // gone does not turn every loop into a full attach.
    if (millis() - last_attach_ms_ < 250) return;
    last_attach_ms_ = millis();
    if (!dm_.attach()) return;
  }
  if (framing_ == 1) pollDmdata(); else pollSdi();
}

// SerialSDI: the target waits for DATA0 to read zero, writes DATA1 = bytes 3..6 and
// DATA0 = length | bytes 0..2 << 8, and we zero DATA0 once we have the frame.
void TargetConsole::pollSdi() {
  uint32_t data0 = 0;
  if (!phy_.read(0x04, data0)) return;
  const uint8_t length = static_cast<uint8_t>(data0 & 0xff);
  if (length == 0 || length > 7) return;       // 0 = nothing waiting; anything else is not a frame
  uint32_t data1 = 0;
  if (!phy_.read(0x05, data1)) return;
  const uint8_t bytes[7] = {
      static_cast<uint8_t>(data0 >> 8), static_cast<uint8_t>(data0 >> 16), static_cast<uint8_t>(data0 >> 24),
      static_cast<uint8_t>(data1), static_cast<uint8_t>(data1 >> 8),
      static_cast<uint8_t>(data1 >> 16), static_cast<uint8_t>(data1 >> 24)};
  for (uint8_t i = 0; i < length; ++i) push(bytes[i]);
  phy_.write(0x04, 0);                         // taken: this is what the target waits for
}

// SerialDMDATA, minichlink's framing. The status byte is the low byte of DATA0: bit 7 says
// the word is the target's, the low bits are a byte count biased by 4. We answer each of its
// words exactly once, and the answer is also our outgoing frame when there is one - three
// bytes at a time, since only DATA0 carries host payload - or zero when there is not.
void TargetConsole::pollDmdata() {
  uint32_t data0 = 0;
  if (!phy_.read(0x04, data0)) return;
  if (data0 & 0x80u) {                         // the target's word
    uint32_t count = data0 & 0x3fu;
    if (count > 4u) {
      count -= 4u;
      if (count > 7u) count = 7u;
      uint32_t data1 = 0;
      if (!phy_.read(0x05, data1)) return;
      const uint8_t bytes[7] = {
          static_cast<uint8_t>(data0 >> 8), static_cast<uint8_t>(data0 >> 16), static_cast<uint8_t>(data0 >> 24),
          static_cast<uint8_t>(data1), static_cast<uint8_t>(data1 >> 8),
          static_cast<uint8_t>(data1 >> 16), static_cast<uint8_t>(data1 >> 24)};
      for (uint32_t i = 0; i < count; ++i) push(bytes[i]);
      saw_empty_ = false;
      sendOrClear();
      return;
    }
    // Count 4 is the target's empty frame: "the mailbox is yours". But its write() leaves
    // that word and then, a moment later, its own frame on top - so an empty frame may be
    // the instant before real bytes land, and answering it then wipes them out. Measured on
    // a CH32X035 at 48 MHz behind the P4's fast PHY: every other frame vanished, "core_ap"
    // and " READY" gone and "i" and "\r\n" arriving (2026-09-23). A slow target hid it. So
    // answer an empty frame only once it has stood still for a whole poll.
    if (!saw_empty_) {
      saw_empty_ = true;
      return;
    }
    saw_empty_ = false;
    sendOrClear();
    return;
  }
  // Bit 7 clear: our own frame not yet collected, or the word we just left. Not ours to
  // touch - the target owns the initiative, and a probe only ever answers the target's
  // words, as minichlink's terminal does. Writing into a clear word races the target's own
  // poll, which leaves its empty frame there at the same moment; on the CH32X035 that ate
  // the host's frames and PING never came back (2026-09-23).
  saw_empty_ = false;
}

// Clearing bit 7 is how the target learns its frame was taken - and our own frame clears
// it too. So when there is something to send, send it here rather than zeroing first: a
// target that keeps printing leaves its empty frame on every poll, and a probe that only
// ever answered with zero would never get a turn (2026-09-23).
void TargetConsole::sendOrClear() {
  const uint16_t waiting = pending();
  if (!waiting) {
    phy_.write(0x04, 0);
    return;
  }
  uint8_t p[3] = {0, 0, 0};
  const uint8_t chunk = waiting > 3 ? 3 : static_cast<uint8_t>(waiting);
  for (uint8_t i = 0; i < chunk; ++i) {
    p[i] = tx_[tx_tail_];
    tx_tail_ = static_cast<uint16_t>((tx_tail_ + 1) % kTxCapacity);
  }
  phy_.write(0x04, uint32_t(chunk + 4u) | (uint32_t(p[0]) << 8) |
                       (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 24));
}

void TargetConsole::abandon() {
  enabled_ = false;
  head_ = tail_ = 0;
  tx_head_ = tx_tail_ = 0;
  dropped_ = 0;
}

Result TargetConsole::handle(uint8_t operation, const uint8_t *payload, size_t length,
                                      uint8_t *out, size_t capacity) {
  switch (operation) {
    case OEP_V0_TARGET_CONSOLE_OP_CONFIGURE: {
      struct oep_v0_target_console_configure_request request;
      if (!oep_v0_target_console_configure_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (request.enable && !dm_.attach()) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      if (request.framing > 1) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      // Every enable is a fresh session, even over one that is still open: a runner that
      // moves from one sketch to the next reprograms the target in between, and bytes
      // queued for the last sketch must not be delivered to this one.
      if (request.enable) {
        head_ = tail_ = 0;
        tx_head_ = tx_tail_ = 0;
        dropped_ = 0;
        saw_empty_ = false;
        // Whatever an earlier session left in the mailbox would read as a frame - including
        // SerialDMDATA's latched timeout, which a host clears by taking the word. Claim it,
        // then let a couple of rounds go by and throw those away, so the first exchange the
        // caller sees is not the tail of somebody else's.
        phy_.write(0x04, 0);
        enabled_ = true;
        framing_ = request.framing;
        for (int i = 0; i < 4; ++i) poll();
        head_ = tail_ = 0;
      }
      enabled_ = request.enable != 0;
      framing_ = request.framing;
      struct oep_v0_target_console_configure_result result = {static_cast<uint8_t>(enabled_ ? 1 : 0), framing_};
      return completed(oep_v0_target_console_configure_result_pack(&result, out, capacity));
    }
    case OEP_V0_TARGET_CONSOLE_OP_READ: {
      struct oep_v0_target_console_read_request request;
      if (!oep_v0_target_console_read_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      poll();                                  // one more frame before answering, if one is waiting
      size_t n = 0;
      const size_t limit = request.maximum < capacity ? request.maximum : capacity;
      while (n < limit && tail_ != head_) {
        out[n++] = buffer_[tail_];
        tail_ = static_cast<uint16_t>((tail_ + 1) % kCapacity);
      }
      return completed(n);
    }
    case OEP_V0_TARGET_CONSOLE_OP_STATUS: {
      struct oep_v0_target_console_status_request request;
      if (!oep_v0_target_console_status_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      struct oep_v0_target_console_status_result result = {static_cast<uint8_t>(enabled_ ? 1 : 0), buffered(), dropped_};
      return completed(oep_v0_target_console_status_result_pack(&result, out, capacity));
    }
    case OEP_V0_TARGET_CONSOLE_OP_WRITE: {
      struct oep_v0_target_console_write_request request;
      if (!oep_v0_target_console_write_request_unpack(payload, length, &request)) return rejected(OEP_V0_REJECT_MALFORMED_PAYLOAD);
      if (!enabled_ || framing_ != 1) return rejected(OEP_V0_REJECT_UNAVAILABLE);
      uint16_t queued = 0;
      while (queued < request.data_length) {
        const uint16_t next = static_cast<uint16_t>((tx_head_ + 1) % kTxCapacity);
        if (next == tx_tail_) break;           // full: the target is not collecting
        tx_[tx_head_] = request.data[queued++];
        tx_head_ = next;
      }
      poll();                                  // start it on its way before answering
      struct oep_v0_target_console_write_result result = {queued};
      return completed(oep_v0_target_console_write_result_pack(&result, out, capacity));
    }
    default:
      return rejected(OEP_V0_REJECT_UNKNOWN_OPERATION);
  }
}

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
