// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepMemo.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2040)

#include <LittleFS.h>

#include "OepFlashFs.h"
#include "OepSettings.h"   // crc32Ieee

namespace oep {
namespace {

namespace rm = reg::probe_memo;

constexpr const char *kDir = "/m", *kStage = "/m/w";
constexpr int kSlots = 8;            // names sharing one CRC-32
constexpr size_t kPath = 16;         // "/m/xxxxxxxx.n"
// Blocks the FS keeps for itself and the other files: the superblock and the directories' metadata pairs, the staging
// file and the copies LittleFS writes before it commits, the settings' file (RP2040 / RP2350, 2 KB) and its copy.
constexpr size_t kReservedBlocks = 12;

uint32_t crcUpdate(uint32_t crc, const uint8_t *d, size_t n) {   // CRC-32/ISO-HDLC, the state kept inverted
  for (size_t i = 0; i < n; ++i) {
    crc ^= d[i];
    for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return crc;
}

// text (core §2.1): len(u8) bytes, 1..64 for a name (min 1) or 0..64 for a prefix (min 0)
bool takeText(const uint8_t *p, size_t len, size_t &at, char *out, size_t min, size_t &out_len) {
  if (at >= len) return false;
  const size_t n = p[at];
  if (n < min || n > 64 || at + 1 + n > len) return false;
  memcpy(out, p + at + 1, n);
  out[n] = 0;
  out_len = n;
  at += 1 + n;
  return true;
}

// a file's header: name_len(u8) name crc(u32); data_at the data's offset
bool header(File &f, char *name, size_t &name_len, uint32_t &crc, size_t &data_at) {
  uint8_t n = 0, c[4];
  if (f.read(&n, 1) != 1 || n < 1 || n > 64) return false;
  if (f.read(reinterpret_cast<uint8_t *>(name), n) != n || f.read(c, 4) != 4) return false;
  name[n] = 0;
  name_len = n;
  crc = getU32(c);
  data_at = 1 + n + 4;
  return true;
}

void slotPath(const char *name, size_t n, int slot, char *path) {
  const uint32_t c = ~crcUpdate(0xFFFFFFFFu, reinterpret_cast<const uint8_t *>(name), n);
  snprintf(path, kPath, "%s/%08lx.%d", kDir, static_cast<unsigned long>(c), slot);
}

// The object's file (true) or the first free slot for it (false; path empty when all slots are taken)
bool find(const char *name, size_t n, char *path) {
  char free_path[kPath] = {}, got[65];
  for (int s = 0; s < kSlots; ++s) {
    char p[kPath];
    slotPath(name, n, s, p);
    File f = LittleFS.open(p, "r");
    if (!f) {
      if (!free_path[0]) memcpy(free_path, p, kPath);
      continue;
    }
    size_t gl = 0, at = 0;
    uint32_t crc = 0;
    if (header(f, got, gl, crc, at) && gl == n && memcmp(got, name, n) == 0) {
      memcpy(path, p, kPath);
      return true;
    }
  }
  memcpy(path, free_path, kPath);
  return false;
}

bool isStage(const char *base) { return strcmp(base, "w") == 0 || strcmp(base, kStage) == 0; }

// every object's file in the folder's order (not the staging file): f(file) -> false stops
template <class F>
void eachObject(F f) {
#if defined(ARDUINO_ARCH_ESP32)
  File dir = LittleFS.open(kDir);
  if (!dir) return;
  for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
    if (isStage(e.name())) continue;
    if (!f(e)) break;
  }
#else
  Dir dir = LittleFS.openDir(kDir);
  while (dir.next()) {
    if (isStage(dir.fileName().c_str())) continue;
    File e = dir.openFile("r");
    if (!f(e)) break;
  }
#endif
}

size_t objectCount() {
  size_t n = 0;
  eachObject([&](File &) { ++n; return true; });
  return n;
}

}  // namespace

bool Memo::begin() {
  if (!FlashFs::mount()) return false;
  if (!LittleFS.exists(kDir)) LittleFS.mkdir(kDir);
  LittleFS.remove(kStage);   // a write a power loss cut short
  const size_t blocks = FlashFs::blocks();
  // one block per object (a 2 KB object is past LittleFS's inline size), the rest kept back (kReservedBlocks)
  const size_t room = blocks > kReservedBlocks ? blocks - kReservedBlocks : 0;
  max_objects_ = static_cast<uint32_t>(room < kMostObjects ? room : kMostObjects);
  return max_objects_ >= 1;
}

bool Memo::offers(uint8_t op) const {
  return max_objects_ && (op == rm::kOpList || op == rm::kOpRead || op == rm::kOpWrite || op == rm::kOpDelete);
}

size_t Memo::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  w.u32(rm::kTlvDescribeMaxObject, kMaxObject);
  w.u32(rm::kTlvDescribeMaxObjects, max_objects_);
  return w.ok() ? w.length() : 0;
}

void Memo::dropStaging() {
  if (!staging_) return;
  stage_.close();
  LittleFS.remove(kStage);
  staging_ = false;
}

Result Memo::handle(uint8_t op, const uint8_t *p, size_t len, uint8_t *out, size_t capacity) {
  switch (op) {
    case rm::kOpList: return list(p, len, out, capacity);
    case rm::kOpRead: return read(p, len, out, capacity);
    case rm::kOpWrite: return write(p, len, out, capacity);
    case rm::kOpDelete: return remove(p, len, out, capacity);
    default: return rejected(kRejectUnknownOperation);
  }
}

// first(u16) prefix(text) [TLV] -> more(u8) count(u8) count x (name(text) size(u32) crc(u32))
Result Memo::list(const uint8_t *p, size_t len, uint8_t *out, size_t capacity) {
  char prefix[65], name[65];
  size_t at = 2, pn = 0;
  if (len < 2 || !takeText(p, len, at, prefix, 0, pn)) return rejected(kRejectMalformed);
  Tail tail;
  const Result t = plainTail(tail, p, len, at, out, capacity);
  if (refused(t)) return t;
  if (capacity < 2) return failed();
  const uint16_t first = getU16(p);
  size_t put = 2, index = 0;
  uint8_t count = 0, more = 0;
  eachObject([&](File &f) {
    size_t n = 0, data_at = 0;
    uint32_t crc = 0;
    if (!header(f, name, n, crc, data_at) || n < pn || memcmp(name, prefix, pn) != 0) return true;
    if (index++ < first) return true;
    const size_t one = 1 + n + 8;
    if (put + one > capacity || count == 0xFF) { more = 1; return false; }
    out[put] = static_cast<uint8_t>(n);
    memcpy(out + put + 1, name, n);
    putU32(out + put + 1 + n, static_cast<uint32_t>(f.size() - data_at));
    putU32(out + put + 1 + n + 4, crc);
    put += one;
    ++count;
    return true;
  });
  out[0] = more;
  out[1] = count;
  return completed(put);
}

// name(text) offset(u32) max(u16) [TLV] -> size(u32) crc(u32) len(u16) data
Result Memo::read(const uint8_t *p, size_t len, uint8_t *out, size_t capacity) {
  char name[65], got[65], path[kPath];
  size_t at = 0, n = 0;
  if (!takeText(p, len, at, name, 1, n) || at + 6 > len) return rejected(kRejectMalformed);
  const uint32_t offset = getU32(p + at);
  size_t most = getU16(p + at + 4);
  Tail tail;
  const Result t = plainTail(tail, p, len, at + 6, out, capacity);
  if (refused(t)) return t;
  if (!find(name, n, path)) return rejected(kRejectNoResource);   // a staged write is not an object yet (§2.3)
  File f = LittleFS.open(path, "r");
  size_t gl = 0, data_at = 0;
  uint32_t crc = 0;
  if (!f || !header(f, got, gl, crc, data_at)) return failed();
  if (capacity < 10) return failed();
  const uint32_t size = static_cast<uint32_t>(f.size() - data_at);
  if (most > capacity - 10) most = capacity - 10;
  size_t got_len = 0;
  if (offset < size) {
    const size_t want = most < size - offset ? most : size - offset;
    if (!f.seek(data_at + offset) || f.read(out + 10, want) != want) return failed();
    got_len = want;
  }
  putU32(out, size);
  putU32(out + 4, crc);
  putU16(out + 8, static_cast<uint16_t>(got_len));
  return completed(10 + got_len);
}

// name(text) size(u32) offset(u32) crc(u32) len(u16) data [TLV] -> -
Result Memo::write(const uint8_t *p, size_t len, uint8_t *out, size_t capacity) {
  char name[65], path[kPath];
  size_t at = 0, nl = 0;
  if (!takeText(p, len, at, name, 1, nl) || at + 14 > len) return rejected(kRejectMalformed);
  const uint32_t size = getU32(p + at), offset = getU32(p + at + 4), want = getU32(p + at + 8);
  const size_t n = getU16(p + at + 12);
  const uint8_t *data = p + at + 14;
  if (at + 14 + n > len) return rejected(kRejectMalformed);
  Tail tail;
  const Result t = plainTail(tail, p, len, at + 14 + n, out, capacity);
  if (refused(t)) return t;
  if (size > kMaxObject) return unsupportedValue(out, capacity);
  if (offset == 0) {                                                    // a new write (§2.3)
    if (n > size) return rejected(kRejectMalformed);
    const bool exists = find(name, nl, path);
    if (!exists && !path[0]) return failed();                           // every slot of this CRC taken
    if (!exists && objectCount() >= max_objects_) return unavailable(out, capacity, reg::core::kUnavailableCauseLimit);
    dropStaging();
    stage_ = LittleFS.open(kStage, "w");
    if (!stage_) return failed();
    uint8_t head[1 + 64 + 4];
    head[0] = static_cast<uint8_t>(nl);
    memcpy(head + 1, name, nl);
    putU32(head + 1 + nl, want);
    if (stage_.write(head, 1 + nl + 4) != 1 + nl + 4) { stage_.close(); LittleFS.remove(kStage); return failed(); }
    staging_ = true;
    memcpy(stage_name_, name, nl + 1);
    stage_name_len_ = nl;
    stage_size_ = size, stage_have_ = 0, stage_crc_ = 0xFFFFFFFFu, stage_want_ = want;
  } else {                                                              // its next part
    if (!staging_ || nl != stage_name_len_ || memcmp(name, stage_name_, nl) != 0 || size != stage_size_ || want != stage_want_ || offset != stage_have_)
      return wrongState(out, capacity);
    if (stage_have_ + n > stage_size_) return rejected(kRejectMalformed);
  }
  if (n && stage_.write(data, n) != n) { dropStaging(); return failed(); }
  stage_have_ += n;
  stage_crc_ = crcUpdate(stage_crc_, data, n);
  if (stage_have_ < stage_size_) return completed();
  // the last part: check the whole, then put it in place of the object at once
  stage_.close();
  staging_ = false;
  const bool exists = find(stage_name_, nl, path);
  if (~stage_crc_ != stage_want_) {
    LittleFS.remove(kStage);
    if (exists) LittleFS.remove(path);                                  // the object goes (§2.3)
    return rejected(kRejectMalformed);
  }
  if (!path[0] || !LittleFS.rename(kStage, path)) { LittleFS.remove(kStage); return failed(); }
  return completed();
}

// name(text) [TLV] -> -
Result Memo::remove(const uint8_t *p, size_t len, uint8_t *out, size_t capacity) {
  char name[65], path[kPath];
  size_t at = 0, nl = 0;
  if (!takeText(p, len, at, name, 1, nl)) return rejected(kRejectMalformed);
  Tail tail;
  const Result t = plainTail(tail, p, len, at, out, capacity);
  if (refused(t)) return t;
  bool had = false;
  if (staging_ && nl == stage_name_len_ && memcmp(name, stage_name_, nl) == 0) { dropStaging(); had = true; }
  if (find(name, nl, path)) {
    if (!LittleFS.remove(path)) return failed();
    had = true;
  }
  return had ? completed() : rejected(kRejectNoResource);
}

}  // namespace oep

#endif
