// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// An interface of the sketch's own from functions: each op registered with its name and its request and answer fields
// (name, type), shown in the describe (oep-core §7.4 op_name / op_field) so a host calls it by name without knowing the
// interface. The fields' lengths are checked before the function runs (a short request is malformed, the tail after
// them follows core §2.3), and the answer is packed in the order registered (docs/guide/simple-interface.md).
//
//   oep::Result add(oep::Args &in, oep::Reply &out) { out.i32(in.i32() + in.i32()); return out.done(); }
//   oep::SimpleInterface mine("io.github.yourname.mine", 1);
//   mine.op(0x10, "add", {{"a", oep::i32}, {"b", oep::i32}}, {{"sum", oep::i32}}, add);
//   endpoint.add(mine);   // or oep::firmware.add(mine) before oep::firmware.begin()
//
// Not for an interface with plan roles, a stream or fine lock rules: those implement oep::Interface itself.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include <initializer_list>

#include "Oep.h"
#include "OepRegistry.h"

namespace oep {

// A field's type (core §7.4 op_field type)
using FieldType = uint8_t;
inline constexpr FieldType u8 = reg::core::kOpFieldTypeU8;
inline constexpr FieldType u16 = reg::core::kOpFieldTypeU16;
inline constexpr FieldType u32 = reg::core::kOpFieldTypeU32;
inline constexpr FieldType u64 = reg::core::kOpFieldTypeU64;
inline constexpr FieldType i8 = reg::core::kOpFieldTypeI8;
inline constexpr FieldType i16 = reg::core::kOpFieldTypeI16;
inline constexpr FieldType i32 = reg::core::kOpFieldTypeI32;
inline constexpr FieldType i64 = reg::core::kOpFieldTypeI64;
inline constexpr FieldType boolean = reg::core::kOpFieldTypeBool;
inline constexpr FieldType bytes = reg::core::kOpFieldTypeBytes;
inline constexpr FieldType text = reg::core::kOpFieldTypeText;
// A list of a number or bool type: count(u16), then count elements
constexpr FieldType list(FieldType element) { return static_cast<FieldType>(element | reg::core::kOpFieldTypeList); }

struct Field {
  const char *name;   // 1-24 of a-z 0-9 - _
  FieldType type;
};

// An op's flags
inline constexpr uint8_t kLockFree = 0x01;   // taken without the lock: it changes nothing (core §6)

// A run of bytes in a request (bytes, text: not terminated)
struct Span {
  const uint8_t *data;
  size_t length;
};

// A request's fields, read in the order registered (their lengths checked before the function runs)
class Args {
 public:
  Args(const uint8_t *p, size_t n) : p_(p), n_(n) {}
  uint8_t u8() { return static_cast<uint8_t>(take(1)); }
  uint16_t u16() { return static_cast<uint16_t>(take(2)); }
  uint32_t u32() { return static_cast<uint32_t>(take(4)); }
  uint64_t u64() { return take(8); }
  int8_t i8() { return static_cast<int8_t>(take(1)); }
  int16_t i16() { return static_cast<int16_t>(take(2)); }
  int32_t i32() { return static_cast<int32_t>(take(4)); }
  int64_t i64() { return static_cast<int64_t>(take(8)); }
  bool boolean() { return take(1) != 0; }
  Span bytes() { return span(); }
  Span text() { return span(); }
  // A list: its count, then each element with the type's reader (count() then u32() count times, ...)
  uint16_t count() { return u16(); }

 private:
  uint64_t take(size_t width);
  Span span();
  const uint8_t *p_;
  size_t n_, at_ = 0;
};

// An answer's fields, packed in the order registered; done() / fail() / unsupported() / busy() end it
class Reply {
 public:
  Reply(uint8_t *out, size_t capacity, const Field *fields, size_t count)
      : out_(out), cap_(capacity), fields_(fields), count_(count) {}
  void u8(uint8_t v) { put(oep::u8, v, 1); }
  void u16(uint16_t v) { put(oep::u16, v, 2); }
  void u32(uint32_t v) { put(oep::u32, v, 4); }
  void u64(uint64_t v) { put(oep::u64, v, 8); }
  void i8(int8_t v) { put(oep::i8, static_cast<uint8_t>(v), 1); }
  void i16(int16_t v) { put(oep::i16, static_cast<uint16_t>(v), 2); }
  void i32(int32_t v) { put(oep::i32, static_cast<uint32_t>(v), 4); }
  void i64(int64_t v) { put(oep::i64, static_cast<uint64_t>(v), 8); }
  void boolean(bool v) { put(oep::boolean, v ? 1 : 0, 1); }
  void bytes(const void *data, size_t length) { span(oep::bytes, data, length); }
  void text(const char *s);
  void printf(const char *format, ...) __attribute__((format(printf, 2, 3)));   // a text field, formatted
  // A list field: count elements of the list's element type, each as its little-endian width
  void list(const void *elements, uint16_t count, size_t width);
  Result done();
  Result fail(int32_t code, const char *message);   // completed failed: code(i32), message(text)
  Result unsupported();                             // a value this op does not take (rejected unsupported)
  Result busy();                                    // not now (rejected unavailable, wrong state)

 private:
  bool next(FieldType type, size_t length);
  void put(FieldType type, uint64_t v, size_t width);
  void span(FieldType type, const void *data, size_t length);
  uint8_t *out_;
  size_t cap_, n_ = 0;
  const Field *fields_;
  size_t count_, index_ = 0;
  bool bad_ = false;
};

using Handler = Result (*)(Args &in, Reply &out);

class SimpleInterface : public Interface {
 public:
  SimpleInterface(const char *name, uint8_t revision, uint16_t instance = 0)
      : name_(name), revision_(revision), instance_(instance) {}
  // false: a code below 0x10 or taken, a bad name, a field list that is not a fixed part (core §7.4), no memory
  bool op(uint8_t code, const char *name, std::initializer_list<Field> request, std::initializer_list<Field> answer,
          Handler handler, uint8_t flags = 0);

  const char *name() const override { return name_; }
  uint16_t instance() const override { return instance_; }
  uint8_t revision() const override { return revision_; }
  bool offers(uint8_t op) const override { return find(op) != nullptr; }
  bool lockFree(uint8_t op) const override;
  size_t describe(uint8_t *out, size_t capacity) override;
  Result handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) override;

  static bool validName(const char *name);

 private:
  struct Op {
    uint8_t code, flags;
    const char *name;
    Field *request, *answer;
    uint8_t requests, answers;
    Handler handler;
    Op *next;
  };
  const Op *find(uint8_t code) const;
  const char *name_;
  uint8_t revision_;
  uint16_t instance_;
  Op *ops_ = nullptr;
};

// The request's fields' bytes (bytes / text with their len, lists with their count), or 0 when they do not fit in n
// or a bool is not 0 / 1 (malformed); the tail (core §2.3) starts after them
size_t fieldsLength(const Field *fields, size_t count, const uint8_t *p, size_t n, bool &ok);

}  // namespace oep
