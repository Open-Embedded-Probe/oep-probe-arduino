// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepSimpleInterface.h"

#include "OepProbeLog.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace oep {

namespace {

constexpr uint8_t kListBit = reg::core::kOpFieldTypeList;

// A number or bool type's width; 0 for bytes / text and for anything else
size_t widthOf(FieldType type) {
  switch (type) {
    case oep::u8: case oep::i8: case oep::boolean: return 1;
    case oep::u16: case oep::i16: return 2;
    case oep::u32: case oep::i32: return 4;
    case oep::u64: case oep::i64: return 8;
    default: return 0;
  }
}

bool validType(FieldType type) {
  if (type & kListBit) return widthOf(static_cast<FieldType>(type & ~kListBit)) != 0;
  return widthOf(type) != 0 || type == oep::bytes || type == oep::text;
}

// The failed answer every op of this class gives (Reply::fail)
const Field kFailed[] = {{"code", oep::i32}, {"message", oep::text}};

Field *copyFields(std::initializer_list<Field> fields) {
  if (fields.size() == 0) return nullptr;
  Field *copy = new Field[fields.size()];
  size_t i = 0;
  for (const Field &f : fields) copy[i++] = f;
  return copy;
}

}  // namespace

size_t fieldsLength(const Field *fields, size_t count, const uint8_t *p, size_t n, bool &ok) {
  size_t at = 0;
  ok = false;
  for (size_t i = 0; i < count; ++i) {
    const FieldType type = fields[i].type;
    if (type & kListBit) {
      if (at + 2 > n) return 0;
      const size_t width = widthOf(static_cast<FieldType>(type & ~kListBit)), elements = getU16(p + at);
      at += 2;
      if (at + width * elements > n) return 0;
      if ((type & ~kListBit) == oep::boolean) {
        for (size_t k = 0; k < elements; ++k) if (p[at + k] > 1) return 0;
      }
      at += width * elements;
    } else if (type == oep::bytes || type == oep::text) {
      if (at + 2 > n) return 0;
      const size_t length = getU16(p + at);
      at += 2;
      if (at + length > n) return 0;
      at += length;
    } else {
      const size_t width = widthOf(type);
      if (at + width > n) return 0;
      if (type == oep::boolean && p[at] > 1) return 0;   // core §7.4: another value is malformed
      at += width;
    }
  }
  ok = true;
  return at;
}

uint64_t Args::take(size_t width) {
  if (at_ + width > n_) return 0;   // past the registered fields: the function read more than it declared
  uint64_t v = 0;
  for (size_t i = 0; i < width; ++i) v |= static_cast<uint64_t>(p_[at_ + i]) << (8 * i);
  at_ += width;
  return v;
}

Span Args::span() {
  if (at_ + 2 > n_) return {nullptr, 0};
  const size_t length = getU16(p_ + at_);
  if (at_ + 2 + length > n_) return {nullptr, 0};
  const Span s{p_ + at_ + 2, length};
  at_ += 2 + length;
  return s;
}

bool Reply::next(FieldType type, size_t length) {
  if (index_ >= count_ || fields_[index_].type != type || n_ + length > cap_) {
    bad_ = true;
    return false;
  }
  ++index_;
  return true;
}

void Reply::put(FieldType type, uint64_t v, size_t width) {
  if (!next(type, width)) return;
  for (size_t i = 0; i < width; ++i) out_[n_ + i] = static_cast<uint8_t>(v >> (8 * i));
  n_ += width;
}

void Reply::span(FieldType type, const void *data, size_t length) {
  if (length > 0xFFFF || !next(type, 2 + length)) {
    bad_ = true;
    return;
  }
  putU16(out_ + n_, static_cast<uint16_t>(length));
  if (length) memcpy(out_ + n_ + 2, data, length);
  n_ += 2 + length;
}

void Reply::text(const char *s) { span(oep::text, s, s ? strlen(s) : 0); }

void Reply::printf(const char *format, ...) {
  if (index_ >= count_ || fields_[index_].type != oep::text || n_ + 2 > cap_) {
    bad_ = true;
    return;
  }
  va_list args;
  va_start(args, format);
  const size_t room = cap_ - n_ - 2;   // vsnprintf writes at most room - 1 characters and its terminator
  const int wrote = vsnprintf(reinterpret_cast<char *>(out_ + n_ + 2), room, format, args);
  va_end(args);
  if (wrote < 0 || (wrote > 0 && static_cast<size_t>(wrote) >= room)) {   // did not fit
    bad_ = true;
    return;
  }
  ++index_;
  putU16(out_ + n_, static_cast<uint16_t>(wrote));
  n_ += 2 + static_cast<size_t>(wrote);
}

void Reply::list(const void *elements, uint16_t count, size_t width) {
  if (index_ >= count_ || !(fields_[index_].type & kListBit) ||
      widthOf(static_cast<FieldType>(fields_[index_].type & ~kListBit)) != width ||
      n_ + 2 + static_cast<size_t>(count) * width > cap_) {
    bad_ = true;
    return;
  }
  ++index_;
  putU16(out_ + n_, count);
  if (count) memcpy(out_ + n_ + 2, elements, static_cast<size_t>(count) * width);   // little-endian chips
  n_ += 2 + static_cast<size_t>(count) * width;
}

Result Reply::done() {
  if (bad_ || index_ != count_) return failed();   // the answer is not the registered one: a firmware bug, said as failed
  return completed(n_);
}

Result Reply::fail(int32_t code, const char *message) {
  n_ = 0;
  index_ = 0;
  fields_ = kFailed;
  count_ = 2;
  bad_ = false;
  i32(code);
  text(message ? message : "");
  if (bad_) {   // the message too long for the frame: the code alone, the message empty
    n_ = 0;
    index_ = 0;
    bad_ = false;
    i32(code);
    text("");
  }
  return failed(n_);
}

Result Reply::unsupported() { return unsupportedValue(out_, cap_); }

Result Reply::busy() { return wrongState(out_, cap_); }

bool SimpleInterface::validName(const char *name) {
  if (!name) return false;
  const size_t n = strlen(name);
  if (n < 1 || n > reg::kLimitOpNameMaxBytes) return false;
  for (size_t i = 0; i < n; ++i) {
    const char c = name[i];
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_')) return false;
  }
  return true;
}

bool SimpleInterface::op(uint8_t code, const char *name, std::initializer_list<Field> request,
                         std::initializer_list<Field> answer, Handler handler, uint8_t flags) {
  return handler && add(code, name, request, answer, kPlain, handler, nullptr, flags);
}

bool SimpleInterface::job(uint8_t code, const char *name, std::initializer_list<Field> request, Worker::JobFn fn) {
  return worker_ && fn && add(code, name, request, {}, kJob, nullptr, fn, 0);
}

bool SimpleInterface::addWorker(Worker &worker, uint8_t base) {
  if (worker_ || base == 0xFF || find(base) || find(base + 1)) return false;
  worker_ = &worker;
  return add(base, "status", {},
             {{"state", oep::u8}, {"op", oep::u8}, {"code", oep::i32}, {"message", oep::text},
              {"start_ns", oep::u64}, {"end_ns", oep::u64}},
             kStatus, nullptr, nullptr, kLockFree) &&
         add(static_cast<uint8_t>(base + 1), "cancel", {}, {}, kCancel, nullptr, nullptr, 0);
}

bool SimpleInterface::addLog(ProbeLog &log) {
  if (log_ || !log.ok() || find(ProbeLog::kOpRead) || find(ProbeLog::kOpMarks) || find(ProbeLog::kOpClear)) return false;
  if (!add(ProbeLog::kOpRead, "log-read", {}, {}, kLog, nullptr, nullptr, kLockFree) ||
      !add(ProbeLog::kOpMarks, "log-marks", {}, {}, kLog, nullptr, nullptr, kLockFree) ||
      !add(ProbeLog::kOpClear, "log-clear", {}, {}, kLog, nullptr, nullptr, 0))
    return false;
  log_ = &log;
  return true;
}

bool SimpleInterface::add(uint8_t code, const char *name, std::initializer_list<Field> request,
                          std::initializer_list<Field> answer, Kind kind, Handler handler, Worker::JobFn job,
                          uint8_t flags) {
  // the interface's own name too (core §13 rule 1, at most 48 bytes): endpoint.add() would refuse it later, unseen
  if (!interfaceName(name_)) return false;
  if (code < 0x10 || find(code) || !validName(name) || request.size() > 0xFF || answer.size() > 0xFF) return false;
  for (const Op *o = ops_; o; o = o->next) if (strcmp(o->name, name) == 0) return false;
  for (const std::initializer_list<Field> *list : {&request, &answer}) {
    for (const Field &f : *list) {
      if (!validName(f.name) || !validType(f.type)) return false;
    }
    for (const Field *a = list->begin(); a != list->end(); ++a)
      for (const Field *b = a + 1; b != list->end(); ++b) if (strcmp(a->name, b->name) == 0) return false;
  }
  Op *o = new Op{code, flags, kind, name, copyFields(request), copyFields(answer), static_cast<uint8_t>(request.size()),
                 static_cast<uint8_t>(answer.size()), handler, job, nullptr};
  Op **at = &ops_;
  while (*at) at = &(*at)->next;
  *at = o;
  return true;
}

Result SimpleInterface::status(Reply &out) const {
  const Worker &w = *worker_;
  const uint8_t state = w.state_;
  __atomic_thread_fence(__ATOMIC_ACQUIRE);   // the code, message and times the job published before its state
  const bool ended = state != Worker::kRunning;
  out.u8(state);
  out.u8(w.op_);
  out.i32(ended ? w.code_ : 0);
  out.text(ended ? w.message() : "");
  out.u64(w.start_ns_);
  out.u64(ended ? w.end_ns_ : 0);
  return out.done();
}

const SimpleInterface::Op *SimpleInterface::find(uint8_t code) const {
  for (const Op *o = ops_; o; o = o->next) if (o->code == code) return o;
  return nullptr;
}

bool SimpleInterface::lockFree(uint8_t op) const {
  const Op *o = find(op);
  return o && (o->flags & kLockFree);
}

size_t SimpleInterface::describe(uint8_t *out, size_t capacity) {
  TlvWriter w(out, capacity);
  uint8_t v[4 + reg::kLimitOpNameMaxBytes];
  for (const Op *o = ops_; o; o = o->next) {
    const size_t n = strlen(o->name);
    v[0] = o->code;
    // a log op: its name only (the console's forms, not fields: the marks are records)
    const uint8_t form = o->kind == kLog ? 0 : reg::core::kOpNameFlagForm;
    v[1] = static_cast<uint8_t>(form | ((o->flags & kLockFree) ? reg::core::kOpNameFlagLockFree : 0));
    memcpy(v + 2, o->name, n);
    w.put(reg::kDescribeOpName, v, 2 + n);
    if (o->kind == kLog) continue;
    const struct { uint8_t part; const Field *fields; size_t count; } parts[] = {
        {reg::core::kOpFieldPartRequest, o->request, o->requests},
        {reg::core::kOpFieldPartSuccess, o->answer, o->answers},
        {reg::core::kOpFieldPartFailed, kFailed, 2}};
    for (const auto &part : parts) {
      for (size_t i = 0; i < part.count; ++i) {
        const size_t m = strlen(part.fields[i].name);
        v[0] = o->code;
        v[1] = part.part;
        v[2] = part.fields[i].type;
        memcpy(v + 3, part.fields[i].name, m);
        w.put(reg::kDescribeOpField, v, 3 + m);
      }
    }
  }
  return w.ok() ? w.length() : 0;
}

Result SimpleInterface::handle(uint8_t op, const uint8_t *payload, size_t length, uint8_t *out, size_t capacity) {
  const Op *o = find(op);
  if (!o) return rejected(kRejectUnknownOperation);
  if (o->kind == kLog) return log_->handle(op, payload, length, out, capacity, max_read_);   // the stream ops' own checks
  bool ok = false;
  const size_t fixed = fieldsLength(o->request, o->requests, payload, length, ok);
  if (!ok) return rejected(kRejectMalformed);
  Tail tail;
  const Result parsed = plainTail(tail, payload, length, fixed, out, capacity);
  if (refused(parsed)) return parsed;
  Args in(payload, fixed);
  Reply reply(out, capacity, o->answer, o->answers);
  switch (o->kind) {
    case kJob: return worker_->startJob(o->code, o->job, payload, fixed) ? reply.done() : reply.busy();
    case kStatus: return status(reply);
    case kCancel:
      worker_->cancel();
      return reply.done();
    case kPlain:
    case kLog: break;
  }
  if (!(o->flags & kInWorker) || !worker_) return o->handler(in, reply);
  struct Call {
    Handler handler;
    Args *in;
    Reply *reply;
    Result result;
  } call{o->handler, &in, &reply, {}};
  if (!worker_->runSync([](void *c) {
        Call &k = *static_cast<Call *>(c);
        k.result = k.handler(*k.in, *k.reply);
      }, &call))
    return reply.busy();   // a job runs on the worker
  return call.result;
}

}  // namespace oep
