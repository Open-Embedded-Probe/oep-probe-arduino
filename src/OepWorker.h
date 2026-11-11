// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// A thread of the sketch's own for an oep::SimpleInterface: a job - an op that starts work and answers at once - runs
// on it while the probe keeps answering (the capture, the lease, the job's status), and an op marked oep::kInWorker
// runs on it to the end before its answer (a peripheral begun there takes its interrupts on that core).
//
//   oep::Worker worker;
//   void collect(oep::Args &in, oep::Worker &w) {
//     for (uint8_t i = 0, n = in.u8(); i < n && !w.cancelled(); ++i) { ... }
//     if (bad) w.fail(3, "no answer");   // the job's status says failed, code 3, the message
//   }
//   mine.addWorker(worker, 0x40);                              // status (0x40, lock-free) and cancel (0x41)
//   mine.job(0x10, "collect", {{"count", oep::u8}}, collect);
//
// ESP32 / ESP32-P4: a FreeRTOS task pinned to the core loop() does not run on (its own core on a single-core chip), made
// at the first job. Elsewhere (RP2040 / RP2350, CH32) there is no such task: a job runs when poll() is called - from
// loop1() on an RP2 to keep it off loop()'s core, from loop() on a CH32 (the probe does not answer while it runs) - and
// a kInWorker op runs in the request's own thread.
#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#define OEP_WORKER_TASK 1
#else
#define OEP_WORKER_TASK 0
#endif

namespace oep {

class Args;

class Worker {
 public:
  // The status op's state
  enum State : uint8_t { kIdle = 0, kRunning = 1, kDone = 2, kFailed = 3, kCancelled = 4 };
  using JobFn = void (*)(Args &in, Worker &w);

  // stack: the task's bytes; priority: below the endpoint's loop() by default (it and the capture keep their pace)
  explicit Worker(uint32_t stack = 8192, uint8_t priority = 1) : stack_(stack), priority_(priority) {}

  // In a job: the host asked to stop (the cancel op) - return soon; the status then says cancelled
  bool cancelled() const { return cancel_; }
  // In a job: it did not work; the status says failed with code and message (the message copied)
  void fail(int32_t code, const char *message);

  // Without a task (RP2, CH32): runs the job asked for, if any. With a task: nothing
  void poll();

  State state() const { return static_cast<State>(state_); }
  uint8_t op() const { return op_; }
  int32_t code() const { return code_; }
  const char *message() const { return message_ ? message_ : ""; }
  uint64_t startNs() const { return start_ns_; }
  uint64_t endNs() const { return end_ns_; }

 private:
  friend class SimpleInterface;
  using SyncFn = void (*)(void *ctx);
  // A job: false while one runs, or without memory / a task. request: the fields' bytes (copied)
  bool startJob(uint8_t op, JobFn fn, const uint8_t *request, size_t length);
  // Runs fn(ctx) on the worker to its end; false while a job runs (with a task: it waits for fn)
  bool runSync(SyncFn fn, void *ctx);
  void cancel() { if (state_ == kRunning) cancel_ = true; }
  void runJob();
  bool ensureTask();
#if OEP_WORKER_TASK
  static void task(void *self);
  TaskHandle_t task_ = nullptr;
  SemaphoreHandle_t done_ = nullptr;
  SyncFn sync_fn_ = nullptr;
  void *sync_ctx_ = nullptr;
#endif
  uint32_t stack_;
  uint8_t priority_;
  volatile uint8_t state_ = kIdle, op_ = 0;
  volatile bool cancel_ = false, pending_ = false;
  volatile int32_t code_ = 0;
  char *message_ = nullptr;     // published at the job's end; freed by the next startJob (the status's thread)
  char *failing_ = nullptr;     // the job's own until its end
  int32_t failing_code_ = 0;
  bool failed_ = false;
  volatile uint64_t start_ns_ = 0, end_ns_ = 0;
  JobFn job_ = nullptr;
  uint8_t *request_ = nullptr;
  size_t length_ = 0;
};

}  // namespace oep
