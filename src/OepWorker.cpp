// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

#include "OepWorker.h"

#include <stdlib.h>
#include <string.h>

#include "Oep.h"
#include "OepSimpleInterface.h"

namespace oep {

namespace {

char *copyText(const char *s) {
  const size_t n = s ? strlen(s) : 0;
  char *c = static_cast<char *>(malloc(n + 1));
  if (!c) return nullptr;
  if (n) memcpy(c, s, n);
  c[n] = '\0';
  return c;
}

}  // namespace

void Worker::fail(int32_t code, const char *message) {
  free(failing_);   // a second fail() in the same job: the last one stands
  failing_ = copyText(message);
  failing_code_ = code;
  failed_ = true;
}

bool Worker::startJob(uint8_t op, JobFn fn, const uint8_t *request, size_t length) {
  if (state_ == kRunning || pending_) return false;
  if (!ensureTask()) return false;
  uint8_t *copy = nullptr;
  if (length) {
    copy = static_cast<uint8_t *>(malloc(length));
    if (!copy) return false;
    memcpy(copy, request, length);
  }
  free(message_);   // the last job's: the status, which reads it, runs in this thread
  message_ = nullptr;
  job_ = fn;
  request_ = copy;
  length_ = length;
  op_ = op;
  code_ = 0;
  start_ns_ = nowNs();   // the op's time; runJob() sets the job's own start when it begins
  end_ns_ = 0;
  cancel_ = false;
  __atomic_thread_fence(__ATOMIC_RELEASE);
  state_ = kRunning;
  pending_ = true;
#if OEP_WORKER_TASK
  xTaskNotifyGive(task_);
#endif
  return true;
}

void Worker::runJob() {
  pending_ = false;
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  failed_ = false;
  failing_ = nullptr;
  failing_code_ = 0;
  start_ns_ = nowNs();
  Args in(request_, length_);
  job_(in, *this);
  const uint64_t end = nowNs();
  free(request_);
  request_ = nullptr;
  length_ = 0;
  code_ = failed_ ? failing_code_ : 0;
  message_ = failing_;
  failing_ = nullptr;
  end_ns_ = end;
  __atomic_thread_fence(__ATOMIC_RELEASE);   // the code, message and times before the state the status looks at
  state_ = cancel_ ? kCancelled : failed_ ? kFailed : kDone;
}

void Worker::poll() {
#if !OEP_WORKER_TASK
  if (pending_) runJob();
#endif
}

bool Worker::runSync(SyncFn fn, void *ctx) {
  if (state_ == kRunning) return false;
#if OEP_WORKER_TASK
  if (!ensureTask()) return false;
  if (xTaskGetCurrentTaskHandle() == task_) {   // already on the worker (a job calling into the interface)
    fn(ctx);
    return true;
  }
  sync_ctx_ = ctx;
  __atomic_thread_fence(__ATOMIC_RELEASE);
  sync_fn_ = fn;
  xTaskNotifyGive(task_);
  xSemaphoreTake(done_, portMAX_DELAY);
#else
  fn(ctx);
#endif
  return true;
}

#if OEP_WORKER_TASK
void Worker::task(void *self) {
  Worker &w = *static_cast<Worker *>(self);
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (w.sync_fn_) {
      __atomic_thread_fence(__ATOMIC_ACQUIRE);
      const SyncFn fn = w.sync_fn_;
      fn(w.sync_ctx_);
      w.sync_fn_ = nullptr;
      xSemaphoreGive(w.done_);
    }
    if (w.pending_) w.runJob();
  }
}

bool Worker::ensureTask() {
  if (task_) return true;
  if (!done_) done_ = xSemaphoreCreateBinary();
  if (!done_) return false;
#if portNUM_PROCESSORS > 1 && defined(ARDUINO_RUNNING_CORE)
  const BaseType_t core = ARDUINO_RUNNING_CORE ^ 1;   // off loop()'s core: an ESP32-P4's core 0
#else
  const BaseType_t core = 0;
#endif
  return xTaskCreatePinnedToCore(task, "oep_worker", stack_, this, priority_, &task_, core) == pdPASS;
}
#else
bool Worker::ensureTask() { return true; }
#endif

}  // namespace oep
