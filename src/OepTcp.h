// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// OEP over TCP (oep-transports §1, §2): one listening socket and a few connection slots, on lwIP's sockets (ESP32).
// Header-only: a sketch that includes it pulls in the network; one that does not stays without it.
//
//   static oep::TcpListener<3> tcp;                       // three connections at once
//   endpoint.addTcpListener(tcp.slots(), rx, sizeof rx[0], tcp.kSlots);   // after every other transport
//   tcp.begin(kPort);                                      // once the network is up; again after it went down
//   tcp.poll();                                            // from loop(), before endpoint.poll()
//
// Each slot is one oep::Connection: the endpoint frames length(u16) message on it, answers on it and sends its
// notifications to it; a length over max_frame closes it (Connection::drop). A connection beyond the slots is accepted
// and closed at once. A slot's writes are buffered (kTxBytes) and sent when the endpoint flushes after a poll, so a
// frame's length and body leave in one segment; a write that does not fit waits for the socket up to kWriteWaitMs,
// after which the connection is taken as dead and closed (a peer that stops reading cannot stall the probe longer).
// TCP keepalive (kKeepIdleS, kKeepIntervalS x kKeepCount) closes a connection whose peer vanished without a FIN.
// Port and discovery are outside the specification (transports §1); the port is the sketch's choice.
#pragma once

#include <Arduino.h>

#include "OepEndpoint.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <lwip/sockets.h>
#include <errno.h>

namespace oep {

class TcpSlot final : public Connection {
 public:
  static constexpr size_t kRxBytes = 1024, kTxBytes = 2048;
  static constexpr uint32_t kWriteWaitMs = 2000;

  uint32_t connection() const override { return fd_ >= 0 ? id_ : 0; }
  void drop() override { close(); }
  bool open() const { return fd_ >= 0; }

  int available() override {
    fill();
    return static_cast<int>(rx_len_ - rx_at_);
  }
  int read() override {
    if (!available()) return -1;
    return rx_[rx_at_++];
  }
  int peek() override { return available() ? rx_[rx_at_] : -1; }
  size_t readBytes(char *buffer, size_t length) override {
    size_t n = 0;
    while (n < length && available()) {
      size_t take = rx_len_ - rx_at_;
      if (take > length - n) take = length - n;
      memcpy(buffer + n, rx_ + rx_at_, take);
      rx_at_ += take;
      n += take;
    }
    return n;
  }
  size_t write(uint8_t byte) override { return write(&byte, 1); }
  size_t write(const uint8_t *data, size_t length) override {
    if (fd_ < 0) return length;   // closed: dropped (a result for a connection that went away)
    size_t done = 0;
    while (done < length) {
      if (tx_len_ == kTxBytes && !drain(true)) return length;   // dead: closed, the rest dropped
      size_t take = kTxBytes - tx_len_;
      if (take > length - done) take = length - done;
      memcpy(tx_ + tx_len_, data + done, take);
      tx_len_ += take;
      done += take;
    }
    return length;
  }
  int availableForWrite() override { return fd_ >= 0 ? static_cast<int>(kTxBytes - tx_len_) : 0; }
  // What a poll wrote goes out now, without waiting for the socket (the rest at the next flush or poll).
  void flush() override { drain(false); }
  // Everything buffered out, waiting for the socket (a restart's answer before the chip resets).
  void flushAll() { while (fd_ >= 0 && tx_len_) if (!drain(true)) break; }

  void take(int fd, uint32_t id) {
    fd_ = fd;
    id_ = id;
    rx_len_ = rx_at_ = tx_len_ = 0;
  }
  void close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
    rx_len_ = rx_at_ = tx_len_ = 0;
  }
  void service() {   // the listener's poll: what is buffered goes out, a closed peer is seen
    if (fd_ < 0) return;
    drain(false);
    fill();
  }

 private:
  int fd_ = -1;
  uint32_t id_ = 0;
  uint8_t rx_[kRxBytes];
  size_t rx_len_ = 0, rx_at_ = 0;
  uint8_t tx_[kTxBytes];
  size_t tx_len_ = 0;

  void fill() {
    if (fd_ < 0 || rx_at_ < rx_len_) return;
    rx_len_ = rx_at_ = 0;
    const int n = ::recv(fd_, rx_, sizeof rx_, MSG_DONTWAIT);
    if (n > 0) rx_len_ = static_cast<size_t>(n);
    else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) close();   // FIN, reset, timeout
  }
  // Send what is buffered. wait: until the socket takes some (up to kWriteWaitMs, else the connection is closed).
  // false: the connection is closed.
  bool drain(bool wait) {
    const uint32_t start = millis();
    while (fd_ >= 0 && tx_len_) {
      const int n = ::send(fd_, tx_, tx_len_, MSG_DONTWAIT);
      if (n > 0) {
        memmove(tx_, tx_ + n, tx_len_ - static_cast<size_t>(n));
        tx_len_ -= static_cast<size_t>(n);
        if (!wait) continue;
        return true;
      }
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR && errno != ENOMEM) { close(); return false; }
      if (!wait) return true;
      if (static_cast<uint32_t>(millis() - start) >= kWriteWaitMs) { close(); return false; }
      fd_set w;
      FD_ZERO(&w);
      FD_SET(fd_, &w);
      timeval tv{0, 10000};
      ::select(fd_ + 1, nullptr, &w, nullptr, &tv);
    }
    return fd_ >= 0;
  }
};

template <size_t N>
class TcpListener {
 public:
  static constexpr size_t kSlots = N;
  static constexpr int kKeepIdleS = 10, kKeepIntervalS = 5, kKeepCount = 3;

  Connection *const *slots() {
    for (size_t k = 0; k < N; ++k) ptrs_[k] = &slot_[k];
    return ptrs_;
  }
  TcpSlot &slot(size_t k) { return slot_[k]; }
  bool listening() const { return fd_ >= 0; }
  uint16_t port() const { return port_; }

  // Listen on `port` (any address). false: the socket could not be made (the network not up yet: call again).
  bool begin(uint16_t port) {
    end();
    port_ = port;
    const int fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) return false;
    const int one = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(fd, reinterpret_cast<sockaddr *>(&a), sizeof a) != 0 || ::listen(fd, static_cast<int>(N)) != 0) {
      ::close(fd);
      return false;
    }
    ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    fd_ = fd;
    return true;
  }
  // No new connection (the listening socket closed; a connection still waiting in its backlog is reset), the
  // connections kept: before a restart, so that a host reconnecting at once is refused rather than accepted by the
  // probe that is about to reset (and left waiting on a connection nobody answers).
  void stopListening() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }
  // Every connection closed, the socket too (the network went down).
  void end() {
    for (TcpSlot &s : slot_) s.close();
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }
  // From loop(), before the endpoint's poll: new connections into free slots, the buffered output out, closed peers seen.
  void poll() {
    for (TcpSlot &s : slot_) s.service();
    if (fd_ < 0) return;
    for (;;) {
      const int c = ::accept(fd_, nullptr, nullptr);
      if (c < 0) break;
      TcpSlot *free_slot = nullptr;
      for (TcpSlot &s : slot_) if (!s.open()) { free_slot = &s; break; }
      if (!free_slot) { ::close(c); continue; }   // every slot taken: refused by closing
      const int one = 1, idle = kKeepIdleS, interval = kKeepIntervalS, count = kKeepCount;
      ::setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
      ::setsockopt(c, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof one);
      ::setsockopt(c, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof idle);
      ::setsockopt(c, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof interval);
      ::setsockopt(c, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof count);
      ::fcntl(c, F_SETFL, ::fcntl(c, F_GETFL, 0) | O_NONBLOCK);
      if (++next_id_ == 0) next_id_ = 1;
      free_slot->take(c, next_id_);
    }
  }
  size_t connections() const {
    size_t n = 0;
    for (const TcpSlot &s : slot_) n += s.open();
    return n;
  }

 private:
  TcpSlot slot_[N];
  Connection *ptrs_[N] = {};
  int fd_ = -1;
  uint16_t port_ = 0;
  uint32_t next_id_ = 0;
};

}  // namespace oep

#endif
