// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Open Embedded Probe

// Wi-Fi for OEP over TCP on an ESP32 with its own radio: joins the networks of oep.probe.config's wifi item (item 0x08,
// probe.config §1.4, OepConfig.h), listens on one TCP port (OepTcp.h) and announces it by mDNS. Header-only: only a sketch that
// includes it links the Wi-Fi stack.
//
//   static oep::TcpListener<3> tcp;
//   static oep::WifiStation wifi(tcp, kPort);
//   config.setWifi(&wifi);                 // before config.load() / applySaved()
//   wifi.begin(unitId);                    // mDNS names from the unit_id
//   loop(): wifi.poll(); tcp.poll(); endpoint.poll(); ...
//
// The list is tried in index order (probe.config §1.4: no entry skipped - a hidden SSID never shows in a scan), each
// entry for up to kTryMs (less when the driver reports a failure: network not found, authentication); the first that
// gets an IPv4 address is kept. After every entry failed the probe waits kRetryMs and starts over; after the link goes
// it starts over at once. A list changed while connected keeps the link when the entry in use is still there
// unchanged; otherwise the change takes effect kApplyDelayMs later (the answer to the set that changed it goes out
// first, also on that link). An empty list turns the radio off.
//
// Discovery (transports §3): DNS-SD over mDNS, host name "oep-<unit_id>", service _oep._tcp on the
// listener's port, instance name "OEP <unit_id>", TXT unit_id=<unit_id>. Nothing about the networks is logged: the
// probe's UART carries OEP and no log (probe guide §3), and a passphrase never leaves the probe.
#pragma once

#include <Arduino.h>

#include "OepConfig.h"
#include "OepTcp.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(SOC_WIFI_SUPPORTED) && SOC_WIFI_SUPPORTED
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_wifi.h>

namespace oep {

class WifiStation final : public WifiControl {
 public:
  static constexpr uint32_t kTryMs = 15000, kRetryMs = 5000, kApplyDelayMs = 300;
  // A disconnect reported this soon after an attempt began is the previous attempt's (the driver reports a
  // disconnect it was asked for a moment later): not this entry's failure.
  static constexpr uint32_t kSettleMs = 300;

  template <size_t N>
  WifiStation(TcpListener<N> &tcp, uint16_t port)
      : port_(port), begin_(&beginTcp<N>), end_(&endTcp<N>), tcp_(&tcp) {}

  // unit_id: the mDNS names'. Before the settings are applied.
  void begin(const char *unit_id) {
    snprintf(unit_, sizeof unit_, "%s", unit_id);
    self_ = this;
    WiFi.persistent(false);   // the networks live in the probe's settings, not in the Wi-Fi driver's own storage
    WiFi.onEvent(onDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  }

  void apply(const WifiEntry *entries, size_t count) override {
    // the entry in use, unchanged in the new list: the link stays
    bool keep = false;
    if (phase_ == kConnected && current_ < count_)
      for (size_t i = 0; i < count; ++i) keep |= entries[i] == list_[current_];
    const WifiEntry was = current_ < count_ ? list_[current_] : WifiEntry{};
    count_ = count > ProbeConfig::kMaxWifi ? ProbeConfig::kMaxWifi : count;
    for (size_t i = 0; i < count_; ++i) list_[i] = entries[i];
    if (keep) {
      for (size_t i = 0; i < count_; ++i) if (list_[i] == was) current_ = i;
      return;
    }
    pending_ = true;
    pending_at_ = millis();
  }

  Status status() const override {
    Status s;
    s.state = state_;
    s.reason = reason_;
    s.entry = (phase_ == kConnected || phase_ == kTrying) && current_ < count_ ? list_[current_].index : kNoEntry;
    if (phase_ == kConnected) {
      s.rssi = static_cast<int8_t>(WiFi.RSSI());
      const uint32_t ip = static_cast<uint32_t>(WiFi.localIP());
      memcpy(s.ipv4, &ip, 4);   // network order, as IPAddress keeps it: a.b.c.d
    }
    return s;
  }

  void poll() {
    const uint32_t now = millis();
    if (pending_ && static_cast<uint32_t>(now - pending_at_) >= kApplyDelayMs) {
      pending_ = false;
      restart();
      return;   // the phases below count from times taken after `now`
    }
    switch (phase_) {
      case kOff: return;
      case kWaiting:
        if (static_cast<uint32_t>(now - since_) >= kRetryMs) startOver();
        return;
      case kTrying:
        if (lost_ && static_cast<uint32_t>(now - since_) < kSettleMs) lost_ = false;
        if (WiFi.status() == WL_CONNECTED && static_cast<uint32_t>(WiFi.localIP()) != 0) {
          phase_ = kConnected;
          state_ = kStateConnected;
          reason_ = kReasonNone;
          up();
          return;
        }
        if (lost_ || static_cast<uint32_t>(now - since_) >= kTryMs) {
          // this entry failed: why (a timeout with the association made is a missing address)
          reason_ = lost_ ? reason(lost_reason_)
                         : static_cast<uint8_t>(WiFi.status() == WL_CONNECTED ? kReasonNoAddress : kReasonNotFound);
          WiFi.disconnect(false, false);
          next();
        }
        return;
      case kConnected:
        if (lost_ || WiFi.status() != WL_CONNECTED) {   // the link went: the list again, from its first entry
          reason_ = lost_ ? reason(lost_reason_) : static_cast<uint8_t>(kReasonOther);
          WiFi.disconnect(false, false);
          startOver();
        }
        return;
    }
  }

 private:
  enum : uint8_t { kOff, kWaiting, kTrying, kConnected };
  uint16_t port_;
  bool (*begin_)(void *, uint16_t);
  void (*end_)(void *);
  void *tcp_;
  char unit_[40] = {};
  WifiEntry list_[ProbeConfig::kMaxWifi];
  size_t count_ = 0, current_ = 0;
  size_t tried_ = 0;   // the entries of list_ tried since the start over
  uint8_t phase_ = kOff, state_ = kStateOff, reason_ = kReasonNone;
  uint32_t since_ = 0, pending_at_ = 0;
  bool pending_ = false, listening_ = false, mdns_ = false;
  volatile bool lost_ = false;
  char trying_[33] = {};            // the SSID of the attempt under way (the disconnect events are matched to it)
  volatile uint8_t trying_length_ = 0;
  volatile uint8_t lost_reason_ = 0;
  static inline WifiStation *self_ = nullptr;

  template <size_t N> static bool beginTcp(void *t, uint16_t port) { return static_cast<TcpListener<N> *>(t)->begin(port); }
  template <size_t N> static void endTcp(void *t) { static_cast<TcpListener<N> *>(t)->end(); }

  static void onDisconnected(arduino_event_id_t, arduino_event_info_t info) {
    if (!self_) return;
    // only the network being tried or used (a disconnect of another one, asked for before, says nothing about it)
    const auto &d = info.wifi_sta_disconnected;
    if (d.ssid_len != self_->trying_length_ || memcmp(d.ssid, self_->trying_, d.ssid_len) != 0) return;
    self_->lost_reason_ = static_cast<uint8_t>(info.wifi_sta_disconnected.reason);   // wifi_err_reason_t's values fit a byte
    self_->lost_ = true;
  }
  // The driver's reason (wifi_err_reason_t) as the state's.
  static uint8_t reason(uint8_t r) {
    switch (r) {
      case WIFI_REASON_NO_AP_FOUND: return kReasonNotFound;
      case WIFI_REASON_AUTH_FAIL:
      case WIFI_REASON_AUTH_EXPIRE:
      case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
      case WIFI_REASON_HANDSHAKE_TIMEOUT:
      case WIFI_REASON_MIC_FAILURE:
        return kReasonAuth;
      default: return kReasonOther;
    }
  }

  void restart() {   // the list as it is now, from the start
    lost_ = false;
    if (count_ == 0) {
      down();
      WiFi.disconnect(true, false);
      WiFi.mode(WIFI_OFF);
      phase_ = kOff;
      state_ = kStateOff;
      reason_ = kReasonNone;
      return;
    }
    if (phase_ == kConnected || phase_ == kTrying) WiFi.disconnect(false, false);
    if (WiFi.getMode() != WIFI_STA) {
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);   // no modem sleep: a request is not held for the next beacon
      WiFi.setAutoReconnect(false);   // the list decides what to join next
      if (!mdns_ && unit_[0]) {
        char host[48];
        snprintf(host, sizeof host, "oep-%s", unit_);
        mdns_ = MDNS.begin(host);
        if (mdns_) {
          char instance[48];
          snprintf(instance, sizeof instance, "OEP %s", unit_);
          MDNS.setInstanceName(instance);
          MDNS.addService("oep", "tcp", port_);
          MDNS.addServiceTxt("oep", "tcp", "unit_id", static_cast<const char *>(unit_));
        }
      }
    }
    startOver();
  }
  void startOver() {   // the list from its first entry
    lost_ = false;
    state_ = kStateConnecting;
    tried_ = 0;
    next();
  }
  void next() {
    lost_ = false;
    if (tried_ >= count_) {   // every entry failed: wait, then start over
      phase_ = kWaiting;
      state_ = kStateWaiting;
      since_ = millis();
      return;
    }
    current_ = tried_++;
    const WifiEntry &e = list_[current_];
    phase_ = kTrying;
    since_ = millis();
    trying_length_ = 0;
    memcpy(trying_, e.ssid, e.ssid_length);
    trying_length_ = e.ssid_length;
    WiFi.begin(e.ssid, e.pass_length ? e.pass : nullptr);
  }
  void up() {   // an address: the listener, once (it stays across links: it listens on every address)
    esp_wifi_set_ps(WIFI_PS_NONE);   // no modem sleep (again: the driver's own default may have come back with the link)
    if (!listening_) listening_ = begin_(tcp_, port_);
  }
  void down() {
    if (listening_) end_(tcp_);
    listening_ = false;
  }
};

}  // namespace oep

#endif
