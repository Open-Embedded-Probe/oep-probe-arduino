#!/bin/bash
# after the probe restarts (WSL + usbipd): wait until Windows shows 303a:4021 as Shared again, attach it, wait for
# WSL; prints the timings. (After the 2026-09-25 Windows restart usbipd re-attached by itself: host/usb_configs.py
# tries to open the probe first and attaches only while it is Shared.)  reattach.sh <start time from date +%s.%N> <busid>
t0=$1
busid=${2:?busid of 303a:4021 in usbipd.exe list}
for i in $(seq 1 80); do
  s=$(usbipd.exe list 2>&1 | tr -d '\r' | grep 303a:4021 | awk '{print $NF}')
  [ "$s" = "Shared" ] && { echo "back in Windows (Shared) at $(echo "$(date +%s.%N) - $t0" | bc) s"; break; }
  sleep 0.2
done
usbipd.exe attach --wsl --busid "$busid" 2>&1 | grep -v info
for i in $(seq 1 50); do lsusb -d 303a:4021 >/dev/null && break; sleep 0.1; done
sleep 1.5
echo "in WSL at $(echo "$(date +%s.%N) - $t0" | bc) s: $(lsusb -d 303a:4021 -v 2>/dev/null | grep -c bInterfaceNumber) interfaces"
