# /// script
# dependencies = ["pyserial"]
# ///
"""Read a P4 USB-Serial/JTAG console without resetting it (DTR held): usj_monitor.py <tty> [seconds] [reset]."""
import sys, time, serial
s = serial.Serial(); s.port = sys.argv[1]; s.baudrate = 115200; s.timeout = 0.2
s.dtr = True; s.rts = False
if "reset" in sys.argv:
    s.dtr = False; s.rts = False
s.open()
if "reset" in sys.argv:
    s.rts = True; time.sleep(0.1); s.rts = False
end = time.time() + float(sys.argv[2] if len(sys.argv) > 2 and sys.argv[2] != "reset" else 3); out = b""
while time.time() < end: out += s.read(4096)
print(out.decode(errors="replace"))
