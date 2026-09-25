# /// script
# dependencies = ["pyusb>=1.3"]
# ///
"""DFU 1.1 download (dfu-util -D) with pyusb: the probe's DFU interface (class FE/01), wTransferSize from its
functional descriptor, DNLOAD blocks with GETSTATUS polling, then the zero-length DNLOAD that manifests."""
import sys, time
import usb.core, usb.util

img = open(sys.argv[1], "rb").read()
dev = usb.core.find(idVendor=0x303A, idProduct=0x4021)
cfg = dev.get_active_configuration()
intf = next(i for i in cfg if i.bInterfaceClass == 0xFE and i.bInterfaceSubClass == 1)
n = intf.bInterfaceNumber
extra = bytes(intf.extra_descriptors)
size = 4096
at = 0
while at + 2 <= len(extra):
    if extra[at + 1] == 0x21 and extra[at] >= 7:
        size = extra[at + 5] | extra[at + 6] << 8
    at += extra[at] or 1
print(f"DFU interface {n}, transfer size {size}, image {len(img)} bytes", flush=True)
usb.util.claim_interface(dev, n)

def status():
    s = dev.ctrl_transfer(0xA1, 3, 0, n, 6, timeout=5000)
    return s[0], s[1] | s[2] << 8 | s[3] << 16, s[4]   # bStatus, bwPollTimeout, bState

t0 = time.monotonic()
block = 0
for off in range(0, len(img), size):
    dev.ctrl_transfer(0x21, 1, block, n, img[off:off + size], timeout=5000)
    while True:
        st, poll, state = status()
        if st:
            sys.exit(f"DFU error status {st} state {state} at block {block}")
        if state == 5:   # dfuDNLOAD-IDLE
            break
        time.sleep(poll / 1000)
    block += 1
dt = time.monotonic() - t0
print(f"downloaded {len(img)} bytes in {dt:.1f} s ({len(img) / dt / 1e3:.0f} kB/s)", flush=True)
dev.ctrl_transfer(0x21, 1, block, n, b"", timeout=5000)   # end: manifest
try:
    for _ in range(50):
        st, poll, state = status()
        print("manifest state", state, "status", st, flush=True)
        if st or state in (2, 8):   # dfuIDLE / dfuMANIFEST-WAIT-RESET
            break
        time.sleep(max(poll, 50) / 1000)
except usb.core.USBError as e:
    print("device went away (restarting):", e)
