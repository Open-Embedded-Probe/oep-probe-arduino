# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
"""DFU 1.1 download that goes on after a control transfer times out.

dfu-util gives up at the first timed-out control transfer. Behind some hubs (the bench's 3-2.3) one control transfer in
a few hundred stalls for over a second while bulk transfers carry on, so a P4 update failed part way. After a timeout
this reads the state and goes on from what the device has: DNLOAD-SYNC / DNBUSY after a DNLOAD means the block was
taken, DNLOAD-IDLE means it was not (it is sent again). The device checks the image whole at the end (esp_ota_end) and
restarts into it, leaving the bus: that is success.

    python dfu_resume.py --serial UNIT --intf N IMAGE
"""
import argparse
import errno
import sys
import time

DNLOAD, GETSTATUS, CLRSTATUS, GETSTATE, ABORT = 1, 3, 4, 5, 6
OUT, IN = 0x21, 0xA1
IDLE, SYNC, BUSY, DIDLE, MANIFEST_SYNC, MANIFEST, WAIT_RESET, ERROR = 2, 3, 4, 5, 6, 7, 8, 10
BLOCK = 4096
TIMEOUT_MS = 3000
ATTEMPTS = 40
STATUS_GAP_S = 0.05   # between status requests: fewer control transfers than a 1 ms poll, the block's write is ~50 ms


class Gone(Exception):
    """The device left the bus."""


class Dfu:
    def __init__(self, dev, intf, timeout_error, is_gone, sleep=time.sleep):
        self.dev, self.intf, self.timeout_error, self.is_gone, self.sleep = dev, intf, timeout_error, is_gone, sleep
        self.retries = 0

    def _ctl(self, *a):
        for _ in range(ATTEMPTS):
            try:
                return self.dev.ctrl_transfer(*a, timeout=TIMEOUT_MS)
            except self.timeout_error:
                self.retries += 1
                self.sleep(0.5)
            except Exception as e:
                if self.is_gone(e):
                    raise Gone(str(e)) from e
                raise
        raise RuntimeError(f'DFU: {ATTEMPTS} control transfers in a row timed out')

    def state(self):
        return self._ctl(IN, GETSTATE, 0, self.intf, 1)[0]

    def status(self):
        s = self._ctl(IN, GETSTATUS, 0, self.intf, 6)
        return s[0], s[4]

    def ready(self):
        s = self.state()
        if s == ERROR:
            self._ctl(OUT, CLRSTATUS, 0, self.intf, None)
        elif s != IDLE:                       # an earlier try left it mid-download
            self._ctl(OUT, ABORT, 0, self.intf, None)
        s = self.state()
        if s != IDLE:
            raise RuntimeError(f'DFU: state {s}, not dfuIDLE')

    def block(self, number, data):
        """DNLOAD one block (or the empty end) until the device has it; for a data block, wait until it is written."""
        while True:
            try:
                self.dev.ctrl_transfer(OUT, DNLOAD, number, self.intf, data, timeout=TIMEOUT_MS)
                break
            except self.timeout_error:
                self.retries += 1
                if self.state() not in (IDLE, DIDLE) or not data:   # taken (sync / busy), or the end: no block to send again
                    break
            except Exception as e:
                if self.is_gone(e):
                    raise Gone(str(e)) from e
                raise
        if not data:
            return
        for _ in range(2000):                 # 100 s at STATUS_GAP_S: far past a sector's erase
            st, s = self.status()
            if st:
                raise RuntimeError(f'DFU block {number}: status {st}, state {s}')
            if s == DIDLE:
                return
            if s not in (SYNC, BUSY):
                raise RuntimeError(f'DFU block {number}: state {s} while writing')
            self.sleep(STATUS_GAP_S)
        raise RuntimeError(f'DFU block {number}: still writing')

    def download(self, image, log=print):
        self.ready()
        blocks = [image[at:at + BLOCK] for at in range(0, len(image), BLOCK)]
        t0 = time.monotonic()
        for n, data in enumerate(blocks):
            self.block(n, data)
        log(f'DFU: {len(blocks)} blocks in {time.monotonic() - t0:.0f} s, {self.retries} timed-out transfers resumed')
        try:
            self.block(len(blocks), b'')
            for _ in range(400):
                st, s = self.status()
                if st:
                    raise RuntimeError(f'DFU manifest: status {st}, state {s} (the image was refused)')
                if s in (IDLE, WAIT_RESET):
                    break
                self.sleep(0.3)
        except Gone:
            pass                                 # restarted into the new image
        log('DFU: done, the device restarts into the new image')


def main(argv=None):
    import usb.core
    import usb.util
    a = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    a.add_argument('--serial', required=True)
    a.add_argument('--intf', type=int, required=True)
    a.add_argument('image')
    a = a.parse_args(argv)
    def serial_of(d):   # another probe whose strings cannot be read now (busy, suspended) is not this one
        try:
            return usb.util.get_string(d, d.iSerialNumber)
        except (usb.core.USBError, ValueError):
            return None
    devs = [d for d in usb.core.find(find_all=True, idVendor=0x1209, idProduct=0x4F45) if serial_of(d) == a.serial]
    if len(devs) != 1:
        sys.exit(f'DFU: {len(devs)} devices with serial {a.serial}')
    dev = devs[0]
    usb.util.claim_interface(dev, a.intf)
    image = open(a.image, 'rb').read()
    gone = lambda e: isinstance(e, usb.core.USBError) and e.errno == errno.ENODEV   # noqa: E731
    Dfu(dev, a.intf, usb.core.USBTimeoutError, gone).download(image)


if __name__ == '__main__':
    main()
