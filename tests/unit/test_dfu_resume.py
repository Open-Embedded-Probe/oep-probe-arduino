# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Open Embedded Probe
"""dfu_resume: a DFU download that goes on after timed-out control transfers, against a fake DFU 1.1 device."""
import pytest

import dfu_resume as d


class Timeout(Exception):
    pass


class GoneError(Exception):
    pass


class FakeDfu:
    """A DFU 1.1 device: DNLOAD -> DNLOAD-SYNC, GETSTATUS -> DNBUSY (written) -> DNLOAD-IDLE; the empty DNLOAD ->
    MANIFEST-SYNC, then the check and the restart (off the bus). `drop`: transfer numbers that time out, `before`
    whether that one is lost before the device sees it (else the device acts and the answer is lost)."""

    def __init__(self, drop=(), lost_before=True, image_ok=True, start=d.IDLE):
        self.state, self.written, self.drop, self.lost_before, self.image_ok = start, [], set(drop), lost_before, image_ok
        self.n, self.gone, self.pending = 0, False, None

    def ctrl_transfer(self, rt, req, value, intf, data, timeout):
        if self.gone:
            raise GoneError('no device')
        self.n += 1
        dropped = self.n in self.drop
        if dropped and self.lost_before:
            raise Timeout()
        out = self._act(req, value, data)
        if dropped:
            raise Timeout()
        return out

    def _act(self, req, value, data):
        if req == d.GETSTATE:
            return [self.state]
        if req == d.ABORT or req == d.CLRSTATUS:
            self.state = d.IDLE
            return None
        if req == d.DNLOAD:
            assert self.state in (d.IDLE, d.DIDLE), f'DNLOAD in state {self.state}'
            if data:
                self.pending, self.state = (value, bytes(data)), d.SYNC
            else:
                self.state = d.MANIFEST_SYNC
            return None
        if req == d.GETSTATUS:
            if self.state == d.SYNC:
                self.written.append(self.pending)
                self.state = d.DIDLE
            elif self.state == d.MANIFEST_SYNC:
                if not self.image_ok:
                    self.state = d.ERROR
                    return [7, 0, 0, 0, d.ERROR, 0]
                self.state = d.MANIFEST
            elif self.state == d.MANIFEST:
                self.gone = True                       # restarts into the new image
            return [0, 1, 0, 0, self.state, 0]
        raise AssertionError(req)


def run(dev):
    dfu = d.Dfu(dev, 4, Timeout, lambda e: isinstance(e, GoneError), sleep=lambda s: None)
    image = bytes(range(256)) * 40                     # 10240 bytes: blocks of 4096, 4096, 2048
    dfu.download(image, log=lambda m: None)
    return dfu, image


def blocks(dev):
    return [n for n, _ in dev.written]


def test_a_clean_download_writes_every_block_once_and_ends_off_the_bus():
    dev = FakeDfu()
    dfu, image = run(dev)
    assert blocks(dev) == [0, 1, 2] and b''.join(b for _, b in dev.written) == image and dev.gone and dfu.retries == 0


@pytest.mark.parametrize('lost_before', [True, False])
@pytest.mark.parametrize('drop', [3, 4, 5, 7])                  # the DNLOADs and GETSTATUSes after ready()
def test_a_timed_out_transfer_is_resumed_without_a_block_lost_or_written_twice(drop, lost_before):
    dev = FakeDfu(drop={drop}, lost_before=lost_before)
    dfu, image = run(dev)
    assert blocks(dev) == [0, 1, 2] and b''.join(b for _, b in dev.written) == image and dev.gone and dfu.retries >= 1


def test_an_earlier_try_left_mid_download_is_aborted_first():
    dev = FakeDfu(start=d.DIDLE)
    run(dev)
    assert blocks(dev) == [0, 1, 2]


def test_a_refused_image_fails():
    with pytest.raises(RuntimeError, match='refused'):
        run(FakeDfu(image_ok=False))
