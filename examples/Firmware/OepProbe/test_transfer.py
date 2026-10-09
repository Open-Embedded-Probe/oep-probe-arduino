"""The plugin compiles/uploads first; OEP then owns the binary serial link."""
import os
import time

from probe_checks import check_after, inspect


def test_transfer(provider_guard):
    deadline = time.monotonic() + 40
    while True:
        try:
            after = inspect(provider_guard['resolved_port'], os.environ['OEP_TRANSFER_UNIT_ID'])
            provider_guard['after'] = after
            check_after(provider_guard['before'], after, os.environ['OEP_TRANSFER_FIRMWARE'])
            break
        except Exception:
            if time.monotonic() >= deadline:
                raise
            time.sleep(1)
    provider_guard['passed'] = True
