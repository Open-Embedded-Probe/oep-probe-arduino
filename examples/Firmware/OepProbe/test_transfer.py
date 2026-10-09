"""The plugin builds the image; its flasher uploads before OEP checks the result."""
import os
import time

from probe_checks import check_after, inspect
from probe_upload import resolve_port, usb_identity


def test_transfer(provider_guard):
    profile = provider_guard['profile']
    unit = os.environ['OEP_TRANSFER_UNIT_ID']
    deadline = time.monotonic() + 45
    while True:
        try:
            port = provider_guard['resolved_port'] if profile == 'esp32' else resolve_port(unit)
            after = inspect(port, unit, profile)
            provider_guard.update(after=after, port_after=port)
            check_after(provider_guard['before'], after, os.environ['OEP_TRANSFER_FIRMWARE'])
            if profile != 'esp32':
                provider_guard['usb_after'] = usb_identity(unit)
                expected = {'esp32p4': 'OEP probe (ESP32-P4)', 'esp32p4x': 'OEP probe (ESP32-P4X)',
                            'rp2040': 'OEP probe (RP2040)', 'rp2350': 'OEP probe (RP2350)',
                            'promicrorp2350': 'OEP probe (RP2350)'}[profile]
                assert provider_guard['usb_after']['product'] == expected
            break
        except Exception:
            if time.monotonic() >= deadline:
                raise
            time.sleep(1)
    if provider_guard['after']['state']['storage'] == 'unreadable':
        provider_guard['settings_preservation'] = 'unknown: stored settings unreadable'
        provider_guard['settings_warning'] = provider_guard['after']['state']['unreadable']
    elif 'settings_items' not in provider_guard['before']:
        provider_guard['settings_preservation'] = 'unknown: old firmware settings could not be read'
    else:
        provider_guard['settings_preservation'] = 'verified'
    provider_guard['passed'] = True
