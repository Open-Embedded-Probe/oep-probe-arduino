"""OEP provider guards around the Arduino CLI plugin's build/upload fixtures."""
from dataclasses import replace
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys

import pytest
from probe_checks import MODELS, inspect, required
from probe_upload import dfu_args, dfu_reenumeration_exit, pico_args, usb_identity, boot_at_location, usb_devices


@pytest.fixture(scope='module')
def provider_guard(request, arduino_cli_build, arduino_cli_app):
    if request.config.getoption('run_mode') == 'build':
        yield None
        return
    profile = arduino_cli_app.profile
    if profile not in MODELS:
        pytest.fail(f'unsupported transfer profile: {profile}')
    port = os.environ.get('TEST_SERIAL_PORT_' + profile.upper()) or required('TEST_SERIAL_PORT')
    if Path(request.config.option.port).resolve() != Path(port).resolve():
        pytest.fail('pytest port differs from the explicit environment port')
    unit = required('OEP_TRANSFER_UNIT_ID')
    if not re.fullmatch(r'[0-9a-fA-F]{12,32}', unit):
        pytest.fail('unit ID must be 12..32 hexadecimal characters')
    required('OEP_TRANSFER_FIRMWARE')
    results = Path(required('OEP_TRANSFER_RESULTS'))
    allow_unreadable = os.environ.get('OEP_TRANSFER_ALLOW_UNREADABLE_CONFIG', '0') == '1'
    with Path(required('OEP_HW_LOCK')).open('r+') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        results.mkdir(parents=True, exist_ok=False)
        repo = Path(__file__).resolve().parents[3]
        record = {'passed': False, 'profile': profile, 'flash_backup': False, 'firmware_restore': False,
                  'allow_unreadable_before': allow_unreadable,
                  'started_at': datetime.now(timezone.utc).isoformat(),
                  'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
                  'source_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=repo, text=True)),
                  'spec_commit': os.environ.get('OEP_TRANSFER_SPEC_COMMIT', 'unknown'),
                  'images_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in arduino_cli_app.build_path.iterdir() if p.suffix in ('.bin', '.uf2')}}
        try:
            resume = os.environ.get('OEP_TRANSFER_RESUME_FROM', '')
            if resume:
                if profile not in ('rp2040', 'rp2350', 'promicrorp2350'):
                    pytest.fail('BOOTSEL resume is supported only for Pico profiles')
                previous = json.loads(Path(resume).read_text())
                if previous['profile'] != profile or previous['before']['unit_id'] != unit:
                    pytest.fail('resume record belongs to another probe/profile')
                boot_at_location(usb_devices(), previous['usb_before'], 3 if profile == 'rp2040' else 15)
                resolved = previous['resolved_port']
                record.update(before=previous['before'], usb_before=previous['usb_before'], resumed_from=resume)
            else:
                resolved = str(Path(port).resolve(strict=True))
                record['before'] = inspect(resolved, unit, profile, before=True, allow_unreadable=allow_unreadable)
                if profile != 'esp32':
                    record['usb_before'] = usb_identity(unit)
            request.config.option.port = resolved
            record.update(port=port, resolved_port=resolved)
            if profile in ('esp32p4', 'esp32p4x'):
                subprocess.run([sys.executable, str(repo / 'tools/check_p4_firmware.py'),
                                str(arduino_cli_app.build_path), profile], check=True)
            yield record
        except Exception as exc:
            record['error'] = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            record['finished_at'] = datetime.now(timezone.utc).isoformat()
            (results / 'transfer.json').write_text(json.dumps(record, indent=2) + '\n')


@pytest.fixture(scope='module', autouse=True)
def arduino_cli_upload(request, arduino_cli_build, provider_guard, arduino_cli_flasher, arduino_cli_device_locks):
    # Build comes from the plugin; customize its public flasher fixture, keeping Arduino CLI upload.
    if request.config.getoption('run_mode') == 'build':
        yield
        return
    profile = provider_guard['profile']
    unit = required('OEP_TRANSFER_UNIT_ID')
    extra = ()
    if profile in ('esp32p4', 'esp32p4x'):
        extra = dfu_args(unit)
    elif profile in ('rp2040', 'rp2350', 'promicrorp2350'):
        extra, provider_guard['bootsel'] = pico_args(provider_guard['resolved_port'],
                                                   provider_guard['usb_before'], profile)
    flasher = replace(arduino_cli_flasher, port=('UF2_Board' if profile.startswith('rp') or profile == 'promicrorp2350'
                                                else provider_guard['resolved_port']), extra_args=extra)
    provider_guard['upload_command'] = flasher.upload_command()
    results = Path(required('OEP_TRANSFER_RESULTS'))
    with (results / 'upload.log').open('w') as log:
        result = subprocess.run(flasher.upload_command(), cwd=flasher.sketch_dir,
                                stdout=log, stderr=subprocess.STDOUT, timeout=600)
    provider_guard['upload_exit'] = result.returncode
    if dfu_reenumeration_exit(profile, result.returncode, (results / 'upload.log').read_text()):
        provider_guard['upload_disconnect_pending_verification'] = True
    elif result.returncode:
        pytest.fail(f'Arduino CLI upload failed ({result.returncode}); see {results / "upload.log"}')
    yield
