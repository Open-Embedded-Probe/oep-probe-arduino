"""Hold the equipment-wide lock and check identity before plugin upload."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess

import pytest
from probe_checks import inspect, required


@pytest.fixture(scope='module')
def provider_guard(request, arduino_cli_build, arduino_cli_app):
    if request.config.getoption('run_mode') == 'build':
        yield None
        return
    if arduino_cli_app.profile != 'esp32':
        pytest.fail('The first transfer gate supports classic ESP32 only; select --profile esp32')
    port = required('TEST_SERIAL_PORT_ESP32')
    configured_port = request.config.option.port
    if Path(configured_port).resolve() != Path(port).resolve():
        pytest.fail('pytest port differs from TEST_SERIAL_PORT_ESP32')
    unit = required('OEP_TRANSFER_UNIT_ID')
    required('OEP_TRANSFER_FIRMWARE')
    results = Path(required('OEP_TRANSFER_RESULTS'))
    with Path(required('OEP_HW_LOCK')).open('r+') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        results.mkdir(parents=True, exist_ok=False)
        repo = Path(__file__).resolve().parents[3]
        record = {'passed': False, 'profile': 'esp32', 'flash_backup': False, 'firmware_restore': False,
                  'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
                  'source_dirty': bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=repo, text=True)),
                  'spec_commit': os.environ.get('OEP_TRANSFER_SPEC_COMMIT', 'unknown'),
                  'images_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in arduino_cli_app.build_path.glob('*.bin')}}
        try:
            # Use a fixed physical port for this upload, checked with the OEP unit id.
            resolved = str(Path(port).resolve(strict=True))
            request.config.option.port = resolved
            record.update(port=port, resolved_port=resolved)
            record['before'] = inspect(resolved, unit)
            yield record
        except Exception as exc:
            record['error'] = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            (results / 'transfer.json').write_text(json.dumps(record, indent=2) + '\n')


@pytest.fixture(scope='module')
def arduino_cli_device_locks(arduino_cli_device_locks, provider_guard):
    # Extend the plugin fixture: its own per-device lock and cleanup stay in force.
    yield
