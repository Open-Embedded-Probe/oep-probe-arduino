#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""OEP checks around upload; build/upload themselves belong to the pytest plugin."""
import dataclasses
import json
import os
import re


def required(name):
    value = os.environ.get(name, '')
    if not value:
        raise ValueError(f'{name} must be explicitly set')
    return value


MODELS = {'esp32': 'esp32', 'esp32p4': 'esp32p4', 'esp32p4x': 'esp32p4x',
          'rp2040': 'rp2040', 'rp2350': 'rp2350'}


def check_identity(identity, unit, profile='esp32', *, before=False):
    expected = MODELS[profile]
    allowed = {expected}
    if before and profile == 'esp32p4x':
        allowed.add('esp32p4')  # Older images named both silicon variants P4.
    if identity['unit_id'] != unit or identity['model'] not in allowed:
        raise RuntimeError(f'wrong probe identity: {identity}')
    if profile in ('esp32p4', 'esp32p4x'):
        match = re.fullmatch(r'esp32p4 v(\d+)\.(\d+)', identity.get('chip', ''))
        if not match:
            raise RuntimeError('P4 silicon revision is missing or unknown')
        revision = int(match[1]) * 100 + int(match[2])
        low, high = (1, 199) if profile == 'esp32p4' else (301, 399)
        if not low <= revision <= high:
            raise RuntimeError(f'{profile} image does not support silicon revision {revision}')


def inspect(port, unit, profile='esp32', *, before=False, allow_unreadable=False):
    from oep_client import core, dump, link, settings, virtual_bench
    h = link.open_host(port)
    try:
        values = dict(core.describe(h))
        identity = {name: values[tag].decode() for name, tag in (
            ('unit_id', virtual_bench.CORE_UNIT_ID), ('model', virtual_bench.CORE_MODEL),
            ('firmware', virtual_bench.CORE_FIRMWARE))}
        identity['chip'] = values.get(virtual_bench.CORE_CHIP, b'').decode()
        check_identity(identity, unit, profile, before=before)
        record = dict(identity)
        record['boot_id'] = h.limits['boot_id']
        try:
            st = settings.Settings(h)          # the core's settings (oep-spec core §13): the user's values by name
            if st.offers('get'):
                record['settings_file'] = st.export()
                if st.offers('state'):
                    record['state'] = dataclasses.asdict(st.state())
        except Exception as exc:
            if not allow_unreadable:
                raise
            record['config_error'] = f'{type(exc).__name__}: {exc}'
        try:
            caps = dump.collect(lambda fn, op, p: h.call(fn, op, p, locked=False).payload,
                                confirm=h.confirm_range())
            record.update(capabilities=json.loads(dump.to_json(caps)), missing_declarations=caps.missing)
        except Exception as exc:
            if not allow_unreadable:
                raise
            record['declarations_error'] = f'{type(exc).__name__}: {exc}'
        return record
    finally:
        try:
            if h.session is not None:
                h.end()
        finally:
            h.link.close()


def check_after(before, after, firmware):
    if 'boot_id' in before and before['boot_id'] == after.get('boot_id'):
        raise RuntimeError('probe did not reboot after transfer')
    if after['firmware'] != firmware:
        raise RuntimeError(f"unexpected firmware: {after['firmware']}")
    if after['missing_declarations']:
        raise RuntimeError(f"missing declarations: {after['missing_declarations']}")
    if 'settings_file' in before and before['settings_file']['settings'] != after.get('settings_file', {}).get('settings'):
        raise RuntimeError('saved settings changed')     # by name (core §13.7): the same values whatever moved
    if before.get('state', {}).get('storage') == 'applied' and after.get('state', {}).get('storage') != 'applied':
        raise RuntimeError('saved settings were not applied')
