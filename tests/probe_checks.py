#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""OEP checks around upload; build/upload themselves belong to the pytest plugin."""
import dataclasses
import json
import os


def required(name):
    value = os.environ.get(name, '')
    if not value:
        raise ValueError(f'{name} must be explicitly set')
    return value


def check_identity(identity, unit):
    if identity['unit_id'] != unit or identity['model'] != 'esp32':
        raise RuntimeError(f'wrong probe identity: {identity}')


def inspect(port, unit):
    from oep_client import config, core, dump, link, virtual_bench
    h = link.open_host(port)
    try:
        values = dict(core.describe(h))
        identity = {name: values[tag].decode() for name, tag in (
            ('unit_id', virtual_bench.CORE_UNIT_ID), ('model', virtual_bench.CORE_MODEL),
            ('firmware', virtual_bench.CORE_FIRMWARE))}
        check_identity(identity, unit)
        cfg = config.ProbeConfig(h)
        cfg_hash, items = cfg.get()
        caps = dump.collect(lambda fn, op, p: h.call(fn, op, p, locked=False).payload,
                            confirm=h.confirm_range())
        return {**identity, 'settings_hash': cfg_hash,
                'settings_items': [{'tag': tag, 'value_hex': value.hex()} for tag, value in items],
                'state': dataclasses.asdict(cfg.state()), 'capabilities': json.loads(dump.to_json(caps)),
                'missing_declarations': caps.missing}
    finally:
        try:
            if h.session is not None:
                h.end()
        finally:
            h.link.close()


def check_after(before, after, firmware):
    from oep_client import config
    if after['firmware'] != firmware:
        raise RuntimeError(f"unexpected firmware: {after['firmware']}")
    if after['missing_declarations']:
        raise RuntimeError(f"missing declarations: {after['missing_declarations']}")
    unpack = lambda record: [(v['tag'], bytes.fromhex(v['value_hex'])) for v in record['settings_items']]
    if not config.same_items(unpack(before), unpack(after)):
        raise RuntimeError('saved settings changed')
    if before['state']['storage'] == 'applied' and after['state']['storage'] != 'applied':
        raise RuntimeError('saved settings were not applied')
    expected = {s['slot'] for s in before['state']['slots'] if s['state'] == 'connected'}
    actual = {s['slot'] for s in after['state']['slots'] if s['state'] == 'connected'}
    if not expected <= actual:
        raise RuntimeError('previously connected slots did not reconnect')
