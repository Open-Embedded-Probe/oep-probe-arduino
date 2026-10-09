# SPDX-License-Identifier: MIT
"""Explicit USB selection and Arduino CLI upload recipe overrides."""
import time
import os
import sys
import json
import shlex
from pathlib import Path

import serial
import usb.core
import usb.util
from serial.tools import list_ports


def select_usb(devices, unit):
    matches = [d for d in devices if d['serial'].lower() == unit.lower()]
    if len(matches) != 1:
        raise RuntimeError(f'expected one USB device for {unit}, found {len(matches)}')
    return matches[0]


def usb_devices():
    out = []
    for dev in usb.core.find(find_all=True):
        try:
            serial_number = usb.util.get_string(dev, dev.iSerialNumber) if dev.iSerialNumber else ''
        except (usb.core.USBError, ValueError):
            serial_number = ''  # BOOTSEL location remains usable without string-descriptor access.
        out.append({'serial': serial_number or '', 'bus': dev.bus, 'address': dev.address,
                    'ports': list(dev.port_numbers or []), 'vid': dev.idVendor, 'pid': dev.idProduct,
                    'device': dev})
    return out


def usb_identity(unit):
    selected = select_usb(usb_devices(), unit)
    dev = selected['device']
    return {k: v for k, v in selected.items() if k != 'device'} | {
        'product': usb.util.get_string(dev, dev.iProduct) if dev.iProduct else ''}


def resolve_port(unit):
    ports = [p.device for p in list_ports.comports() if (p.serial_number or '').lower() == unit.lower()]
    if len(ports) != 1:
        raise RuntimeError(f'expected one CDC port for {unit}, found {len(ports)}')
    return ports[0]


def properties(**values):
    return tuple(arg for name, value in values.items()
                 for arg in ('--upload-property', f'{name}={value}'))


def dfu_args(unit):
    dev = select_usb(usb_devices(), unit)['device']
    interfaces = [i.bInterfaceNumber for i in dev.get_active_configuration()
                  if (i.bInterfaceClass, i.bInterfaceSubClass, i.bInterfaceProtocol) == (0xFE, 1, 2)]
    if len(interfaces) != 1:
        raise RuntimeError(f'expected one DFU download interface, found {interfaces}')
    # A serial selector is mandatory: VID:PID alone could update another probe.
    pattern = ('"{runtime.tools.dfu-util.path}/dfu-util" --device 1209:4f45 --serial ' + unit +
               ' --intf ' + str(interfaces[0]) + ' --alt 0 -D "{build.path}/{build.project_name}.bin"')
    return properties(**{'upload.pattern': pattern})


def boot_at_location(devices, location, pid):
    matches = [d for d in devices if d['bus'] == location['bus'] and d['ports'] == location['ports']
               and d['vid'] == 0x2E8A and d['pid'] == pid]
    if len(matches) != 1:
        raise RuntimeError(f'expected one BOOTSEL device at the original USB location, found {len(matches)}')
    return matches[0]


def pico_args(port, location, profile):
    pid = 0x0003 if profile == 'rp2040' else 0x000F
    deadline = time.monotonic() + 30
    try:
        boot = boot_at_location(usb_devices(), location, pid)
    except RuntimeError:
        # Touch only the port whose OEP identity was checked under the common lock.
        with serial.Serial(port, 1200, timeout=0.1) as stream:
            stream.dtr = False
            time.sleep(0.1)
        while True:
            try:
                boot = boot_at_location(usb_devices(), location, pid)
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.25)
    drive = os.environ.get('OEP_HW_UF2_DRIVE', '')
    if drive:
        while True:
            try:
                if (Path(drive) / 'INFO_UF2.TXT').is_file():
                    break
            except PermissionError:
                pass  # Automount may create the parent before installing its user ACL.
            if time.monotonic() >= deadline:
                raise RuntimeError('explicit BOOTSEL drive did not become accessible')
            time.sleep(0.25)
        verify_uf2_mount(Path(drive), location)
        copy_script = Path(__file__).with_name('copy_uf2.py')
        where = json.dumps({'bus': location['bus'], 'ports': location['ports']})
        pattern = ' '.join(shlex.quote(arg) for arg in (
            sys.executable, str(copy_script), '{build.path}/{build.project_name}.uf2', drive, where))
        return properties(**{'upload.wait_for_upload_port': 'false', 'upload.pattern': pattern}), {
            k: v for k, v in boot.items() if k != 'device'} | {'drive': drive}
    # Use the pinned core's picotool, selected by bus/address, never an unqualified UF2 drive scan.
    pattern = ('"{runtime.tools.pqt-picotool.path}/picotool" load "{build.path}/{build.project_name}.uf2"'
               f" --bus {boot['bus']} --address {boot['address']} -v -x")
    return properties(**{'upload.wait_for_upload_port': 'false', 'upload.pattern': pattern}), {
                             k: v for k, v in boot.items() if k != 'device'}


def dfu_reenumeration_exit(profile, returncode, output):
    """Only defer this specific post-download disconnect to strict probe verification."""
    return (profile in ('esp32p4', 'esp32p4x') and returncode == 1
            and 'Download done.' in output
            and 'DFU state(7) = dfuMANIFEST, status(0) = No error condition is present' in output
            and 'dfu-util: unable to read DFU status after completion (LIBUSB_ERROR_NO_DEVICE)' in output
            and output.rstrip().endswith('Failed uploading: uploading error: exit status 74'))


def verify_uf2_mount(drive, location):
    stat = drive.stat()
    block = Path(f'/sys/dev/block/{os.major(stat.st_dev)}:{os.minor(stat.st_dev)}').resolve(strict=True)
    expected = str(location['bus']) + '-' + '.'.join(map(str, location['ports']))
    if expected not in [p.name for p in block.parents]:
        raise RuntimeError('UF2 drive does not belong to the original USB location')
