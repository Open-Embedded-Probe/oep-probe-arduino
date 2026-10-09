import pytest

from probe_checks import check_identity
from probe_upload import boot_at_location, dfu_reenumeration_exit, select_usb


@pytest.mark.parametrize('profile, chip, model', [
    ('esp32p4', 'esp32p4 v0.1', 'esp32p4'),
    ('esp32p4', 'esp32p4 v1.3', 'esp32p4'),
    ('esp32p4x', 'esp32p4 v3.2', 'esp32p4x'),
    ('rp2040', 'rp2040', 'rp2040'),
    ('rp2350', 'rp2350', 'rp2350'),
])
def test_matching_image_family(profile, chip, model):
    check_identity({'unit_id': 'a', 'model': model, 'chip': chip}, 'a', profile)


@pytest.mark.parametrize('profile, chip', [
    ('esp32p4', 'esp32p4 v3.2'), ('esp32p4x', 'esp32p4 v1.3'),
    ('esp32p4x', 'esp32p4 v3.0'), ('esp32p4x', 'esp32p4 v4.0'),
    ('esp32p4', ''),
])
def test_wrong_or_unconfirmed_silicon_is_rejected(profile, chip):
    with pytest.raises(RuntimeError):
        check_identity({'unit_id': 'a', 'model': profile, 'chip': chip}, 'a', profile)


def test_legacy_p4_name_is_allowed_only_before_p4x_update():
    identity = {'unit_id': 'a', 'model': 'esp32p4', 'chip': 'esp32p4 v3.2'}
    check_identity(identity, 'a', 'esp32p4x', before=True)
    with pytest.raises(RuntimeError):
        check_identity(identity, 'a', 'esp32p4x')


def test_usb_selection_uses_exact_serial_among_multiple_devices():
    devices = [{'serial': 'ABC'}, {'serial': 'ABCD'}, {'serial': 'other'}]
    assert select_usb(devices, 'abc') is devices[0]
    with pytest.raises(RuntimeError):
        select_usb(devices, 'ab')
    with pytest.raises(RuntimeError):
        select_usb(devices + [devices[0]], 'abc')


def test_bootsel_is_selected_by_original_location_and_chip_family():
    location = {'bus': 3, 'ports': [1, 4, 2]}
    def boot(ports, pid=0x0003):
        return {'bus': 3, 'ports': ports, 'vid': 0x2E8A, 'pid': pid}
    devices = [boot([1, 4, 1]), boot([1, 4, 2])]
    assert boot_at_location(devices, location, 0x0003) is devices[1]
    with pytest.raises(RuntimeError):
        boot_at_location(devices[:1], location, 0x0003)
    with pytest.raises(RuntimeError):
        boot_at_location(devices, location, 0x000F)
    with pytest.raises(RuntimeError):
        boot_at_location(devices + [devices[1]], location, 0x0003)


DISCONNECT = '''Download done.
DFU state(7) = dfuMANIFEST, status(0) = No error condition is present
dfu-util: unable to read DFU status after completion (LIBUSB_ERROR_NO_DEVICE)
Failed uploading: uploading error: exit status 74
'''


def test_only_completed_dfu_disconnect_is_deferred_to_probe_verification():
    assert dfu_reenumeration_exit('esp32p4x', 1, DISCONNECT)
    assert not dfu_reenumeration_exit('rp2040', 1, DISCONNECT)
    assert not dfu_reenumeration_exit('esp32p4x', 2, DISCONNECT)
    assert not dfu_reenumeration_exit('esp32p4x', 1, DISCONNECT.replace('Download done.', ''))
    assert not dfu_reenumeration_exit('esp32p4x', 1, DISCONNECT.replace('status(0)', 'status(7)'))
    assert not dfu_reenumeration_exit('esp32p4x', 1, DISCONNECT.replace('exit status 74', 'exit status 2'))
    assert not dfu_reenumeration_exit('esp32p4x', 1, 'LIBUSB_ERROR_NO_DEVICE')


def test_uf2_copy_writes_only_the_verified_drive(tmp_path, monkeypatch):
    import struct
    import copy_uf2
    first, other = tmp_path / 'first', tmp_path / 'other'
    first.mkdir()
    other.mkdir()
    (first / 'INFO_UF2.TXT').write_text('RP2')
    (other / 'INFO_UF2.TXT').write_text('RP2')
    image = tmp_path / 'app.uf2'
    image.write_bytes(struct.pack('<II', 0x0A324655, 0x9E5D5157) + bytes(504))
    checked = []
    monkeypatch.setattr(copy_uf2, 'verify_uf2_mount', lambda drive, where: checked.append(drive))
    copy_uf2.copy_uf2(image, first, {'bus': 3, 'ports': [1]})
    assert checked == [first]
    assert (first / 'NEW.UF2').read_bytes() == image.read_bytes()
    assert not (other / 'NEW.UF2').exists()


def test_uf2_copy_refuses_a_drive_at_another_usb_location(tmp_path, monkeypatch):
    import copy_uf2
    def refuse(*args):
        raise RuntimeError('wrong USB location')
    monkeypatch.setattr(copy_uf2, 'verify_uf2_mount', refuse)
    with pytest.raises(RuntimeError, match='wrong USB location'):
        copy_uf2.copy_uf2(tmp_path / 'app.uf2', tmp_path, {})
    assert not (tmp_path / 'NEW.UF2').exists()
