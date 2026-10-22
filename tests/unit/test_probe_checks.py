import copy
import pytest
from probe_checks import check_after, check_identity


def snapshot():
    return {'unit_id': 'unit-a', 'model': 'esp32', 'firmware': 'candidate',
            'missing_declarations': [],
            'settings_file': {'oep_settings': 1, 'settings': {'core': {'channel.GPIO1.label': 'a'}}},
            'state': {'storage': 'applied', 'saved_generation': 1, 'reason': None}}


def test_wrong_individual_is_rejected():
    with pytest.raises(RuntimeError, match='wrong probe identity'):
        check_identity(snapshot(), 'unit-b')


def test_same_individual_with_wrong_model_is_rejected():
    rec = snapshot()
    rec['model'] = 'esp32p4'
    with pytest.raises(RuntimeError, match='wrong probe identity'):
        check_identity(rec, 'unit-a')


@pytest.mark.parametrize('change, error', [
    ({'firmware': 'old'}, 'unexpected firmware'),
    ({'missing_declarations': ['ops']}, 'missing declarations'),
    ({'settings_file': {'oep_settings': 1, 'settings': {}}}, 'saved settings changed'),
    ({'state': {'storage': 'partial', 'saved_generation': 0, 'reason': 'item'}}, 'not applied'),
])
def test_update_failure_is_detected(change, error):
    before = snapshot()
    after = copy.deepcopy(before)
    after.update(change)
    with pytest.raises(RuntimeError, match=error):
        check_after(before, after, 'candidate')


def test_expected_candidate_and_settings_pass():
    check_after(snapshot(), snapshot(), 'candidate')


def test_same_firmware_transfer_still_requires_a_new_boot():
    before = snapshot() | {'boot_id': 123}
    with pytest.raises(RuntimeError, match='did not reboot'):
        check_after(before, before, 'candidate')
    check_after(before, snapshot() | {'boot_id': 124}, 'candidate')


def test_unreadable_old_settings_do_not_relax_new_declaration_checks():
    before = {'config_error': 'old format could not be read'}
    check_after(before, snapshot(), 'candidate')
    after = snapshot() | {'missing_declarations': ['ops']}
    with pytest.raises(RuntimeError, match='missing declarations'):
        check_after(before, after, 'candidate')
