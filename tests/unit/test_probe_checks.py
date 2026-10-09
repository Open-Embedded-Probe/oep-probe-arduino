import copy
import pytest
from probe_checks import check_after, check_identity


def snapshot():
    return {'unit_id': 'unit-a', 'model': 'esp32', 'firmware': 'candidate',
            'missing_declarations': [], 'settings_items': [{'tag': 2, 'value_hex': '010061'}],
            'state': {'storage': 'applied', 'slots': [{'slot': 0, 'state': 'connected'}]}}


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
    ({'settings_items': []}, 'saved settings changed'),
    ({'state': {'storage': 'needs_save', 'slots': []}}, 'not applied'),
    ({'state': {'storage': 'applied', 'slots': []}}, 'did not reconnect'),
])
def test_update_failure_is_detected(change, error):
    before = snapshot()
    after = copy.deepcopy(before)
    after.update(change)
    with pytest.raises(RuntimeError, match=error):
        check_after(before, after, 'candidate')


def test_expected_candidate_and_settings_pass():
    check_after(snapshot(), snapshot(), 'candidate')
