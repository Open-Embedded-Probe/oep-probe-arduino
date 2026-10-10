"""Explicit equipment only; every core case has its own pytest/JUnit result."""
import pytest
from oep_client.pytest_conformance import CORE_CASES, assert_case, oep_conformance_report


@pytest.mark.equipment
@pytest.mark.parametrize('case', CORE_CASES)
def test_core_contract(case, oep_conformance_report, record_property):
    assert_case(oep_conformance_report, case, record_property)


@pytest.mark.equipment
def test_common_interface_contracts(oep_conformance_report, record_property):
    rows = [row for row in oep_conformance_report['checks'] if row['level'] == 'interface']
    for row in rows:
        assert_case(oep_conformance_report, row['id'], record_property)
    # An empty list is legitimate; a failed/blocked list operation is not.
    assert_case(oep_conformance_report, 'CORE-LIST', record_property)
