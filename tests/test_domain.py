import pytest
from pentograph_core.domain import is_in_domain, validate_domain, Domain

def test_is_in_domain_inside_default():
    assert is_in_domain(0.0, 0.10) is True

def test_is_in_domain_outside_default():
    assert is_in_domain(0.30, 0.10) is False  # x trop grand

def test_validate_domain_raises_outside():
    with pytest.raises(ValueError):
        validate_domain(0.30, 0.10)

def test_custom_domain():
    d = Domain(x_min=-1.0, x_max=1.0, y_min=-1.0, y_max=1.0)
    assert is_in_domain(0.9, -0.9, d) is True
    assert is_in_domain(1.1, 0.0, d) is False
