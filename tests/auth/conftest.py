"""
Pytest configuration and shared fixtures for auth tests.
"""

import pytest
import os
import sys
from datetime import datetime

# Add backend directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))


@pytest.fixture
def test_user_data():
    """Provide test user data."""
    return {
        "email": "test@example.com",
        "password": "SecurePass123",
        "full_name": "Test User"
    }


@pytest.fixture
def weak_password_data():
    """Provide weak password data for testing validation."""
    return {
        "email": "weak@example.com",
        "password": "weak",
        "full_name": "Weak User"
    }


@pytest.fixture
def invalid_email_data():
    """Provide invalid email data for testing validation."""
    return {
        "email": "notanemail",
        "password": "ValidPass123",
        "full_name": "Invalid Email"
    }


@pytest.fixture
def mock_user():
    """Create a mock user object."""
    class MockUser:
        def __init__(self):
            self.id = 1
            self.email = "test@example.com"
            self.password_hash = "hashed_password"
            self.full_name = "Test User"
            self.is_deleted = False
            self.created_at = datetime.utcnow()

    return MockUser()


@pytest.fixture
def mock_session():
    """Create a mock session object."""
    class MockSession:
        def __init__(self):
            self.id = "session_123"
            self.user_id = 1
            self.token = "jwt_token"
            self.expires_at = datetime.utcnow()
            self.created_at = datetime.utcnow()

    return MockSession()


@pytest.fixture
def mock_reset_token():
    """Create a mock reset token object."""
    class MockResetToken:
        def __init__(self):
            self.token = "reset_token_123"
            self.user_id = 1
            self.expires_at = datetime.utcnow()
            self.created_at = datetime.utcnow()

    return MockResetToken()


# Test markers for pytest
pytest_plugins = []


def pytest_configure(config):
    """Configure pytest with custom markers."""
    config.addinivalue_line(
        "markers", "unit: mark test as a unit test"
    )
    config.addinivalue_line(
        "markers", "integration: mark test as an integration test"
    )
    config.addinivalue_line(
        "markers", "security: mark test as a security test"
    )
    config.addinivalue_line(
        "markers", "slow: mark test as slow running"
    )
