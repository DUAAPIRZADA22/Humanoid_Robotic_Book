"""
Integration tests for authentication API routes.
Tests all /api/auth endpoints with real HTTP requests.
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch
from sqlmodel import Session

from backend.main import app
from backend.auth.database import get_session
from backend.auth.models import User


@pytest.fixture
def client():
    """Create a test client for the FastAPI app."""
    return TestClient(app)


@pytest.fixture
def mock_db_session():
    """Create a mock database session."""
    session = Mock(spec=Session)
    return session


class TestRegisterEndpoint:
    """Tests for POST /api/auth/register endpoint."""

    def test_register_success(self, client, mock_db_session):
        """Test successful registration."""
        with patch('backend.auth.routes.get_session') as mock_get_session, \
             patch('backend.auth.service.register_user') as mock_register, \
             patch('backend.auth.service.create_user_session') as mock_create_session, \
             patch('backend.auth.security.create_access_token') as mock_token:

            mock_get_session.return_value = mock_db_session
            mock_user = Mock(
                id=1,
                email="test@example.com",
                full_name="Test User",
                is_deleted=False
            )
            mock_register.return_value = mock_user
            mock_token.return_value = "jwt_token"
            mock_create_session.return_value = Mock(token="jwt_token")

            response = client.post(
                "/api/auth/register",
                json={
                    "email": "test@example.com",
                    "password": "SecurePass123",
                    "full_name": "Test User"
                }
            )

            assert response.status_code == 201
            data = response.json()
            assert data["access_token"] == "jwt_token"
            assert data["user"]["email"] == "test@example.com"

    def test_register_duplicate_email(self, client, mock_db_session):
        """Test registration with duplicate email returns 400."""
        from backend.auth.exceptions import DuplicateEmailError

        with patch('backend.auth.routes.get_session') as mock_get_session, \
             patch('backend.auth.service.register_user') as mock_register:

            mock_get_session.return_value = mock_db_session
            mock_register.side_effect = DuplicateEmailError("test@example.com")

            response = client.post(
                "/api/auth/register",
                json={
                    "email": "test@example.com",
                    "password": "SecurePass123"
                }
            )

            assert response.status_code == 400

    def test_register_weak_password(self, client, mock_db_session):
        """Test registration with weak password returns 400."""
        from backend.auth.exceptions import WeakPasswordError

        with patch('backend.auth.routes.get_session') as mock_get_session, \
             patch('backend.auth.service.register_user') as mock_register:

            mock_get_session.return_value = mock_db_session
            mock_register.side_effect = WeakPasswordError(["Password too short"])

            response = client.post(
                "/api/auth/register",
                json={
                    "email": "test@example.com",
                    "password": "weak"
                }
            )

            assert response.status_code == 400


class TestSignInEndpoint:
    """Tests for POST /api/auth/signin endpoint."""

    def test_signin_success(self, client, mock_db_session):
        """Test successful sign in."""
        with patch('backend.auth.routes.get_session') as mock_get_session, \
             patch('backend.auth.service.authenticate_user') as mock_auth, \
             patch('backend.auth.service.create_user_session') as mock_create_session, \
             patch('backend.auth.security.create_access_token') as mock_token:

            mock_get_session.return_value = mock_db_session
            mock_user = Mock(
                id=1,
                email="test@example.com",
                full_name="Test User"
            )
            mock_auth.return_value = mock_user
            mock_token.return_value = "jwt_token"
            mock_create_session.return_value = Mock(token="jwt_token")

            response = client.post(
                "/api/auth/signin",
                json={
                    "email": "test@example.com",
                    "password": "CorrectPassword123"
                }
            )

            assert response.status_code == 200
            data = response.json()
            assert data["access_token"] == "jwt_token"
            assert data["user"]["email"] == "test@example.com"

    def test_signin_invalid_credentials(self, client, mock_db_session):
        """Test sign in with invalid credentials returns 401."""
        from backend.auth.exceptions import InvalidCredentialsError

        with patch('backend.auth.routes.get_session') as mock_get_session, \
             patch('backend.auth.service.authenticate_user') as mock_auth:

            mock_get_session.return_value = mock_db_session
            mock_auth.side_effect = InvalidCredentialsError()

            response = client.post(
                "/api/auth/signin",
                json={
                    "email": "test@example.com",
                    "password": "WrongPassword"
                }
            )

            assert response.status_code == 401


class TestSignOutEndpoint:
    """Tests for POST /api/auth/signout endpoint."""

    def test_signout_success(self, client, mock_db_session):
        """Test successful sign out."""
        with patch('backend.auth.routes.get_session') as mock_get_session, \
             patch('backend.auth.routes.get_current_user') as mock_current_user, \
             patch('backend.auth.service.invalidate_session') as mock_invalidate:

            mock_get_session.return_value = mock_db_session
            mock_user = Mock(id=1, email="test@example.com")
            mock_current_user.return_value = mock_user

            response = client.post(
                "/api/auth/signout",
                headers={"Authorization": "Bearer valid_token"}
            )

            assert response.status_code == 200
            mock_invalidate.assert_called_once()


class TestGetCurrentUserEndpoint:
    """Tests for GET /api/auth/me endpoint."""

    def test_get_current_user_success(self, client, mock_db_session):
        """Test getting current user info."""
        with patch('backend.auth.routes.get_session') as mock_get_session, \
             patch('backend.auth.routes.get_current_user') as mock_current_user:

            mock_get_session.return_value = mock_db_session
            mock_user = Mock(
                id=1,
                email="test@example.com",
                full_name="Test User",
                is_deleted=False
            )
            mock_current_user.return_value = mock_user

            response = client.get(
                "/api/auth/me",
                headers={"Authorization": "Bearer valid_token"}
            )

            assert response.status_code == 200
            data = response.json()
            assert data["email"] == "test@example.com"


class TestPasswordResetEndpoints:
    """Tests for password reset endpoints."""

    def test_reset_request_success(self, client, mock_db_session):
        """Test password reset request."""
        with patch('backend.auth.routes.get_session') as mock_get_session, \
             patch('backend.auth.reset_service.create_reset_token') as mock_create:

            mock_get_session.return_value = mock_db_session
            mock_user = Mock(id=1, email="test@example.com")
            mock_create.return_value = Mock(
                token="reset_token",
                user_id=1
            )

            response = client.post(
                "/api/auth/reset-request",
                json={"email": "test@example.com"}
            )

            assert response.status_code == 200

    def test_reset_confirm_success(self, client, mock_db_session):
        """Test password reset confirmation."""
        with patch('backend.auth.routes.get_session') as mock_get_session, \
             patch('backend.auth.reset_service.validate_reset_token') as mock_validate, \
             patch('backend.auth.reset_service.reset_user_password') as mock_reset:

            mock_get_session.return_value = mock_db_session
            mock_token = Mock(
                token="valid_token",
                user_id=1
            )
            mock_validate.return_value = mock_token

            response = client.post(
                "/api/auth/reset-confirm",
                json={
                    "token": "valid_token",
                    "new_password": "NewPassword123"
                }
            )

            assert response.status_code == 200
            mock_reset.assert_called_once()


class TestProfileEndpoints:
    """Tests for profile management endpoints."""

    def test_get_profile_success(self, client, mock_db_session):
        """Test getting user profile."""
        with patch('backend.auth.routes.get_session') as mock_get_session, \
             patch('backend.auth.routes.get_current_user') as mock_current_user:

            mock_get_session.return_value = mock_db_session
            mock_user = Mock(
                id=1,
                email="test@example.com",
                full_name="Test User",
                is_deleted=False
            )
            mock_current_user.return_value = mock_user

            response = client.get(
                "/api/auth/profile",
                headers={"Authorization": "Bearer valid_token"}
            )

            assert response.status_code == 200

    def test_update_profile_success(self, client, mock_db_session):
        """Test updating user profile."""
        with patch('backend.auth.routes.get_session') as mock_get_session, \
             patch('backend.auth.routes.get_current_user') as mock_current_user, \
             patch('backend.auth.profile_service.update_profile') as mock_update:

            mock_get_session.return_value = mock_db_session
            mock_user = Mock(
                id=1,
                email="test@example.com",
                full_name="Test User"
            )
            mock_current_user.return_value = mock_user
            mock_updated = Mock(
                id=1,
                email="test@example.com",
                full_name="Updated Name"
            )
            mock_update.return_value = mock_updated

            response = client.put(
                "/api/auth/profile",
                headers={"Authorization": "Bearer valid_token"},
                json={"full_name": "Updated Name"}
            )

            assert response.status_code == 200
            data = response.json()
            assert data["full_name"] == "Updated Name"


class TestProtectedEndpoint:
    """Tests for protected endpoint authentication."""

    def test_protected_endpoint_without_token(self, client):
        """Test accessing protected endpoint without token returns 401."""
        response = client.get("/api/auth/me")
        assert response.status_code == 401

    def test_protected_endpoint_with_invalid_token(self, client):
        """Test accessing protected endpoint with invalid token returns 401."""
        response = client.get(
            "/api/auth/me",
            headers={"Authorization": "Bearer invalid_token"}
        )
        assert response.status_code == 401
