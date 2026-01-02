"""
Unit tests for authentication service.
Tests user registration, authentication, and session management.
"""

import pytest
from datetime import datetime
from unittest.mock import Mock, patch
from sqlmodel import Session

from backend.auth.service import (
    register_user,
    authenticate_user,
    create_user_session,
    get_user_by_email,
    get_user_by_id,
)
from backend.auth.exceptions import (
    DuplicateEmailError,
    WeakPasswordError,
    InvalidCredentialsError,
    UserNotFoundError,
)


class TestUserRegistration:
    """Tests for user registration functionality."""

    @pytest.fixture
    def mock_db_session(self):
        """Create a mock database session."""
        session = Mock(spec=Session)
        return session

    def test_register_user_success(self, mock_db_session):
        """Test successful user registration."""
        with patch('backend.auth.service.get_user_by_email') as mock_get_user, \
             patch('backend.auth.service.hash_password') as mock_hash, \
             patch('backend.auth.service.add_user') as mock_add:

            mock_get_user.return_value = None
            mock_hash.return_value = "hashed_password"
            mock_add.return_value = Mock(id=1, email="test@example.com", full_name="Test User")

            user = register_user(
                mock_db_session,
                email="test@example.com",
                password="SecurePass123",
                full_name="Test User"
            )

            assert user.email == "test@example.com"
            assert user.full_name == "Test User"

    def test_register_user_duplicate_email(self, mock_db_session):
        """Test registration with duplicate email raises error."""
        with patch('backend.auth.service.get_user_by_email') as mock_get_user:
            mock_get_user.return_value = Mock(id=1, email="test@example.com")

            with pytest.raises(DuplicateEmailError):
                register_user(
                    mock_db_session,
                    email="test@example.com",
                    password="SecurePass123"
                )

    def test_register_user_weak_password(self, mock_db_session):
        """Test registration with weak password raises error."""
        with patch('backend.auth.service.get_user_by_email') as mock_get_user:
            mock_get_user.return_value = None

            with pytest.raises(WeakPasswordError):
                register_user(
                    mock_db_session,
                    email="test@example.com",
                    password="weak"
                )

    def test_register_user_invalid_email(self, mock_db_session):
        """Test registration with invalid email raises error."""
        with pytest.raises(ValueError):
            register_user(
                mock_db_session,
                email="notanemail",
                password="SecurePass123"
            )


class TestUserAuthentication:
    """Tests for user authentication functionality."""

    @pytest.fixture
    def mock_db_session(self):
        """Create a mock database session."""
        return Mock(spec=Session)

    def test_authenticate_user_success(self, mock_db_session):
        """Test successful authentication."""
        mock_user = Mock(
            id=1,
            email="test@example.com",
            password_hash="hashed_password"
        )

        with patch('backend.auth.service.get_user_by_email') as mock_get_user, \
             patch('backend.auth.service.verify_password') as mock_verify:

            mock_get_user.return_value = mock_user
            mock_verify.return_value = True

            result = authenticate_user(
                mock_db_session,
                email="test@example.com",
                password="CorrectPassword123"
            )

            assert result == mock_user

    def test_authenticate_user_wrong_password(self, mock_db_session):
        """Test authentication with wrong password."""
        mock_user = Mock(
            id=1,
            email="test@example.com",
            password_hash="hashed_password"
        )

        with patch('backend.auth.service.get_user_by_email') as mock_get_user, \
             patch('backend.auth.service.verify_password') as mock_verify:

            mock_get_user.return_value = mock_user
            mock_verify.return_value = False

            with pytest.raises(InvalidCredentialsError):
                authenticate_user(
                    mock_db_session,
                    email="test@example.com",
                    password="WrongPassword123"
                )

    def test_authenticate_user_not_found(self, mock_db_session):
        """Test authentication with non-existent user."""
        with patch('backend.auth.service.get_user_by_email') as mock_get_user:
            mock_get_user.return_value = None

            with pytest.raises(InvalidCredentialsError):
                authenticate_user(
                    mock_db_session,
                    email="nonexistent@example.com",
                    password="Password123"
                )


class TestSessionManagement:
    """Tests for session management functionality."""

    @pytest.fixture
    def mock_db_session(self):
        """Create a mock database session."""
        return Mock(spec=Session)

    def test_create_session_success(self, mock_db_session):
        """Test successful session creation."""
        mock_user = Mock(id=1, email="test@example.com")

        with patch('backend.auth.service.add_session') as mock_add:
            mock_session = Mock(
                id="session_id",
                user_id=1,
                token="access_token",
                expires_at=datetime.utcnow()
            )
            mock_add.return_value = mock_session

            session = create_user_session(
                mock_db_session,
                user=mock_user,
                token="access_token"
            )

            assert session.user_id == 1
            assert session.token == "access_token"


class TestUserRetrieval:
    """Tests for user retrieval functions."""

    @pytest.fixture
    def mock_db_session(self):
        """Create a mock database session."""
        return Mock(spec=Session)

    def test_get_user_by_email_found(self, mock_db_session):
        """Test retrieving user by email when found."""
        mock_user = Mock(id=1, email="test@example.com")

        with patch('backend.auth.service._exec_query') as mock_exec:
            mock_exec.return_value.first.return_value = mock_user

            result = get_user_by_email(mock_db_session, "test@example.com")
            assert result == mock_user

    def test_get_user_by_email_not_found(self, mock_db_session):
        """Test retrieving user by email when not found."""
        with patch('backend.auth.service._exec_query') as mock_exec:
            mock_exec.return_value.first.return_value = None

            result = get_user_by_email(mock_db_session, "nonexistent@example.com")
            assert result is None

    def test_get_user_by_id_found(self, mock_db_session):
        """Test retrieving user by ID when found."""
        mock_user = Mock(id=1, email="test@example.com")

        with patch('backend.auth.service._exec_query') as mock_exec:
            mock_exec.return_value.first.return_value = mock_user

            result = get_user_by_id(mock_db_session, 1)
            assert result == mock_user

    def test_get_user_by_id_not_found(self, mock_db_session):
        """Test retrieving user by ID when not found."""
        with patch('backend.auth.service._exec_query') as mock_exec:
            mock_exec.return_value.first.return_value = None

            result = get_user_by_id(mock_db_session, 999)
            assert result is None

    def test_get_user_by_id_with_deleted(self, mock_db_session):
        """Test retrieving deleted user is not returned."""
        mock_user = Mock(id=1, email="test@example.com", is_deleted=True)

        with patch('backend.auth.service._exec_query') as mock_exec:
            mock_exec.return_value.first.return_value = mock_user

            result = get_user_by_id(mock_db_session, 1)
            # Should return None for deleted users
            assert result is None


class TestPasswordResetService:
    """Tests for password reset service functionality."""

    @pytest.fixture
    def mock_db_session(self):
        """Create a mock database session."""
        return Mock(spec=Session)

    def test_create_reset_token_for_user(self, mock_db_session):
        """Test creating a password reset token for a user."""
        from backend.auth.reset_service import create_reset_token

        mock_user = Mock(id=1, email="test@example.com")

        with patch('backend.auth.reset_service.add_reset_token') as mock_add, \
             patch('backend.auth.security.generate_reset_token') as mock_generate:

            mock_generate.return_value = "reset_token_123"
            mock_add.return_value = Mock(
                token="reset_token_123",
                user_id=1,
                expires_at=datetime.utcnow()
            )

            token = create_reset_token(mock_db_session, mock_user)
            assert token.token == "reset_token_123"

    def test_validate_reset_token_valid(self, mock_db_session):
        """Test validating a valid reset token."""
        from backend.auth.reset_service import validate_reset_token

        with patch('backend.auth.reset_service.get_valid_reset_token') as mock_get:
            mock_token = Mock(
                token="valid_token",
                user_id=1,
                expires_at=datetime.utcnow()
            )
            mock_get.return_value = mock_token

            result = validate_reset_token(mock_db_session, "valid_token")
            assert result == mock_token

    def test_reset_user_password(self, mock_db_session):
        """Test resetting a user's password."""
        from backend.auth.reset_service import reset_user_password

        mock_user = Mock(id=1, email="test@example.com")

        with patch('backend.auth.reset_service.hash_password') as mock_hash, \
             patch('backend.auth.reset_service.invalidate_reset_token') as mock_invalidate, \
             patch('backend.auth.reset_service.update_user') as mock_update:

            mock_hash.return_value = "new_hashed_password"

            reset_user_password(mock_db_session, mock_user, "NewPassword123")

            mock_update.assert_called_once()
