"""
Unit tests for security utilities.
Tests password hashing, JWT token creation/validation, and password validation.
"""

import pytest
from datetime import datetime, timedelta
from backend.auth.security import (
    hash_password,
    verify_password,
    create_access_token,
    decode_access_token,
    validate_password_strength,
    is_valid_email,
    generate_reset_token,
    validate_reset_token,
)


class TestPasswordHashing:
    """Tests for password hashing and verification."""

    def test_hash_password_returns_string(self):
        """Test that hash_password returns a string."""
        result = hash_password("test_password")
        assert isinstance(result, str)
        assert len(result) > 0

    def test_hash_password_different_hashes(self):
        """Test that hashing the same password twice produces different hashes (salt)."""
        password = "test_password"
        hash1 = hash_password(password)
        hash2 = hash_password(password)
        assert hash1 != hash2

    def test_verify_password_correct(self):
        """Test password verification with correct password."""
        password = "test_password"
        hashed = hash_password(password)
        assert verify_password(password, hashed) is True

    def test_verify_password_incorrect(self):
        """Test password verification with incorrect password."""
        password = "test_password"
        wrong_password = "wrong_password"
        hashed = hash_password(password)
        assert verify_password(wrong_password, hashed) is False


class TestJWTTokens:
    """Tests for JWT token creation and validation."""

    def test_create_access_token_returns_string(self):
        """Test that create_access_token returns a string."""
        token = create_access_token(user_id=1)
        assert isinstance(token, str)
        assert len(token) > 0

    def test_decode_valid_token(self):
        """Test decoding a valid token."""
        token = create_access_token(user_id=123)
        payload = decode_access_token(token)
        assert payload is not None
        assert payload["user_id"] == 123

    def test_decode_invalid_token(self):
        """Test decoding an invalid token."""
        payload = decode_access_token("invalid_token")
        assert payload is None

    def test_token_expiration(self):
        """Test that expired tokens are rejected."""
        # Create a token that expires immediately
        from backend.auth.security import jwt, SECRET_KEY, ALGORITHM

        expires = datetime.utcnow() - timedelta(seconds=1)
        payload = {"user_id": 1, "exp": expires, "iat": datetime.utcnow()}
        token = jwt.encode(payload, SECRET_KEY, algorithm=ALGORITHM)

        decoded = decode_access_token(token)
        assert decoded is None


class TestPasswordValidation:
    """Tests for password strength validation."""

    def test_valid_password(self):
        """Test a password that meets all requirements."""
        is_valid, errors = validate_password_strength("SecurePass123")
        assert is_valid is True
        assert len(errors) == 0

    def test_password_too_short(self):
        """Test password that is too short."""
        is_valid, errors = validate_password_strength("Short1")
        assert is_valid is False
        assert "at least 8 characters" in str(errors)

    def test_password_no_uppercase(self):
        """Test password without uppercase letter."""
        is_valid, errors = validate_password_strength("lowercase123")
        assert is_valid is False
        assert "uppercase letter" in str(errors)

    def test_password_no_lowercase(self):
        """Test password without lowercase letter."""
        is_valid, errors = validate_password_strength("UPPERCASE123")
        assert is_valid is False
        assert "lowercase letter" in str(errors)

    def test_password_no_number(self):
        """Test password without number."""
        is_valid, errors = validate_password_strength("NoNumbers")
        assert is_valid is False
        assert "number" in str(errors)

    def test_password_all_requirements(self):
        """Test password with all requirements."""
        is_valid, errors = validate_password_strength("VerySecure123!@#")
        assert is_valid is True


class TestEmailValidation:
    """Tests for email validation."""

    def test_valid_email(self):
        """Test valid email addresses."""
        assert is_valid_email("user@example.com") is True
        assert is_valid_email("test.user+tag@domain.co.uk") is True
        assert is_valid_email("simple@localhost") is True

    def test_invalid_email(self):
        """Test invalid email addresses."""
        assert is_valid_email("") is False
        assert is_valid_email("notanemail") is False
        assert is_valid_email("@example.com") is False
        assert is_valid_email("user@") is False
        assert is_valid_email("user..name@example.com") is False


class TestResetTokens:
    """Tests for password reset token generation and validation."""

    def test_generate_reset_token_returns_string(self):
        """Test that generate_reset_token returns a URL-safe string."""
        token = generate_reset_token()
        assert isinstance(token, str)
        assert len(token) > 0

    def test_reset_tokens_are_unique(self):
        """Test that reset tokens are unique."""
        token1 = generate_reset_token()
        token2 = generate_reset_token()
        assert token1 != token2

    def test_validate_reset_token_valid_format(self):
        """Test that properly formatted tokens are valid."""
        token = generate_reset_token()
        is_valid = validate_reset_token(token)
        assert is_valid is True

    def test_validate_reset_token_invalid_format(self):
        """Test that malformed tokens are rejected."""
        assert validate_reset_token("") is False
        assert validate_reset_token("invalid") is False
        assert validate_reset_token("token with spaces") is False
