"""
Security utilities for User Authentication System.

This module provides password hashing, verification, and JWT token management.
"""

import os
import re
import bcrypt
import jwt
from datetime import datetime, timedelta
from typing import Optional, Tuple
from hashlib import sha256


# Configuration
SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-secret-key-change-in-production")
ALGORITHM = "HS256"
TOKEN_EXPIRY_DAYS = 7


def hash_password(password: str) -> str:
    """
    Hash a password using bcrypt.

    Args:
        password: Plain text password

    Returns:
        Hashed password string
    """
    salt = bcrypt.gensalt()
    return bcrypt.hashpw(password.encode('utf-8'), salt).decode('utf-8')


def verify_password(password: str, password_hash: str) -> bool:
    """
    Verify a password against its hash.

    Args:
        password: Plain text password to verify
        password_hash: Stored password hash

    Returns:
        True if password matches hash, False otherwise
    """
    return bcrypt.checkpw(password.encode('utf-8'), password_hash.encode('utf-8'))


def validate_password_strength(password: str) -> Tuple[bool, list[str]]:
    """
    Validate password strength against security requirements.

    Requirements:
    - Minimum 8 characters
    - At least one uppercase letter
    - At least one lowercase letter
    - At least one number

    Args:
        password: Password to validate

    Returns:
        Tuple of (is_valid, list_of_errors)
    """
    errors = []

    if len(password) < 8:
        errors.append("Password must be at least 8 characters")

    if not re.search(r'[A-Z]', password):
        errors.append("Password must contain at least one uppercase letter")

    if not re.search(r'[a-z]', password):
        errors.append("Password must contain at least one lowercase letter")

    if not re.search(r'[0-9]', password):
        errors.append("Password must contain at least one number")

    return len(errors) == 0, errors


def hash_token(token: str) -> str:
    """
    Hash a token for secure storage (e.g., reset tokens, session tokens).

    Args:
        token: Plain text token

    Returns:
        SHA-256 hash of the token
    """
    return sha256(token.encode('utf-8')).hexdigest()


def create_access_token(user_id: int, expiry_days: int = TOKEN_EXPIRY_DAYS) -> str:
    """
    Create a JWT access token for a user.

    Args:
        user_id: User ID to embed in token
        expiry_days: Token expiry in days (default: TOKEN_EXPIRY_DAYS)

    Returns:
        JWT token string
    """
    expires = datetime.utcnow() + timedelta(days=expiry_days)
    payload = {
        "user_id": user_id,
        "exp": expires,
        "iat": datetime.utcnow()
    }
    return jwt.encode(payload, SECRET_KEY, algorithm=ALGORITHM)


def decode_access_token(token: str) -> Optional[dict]:
    """
    Decode and validate a JWT access token.

    Args:
        token: JWT token string

    Returns:
        Token payload dict if valid, None if invalid/expired
    """
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        return None
    except jwt.InvalidTokenError:
        return None


def create_session_token() -> str:
    """
    Generate a secure random session token.

    Returns:
        Secure random token string
    """
    import secrets
    return secrets.token_urlsafe(32)


__all__ = [
    "hash_password",
    "verify_password",
    "validate_password_strength",
    "hash_token",
    "create_access_token",
    "decode_access_token",
    "create_session_token",
]
