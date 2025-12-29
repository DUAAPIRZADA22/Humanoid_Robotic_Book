"""
Authentication package for User Authentication System.

This package provides authentication services including:
- User registration and sign-in
- JWT token management
- Password reset
- Profile management
"""

from .models import User, UserPreferences, Session, PasswordResetToken
from .security import (
    hash_password,
    verify_password,
    validate_password_strength,
    create_access_token,
    decode_access_token,
)
from .service import (
    register_user,
    authenticate_user,
    get_user_by_email,
    get_user_by_id,
    create_session,
    invalidate_session,
)
from .dependencies import get_current_user, get_current_user_optional
from .exceptions import (
    AuthenticationError,
    RegistrationError,
    InvalidTokenError,
    UserNotFoundError,
)

__all__ = [
    # Models
    "User",
    "UserPreferences",
    "Session",
    "PasswordResetToken",
    # Security
    "hash_password",
    "verify_password",
    "validate_password_strength",
    "create_access_token",
    "decode_access_token",
    # Services
    "register_user",
    "authenticate_user",
    "get_user_by_email",
    "get_user_by_id",
    "create_session",
    "invalidate_session",
    # Dependencies
    "get_current_user",
    "get_current_user_optional",
    # Exceptions
    "AuthenticationError",
    "RegistrationError",
    "InvalidTokenError",
    "UserNotFoundError",
]
