"""
Authentication package for User Authentication System.

This package provides authentication services including:
- User registration and sign-in
- JWT token management
- Password reset
- Profile management

NOTE: This module is OPTIONAL and requires PostgreSQL database.
If sqlmodel/psycopg2 are not installed, the package will be disabled.
"""

# Try to import all auth components - fail gracefully if dependencies missing
try:
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

    # Indicate auth is available
    AUTH_AVAILABLE = True

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

except ImportError as e:
    # Auth dependencies not available - provide empty exports
    AUTH_AVAILABLE = False

    # Provide no-op stubs for any imports
    User = None
    UserPreferences = None
    Session = None
    PasswordResetToken = None
    hash_password = None
    verify_password = None
    validate_password_strength = None
    create_access_token = None
    decode_access_token = None
    register_user = None
    authenticate_user = None
    get_user_by_email = None
    get_user_by_id = None
    create_session = None
    invalidate_session = None
    get_current_user = None
    get_current_user_optional = None
    AuthenticationError = None
    RegistrationError = None
    InvalidTokenError = None
    UserNotFoundError = None

    __all__ = []
