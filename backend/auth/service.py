"""
Authentication services for User Authentication System.

This module provides user registration, sign-in, and session management.
"""

import re
from datetime import datetime
from typing import Optional
from sqlmodel import Session, select

from .models import User, UserPreferences, Session as SessionModel
from .security import (
    hash_password,
    verify_password,
    validate_password_strength,
    create_access_token,
    hash_token,
)
from .exceptions import (
    DuplicateEmailError,
    WeakPasswordError,
    InvalidCredentialsError,
    UserNotFoundError,
)


def is_valid_email(email: str) -> bool:
    """
    Validate email format using regex.

    Args:
        email: Email address to validate

    Returns:
        True if email format is valid, False otherwise
    """
    pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
    return re.match(pattern, email) is not None


def get_user_by_email(db_session: Session, email: str) -> Optional[User]:
    """
    Get a user by email address.

    Args:
        db_session: Database session
        email: User email address

    Returns:
        User object if found, None otherwise
    """
    statement = select(User).where(User.email == email, User.is_deleted == False)
    result = db_session.exec(statement).first()
    return result


def get_user_by_id(db_session: Session, user_id: int) -> Optional[User]:
    """
    Get a user by ID.

    Args:
        db_session: Database session
        user_id: User ID

    Returns:
        User object if found, None otherwise
    """
    return db_session.get(User, user_id)


def create_user_preferences(db_session: Session, user_id: int) -> UserPreferences:
    """
    Create default preferences for a new user.

    Args:
        db_session: Database session
        user_id: User ID

    Returns:
        Created UserPreferences object
    """
    preferences = UserPreferences(user_id=user_id)
    db_session.add(preferences)
    db_session.commit()
    db_session.refresh(preferences)
    return preferences


def register_user(
    db_session: Session,
    email: str,
    password: str,
    full_name: Optional[str] = None
) -> User:
    """
    Register a new user account.

    Args:
        db_session: Database session
        email: User email address
        password: User password
        full_name: Optional user full name

    Returns:
        Created User object

    Raises:
        DuplicateEmailError: If email is already registered
        WeakPasswordError: If password doesn't meet requirements
        ValueError: If email format is invalid
    """
    # Validate email format
    if not is_valid_email(email):
        raise ValueError("Invalid email format")

    # Check if user already exists
    existing = get_user_by_email(db_session, email)
    if existing:
        raise DuplicateEmailError(email)

    # Validate password strength
    is_valid, errors = validate_password_strength(password)
    if not is_valid:
        raise WeakPasswordError(errors)

    # Hash password
    password_hash = hash_password(password)

    # Create user
    user = User(
        email=email,
        password_hash=password_hash,
        full_name=full_name,
    )
    db_session.add(user)
    db_session.commit()
    db_session.refresh(user)

    # Create default preferences
    create_user_preferences(db_session, user.id)

    return user


def authenticate_user(
    db_session: Session,
    email: str,
    password: str
) -> tuple[User, str]:
    """
    Authenticate a user with email and password.

    Args:
        db_session: Database session
        email: User email address
        password: User password

    Returns:
        Tuple of (User object, JWT token)

    Raises:
        InvalidCredentialsError: If credentials are invalid
    """
    # Get user by email
    user = get_user_by_email(db_session, email)
    if not user:
        raise InvalidCredentialsError()

    # Verify password
    if not verify_password(password, user.password_hash):
        raise InvalidCredentialsError()

    # Create JWT token
    token = create_access_token(user.id)

    return user, token


def create_session(
    db_session: Session,
    user_id: int,
    token: str,
    expires_at: datetime
) -> SessionModel:
    """
    Create a new user session.

    Args:
        db_session: Database session
        user_id: User ID
        token: JWT token
        expires_at: Token expiration datetime

    Returns:
        Created Session object
    """
    token_hash = hash_token(token)
    session = SessionModel(
        user_id=user_id,
        token_hash=token_hash,
        expires_at=expires_at
    )
    db_session.add(session)
    db_session.commit()
    db_session.refresh(session)
    return session


def invalidate_session(db_session: Session, token: str) -> bool:
    """
    Invalidate a user session (sign out).

    Args:
        db_session: Database session
        token: JWT token to invalidate

    Returns:
        True if session was invalidated, False otherwise
    """
    token_hash = hash_token(token)
    statement = select(SessionModel).where(
        SessionModel.token_hash == token_hash,
        SessionModel.is_active == True
    )
    session = db_session.exec(statement).first()

    if session:
        session.is_active = False
        db_session.commit()
        return True

    return False


def invalidate_all_user_sessions(db_session: Session, user_id: int) -> int:
    """
    Invalidate all sessions for a user.

    Args:
        db_session: Database session
        user_id: User ID

    Returns:
        Number of sessions invalidated
    """
    statement = select(SessionModel).where(
        SessionModel.user_id == user_id,
        SessionModel.is_active == True
    )
    sessions = db_session.exec(statement).all()

    count = 0
    for session in sessions:
        session.is_active = False
        count += 1

    db_session.commit()
    return count


def is_session_active(db_session: Session, token: str) -> bool:
    """
    Check if a session is still active.

    Args:
        db_session: Database session
        token: JWT token to check

    Returns:
        True if session is active, False otherwise
    """
    token_hash = hash_token(token)
    statement = select(SessionModel).where(
        SessionModel.token_hash == token_hash,
        SessionModel.is_active == True
    )
    session = db_session.exec(statement).first()

    if not session:
        return False

    # Check if session has expired
    if session.expires_at < datetime.utcnow():
        session.is_active = False
        db_session.commit()
        return False

    return True


__all__ = [
    "is_valid_email",
    "get_user_by_email",
    "get_user_by_id",
    "create_user_preferences",
    "register_user",
    "authenticate_user",
    "create_session",
    "invalidate_session",
    "invalidate_all_user_sessions",
    "is_session_active",
]
