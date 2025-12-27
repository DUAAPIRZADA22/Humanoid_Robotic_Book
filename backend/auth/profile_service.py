"""
Profile management service for User Authentication System.

This module provides user profile update functionality.
"""

from datetime import datetime
from sqlmodel import Session, select

from .models import User
from .service import get_user_by_id, get_user_by_email
from .security import verify_password
from .exceptions import DuplicateEmailError
from .schemas import ProfileUpdateRequest


def update_profile(
    db_session: Session,
    user_id: int,
    data: ProfileUpdateRequest
) -> User:
    """
    Update user profile information.

    Args:
        db_session: Database session
        user_id: User ID to update
        data: Profile update request data

    Returns:
        Updated User object

    Raises:
        ValueError: If password confirmation is required but invalid
        DuplicateEmailError: If email is already in use
    """
    user = get_user_by_id(db_session, user_id)
    if not user:
        raise ValueError("User not found")

    # Update full name if provided
    if data.full_name is not None:
        user.full_name = data.full_name

    # Update email if provided
    if data.email is not None and data.email != user.email:
        # Require password confirmation for email change
        if not data.password_confirmation:
            raise ValueError("Password confirmation required for email change")

        # Verify password
        if not verify_password(data.password_confirmation, user.password_hash):
            raise ValueError("Invalid password")

        # Check email uniqueness
        existing = get_user_by_email(db_session, data.email)
        if existing:
            raise DuplicateEmailError(data.email)

        user.email = data.email

    # Update timestamp
    user.updated_at = datetime.utcnow()

    db_session.commit()
    db_session.refresh(user)

    return user


def change_password(
    db_session: Session,
    user_id: int,
    current_password: str,
    new_password: str
) -> bool:
    """
    Change user password.

    Args:
        db_session: Database session
        user_id: User ID
        current_password: Current password for verification
        new_password: New password to set

    Returns:
        True if password was changed successfully

    Raises:
        ValueError: If current password is invalid
    """
    from .security import hash_password, validate_password_strength

    user = get_user_by_id(db_session, user_id)
    if not user:
        raise ValueError("User not found")

    # Verify current password
    if not verify_password(current_password, user.password_hash):
        raise ValueError("Invalid current password")

    # Validate new password strength
    is_valid, errors = validate_password_strength(new_password)
    if not is_valid:
        raise ValueError(f"Weak password: {', '.join(errors)}")

    # Hash new password
    password_hash = hash_password(new_password)
    user.password_hash = password_hash
    user.updated_at = datetime.utcnow()

    db_session.commit()

    # Invalidate all existing sessions for security
    from .service import invalidate_all_user_sessions
    invalidate_all_user_sessions(db_session, user_id)

    return True


def delete_account(db_session: Session, user_id: int, password: str) -> bool:
    """
    Soft delete user account.

    Args:
        db_session: Database session
        user_id: User ID to delete
        password: Password for verification

    Returns:
        True if account was deleted successfully

    Raises:
        ValueError: If password is invalid
    """
    user = get_user_by_id(db_session, user_id)
    if not user:
        raise ValueError("User not found")

    # Verify password
    if not verify_password(password, user.password_hash):
        raise ValueError("Invalid password")

    # Soft delete (mark as deleted)
    user.is_deleted = True
    user.updated_at = datetime.utcnow()

    # Add deletion marker to email to allow reuse
    user.email = f"deleted_{user_id}_{user.email}"

    db_session.commit()

    # Invalidate all sessions
    from .service import invalidate_all_user_sessions
    invalidate_all_user_sessions(db_session, user_id)

    return True


__all__ = [
    "update_profile",
    "change_password",
    "delete_account",
]
