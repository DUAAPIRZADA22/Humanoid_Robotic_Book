"""
Password reset service for User Authentication System.

This module provides secure password reset token generation and validation.
"""

from datetime import datetime, timedelta
from typing import Optional
import secrets
from sqlmodel import Session, select

from .models import PasswordResetToken, User
from .security import hash_token, hash_password
from .service import get_user_by_id


def create_reset_token(db_session: Session, user_id: int) -> str:
    """
    Create a secure password reset token for a user.

    Args:
        db_session: Database session
        user_id: User ID

    Returns:
        Unhashed reset token (to be sent via email)
    """
    # Generate secure random token
    token = secrets.token_urlsafe(32)

    # Hash token for storage
    token_hash = hash_token(token)

    # Calculate expiration (1 hour from now)
    expires_at = datetime.utcnow() + timedelta(hours=1)

    # Create reset token record
    reset_token = PasswordResetToken(
        user_id=user_id,
        token_hash=token_hash,
        expires_at=expires_at
    )

    db_session.add(reset_token)
    db_session.commit()
    db_session.refresh(reset_token)

    return token


def validate_reset_token(db_session: Session, token: str) -> Optional[int]:
    """
    Validate a password reset token and return the user ID.

    Args:
        db_session: Database session
        token: Reset token to validate

    Returns:
        User ID if token is valid, None otherwise
    """
    # Hash the provided token
    token_hash = hash_token(token)

    # Look for the token in database
    statement = select(PasswordResetToken).where(
        PasswordResetToken.token_hash == token_hash,
        PasswordResetToken.used == False
    )
    reset_token = db_session.exec(statement).first()

    if not reset_token:
        return None

    # Check if token has expired
    if reset_token.expires_at < datetime.utcnow():
        return None

    # Return user ID
    return reset_token.user_id


def reset_password_with_token(
    db_session: Session,
    token: str,
    new_password: str
) -> bool:
    """
    Reset user password using a valid reset token.

    Args:
        db_session: Database session
        token: Valid reset token
        new_password: New password to set

    Returns:
        True if password was reset successfully, False otherwise
    """
    # Validate token and get user ID
    user_id = validate_reset_token(db_session, token)
    if not user_id:
        return False

    # Get user
    user = get_user_by_id(db_session, user_id)
    if not user:
        return False

    # Hash new password
    password_hash = hash_password(new_password)

    # Update user password
    user.password_hash = password_hash
    user.updated_at = datetime.utcnow()

    # Mark token as used
    token_hash = hash_token(token)
    statement = select(PasswordResetToken).where(
        PasswordResetToken.token_hash == token_hash
    )
    reset_token = db_session.exec(statement).first()
    if reset_token:
        reset_token.used = True

    db_session.commit()

    # Invalidate all existing sessions for security
    from .service import invalidate_all_user_sessions
    invalidate_all_user_sessions(db_session, user_id)

    return True


def cleanup_expired_tokens(db_session: Session, days: int = 7) -> int:
    """
    Clean up expired password reset tokens from the database.

    Args:
        db_session: Database session
        days: Delete tokens older than this many days

    Returns:
        Number of tokens deleted
    """
    cutoff_date = datetime.utcnow() - timedelta(days=days)

    statement = select(PasswordResetToken).where(
        PasswordResetToken.expires_at < cutoff_date
    )
    expired_tokens = db_session.exec(statement).all()

    count = 0
    for token in expired_tokens:
        db_session.delete(token)
        count += 1

    db_session.commit()
    return count


__all__ = [
    "create_reset_token",
    "validate_reset_token",
    "reset_password_with_token",
    "cleanup_expired_tokens",
]
