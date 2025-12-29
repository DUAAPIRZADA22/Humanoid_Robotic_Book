"""
FastAPI dependencies for authentication.

This module provides dependency functions for protecting routes with JWT authentication.
"""

from typing import Optional
from fastapi import Header, HTTPException, Depends, status
from sqlmodel import Session

from .database import get_session
from .security import decode_access_token
from .service import get_user_by_id
from .models import User
from .exceptions import InvalidTokenError, UserNotFoundError


async def get_current_user(
    authorization: str = Header(...),
    db_session: Session = Depends(get_session)
) -> User:
    """
    Dependency to get the current authenticated user from JWT token.

    Args:
        authorization: Authorization header (Bearer token)
        db_session: Database session

    Returns:
        Authenticated User object

    Raises:
        HTTPException: If token is invalid, expired, or user not found (401)
    """
    # Validate token format
    if not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token format. Use 'Bearer <token>'",
        )

    token = authorization[7:]  # Remove "Bearer " prefix

    # Decode token
    payload = decode_access_token(token)
    if not payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    # Get user from database
    user_id = payload.get("user_id")
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload",
        )

    user = get_user_by_id(db_session, user_id)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found",
        )

    # Check if user is deleted
    if user.is_deleted:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User account has been deleted",
        )

    return user


async def get_current_user_optional(
    authorization: Optional[str] = Header(None),
    db_session: Session = Depends(get_session)
) -> Optional[User]:
    """
    Optional dependency to get the current user if authenticated.

    Unlike get_current_user, this returns None instead of raising an error
    if no valid token is provided.

    Args:
        authorization: Authorization header (optional)
        db_session: Database session

    Returns:
        User object if authenticated, None otherwise
    """
    if not authorization:
        return None

    if not authorization.startswith("Bearer "):
        return None

    token = authorization[7:]

    payload = decode_access_token(token)
    if not payload:
        return None

    user_id = payload.get("user_id")
    if not user_id:
        return None

    user = get_user_by_id(db_session, user_id)
    if not user or user.is_deleted:
        return None

    return user


__all__ = [
    "get_current_user",
    "get_current_user_optional",
]
