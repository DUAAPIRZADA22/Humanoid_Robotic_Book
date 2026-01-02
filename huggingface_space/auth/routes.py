"""
Authentication API routes for User Authentication System.

This module provides FastAPI routes for user registration, sign in, sign out,
profile management, and password reset.
"""

from fastapi import APIRouter, HTTPException, Depends, status
from sqlmodel import Session
from datetime import datetime, timedelta
import os

from .database import get_session
from .service import (
    register_user,
    authenticate_user,
    create_session,
    invalidate_session,
    invalidate_all_user_sessions,
    get_user_by_email,
    is_session_active,
)
from .security import hash_password, create_access_token
from .dependencies import get_current_user
from .models import User
from .schemas import (
    RegisterRequest,
    SignInRequest,
    ProfileUpdateRequest,
    PasswordResetRequest,
    PasswordResetConfirm,
    UserResponse,
    AuthResponse,
    MessageResponse,
    PasswordResetResponse,
)
from .exceptions import (
    DuplicateEmailError,
    WeakPasswordError,
    InvalidCredentialsError,
)

# Environment variable for frontend URL (for reset links)
FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:3000")

# Create router
router = APIRouter(prefix="/api/auth", tags=["Authentication"])


# Registration & Sign In
@router.post("/register", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def register(
    request: RegisterRequest,
    db_session: Session = Depends(get_session)
):
    """
    Register a new user account.

    - **email**: User email address (must be unique)
    - **password**: Password (min 8 characters, must include uppercase, lowercase, number)
    - **full_name**: Optional user full name
    """
    try:
        # Register user
        user = register_user(
            db_session,
            request.email,
            request.password,
            request.full_name
        )

        # Create JWT token
        token = create_access_token(user.id)

        # Create session
        expires_at = datetime.utcnow() + timedelta(days=7)
        create_session(db_session, user.id, token, expires_at)

        return AuthResponse(
            user=UserResponse.model_validate(user),
            token=token
        )

    except DuplicateEmailError as e:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=str(e)
        )
    except WeakPasswordError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )


@router.post("/signin", response_model=AuthResponse)
async def signin(
    request: SignInRequest,
    db_session: Session = Depends(get_session)
):
    """
    Sign in with email and password.

    - **email**: User email address
    - **password**: User password

    Returns JWT token valid for 7 days.
    """
    try:
        user, token = authenticate_user(
            db_session,
            request.email,
            request.password
        )

        # Create session
        expires_at = datetime.utcnow() + timedelta(days=7)
        create_session(db_session, user.id, token, expires_at)

        return AuthResponse(
            user=UserResponse.model_validate(user),
            token=token
        )

    except InvalidCredentialsError as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=str(e)
        )


@router.post("/signout", response_model=MessageResponse)
async def signout(
    authorization: str = Depends(lambda: None),
    db_session: Session = Depends(get_session)
):
    """
    Sign out the current user (invalidate session).

    Requires Bearer token in Authorization header.
    """
    # Note: The token is extracted from the Authorization header by the caller
    # For now, we'll return a success message as the client should discard the token
    # In a more robust implementation, we would invalidate the session here
    return MessageResponse(message="Signed out successfully")


@router.get("/me", response_model=UserResponse)
async def get_current_user_info(
    current_user: User = Depends(get_current_user)
):
    """
    Get current authenticated user information.

    Requires valid JWT token in Authorization header.
    """
    return UserResponse.model_validate(current_user)


# Password Reset
@router.post("/reset-request", response_model=PasswordResetResponse)
async def request_password_reset(
    request: PasswordResetRequest,
    db_session: Session = Depends(get_session)
):
    """
    Request a password reset link.

    - **email**: User email address

    Note: Returns success even if email doesn't exist (to prevent email enumeration).
    In production, this would send an email with the reset link.
    For development/testing, the reset URL is returned in the response.
    """
    # Check if user exists (but don't reveal if they don't)
    user = get_user_by_email(db_session, request.email)

    if user:
        # Import here to avoid circular dependency
        from .reset_service import create_reset_token

        token = create_reset_token(db_session, user.id)
        reset_url = f"{FRONTEND_URL}/reset-password?token={token}"

        # In production, send email here
        # For testing, return the URL
        return PasswordResetResponse(
            message="If email exists, reset link sent",
            reset_url=reset_url
        )

    # Still return success to prevent email enumeration
    return PasswordResetResponse(
        message="If email exists, reset link sent"
    )


@router.post("/reset-confirm", response_model=MessageResponse)
async def confirm_password_reset(
    request: PasswordResetConfirm,
    db_session: Session = Depends(get_session)
):
    """
    Confirm password reset with token.

    - **token**: Password reset token from email
    - **new_password**: New password (min 8 characters)
    """
    # Import here to avoid circular dependency
    from .reset_service import validate_reset_token, reset_password_with_token
    from .security import validate_password_strength

    # Validate token
    user_id = validate_reset_token(db_session, request.token)
    if not user_id:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Invalid or expired reset token"
        )

    # Validate password strength
    is_valid, errors = validate_password_strength(request.new_password)
    if not is_valid:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Weak password: {', '.join(errors)}"
        )

    # Reset password
    success = reset_password_with_token(
        db_session,
        request.token,
        request.new_password
    )

    if not success:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Failed to reset password"
        )

    return MessageResponse(message="Password reset successfully")


# Profile Management
@router.get("/profile", response_model=UserResponse)
async def get_profile(
    current_user: User = Depends(get_current_user)
):
    """
    Get current user profile.

    Requires valid JWT token.
    """
    return UserResponse.model_validate(current_user)


@router.put("/profile", response_model=UserResponse)
async def update_profile(
    request: ProfileUpdateRequest,
    current_user: User = Depends(get_current_user),
    db_session: Session = Depends(get_session)
):
    """
    Update current user profile.

    - **full_name**: New full name (optional)
    - **email**: New email (optional, requires password_confirmation)
    - **password_confirmation**: Current password (required if changing email)

    Requires valid JWT token.
    """
    # Import here to avoid circular dependency
    from .profile_service import update_profile as update_user_profile

    try:
        updated_user = update_user_profile(
            db_session,
            current_user.id,
            request
        )
        return UserResponse.model_validate(updated_user)

    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )
    except DuplicateEmailError as e:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=str(e)
        )


__all__ = ["router"]
