"""
Pydantic schemas for Authentication API.

This module defines request and response models for authentication endpoints.
"""

from typing import Optional
from pydantic import BaseModel, EmailStr, Field


# Request schemas
class RegisterRequest(BaseModel):
    """Request model for user registration."""
    email: EmailStr
    password: str = Field(..., min_length=8)
    full_name: Optional[str] = None


class SignInRequest(BaseModel):
    """Request model for user sign in."""
    email: EmailStr
    password: str


class ProfileUpdateRequest(BaseModel):
    """Request model for updating user profile."""
    full_name: Optional[str] = None
    email: Optional[EmailStr] = None
    password_confirmation: Optional[str] = None


class PasswordResetRequest(BaseModel):
    """Request model for initiating password reset."""
    email: EmailStr


class PasswordResetConfirm(BaseModel):
    """Request model for confirming password reset."""
    token: str
    new_password: str = Field(..., min_length=8)


# Response schemas
class UserResponse(BaseModel):
    """Response model for user data."""
    id: int
    email: str
    full_name: Optional[str] = None

    class Config:
        from_attributes = True


class AuthResponse(BaseModel):
    """Response model for authentication operations."""
    user: UserResponse
    token: str


class MessageResponse(BaseModel):
    """Generic message response."""
    message: str


class PasswordResetResponse(BaseModel):
    """Response model for password reset request."""
    message: str
    # For testing purposes, include the reset URL
    # In production, this would be sent via email
    reset_url: Optional[str] = None


__all__ = [
    "RegisterRequest",
    "SignInRequest",
    "ProfileUpdateRequest",
    "PasswordResetRequest",
    "PasswordResetConfirm",
    "UserResponse",
    "AuthResponse",
    "MessageResponse",
    "PasswordResetResponse",
]
