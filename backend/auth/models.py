"""
SQLModel database models for User Authentication System.

This module defines all database models for user accounts, preferences,
sessions, and password reset tokens.

NOTE: Requires sqlmodel to be installed.
"""

from typing import Optional, List
from datetime import datetime
from enum import Enum

# Try to import SQLModel - fail gracefully if not available
try:
    from sqlmodel import SQLModel, Field, Relationship
    SQLMODEL_AVAILABLE = True
except ImportError:
    SQLMODEL_AVAILABLE = False
    # Create stub classes for type checking
    SQLModel = object
    Field = lambda **kwargs: None
    Relationship = lambda **kwargs: None


class User(SQLModel, table=True):
    """
    User account model.

    Represents a user account with email/password authentication.
    """
    __tablename__ = "users"

    id: Optional[int] = Field(default=None, primary_key=True)
    email: str = Field(unique=True, index=True, max_length=255)
    password_hash: str = Field(max_length=255)
    full_name: Optional[str] = Field(default=None, max_length=255)
    is_deleted: bool = Field(default=False)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    preferences: Optional["UserPreferences"] = Relationship(
        back_populates="user",
        sa_relationship_kwargs={"uselist": False, "cascade": "all, delete-orphan"}
    )
    sessions: List["Session"] = Relationship(
        back_populates="user",
        sa_relationship_kwargs={"cascade": "all, delete-orphan"}
    )
    reset_tokens: List["PasswordResetToken"] = Relationship(
        back_populates="user",
        sa_relationship_kwargs={"cascade": "all, delete-orphan"}
    )


class UserPreferences(SQLModel, table=True):
    """
    User preferences model.

    Stores user-specific settings and preferences.
    """
    __tablename__ = "user_preferences"

    id: Optional[int] = Field(default=None, primary_key=True)
    user_id: int = Field(foreign_key="users.id", unique=True, index=True)

    # Theme preferences
    theme: str = Field(default="light", max_length=20)  # light, dark, auto

    # Notification preferences
    email_notifications: bool = Field(default=True)
    chat_notifications: bool = Field(default=True)

    # Chat preferences
    chat_history_enabled: bool = Field(default=True)
    auto_save_chats: bool = Field(default=False)

    # Display preferences
    language: str = Field(default="en", max_length=10)
    timezone: str = Field(default="UTC", max_length=50)

    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationship
    user: User = Relationship(back_populates="preferences")


class Session(SQLModel, table=True):
    """
    User session model.

    Tracks active user sessions for JWT token validation.
    """
    __tablename__ = "sessions"

    id: Optional[int] = Field(default=None, primary_key=True)
    user_id: int = Field(foreign_key="users.id", index=True)
    token_hash: str = Field(max_length=255, index=True)
    expires_at: datetime
    created_at: datetime = Field(default_factory=datetime.utcnow)
    is_active: bool = Field(default=True)

    # Relationship
    user: User = Relationship(back_populates="sessions")


class PasswordResetToken(SQLModel, table=True):
    """
    Password reset token model.

    Stores secure tokens for password reset flow.
    """
    __tablename__ = "password_reset_tokens"

    id: Optional[int] = Field(default=None, primary_key=True)
    user_id: int = Field(foreign_key="users.id", index=True)
    token_hash: str = Field(max_length=255, unique=True, index=True)
    expires_at: datetime
    used: bool = Field(default=False)
    created_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationship
    user: User = Relationship(back_populates="reset_tokens")


# Export models for use in other modules
__all__ = [
    "User",
    "UserPreferences",
    "Session",
    "PasswordResetToken",
]
