"""
Database session management for Authentication System.

This module provides database session creation and management.
"""

from sqlmodel import Session, create_engine
from os import getenv
import os


# Database URL from environment variable
DATABASE_URL = getenv(
    "DATABASE_URL",
    "postgresql://user:password@localhost/humanoid_robotic_book"
)

# Global engine (lazy initialization - only created when needed)
_engine = None


def _get_engine():
    """Get or create the database engine (lazy initialization)."""
    global _engine
    if _engine is None:
        # Create database engine with pool settings for cloud databases
        # Neon PostgreSQL requires pool_pre_ping and shorter pool_recycle
        _engine = create_engine(
            DATABASE_URL,
            echo=False,
            pool_pre_ping=True,  # Verify connections before using
            pool_recycle=300,     # Recycle connections every 5 minutes
            pool_size=5,          # Smaller pool for Neon free tier
            max_overflow=10,
            connect_args={
                "sslmode": "require",
                "connect_timeout": 10
            }
        )
    return _engine


# Lazy engine descriptor for module-level access
class _LazyEngine:
    """Lazy engine descriptor - only creates engine when accessed."""
    def __get__(self, obj, objtype=None):
        return _get_engine()


# Module-level engine (lazy)
engine = _LazyEngine()


def get_session():
    """
    Get a database session.

    Yields:
        Database session
    """
    with Session(engine) as session:
        yield session


def init_db():
    """
    Initialize database tables.

    This function creates all tables if they don't exist.
    """
    from .models import SQLModel, User, UserPreferences, Session, PasswordResetToken

    SQLModel.metadata.create_all(engine)


__all__ = [
    "get_session",
    "init_db",
    "engine",
    "DATABASE_URL",
]
