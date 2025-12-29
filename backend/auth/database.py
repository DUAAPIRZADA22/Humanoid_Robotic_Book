"""
Database session management for Authentication System.

This module provides database session creation and management.
NOTE: This module requires sqlmodel and psycopg2 to be installed.
If these dependencies are missing, the module will be disabled.
"""

import os
from os import getenv

# Try to import database dependencies
try:
    from sqlmodel import Session, create_engine
    DB_AVAILABLE = True
except ImportError:
    DB_AVAILABLE = False
    Session = None
    create_engine = None


# Database URL from environment variable
DATABASE_URL = getenv(
    "DATABASE_URL",
    "postgresql://user:password@localhost/humanoid_robotic_book"
)

# Global engine (created only when dependencies are available)
engine = None


def _get_engine():
    """Get or create the database engine (lazy initialization)."""
    global engine
    if not DB_AVAILABLE:
        raise RuntimeError("Database dependencies (sqlmodel/psycopg2) not installed")
    if engine is None:
        engine = create_engine(
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
    return engine


def get_session():
    """
    Get a database session.

    Yields:
        Database session

    Raises:
        RuntimeError: If database dependencies are not available
    """
    if not DB_AVAILABLE:
        raise RuntimeError("Database dependencies (sqlmodel/psycopg2) not installed")

    from sqlmodel import Session as _Session
    _engine = _get_engine()
    with _Session(_engine) as session:
        yield session


def init_db():
    """
    Initialize database tables.

    This function creates all tables if they don't exist.

    Raises:
        RuntimeError: If database dependencies are not available
    """
    if not DB_AVAILABLE:
        raise RuntimeError("Database dependencies (sqlmodel/psycopg2) not installed")

    try:
        from .models import SQLModel
        _engine = _get_engine()
        SQLModel.metadata.create_all(_engine)
    except ImportError:
        raise RuntimeError("Auth models not available")
