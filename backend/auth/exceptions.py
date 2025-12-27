"""
Custom exceptions for User Authentication System.

This module defines all custom exceptions used throughout the auth system.
"""


class AuthenticationError(Exception):
    """Base exception for authentication errors."""

    def __init__(self, message: str = "Authentication failed"):
        self.message = message
        super().__init__(self.message)


class RegistrationError(Exception):
    """Exception raised when user registration fails."""

    def __init__(self, message: str = "Registration failed"):
        self.message = message
        super().__init__(self.message)


class InvalidTokenError(AuthenticationError):
    """Exception raised when a token is invalid or expired."""

    def __init__(self, message: str = "Invalid or expired token"):
        self.message = message
        super().__init__(self.message)


class UserNotFoundError(AuthenticationError):
    """Exception raised when a user is not found."""

    def __init__(self, message: str = "User not found"):
        self.message = message
        super().__init__(self.message)


class WeakPasswordError(RegistrationError):
    """Exception raised when password doesn't meet strength requirements."""

    def __init__(self, errors: list[str]):
        self.errors = errors
        message = f"Weak password: {', '.join(errors)}"
        super().__init__(message)


class DuplicateEmailError(RegistrationError):
    """Exception raised when attempting to register with an existing email."""

    def __init__(self, email: str):
        self.email = email
        message = f"Email '{email}' is already registered"
        super().__init__(message)


class InvalidCredentialsError(AuthenticationError):
    """Exception raised when login credentials are invalid."""

    def __init__(self, message: str = "Invalid email or password"):
        self.message = message
        super().__init__(self.message)


__all__ = [
    "AuthenticationError",
    "RegistrationError",
    "InvalidTokenError",
    "UserNotFoundError",
    "WeakPasswordError",
    "DuplicateEmailError",
    "InvalidCredentialsError",
]
