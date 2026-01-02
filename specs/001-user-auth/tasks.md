# Implementation Tasks: User Authentication System

**Feature Branch**: 001-user-auth
**Date**: 2025-12-24
**Spec**: [spec.md](./spec.md)
**Status**: Ready for Implementation

## Summary

This document outlines all implementation tasks for the User Authentication System feature. Tasks are organized by user story priority and include dependencies, acceptance criteria, and testing requirements.

---

## Tech Stack

**Backend**:
- Python 3.9+
- FastAPI (web framework)
- Neon PostgreSQL (database)
- SQLModel (ORM)
- bcrypt (password hashing)
- PyJWT (JWT tokens)
- python-dotenv (environment configuration)

**Frontend**:
- TypeScript 5.x
- React 18
- Docusaurus 3.9
- Tailwind CSS
- crypto-js (AES-256 encryption for localStorage)

**Infrastructure**:
- Railway (deployment)
- Qdrant (existing vector store - separate from auth)

---

## Task Legend

| Symbol | Meaning |
|--------|---------|
| [US1] | User Story 1 - New User Registration |
| [US2] | User Story 2 - Existing User Sign In |
| [US3] | User Story 3 - Password Reset |
| [US4] | User Story 4 - Profile Management |
| [US5] | User Story 5 - Protected Chat Access |
| [BE]  | Backend Task |
| [FE]  | Frontend Task |
| [DB]  | Database Task |
| [TEST] | Testing Task |

---

## Phase 1: Foundation & Database Setup

### Task 1.1: Set up Neon PostgreSQL Database [DB]

**Priority**: P0 (Blocker)
**Estimated Complexity**: Medium
**Dependencies**: None

**Description**:
- Create Neon PostgreSQL project/instance
- Set up environment variables for database connection
- Verify database connectivity from backend

**Acceptance Criteria**:
- [ ] Neon PostgreSQL project created
- [ ] Database connection URL available in environment
- [ ] Connection test successful from backend
- [ ] Environment variables documented in `.env.example`

**Implementation Notes**:
```
Required environment variables:
- DATABASE_URL: PostgreSQL connection string
- NEON_API_KEY: Neon management API key (optional for migrations)
```

**Testing**:
- Manual: Verify connection using Python script
- Automated: Health check endpoint returns database status

---

### Task 1.2: Create Database Models with SQLModel [DB][BE]

**Priority**: P0 (Blocker)
**Estimated Complexity**: Medium
**Dependencies**: Task 1.1

**Description**:
- Define SQLModel schemas for User, UserPreferences, Session, PasswordResetToken
- Set up relationships between models
- Add timestamps and soft delete support

**Acceptance Criteria**:
- [ ] `User` model with fields: id, email, password_hash, full_name, created_at, updated_at
- [ ] `UserPreferences` model with fields: id, user_id, theme, notifications, etc.
- [ ] `Session` model with fields: id, user_id, token_hash, expires_at, created_at
- [ ] `PasswordResetToken` model with fields: id, user_id, token_hash, expires_at, used
- [ ] All models have proper relationships defined
- [ ] Soft delete support with `is_deleted` flag

**Implementation Notes**:
```python
# backend/auth/models.py (new file)
from typing import Optional
from datetime import datetime
from sqlmodel import SQLModel, Field, Relationship
from enum import Enum

class User(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    email: str = Field(unique=True, index=True)
    password_hash: str
    full_name: Optional[str] = None
    is_deleted: bool = False
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    # Relationships
    preferences: Optional["UserPreferences"] = Relationship(back_populates="user")
    sessions: list["Session"] = Relationship(back_populates="user")
    reset_tokens: list["PasswordResetToken"] = Relationship(back_populates="user")
```

**Testing**:
- Unit tests for model validation
- Test relationship queries

---

### Task 1.3: Create Database Migration Scripts [DB]

**Priority**: P0 (Blocker)
**Estimated Complexity**: Medium
**Dependencies**: Task 1.2

**Description**:
- Create initial migration script to set up tables
- Create rollback migration
- Test migration on local database first

**Acceptance Criteria**:
- [ ] `migrations/001_initial_schema.sql` creates all tables
- [ ] `migrations/001_rollback.sql` drops all tables safely
- [ ] Migration tested on Neon PostgreSQL
- [ ] Migration script includes indexes for performance

**Implementation Notes**:
```sql
-- migrations/001_initial_schema.sql
CREATE TABLE IF NOT EXISTS users (
    id SERIAL PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    full_name VARCHAR(255),
    is_deleted BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_is_deleted ON users(is_deleted);

-- ... additional tables
```

**Testing**:
- Run migration forward and backward
- Verify all tables and indexes created

---

## Phase 2: Authentication Backend (P1 User Stories)

### Task 2.1: Implement Password Hashing Utilities [BE][US1][US2]

**Priority**: P1
**Estimated Complexity**: Low
**Dependencies**: None

**Description**:
- Create password hashing utility using bcrypt
- Implement password validation helper

**Acceptance Criteria**:
- [ ] `hash_password(password: str) -> str` function
- [ ] `verify_password(password: str, hash: str) -> bool` function
- [ ] `validate_password_strength(password: str) -> tuple[bool, list[str]]` function
- [ ] Minimum 8 characters, uppercase, lowercase, number required

**Implementation Notes**:
```python
# backend/auth/security.py (new file)
import bcrypt
import re

def hash_password(password: str) -> str:
    salt = bcrypt.gensalt()
    return bcrypt.hashpw(password.encode('utf-8'), salt).decode('utf-8')

def verify_password(password: str, password_hash: str) -> bool:
    return bcrypt.checkpw(password.encode('utf-8'), password_hash.encode('utf-8'))
```

**Testing**:
- Unit tests for hashing and verification
- Test password strength validation with various inputs

---

### Task 2.2: Implement JWT Token Management [BE][US1][US2]

**Priority**: P1
**Estimated Complexity**: Low
**Dependencies**: None

**Description**:
- Create JWT token generation and validation
- Set up 7-day token expiry

**Acceptance Criteria**:
- [ ] `create_access_token(user_id: int) -> str` function
- [ ] `decode_access_token(token: str) -> dict | None` function
- [ ] Token expires after 7 days
- [ ] Token contains user_id and issued_at claims

**Implementation Notes**:
```python
# backend/auth/security.py (continued)
import jwt
from datetime import datetime, timedelta

SECRET_KEY = os.getenv("JWT_SECRET_KEY")
ALGORITHM = "HS256"

def create_access_token(user_id: int) -> str:
    expires = datetime.utcnow() + timedelta(days=7)
    payload = {"user_id": user_id, "exp": expires, "iat": datetime.utcnow()}
    return jwt.encode(payload, SECRET_KEY, algorithm=ALGORITHM)

def decode_access_token(token: str) -> Optional[dict]:
    try:
        return jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
    except jwt.PyJWTError:
        return None
```

**Testing**:
- Unit tests for token creation and decoding
- Test token expiration handling

---

### Task 2.3: Create User Registration Service [BE][US1]

**Priority**: P1
**Estimated Complexity**: Medium
**Dependencies**: Tasks 1.2, 1.3, 2.1

**Description**:
- Implement user registration logic
- Validate email format and uniqueness
- Create default user preferences

**Acceptance Criteria**:
- [ ] `register_user(email: str, password: str, full_name: str) -> User` function
- [ ] Email validation checks format and existing users
- [ ] Password is hashed before storage
- [ ] Default preferences created on registration
- [ ] Returns created User object

**Implementation Notes**:
```python
# backend/auth/service.py (new file)
def register_user(email: str, password: str, full_name: str) -> User:
    # Validate email format
    if not is_valid_email(email):
        raise ValueError("Invalid email format")

    # Check if user exists
    existing = get_user_by_email(email)
    if existing:
        raise ValueError("Email already registered")

    # Validate password
    valid, errors = validate_password_strength(password)
    if not valid:
        raise ValueError(f"Weak password: {', '.join(errors)}")

    # Hash password
    password_hash = hash_password(password)

    # Create user
    user = User(email=email, password_hash=password_hash, full_name=full_name)
    # ... save to database

    return user
```

**Testing**:
- Unit tests for successful registration
- Test duplicate email handling
- Test invalid email handling
- Test weak password handling

---

### Task 2.4: Create User Sign In Service [BE][US2]

**Priority**: P1
**Estimated Complexity**: Medium
**Dependencies**: Tasks 1.2, 1.3, 2.1, 2.2

**Description**:
- Implement sign in logic with email/password
- Generate JWT token on successful authentication
- Create session record

**Acceptance Criteria**:
- [ ] `authenticate_user(email: str, password: str) -> tuple[User, str]` function
- [ ] Returns User and JWT token on success
- [ ] Raises appropriate errors for invalid credentials
- [ ] Creates session record in database
- [ ] Handles non-existent users

**Implementation Notes**:
```python
# backend/auth/service.py (continued)
def authenticate_user(email: str, password: str) -> tuple[User, str]:
    user = get_user_by_email(email)
    if not user:
        raise AuthenticationError("Invalid credentials")

    if not verify_password(password, user.password_hash):
        raise AuthenticationError("Invalid credentials")

    # Create session
    token = create_access_token(user.id)
    create_session(user.id, token)

    return user, token
```

**Testing**:
- Unit tests for successful authentication
- Test invalid email handling
- Test invalid password handling
- Test token generation

---

### Task 2.5: Create Authentication API Endpoints [BE][US1][US2]

**Priority**: P1
**Estimated Complexity**: Medium
**Dependencies**: Tasks 2.3, 2.4

**Description**:
- Create FastAPI endpoints for registration and sign in
- Add request/response models with Pydantic
- Include proper error handling

**Acceptance Criteria**:
- [ ] `POST /api/auth/register` endpoint with RegisterRequest model
- [ ] `POST /api/auth/signin` endpoint with SignInRequest model
- [ ] `POST /api/auth/signout` endpoint to invalidate session
- [ ] `GET /api/auth/me` endpoint to get current user
- [ ] All endpoints return appropriate HTTP status codes
- [ ] Error responses include helpful messages

**Implementation Notes**:
```python
# backend/auth/routes.py (new file)
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, EmailStr

class RegisterRequest(BaseModel):
    email: EmailStr
    password: str
    full_name: str

class SignInRequest(BaseModel):
    email: EmailStr
    password: str

class AuthResponse(BaseModel):
    user: UserResponse
    token: str

@router.post("/register", response_model=AuthResponse)
async def register(request: RegisterRequest):
    try:
        user = register_user(request.email, request.password, request.full_name)
        token = create_access_token(user.id)
        return AuthResponse(user=user, token=token)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.post("/signin", response_model=AuthResponse)
async def signin(request: SignInRequest):
    try:
        user, token = authenticate_user(request.email, request.password)
        return AuthResponse(user=user, token=token)
    except AuthenticationError as e:
        raise HTTPException(status_code=401, detail=str(e))
```

**Testing**:
- Integration tests for all endpoints
- Test with valid and invalid inputs
- Verify HTTP status codes

---

### Task 2.6: Create Authentication Dependency for Protected Routes [BE][US5]

**Priority**: P1 (for US5)
**Estimated Complexity**: Low
**Dependencies**: Task 2.2

**Description**:
- Create FastAPI dependency to validate JWT tokens
- Return current user from token

**Acceptance Criteria**:
- [ ] `get_current_user(token: str) -> User` dependency function
- [ ] Returns User object if token valid
- [ ] Raises 401 if token invalid/expired
- [ ] Can be used with FastAPI's `Depends()`

**Implementation Notes**:
```python
# backend/auth/dependencies.py (new file)
from fastapi import Header, HTTPException, Depends

async def get_current_user(authorization: str = Header(...)) -> User:
    if not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Invalid token format")

    token = authorization[7:]  # Remove "Bearer " prefix
    payload = decode_access_token(token)

    if not payload:
        raise HTTPException(status_code=401, detail="Invalid token")

    user = get_user_by_id(payload["user_id"])
    if not user:
        raise HTTPException(status_code=401, detail="User not found")

    return user

# Usage:
# @app.get("/protected")
# async def protected_route(user: User = Depends(get_current_user)):
#     return {"message": f"Hello {user.email}"}
```

**Testing**:
- Test with valid token
- Test with invalid token
- Test with expired token
- Test with malformed header

---

## Phase 3: Password Reset (P2 User Story)

### Task 3.1: Create Password Reset Token Service [BE][US3]

**Priority**: P2
**Estimated Complexity**: Medium
**Dependencies**: Tasks 1.2, 1.3

**Description**:
- Implement secure token generation for password reset
- Store hashed tokens in database

**Acceptance Criteria**:
- [ ] `create_reset_token(user_id: str) -> str` function
- [ ] Token expires after 1 hour
- [ ] Token is securely hashed before storage
- [ ] `validate_reset_token(token: str) -> bool` function

**Implementation Notes**:
```python
# backend/auth/reset_service.py (new file)
import secrets
from datetime import datetime, timedelta

def create_reset_token(user_id: int) -> str:
    token = secrets.token_urlsafe(32)
    token_hash = hashlib.sha256(token.encode()).hexdigest()
    expires = datetime.utcnow() + timedelta(hours=1)

    # Store in database
    reset_token = PasswordResetToken(
        user_id=user_id,
        token_hash=token_hash,
        expires_at=expires
    )
    # ... save to database

    return token  # Return unhashed token for email
```

**Testing**:
- Test token generation
- Test token validation
- Test expired token handling

---

### Task 3.2: Create Password Reset API Endpoints [BE][US3]

**Priority**: P2
**Estimated Complexity**: Medium
**Dependencies**: Task 3.1

**Description**:
- Create endpoint to request password reset
- Create endpoint to reset password with token
- Return reset URL (email integration is out of scope per spec)

**Acceptance Criteria**:
- [ ] `POST /api/auth/reset-request` endpoint
- [ ] `POST /api/auth/reset-confirm` endpoint
- [ ] Reset request returns reset URL (or token for manual testing)
- [ ] Reset confirm validates token and updates password

**Implementation Notes**:
```python
@router.post("/reset-request")
async def request_reset(request: ResetRequest):
    user = get_user_by_email(request.email)
    if not user:
        # Still return success to prevent email enumeration
        return {"message": "If email exists, reset link sent"}

    token = create_reset_token(user.id)
    reset_url = f"{FRONTEND_URL}/reset-password?token={token}"

    # In production, send email here (out of scope)
    return {"reset_url": reset_url}  # For testing only

@router.post("/reset-confirm")
async def confirm_reset(request: ResetConfirmRequest):
    if not validate_reset_token(request.token):
        raise HTTPException(status_code=400, detail="Invalid or expired token")

    # Update password
    user = get_user_by_reset_token(request.token)
    user.password_hash = hash_password(request.new_password)
    # ... save

    return {"message": "Password reset successfully"}
```

**Testing**:
- Integration tests for reset flow
- Test invalid token handling
- Test expired token handling

---

## Phase 4: Profile Management (P3 User Story)

### Task 4.1: Create Profile Update Service [BE][US4]

**Priority**: P3
**Estimated Complexity**: Low
**Dependencies**: Task 1.2

**Description**:
- Implement profile update logic
- Handle email change with verification

**Acceptance Criteria**:
- [ ] `update_profile(user_id: int, data: ProfileUpdate) -> User` function
- [ ] Allows updating full_name
- [ ] Email change requires password confirmation
- [ ] Returns updated User object

**Implementation Notes**:
```python
# backend/auth/profile_service.py (new file)
def update_profile(user_id: int, data: ProfileUpdate) -> User:
    user = get_user_by_id(user_id)

    if data.full_name is not None:
        user.full_name = data.full_name

    if data.email is not None:
        if data.email != user.email:
            # Require password confirmation for email change
            if not data.password_confirmation:
                raise ValueError("Password required for email change")
            if not verify_password(data.password_confirmation, user.password_hash):
                raise ValueError("Invalid password")

            # Check email uniqueness
            if get_user_by_email(data.email):
                raise ValueError("Email already in use")
            user.email = data.email

    # ... save and return
    return user
```

**Testing**:
- Unit tests for profile update
- Test email change with confirmation

---

### Task 4.2: Create Profile API Endpoints [BE][US4]

**Priority**: P3
**Estimated Complexity**: Low
**Dependencies**: Task 4.1

**Description**:
- Create endpoint to get profile
- Create endpoint to update profile

**Acceptance Criteria**:
- [ ] `GET /api/auth/profile` endpoint (uses auth dependency)
- [ ] `PUT /api/auth/profile` endpoint
- [ ] Returns current user profile

**Implementation Notes**:
```python
@router.get("/profile", response_model=UserResponse)
async def get_profile(user: User = Depends(get_current_user)):
    return user

@router.put("/profile", response_model=UserResponse)
async def update_profile(
    data: ProfileUpdateRequest,
    user: User = Depends(get_current_user)
):
    updated = update_update(user.id, data)
    return updated
```

**Testing**:
- Integration tests for profile endpoints
- Test with authenticated user

---

## Phase 5: Frontend Authentication Components

### Task 5.1: Create Auth Context Provider [FE][US1][US2]

**Priority**: P1
**Estimated Complexity**: Medium
**Dependencies**: Task 2.5

**Description**:
- Create React context for authentication state
- Implement token storage in encrypted localStorage
- Provide auth methods to consumers

**Acceptance Criteria**:
- [ ] `AuthContext` with user, token, loading, error state
- [ ] `signIn(email, password)` function
- [ ] `signUp(email, password, name)` function
- [ ] `signOut()` function
- [ ] Token stored in encrypted localStorage using crypto-js
- [ ] Auto-restore session on page load

**Implementation Notes**:
```typescript
// src/contexts/AuthContext.tsx (new file)
import React, { createContext, useContext, useState, useEffect } from 'react';
import CryptoJS from 'crypto-js';

interface AuthContextValue {
  user: User | null;
  token: string | null;
  loading: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (email: string, password: string, name: string) => Promise<void>;
  signOut: () => void;
}

const ENCRYPTION_KEY = process.env.AUTH_ENCRYPTION_KEY || 'default-key';

const AuthContext = createContext<AuthContextValue | undefined>(undefined);

export function AuthProvider({ children }: { children: React.ReactNode }) {
  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [loading, setLoading] = useState(true);

  // Load token from encrypted storage on mount
  useEffect(() => {
    const stored = localStorage.getItem('auth_token');
    if (stored) {
      const decrypted = CryptoJS.AES.decrypt(stored, ENCRYPTION_KEY).toString(CryptoJS.enc.Utf8);
      setToken(decrypted);
    }
    setLoading(false);
  }, []);

  const signIn = async (email: string, password: string) => {
    const response = await fetch(`${API_URL}/api/auth/signin`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ email, password }),
    });
    if (!response.ok) throw new Error('Sign in failed');
    const data = await response.json();
    setUser(data.user);
    setToken(data.token);
    // Encrypt and store token
    const encrypted = CryptoJS.AES.encrypt(data.token, ENCRYPTION_KEY).toString();
    localStorage.setItem('auth_token', encrypted);
  };

  // ... other methods

  return (
    <AuthContext.Provider value={{ user, token, loading, signIn, signUp, signOut }}>
      {children}
    </AuthContext.Provider>
  );
}

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) throw new Error('useAuth must be used within AuthProvider');
  return context;
};
```

**Testing**:
- Test context renders correctly
- Test token encryption/decryption
- Test auth methods

---

### Task 5.2: Create Sign In Modal Component [FE][US2]

**Priority**: P1
**Estimated Complexity**: Medium
**Dependencies**: Task 5.1

**Description**:
- Create sign-in modal using existing modal patterns
- Add form validation
- Handle errors with user-friendly messages

**Acceptance Criteria**:
- [ ] SignInModal component with email and password fields
- [ ] Shows loading state during authentication
- [ ] Displays error messages for invalid credentials
- [ ] Closes on successful sign in
- [ ] Matches existing modal styling (glassmorphism)

**Implementation Notes**:
```typescript
// src/components/auth/SignInModal.tsx (new file)
import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';

export function SignInModal({ onClose, onSwitchToSignUp }: SignInModalProps) {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const { signIn } = useAuth();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      await signIn(email, password);
      onClose();
    } catch (err) {
      setError('Invalid email or password');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="glass-card modal">
      <h2>Sign In</h2>
      <form onSubmit={handleSubmit}>
        {/* Form fields */}
      </form>
      <button type="button" onClick={onSwitchToSignUp}>
        Create an account
      </button>
    </div>
  );
}
```

**Testing**:
- Test form submission
- Test validation
- Test error handling

---

### Task 5.3: Create Sign Up Modal Component [FE][US1]

**Priority**: P1
**Estimated Complexity**: Medium
**Dependencies**: Task 5.1

**Description**:
- Create sign-up modal using existing modal patterns
- Add form validation (password strength)
- Handle errors with user-friendly messages

**Acceptance Criteria**:
- [ ] SignUpModal component with name, email, password fields
- [ ] Real-time password strength indicator
- [ ] Email format validation
- [ ] Shows loading state during registration
- [ ] Displays error messages
- [ ] Closes on successful registration
- [ ] Matches existing modal styling

**Implementation Notes**:
```typescript
// src/components/auth/SignUpModal.tsx (new file)
export function SignUpModal({ onClose, onSwitchToSignIn }: SignUpModalProps) {
  // Similar to SignInModal but with additional fields
  // Add password strength validation

  const validatePassword = (password: string) => {
    const checks = {
      length: password.length >= 8,
      uppercase: /[A-Z]/.test(password),
      lowercase: /[a-z]/.test(password),
      number: /[0-9]/.test(password),
    };
    return checks;
  };

  // ...
}
```

**Testing**:
- Test form submission
- Test password strength validation
- Test email validation

---

### Task 5.4: Create Password Reset UI [FE][US3]

**Priority**: P2
**Estimated Complexity**: Medium
**Dependencies**: Task 3.2

**Description**:
- Create password reset request modal/page
- Create password reset confirmation modal/page

**Acceptance Criteria**:
- [ ] ResetRequestModal with email field
- [ ] ResetPasswordModal with new password and confirmation
- [ ] Handles token from URL query parameter
- [ ] Shows success/error messages

**Implementation Notes**:
```typescript
// src/components/auth/PasswordResetModals.tsx (new file)
export function ResetRequestModal({ onClose }: ModalProps) {
  const [email, setEmail] = useState('');
  const [message, setMessage] = useState('');

  const handleSubmit = async () => {
    await fetch('/api/auth/reset-request', {
      method: 'POST',
      body: JSON.stringify({ email }),
    });
    setMessage('If email exists, reset link sent');
  };

  // ...
}

export function ResetPasswordModal({ token, onClose }: { token: string } & ModalProps) {
  const [password, setPassword] = useState('');
  const [confirm, setConfirm] = useState('');

  const handleSubmit = async () => {
    if (password !== confirm) {
      // Show error
      return;
    }
    await fetch('/api/auth/reset-confirm', {
      method: 'POST',
      body: JSON.stringify({ token, new_password: password }),
    });
    // Show success and redirect
  };

  // ...
}
```

**Testing**:
- Test reset request flow
- Test reset confirmation flow

---

### Task 5.5: Create Profile Settings Component [FE][US4]

**Priority**: P3
**Estimated Complexity**: Low
**Dependencies**: Task 4.2, Task 5.1

**Description**:
- Create profile settings modal/page
- Allow updating name and email

**Acceptance Criteria**:
- [ ] ProfileSettings component with editable fields
- [ ] Email change requires password confirmation
- [ ] Shows success message on update

**Implementation Notes**:
```typescript
// src/components/auth/ProfileSettings.tsx (new file)
export function ProfileSettings({ onClose }: ModalProps) {
  const { user } = useAuth();
  const [name, setName] = useState(user?.full_name || '');
  const [email, setEmail] = useState(user?.email || '');
  const [passwordConfirm, setPasswordConfirm] = useState('');

  const handleUpdate = async () => {
    await fetch('/api/auth/profile', {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${token}`,
      },
      body: JSON.stringify({
        full_name: name,
        email: email !== user?.email ? email : undefined,
        password_confirmation: passwordConfirm,
      }),
    });
  };

  // ...
}
```

**Testing**:
- Test profile update
- Test email change with password

---

## Phase 6: Chat Protection (P2 User Story)

### Task 6.1: Add Auth Dependency to Chat Endpoint [BE][US5]

**Priority**: P2
**Estimated Complexity**: Low
**Dependencies**: Task 2.6

**Description**:
- Modify existing `/chat` endpoint to require authentication
- Pass user context to chat service

**Acceptance Criteria**:
- [ ] `/chat` endpoint requires valid JWT token
- [ ] Returns 401 if no token provided
- [ ] User context available in chat handler
- [ ] Chat messages associated with user

**Implementation Notes**:
```python
# backend/main.py - modify existing endpoint

@app.post("/chat")
async def chat_stream(
    request: ChatRequest,
    user: User = Depends(get_current_user)  # Add auth dependency
) -> StreamingResponse:
    # Existing implementation, now with user context
    logger.info(f"Chat request from user: {user.email}")

    async def generate_response() -> AsyncGenerator[str, None]:
        # ... existing logic
        pass

    return StreamingResponse(generate_response(), ...)
```

**Testing**:
- Test with valid token
- Test without token (should get 401)
- Test with invalid token

---

### Task 6.2: Update Chat Widget to Require Authentication [FE][US5]

**Priority**: P2
**Estimated Complexity**: Medium
**Dependencies**: Task 6.1, Task 5.1

**Description**:
- Modify ChatWidget to check authentication status
- Show sign-in modal if not authenticated
- Add Authorization header to API requests

**Acceptance Criteria**:
- [ ] ChatWidget shows sign-in prompt if not authenticated
- [ ] ChatWidget opens sign-in modal when toggled
- [ ] API requests include Authorization header
- [ ] Handles 401 responses by showing sign-in modal

**Implementation Notes**:
```typescript
// src/components/ChatWidget/utils/api.ts - modify existing file

export async function sendChatRequest(
  request: ChatRequest,
  token: string | null
): Promise<Response> {
  const headers: Record<string, string> = {
    'Content-Type': 'application/json',
  };

  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }

  const response = await fetch(`${API_URL}/chat`, {
    method: 'POST',
    headers,
    body: JSON.stringify(request),
  });

  if (response.status === 401) {
    // Trigger sign-in modal
    window.dispatchEvent(new CustomEvent('auth:required'));
    throw new Error('Authentication required');
  }

  return response;
}

// src/components/ChatWidget/index.tsx - modify
function ChatWidgetInner() {
  const { user, token } = useAuth();
  const [showSignIn, setShowSignIn] = useState(false);

  useEffect(() => {
    const handleAuthRequired = () => setShowSignIn(true);
    window.addEventListener('auth:required', handleAuthRequired);
    return () => window.removeEventListener('auth:required', handleAuthRequired);
  }, []);

  const handleToggle = () => {
    if (!user) {
      setShowSignIn(true);
      return;
    }
    setIsOpen(!isOpen);
  };

  return (
    <>
      {/* Existing chat UI */}
      {showSignIn && <SignInModal onClose={() => setShowSignIn(false)} />}
    </>
  );
}
```

**Testing**:
- Test chat access when authenticated
- Test chat access when not authenticated
- Test 401 handling

---

### Task 6.3: Store User Chat Messages [BE][US5]

**Priority**: P2
**Estimated Complexity**: Medium
**Dependencies**: Task 6.1

**Description**:
- Modify chat endpoint to store messages with user association
- Add chat_session and chat_message tables (optional enhancement)

**Acceptance Criteria**:
- [ ] Chat messages stored with user_id reference
- [ ] Messages can be retrieved per user
- [ ] Chat history preserved across sessions

**Implementation Notes**:
```sql
-- Optional enhancement for chat persistence
CREATE TABLE chat_sessions (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE chat_messages (
    id SERIAL PRIMARY KEY,
    session_id INTEGER REFERENCES chat_sessions(id),
    role VARCHAR(20) NOT NULL,
    content TEXT NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

**Testing**:
- Test message storage
- Test message retrieval

---

## Phase 7: Testing & Quality Assurance

### Task 7.1: Write Backend Unit Tests [TEST]

**Priority**: P1
**Estimated Complexity**: Medium
**Dependencies**: All backend implementation tasks

**Description**:
- Write pytest unit tests for all auth services
- Mock database dependencies
- Test edge cases and error paths

**Acceptance Criteria**:
- [ ] Tests for password hashing/verification
- [ ] Tests for JWT creation/decoding
- [ ] Tests for user registration (success and failures)
- [ ] Tests for authentication (success and failures)
- [ ] Tests for password reset flow
- [ ] >80% code coverage

**Implementation Notes**:
```python
# tests/auth/test_security.py
def test_hash_password():
    password = "TestPassword123"
    hashed = hash_password(password)
    assert hashed != password
    assert verify_password(password, hashed)

def test_verify_password_wrong():
    password = "TestPassword123"
    hashed = hash_password(password)
    assert not verify_password("WrongPassword", hashed)

# tests/auth/test_service.py
def test_register_user_success(db_session):
    user = register_user("test@example.com", "Password123", "Test User")
    assert user.email == "test@example.com"

def test_register_user_duplicate_email(db_session):
    register_user("test@example.com", "Password123", "Test User")
    with pytest.raises(ValueError):
        register_user("test@example.com", "Password123", "Another User")
```

**Testing**:
- Run pytest with coverage report
- Ensure all tests pass

---

### Task 7.2: Write Backend Integration Tests [TEST]

**Priority**: P1
**Estimated Complexity**: Medium
**Dependencies**: Task 2.5

**Description**:
- Write FastAPI TestClient tests for all endpoints
- Test full request/response cycle
- Test error handling

**Acceptance Criteria**:
- [ ] Tests for /api/auth/register endpoint
- [ ] Tests for /api/auth/signin endpoint
- [ ] Tests for /api/auth/signout endpoint
- [ ] Tests for /api/auth/me endpoint
- [ ] Tests for /api/auth/reset-request endpoint
- [ ] Tests for /api/auth/reset-confirm endpoint
- [ ] Tests for /api/auth/profile endpoints

**Implementation Notes**:
```python
# tests/auth/test_routes.py
from fastapi.testclient import TestClient

def test_register_success(client: TestClient):
    response = client.post("/api/auth/register", json={
        "email": "test@example.com",
        "password": "Password123",
        "full_name": "Test User"
    })
    assert response.status_code == 200
    data = response.json()
    assert "token" in data
    assert data["user"]["email"] == "test@example.com"

def test_register_duplicate_email(client: TestClient):
    # First registration
    client.post("/api/auth/register", json={
        "email": "test@example.com",
        "password": "Password123",
        "full_name": "Test User"
    })

    # Duplicate registration
    response = client.post("/api/auth/register", json={
        "email": "test@example.com",
        "password": "Password123",
        "full_name": "Another User"
    })
    assert response.status_code == 400
```

**Testing**:
- Run all integration tests
- Ensure endpoints behave correctly

---

### Task 7.3: Write Frontend Component Tests [TEST]

**Priority**: P2
**Estimated Complexity**: Medium
**Dependencies**: All frontend implementation tasks

**Description**:
- Write React Testing Library tests for auth components
- Mock API calls
- Test user interactions

**Acceptance Criteria**:
- [ ] Tests for AuthContext provider
- [ ] Tests for SignInModal component
- [ ] Tests for SignUpModal component
- [ ] Tests for PasswordResetModals
- [ ] Tests for ProfileSettings

**Implementation Notes**:
```typescript
// src/components/auth/__tests__/SignInModal.test.tsx
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { SignInModal } from '../SignInModal';
import { AuthProvider } from '../../contexts/AuthContext';

test('shows sign in form', () => {
  render(
    <AuthProvider>
      <SignInModal onClose={() => {}} onSwitchToSignUp={() => {}} />
    </AuthProvider>
  );
  expect(screen.getByLabelText(/email/i)).toBeInTheDocument();
  expect(screen.getByLabelText(/password/i)).toBeInTheDocument();
});

test('submits sign in form', async () => {
  // Mock fetch
  global.fetch = jest.fn(() =>
    Promise.resolve({
      ok: true,
      json: () => Promise.resolve({ user: { email: 'test@example.com' }, token: 'abc123' }),
    })
  );

  render(
    <AuthProvider>
      <SignInModal onClose={() => {}} onSwitchToSignUp={() => {}} />
    </AuthProvider>
  );

  fireEvent.change(screen.getByLabelText(/email/i), { target: { value: 'test@example.com' } });
  fireEvent.change(screen.getByLabelText(/password/i), { target: { value: 'Password123' } });
  fireEvent.click(screen.getByRole('button', { name: /sign in/i }));

  await waitFor(() => {
    expect(global.fetch).toHaveBeenCalledWith('/api/auth/signin', expect.any(Object));
  });
});
```

**Testing**:
- Run frontend tests
- Ensure components render and interact correctly

---

### Task 7.4: Manual Testing Checklist [TEST]

**Priority**: P1
**Estimated Complexity**: Low
**Dependencies**: All implementation tasks

**Description**:
- Create manual testing checklist for QA
- Perform end-to-end testing

**Acceptance Criteria**:
- [ ] All user stories tested manually
- [ ] Edge cases documented
- [ ] Browser compatibility tested

**Manual Test Cases**:

| Test Case | Steps | Expected Result |
|-----------|-------|-----------------|
| Registration | Navigate to site, click "Sign Up", enter valid credentials | Account created, user signed in |
| Registration - weak password | Enter password < 8 chars | Error message shown |
| Registration - duplicate email | Register with existing email | Error message shown |
| Sign In | Enter valid credentials | User signed in |
| Sign In - invalid credentials | Enter wrong password | Error message shown |
| Sign In - non-existent email | Enter unregistered email | Error message shown |
| Sign Out | Click sign out | Session cleared, signed out |
| Password Reset | Request reset with email | Reset URL returned/shown |
| Chat access - authenticated | Open chat widget while signed in | Chat interface opens |
| Chat access - not authenticated | Open chat widget while not signed in | Sign in modal appears |
| Profile Update | Update name in profile settings | Name updated successfully |
| Profile Update - email change | Change email with password | Email updated successfully |
| Session persistence | Refresh page after sign in | User still signed in |
| Token expiration | Wait 7 days after sign in | Token invalid, must re-authenticate |

---

## Phase 8: Deployment & Documentation

### Task 8.1: Set Environment Variables on Railway [Infrastructure]

**Priority**: P0 (Blocker)
**Estimated Complexity**: Low
**Dependencies**: Task 1.1

**Description**:
- Add all required environment variables to Railway deployment
- Include database URL, JWT secret, encryption key

**Acceptance Criteria**:
- [ ] `DATABASE_URL` set to Neon PostgreSQL connection string
- [ ] `JWT_SECRET_KEY` set to secure random value
- [ ] `AUTH_ENCRYPTION_KEY` set (used by frontend)
- [ ] All variables documented

**Implementation Notes**:
```
Required environment variables:

Backend:
- DATABASE_URL: PostgreSQL connection string
- JWT_SECRET_KEY: Secret for JWT signing (generate with openssl rand -hex 32)
- NEON_API_KEY: Neon API key (optional, for migrations)

Frontend:
- AUTH_ENCRYPTION_KEY: AES encryption key for localStorage
- API_URL: Backend API URL
```

**Testing**:
- Verify backend starts with environment variables
- Verify frontend can connect to backend

---

### Task 8.2: Deploy Backend Changes [Infrastructure]

**Priority**: P1
**Estimated Complexity**: Low
**Dependencies**: Task 2.5, Task 8.1

**Description**:
- Deploy FastAPI backend with auth endpoints
- Run database migrations on production

**Acceptance Criteria**:
- [ ] Backend deployed to Railway
- [ ] Database migrations run successfully
- [ ] Health check endpoint includes auth status

**Testing**:
- Test production endpoints
- Verify database connectivity

---

### Task 8.3: Deploy Frontend Changes [Infrastructure]

**Priority**: P1
**Estimated Complexity**: Low
**Dependencies**: Task 5.1, Task 5.2, Task 5.3, Task 6.2

**Description**:
- Deploy Docusaurus site with auth components
- Set up AuthProvider at app root

**Acceptance Criteria**:
- [ ] Frontend deployed to Vercel
- [ ] AuthProvider wraps entire app
- [ ] Environment variables configured

**Testing**:
- Test authentication flow in production
- Verify chat protection

---

### Task 8.4: Create API Documentation [Documentation]

**Priority**: P2
**Estimated Complexity**: Low
**Dependencies**: Task 2.5, Task 3.2, Task 4.2

**Description**:
- Document all auth API endpoints
- Include request/response examples

**Acceptance Criteria**:
- [ ] API documentation for all auth endpoints
- [ ] Request/response examples
- [ ] Error codes documented

**Implementation Notes**:
```markdown
# Authentication API

## POST /api/auth/register

Register a new user account.

### Request

```json
{
  "email": "user@example.com",
  "password": "SecurePass123",
  "full_name": "John Doe"
}
```

### Response (200 OK)

```json
{
  "user": {
    "id": 1,
    "email": "user@example.com",
    "full_name": "John Doe"
  },
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```

### Errors

- 400 Bad Request: Invalid input (weak password, invalid email)
- 409 Conflict: Email already registered
```

---

### Task 8.5: Create Setup Documentation [Documentation]

**Priority**: P2
**Estimated Complexity**: Low
**Dependencies**: All implementation tasks

**Description**:
- Document local development setup
- Include database setup and migration steps

**Acceptance Criteria**:
- [ ] Local development setup guide
- [ ] Database setup instructions
- [ ] Migration commands documented

**Implementation Notes**:
```markdown
# Local Development Setup

## Prerequisites

- Python 3.9+
- Node.js 18+
- Neon PostgreSQL account

## Backend Setup

1. Create virtual environment:
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Set up environment variables:
```bash
cp .env.example .env
# Edit .env with your values
```

4. Run database migrations:
```bash
python -m auth.migrations migrate
```

5. Run backend:
```bash
python -m uvicorn main:app --reload
```

## Frontend Setup

1. Install dependencies:
```bash
npm install
```

2. Set up environment variables:
```bash
cp .env.frontend.example .env.frontend
# Edit .env.frontend with your values
```

3. Run development server:
```bash
npm start
```
```

---

## Task Dependencies Graph

```
[Phase 1: Foundation]
1.1 (Neon Setup) ──────┐
                       │
1.2 (Models) ──────────┼───> 1.3 (Migrations)
                       │
[Phase 2: Auth Backend] │
2.1 (Password Hash) ────┤
2.2 (JWT) ──────────────┤
                       │
2.3 (Register Service) <┘
2.4 (SignIn Service) <───┘──┐
                          │
2.5 (Auth Endpoints) <─────┴───> 2.6 (Auth Dependency)
                                                    │
[Phase 3: Password Reset]                          │
3.1 (Reset Token) ────────> 3.2 (Reset Endpoints) │
                                                    │
[Phase 4: Profile]                                  │
4.1 (Profile Service) ─────> 4.2 (Profile Endpoints)
                                                    │
[Phase 5: Frontend]                                │
5.1 (Auth Context) ──┬──> 5.2 (SignIn Modal)      │
                     ├──> 5.3 (SignUp Modal)      │
                     ├──> 5.4 (Reset Modals) <────┤
                     └──> 5.5 (Profile)     <──────┤
                                                  │
[Phase 6: Chat Protection]                         │
6.1 (Chat Auth) <──────────────────────────────────┘
6.2 (Chat Widget Auth) <──> 6.3 (Chat Storage)
                           │
[Phase 7: Testing]           │
7.1, 7.2, 7.3 <─────────────┘
        │
[Phase 8: Deployment]
7.4 ───> 8.1, 8.2, 8.3, 8.4, 8.5
```

---

## Parallel Execution Examples

### Example 1: Backend Development (Tasks 2.1-2.4)

These tasks can be developed in parallel after Phase 1 is complete:

```bash
# Terminal 1: Work on password hashing
# Task 2.1
vim backend/auth/security.py

# Terminal 2: Work on JWT (separate concern, no conflicts)
# Task 2.2
vim backend/auth/security.py  # Same file, but can work on different functions
# Or use feature branches

# After 2.1 and 2.2 are done, work in parallel on services:
# Terminal 3: Task 2.3 - Registration service
vim backend/auth/service.py

# Terminal 4: Task 2.4 - Sign in service
vim backend/auth/service.py  # Same file, coordinate with team
```

### Example 2: Frontend Development (Tasks 5.2-5.5)

After AuthContext is complete, modals can be built in parallel:

```bash
# Terminal 1: SignInModal
# Task 5.2
vim src/components/auth/SignInModal.tsx

# Terminal 2: SignUpModal
# Task 5.3
vim src/components/auth/SignUpModal.tsx

# Terminal 3: Password Reset Modals
# Task 5.4
vim src/components/auth/PasswordResetModals.tsx

# Terminal 4: Profile Settings
# Task 5.5
vim src/components/auth/ProfileSettings.tsx
```

### Example 3: Testing (Tasks 7.1-7.3)

All testing tasks can run in parallel once implementation is complete:

```bash
# Terminal 1: Backend unit tests
# Task 7.1
pytest tests/auth/test_security.py tests/auth/test_service.py

# Terminal 2: Backend integration tests
# Task 7.2
pytest tests/auth/test_routes.py

# Terminal 3: Frontend tests
# Task 7.3
npm test -- src/components/auth/__tests__/
```

---

## Completion Checklist

**Before marking this feature complete, verify:**

- [ ] All Phase 1 (Foundation) tasks complete
- [ ] All Phase 2 (Auth Backend) tasks complete
- [ ] Phase 3 (Password Reset) complete OR explicitly deferred
- [ ] Phase 4 (Profile) complete OR explicitly deferred
- [ ] All Phase 5 (Frontend Components) tasks complete
- [ ] All Phase 6 (Chat Protection) tasks complete
- [ ] All Phase 7 (Testing) tasks complete
- [ ] All Phase 8 (Deployment) tasks complete
- [ ] All success criteria from spec.md are met
- [ ] No NEEDS CLARIFICATION items remain
- [ ] All acceptance criteria passing
- [ ] Code reviewed and approved
- [ ] Documentation complete
- [ ] PHR created for /sp.tasks session

---

**End of tasks.md**
