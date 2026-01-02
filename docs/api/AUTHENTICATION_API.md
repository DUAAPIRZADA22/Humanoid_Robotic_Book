# Authentication API Documentation

The Authentication API provides endpoints for user registration, sign in, password reset, and profile management.

## Base URL

```
http://localhost:8000/api/auth
```

## Authentication

Protected endpoints require a Bearer token in the Authorization header:

```
Authorization: Bearer <access_token>
```

## Endpoints

### 1. Register User

Register a new user account.

**Endpoint:** `POST /register`

**Request Body:**
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123",
  "full_name": "John Doe"  // optional
}
```

**Response (201 Created):**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer",
  "user": {
    "id": 1,
    "email": "user@example.com",
    "full_name": "John Doe"
  }
}
```

**Errors:**
- `400 Bad Request` - Invalid email format or weak password
- `400 Bad Request` - Email already registered

**Password Requirements:**
- Minimum 8 characters
- At least one uppercase letter
- At least one lowercase letter
- At least one number

---

### 2. Sign In

Authenticate with email and password.

**Endpoint:** `POST /signin`

**Request Body:**
```json
{
  "email": "user@example.com",
  "password": "SecurePassword123"
}
```

**Response (200 OK):**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer",
  "user": {
    "id": 1,
    "email": "user@example.com",
    "full_name": "John Doe"
  }
}
```

**Errors:**
- `401 Unauthorized` - Invalid email or password

---

### 3. Sign Out

End the current user session.

**Endpoint:** `POST /signout`

**Headers:**
```
Authorization: Bearer <access_token>
```

**Response (200 OK):**
```json
{
  "message": "Signed out successfully"
}
```

**Errors:**
- `401 Unauthorized` - Invalid or expired token

---

### 4. Get Current User

Get the authenticated user's information.

**Endpoint:** `GET /me`

**Headers:**
```
Authorization: Bearer <access_token>
```

**Response (200 OK):**
```json
{
  "id": 1,
  "email": "user@example.com",
  "full_name": "John Doe"
}
```

**Errors:**
- `401 Unauthorized` - Invalid or expired token

---

### 5. Request Password Reset

Request a password reset token via email.

**Endpoint:** `POST /reset-request`

**Request Body:**
```json
{
  "email": "user@example.com"
}
```

**Response (200 OK):**
```json
{
  "message": "If the email exists, a reset link has been sent",
  "reset_url": "http://localhost:3000/?reset_token=abc123..."  // Development only
}
```

**Note:** In production, the reset URL is sent via email. In development, it's returned in the response for convenience.

---

### 6. Confirm Password Reset

Reset password using a valid reset token.

**Endpoint:** `POST /reset-confirm`

**Request Body:**
```json
{
  "token": "reset_token_here",
  "new_password": "NewSecurePassword456"
}
```

**Response (200 OK):**
```json
{
  "message": "Password reset successfully"
}
```

**Errors:**
- `400 Bad Request` - Invalid or expired token
- `400 Bad Request` - Weak password

---

### 7. Get Profile

Get the authenticated user's profile information.

**Endpoint:** `GET /profile`

**Headers:**
```
Authorization: Bearer <access_token>
```

**Response (200 OK):**
```json
{
  "id": 1,
  "email": "user@example.com",
  "full_name": "John Doe",
  "created_at": "2024-01-15T10:30:00Z"
}
```

---

### 8. Update Profile

Update the authenticated user's profile information.

**Endpoint:** `PUT /profile`

**Headers:**
```
Authorization: Bearer <access_token>
```

**Request Body:**
```json
{
  "full_name": "Jane Doe",  // optional
  "email": "newemail@example.com",  // optional
  "password_confirmation": "CurrentPassword123"  // required if changing email
}
```

**Response (200 OK):**
```json
{
  "id": 1,
  "email": "newemail@example.com",
  "full_name": "Jane Doe",
  "created_at": "2024-01-15T10:30:00Z"
}
```

**Errors:**
- `400 Bad Request` - Invalid email format
- `400 Bad Request` - Email already in use
- `401 Unauthorized` - Invalid password confirmation (when changing email)

---

### 9. Change Password

Change the authenticated user's password.

**Endpoint:** `POST /profile/change-password`

**Headers:**
```
Authorization: Bearer <access_token>
```

**Request Body:**
```json
{
  "current_password": "OldPassword123",
  "new_password": "NewPassword456"
}
```

**Response (200 OK):**
```json
{
  "message": "Password changed successfully"
}
```

**Errors:**
- `400 Bad Request` - Invalid current password
- `400 Bad Request` - Weak new password

---

### 10. Delete Account

Delete the authenticated user's account.

**Endpoint:** `DELETE /profile`

**Headers:**
```
Authorization: Bearer <access_token>
```

**Response (200 OK):**
```json
{
  "message": "Account deleted successfully"
}
```

**Note:** This is a soft delete. The account is marked as deleted but data is retained for a period before permanent deletion.

---

## Error Response Format

All error responses follow this format:

```json
{
  "detail": "Error message description"
}
```

## Rate Limiting

API endpoints may be rate limited to prevent abuse:

- `429 Too Many Requests` - Rate limit exceeded

## Token Expiration

Access tokens expire after 7 days. After expiration, the user must sign in again to obtain a new token.

## Security

- All passwords are hashed using bcrypt before storage
- Tokens are signed using HS256 (HMAC-SHA256)
- HTTPS should be used in production
- CORS is configured for allowed origins

---

## Integration with Chat API

The `/chat` endpoint requires authentication. Include the access token in the Authorization header:

```
POST /chat
Authorization: Bearer <access_token>
Content-Type: application/json

{
  "message": "Hello, how can you help me learn about robotics?",
  "conversation_id": "optional-conversation-id"
}
```

---

## Examples

### cURL Examples

```bash
# Register
curl -X POST http://localhost:8000/api/auth/register \
  -H "Content-Type: application/json" \
  -d '{"email":"user@example.com","password":"SecurePass123","full_name":"Test User"}'

# Sign In
curl -X POST http://localhost:8000/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{"email":"user@example.com","password":"SecurePass123"}'

# Get Current User
curl -X GET http://localhost:8000/api/auth/me \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN"

# Request Password Reset
curl -X POST http://localhost:8000/api/auth/reset-request \
  -H "Content-Type: application/json" \
  -d '{"email":"user@example.com"}'
```

### JavaScript/Fetch Examples

```javascript
// Register
const response = await fetch('http://localhost:8000/api/auth/register', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    email: 'user@example.com',
    password: 'SecurePass123',
    full_name: 'Test User'
  })
});
const data = await response.json();

// Sign In
const response = await fetch('http://localhost:8000/api/auth/signin', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    email: 'user@example.com',
    password: 'SecurePass123'
  })
});
const { access_token, user } = await response.json();

// Get Current User
const response = await fetch('http://localhost:8000/api/auth/me', {
  headers: { 'Authorization': `Bearer ${access_token}` }
});
const user = await response.json();
```
