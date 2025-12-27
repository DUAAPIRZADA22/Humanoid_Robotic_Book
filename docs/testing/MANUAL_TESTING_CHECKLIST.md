# Manual Testing Checklist - User Authentication System

This checklist provides manual testing steps for verifying the authentication system works correctly.

## Prerequisites

- [ ] Backend server running on `http://localhost:8000`
- [ ] Frontend running on `http://localhost:3000`
- [ ] Database (Neon PostgreSQL) connected and migrated
- [ ] Required environment variables set:
  - `DATABASE_URL`
  - `SECRET_KEY`
  - `API_KEY` (for frontend chat widget)

## Test Accounts

Create these test accounts for testing:

| Email | Password | Purpose |
|-------|----------|---------|
| `test@example.com` | `TestPass123` | Primary test account |
| `admin@example.com` | `AdminPass123` | Admin testing |
| `delete.me@example.com` | `DeleteMe123` | Account deletion testing |

---

## 1. User Registration

### 1.1 Valid Registration
- [ ] Navigate to the homepage
- [ ] Click "Sign In" button
- [ ] Click "Create account" link
- [ ] Enter valid email: `newuser@example.com`
- [ ] Enter strong password: `NewPassword123`
- [ ] Enter full name: `New User`
- [ ] Click "Sign Up"
- [ ] **Expected**: Account created, user signed in automatically, modal closes

### 1.2 Duplicate Email Registration
- [ ] Try to register with existing email: `test@example.com`
- [ ] **Expected**: Error message "Email already registered"

### 1.3 Weak Password Registration
- [ ] Try to register with password: `weak`
- [ ] **Expected**: Password strength indicator shows weak, error message
- [ ] Try password: `nouppercase123`
- [ ] **Expected**: Error "Must contain uppercase letter"
- [ ] Try password: `NOLOWERCASE123`
- [ ] **Expected**: Error "Must contain lowercase letter"
- [ ] Try password: `NoNumbers`
- [ ] **Expected**: Error "Must contain a number"

### 1.4 Invalid Email Registration
- [ ] Try to register with email: `notanemail`
- [ ] **Expected**: Validation error on email field

---

## 2. User Sign In

### 2.1 Valid Sign In
- [ ] Click "Sign In" button
- [ ] Enter email: `test@example.com`
- [ ] Enter password: `TestPass123`
- [ ] Click "Sign In"
- [ ] **Expected**: Successful sign in, modal closes, user menu shows email

### 2.2 Invalid Credentials
- [ ] Enter correct email but wrong password
- [ ] Click "Sign In"
- [ ] **Expected**: Error "Invalid email or password"

### 2.3 Non-existent Account
- [ ] Enter email: `nonexistent@example.com`
- [ ] Enter any password
- [ ] Click "Sign In"
- [ ] **Expected**: Error "Invalid email or password"

---

## 3. Password Reset

### 3.1 Request Password Reset
- [ ] Click "Sign In" button
- [ ] Click "Forgot password?" link
- [ ] Enter email: `test@example.com`
- [ ] Click "Send Reset Link"
- [ ] **Expected**: Success message "Reset link sent to your email"
- [ ] Check console for reset URL (development mode)

### 3.2 Reset with Valid Token
- [ ] Use reset token from console or email
- [ ] Navigate to reset URL: `/?reset_token=<token>`
- [ ] **Expected**: Reset password modal opens automatically
- [ ] Enter new password: `NewTestPass456`
- [ ] Confirm password: `NewTestPass456`
- [ ] Click "Reset Password"
- [ ] **Expected**: Success message, can sign in with new password

### 3.3 Reset with Invalid Token
- [ ] Navigate to: `/?reset_token=invalid_token`
- [ ] **Expected**: Error message "Invalid or expired reset token"

### 3.4 Reset with Weak Password
- [ ] Open valid reset URL
- [ ] Enter weak password: `weak`
- [ ] Click "Reset Password"
- [ ] **Expected**: Validation errors displayed

---

## 4. Profile Management

### 4.1 View Profile
- [ ] Sign in as test user
- [ ] Click user menu (email in header)
- [ ] Click "Profile Settings"
- [ ] **Expected**: Profile modal opens with user details displayed

### 4.2 Update Name
- [ ] In Profile Settings, change full name
- [ ] Click "Update Profile"
- [ ] **Expected**: Success message, name updated

### 4.3 Update Email (with password confirmation)
- [ ] Change email to: `newemail@example.com`
- [ ] **Expected**: Password confirmation field appears
- [ ] Enter current password: `TestPass123`
- [ ] Click "Update Profile"
- [ ] **Expected**: Success message, email updated

### 4.4 Update Email (wrong password)
- [ ] Change email again
- [ ] Enter incorrect password
- [ ] Click "Update Profile"
- [ ] **Expected**: Error "Invalid password confirmation"

---

## 5. Chat Authentication

### 5.1 Chat While Signed In
- [ ] Sign in as test user
- [ ] Navigate to a page with ChatWidget
- [ ] Type message and send
- [ ] **Expected**: Chat works normally, response streams

### 5.2 Chat While Signed Out
- [ ] Sign out
- [ ] Try to send a chat message
- [ ] **Expected**: Sign in modal appears automatically

### 5.3 Session Expiry
- [ ] Sign in
- [ ] Wait for token to expire (or modify token expiry for testing)
- [ ] Try to send chat message
- [ ] **Expected**: Sign in modal appears with session expiry message

---

## 6. Session Persistence

### 6.1 Refresh After Sign In
- [ ] Sign in as test user
- [ ] Refresh the page (F5)
- [ ] **Expected**: User remains signed in, menu shows email

### 6.2 Close and Reopen Browser
- [ ] Sign in as test user
- [ ] Close browser tab/window
- [ ] Reopen and navigate to site
- [ ] **Expected**: User remains signed in (encrypted token persists)

---

## 7. Sign Out

### 7.1 Manual Sign Out
- [ ] Sign in as test user
- [ ] Click user menu
- [ ] Click "Sign Out"
- [ ] **Expected**: User signed out, "Sign In" button appears

### 7.2 Sign Out Clears State
- [ ] After sign out, check localStorage
- [ ] **Expected**: No user data or encrypted tokens remain

### 7.3 Sign Out and Chat
- [ ] After sign out, try to use chat
- [ ] **Expected**: Sign in modal appears

---

## 8. Security Verification

### 8.1 Password Hashing
- [ ] Check database for user record
- [ ] **Expected**: `password_hash` is NOT plain text, starts with `$2b$` (bcrypt)

### 8.2 JWT Token Format
- [ ] Sign in and inspect access token (browser dev tools)
- [ ] **Expected**: Token is valid JWT format (header.payload.signature)

### 8.3 Token Expiry
- [ ] Decode JWT token (use jwt.io or similar)
- [ ] Check `exp` claim
- [ ] **Expected**: Expiration is 7 days from `iat`

### 8.4 HTTPS/TLS (Production)
- [ ] If deployed to production, check HTTPS
- [ ] **Expected**: All API calls use HTTPS, credentials protected in transit

### 8.5 CORS Configuration
- [ ] Check browser console for CORS errors
- [ ] **Expected**: No CORS errors, frontend can call backend

---

## 9. Error Handling

### 9.1 Network Errors
- [ ] Stop backend server
- [ ] Try to sign in
- [ ] **Expected**: User-friendly error message, no crash

### 9.2 Database Errors
- [ ] Disconnect database (if possible for testing)
- [ ] Try to register
- [ ] **Expected**: Error message, no crash

### 9.3 Validation Errors
- [ ] Submit empty forms
- [ ] Submit invalid data
- [ ] **Expected**: Clear validation messages, helpful text

---

## 10. Accessibility

### 10.1 Keyboard Navigation
- [ ] Navigate modals using Tab key
- [ ] **Expected**: Logical focus order, visible focus indicators

### 10.2 Screen Reader
- [ ] Test with screen reader (NVDA, JAWS, or VoiceOver)
- [ ] **Expected**: Labels announced, errors read, status changes announced

### 10.3 ARIA Labels
- [ ] Inspect modal buttons and inputs
- [ ] **Expected**: Proper `aria-label`, `role`, and `aria-describedby` attributes

---

## 11. Browser Compatibility

Test in the following browsers:

- [ ] Chrome/Edge (Chromium)
- [ ] Firefox
- [ ] Safari (if available)
- [ ] Mobile browsers:
  - [ ] iOS Safari
  - [ ] Android Chrome

**Expected**: Consistent behavior across all browsers

---

## 12. Performance

### 12.1 Sign In Response Time
- [ ] Measure sign in API response
- [ ] **Expected**: < 500ms typical, < 2s p95

### 12.2 Chat Response Time
- [ ] Measure chat API response (with auth)
- [ ] **Expected**: No significant slowdown vs unauthenticated chat

### 12.3 Modal Load Time
- [ ] Measure time to open sign in modal
- [ ] **Expected**: Instant (< 100ms perceived)

---

## Test Results Summary

| Category | Passed | Failed | Notes |
|----------|--------|--------|-------|
| Registration | | | |
| Sign In | | | |
| Password Reset | | | |
| Profile Management | | | |
| Chat Authentication | | | |
| Session Persistence | | | |
| Sign Out | | | |
| Security | | | |
| Error Handling | | | |
| Accessibility | | | |
| Browser Compatibility | | | |
| Performance | | | |

**Total**: ___ / ___ passed

**Tester**: ____________________
**Date**: ____________________
**Browser/Version**: ____________________
