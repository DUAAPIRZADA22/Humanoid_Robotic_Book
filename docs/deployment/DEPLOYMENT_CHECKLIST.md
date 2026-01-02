# Deployment Checklist - User Authentication System

Use this checklist when deploying the authentication system to production.

---

## Pre-Deployment Checklist

### Environment Configuration

- [ ] **DATABASE_URL** - Set production Neon PostgreSQL connection string
- [ ] **SECRET_KEY** - Generate and set a strong, unique secret key
- [ ] **FRONTEND_URL** - Set to production frontend URL (e.g., `https://your-domain.com`)
- [ ] **OPENROUTER_API_KEY** - Set production API key for chat functionality
- [ ] **COHERE_API_KEY** - Set production API key for embeddings
- [ ] **QDRANT_URL** - Set production Qdrant instance URL
- [ ] **QDRANT_API_KEY** - Set Qdrant API key if using cloud

### Generate Strong SECRET_KEY

```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

---

## Backend Deployment

### Railway Deployment (Recommended)

1. [ ] Connect GitHub repository to Railway
2. [ ] Select `backend/` as root directory
3. [ ] Configure environment variables in Railway dashboard
4. [ ] Deploy backend service
5. [ ] Verify health endpoint returns 200 OK
6. [ ] Test authentication endpoints

### Alternative: VPS/Docker

1. [ ] Set up server (Ubuntu 20.04+ recommended)
2. [ ] Install Docker and Docker Compose
3. [ ] Build backend image
4. [ ] Configure environment variables
5. [ ] Set up reverse proxy (nginx)
6. [ ] Configure SSL certificate (Let's Encrypt)
7. [ ] Deploy and verify

### Backend Verification

```bash
# Health check
curl https://your-backend-url.com/health

# Expected response
{
  "status": "healthy",
  "database": "connected"
}
```

---

## Frontend Deployment

### Vercel Deployment (Recommended)

1. [ ] Connect GitHub repository to Vercel
2. [ ] Configure build settings:
   - Framework preset: Docusaurus
   - Build command: `npm run build`
   - Output directory: `build`
3. [ ] Add environment variables:
   - `API_URL`: Production backend URL
   - `CHAT_API_KEY`: Production API key
4. [ ] Deploy and verify
5. [ ] Configure custom domain (optional)

### Alternative: Netlify

1. [ ] Connect repository to Netlify
2. [ ] Configure build settings
3. [ ] Add environment variables
4. [ ] Deploy and verify

### Frontend Verification

- [ ] Site loads without console errors
- [ ] "Sign In" button is visible
- [ ] Registration form works
- [ ] Chat widget requires authentication

---

## Database Setup

### Neon PostgreSQL

1. [ ] Create production Neon project
2. [ ] Create database tables (automatic on first run)
3. [ ] Verify connection from backend
4. [ ] Set up connection pooling (if needed)
5. [ ] Configure automated backups

### Database Verification

```python
# Test database connection
from backend.auth.database import engine
from sqlmodel import Session, select
from backend.auth.models import User

with Session(engine) as session:
    result = session.exec(select(User)).first()
    print(f"Database connection: {'OK' if result is not None else 'EMPTY'}")
```

---

## Security Configuration

### HTTPS/SSL

- [ ] Backend uses HTTPS only
- [ ] Frontend uses HTTPS only
- [ ] SSL certificates are valid
- [ ] Redirect HTTP to HTTPS

### CORS Configuration

- [ ] `FRONTEND_URL` matches production domain
- [ ] Only allowed origins can access API
- [ ] Credentials mode is properly configured

### Secrets Management

- [ ] `.env` file is NOT committed to git
- [ ] `.env` is in `.gitignore`
- [ ] Production secrets are stored securely (Railway/Vercel env vars)
- [ ] SECRET_KEY is unique per environment

### Rate Limiting (Optional)

- [ ] Implement rate limiting on registration
- [ ] Implement rate limiting on sign in
- [ ] Implement rate limiting on password reset

---

## Email Configuration (Production)

### Password Reset Emails

In production, password reset links should be sent via email:

1. [ ] Set up email service (SendGrid, Mailgun, AWS SES)
2. [ ] Configure SMTP settings in backend
3. [ ] Update `reset_service.py` to send real emails
4. [ ] Remove `reset_url` from API response (development only)
5. [ ] Test password reset flow end-to-end

### Email Template

Required email template:
- Subject: "Reset Your Password"
- Body: Includes reset link with token
- Expiration notice (1 hour)
- Support contact information

---

## Testing

### Pre-Production Testing

- [ ] Run all backend tests: `pytest tests/`
- [ ] Run manual testing checklist
- [ ] Test registration flow
- [ ] Test sign in flow
- [ ] Test password reset flow
- [ ] Test profile updates
- [ ] Test chat authentication
- [ ] Test sign out flow

### Load Testing (Optional)

- [ ] Test API under load (100 concurrent users)
- [ ] Verify response times < 2s p95
- [ ] Check database connection pool size

---

## Monitoring & Logging

### Logging Configuration

- [ ] Set appropriate `LOG_LEVEL` (info for production)
- [ ] Configure structured logging (JSON format)
- [ ] Set up log aggregation (Railway logs, CloudWatch, etc.)

### Monitoring Setup

- [ ] Monitor backend health endpoint
- [ ] Monitor error rates
- [ ] Monitor database connection health
- [ ] Set up alerts for failures

### Key Metrics to Monitor

- Request latency (p50, p95, p99)
- Error rate by endpoint
- Database query performance
- Authentication success/failure rates
- Rate of password reset requests

---

## Rollback Plan

### Rollback Triggers

- [ ] Database connection failures
- [ ] High error rates (> 5%)
- [ ] Security vulnerability discovered
- [ ] Authentication not working

### Rollback Steps

1. [ ] Revert to previous git commit
2. [ ] Redeploy backend
3. [ ] Verify functionality
4. [ ] Monitor logs for issues

### Database Rollback

- [ ] Database migrations are reversible
- [ ] Backup plan for user data
- [ ] Document rollback procedure

---

## Post-Deployment

### Verification Steps

- [ ] Smoke test all authentication flows
- [ ] Verify chat widget requires authentication
- [ ] Test password reset (email delivery)
- [ ] Verify HTTPS redirect works
- [ ] Check CORS configuration

### User Communication

- [ ] Update user documentation
- [ ] Announce new authentication feature
- [ ] Provide user guide link
- [ ] Set up support channels

---

## Ongoing Maintenance

### Regular Tasks

- [ ] Rotate SECRET_KEY periodically (quarterly)
- [ ] Update dependencies monthly
- [ ] Review and rotate API keys
- [ ] Monitor and clean up expired sessions
- [ ] Review security logs

### Security Audits

- [ ] Quarterly dependency vulnerability scan
- [ ] Annual penetration testing
- [ ] Review authentication flows
- [ ] Check for unauthorized access attempts

---

## Troubleshooting

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| 401 Unauthorized | Invalid/expired token | Sign in again |
| CORS errors | Frontend URL mismatch | Update FRONTEND_URL |
| Database connection | DATABASE_URL wrong | Verify Neon connection string |
| Password reset not working | Email not configured | Set up email service |

### Emergency Contacts

- Backend Lead: ____________________
- DevOps: ____________________
- Database Admin: ____________________

---

## Sign-Off

**Deployment completed by:** ____________________

**Date:** ____________________

**Verified by:** ____________________

**Notes:**

_________________________________________________________________________

_________________________________________________________________________

_________________________________________________________________________
