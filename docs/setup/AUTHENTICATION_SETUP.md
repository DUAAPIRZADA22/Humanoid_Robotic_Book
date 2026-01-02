# Authentication System Setup Guide

This guide walks you through setting up the User Authentication System for the Humanoid Robotics Textbook project.

## Prerequisites

- Python 3.9+
- Node.js 18+
- Neon PostgreSQL account (free tier available)
- Git

---

## 1. Backend Setup

### 1.1 Install Python Dependencies

```bash
cd backend
pip install -r requirements.txt
```

**Required packages:**
- `fastapi` - Web framework
- `uvicorn` - ASGI server
- `sqlmodel` - ORM
- `pydantic` - Data validation
- `pyjwt` - JWT tokens
- `bcrypt` - Password hashing
- `psycopg2-binary` - PostgreSQL adapter
- `python-multipart` - Form data parsing
- `python-dotenv` - Environment variables

### 1.2 Configure Environment Variables

Create a `.env` file in the `backend` directory:

```bash
cd backend
cp .env.example .env
```

Edit `.env` with your values:

```env
# Database (Neon PostgreSQL)
DATABASE_URL=postgresql://username:password@ep-xxx.region.aws.neon.tech/neondb?sslmode=require

# JWT Secret (generate a secure random string)
SECRET_KEY=your-super-secret-key-change-this-in-production

# API Keys for chat functionality
OPENROUTER_API_KEY=your-openrouter-key
COHERE_API_KEY=your-cohere-key
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-qdrant-key

# CORS
FRONTEND_URL=http://localhost:3000
```

### 1.3 Generate a Secure SECRET_KEY

```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

Copy the output and use it as your `SECRET_KEY`.

### 1.4 Create Database Tables

The tables will be created automatically on first run. To create them manually:

```bash
cd backend
python -c "from auth.database import engine; from sqlmodel import SQLModel; from auth.models import User, UserPreferences, Session, PasswordResetToken; SQLModel.metadata.create_all(engine)"
```

### 1.5 Run the Backend Server

```bash
cd backend
uvicorn main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`

---

## 2. Frontend Setup

### 2.1 Install Node Dependencies

```bash
npm install
```

### 2.2 Configure Environment Variables

Create a `.env.local` file in the project root (or use the existing `.env.frontend.example`):

```bash
cp .env.frontend.example .env.local
```

Edit `.env.local`:

```env
# Backend API URL
API_URL=http://localhost:8000

# Chat API Key (for X-API-Key header)
CHAT_API_KEY=your-api-key-here
```

### 2.3 Run the Development Server

```bash
npm run start
```

The site will be available at `http://localhost:3000`

---

## 3. Neon PostgreSQL Setup

### 3.1 Create a Neon Account

1. Go to [https://neon.tech](https://neon.tech)
2. Sign up for a free account
3. Create a new project

### 3.2 Get Your Database URL

1. In the Neon dashboard, select your project
2. Go to "Connection Details"
3. Copy the connection string

Format:
```
postgresql://username:password@ep-xxx.region.aws.neon.tech/neondb?sslmode=require
```

### 3.3 Configure DATABASE_URL

Add the connection string to your `backend/.env` file:

```env
DATABASE_URL=postgresql://username:password@ep-xxx.region.aws.neon.tech/neondb?sslmode=require
```

---

## 4. Verification

### 4.1 Test Backend Health

```bash
curl http://localhost:8000/health
```

Expected response:
```json
{
  "status": "healthy",
  "database": "connected"
}
```

### 4.2 Test Registration

```bash
curl -X POST http://localhost:8000/api/auth/register \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"TestPass123","full_name":"Test User"}'
```

Expected response:
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer",
  "user": {
    "id": 1,
    "email": "test@example.com",
    "full_name": "Test User"
  }
}
```

### 4.3 Test Frontend Integration

1. Open `http://localhost:3000` in your browser
2. Click "Sign In" button
3. Click "Create account" link
4. Fill in registration form
5. Submit and verify successful registration

---

## 5. Docker Setup (Optional)

### 5.1 Backend Dockerfile

A `Dockerfile` is provided in the `backend` directory:

```dockerfile
FROM python:3.9-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### 5.2 Build and Run with Docker

```bash
# Build backend image
docker build -t textbook-backend backend/

# Run backend container
docker run -d \
  --name textbook-backend \
  -p 8000:8000 \
  --env-file backend/.env \
  textbook-backend
```

---

## 6. Production Deployment

### 6.1 Environment Variables for Production

Update your `.env` file for production:

```env
# Production database
DATABASE_URL=postgresql://user:pass@production-host/dbname?sslmode=require

# Strong SECRET_KEY (must be changed from development)
SECRET_KEY=<generate-with-python-command>

# Production frontend URL
FRONTEND_URL=https://your-domain.com

# API Keys
OPENROUTER_API_KEY=your-production-key
COHERE_API_KEY=your-production-key
```

### 6.2 Railway Deployment

1. Connect your GitHub repository to Railway
2. Add environment variables in Railway dashboard
3. Deploy the backend service

### 6.3 Frontend Deployment

The frontend is built with Docusaurus. Deploy to Vercel, Netlify, or similar:

```bash
npm run build
```

Then deploy the `build/` directory.

---

## 7. Troubleshooting

### Database Connection Error

```
Error: could not connect to server: Connection refused
```

**Solution:**
- Verify `DATABASE_URL` is correct
- Check Neon database is active
- Ensure `sslmode=require` is in the connection string

### Import Error: No module named 'auth'

```
ModuleNotFoundError: No module named 'auth'
```

**Solution:**
- Ensure you're running from the `backend` directory
- Or add `backend/` to PYTHONPATH:
  ```bash
  export PYTHONPATH="${PYTHONPATH}:$(pwd)/backend"
  ```

### CORS Error in Browser

```
Access to XMLHttpRequest has been blocked by CORS policy
```

**Solution:**
- Check `FRONTEND_URL` in backend `.env` matches your frontend URL
- For local development, ensure both are using `http://localhost:3000`

### Token Validation Fails

**Solution:**
- Verify `SECRET_KEY` is the same on all backend instances
- Check token hasn't expired (7 day default)

---

## 8. Development Tips

### Running Tests

```bash
# Backend tests
cd backend
pytest tests/

# With coverage
pytest tests/ --cov=auth --cov-report=html
```

### Viewing Database

Use a PostgreSQL client like DBeaver, pgAdmin, or psql:

```bash
psql $DATABASE_URL
```

### Resetting Database

```sql
-- Drop all tables
DROP TABLE IF EXISTS password_reset_tokens CASCADE;
DROP TABLE IF EXISTS sessions CASCADE;
DROP TABLE IF EXISTS user_preferences CASCADE;
DROP TABLE IF EXISTS users CASCADE;
```

Then restart the backend to recreate tables.

---

## 9. Next Steps

After setup is complete:

1. Review [API Documentation](/docs/api/AUTHENTICATION_API.md)
2. Follow [Manual Testing Checklist](/docs/testing/MANUAL_TESTING_CHECKLIST.md)
3. Integrate with your application using the examples provided
4. Configure email service for production password reset emails

---

## 10. Support

For issues or questions:

- Check the [Troubleshooting](#7-troubleshooting) section
- Review API documentation
- Check logs in backend console
- Enable debug mode by setting `LOG_LEVEL=debug` in `.env`
