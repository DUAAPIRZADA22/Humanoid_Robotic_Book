# Complete Deployment Guide

## Overview

This guide covers deploying the **Humanoid Robotics Book Chatbot** with:
- **Backend**: FastAPI on Railway (Docker)
- **Frontend**: Docusaurus on Vercel
- **SSE Support**: Server-Sent Events for streaming chat
- **CORS**: Properly configured for cross-origin requests

---

## ✅ Fixes Applied (Already in GitHub)

| Issue | Fix | Commit |
|-------|-----|--------|
| Dockerfile Python 3.8 error | Uses `python3` (3.10) on Ubuntu 22.04 | `0275ce6` |
| .dockerignore excluding Dockerfile | Removed Dockerfile/railway.json from ignore list | `0275ce6` |
| CORS wildcard not working | Reads CORS_ORIGINS from environment | `841c342` |
| Frontend hardcoded localhost | Uses NEXT_PUBLIC_API_URL with fallback | `0a41cf6` |
| Auth components hardcoded API | Added getApiUrl() helper function | `0a41cf6` |

---

## 📋 Prerequisites

You will need:
- Railway account: https://railway.app
- Vercel account: https://vercel.com
- GitHub repository: https://github.com/DUAAPIRZADA22/Humanoid_Robotic_Book

---

## Part 1: Railway Backend Deployment

### Step 1: Create/Select Railway Project

1. Go to https://railway.app
2. Click **New Project** → **Deploy from GitHub repo**
3. Select: `DUAAPIRZADA22/Humanoid_Robotic_Book`
4. Set **Root Directory**: `backend`
5. Click **Deploy**

### Step 2: Set Environment Variables (CRITICAL)

Go to your Railway project → **Variables** tab → Add these variables:

```bash
# ===== REQUIRED VARIABLES =====

# OpenRouter API (for AI responses)
OPENROUTER_API_KEY=your_openrouter_key_here
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1

# Qdrant Vector Database
QDRANT_URL=https://6740646f-3e31-418e-a10b-225c3324c6ca.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=humanoid_robotics_book_openrouter

# Authentication
JWT_SECRET_KEY=generate_a_secure_random_string_here

# ===== CORS VARIABLES (CRITICAL FOR VERCEL) =====
# Must be valid JSON format with double quotes and brackets
CORS_ORIGINS=["https://humanoid-robotic-book-livid.vercel.app"]
CORS_METHODS=["GET", "POST", "PUT", "DELETE", "OPTIONS"]
CORS_HEADERS=["Content-Type", "Authorization", "X-API-Key"]

# ===== APP SETTINGS =====
PORT=8000
PYTHONUNBUFFERED=1
```

**Important Notes:**
- ⚠️ CORS variables must be valid JSON: `["url"]` not `url` or `['url']`
- ⚠️ Use double quotes, not single quotes
- ✅ Correct: `["https://humanoid-robotic-book-livid.vercel.app"]`
- ❌ Wrong: `https://humanoid-robotic-book-livid.vercel.app`

### Step 3: Redeploy

After adding variables:
1. Click **Redeploy** button
2. Wait for build to complete (~2-3 minutes)
3. Check logs for: `CORS configured for origins: ['https://humanoid-robotic-book-livid.vercel.app']`

### Step 4: Verify Backend

```bash
# Test health endpoint
curl https://humanoidroboticbook-production-ccff.up.railway.app/health

# Test CORS preflight
curl -X OPTIONS https://humanoidroboticbook-production-ccff.up.railway.app/chat \
  -H "Origin: https://humanoid-robotic-book-livid.vercel.app" \
  -H "Access-Control-Request-Method: POST" \
  -v
```

---

## Part 2: Vercel Frontend Deployment

### Step 1: Connect Repository to Vercel

1. Go to https://vercel.com
2. Click **Add New** → **Project**
3. Import: `DUAAPIRZADA22/Humanoid_Robotic_Book`
4. Click **Import**

### Step 2: Set Environment Variables

In **Configure Project** → **Environment Variables**, add:

| Name | Value | Environments |
|------|-------|-------------|
| `NEXT_PUBLIC_API_URL` | `https://humanoidroboticbook-production-ccff.up.railway.app` | ✅ Production, ✅ Preview, ✅ Development |

### Step 3: Deploy

1. Click **Deploy**
2. Wait for build to complete (~1-2 minutes)
3. Get your Vercel URL: `https://humanoid-robotic-book-livid.vercel.app`

### Step 4: Verify Frontend Configuration

Open browser console on your Vercel site and run:

```javascript
console.log(window.DOCUSAURUS_INSTALLED.siteConfig.customFields.chatApiEndpoint);
```

Should output: `https://humanoidroboticbook-production-ccff.up.railway.app`

---

## Part 3: Testing the Complete Setup

### Test 1: Health Check

```bash
curl https://humanoidroboticbook-production-ccff.up.railway.app/health
```

Expected response:
```json
{
  "status": "healthy",
  "components": {
    "chunker": true,
    "embedding_service": true,
    "vector_store": true,
    "retrieval_pipeline": true,
    "reranker": true,
    "llm_service": true
  }
}
```

### Test 2: CORS Preflight (OPTIONS)

```bash
curl -X OPTIONS https://humanoidroboticbook-production-ccff.up.railway.app/chat \
  -H "Origin: https://humanoid-robotic-book-livid.vercel.app" \
  -H "Access-Control-Request-Method: POST" \
  -H "Access-Control-Request-Headers: content-type" \
  -v 2>&1 | grep -i "access-control"
```

Expected headers:
```
< Access-Control-Allow-Origin: https://humanoid-robotic-book-livid.vercel.app
< Access-Control-Allow-Credentials: true
< Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS
< Access-Control-Allow-Headers: Content-Type, Authorization, X-API-Key
```

### Test 3: Chat Endpoint (SSE)

Visit your Vercel site: https://humanoid-robotic-book-livid.vercel.app

Open the chat widget and send a message. Should receive streaming responses.

**Or test via curl:**

```bash
curl -X POST https://humanoidroboticbook-production-ccff.up.railway.app/chat \
  -H "Origin: https://humanoid-robotic-book-livid.vercel.app" \
  -H "Content-Type: application/json" \
  -d '{"question": "What is humanoid robotics?", "stream": true}'
```

### Test 4: Local Development (Fallback)

```bash
# Start backend locally
cd backend && python main.py

# Start frontend locally
npm run start

# Open http://localhost:3000
# Chat should connect to http://localhost:8000
```

---

## File Structure Reference

### Backend Files

| File | Purpose |
|------|---------|
| `backend/Dockerfile` | Railway Docker build (Ubuntu 22.04 + Python 3) |
| `backend/main.py` | FastAPI app with CORS middleware |
| `backend/.env.railway` | Reference for Railway env variables |
| `backend/requirements.txt` | Python dependencies |

### Frontend Files

| File | Purpose |
|------|---------|
| `docusaurus.config.js` | Uses NEXT_PUBLIC_API_URL for backend |
| `.env.frontend.example` | Reference for local env variables |
| `src/contexts/AuthContext.tsx` | Uses getApiUrl() helper |
| `src/components/ChatWidget/hooks/useChatStream.ts` | Chat streaming with siteConfig |

---

## Troubleshooting

### Railway Build Fails

**Error:** `Package python3.8 has no installation candidate`

**Solution:** Already fixed in commit `0275ce6`. If still failing:
1. Go to Railway → Settings → Clear Build Cache
2. Click Redeploy

### CORS Error on Vercel

**Error:** `No 'Access-Control-Allow-Origin' header is present`

**Solution:**
1. Check Railway Variables: `CORS_ORIGINS=["https://humanoid-robotic-book-livid.vercel.app"]`
2. Verify JSON format (double quotes, brackets)
3. Redeploy Railway
4. Check logs for: `CORS configured for origins: [...]`

### "Failed to fetch" on Vercel

**Possible Causes:**
1. `NEXT_PUBLIC_API_URL` not set in Vercel
2. Railway backend not running
3. CORS not configured properly

**Solution:**
1. Set `NEXT_PUBLIC_API_URL` in Vercel Environment Variables
2. Redeploy Vercel (env vars are build-time in Docusaurus)
3. Verify Railway backend is running
4. Test backend health endpoint

### SSE Streaming Not Working

**Symptoms:** Chat doesn't show streaming responses

**Solution:**
1. Open browser DevTools → Network tab
2. Look for `/chat` request
3. Check if it's using SSE (event-stream content type)
4. Verify CORS headers are present
5. Check for any JavaScript errors in console

### Local Development Not Working

**Problem:** Frontend can't connect to local backend

**Solution:**
1. Start backend: `cd backend && python main.py`
2. Check it's running on `http://localhost:8000`
3. Verify logs show: `CORS configured for origins: [...]`
4. Restart frontend: `npm run start`

---

## Environment Variables Quick Reference

### Railway (Backend)

```bash
# Copy these exactly into Railway Variables
CORS_ORIGINS=["https://humanoid-robotic-book-livid.vercel.app"]
CORS_METHODS=["GET", "POST", "PUT", "DELETE", "OPTIONS"]
CORS_HEADERS=["Content-Type", "Authorization", "X-API-Key"]

# Plus your actual API keys
OPENROUTER_API_KEY=sk-or-v1-xxxxx
QDRANT_API_KEY=eyJxxxxx
JWT_SECRET_KEY=generate_secure_string
PORT=8000
```

### Vercel (Frontend)

```bash
# Copy this exactly into Vercel Environment Variables
NEXT_PUBLIC_API_URL=https://humanoidroboticbook-production-ccff.up.railway.app
```

---

## Verification Checklist

After deployment, verify:

- [ ] Railway builds successfully (no Python 3.8 errors)
- [ ] Railway health endpoint returns 200 OK
- [ ] Railway logs show: `CORS configured for origins: ['https://humanoid-robotic-book-livid.vercel.app']`
- [ ] Vercel frontend loads without errors
- [ ] Browser console shows correct API URL
- [ ] Chat widget sends messages successfully
- [ ] Chat responses stream in real-time (SSE working)
- [ ] No "Failed to fetch" errors
- [ ] No CORS errors in browser console
- [ ] Local development still works with `http://localhost:8000`

---

## Summary of All Fixes

### A. Dockerfile (Fixed)

**Before:** Used `python3.10`, `python3.10-dev`, `libgthread-2.0-0` (not available on Ubuntu 22.04)

**After:** Uses `python3`, `python3-pip`, `python3-dev`, `ca-certificates`

```dockerfile
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    python3 python3-pip python3-dev python3-venv \
    build-essential curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*
```

### B. CORS Configuration (Fixed)

**Before:** Used wildcard origins with credentials (doesn't work)

**After:** Reads from environment variables

```python
def parse_cors_origins() -> List[str]:
    cors_origins_str = os.getenv("CORS_ORIGINS")
    if cors_origins_str:
        try:
            import json
            origins = json.loads(cors_origins_str)
            if isinstance(origins, list):
                return origins
        except json.JSONDecodeError:
            logger.warning(f"Failed to parse CORS_ORIGINS: {cors_origins_str}")
    return [
        "https://humanoid-robotic-book-livid.vercel.app",
        "http://localhost:3000",
        "http://localhost:8000",
    ]

app.add_middleware(
    CORSMiddleware,
    allow_origins=parse_cors_origins(),
    allow_credentials=True,
    allow_methods=parse_cors_list("CORS_METHODS", ["GET", "POST", "PUT", "DELETE", "OPTIONS"]),
    allow_headers=parse_cors_list("CORS_HEADERS", ["Content-Type", "Authorization", "X-API-Key"]),
)
```

### C. Frontend API Configuration (Fixed)

**Before:** Hardcoded `http://localhost:8000` or `http://localhost:7860`

**After:** Uses `NEXT_PUBLIC_API_URL` with fallback

```javascript
// docusaurus.config.js
customFields: {
  chatApiEndpoint: process.env.NEXT_PUBLIC_API_URL ||
    (process.env.NODE_ENV === 'production'
      ? 'https://humanoidroboticbook-production-ccff.up.railway.app'
      : 'http://localhost:8000'),
}
```

### D. Auth Components (Fixed)

**Before:** Used `process.env?.API_URL` (doesn't work in Docusaurus)

**After:** Uses `getApiUrl()` helper function

```typescript
function getApiUrl(): string {
  if (typeof window !== 'undefined' && (window as any).DOCUSAURUS_INSTALLED) {
    const siteConfig = (window as any).DOCUSAURUS_INSTALLED?.siteConfig;
    if (siteConfig?.customFields?.chatApiEndpoint) {
      return siteConfig.customFields.chatApiEndpoint;
    }
  }
  return 'http://localhost:8000';
}

// Usage
const response = await fetch(`${getApiUrl()}/api/auth/signin`, {...});
```

---

## Git Commits Reference

| Commit | Description |
|--------|-------------|
| `25f2803` | Add Railway deployment setup guide |
| `0275ce6` | Fix Dockerfile for Railway (remove Python 3.8, libgthread) |
| `841c342` | Enable proper CORS for Vercel frontend |
| `0a41cf6` | Use env-based API URL for Railway backend |

All changes are pushed to GitHub and ready for deployment.

---

## Need Help?

1. **Railway Documentation:** https://docs.railway.app
2. **Vercel Documentation:** https://vercel.com/docs
3. **FastAPI CORS:** https://fastapi.tiangolo.com/tutorial/cors/
4. **Docusaurus Environment Variables:** https://docusaurus.io/docs/docusaurus-core
