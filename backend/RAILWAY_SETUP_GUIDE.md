# Railway Deployment Setup Guide

## Fixed Issues

### A. Dockerfile Build Error ✅ FIXED

**Problem:**
```
Package python3.8 has no installation candidate
Unable to locate package python3.8-venv
Unable to locate package libgthread-2.0-0
```

**Solution Applied:**
- Updated Dockerfile to use Ubuntu 22.04 compatible packages
- Changed from `python3.10` to `python3` (which is 3.10 on Ubuntu 22.04)
- Removed `libgthread-2.0-0` (not needed)
- Fixed `.dockerignore` which was excluding `Dockerfile`

**New Dockerfile:**
```dockerfile
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    build-essential \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN ln -sf /usr/bin/python3 /usr/bin/python && \
    ln -sf /usr/bin/pip3 /usr/bin/pip

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir -r requirements.txt

COPY . .

RUN useradd -m -u 1000 appuser && \
    chown -R appuser:appuser /app

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PORT=8000

USER appuser
EXPOSE 8000

HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
    CMD curl -f http://localhost:${PORT:-8000}/health || exit 1

CMD exec uvicorn main:app --host 0.0.0.0 --port ${PORT:-8000}
```

---

### B. CORS Error for Vercel Frontend ⚠️ NEEDS MANUAL SETUP

**Problem:**
```
No 'Access-Control-Allow-Origin' header is present
```

**Solution:** Set CORS environment variables in Railway Dashboard

---

## Railway CORS Setup Instructions

### Step 1: Go to Railway Dashboard

1. Visit: https://railway.app
2. Select your project: `humanoidroboticbook-production-ccff`

### Step 2: Add CORS Environment Variables

Go to **Variables** tab and add these **exact** values:

| Variable Name | Value | Notes |
|---------------|-------|-------|
| `CORS_ORIGINS` | `["https://humanoid-robotic-book-livid.vercel.app"]` | Must be valid JSON! |
| `CORS_METHODS` | `["GET", "POST", "PUT", "DELETE", "OPTIONS"]` | Must be valid JSON! |
| `CORS_HEADERS` | `["Content-Type", "Authorization", "X-API-Key"]` | Must be valid JSON! |

**Important Notes:**
- ⚠️ **Must be valid JSON format** with brackets and double quotes
- ✅ Correct: `["https://example.com"]`
- ❌ Wrong: `https://example.com`
- ❌ Wrong: `['https://example.com']` (single quotes)

**For local development testing**, you can add multiple origins:
```
["https://humanoid-robotic-book-livid.vercel.app", "http://localhost:3000"]
```

### Step 3: Redeploy on Railway

1. Click the **Redeploy** button in Railway
2. Or push a new commit to trigger automatic rebuild

### Step 4: Verify CORS is Working

Check the deployment logs for:
```
CORS configured for origins: ['https://humanoid-robotic-book-livid.vercel.app']
```

---

## How CORS Works with SSE (Server-Sent Events)

The backend uses FastAPI's CORSMiddleware which properly handles:

1. **Pre-flight OPTIONS requests** - Automatically handled
2. **SSE streaming** - Works with CORS when credentials are enabled
3. **Custom headers** - `Access-Control-Allow-Origin`, `Access-Control-Allow-Credentials`, etc.

### CORS Headers Sent by Backend:

```
Access-Control-Allow-Origin: https://humanoid-robotic-book-livid.vercel.app
Access-Control-Allow-Credentials: true
Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS
Access-Control-Allow-Headers: Content-Type, Authorization, X-API-Key
```

---

## Testing the Setup

### 1. Test Health Endpoint

```bash
curl https://humanoidroboticbook-production-ccff.up.railway.app/health
```

Should return:
```json
{"status": "healthy", "components": {...}}
```

### 2. Test CORS Preflight

```bash
curl -X OPTIONS https://humanoidroboticbook-production-ccff.up.railway.app/chat \
  -H "Origin: https://humanoid-robotic-book-livid.vercel.app" \
  -H "Access-Control-Request-Method: POST" \
  -v
```

Should include headers:
```
Access-Control-Allow-Origin: https://humanoid-robotic-book-livid.vercel.app
Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS
```

### 3. Test Chat Endpoint (SSE)

```bash
curl -X POST https://humanoidroboticbook-production-ccff.up.railway.app/chat \
  -H "Origin: https://humanoid-robotic-book-livid.vercel.app" \
  -H "Content-Type: application/json" \
  -d '{"question": "Hello", "stream": true}' \
  -v
```

---

## Troubleshooting

### Railway Build Still Failing

1. **Clear Railway cache:**
   - Go to Railway project
   - Click **Settings** → **Clear Build Cache**
   - Click **Redeploy**

2. **Check recent deployments:**
   - Go to **Deployments** tab
   - Click on the latest deployment to see logs

### CORS Error Still Occurring

1. **Verify environment variables are set:**
   - Go to Railway **Variables** tab
   - Make sure `CORS_ORIGINS`, `CORS_METHODS`, `CORS_HEADERS` are set
   - Check for correct JSON format (double quotes, brackets)

2. **Check deployment logs:**
   - Look for: `CORS configured for origins: [...]`
   - If not present, the app didn't start properly

3. **Test with curl:**
   ```bash
   curl -X OPTIONS https://your-backend.railway.app/chat \
     -H "Origin: https://humanoid-robotic-book-livid.vercel.app" \
     -H "Access-Control-Request-Method: POST" \
     -v 2>&1 | grep -i "access-control"
   ```

### SSE Streaming Not Working

The FastAPI CORSMiddleware properly handles SSE. If streaming fails:

1. Check browser console for errors
2. Verify the frontend is using the correct backend URL
3. Make sure `NEXT_PUBLIC_API_URL` is set in Vercel

---

## Summary

| Issue | Status | Action Required |
|-------|--------|-----------------|
| Dockerfile Python 3.8 error | ✅ Fixed | Committed to GitHub |
| .dockerignore excluding Dockerfile | ✅ Fixed | Committed to GitHub |
| CORS configuration in code | ✅ Fixed | Committed to GitHub |
| CORS environment variables in Railway | ⚠️ Pending | **YOU MUST SET THESE MANUALLY** |

### Next Steps:

1. ✅ Dockerfile fixes pushed to GitHub (`0275ce6`)
2. ⚠️ **Set CORS variables in Railway Dashboard** (see above)
3. ⚠️ **Click Redeploy in Railway**
4. ⚠️ **Set `NEXT_PUBLIC_API_URL` in Vercel** (if not already set)
5. ✅ Test your Vercel frontend

---

## Railway Variables Reference

Copy and paste these exact values into Railway Variables:

```
CORS_ORIGINS=["https://humanoid-robotic-book-livid.vercel.app"]
CORS_METHODS=["GET", "POST", "PUT", "DELETE", "OPTIONS"]
CORS_HEADERS=["Content-Type", "Authorization", "X-API-Key"]
```

**Other required variables:**
```
OPENROUTER_API_KEY=your_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_key_here
JWT_SECRET_KEY=your_secret_here
PORT=8000
```
