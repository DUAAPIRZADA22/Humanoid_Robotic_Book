# Frontend API Setup Guide

## Overview

This guide explains how the frontend connects to the backend using environment variables and how to configure it for both local development and Vercel deployment.

---

## Configuration Files Updated

### 1. `docusaurus.config.js` (Main Configuration)

```javascript
customFields: {
  // Backend API URL - use NEXT_PUBLIC_API_URL env var, fallback to Railway/localhost
  chatApiEndpoint: process.env.NEXT_PUBLIC_API_URL ||
    (process.env.NODE_ENV === 'production'
      ? 'https://humanoidroboticbook-production-ccff.up.railway.app'
      : 'http://localhost:8000'),
  chatApiKey: process.env.NEXT_PUBLIC_API_KEY || '',
  // Translation service API URL (same as backend)
  translateApiUrl: process.env.NEXT_PUBLIC_API_URL ||
    (process.env.NODE_ENV === 'production'
      ? 'https://humanoidroboticbook-production-ccff.up.railway.app'
      : 'http://localhost:8000'),
},
```

### 2. Components Using the API URL

All frontend components now use the `getApiUrl()` helper function that reads from the Docusaurus site config:

| File | Purpose |
|------|---------|
| `src/contexts/AuthContext.tsx` | Authentication (sign in, sign up) |
| `src/components/auth/ProfileSettings.tsx` | User profile updates |
| `src/components/auth/PasswordResetModals.tsx` | Password reset |
| `src/components/ChatWidget/hooks/useChatStream.ts` | Chat streaming (via siteConfig) |
| `src/components/TranslateButton/index.tsx` | Translation service (via siteConfig) |

---

## Environment Variables

### Development (Local)

Create a `.env` file in your project root:

```bash
# For local backend (default)
NEXT_PUBLIC_API_URL=http://localhost:8000

# Or directly test against Railway backend
# NEXT_PUBLIC_API_URL=https://humanoidroboticbook-production-ccff.up.railway.app
```

### Production (Vercel)

Set `NEXT_PUBLIC_API_URL` to your Railway backend URL.

---

## Vercel Environment Variables Setup

### Step-by-Step Guide

1. **Go to Vercel Dashboard**
   - Visit: https://vercel.com/dashboard
   - Select your project: `humanoid-robotic-book`

2. **Navigate to Environment Variables**
   - Go to **Settings** → **Environment Variables**

3. **Add the Environment Variable**
   - Click **Add New**
   - Fill in:
     | Field | Value |
     |-------|-------|
     | **Name** | `NEXT_PUBLIC_API_URL` |
     | **Value** | `https://humanoidroboticbook-production-ccff.up.railway.app` |
     | **Environments** | ✅ Production, ✅ Preview, ✅ Development |
   - Click **Save**

4. **Redeploy Your Application**
   - Go to **Deployments** tab
   - Click the three dots (⋯) next to the latest deployment
   - Click **Redeploy**

   This is **required** because Docusaurus resolves environment variables at **build time**.

---

## Example: Chat Request Using Fetch

Here's a complete example of how to make a chat request using the configured API URL:

### Using the ChatWidget Component (Recommended)

The ChatWidget component already handles all the API communication. Here's how to use it:

```tsx
import { ChatWidget } from '@site/src/components/ChatWidget';

export default function YourPage() {
  return (
    <div>
      <h1>Chat with the AI Assistant</h1>
      <ChatWidget />
    </div>
  );
}
```

### Direct Fetch Example (Advanced)

If you need to make direct API calls:

```typescript
/**
 * Get API URL from Docusaurus config
 */
function getApiUrl(): string {
  if (typeof window !== 'undefined' && (window as any).DOCUSAURUS_INSTALLED) {
    const siteConfig = (window as any).DOCUSAURUS_INSTALLED?.siteConfig;
    if (siteConfig?.customFields?.chatApiEndpoint) {
      return siteConfig.customFields.chatApiEndpoint;
    }
  }
  return 'http://localhost:8000';
}

/**
 * Send a chat message with streaming support
 */
async function sendChatMessage(
  question: string,
  onChunk: (chunk: string) => void
): Promise<void> {
  const apiUrl = getApiUrl();

  const response = await fetch(`${apiUrl}/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      // Add auth token if user is logged in
      // 'Authorization': `Bearer ${token}`,
    },
    body: JSON.stringify({
      question,
      top_k: 3,
      similarity_threshold: 0.2,
      use_rerank: false,
      stream: true,
    }),
  });

  if (!response.ok) {
    throw new Error(`HTTP ${response.status}: ${response.statusText}`);
  }

  // Handle streaming response
  const reader = response.body?.getReader();
  const decoder = new TextDecoder();
  let buffer = '';

  while (true) {
    const { done, value } = await reader!.read();
    if (done) break;

    buffer += decoder.decode(value, { stream: true });
    const lines = buffer.split('\n');
    buffer = lines.pop() || '';

    for (const line of lines) {
      if (line.trim().startsWith('data: ')) {
        const data = line.trim().slice(6);
        if (data === '[DONE]') return;

        try {
          const chunk = JSON.parse(data);
          if (chunk.type === 'content') {
            onChunk(chunk.content);
          }
        } catch (e) {
          // Skip invalid JSON
        }
      }
    }
  }
}

// Usage:
sendChatMessage('What is humanoid robotics?', (chunk) => {
  console.log('Received:', chunk);
});
```

### Non-Streaming Example

```typescript
async function sendSimpleChatMessage(question: string): Promise<string> {
  const apiUrl = getApiUrl();

  const response = await fetch(`${apiUrl}/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      question,
      stream: false, // Disable streaming
    }),
  });

  if (!response.ok) {
    throw new Error(`HTTP ${response.status}: ${response.statusText}`);
  }

  const data = await response.json();
  return data.answer;
}
```

---

## API Endpoints Reference

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/chat` | POST | Send chat message (streaming) |
| `/health` | GET | Check API health |
| `/api/auth/signin` | POST | User sign in |
| `/api/auth/register` | POST | User registration |
| `/api/auth/profile` | PUT | Update user profile |
| `/api/auth/reset-request` | POST | Request password reset |
| `/api/auth/reset-confirm` | POST | Confirm password reset |
| `/api/translate` | POST | Translate page content |

---

## Troubleshooting

### "Failed to fetch" Error on Vercel

**Cause**: The `NEXT_PUBLIC_API_URL` environment variable is not set or the site hasn't been rebuilt.

**Solution**:
1. Check that `NEXT_PUBLIC_API_URL` is set in Vercel Environment Variables
2. Redeploy the application from the Deployments tab
3. Check the browser console for the actual URL being used

### CORS Errors

**Cause**: The backend CORS configuration doesn't allow your Vercel domain.

**Solution**: Add your Vercel domain to the backend CORS settings in `backend/.env`:
```bash
CORS_ORIGINS=["https://humanoid-robotic-book.vercel.app", "https://your-username.vercel.app", "http://localhost:3000"]
```

### Local Development Not Working

**Cause**: The backend is not running on port 8000.

**Solution**:
1. Start the backend: `cd backend && python main.py`
2. Verify it's running: `curl http://localhost:8000/health`
3. Check that no firewall is blocking the connection

---

## Important Notes

### Build-Time vs Runtime Variables

| Platform | Environment Variable Resolution |
|----------|-------------------------------|
| **Next.js** | `NEXT_PUBLIC_*` variables are bundled at build time and available in browser |
| **Docusaurus** | All `process.env` variables are resolved at **build time** only |

This means:
- Changing `NEXT_PUBLIC_API_URL` requires a **rebuild/redeploy**
- The environment variable value is "baked in" during the build
- Cannot be changed at runtime without redeploying

### Security Considerations

- `NEXT_PUBLIC_*` variables are exposed in the browser bundle
- Never store secrets in `NEXT_PUBLIC_*` variables
- The backend URL is public (which is fine for this use case)
- API keys should only be used if they are non-sensitive or properly secured

---

## Testing the Setup

### 1. Test Locally

```bash
# Start the backend
cd backend && python main.py

# Start the frontend (in another terminal)
npm run start

# Open browser to http://localhost:3000
# Open DevTools Console and run:
console.log(window.DOCUSAURUS_INSTALLED.siteConfig.customFields.chatApiEndpoint);
# Should output: "http://localhost:8000"
```

### 2. Test on Vercel

After deployment:
1. Open your Vercel URL
2. Open DevTools Console
3. Run:
```javascript
console.log(window.DOCUSAURUS_INSTALLED.siteConfig.customFields.chatApiEndpoint);
```
Should output: `https://humanoidroboticbook-production-ccff.up.railway.app`

---

## Summary

| Task | Command/Action |
|------|---------------|
| **Local Development** | `NEXT_PUBLIC_API_URL=http://localhost:8000 npm run start` |
| **Set Vercel Variable** | Settings → Environment Variables → Add `NEXT_PUBLIC_API_URL` |
| **Redeploy on Vercel** | Deployments → ⋯ → Redeploy |
| **Verify Config** | `console.log(window.DOCUSAURUS_INSTALLED.siteConfig.customFields.chatApiEndpoint)` |

---

## Files Modified

| File | Changes |
|------|---------|
| `docusaurus.config.js` | Updated `customFields.chatApiEndpoint` to use `NEXT_PUBLIC_API_URL` |
| `.env.frontend.example` | Added `NEXT_PUBLIC_API_URL` documentation |
| `src/contexts/AuthContext.tsx` | Added `getApiUrl()` helper function |
| `src/components/auth/ProfileSettings.tsx` | Added `getApiUrl()` helper function |
| `src/components/auth/PasswordResetModals.tsx` | Added `getApiUrl()` helper function |
