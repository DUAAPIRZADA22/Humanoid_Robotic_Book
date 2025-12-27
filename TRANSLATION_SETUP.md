# Translation Service Quick Setup

Complete guide to set up the Docusaurus Translation Service.

## Prerequisites

1. **Neon PostgreSQL Account**: https://neon.tech (free tier available)
2. **OpenRouter API Key**: https://openrouter.ai/ (free tier available)
3. **Node.js 18+**: https://nodejs.org/

## Step 1: Get Neon Database URL

1. Sign up at https://neon.tech
2. Create a new project
3. Copy the Connection String (looks like `postgresql://...`)
4. Save it for Step 3

## Step 2: Get OpenRouter API Key

1. Sign up at https://openrouter.ai
2. Go to https://openrouter.ai/keys
3. Create a new API key
4. Save it for Step 3

## Step 3: Configure Translation Service

```bash
cd translation-service
cp .env.example .env
```

Edit `.env`:

```env
# Required: Replace with your actual values
OPENROUTER_API_KEY=sk-or-v1-xxxxx
NEON_DATABASE_URL=postgresql://user:password@ep-xxx.aws.neon.tech/neondb?sslmode=require

# Optional: Defaults shown
PORT=3001
TRANSLATION_MODEL=google/gemma-3-27b-it:free
DEFAULT_TARGET_LANG=ur
```

## Step 4: Install & Run

```bash
# Install dependencies
npm install

# Run database migration
npm run db:migrate

# Start server
npm start
```

You should see:
```
✅ Neon database connected
🚀 Translation service running on http://0.0.0.0:3001
```

## Step 5: Test the API

```bash
curl http://localhost:3001/api/health
```

Expected response:
```json
{
  "service": "docusaurus-translation",
  "status": "running",
  "translator": {
    "status": "healthy",
    "model": "google/gemma-3-27b-it:free"
  }
}
```

## Step 6: Test Translation

```bash
curl -X POST http://localhost:3001/api/translate \
  -H "Content-Type: application/json" \
  -d '{
    "content": "# Hello World\n\nThis is a test.",
    "pageSlug": "test",
    "targetLang": "ur"
  }'
```

## Step 7: Start Docusaurus

In a new terminal:

```bash
# From project root
npm start
```

The Translate Button (🌐 Urdu) will appear in the navbar.

## Troubleshooting

### `NEON_DATABASE_URL not set`
- Make sure you created `.env` file in `translation-service/`
- Check the URL format is correct

### `Invalid API key`
- Verify your OpenRouter key at https://openrouter.ai/keys
- Ensure no extra spaces in `.env` file

### `Migration failed`
- Check Neon database is active
- Verify database URL includes `?sslmode=require`

### Translate button not appearing
- Clear browser cache (Ctrl+Shift+R)
- Check Docusaurus is running
- Inspect browser console for errors

### Translation not working
- Check translation service is running on port 3001
- Verify CORS settings: `ALLOWED_ORIGINS=http://localhost:3000`
- Check browser Network tab for failed requests

## Production Deployment

### Railway (Recommended)

```bash
# Install Railway CLI
npm install -g railway

# Login and link project
cd translation-service
railway login
railway init

# Set environment variables
railway variables set OPENROUTER_API_KEY=sk-or-v1-xxxxx
railway variables set NEON_DATABASE_URL=postgresql://...
railway variables set PORT=3001

# Deploy
railway up
```

Update `docusaurus.config.js`:
```javascript
translateApiUrl: 'https://your-app.railway.app/api',
```

### Environment Variables for Production

Add to `.env`:

```env
NODE_ENV=production
PORT=3001
ALLOWED_ORIGINS=https://your-site.vercel.app
```

## Cost Estimates

### Free Tier (Default)
- **Model**: `google/gemma-3-27b-it:free`
- **Cost**: $0/month
- **Rate Limit**: 20 requests/day
- **Best for**: Development, testing

### Paid Tier
- **Model**: `anthropic/claude-3-haiku`
- **Cost**: ~$0.25 per 1M characters
- **Rate Limit**: Higher limits
- **Best for**: Production

For a typical 1000-character page:
- Free tier: 0 pages/day after 20 pages
- Paid tier: ~8000 pages for $1

## Next Steps

1. ✅ Translation service running
2. ✅ Docusaurus showing translate button
3. ✅ Test translations working
4. [Optional] Deploy to Railway
5. [Optional] Upgrade to paid OpenRouter model
6. [Optional] Add more languages (edit `src/config/openrouter.js`)

## Support

For issues:
1. Check logs: `cd translation-service && npm run dev`
2. Verify environment variables
3. Test API health: `curl http://localhost:3001/api/health`
4. Open an issue on GitHub
