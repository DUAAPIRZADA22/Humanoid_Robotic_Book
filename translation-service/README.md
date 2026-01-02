# Docusaurus Translation Service

Production-ready AI translation service for Docusaurus sites using OpenAI Agents SDK + OpenRouter + Neon PostgreSQL.

## Features

- **Instant Translation**: Translate pages to Urdu (or any language) with a single click
- **Smart Caching**: Neon PostgreSQL cache stores translations for instant retrieval
- **Cost-Effective**: Uses free OpenRouter models by default
- **Markdown Preservation**: Preserves headings, code blocks, links, and formatting
- **Production-Ready**: Rate limiting, error handling, health checks, timeouts

## Architecture

```
┌─────────────────┐      ┌──────────────────┐      ┌─────────────────┐
│   Docusaurus    │─────▶│ Translation API  │─────▶│   OpenRouter    │
│  (Frontend)     │◀────▶│   (Node.js)      │      │    (LLM)        │
└─────────────────┘      │                  │      └─────────────────┘
                         │   + Neon Cache   │
                         └──────────────────┘
```

## Folder Structure

```
translation-service/
├── src/
│   ├── agents/
│   │   └── TranslatorAgent.js    # OpenAI Agents SDK translator
│   ├── api/
│   │   └── translate.js          # /translate API endpoint
│   ├── config/
│   │   └── openrouter.js         # Model & language configs
│   ├── db/
│   │   ├── client.js             # Neon database client
│   │   ├── migrate.js            # Database migration script
│   │   └── schema.sql            # Database schema
│   └── index.js                  # Main server entry
├── package.json
├── .env.example
└── README.md
```

## Neon Database Schema

```sql
CREATE TABLE translations (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  page_slug VARCHAR(500) NOT NULL,
  language_code VARCHAR(10) NOT NULL,
  original_text TEXT NOT NULL,
  translated_text TEXT NOT NULL,
  content_hash VARCHAR(64) NOT NULL,
  model_used VARCHAR(100) NOT NULL,
  character_count INTEGER NOT NULL,
  created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
  last_accessed_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP,
  CONSTRAINT unique_page_language UNIQUE (page_slug, language_code)
);

CREATE INDEX idx_translations_lookup ON translations(page_slug, language_code);
```

## Setup Instructions

### 1. Prerequisites

- Node.js 18+
- Neon PostgreSQL account
- OpenRouter API key (free tier available)

### 2. Install Dependencies

```bash
cd translation-service
npm install
```

### 3. Configure Environment

```bash
cp .env.example .env
```

Edit `.env`:

```env
# Server
PORT=3001
NODE_ENV=production

# OpenRouter
OPENROUTER_API_KEY=sk-or-v1-xxxxx
OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
TRANSLATION_MODEL=google/gemma-3-27b-it:free

# Neon PostgreSQL
NEON_DATABASE_URL=postgresql://user:pass@host/db?sslmode=require

# Translation Settings
DEFAULT_TARGET_LANG=ur
MAX_CONTENT_LENGTH=100000
TRANSLATION_TIMEOUT=30000

# Rate Limiting
RATE_LIMIT_MAX=100
RATE_LIMIT_WINDOW=60000

# CORS
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001
```

### 4. Run Database Migration

```bash
npm run db:migrate
```

Expected output:
```
🔄 Connecting to Neon PostgreSQL...
📜 Applying schema...
✅ Migration completed successfully!
✅ Verified: "translations" table exists
```

### 5. Start Server

```bash
# Development (with hot reload)
npm run dev

# Production
npm start
```

Server will run on `http://localhost:3001`

## API Endpoints

### POST /api/translate

Translate page content.

**Request:**
```json
{
  "content": "# Introduction\n\nThis is a test page.",
  "pageSlug": "introduction",
  "targetLang": "ur",
  "skipCache": false
}
```

**Response:**
```json
{
  "success": true,
  "translatedText": "# تعارف\n\nیہ ایک ٹیسٹ پیج ہے۔",
  "originalText": "# Introduction\n\nThis is a test page.",
  "language": "ur",
  "languageName": "Urdu",
  "direction": "rtl",
  "fromCache": false,
  "characterCount": 38,
  "model": "google/gemma-3-27b-it:free"
}
```

### GET /api/health

Check service health.

**Response:**
```json
{
  "service": "docusaurus-translation",
  "status": "running",
  "translator": {
    "status": "healthy",
    "model": "google/gemma-3-27b-it:free",
    "testTranslation": "OK"
  },
  "cache": {
    "enabled": true,
    "total_translations": 42,
    "unique_pages": 15
  }
}
```

### GET /api/cache/stats

Get cache statistics.

### DELETE /api/cache?clear=all

Clear all cached translations.

## Frontend Integration

### 1. Configure Docusaurus

Add to `docusaurus.config.js`:

```javascript
export default {
  // ... other config
  customFields: {
    translateApiUrl: 'http://localhost:3001/api',
  },
};
```

### 2. Translate Button

The Translate Button is already integrated in your Navbar at `src/theme/Navbar/index.tsx`.

It automatically:
- Detects current page content
- Sends translation request
- Updates page with translated text
- Supports RTL languages (Urdu, Arabic, etc.)
- Shows loading/error states

### 3. Programmatic Usage

```typescript
import { useTranslation } from '@site/src/components/TranslateButton';

function MyComponent() {
  const { translate, restore, isTranslated, currentLang } = useTranslation();

  const handleTranslate = async () => {
    const translated = await translate(content, 'my-page', 'ur');
    console.log(translated);
  };

  return (
    <button onClick={handleTranslate}>
      {isTranslated ? 'Restore' : 'Translate'}
    </button>
  );
}
```

## Supported Languages

| Code | Name | Direction |
|------|------|-----------|
| `ur` | Urdu | RTL |
| `ar` | Arabic | RTL |
| `hi` | Hindi | LTR |
| `bn` | Bengali | LTR |
| `fa` | Persian | RTL |
| `es` | Spanish | LTR |
| `fr` | French | LTR |
| `de` | German | LTR |

Add more in `src/config/openrouter.js`.

## OpenRouter Models

### Free Models (Default)
- `google/gemma-3-27b-it:free` - Good quality, no cost
- `meta-llama/llama-3-8b-instruct:free` - Fast, decent quality

### Paid Models (Better Quality)
- `anthropic/claude-3-haiku` - Excellent quality
- `openai/gpt-4o-mini` - Great quality, fast

### Premium Models (Best Quality)
- `anthropic/claude-3.5-sonnet` - Best quality
- `openai/gpt-4o` - Top tier

## Error Handling

The service handles:
- **Timeout**: 30s timeout with user-friendly message
- **Rate Limiting**: 100 requests per minute per IP
- **Empty Content**: Validates content before translation
- **API Errors**: Graceful degradation with clear messages
- **Database Failures**: Continues without cache if DB unavailable

## Deployment

### Railway

```bash
# Install Railway CLI
npm install -g railway

# Login
railway login

# Create project
railway init

# Add variables
railway variables set OPENROUTER_API_KEY=sk-or-v1-xxxxx
railway variables set NEON_DATABASE_URL=postgresql://...
railway variables set PORT=3001

# Deploy
railway up
```

### Docker

```dockerfile
FROM node:18-alpine
WORKDIR /app
COPY package*.json ./
RUN npm ci --only=production
COPY src ./src
EXPOSE 3001
CMD ["node", "src/index.js"]
```

## Monitoring

### Health Check

```bash
curl http://localhost:3001/api/health
```

### Cache Statistics

```bash
curl http://localhost:3001/api/cache/stats
```

### Logs

```bash
# View logs
npm run dev

# Production logs
pm2 logs translation-service
```

## Troubleshooting

### "Translation service configuration error"
- Check `OPENROUTER_API_KEY` is set correctly
- Verify API key is valid at https://openrouter.ai/keys

### "Database not configured"
- Check `NEON_DATABASE_URL` is set
- Run `npm run db:migrate` to create tables

### "Rate limit exceeded"
- Wait 60 seconds before retrying
- Consider upgrading to paid OpenRouter tier

### Translation quality issues
- Try a different model in `.env`: `TRANSLATION_MODEL=anthropic/claude-3-haiku`
- Ensure content is well-structured markdown

## License

MIT

## Support

For issues or questions, please open a GitHub issue.
