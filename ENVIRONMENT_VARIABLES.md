# Environment Variables Guide

This document provides a comprehensive guide to all environment variables used in the Physical AI & Humanoid Robotics Book project.

## Table of Contents

- [Quick Start](#quick-start)
- [Frontend Variables](#frontend-variables)
- [Backend Variables](#backend-variables)
- [RAG Configuration](#rag-configuration)
- [Deployment Variables](#deployment-variables)
- [Security Variables](#security-variables)
- [Optional Services](#optional-services)

## Quick Start

### 1. Copy the example files
```bash
# Frontend environment variables
cp .env.frontend.example .env.frontend.local

# Backend environment variables
cp backend/.env.example backend/.env.local
```

### 2. Fill in required variables
```bash
# Frontend (.env.frontend.local)
CHAT_API_ENDPOINT=http://localhost:7860/api/v1
CHAT_API_KEY=your_api_key_here

# Backend (backend/.env.local)
COHERE_API_KEY=your_cohere_api_key

# Required for Railway deployment (add to Railway dashboard)
ROOT_DIRECTORY=backend
PORT=7860
```

### 3. Never commit environment files with actual values
```bash
# Add to .gitignore
echo ".env.local" >> .gitignore
echo ".env.frontend.local" >> .gitignore
echo "backend/.env.local" >> .gitignore
```

## Frontend Variables

Frontend environment variables are configured in `.env.frontend.example` and should be copied to `.env.frontend.local` for local development.

### Chat Widget Configuration

| Variable | Type | Default | Required | Description |
|----------|------|---------|----------|-------------|
| `CHAT_API_ENDPOINT` | URL | `http://localhost:7860/api/v1` | Yes | Backend API endpoint |
| `CHAT_API_KEY` | String | `railway-demo-key` | Yes | API authentication key |
| `NODE_ENV` | String | `development` | No | Node.js environment |
| `DEPLOYMENT_URL` | URL | `https://humanoid-robotic-book.vercel.app` | No | Production deployment URL |
| `SITE_URL` | URL | `https://humanoid-robotic-book.vercel.app` | No | Site URL for sitemap and social cards |

#### Production Setup
```bash
# For production deployment in .env.frontend.local
CHAT_API_ENDPOINT=https://your-app.up.railway.app/api/v1
NODE_ENV=production
DEPLOYMENT_URL=https://your-app-name.vercel.app
SITE_URL=https://your-app-name.vercel.app
```

### Analytics & Tracking (Optional)

| Variable | Type | Description |
|----------|------|-------------|
| `GOOGLE_ANALYTICS_ID` | String | Google Analytics measurement ID |
| `SENTRY_DSN` | String | Sentry error tracking DSN |
| `HOTJAR_SITE_ID` | String | Hotjar analytics site ID |

### Integration Settings (Optional)

| Variable | Type | Description |
|----------|------|-------------|
| `ALGOLIA_APP_ID` | String | Algolia DocSearch app ID |
| `ALGOLIA_API_KEY` | String | Algolia DocSearch API key |
| `ALGOLIA_INDEX_NAME` | String | Algolia search index name |
| `GITHUB_REPO` | URL | GitHub repository URL for edit links |

## Backend Variables

Backend environment variables are configured in `backend/.env.example` and should be copied to `backend/.env.local` for local development.

### API Keys & Authentication

| Variable | Type | Required | Description |
|----------|------|----------|-------------|
| `COHERE_API_KEY` | String | **Yes** | Cohere AI API key for embeddings |
| `OPENAI_API_KEY` | String | Optional | Alternative AI service API key |
| `QDRANT_API_KEY` | String | Optional | Qdrant Cloud API key |

### Application Metadata

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `APP_NAME` | String | `Physical AI & Humanoid Robotics Book Chatbot` | Application name |
| `APP_VERSION` | String | `1.0.0` | Application version |
| `APP_DESCRIPTION` | String | `AI-powered chatbot for Physical AI and Humanoid Robotics content` | Application description |

### Vector Database Configuration

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `QDRANT_HOST` | String | `localhost` | Qdrant server host (local only) |
| `QDRANT_PORT` | Integer | `6333` | Qdrant server port (local only) |
| `QDRANT_URL` | String | - | Full Qdrant Cloud URL (cloud only) |
| `QDRANT_API_KEY` | String | `your_qdrant_api_key_if_cloud` | Cloud API key |
| `QDRANT_COLLECTION_NAME` | String | `humanoid_robotics_book` | Default collection name |
| `QDRANT_VECTOR_SIZE` | Integer | `1024` | Embedding dimension |
| `QDRANT_DISTANCE_METRIC` | String | `cosine` | Similarity metric (cosine, euclidean, dot) |
| `QDRANT_ON_DISK` | Boolean | `true` | Persist vectors to disk |
| `QDRANT_TIMEOUT` | Integer | `30` | Request timeout in seconds |

#### Local vs Cloud Configuration

**Local Development:**
```bash
QDRANT_HOST=localhost
QDRANT_PORT=6333
# QDRANT_URL= (leave empty)
```

**Qdrant Cloud (Production):**
```bash
QDRANT_URL=https://your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your_api_key
# QDRANT_HOST= (ignored when using URL)
```

### Application Configuration

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `APP_NAME` | String | `Physical AI & Humanoid Robotics Book Chatbot` | Application name |
| `APP_VERSION` | String | `1.0.0` | Application version |
| `DEBUG` | Boolean | `false` | Debug mode flag |
| `HOST` | String | `0.0.0.0` | Server host |
| `PORT` | Integer | `7860` | Server port |

### Python Runtime

| Variable | Type | Recommended | Description |
|----------|------|-------------|-------------|
| `PYTHONUNBUFFERED` | String | `1` | Prevents output buffering |
| `PYTHONDONTWRITEBYTECODE` | String | `1` | Prevents .pyc files |
| `FLASK_ENV` | String | `development` | Flask environment |

### Security & Authentication

| Variable | Type | Description |
|----------|------|-------------|
| `JWT_SECRET_KEY` | String | JWT authentication secret |
| `ENCRYPTION_KEY` | String | Data encryption key |
| `SESSION_SECRET` | String | Session management secret |
| `API_KEY_REQUIRED` | Boolean | Require API key for authentication |
| `API_KEY_HEADER` | String | HTTP header for API key |

### Content Processing

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `BOOK_CONTENT_DIR` | String | `book_content` | Directory for book content |
| `SUPPORTED_FILE_TYPES` | JSON | `[".md", ".txt", ".pdf", ".docx"]` | Supported file types |
| `MAX_FILE_SIZE_MB` | Integer | `50` | Maximum file size in MB |

### CORS Configuration

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `CORS_ORIGINS` | JSON | `["https://humanoid-robotic-book.vercel.app", "http://localhost:3000", "http://localhost:7860"]` | Allowed origins |

## RAG Configuration

### Text Chunking Settings

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `MIN_CHUNK_SIZE` | Integer | `50` | Minimum chunk size in characters |
| `MAX_CHUNK_SIZE` | Integer | `1000` | Maximum chunk size in characters |
| `CHUNK_OVERLAP` | Integer | `100` | Overlap between chunks |
| `TOP_K_RESULTS` | Integer | `10` | Number of retrieved documents |
| `SIMILARITY_THRESHOLD` | Float | `0.7` | Minimum similarity score |
| `EMBEDDING_BATCH_SIZE` | Integer | `100` | Documents per embedding batch |

### Model Configuration

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `EMBEDDING_MODEL` | String | `embed-english-v3.0` | Cohere embedding model |
| `RERANK_MODEL` | String | `rerank-english-v3.0` | Cohere reranking model |
| `USE_RERANK` | Boolean | `true` | Enable reranking for better results |

## Performance & Monitoring

### Logging

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `LOG_LEVEL` | String | `INFO` | Logging level (DEBUG, INFO, WARNING, ERROR) |
| `LOG_FORMAT` | String | `json` | Log format (json, text) |

### Rate Limiting

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `RATE_LIMIT_REQUESTS` | Integer | `100` | Requests per window |
| `RATE_LIMIT_WINDOW` | Integer | `60` | Time window in seconds |

### Caching

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `ENABLE_EMBEDDING_CACHE` | Boolean | `true` | Enable embedding cache |
| `CACHE_MAX_SIZE` | Integer | `10000` | Maximum cache size |

### Health Check

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `HEALTH_CHECK_INTERVAL` | Integer | `30` | Health check interval in seconds |

## Deployment Variables

### Railway Deployment

| Variable | Type | Required | Description |
|----------|------|----------|-------------|
| `ROOT_DIRECTORY` | String | **Yes** | Forces Railway to use backend directory |
| `PORT` | Integer | `7860` | Railway deployment port |

### Railway Environment Setup

#### Method 1: Railway Dashboard
1. Go to Project → Settings → Variables
2. Add each variable from `.env.example`
3. Deploy

#### Method 2: Railway CLI
```bash
railway variables set ROOT_DIRECTORY=backend
railway variables set PORT=7860
railway variables set COHERE_API_KEY=your_key_here
railway variables set CHAT_API_KEY=your_chat_key
```

## Security Variables

| Variable | Type | Description |
|----------|------|-------------|
| `JWT_SECRET_KEY` | String | JWT authentication secret |
| `ENCRYPTION_KEY` | String | Data encryption key |
| `SESSION_SECRET` | String | Session management secret |

## Optional Services

### Database

| Variable | Type | Description |
|----------|------|-------------|
| `DATABASE_URL` | String | PostgreSQL connection string |

### Caching

| Variable | Type | Description |
|----------|------|-------------|
| `REDIS_URL` | String | Redis connection string |

### Analytics

| Variable | Type | Description |
|----------|------|-------------|
| `GOOGLE_ANALYTICS_ID` | String | Google Analytics measurement ID |

### Error Tracking

| Variable | Type | Description |
|----------|------|-------------|
| `SENTRY_DSN` | String | Sentry error tracking DSN |

## Development & Testing

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `TESTING` | Boolean | `false` | Enable testing mode |
| `USE_MOCK_RESPONSES` | Boolean | `false` | Use mock responses for testing |

## Environment Profiles

### Development Profile
```bash
PROFILE=development
DEBUG=true
LOG_LEVEL=DEBUG
```

### Production Profile
```bash
PROFILE=production
DEBUG=false
LOG_LEVEL=WARNING
NODE_ENV=production
```

### Testing Profile
```bash
PROFILE=testing
DEBUG=true
TESTING=true
```

## API Key Setup Guide

### Cohere API

1. **Sign up**: Go to [cohere.com](https://cohere.com)
2. **Dashboard**: Navigate to API Keys
3. **Create Key**: Generate a new API key
4. **Copy**: Add to `COHERE_API_KEY` in your `.env.local`

### Qdrant Setup Options

#### Option 1: Local Development
```bash
# Install Qdrant locally
docker run -p 6333:6333 qdrant/qdrant

# Environment variables
QDRANT_HOST=localhost
QDRANT_PORT=6333
```

#### Option 2: Railway Hosted
1. Deploy Qdrant as separate Railway project
2. Get Railway URL: `your-qdrant.up.railway.app`
3. Set `QDRANT_HOST=your-qdrant.up.railway.app`

#### Option 3: Qdrant Cloud
1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Create cluster
3. Get cluster URL and API key
4. Set both variables in environment

## Troubleshooting

### Common Issues

#### 1. "Connection Refused" Error
- Check if backend is running on correct port
- Verify `CHAT_API_ENDPOINT` in frontend
- Ensure CORS origins include your frontend URL

#### 2. "Invalid API Key" Error
- Verify `COHERE_API_KEY` is correct
- Check if API key has proper permissions
- Ensure key is not expired

#### 3. Railway Deployment Issues
- Ensure `ROOT_DIRECTORY=backend` is set
- Check that all required environment variables are present
- Verify Railway build logs

### Debug Commands

```bash
# Check environment variables (Unix/Linux)
printenv | grep -E "(CHAT|COHERE|QDRANT)"

# Check environment variables (Windows)
echo %CHAT_API_ENDPOINT%
echo %COHERE_API_KEY%

# Test backend health
curl http://localhost:7860/api/v1/health

# Test with environment variables
CHAT_API_ENDPOINT=http://localhost:7860/api/v1 \
CHAT_API_KEY=demo-key \
python -c "
import os
print('API Endpoint:', os.getenv('CHAT_API_ENDPOINT'))
print('API Key:', os.getenv('CHAT_API_KEY'))
"
```

## Best Practices

1. **Never commit secrets**: Always use `.env.local` for actual values
2. **Use environment-specific profiles**: Different configs for dev/staging/prod
3. **Regular key rotation**: Update API keys periodically
4. **Monitor usage**: Track API usage to control costs
5. **Documentation**: Keep this file updated with new variables

## Support

For issues with environment variables:
1. Check this documentation first
2. Verify variable names and formats
3. Test with minimal configuration first
4. Check logs for specific error messages