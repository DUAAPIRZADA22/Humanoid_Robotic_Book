---
title: Hackathon Book Backend
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
license: mit
port: 8000
---

# Hackathon Book Chatbot Backend

FastAPI backend for the Physical AI & Humanoid Robotics book chatbot with RAG pipeline.

## Features

- 🤖 RAG-powered chatbot using Qdrant vector store
- 📚 Markdown document ingestion
- 🌐 Multi-language translation support
- ⚡ Fast streaming responses with SSE
- 🔧 Built with FastAPI and Python

## Environment Variables

### REQUIRED for Full Chat Functionality

Set these in your Space Settings → Variables and secrets:

| Variable | Description | Where to Get |
|----------|-------------|--------------|
| `OPENROUTER_API_KEY` | OpenRouter API key for embeddings and LLM | https://openrouter.ai/keys |
| `QDRANT_URL` | Qdrant cloud URL | Your Qdrant dashboard |
| `QDRANT_API_KEY` | Qdrant API key | Your Qdrant dashboard |
| `CORS_ORIGINS` | JSON array of allowed origins | `["https://your-frontend.vercel.app"]` |

### Setup Steps

1. **Get OpenRouter API Key:**
   - Go to https://openrouter.ai/keys
   - Sign up/login and create an API key
   - Copy the key (starts with `sk-or-v1-`)

2. **Get Qdrant Credentials:**
   - Go to https://cloud.qdrant.io/
   - Create a free cluster
   - Copy the API key and URL from your cluster dashboard

3. **Configure Space Variables:**
   - Open your Space Settings → Variables and secrets
   - Add each variable from the table above with your actual values
   - Click "Restart" to apply changes

### Verification

After setting variables, check if they're working:
- Go to https://huggingface.co/spaces/YOUR_USERNAME/hackathon-book/logs
- Look for: `INFO - RAG pipeline initialized successfully`
- NOT: `WARNING - OPENROUTER_API_KEY not set`

### Local Testing

For local development, copy `.env.template` to `.env` and fill in your keys:
```bash
cp .env.template .env
# Edit .env with your actual API keys
python verify_env.py  # Verify configuration
```

## API Endpoints

- `POST /chat` - Chat with the book (streaming)
- `POST /ingest` - Ingest markdown documents
- `GET /health` - Health check
- `POST /api/translate` - Translate text

## Port

This Space runs on **port 8000**.

## Deployment

This Space uses Docker SDK. Click "Settings" to configure your environment variables.

**API Endpoint**: https://DuaaPirzada-hackathon-book.hf.space
