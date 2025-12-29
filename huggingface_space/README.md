---
title: Hackathon Book Backend
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
license: mit
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

Set these in the Space settings (Settings → Variables):

- `OPENROUTER_API_KEY` - Your OpenRouter API key (required)
- `QDRANT_URL` - Qdrant cloud URL (or use QDRANT_HOST)
- `QDRANT_API_KEY` - Qdrant API key (if using cloud)
- `QDRANT_HOST` - Qdrant host (alternative to URL)
- `QDRANT_PORT` - Qdrant port (default: 6333)
- `CORS_ORIGINS` - JSON array of allowed CORS origins (e.g., `["https://humanoid-robotic-book-livid.vercel.app"]`)

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
