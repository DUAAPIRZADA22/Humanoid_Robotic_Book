# Environment Variables Setup Guide

This document provides a quick reference for setting up environment variables for the Physical AI & Humanoid Robotics Book project.

## Required API Keys

### 1. Cohere AI (Required for RAG functionality)
```bash
# Get from: https://cohere.com/
COHERE_API_KEY=your_cohere_api_key_here
```

### 2. Qdrant Vector Database
```bash
# Option 1: Local Development (Docker required)
QDRANT_HOST=localhost
QDRANT_PORT=6333

# Option 2: Qdrant Cloud
QDRANT_HOST=your-cluster.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_cloud_api_key
```

## Quick Setup Steps

### 1. Copy Environment Files
```bash
# Frontend
cp .env.frontend.example .env.frontend.local

# Backend (if running separately)
cp backend/.env.example backend/.env.local
```

### 2. Fill in Required Values
Edit `.env` or `.env.local` and update:
- `COHERE_API_KEY` - Your Cohere API key
- `QDRANT_HOST` - Qdrant server host
- `QDRANT_API_KEY` - If using Qdrant Cloud
- `CHAT_API_KEY` - For frontend-backend communication

### 3. Local Development Setup
```bash
# Start Qdrant locally (if not using cloud)
docker run -p 6333:6333 qdrant/qdrant

# Install backend dependencies
cd backend
pip install -r requirements.txt

# Run the backend
python main.py
```

### 4. Frontend Setup
```bash
# Install frontend dependencies
npm install

# Run development server
npm start
```

## Environment File Structure

### Root `.env` (Combined Configuration)
Contains both frontend and backend variables for convenience.

### `.env.frontend.example`
Frontend-specific variables for the Docusaurus site and chat widget.

### `backend/.env.example`
Backend-specific variables for the Flask/FastAPI server.

## Key Configuration Categories

### AI Services
- `COHERE_API_KEY` - Embeddings and reranking
- `EMBEDDING_MODEL` - embed-english-v3.0
- `RERANK_MODEL` - rerank-english-v3.0

### Vector Database
- `QDRANT_HOST` - Database host
- `QDRANT_PORT` - Database port
- `QDRANT_API_KEY` - Cloud authentication

### Chat Widget
- `CHAT_API_ENDPOINT` - Backend API URL
- `CHAT_API_KEY` - Authentication key
- `CHAT_WIDGET_POSITION` - UI position

### Railway Deployment
- `ROOT_DIRECTORY=backend` - Critical for Railway
- `PORT=7860` - Railway port
- `RAILWAY_ENVIRONMENT=production`

## Security Notes

1. **Never commit actual API keys** to version control
2. Use `.env.local` for actual values
3. Keep `.env.example` with placeholder values only
4. Rotate API keys regularly

## Testing Without API Keys

Set `USE_MOCK_RESPONSES=true` to test the application without actual API keys.

## Common Issues

1. **Connection Refused**: Ensure backend is running on correct port
2. **Invalid API Key**: Verify Cohere key is correct and active
3. **CORS Errors**: Check `CORS_ORIGINS` includes your frontend URL
4. **Railway Deployment**: Must set `ROOT_DIRECTORY=backend`

## Production Checklist

- [ ] Update all `your_*_here` placeholders
- [ ] Set `NODE_ENV=production`
- [ ] Set `DEBUG=false`
- [ ] Configure CORS origins for production domain
- [ ] Set up monitoring/logging
- [ ] Test all API integrations