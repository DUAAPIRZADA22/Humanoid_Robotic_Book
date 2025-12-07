#!/bin/bash
set -e

# Print environment info for debugging
echo "=== Physical AI & Humanoid Robotics Book Chatbot ==="
echo "Starting at $(date)"
echo "User: $(whoami)"
echo "Python version: $(python --version)"
echo "Working directory: $(pwd)"

# Ensure book_content directory exists
mkdir -p /app/book_content
mkdir -p /app/data

# Print environment variables (without sensitive data)
echo ""
echo "Configuration:"
echo "  Host: ${HOST:-0.0.0.0}"
echo "  Port: ${PORT:-7860}"
echo "  Qdrant Host: ${QDRANT_HOST:-localhost}"
echo "  Qdrant Port: ${QDRANT_PORT:-6333}"
echo "  Embedding Model: ${EMBEDDING_MODEL:-embed-english-v3.0}"
echo "  Rerank Model: ${RERANK_MODEL:-rerank-english-v3.0}"
echo ""

# Check if required environment variables are set
if [ -z "$COHERE_API_KEY" ]; then
    echo "WARNING: COHERE_API_KEY is not set. The application will not work without it."
    echo "Please set this environment variable in your Hugging Face Space settings."
fi

# Try to connect to Qdrant if it's external
if [ "$QDRANT_HOST" != "localhost" ] && [ -n "$QDRANT_HOST" ]; then
    echo "Checking Qdrant connection at ${QDRANT_HOST}:${QDRANT_PORT:-6333}..."
    if timeout 5 bash -c "</dev/tcp/${QDRANT_HOST}/${QDRANT_PORT:-6333}"; then
        echo "✅ Qdrant is reachable"
    else
        echo "⚠️  Could not connect to Qdrant. Make sure it's running and accessible."
    fi
fi

echo ""
echo "Starting FastAPI application..."
echo ""

# Run the application with uvicorn
exec python -m uvicorn main:app \
    --host ${HOST:-0.0.0.0} \
    --port ${PORT:-7860} \
    --workers ${UVICORN_WORKERS:-1} \
    --access-log