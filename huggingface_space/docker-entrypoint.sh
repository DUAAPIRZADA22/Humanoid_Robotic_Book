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
echo "  Port: ${PORT:-8000}"
echo "  Qdrant Host: ${QDRANT_HOST:-localhost}"
echo "  Qdrant Port: ${QDRANT_PORT:-6333}"
echo ""

# Check if required environment variables are set
if [ -z "$OPENROUTER_API_KEY" ]; then
    echo "WARNING: OPENROUTER_API_KEY is not set. The application will not work without it."
    echo "Please set this environment variable in your Hugging Face Space settings."
fi

# Try to connect to Qdrant if it's external
if [ -n "$QDRANT_URL" ]; then
    echo "Checking Qdrant connection at ${QDRANT_URL}..."
    if timeout 5 curl -sf "${QDRANT_URL}" > /dev/null; then
        echo "✅ Qdrant is reachable"
    else
        echo "⚠️  Could not connect to Qdrant. Make sure it's running and accessible."
    fi
fi

echo ""
echo "Starting FastAPI application..."
echo ""

# Run the application with uvicorn (port 8000 for HuggingFace Spaces)
exec python -m uvicorn main:app \
    --host ${HOST:-0.0.0.0} \
    --port ${PORT:-8000} \
    --workers 1 \
    --access-log