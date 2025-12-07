# Physical AI & Humanoid Robotics Book Chatbot Backend

A robust RAG (Retrieval-Augmented Generation) backend for serving answers about the "Physical AI & Humanoid Robotics" book. This backend provides streaming chat responses and document ingestion capabilities optimized for Hugging Face Spaces deployment.

## Features

- **Streaming Chat**: Server-Sent Events (SSE) for real-time response streaming
- **Advanced RAG Pipeline**: Custom chunking, Cohere embeddings, and Qdrant vector storage
- **Deduplication**: Intelligent duplicate filtering during retrieval
- **Reranking**: Cohere reranking for improved relevance
- **Batch Processing**: Efficient batching for embedding generation
- **Error Handling**: Graceful degradation and retry logic
- **Health Monitoring**: Comprehensive health checks and statistics
- **Hugging Face Spaces Ready**: Optimized Docker configuration

## Architecture

```
backend/
├── main.py              # FastAPI application with SSE endpoints
├── rag/                 # RAG pipeline components
│   ├── chunking.py      # Markdown document chunking
│   ├── embeddings.py    # Cohere embedding service
│   ├── vector_store.py  # Qdrant vector store client
│   ├── retrieval.py     # Retrieval pipeline orchestration
│   └── rerank.py        # Cohere reranking service
├── scripts/             # Utility scripts
│   ├── setup_environment.py  # Environment setup
│   └── ingest_content.py     # Content ingestion
├── Dockerfile           # Hugging Face Spaces deployment
├── requirements.txt     # Python dependencies
└── .env.example        # Environment variables template
```

## Quick Start

### 1. Prerequisites

- Python 3.9+
- Qdrant vector database
- Cohere API key

### 2. Setup

```bash
# Clone the repository
git clone <repository-url>
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Copy environment template
cp .env.example .env

# Edit .env with your API keys
nano .env  # Add your COHERE_API_KEY
```

### 3. Initialize Environment

```bash
# Run the setup script
python scripts/setup_environment.py
```

This will:
- Initialize the Qdrant collection
- Perform health checks on all services
- Create sample content for testing

### 4. Add Book Content

Place your markdown files in the `book_content/` directory:

```bash
# Create content directory
mkdir -p book_content

# Add your markdown files
cp /path/to/book/content/*.md book_content/
```

### 5. Ingest Content

```bash
# Ingest all markdown files
python scripts/ingest_content.py

# Or with options
python scripts/ingest_content.py --content-dir book_content --overwrite
```

### 6. Run the Application

```bash
# Development mode
python -m uvicorn main:app --reload

# Production mode
python -m uvicorn main:app --host 0.0.0.0 --port 7860
```

## API Endpoints

### POST `/chat`

Stream chat responses using Server-Sent Events.

**Request:**
```json
{
  "query": "What are the key components of humanoid robots?",
  "top_k": 5,
  "similarity_threshold": 0.7,
  "use_rerank": true
}
```

**Response (SSE):**
- `metadata`: Query information and settings
- `content`: Relevant content chunks
- `complete`: Final signal with statistics

### POST `/ingest`

Trigger content ingestion from markdown files.

**Request:**
```json
{
  "content_dir": "book_content",
  "file_pattern": "*.md",
  "overwrite": false
}
```

### GET `/health`

Check the health of all components.

**Response:**
```json
{
  "status": "healthy",
  "components": {
    "chunker": true,
    "embedding_service": true,
    "vector_store": true,
    "retrieval_pipeline": true,
    "reranker": true
  }
}
```

## Configuration

### Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `COHERE_API_KEY` | Cohere API key for embeddings and reranking | Required |
| `QDRANT_HOST` | Qdrant host address | localhost |
| `QDRANT_PORT` | Qdrant port number | 6333 |
| `QDRANT_API_KEY` | Qdrant API key (if using cloud) | None |
| `EMBEDDING_MODEL` | Cohere embedding model | embed-english-v3.0 |
| `RERANK_MODEL` | Cohere reranking model | rerank-english-v3.0 |
| `MIN_CHUNK_SIZE` | Minimum chunk size in characters | 50 |
| `MAX_CHUNK_SIZE` | Maximum chunk size in characters | 1000 |
| `TOP_K_RESULTS` | Default number of search results | 10 |
| `SIMILARITY_THRESHOLD` | Minimum similarity score | 0.7 |

## RAG Pipeline Details

### Chunking Strategy

- **Semantic Sectioning**: Preserves document structure and headers
- **Code Block Handling**: Keeps code blocks intact
- **Size Control**: Flexible min/max chunk sizes
- **Overlap**: Configurable overlap for context retention
- **Deduplication**: Hash-based duplicate removal

### Embedding Process

- **Batching**: Processes up to 100 texts per request
- **Rate Limiting**: Built-in delays to avoid API limits
- **Retry Logic**: Exponential backoff for failed requests
- **Caching**: Optional in-memory cache for repeated queries
- **Error Handling**: Graceful degradation on failures

### Retrieval Optimization

- **Multi-stage**: Fetches 2x results before reranking
- **Hybrid Search**: Combines vector similarity with keyword matching
- **Metadata Filtering**: Supports flexible metadata-based filtering
- **Context Building**: Optimizes context for LLM input limits

## Deployment

### Hugging Face Spaces

1. Create a new Space on Hugging Face
2. Choose Docker template
3. Push the code:
   ```bash
   git add .
   git commit -m "Initial backend deployment"
   git push origin main
   ```

The Dockerfile is optimized for Hugging Face Spaces with:
- Port 7860 configuration
- Non-root user support
- Health checks
- Efficient layer caching

### Docker Local Deployment

```bash
# Build image
docker build -t humanoid-robotics-chatbot .

# Run container
docker run -p 7860:7860 \
  -e COHERE_API_KEY=your_key \
  -e QDRANT_HOST=host.docker.internal \
  humanoid-robotics-chatbot
```

## Monitoring

### Health Checks

The application provides comprehensive health monitoring:

```bash
# Check health
curl http://localhost:7860/health

# View detailed stats
curl http://localhost:7860/stats
```

### Logging

Structured logging with configurable levels:
- JSON format for production
- Include timestamps, levels, and trace information
- Separate loggers for each component

## Performance Tips

1. **Batch Processing**: Use the provided ingestion script for large content
2. **Caching**: Enable embedding cache for repeated queries
3. **Vector Store**: Use persistent storage for Qdrant
4. **Rate Limiting**: Monitor API usage and adjust batch sizes
5. **Memory Management**: Monitor memory usage with large document sets

## Troubleshooting

### Common Issues

1. **Connection Refused**: Ensure Qdrant is running and accessible
2. **API Key Errors**: Verify COHERE_API_KEY is set correctly
3. **No Results**: Check similarity threshold and content ingestion
4. **Slow Responses**: Consider reducing batch size or enabling cache

### Debug Mode

Enable debug logging:
```bash
export LOG_LEVEL=DEBUG
python -m uvicorn main:app --reload
```

## Development

### Running Tests

```bash
# Install test dependencies
pip install pytest pytest-asyncio

# Run tests
pytest tests/
```

### Code Style

The project follows PEP 8 with:
- Black for formatting
- isort for imports
- mypy for type checking

## License

This project is part of the Physical AI & Humanoid Robotics book educational materials.