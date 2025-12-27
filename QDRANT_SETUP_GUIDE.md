# Qdrant Setup and Usage Guide

This guide helps you set up and use Qdrant vector database for your Physical AI & Humanoid Robotics Book project.

## Quick Start

### 1. Install Dependencies
```bash
cd backend
pip install qdrant-client cohere
```

### 2. Run Qdrant Server

#### Option A: Local Docker (for development)
```bash
# Quick start
docker run -p 6333:6333 qdrant/qdrant

# Persistent storage (recommended for development)
docker run -d -p 6333:6333 -v $(pwd)/qdrant_storage:/qdrant/storage qdrant/qdrant
```

#### Option B: Qdrant Cloud (recommended for production)
1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Create a free cluster
3. Get cluster URL and API key
4. Update your `.env` file:
```bash
QDRANT_URL=https://your-cluster-name.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_cloud_api_key_here
```
For detailed setup, see [QDRANT_CLOUD_SETUP.md](./QDRANT_CLOUD_SETUP.md)

### 3. Configure Environment Variables
Add to your `.env` file:

**For Local Development:**
```bash
# Vector Database Configuration
QDRANT_HOST=localhost
QDRANT_PORT=6333
```

**For Qdrant Cloud (Production):**
```bash
# Vector Database Configuration
QDRANT_URL=https://your-cluster-name.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_cloud_api_key_here
```

**Additional Settings (optional):**
```bash
QDRANT_COLLECTION_NAME=humanoid_robotics_book
QDRANT_VECTOR_SIZE=1024
QDRANT_DISTANCE_METRIC=cosine
QDRANT_ON_DISK=true
QDRANT_TIMEOUT=30
```

### 4. Test the Setup
```bash
cd backend
python test_qdrant.py
```

## Using Qdrant in Your Project

### Basic Usage

```python
from rag.vector_store import QdrantVectorStore
import asyncio

async def use_qdrant():
    # Initialize vector store
    store = QdrantVectorStore(
        collection_name="robotics_docs",
        embedding_dim=1024,  # Cohere embedding dimension
        host="localhost",
        port=6333
    )

    # Ensure collection exists
    await store.ensure_collection()

    # Add documents
    documents = [
        {
            "id": "doc1",
            "text": "Humanoid robots use advanced actuators for movement",
            "vector": [0.1, 0.2, ...],  # Your embedding vector
            "metadata": {"category": "actuators", "topic": "movement"}
        }
    ]

    await store.upsert(documents)

    # Search
    query_vector = [0.15, 0.25, ...]  # Your query embedding
    results = await store.search(
        query_vector=query_vector,
        top_k=5,
        score_threshold=0.7
    )

    return results

# Run the async function
results = asyncio.run(use_qdrant())
```

### With Cohere Embeddings

```python
from rag.vector_store import QdrantVectorStore
from rag.embeddings import CohereEmbeddingService
import asyncio

async def search_with_cohere():
    # Initialize services
    embedding_service = CohereEmbeddingService(
        api_key="your_cohere_api_key",
        model="embed-english-v3.0"
    )

    vector_store = QdrantVectorStore(
        collection_name="robotics_book",
        embedding_dim=1024,
        host="localhost",
        port=6333
    )

    await vector_store.ensure_collection()

    # Index your book content
    documents = [
        "Chapter 1: Introduction to Humanoid Robotics...",
        "Chapter 2: Sensors and Perception Systems...",
        # More documents...
    ]

    # Generate embeddings
    embeddings = await embedding_service.get_embeddings(
        documents,
        input_type="search_document"
    )

    # Store in Qdrant
    qdrant_docs = []
    for i, (doc, embedding) in enumerate(zip(documents, embeddings)):
        qdrant_docs.append({
            "id": f"chapter_{i}",
            "text": doc,
            "vector": embedding,
            "metadata": {"chapter": i+1, "source": "book"}
        })

    await vector_store.upsert(qdrant_docs)

    # Semantic search
    query = "How do humanoid robots maintain balance?"
    query_embedding = await embedding_service.get_embedding(
        query,
        input_type="search_query"
    )

    results = await vector_store.search(
        query_vector=query_embedding,
        top_k=3
    )

    return results
```

## API Reference

### QdrantVectorStore Methods

#### `__init__(collection_name, embedding_dim, host, port, api_key, url)`
Initialize the vector store connection.

#### `async ensure_collection()`
Create the collection if it doesn't exist with proper configuration.

#### `async upsert(documents)`
Insert or update documents in the collection.
- `documents`: List of dicts with `id`, `text`, `vector`, and `metadata`

#### `async search(query_vector, top_k, score_threshold, filter)`
Search for similar documents.
- `query_vector`: Query embedding vector
- `top_k`: Number of results to return
- `score_threshold`: Minimum similarity score
- `filter`: Optional metadata filter

#### `async hybrid_search(query_vector, text_query, top_k, vector_weight, text_weight)`
Perform hybrid search combining vector and text search.

#### `async get_document(doc_id)`
Retrieve a specific document by ID.

#### `async delete(doc_ids)`
Delete documents by IDs.

#### `async count_documents()`
Count total documents in collection.

#### `async health_check()`
Check the health of the vector store.

## Common Use Cases

### 1. Document Search
```python
# Search for relevant content
results = await vector_store.search(
    query_vector=embedding,
    top_k=5,
    score_threshold=0.5
)

for result in results:
    print(f"Score: {result['score']:.4f}")
    print(f"Content: {result['text']}")
    print(f"Source: {result['metadata']}")
```

### 2. Filtered Search
```python
from qdrant_client.models import Filter, FieldCondition, MatchValue

# Search only in specific chapters
filter = Filter(
    must=[
        FieldCondition(
            key="metadata.chapter",
            match=MatchValue(value=3)
        )
    ]
)

results = await vector_store.search(
    query_vector=embedding,
    filter=filter,
    top_k=10
)
```

### 3. Hybrid Search
```python
# Combine semantic and keyword search
results = await vector_store.hybrid_search(
    query_vector=embedding,
    text_query="actuators motors movement",
    top_k=10,
    vector_weight=0.7,
    text_weight=0.3
)
```

## Performance Tips

1. **Batch Operations**: Upsert documents in batches (100-1000 at a time)
2. **Embedding Cache**: Enable caching to avoid recomputing embeddings
3. **On-disk Storage**: Use `on_disk=True` for large collections
4. **Quantization**: Consider vector quantization for very large datasets

## Troubleshooting

### Connection Refused
- Ensure Qdrant is running: `docker run -p 6333:6333 qdrant/qdrant`
- Check if port 6333 is available
- Verify host and port in `.env`

### Dimension Mismatch
- Ensure embedding dimension matches collection configuration
- Cohere `embed-english-v3.0` produces 1024-dimensional vectors
- Check `model_dimensions` in `CohereEmbeddingService`

### Memory Issues
- Use on-disk storage for large collections
- Implement pagination for search results
- Consider using Qdrant Cloud for production

## Production Deployment

### Railway Setup
1. Deploy Qdrant as separate Railway service
2. Get Railway URL: `your-qdrant.up.railway.app`
3. Update environment variables:
```bash
QDRANT_HOST=your-qdrant.up.railway.app
QDRANT_API_KEY=your_railway_qdrant_key
```

### Monitoring
```python
# Check collection health
health = await vector_store.health_check()
print(f"Status: {health['status']}")
print(f"Documents: {health['document_count']}")

# Get collection stats
info = await vector_store.get_collection_info()
print(f"Vector count: {info.points_count}")
```

## Integration with Chatbot

The Qdrant vector store integrates with your chatbot backend to provide:

1. **Context Retrieval**: Find relevant book content for user queries
2. **Semantic Search**: Understand meaning beyond keywords
3. **Fast Retrieval**: Millisecond response times for large document sets
4. **Scalability**: Handle growing content as your book expands

## Next Steps

1. ✅ Install dependencies
2. ✅ Set up Qdrant server
3. ✅ Configure environment variables
4. ✅ Test connection
5. 🔄 Index your book content
6. 🔄 Implement search in chatbot
7. 🔄 Add filtering by chapters/topics
8. 🔄 Optimize for performance

For more information, see:
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Cohere API Docs](https://docs.cohere.com/docs)
- [Your RAG Implementation](backend/rag/)