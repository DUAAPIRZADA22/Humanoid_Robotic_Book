"""
Test script for updated RAG implementations using latest Cohere and Qdrant APIs.
"""

import os
import asyncio
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from dotenv import load_dotenv

# Import updated implementations
from rag.embeddings_new import CohereEmbeddingService
from rag.vector_store_new import QdrantVectorStore
from rag.rerank_new import CohereReranker

load_dotenv()

async def test_updated_implementations():
    """Test all updated RAG components"""
    print("Testing Updated RAG Implementations")
    print("=" * 50)

    # Get environment variables
    cohere_api_key = os.getenv('COHERE_API_KEY')
    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')

    if not all([cohere_api_key, qdrant_url, qdrant_api_key]):
        print("Missing required environment variables!")
        print(f"COHERE_API_KEY: {'OK' if cohere_api_key else 'MISSING'}")
        print(f"QDRANT_URL: {'OK' if qdrant_url else 'MISSING'}")
        print(f"QDRANT_API_KEY: {'OK' if qdrant_api_key else 'MISSING'}")
        return

    # Test 1: Cohere Embeddings
    print("\n1. Testing Cohere Embeddings...")
    try:
        embedder = CohereEmbeddingService(
            api_key=cohere_api_key,
            model="embed-english-v3.0"
        )

        test_texts = ["What is Physical AI?", "How do robots work?"]
        embeddings = await embedder.get_embeddings(test_texts, "search_query")

        print(f"Generated {len(embeddings)} embeddings")
        print(f"   Embedding dimension: {len(embeddings[0]) if embeddings else 'N/A'}")

        # Test health check
        health = await embedder.health_check()
        print(f"   Health status: {health['status']}")

    except Exception as e:
        print(f"Embeddings test failed: {e}")

    # Test 2: Qdrant Vector Store
    print("\n2. Testing Qdrant Vector Store...")
    try:
        vector_store = QdrantVectorStore(
            collection_name="test_collection",
            embedding_dim=1024,
            url=qdrant_url,
            api_key=qdrant_api_key
        )

        # Ensure collection exists
        collection_ready = await vector_store.ensure_collection()
        print(f"Collection ready: {collection_ready}")

        # Test search with a sample vector
        sample_vector = [0.1] * 1024
        search_results = await vector_store.search(
            query_vector=sample_vector,
            top_k=3,
            score_threshold=0.1
        )

        print(f"Search completed, found {len(search_results)} results")

        # Test health check
        health = await vector_store.health_check()
        print(f"   Health status: {health['status']}")
        print(f"   Document count: {health.get('document_count', 'N/A')}")

    except Exception as e:
        print(f"Vector store test failed: {e}")

    # Test 3: Cohere Reranker
    print("\n3. Testing Cohere Reranker...")
    try:
        reranker = CohereReranker(
            api_key=cohere_api_key,
            model="rerank-v3.5"
        )

        test_docs = [
            "Physical AI is about artificial intelligence in physical systems.",
            "Machine learning is a subset of artificial intelligence.",
            "Cooking recipes involve ingredients and instructions."
        ]

        query = "What is artificial intelligence?"
        reranked = await reranker.rerank(query, test_docs, top_n=2)

        print(f"Reranked {len(test_docs)} documents")
        print(f"   Top result: {reranked[0]['text'][:50]}...")
        print(f"   Relevance score: {reranked[0].get('relevance_score', 'N/A')}")

        # Test health check
        health = await reranker.health_check()
        print(f"   Health status: {health['status']}")

    except Exception as e:
        print(f"Reranker test failed: {e}")

    print("\nUpdated RAG implementation tests completed!")

if __name__ == "__main__":
    asyncio.run(test_updated_implementations())