"""
Quick test of the search functionality
"""

import os
import asyncio
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rag.embeddings import CohereEmbeddingService
from rag.vector_store import QdrantVectorStore
from dotenv import load_dotenv

load_dotenv()

async def test_search():
    # Initialize services
    cohere_api_key = os.getenv('COHERE_API_KEY')
    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')

    embedding_service = CohereEmbeddingService(api_key=cohere_api_key)
    vector_store = QdrantVectorStore(
        collection_name="humanoid_robotics_book",
        embedding_dim=1024,
        url=qdrant_url,
        api_key=qdrant_api_key
    )

    # Test query
    test_query = "What is Physical AI?"
    query_embedding = await embedding_service.get_embeddings([test_query])

    # Search
    search_results = await vector_store.search(
        query_vector=query_embedding[0],
        top_k=5,
        score_threshold=0.3
    )

    print(f"Found {len(search_results)} results:")
    for i, result in enumerate(search_results):
        print(f"\n{i+1}. Score: {result['score']:.4f}")
        print(f"   Source: {result['metadata'].get('source', 'Unknown')}")
        print(f"   Preview: {result['text'][:150]}...")

if __name__ == "__main__":
    asyncio.run(test_search())