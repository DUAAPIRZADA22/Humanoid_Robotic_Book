"""Test embedding generation and search."""
import os
import asyncio
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from rag.openrouter_embeddings import OpenRouterEmbeddingService

load_dotenv()

async def test_search():
    # Initialize services
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    qdrant_url = os.getenv("QDRANT_URL")
    openrouter_api_key = os.getenv("OPENROUTER_API_KEY")

    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    collection_name = "humanoid_robotics_book_openrouter"

    embedding_service = OpenRouterEmbeddingService(
        api_key=openrouter_api_key,
        model="openai/text-embedding-3-small",
        base_url=os.getenv("OPENROUTER_BASE_URL")
    )

    # Generate query embedding
    query = "development setup"
    print(f"Generating embedding for query: '{query}'")
    query_vector = await embedding_service.embed_single_text(query)
    print(f"Embedding dimension: {len(query_vector)}")

    # Search with the generated embedding
    print(f"\nSearching Qdrant with generated embedding...")
    results = client.query_points(
        collection_name=collection_name,
        query=query_vector,
        limit=5,
        with_payload=True,
        score_threshold=0.0
    )

    print(f"Search returned {len(results.points)} results")
    for i, point in enumerate(results.points[:3]):
        print(f"\nResult {i+1}:")
        print(f"  Score: {point.score}")
        print(f"  Text preview: {point.payload.get('text', '')[:100]}...")

asyncio.run(test_search())
