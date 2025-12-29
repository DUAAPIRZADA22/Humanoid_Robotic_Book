#!/usr/bin/env python3
"""Script to check vector store collection status"""

import asyncio
import os
from dotenv import load_dotenv

from rag.vector_store_new import QdrantVectorStore

load_dotenv()

async def check_collection():
    """Check the number of documents in the collection"""
    try:
        # Initialize vector store
        vector_store = QdrantVectorStore(
            collection_name='humanoid_robotics_book',
            embedding_dim=1024,
            url=os.getenv('QDRANT_URL'),
            port=int(os.getenv('QDRANT_PORT', '6333')),
            api_key=os.getenv('QDRANT_API_KEY')
        )

        # Get collection info
        count_result = vector_store.client.count(collection_name='humanoid_robotics_book')
        print(f"Document count in collection: {count_result.count}")

        # Get collection info to verify dimension
        collection_info = vector_store.client.get_collection(collection_name='humanoid_robotics_book')
        print(f"Vector dimension: {collection_info.config.params.vectors.size}")
        print(f"Distance metric: {collection_info.config.params.vectors.distance}")

        # Test a simple search
        from rag.embeddings_new import CohereEmbeddingService

        embedding_service = CohereEmbeddingService(
            api_key=os.getenv('COHERE_API_KEY'),
            model='embed-english-v3.0'
        )

        # Create a test embedding
        test_query = "humanoid robotics"
        embeddings = await embedding_service.get_embeddings([test_query])
        embedding = embeddings[0]
        print(f"Test embedding dimension: {len(embedding)}")

        # Search for results
        results = await vector_store.search(
            query_vector=embedding,
            top_k=5,
            score_threshold=0.1
        )
        print(f"\nFound {len(results)} test results")
        for i, result in enumerate(results[:3]):
            print(f"Result {i+1}: {result}")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(check_collection())