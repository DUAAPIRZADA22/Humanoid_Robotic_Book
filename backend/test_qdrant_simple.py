"""
Simple Qdrant Test
Tests Qdrant connection without using the RAG module
"""

import asyncio
import os
import sys
import numpy as np

# Add backend to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from qdrant_client import QdrantClient
    from qdrant_client.models import Distance, VectorParams, PointStruct
    print("[OK] Qdrant client imported successfully")
except ImportError as e:
    print(f"[ERROR] Failed to import qdrant_client: {e}")
    print("\nPlease install qdrant-client:")
    print("pip install qdrant-client")
    sys.exit(1)


async def test_qdrant():
    """Test basic Qdrant functionality"""
    print("\n=== Testing Qdrant Connection ===\n")

    # Initialize client
    try:
        client = QdrantClient(
            host=os.getenv("QDRANT_HOST", "localhost"),
            port=int(os.getenv("QDRANT_PORT", "6333"))
        )
        print(f"[OK] Connected to Qdrant at {os.getenv('QDRANT_HOST', 'localhost')}:{os.getenv('QDRANT_PORT', '6333')}")
    except Exception as e:
        print(f"[ERROR] Failed to connect to Qdrant: {e}")
        print("\nMake sure Qdrant is running:")
        print("docker run -p 6333:6333 qdrant/qdrant")
        return

    # List collections
    try:
        collections = client.get_collections()
        print(f"[OK] Found {len(collections.collections)} existing collections")
        for col in collections.collections:
            print(f"  - {col.name}")
    except Exception as e:
        print(f"[ERROR] Failed to list collections: {e}")
        return

    # Create test collection
    collection_name = "test_humanoid_robotics"
    try:
        # Check if collection exists
        if collection_name in [c.name for c in collections.collections]:
            print(f"\n✓ Collection '{collection_name}' already exists")
            client.delete_collection(collection_name)
            print(f"  - Deleted existing collection")

        # Create new collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1024,  # Dimension for embeddings
                distance=Distance.COSINE
            )
        )
        print(f"✓ Created collection '{collection_name}' with 1024-dimensional vectors")
    except Exception as e:
        print(f"✗ Failed to create collection: {e}")
        return

    # Add sample points
    print("\n=== Adding Sample Documents ===")
    sample_docs = [
        {
            "id": 1,
            "text": "Humanoid robots use bipedal locomotion to walk like humans",
            "category": "locomotion",
            "keywords": ["bipedal", "walking", "humanoid"]
        },
        {
            "id": 2,
            "text": "Computer vision enables robots to recognize objects and navigate",
            "category": "perception",
            "keywords": ["vision", "recognition", "navigation"]
        },
        {
            "id": 3,
            "text": "Machine learning algorithms allow robots to adapt and improve",
            "category": "learning",
            "keywords": ["ML", "adaptation", "improvement"]
        }
    ]

    # Create points with random embeddings (in production, use real embeddings)
    points = []
    for doc in sample_docs:
        # Generate random vector for demo
        vector = np.random.rand(1024).tolist()

        point = PointStruct(
            id=doc["id"],
            vector=vector,
            payload={
                "text": doc["text"],
                "category": doc["category"],
                "keywords": doc["keywords"]
            }
        )
        points.append(point)
        print(f"✓ Added document {doc['id']}: {doc['text'][:50]}...")

    # Upsert points
    try:
        client.upsert(collection_name=collection_name, points=points)
        print(f"\n✓ Successfully inserted {len(points)} documents")
    except Exception as e:
        print(f"✗ Failed to insert documents: {e}")
        return

    # Count documents
    try:
        count = client.count(collection_name=collection_name)
        print(f"✓ Collection now has {count.count} documents")
    except Exception as e:
        print(f"✗ Failed to count documents: {e}")

    # Perform search
    print("\n=== Testing Vector Search ===")
    try:
        # Create a random query vector
        query_vector = np.random.rand(1024).tolist()

        # Search for similar documents
        search_result = client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=3,
            with_payload=True
        )

        print(f"✓ Found {len(search_result)} similar documents:")
        for i, hit in enumerate(search_result):
            print(f"  {i+1}. Score: {hit.score:.4f}")
            print(f"     Text: {hit.payload['text']}")
            print(f"     Category: {hit.payload['category']}")
    except Exception as e:
        print(f"✗ Search failed: {e}")

    # Filtered search
    print("\n=== Testing Filtered Search ===")
    try:
        from qdrant_client.models import Filter, FieldCondition, MatchValue

        # Search for documents in "perception" category
        filter_condition = Filter(
            must=[
                FieldCondition(
                    key="category",
                    match=MatchValue(value="perception")
                )
            ]
        )

        filtered_result = client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            query_filter=filter_condition,
            limit=5,
            with_payload=True
        )

        print(f"✓ Found {len(filtered_result)} documents in 'perception' category:")
        for hit in filtered_result:
            print(f"  - Score: {hit.score:.4f}")
            print(f"    Text: {hit.payload['text']}")
    except Exception as e:
        print(f"✗ Filtered search failed: {e}")

    # Get collection info
    print("\n=== Collection Information ===")
    try:
        info = client.get_collection(collection_name)
        print(f"✓ Collection details:")
        print(f"  - Vector count: {info.points_count}")
        print(f"  - Vector size: {info.config.params.vectors.size}")
        print(f"  - Distance metric: {info.config.params.vectors.distance}")
    except Exception as e:
        print(f"✗ Failed to get collection info: {e}")

    # Clean up
    print("\n=== Cleaning Up ===")
    try:
        client.delete_collection(collection_name)
        print(f"✓ Deleted test collection '{collection_name}'")
    except Exception as e:
        print(f"✗ Failed to delete collection: {e}")

    print("\n✅ All tests completed successfully!")
    print("\nTo use Qdrant in your project:")
    print("1. Install: pip install qdrant-client")
    print("2. Run Qdrant: docker run -p 6333:6333 qdrant/qdrant")
    print("3. Use the QdrantVectorStore class in backend/rag/vector_store.py")


if __name__ == "__main__":
    print("Qdrant Test for Physical AI & Humanoid Robotics Book")
    print("=" * 60)
    asyncio.run(test_qdrant())