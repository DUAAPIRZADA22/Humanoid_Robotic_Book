"""
Simple Qdrant Connection Test
Tests basic Qdrant functionality
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


def test_qdrant():
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
        return False

    # Test basic operations
    try:
        # List collections
        collections = client.get_collections()
        print(f"[OK] Found {len(collections.collections)} existing collections")

        # Create test collection
        collection_name = "test_humanoid_robotics"

        # Delete if exists
        if collection_name in [c.name for c in collections.collections]:
            client.delete_collection(collection_name)
            print(f"[OK] Deleted existing collection '{collection_name}'")

        # Create new collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1024,
                distance=Distance.COSINE
            )
        )
        print(f"[OK] Created collection '{collection_name}'")

        # Add sample points
        points = []
        for i in range(3):
            vector = np.random.rand(1024).tolist()
            point = PointStruct(
                id=i,
                vector=vector,
                payload={
                    "text": f"Sample document {i} about robotics and AI",
                    "category": "demo"
                }
            )
            points.append(point)

        client.upsert(collection_name=collection_name, points=points)
        print(f"[OK] Inserted {len(points)} test documents")

        # Search
        query_vector = np.random.rand(1024).tolist()
        results = client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=3
        )
        print(f"[OK] Search returned {len(results)} results")

        # Cleanup
        client.delete_collection(collection_name)
        print(f"[OK] Cleaned up test collection")

        print("\n[SUCCESS] All Qdrant tests passed!")
        return True

    except Exception as e:
        print(f"[ERROR] Test failed: {e}")
        return False


if __name__ == "__main__":
    print("Qdrant Test for Physical AI & Humanoid Robotics Book")
    print("=" * 60)

    success = test_qdrant()

    if success:
        print("\n" + "=" * 60)
        print("Qdrant is working correctly!")
        print("\nNext steps:")
        print("1. Set up Qdrant permanently: docker run -d -p 6333:6333 qdrant/qdrant")
        print("2. Use the QdrantVectorStore class in your backend")
        print("3. Configure COHERE_API_KEY for embeddings")
    else:
        print("\nPlease fix the errors above and try again")