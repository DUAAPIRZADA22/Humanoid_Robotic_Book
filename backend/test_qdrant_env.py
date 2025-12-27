"""
Qdrant Connection Test with Environment Variables
Tests Qdrant connection using .env configuration
"""

import asyncio
import os
import sys
from dotenv import load_dotenv

# Load environment variables from .env
load_dotenv()

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
    """Test basic Qdrant functionality using environment variables"""
    print("\n=== Testing Qdrant Connection ===\n")

    # Get configuration from environment
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_host = os.getenv("QDRANT_HOST", "localhost")
    qdrant_port = os.getenv("QDRANT_PORT", "6333")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    print(f"Configuration:")
    if qdrant_url:
        print(f"  - URL: {qdrant_url}")
        print(f"  - API Key: {'*' * 10 if qdrant_api_key else 'Not set'}")
    else:
        print(f"  - Host: {qdrant_host}")
        print(f"  - Port: {qdrant_port}")

    # Initialize client with the correct configuration
    try:
        if qdrant_url:
            # Connect to Qdrant Cloud
            client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                timeout=30
            )
            print(f"[OK] Connected to Qdrant Cloud")
        else:
            # Connect to local Qdrant
            client = QdrantClient(
                host=qdrant_host,
                port=int(qdrant_port),
                api_key=qdrant_api_key if qdrant_api_key else None
            )
            print(f"[OK] Connected to local Qdrant at {qdrant_host}:{qdrant_port}")

        # Test connection by listing collections
        collections = client.get_collections()
        print(f"[OK] Found {len(collections.collections)} existing collections")

        # Show collection names
        if collections.collections:
            print("[OK] Collections:")
            for col in collections.collections:
                print(f"  - {col.name}")
        else:
            print("[OK] No existing collections (fresh database)")

    except Exception as e:
        print(f"[ERROR] Failed to connect to Qdrant: {e}")

        if "10061" in str(e) and not qdrant_url:
            print("\nThis is a local connection error. For Qdrant Cloud, ensure QDRANT_URL is set.")
            print("For local Qdrant, start it with: docker run -p 6333:6333 qdrant/qdrant")

        return False

    # Test creating a collection
    print("\n=== Testing Collection Operations ===")
    collection_name = "test_connection"

    try:
        # Delete if exists
        existing_collections = [c.name for c in client.get_collections().collections]
        if collection_name in existing_collections:
            client.delete_collection(collection_name)
            print(f"[OK] Deleted existing test collection")

        # Create new collection
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1024,
                distance=Distance.COSINE
            )
        )
        print(f"[OK] Created test collection '{collection_name}'")

        # Add a test point
        import numpy as np
        test_vector = np.random.rand(1024).tolist()

        client.upsert(
            collection_name=collection_name,
            points=[PointStruct(
                id=1,
                vector=test_vector,
                payload={
                    "text": "Test document for Qdrant connection",
                    "source": "connection_test"
                }
            )]
        )
        print("[OK] Added test document")

        # Search (using query_points method)
        search_result = client.query_points(
            collection_name=collection_name,
            query=test_vector,
            limit=5
        )
        print(f"[OK] Search returned {len(search_result.points)} results")

        # Clean up
        client.delete_collection(collection_name)
        print("[OK] Cleaned up test collection")

        print("\n[SUCCESS] All Qdrant tests passed!")
        return True

    except Exception as e:
        print(f"[ERROR] Collection operation failed: {e}")
        return False


if __name__ == "__main__":
    print("Qdrant Connection Test (with .env configuration)")
    print("=" * 60)

    # Show environment info
    print("\nEnvironment Variables:")
    print(f"  - QDRANT_URL: {os.getenv('QDRANT_URL', 'Not set')}")
    print(f"  - QDRANT_HOST: {os.getenv('QDRANT_HOST', 'localhost')}")
    print(f"  - QDRANT_PORT: {os.getenv('QDRANT_PORT', '6333')}")
    print(f"  - QDRANT_API_KEY: {'Set' if os.getenv('QDRANT_API_KEY') else 'Not set'}")
    print(f"  - COHERE_API_KEY: {'Set' if os.getenv('COHERE_API_KEY') else 'Not set'}")

    # Run the test
    success = asyncio.run(test_qdrant())

    if success:
        print("\n" + "=" * 60)
        print("[SUCCESS] Qdrant is ready!")
        print("\nNext steps:")
        print("1. Run the backend: python -m uvicorn main:app --reload")
        print("2. Open http://localhost:7860 in your browser")
    else:
        print("\n" + "=" * 60)
        print("[ERROR] Connection failed. Please check your configuration.")