"""
Qdrant Demo Script
Demonstrates basic Qdrant operations for vector storage and retrieval
"""

import asyncio
import os
import numpy as np
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from rag.vector_store import QdrantVectorStore
from rag.embeddings import CohereEmbeddingService


async def demo_qdrant_basic():
    """Demonstrate basic Qdrant operations"""
    print("=== Qdrant Basic Operations Demo ===\n")

    # Initialize Qdrant vector store
    vector_store = QdrantVectorStore(
        collection_name="demo_collection",
        embedding_dim=1024,  # Cohere embed-english-v3.0 dimension
        host=os.getenv("QDRANT_HOST", "localhost"),
        port=int(os.getenv("QDRANT_PORT", "6333"))
    )

    # Check health
    print("1. Checking Qdrant health...")
    health = await vector_store.health_check()
    print(f"   Status: {health.get('status', 'unknown')}")
    if health.get('status') == 'unhealthy':
        print(f"   Error: {health.get('error', 'Unknown error')}")
        print("\n   Make sure Qdrant is running:")
        print("   docker run -p 6333:6333 qdrant/qdrant")
        return

    # Ensure collection exists
    print("\n2. Creating/ensuring collection...")
    await vector_store.ensure_collection()
    print("   Collection ready!")

    # Create sample documents with dummy embeddings
    print("\n3. Adding sample documents...")
    sample_docs = [
        {
            "id": "doc1",
            "text": "Humanoid robots are bipedal machines designed to resemble human form and movement.",
            "metadata": {"category": "robotics", "topic": "humanoid"}
        },
        {
            "id": "doc2",
            "text": "Physical AI combines artificial intelligence with robotic hardware to interact with the real world.",
            "metadata": {"category": "AI", "topic": "physical_intelligence"}
        },
        {
            "id": "doc3",
            "text": "Computer vision enables robots to perceive and understand visual information from cameras.",
            "metadata": {"category": "AI", "topic": "vision"}
        }
    ]

    # Generate random embeddings for demo (in real use, use Cohere embeddings)
    for doc in sample_docs:
        doc["vector"] = np.random.rand(1024).tolist()

    # Upsert documents
    await vector_store.upsert(sample_docs)
    print(f"   Added {len(sample_docs)} documents")

    # Count documents
    count = await vector_store.count_documents()
    print(f"\n4. Total documents in collection: {count}")

    # Search for similar documents
    print("\n5. Searching for similar documents...")
    query_vector = np.random.rand(1024).tolist()
    results = await vector_store.search(
        query_vector=query_vector,
        top_k=3,
        score_threshold=0.0
    )

    print(f"   Found {len(results)} results:")
    for i, result in enumerate(results):
        print(f"   {i+1}. Score: {result['score']:.4f}")
        print(f"      Text: {result['text'][:100]}...")
        print(f"      Metadata: {result['metadata']}")

    # Retrieve specific document
    print("\n6. Retrieving specific document...")
    doc = await vector_store.get_document("doc1")
    if doc:
        print(f"   Found: {doc['text'][:100]}...")
    else:
        print("   Document not found")

    # Hybrid search (combines vector and text search)
    print("\n7. Performing hybrid search...")
    hybrid_results = await vector_store.hybrid_search(
        query_vector=query_vector,
        text_query="robotics AI vision",
        top_k=2
    )

    print(f"   Hybrid search results:")
    for i, result in enumerate(hybrid_results):
        print(f"   {i+1}. Hybrid Score: {result.get('hybrid_score', 0):.4f}")
        print(f"      Vector Score: {result['score']:.4f}")
        print(f"      Text Score: {result.get('text_score', 0):.4f}")
        print(f"      Text: {result['text'][:100]}...")

    # Get collection info
    print("\n8. Collection information:")
    info = await vector_store.get_collection_info()
    print(f"   Vector size: {info.config.params.vectors.size}")
    print(f"   Distance metric: {info.config.params.vectors.distance.value}")

    # Clean up
    print("\n9. Cleaning up demo collection...")
    await vector_store.clear_collection()
    print("   Demo completed successfully!")


async def demo_with_cohere():
    """Demonstrate Qdrant with real Cohere embeddings"""
    print("\n\n=== Qdrant with Cohere Embeddings Demo ===\n")

    # Check for Cohere API key
    api_key = os.getenv("COHERE_API_KEY")
    if not api_key or api_key == "your_cohere_api_key_here":
        print("Cohere API key not found. Skipping real embeddings demo.")
        print("Set COHERE_API_KEY in your .env file to run this demo.")
        return

    # Initialize Cohere embedding service
    print("1. Initializing Cohere embedding service...")
    embedding_service = CohereEmbeddingService(
        api_key=api_key,
        model="embed-english-v3.0"
    )

    # Test embedding service health
    health = await embedding_service.health_check()
    print(f"   Embedding service status: {health.get('status', 'unknown')}")

    if health.get('status') != 'healthy':
        print(f"   Error: {health.get('error', 'Unknown error')}")
        return

    # Initialize Qdrant with Cohere dimension
    vector_store = QdrantVectorStore(
        collection_name="cohere_demo",
        embedding_dim=health.get("test_embedding_shape", 1024),
        host=os.getenv("QDRANT_HOST", "localhost"),
        port=int(os.getenv("QDRANT_PORT", "6333"))
    )

    await vector_store.ensure_collection()

    # Create documents with real embeddings
    print("\n2. Creating documents with real embeddings...")
    documents = [
        "Bipedal locomotion is a key challenge in humanoid robotics development",
        "Machine learning algorithms enable adaptive behavior in autonomous systems",
        "Sensor fusion combines multiple sensor inputs for robust perception",
        "Deep reinforcement learning teaches robots complex motor skills",
        "Computer vision algorithms process visual data for object recognition"
    ]

    # Get embeddings
    print("3. Generating embeddings with Cohere...")
    embeddings = await embedding_service.get_embeddings(
        documents,
        input_type="search_document"
    )

    # Prepare documents for Qdrant
    qdrant_docs = []
    for i, (text, embedding) in enumerate(zip(documents, embeddings)):
        qdrant_docs.append({
            "id": f"cohere_doc_{i}",
            "text": text,
            "vector": embedding,
            "metadata": {"source": "demo", "index": i}
        })

    # Store in Qdrant
    print("4. Storing documents in Qdrant...")
    await vector_store.upsert(qdrant_docs)

    # Semantic search
    print("\n5. Performing semantic search...")
    query = "How do robots learn to walk?"
    query_embedding = await embedding_service.get_embedding(
        query,
        input_type="search_query"
    )

    results = await vector_store.search(
        query_vector=query_embedding,
        top_k=3,
        score_threshold=0.3
    )

    print(f"   Query: '{query}'")
    print(f"   Results:")
    for i, result in enumerate(results):
        print(f"   {i+1}. Score: {result['score']:.4f}")
        print(f"      {result['text']}")

    # Clean up
    await vector_store.clear_collection()
    print("\n6. Demo completed!")


if __name__ == "__main__":
    print("Qdrant Demo for Physical AI & Humanoid Robotics Book")
    print("=" * 60)

    # Run basic demo
    asyncio.run(demo_qdrant_basic())

    # Run Cohere demo if API key is available
    asyncio.run(demo_with_cohere())

    print("\n" + "=" * 60)
    print("Demo finished!")
    print("\nTo start using Qdrant in your project:")
    print("1. Make sure Qdrant is running: docker run -p 6333:6333 qdrant/qdrant")
    print("2. Set your environment variables in .env")
    print("3. Import and use the QdrantVectorStore class in your code")