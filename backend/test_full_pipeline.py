"""
Test the full RAG pipeline with ingestion and retrieval
"""

import os
import asyncio
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rag.embeddings import CohereEmbeddingService
from rag.vector_store import QdrantVectorStore
from rag.retrieval import RetrievalPipeline
from rag.rerank import CohereReranker
from dotenv import load_dotenv

load_dotenv()

async def test_full_pipeline():
    """Test the complete RAG pipeline"""
    print("Initializing RAG components...")

    # Load API keys
    cohere_api_key = os.getenv('COHERE_API_KEY')
    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')

    if not all([cohere_api_key, qdrant_url, qdrant_api_key]):
        print("Missing required environment variables!")
        return

    # Initialize services
    embedding_service = CohereEmbeddingService(api_key=cohere_api_key)
    vector_store = QdrantVectorStore(
        collection_name="humanoid_robotics_book",
        embedding_dim=1024,
        url=qdrant_url,
        api_key=qdrant_api_key
    )
    reranker = CohereReranker(api_key=cohere_api_key)

    # Create retrieval pipeline
    retrieval_pipeline = RetrievalPipeline(
        embedding_service=embedding_service,
        vector_store=vector_store,
        reranker=reranker
    )

    print("RAG components initialized!")

    # Test queries
    test_queries = [
        "What is Physical AI?",
        "How do humanoid robots work?",
        "What is ROS2?",
        "Explain robot kinematics"
    ]

    for query in test_queries:
        print(f"\nQuery: {query}")
        print("-" * 50)

        try:
            # Retrieve relevant documents
            results = await retrieval_pipeline.retrieve(
                query=query,
                top_k=3,
                similarity_threshold=0.5
            )

            print(f"Found {len(results)} results:")
            for i, result in enumerate(results):
                print(f"\n{i+1}. Score: {result['score']:.4f}")
                print(f"   Source: {result.get('metadata', {}).get('source', 'Unknown')}")
                print(f"   Preview: {result['text'][:150]}...")

        except Exception as e:
            print(f"Error: {e}")

    print(f"\nFull pipeline test completed!")
    print("If the backend is restarted, it should now return these results in chat responses.")

if __name__ == "__main__":
    asyncio.run(test_full_pipeline())